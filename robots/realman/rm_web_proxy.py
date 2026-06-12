#!/usr/bin/env python3
"""HTTP reverse proxy for the RM web pendant with JS rewrites for local forwarding."""

from __future__ import annotations

import argparse
import http.client
import socketserver
from http.server import BaseHTTPRequestHandler


LOGIN_OLD = (
    'let e=window.location.href.slice(0,window.location.href.indexOf("/#"))+":8090";'
    'localStorage.setItem("baseUrl",e.substring(0,e.length-5))'
)
LOGIN_NEW = (
    'let e=window.location.protocol+"//"+window.location.hostname+":8090";'
    'localStorage.setItem("baseUrl",window.location.origin)'
)
LOCK_OLD = 'window.location.href.slice(0,window.location.href.indexOf("/#"))+":8090"'
LOCK_NEW = 'window.location.protocol+"//"+window.location.hostname+":8090"'
MAIN_WS_OLD = '"ws"+localStorage.getItem("baseUrl").slice(4)+":8060"'
MAIN_WS_NEW = 'window.location.protocol.replace("http","ws")+"//"+window.location.hostname+":8060"'
HTML_INJECT = (
    '<style id="rm-localfix-style">'
    '.homepageLoadingDiv{display:none!important;}'
    '</style>'
    '<script>'
    '(function(){'
    'function rmFix(){'
    'if(document.body){document.body.classList.remove("el-popup-parent--hidden");}'
    'document.querySelectorAll(".homepageLoadingDiv").forEach(function(el){'
    'el.style.display="none";'
    'if(el.parentNode){el.parentNode.removeChild(el);}'
    '});'
    '}'
    'window.addEventListener("hashchange",rmFix);'
    'document.addEventListener("DOMContentLoaded",rmFix);'
    'setInterval(rmFix,500);'
    '})();'
    '</script>'
)


class ThreadingHTTPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


class ProxyHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    remote_host = ""
    remote_port = 80

    def do_GET(self) -> None:  # noqa: N802
        self._proxy("GET")

    def do_HEAD(self) -> None:  # noqa: N802
        self._proxy("HEAD")

    def _proxy(self, method: str) -> None:
        conn = http.client.HTTPConnection(self.remote_host, self.remote_port, timeout=15)
        try:
            headers = {k: v for k, v in self.headers.items()}
            headers["Host"] = self.remote_host
            headers.pop("Accept-Encoding", None)
            conn.request(method, self.path, headers=headers)
            resp = conn.getresponse()
            body = resp.read()
            status = resp.status
            reason = resp.reason
            response_headers = resp.getheaders()

            content_type = dict((k.lower(), v) for k, v in response_headers).get("content-type", "")
            if method == "GET":
                body = self._maybe_rewrite(self.path, content_type, body)

            self.send_response(status, reason)
            skip = {
                "connection",
                "keep-alive",
                "proxy-authenticate",
                "proxy-authorization",
                "cache-control",
                "etag",
                "expires",
                "last-modified",
                "pragma",
                "te",
                "trailers",
                "transfer-encoding",
                "upgrade",
                "content-length",
            }
            for key, value in response_headers:
                if key.lower() in skip:
                    continue
                self.send_header(key, value)
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Connection", "close")
            self.end_headers()
            if method != "HEAD":
                self.wfile.write(body)
        finally:
            conn.close()

    def _maybe_rewrite(self, path: str, content_type: str, body: bytes) -> bytes:
        text = body.decode("utf-8", errors="ignore")
        if "text/html" in content_type and "</head>" in text and HTML_INJECT not in text:
            text = text.replace("</head>", f"{HTML_INJECT}</head>", 1)
        if "rm_web-login" in path and LOGIN_OLD in text:
            text = text.replace(LOGIN_OLD, LOGIN_NEW, 1)
        if "rm_web-lock" in path and LOCK_OLD in text:
            text = text.replace(LOCK_OLD, LOCK_NEW, 1)
        if MAIN_WS_OLD in text:
            text = text.replace(MAIN_WS_OLD, MAIN_WS_NEW, 1)
        return text.encode("utf-8")

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def main() -> int:
    parser = argparse.ArgumentParser(description="RM web local proxy")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--bind-port", type=int, default=18080)
    parser.add_argument("--remote-host", required=True)
    parser.add_argument("--remote-port", type=int, default=80)
    args = parser.parse_args()

    ProxyHandler.remote_host = args.remote_host
    ProxyHandler.remote_port = args.remote_port

    with ThreadingHTTPServer((args.bind_host, args.bind_port), ProxyHandler) as server:
        server.serve_forever()


if __name__ == "__main__":
    raise SystemExit(main())
