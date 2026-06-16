在此目录下放置你要通过语音调用的自定义函数模块。

使用方法：
1) 新建一个模块文件，如 `src/user_functions/my_funcs.py`
2) 在其中定义函数，例如：
```python
def say_hello(name: str) -> str:
    return f"Hello, {name}!"
```
3) 通过 MCP 工具 `self.runner.run_function` 调用：
   - module: `src.user_functions.my_funcs`
   - function: `say_hello`
   - args: `["小智"]`
   - kwargs: `{}`

语音示例：
“运行我的函数 say_hello，参数是‘小智’，模块是 user_functions 下的 my_funcs”


