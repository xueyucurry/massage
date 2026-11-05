import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    width: 1460
    height: 620
    color: "#eef4f8"

    signal manualButtonPressed()
    signal manualButtonReleased()
    signal autoButtonClicked()
    signal abortButtonClicked()
    signal modeButtonClicked()
    signal sendButtonClicked(string text)
    signal settingsButtonClicked()
    signal healthReceiverStartClicked()
    signal healthReceiverStopClicked()
    signal titleMinimize()
    signal titleClose()
    signal titleDragStart(real mouseX, real mouseY)
    signal titleDragMoveTo(real mouseX, real mouseY)
    signal titleDragEnd()

    function accentColor() {
        return displayModel ? displayModel.robotStatusColor : "#8A94A6"
    }

    function quickCommand(text) {
        root.sendButtonClicked(text)
    }

    function healthTrendText(label, values, suffix) {
        if (!values || values.length < 2) return label + "：等待更多数据"
        return label + "：" + values[0] + " → " + values[values.length - 1] + suffix
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        Rectangle {
            id: titleBar
            Layout.fillWidth: true
            Layout.preferredHeight: 34
            color: "#f7f9fb"

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton
                onPressed: root.titleDragStart(mouse.x, mouse.y)
                onPositionChanged: {
                    if (pressed) {
                        root.titleDragMoveTo(mouse.x, mouse.y)
                    }
                }
                onReleased: root.titleDragEnd()
                z: 0
            }

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 14
                anchors.rightMargin: 8
                spacing: 8
                z: 1

                Text {
                    text: "FAIRINO 按摩机器人"
                    font.family: "PingFang SC, Microsoft YaHei UI"
                    font.pixelSize: 13
                    font.weight: Font.Bold
                    color: "#1f2a37"
                }

                Item { Layout.fillWidth: true }

                Rectangle {
                    width: 24
                    height: 24
                    radius: 6
                    color: btnMinMouse.pressed ? "#dfe5ec" : (btnMinMouse.containsMouse ? "#edf2f7" : "transparent")
                    Text { anchors.centerIn: parent; text: "-"; font.pixelSize: 14; color: "#64748b" }
                    MouseArea {
                        id: btnMinMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: root.titleMinimize()
                    }
                }

                Rectangle {
                    width: 24
                    height: 24
                    radius: 6
                    color: btnCloseMouse.pressed ? "#d64545" : (btnCloseMouse.containsMouse ? "#ef6f6c" : "transparent")
                    Text { anchors.centerIn: parent; text: "x"; font.pixelSize: 13; color: btnCloseMouse.containsMouse ? "white" : "#8a94a6" }
                    MouseArea {
                        id: btnCloseMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        onClicked: root.titleClose()
                    }
                }
            }

            Rectangle {
                visible: false
                Layout.preferredWidth: 260
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: displayModel ? displayModel.healthStatusColor : "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 9

                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "健康监测"; font.pixelSize: 14; font.weight: Font.Bold; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Item { Layout.fillWidth: true }
                        Rectangle {
                            Layout.preferredWidth: 68
                            Layout.preferredHeight: 22
                            radius: 11
                            color: displayModel ? displayModel.healthStatusColor : "#8A94A6"
                            Text { anchors.centerIn: parent; text: displayModel ? displayModel.healthStatusLabel : "等待接收"; font.pixelSize: 10; color: "white"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        }
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        rowSpacing: 8
                        columnSpacing: 12
                        Text { text: "心率"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.healthHeartRateText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: "血氧"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.healthSpo2Text : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: "体温"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.healthTemperatureText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: "电量"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.healthBatteryText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI" }
                    }

                    Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }

                    Text { text: "最近 30 分钟趋势"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                    Repeater {
                        model: [
                            { name: "心率", values: displayModel ? displayModel.healthHeartRateTrend : [], color: "#e85d75" },
                            { name: "血氧", values: displayModel ? displayModel.healthSpo2Trend : [], color: "#3d8bfd" },
                            { name: "体温", values: displayModel ? displayModel.healthTemperatureTrend : [], color: "#f0a542" }
                        ]
                        delegate: Canvas {
                            id: fullSparkline
                            Layout.fillWidth: true
                            Layout.preferredHeight: 42
                            property var trendValues: modelData.values
                            property color trendColor: modelData.color
                            onTrendValuesChanged: requestPaint()
                            Connections { target: displayModel; function onHealthStateChanged() { fullSparkline.requestPaint() } }
                            onPaint: {
                                var ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                var values = trendValues || []
                                ctx.fillStyle = "#64748b"; ctx.font = "10px sans-serif"; ctx.fillText(modelData.name, 1, 11)
                                if (values.length < 2) { ctx.fillText("等待更多数据", 46, 24); return }
                                var low = values[0], high = values[0]
                                for (var i = 1; i < values.length; i++) { low = Math.min(low, values[i]); high = Math.max(high, values[i]) }
                                if (high === low) high = low + 1
                                ctx.strokeStyle = trendColor; ctx.lineWidth = 1.8; ctx.beginPath()
                                for (var j = 0; j < values.length; j++) {
                                    var x = 2 + j * (width - 4) / (values.length - 1)
                                    var y = 39 - (values[j] - low) * 22 / (high - low)
                                    if (j === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                                }
                                ctx.stroke()
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }
                    Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }
                    Text { text: displayModel ? displayModel.healthReceiverStatusText : "接收器状态未知"; font.pixelSize: 11; color: displayModel && displayModel.healthReceiverRunning ? "#0B8F77" : "#D9822B"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                    RowLayout {
                        Layout.fillWidth: true
                        Button {
                            id: healthStartWide
                            Layout.fillWidth: true; Layout.preferredHeight: 28; text: "启动接收"
                            enabled: displayModel ? !displayModel.healthReceiverRunning : true
                            background: Rectangle { color: healthStartWide.enabled ? "#e8f8f4" : "#eef2f5"; radius: 6; border.color: "#bfe5db" }
                            contentItem: Text { text: healthStartWide.text; color: "#0b6b5e"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStartClicked()
                        }
                        Button {
                            id: healthStopWide
                            Layout.fillWidth: true; Layout.preferredHeight: 28; text: "停止接收"
                            enabled: displayModel ? displayModel.healthReceiverRunning : false
                            background: Rectangle { color: healthStopWide.enabled ? "#fff0f0" : "#eef2f5"; radius: 6; border.color: "#f4b8b8" }
                            contentItem: Text { text: healthStopWide.text; color: "#b42323"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStopClicked()
                        }
                    }
                    Text { text: displayModel ? displayModel.healthMessage : "尚未收到手环数据"; font.pixelSize: 10; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }

            Rectangle {
                visible: false
                Layout.preferredWidth: 260
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: displayModel ? displayModel.healthStatusColor : "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 9
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "健康监测"; font.pixelSize: 14; font.weight: Font.Bold; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Item { Layout.fillWidth: true }
                        Rectangle { Layout.preferredWidth: 68; Layout.preferredHeight: 22; radius: 11; color: displayModel ? displayModel.healthStatusColor : "#8A94A6"; Text { anchors.centerIn: parent; text: displayModel ? displayModel.healthStatusLabel : "等待接收"; font.pixelSize: 10; color: "white" } }
                    }
                    GridLayout {
                        Layout.fillWidth: true; columns: 2; rowSpacing: 8; columnSpacing: 12
                        Text { text: "心率"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthHeartRateText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "血氧"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthSpo2Text : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "体温"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthTemperatureText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "电量"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthBatteryText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                    }
                    Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }
                    Text { text: "最近 10 分钟趋势"; font.pixelSize: 11; color: "#64748b" }
                    Text { text: root.healthTrendText("心率", displayModel ? displayModel.healthHeartRateTrend : [], " bpm"); font.pixelSize: 11; color: "#e85d75"; Layout.fillWidth: true }
                    Text { text: root.healthTrendText("血氧", displayModel ? displayModel.healthSpo2Trend : [], " %"); font.pixelSize: 11; color: "#3d8bfd"; Layout.fillWidth: true }
                    Text { text: root.healthTrendText("体温", displayModel ? displayModel.healthTemperatureTrend : [], " ℃"); font.pixelSize: 11; color: "#f0a542"; Layout.fillWidth: true }
                    Repeater {
                        model: [
                            { name: "心率", values: displayModel ? displayModel.healthHeartRateTrend : [], color: "#e85d75" },
                            { name: "血氧", values: displayModel ? displayModel.healthSpo2Trend : [], color: "#3d8bfd" },
                            { name: "体温", values: displayModel ? displayModel.healthTemperatureTrend : [], color: "#f0a542" }
                        ]
                        delegate: Canvas {
                            id: rightSparkline
                            Layout.fillWidth: true; Layout.preferredHeight: 42
                            property var trendValues: modelData.values
                            property color trendColor: modelData.color
                            onTrendValuesChanged: requestPaint()
                            Connections { target: displayModel; function onHealthStateChanged() { rightSparkline.requestPaint() } }
                            onPaint: {
                                var ctx = getContext("2d"); ctx.clearRect(0, 0, width, height)
                                var values = trendValues || []; ctx.fillStyle = "#64748b"; ctx.font = "10px sans-serif"; ctx.fillText(modelData.name, 1, 11)
                                if (values.length < 2) { ctx.fillText("等待更多数据", 46, 24); return }
                                var low = values[0], high = values[0]
                                for (var i = 1; i < values.length; i++) { low = Math.min(low, values[i]); high = Math.max(high, values[i]) }
                                if (high === low) high = low + 1
                                ctx.strokeStyle = trendColor; ctx.lineWidth = 1.8; ctx.beginPath()
                                for (var j = 0; j < values.length; j++) { var x = 2 + j * (width - 4) / (values.length - 1); var y = 39 - (values[j] - low) * 22 / (high - low); if (j === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y) }
                                ctx.stroke()
                            }
                        }
                    }
                    Item { Layout.fillHeight: true }
                    Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }
                    Text { text: displayModel ? displayModel.healthReceiverStatusText : "接收器状态未知"; font.pixelSize: 11; color: displayModel && displayModel.healthReceiverRunning ? "#0B8F77" : "#D9822B"; Layout.fillWidth: true; elide: Text.ElideRight }
                    RowLayout {
                        Layout.fillWidth: true
                        Button { id: rightHealthStart; Layout.fillWidth: true; Layout.preferredHeight: 28; text: "启动接收"; enabled: displayModel ? !displayModel.healthReceiverRunning : true; background: Rectangle { color: rightHealthStart.enabled ? "#e8f8f4" : "#eef2f5"; radius: 6; border.color: "#bfe5db" }
                            contentItem: Text { text: rightHealthStart.text; color: "#0b6b5e"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStartClicked() }
                        Button { id: rightHealthStop; Layout.fillWidth: true; Layout.preferredHeight: 28; text: "停止接收"; enabled: displayModel ? displayModel.healthReceiverRunning : false; background: Rectangle { color: rightHealthStop.enabled ? "#fff0f0" : "#eef2f5"; radius: 6; border.color: "#f4b8b8" }
                            contentItem: Text { text: rightHealthStop.text; color: "#b42323"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStopClicked() }
                    }
                    Text { text: displayModel ? displayModel.healthMessage : "尚未收到手环数据"; font.pixelSize: 10; color: "#64748b"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 58
            Layout.leftMargin: 14
            Layout.rightMargin: 14
            Layout.topMargin: 10
            radius: 8
            color: "#ffffff"
            border.color: "#dbe5ef"
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin: 16
                anchors.rightMargin: 16
                spacing: 12

                Rectangle {
                    width: 9
                    height: 34
                    radius: 5
                    color: root.accentColor()
                }

                ColumnLayout {
                    spacing: 2
                    Layout.fillWidth: true

                    Text {
                        text: displayModel ? displayModel.massageTaskTitle : "按摩机器人就绪"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 18
                        font.weight: Font.Bold
                        color: "#172033"
                        elide: Text.ElideRight
                        Layout.fillWidth: true
                    }

                    Text {
                        text: displayModel ? displayModel.massageMessage : "就绪，等待语音指令"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 12
                        color: "#5b677a"
                        elide: Text.ElideRight
                        Layout.fillWidth: true
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 110
                    Layout.preferredHeight: 30
                    radius: 15
                    color: root.accentColor()
                    Text {
                        anchors.centerIn: parent
                        text: displayModel ? displayModel.massageStatusBadgeText : "就绪"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 13
                        font.weight: Font.Bold
                        color: "white"
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 128
                    Layout.preferredHeight: 30
                    radius: 15
                    color: "#eef6ff"
                    border.color: "#cfe3fa"
                    Text {
                        anchors.centerIn: parent
                        text: displayModel ? displayModel.statusText : "状态: 未连接"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 12
                        color: "#1d6fd1"
                        elide: Text.ElideRight
                    }
                }
            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.leftMargin: 16
            Layout.rightMargin: 16
            Layout.topMargin: 8
            Layout.bottomMargin: 8
            spacing: 10

            Rectangle {
                Layout.preferredWidth: 250
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    Text {
                        text: "经络示意"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 15
                        font.weight: Font.Bold
                        color: "#1f2a37"
                    }

                    Canvas {
                        id: meridianCanvas
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 130
                        Component.onCompleted: requestPaint()
                        onWidthChanged: requestPaint()
                        onHeightChanged: requestPaint()
                        Connections {
                            target: displayModel
                            function onMassageStateChanged() { meridianCanvas.requestPaint() }
                        }

                        onPaint: {
                            var ctx = getContext("2d")
                            ctx.clearRect(0, 0, width, height)

                            var cx = width / 2
                            var top = 20
                            var bottom = height - 16
                            var accent = root.accentColor()
                            var target = displayModel ? displayModel.massageTargetLabel : ""
                            var activeBack = target.indexOf("背") >= 0 || target.indexOf("膀胱") >= 0
                            var activeLeg = target.indexOf("腿") >= 0

                            ctx.lineCap = "round"
                            ctx.lineJoin = "round"

                            ctx.strokeStyle = "#9fb1c7"
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.arc(cx, top + 24, 18, 0, Math.PI * 2)
                            ctx.stroke()

                            ctx.beginPath()
                            ctx.moveTo(cx - 44, top + 56)
                            ctx.quadraticCurveTo(cx - 30, top + 46, cx, top + 50)
                            ctx.quadraticCurveTo(cx + 30, top + 46, cx + 44, top + 56)
                            ctx.lineTo(cx + 30, bottom - 54)
                            ctx.quadraticCurveTo(cx + 18, bottom - 20, cx + 10, bottom)
                            ctx.moveTo(cx - 44, top + 56)
                            ctx.lineTo(cx - 30, bottom - 54)
                            ctx.quadraticCurveTo(cx - 18, bottom - 20, cx - 10, bottom)
                            ctx.stroke()

                            ctx.strokeStyle = "#c8d4e3"
                            ctx.lineWidth = 1.5
                            ctx.beginPath()
                            ctx.moveTo(cx, top + 58)
                            ctx.lineTo(cx, bottom - 52)
                            ctx.stroke()

                            ctx.strokeStyle = activeBack ? accent : "#d7e0ea"
                            ctx.lineWidth = activeBack ? 4 : 2
                            ctx.beginPath()
                            ctx.moveTo(cx - 20, top + 66)
                            ctx.bezierCurveTo(cx - 34, top + 104, cx - 32, bottom - 98, cx - 22, bottom - 56)
                            ctx.moveTo(cx + 20, top + 66)
                            ctx.bezierCurveTo(cx + 34, top + 104, cx + 32, bottom - 98, cx + 22, bottom - 56)
                            ctx.stroke()

                            ctx.strokeStyle = activeLeg ? accent : "#d7e0ea"
                            ctx.lineWidth = activeLeg ? 4 : 2
                            ctx.beginPath()
                            ctx.moveTo(cx - 16, bottom - 58)
                            ctx.lineTo(cx - 22, bottom - 8)
                            ctx.moveTo(cx + 16, bottom - 58)
                            ctx.lineTo(cx + 22, bottom - 8)
                            ctx.stroke()

                            ctx.fillStyle = activeBack || activeLeg ? accent : "#b9c7d7"
                            for (var i = 0; i < 10; i++) {
                                var y = top + 74 + i * ((bottom - 140) / 9)
                                ctx.beginPath()
                                ctx.arc(cx - 20, y, 3, 0, Math.PI * 2)
                                ctx.arc(cx + 20, y, 3, 0, Math.PI * 2)
                                ctx.fill()
                            }
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 54
                        radius: 8
                        color: "#f6f9fc"
                        border.color: "#e1eaf3"

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 2

                            Text {
                                text: displayModel ? displayModel.massageTargetLabel : "未选择"
                                font.family: "PingFang SC, Microsoft YaHei UI"
                                font.pixelSize: 13
                                font.weight: Font.Bold
                                color: "#1f2a37"
                                elide: Text.ElideRight
                                Layout.fillWidth: true
                            }
                            Text {
                                text: displayModel ? displayModel.massageTrajectoryText : "未保存"
                                font.family: "PingFang SC, Microsoft YaHei UI"
                                font.pixelSize: 11
                                color: "#64748b"
                                elide: Text.ElideMiddle
                                Layout.fillWidth: true
                            }
                        }
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 8

                    Item {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 84

                        Rectangle {
                            width: 82
                            height: 82
                            radius: 41
                            anchors.centerIn: parent
                            color: "#f2f8ff"
                            border.color: root.accentColor()
                            border.width: 2

                            Loader {
                                id: emotionLoader
                                anchors.centerIn: parent
                                width: 66
                                height: 66

                                sourceComponent: {
                                    var path = displayModel ? displayModel.emotionPath : ""
                                    if (!path || path.length === 0) {
                                        return emojiComponent
                                    }
                                    if (path.indexOf(".gif") !== -1) {
                                        return gifComponent
                                    }
                                    if (path.indexOf(".") !== -1) {
                                        return imageComponent
                                    }
                                    return emojiComponent
                                }

                                Component {
                                    id: gifComponent
                                    AnimatedImage {
                                        fillMode: Image.PreserveAspectFit
                                        source: displayModel ? displayModel.emotionPath : ""
                                        playing: true
                                        speed: 1.05
                                        cache: true
                                    }
                                }

                                Component {
                                    id: imageComponent
                                    Image {
                                        fillMode: Image.PreserveAspectFit
                                        source: displayModel ? displayModel.emotionPath : ""
                                        cache: true
                                    }
                                }

                                Component {
                                    id: emojiComponent
                                    Text {
                                        text: displayModel ? displayModel.emotionPath : "😊"
                                        font.pixelSize: 46
                                        horizontalAlignment: Text.AlignHCenter
                                        verticalAlignment: Text.AlignVCenter
                                    }
                                }
                            }
                        }
                    }

                    Text {
                        text: displayModel ? displayModel.massageMessage : "就绪，等待语音指令"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 14
                        font.weight: Font.Bold
                        color: "#1f2a37"
                        horizontalAlignment: Text.AlignHCenter
                        wrapMode: Text.WordWrap
                        maximumLineCount: 2
                        elide: Text.ElideRight
                        Layout.preferredHeight: 36
                        Layout.fillWidth: true
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 10
                        radius: 5
                        color: "#e7eef6"
                        clip: true

                        Rectangle {
                            height: parent.height
                            width: parent.width * (displayModel ? displayModel.massageProgressValue : 0)
                            radius: 6
                            color: root.accentColor()
                            Behavior on width { NumberAnimation { duration: 180; easing.type: Easing.OutCubic } }
                        }
                    }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 56
                            radius: 8
                            color: "#f6f9fc"
                            border.color: "#e1eaf3"
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 8
                                spacing: 2
                                Text { text: "当前手法"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                                Text { text: displayModel ? displayModel.massageActionLabel : "就绪"; font.pixelSize: 15; font.weight: Font.Bold; color: "#172033"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 56
                            radius: 8
                            color: "#f6f9fc"
                            border.color: "#e1eaf3"
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 8
                                spacing: 2
                                Text { text: "当前点位"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                                Text { text: displayModel ? displayModel.massagePointText : "0 / 0"; font.pixelSize: 15; font.weight: Font.Bold; color: "#172033"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 56
                            radius: 8
                            color: "#f6f9fc"
                            border.color: "#e1eaf3"
                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 8
                                spacing: 2
                                Text { text: "目标力度"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                                Text { text: displayModel ? displayModel.massageForceText : "-"; font.pixelSize: 15; font.weight: Font.Bold; color: "#172033"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }
                            }
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 38
                        radius: 8
                        color: "#f9fbfd"
                        border.color: "#e1eaf3"
                        Text {
                            anchors.fill: parent
                            anchors.margins: 8
                            text: displayModel ? displayModel.ttsText : "就绪"
                            font.family: "PingFang SC, Microsoft YaHei UI"
                            font.pixelSize: 12
                            color: "#4b5563"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            wrapMode: Text.WordWrap
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Rectangle {
                Layout.preferredWidth: 230
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 8

                    Text {
                        text: "任务详情"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 14
                        font.weight: Font.Bold
                        color: "#1f2a37"
                    }

                    Rectangle {
                        visible: false
                        Layout.preferredHeight: 0
                        Layout.fillWidth: true
                        radius: 8
                        color: "#f9fbfd"
                        border.color: displayModel ? displayModel.healthStatusColor : "#dbe5ef"
                        border.width: 1

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 3

                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "健康监测"; font.pixelSize: 12; font.weight: Font.Bold; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI" }
                                Item { Layout.fillWidth: true }
                                Rectangle {
                                    Layout.preferredHeight: 20
                                    Layout.preferredWidth: 64
                                    radius: 10
                                    color: displayModel ? displayModel.healthStatusColor : "#8A94A6"
                                    Text { anchors.centerIn: parent; text: displayModel ? displayModel.healthStatusLabel : "等待接收"; font.pixelSize: 10; color: "white"; font.family: "PingFang SC, Microsoft YaHei UI" }
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 6
                                Text { text: "心率 " + (displayModel ? displayModel.healthHeartRateText : "-"); font.pixelSize: 11; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                                Text { text: "血氧 " + (displayModel ? displayModel.healthSpo2Text : "-"); font.pixelSize: 11; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 6
                                Text { text: "体温 " + (displayModel ? displayModel.healthTemperatureText : "-"); font.pixelSize: 11; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                                Text { text: "电量 " + (displayModel ? displayModel.healthBatteryText : "-"); font.pixelSize: 11; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 26
                                spacing: 4
                                Repeater {
                                    model: [
                                        { name: "心率", values: displayModel ? displayModel.healthHeartRateTrend : [], color: "#e85d75" },
                                        { name: "血氧", values: displayModel ? displayModel.healthSpo2Trend : [], color: "#3d8bfd" },
                                        { name: "体温", values: displayModel ? displayModel.healthTemperatureTrend : [], color: "#f0a542" }
                                    ]
                                    delegate: Canvas {
                                        id: sparkline
                                        Layout.fillWidth: true
                                        Layout.preferredHeight: 26
                                        property var trendValues: modelData.values
                                        property color trendColor: modelData.color
                                        onTrendValuesChanged: requestPaint()
                                        Connections { target: displayModel; function onHealthStateChanged() { sparkline.requestPaint() } }
                                        onPaint: {
                                            var ctx = getContext("2d")
                                            ctx.clearRect(0, 0, width, height)
                                            var values = trendValues || []
                                            ctx.fillStyle = "#64748b"
                                            ctx.font = "9px sans-serif"
                                            ctx.fillText(modelData.name, 1, 10)
                                            if (values.length < 2) {
                                                ctx.fillText("-", width - 8, 18)
                                                return
                                            }
                                            var low = values[0], high = values[0]
                                            for (var i = 1; i < values.length; i++) { low = Math.min(low, values[i]); high = Math.max(high, values[i]) }
                                            if (high === low) { high = low + 1 }
                                            ctx.strokeStyle = trendColor
                                            ctx.lineWidth = 1.5
                                            ctx.beginPath()
                                            for (var j = 0; j < values.length; j++) {
                                                var x = 1 + j * (width - 2) / (values.length - 1)
                                                var y = 24 - (values[j] - low) * 12 / (high - low)
                                                if (j === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                                            }
                                            ctx.stroke()
                                        }
                                    }
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 5
                                Text { text: displayModel ? displayModel.healthReceiverStatusText : "接收器状态未知"; font.pixelSize: 10; color: displayModel && displayModel.healthReceiverRunning ? "#0B8F77" : "#D9822B"; font.family: "PingFang SC, Microsoft YaHei UI"; Layout.fillWidth: true; elide: Text.ElideRight }
                                Button {
                                    id: healthStart
                                    Layout.preferredWidth: 38
                                    Layout.preferredHeight: 22
                                    text: "启动"
                                    enabled: displayModel ? !displayModel.healthReceiverRunning : true
                                    background: Rectangle { color: healthStart.enabled ? "#e8f8f4" : "#eef2f5"; radius: 5; border.color: "#bfe5db" }
                                    contentItem: Text { text: healthStart.text; color: "#0b6b5e"; font.pixelSize: 10; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                    onClicked: root.healthReceiverStartClicked()
                                }
                                Button {
                                    id: healthStop
                                    Layout.preferredWidth: 38
                                    Layout.preferredHeight: 22
                                    text: "停止"
                                    enabled: displayModel ? displayModel.healthReceiverRunning : false
                                    background: Rectangle { color: healthStop.enabled ? "#fff0f0" : "#eef2f5"; radius: 5; border.color: "#f4b8b8" }
                                    contentItem: Text { text: healthStop.text; color: "#b42323"; font.pixelSize: 10; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                    onClicked: root.healthReceiverStopClicked()
                                }
                            }

                            Text { text: displayModel ? displayModel.healthMessage : "尚未收到手环数据"; font.pixelSize: 10; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }
                        }
                    }

                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        rowSpacing: 6
                        columnSpacing: 10

                        Text { text: "连接"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.robotStatusText : "未连接"; font.pixelSize: 11; color: root.accentColor(); font.weight: Font.Bold; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "部位"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageTargetLabel : "未选择"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "轨迹"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageTrajectoryText : "未保存"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideMiddle; Layout.fillWidth: true }

                        Text { text: "阶段"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageStageLabel : "-"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "动作"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageActionLabel : "就绪"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "点位"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massagePointText : "0 / 0"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "力度"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageForceText : "-"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "进程"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageWorkerText : "未运行"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }

                        Text { text: "更新"; font.pixelSize: 11; color: "#64748b"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Text { text: displayModel ? displayModel.massageUpdatedAt : "-"; font.pixelSize: 11; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI"; elide: Text.ElideRight; Layout.fillWidth: true }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 48
                        radius: 8
                        color: "#f6f9fc"
                        border.color: "#e1eaf3"
                        Text {
                            anchors.fill: parent
                            anchors.margins: 8
                            text: displayModel ? displayModel.massageMessage : "就绪，等待语音指令"
                            font.family: "PingFang SC, Microsoft YaHei UI"
                            font.pixelSize: 11
                            color: "#4b5563"
                            wrapMode: Text.WordWrap
                            maximumLineCount: 2
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Rectangle {
                Layout.preferredWidth: 260
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: displayModel ? displayModel.healthStatusColor : "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 9
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "健康监测"; font.pixelSize: 14; font.weight: Font.Bold; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Item { Layout.fillWidth: true }
                        Rectangle { Layout.preferredWidth: 68; Layout.preferredHeight: 22; radius: 11; color: displayModel ? displayModel.healthStatusColor : "#8A94A6"; Text { anchors.centerIn: parent; text: displayModel ? displayModel.healthStatusLabel : "等待接收"; font.pixelSize: 10; color: "white" } }
                    }
                    GridLayout {
                        Layout.fillWidth: true; columns: 2; rowSpacing: 8; columnSpacing: 12
                        Text { text: "心率"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthHeartRateText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "血氧"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthSpo2Text : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "体温"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthTemperatureText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                        Text { text: "电量"; font.pixelSize: 11; color: "#64748b" }
                            Text { text: displayModel ? displayModel.healthBatteryText : "-"; font.pixelSize: 13; font.weight: Font.Bold; color: "#334155" }
                    }
                    Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }
                    Text { text: displayModel ? displayModel.healthReceiverStatusText : "接收器状态未知"; font.pixelSize: 11; color: displayModel && displayModel.healthReceiverRunning ? "#0B8F77" : "#D9822B"; Layout.fillWidth: true; elide: Text.ElideRight }
                    RowLayout {
                        Layout.fillWidth: true
                        Button { id: healthStartPanel; Layout.fillWidth: true; Layout.preferredHeight: 28; text: "启动接收"; enabled: displayModel ? !displayModel.healthReceiverRunning : true; background: Rectangle { color: healthStartPanel.enabled ? "#e8f8f4" : "#eef2f5"; radius: 6; border.color: "#bfe5db" }
                            contentItem: Text { text: healthStartPanel.text; color: "#0b6b5e"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStartClicked() }
                        Button { id: healthStopPanel; Layout.fillWidth: true; Layout.preferredHeight: 28; text: "停止接收"; enabled: displayModel ? displayModel.healthReceiverRunning : false; background: Rectangle { color: healthStopPanel.enabled ? "#fff0f0" : "#eef2f5"; radius: 6; border.color: "#f4b8b8" }
                            contentItem: Text { text: healthStopPanel.text; color: "#b42323"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            onClicked: root.healthReceiverStopClicked() }
                    }
                    Text { text: displayModel ? displayModel.healthMessage : "等待接收"; font.pixelSize: 10; color: "#64748b"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }

            Rectangle {
                // Keep charts in their own right-hand column so each trace is
                // readable at the default window size.
                Layout.preferredWidth: 330
                Layout.fillHeight: true
                radius: 8
                color: "#ffffff"
                border.color: displayModel ? displayModel.healthStatusColor : "#dbe5ef"
                border.width: 1

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 8

                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "体征趋势"; font.pixelSize: 14; font.weight: Font.Bold; color: "#1f2a37"; font.family: "PingFang SC, Microsoft YaHei UI" }
                        Item { Layout.fillWidth: true }
                        Text { text: "最近 10 分钟"; font.pixelSize: 10; color: "#64748b" }
                    }

                    Repeater {
                        model: [
                            { name: "心率", unit: "bpm", values: displayModel ? displayModel.healthHeartRateTrend : [], color: "#e85d75" },
                            { name: "血氧", unit: "%", values: displayModel ? displayModel.healthSpo2Trend : [], color: "#3d8bfd" },
                            { name: "体温", unit: "℃", values: displayModel ? displayModel.healthTemperatureTrend : [], color: "#f0a542" }
                        ]
                        delegate: Canvas {
                            id: trendChart
                            Layout.fillWidth: true
                            Layout.preferredHeight: 106
                            property var trendValues: modelData.values
                            property color trendColor: modelData.color
                            property string trendName: modelData.name
                            property string trendUnit: modelData.unit
                            Component.onCompleted: requestPaint()
                            onTrendValuesChanged: requestPaint()
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()
                            Connections { target: displayModel; function onHealthStateChanged() { trendChart.requestPaint() } }
                            onPaint: {
                                var ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                var values = trendValues || []
                                var left = 37, right = 9, top = 24, bottom = height - 17
                                ctx.font = "11px sans-serif"
                                ctx.fillStyle = "#334155"
                                ctx.fillText(trendName + "（" + trendUnit + "）", 1, 13)
                                if (values.length < 2) {
                                    ctx.fillStyle = "#8a94a6"
                                    ctx.fillText("等待更多有效数据", left, Math.round(height / 2))
                                    return
                                }
                                var low = values[0], high = values[0]
                                for (var i = 1; i < values.length; i++) {
                                    low = Math.min(low, values[i])
                                    high = Math.max(high, values[i])
                                }
                                var padding = Math.max(Math.abs(high - low) * 0.12, Math.abs(high) * 0.03, 0.5)
                                high += padding
                                low -= padding
                                ctx.strokeStyle = "#e7edf4"
                                ctx.lineWidth = 1
                                for (var grid = 0; grid < 3; grid++) {
                                    var gridY = top + grid * (bottom - top) / 2
                                    ctx.beginPath(); ctx.moveTo(left, gridY); ctx.lineTo(width - right, gridY); ctx.stroke()
                                }
                                ctx.font = "9px sans-serif"
                                ctx.fillStyle = "#8a94a6"
                                ctx.fillText(high.toFixed(1), 1, top + 3)
                                ctx.fillText(low.toFixed(1), 1, bottom + 3)
                                ctx.strokeStyle = trendColor
                                ctx.lineWidth = 2
                                ctx.lineJoin = "round"
                                ctx.lineCap = "round"
                                ctx.beginPath()
                                for (var point = 0; point < values.length; point++) {
                                    var x = left + point * (width - left - right) / (values.length - 1)
                                    var y = bottom - (values[point] - low) * (bottom - top) / (high - low)
                                    if (point === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                                }
                                ctx.stroke()
                                ctx.fillStyle = trendColor
                                for (var dot = 0; dot < values.length; dot++) {
                                    var dotX = left + dot * (width - left - right) / (values.length - 1)
                                    var dotY = bottom - (values[dot] - low) * (bottom - top) / (high - low)
                                    ctx.beginPath(); ctx.arc(dotX, dotY, 2.5, 0, Math.PI * 2); ctx.fill()
                                }
                            }
                        }
                    }
                    Item { Layout.fillHeight: true }
                    Text { text: displayModel ? displayModel.healthMessage : "等待接收"; font.pixelSize: 10; color: "#64748b"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 92
            color: "#f7f9fb"
            border.color: "#e1eaf3"
            border.width: 1

            ColumnLayout {
                anchors.fill: parent
                anchors.leftMargin: 14
                anchors.rightMargin: 14
                anchors.topMargin: 7
                anchors.bottomMargin: 7
                spacing: 5

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 36
                    spacing: 8

                    Button {
                        id: manualBtn
                        Layout.preferredWidth: 126
                        Layout.preferredHeight: 36
                        text: "按住后说话"
                        visible: displayModel ? !displayModel.autoMode : true
                        background: Rectangle { color: manualBtn.pressed ? "#0b6b5e" : (manualBtn.hovered ? "#12a87a" : "#0b8f77"); radius: 8 }
                        contentItem: Text { text: manualBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "white"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onPressed: { manualBtn.text = "松开停止"; root.manualButtonPressed() }
                        onReleased: { manualBtn.text = "按住后说话"; root.manualButtonReleased() }
                    }

                    Button {
                        id: autoBtn
                        Layout.preferredWidth: 126
                        Layout.preferredHeight: 36
                        text: displayModel ? displayModel.buttonText : "开始对话"
                        visible: displayModel ? displayModel.autoMode : false
                        background: Rectangle { color: autoBtn.pressed ? "#0b6b5e" : (autoBtn.hovered ? "#12a87a" : "#0b8f77"); radius: 8 }
                        contentItem: Text { text: autoBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "white"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.autoButtonClicked()
                    }

                    Button {
                        id: abortBtn
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 36
                        text: "打断对话"
                        background: Rectangle { color: abortBtn.pressed ? "#dfe5ec" : (abortBtn.hovered ? "#edf2f7" : "#e8eef5"); radius: 8 }
                        contentItem: Text { text: abortBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "#1f2a37"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.abortButtonClicked()
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 36
                        radius: 8
                        color: "white"
                        border.color: textInput.activeFocus ? "#0b8f77" : "#d8e2ed"
                        border.width: 1

                        TextInput {
                            id: textInput
                            anchors.fill: parent
                            anchors.leftMargin: 12
                            anchors.rightMargin: 12
                            verticalAlignment: TextInput.AlignVCenter
                            font.family: "PingFang SC, Microsoft YaHei UI"
                            font.pixelSize: 13
                            color: "#1f2a37"
                            selectByMouse: true
                            clip: true
                            Text { anchors.fill: parent; text: "输入语音等效指令..."; font: textInput.font; color: "#a8b3c2"; verticalAlignment: Text.AlignVCenter; visible: !textInput.text && !textInput.activeFocus }
                            Keys.onReturnPressed: {
                                if (textInput.text.trim().length > 0) {
                                    root.sendButtonClicked(textInput.text)
                                    textInput.text = ""
                                }
                            }
                        }
                    }

                    Button {
                        id: sendBtn
                        Layout.preferredWidth: 76
                        Layout.preferredHeight: 36
                        text: "发送"
                        background: Rectangle { color: sendBtn.pressed ? "#0b6b5e" : (sendBtn.hovered ? "#12a87a" : "#0b8f77"); radius: 8 }
                        contentItem: Text { text: sendBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "white"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: {
                            if (textInput.text.trim().length > 0) {
                                root.sendButtonClicked(textInput.text)
                                textInput.text = ""
                            }
                        }
                    }

                    Button {
                        id: modeBtn
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 36
                        text: displayModel ? displayModel.modeText : "手动对话"
                        background: Rectangle { color: modeBtn.pressed ? "#dfe5ec" : (modeBtn.hovered ? "#edf2f7" : "#e8eef5"); radius: 8 }
                        contentItem: Text { text: modeBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "#1f2a37"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.modeButtonClicked()
                    }

                    Button {
                        id: settingsBtn
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 36
                        text: "参数配置"
                        background: Rectangle { color: settingsBtn.pressed ? "#dfe5ec" : (settingsBtn.hovered ? "#edf2f7" : "#e8eef5"); radius: 8 }
                        contentItem: Text { text: settingsBtn.text; font.family: "PingFang SC, Microsoft YaHei UI"; font.pixelSize: 13; color: "#1f2a37"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.settingsButtonClicked()
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    spacing: 8

                    Text {
                        text: "快捷指令"
                        font.family: "PingFang SC, Microsoft YaHei UI"
                        font.pixelSize: 12
                        font.weight: Font.Bold
                        color: "#64748b"
                        Layout.preferredWidth: 64
                    }

                    Button {
                        id: quickDetectBack
                        Layout.preferredWidth: 112
                        Layout.preferredHeight: 30
                        text: "检测旁光经"
                        background: Rectangle { color: quickDetectBack.pressed ? "#d7f0eb" : (quickDetectBack.hovered ? "#e8f8f4" : "#eef8f5"); radius: 8; border.color: "#bfe5db" }
                        contentItem: Text { text: quickDetectBack.text; font.pixelSize: 12; color: "#0b6b5e"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("检测旁光经")
                    }

                    Button {
                        id: quickStart
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 30
                        text: "开始按摩"
                        background: Rectangle { color: quickStart.pressed ? "#d7f0eb" : (quickStart.hovered ? "#e8f8f4" : "#eef8f5"); radius: 8; border.color: "#bfe5db" }
                        contentItem: Text { text: quickStart.text; font.pixelSize: 12; color: "#0b6b5e"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("开始按摩")
                    }

                    Button {
                        id: quickShunJin
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 30
                        text: "顺筋按摩"
                        background: Rectangle { color: quickShunJin.pressed ? "#d7f0eb" : (quickShunJin.hovered ? "#e8f8f4" : "#eef8f5"); radius: 8; border.color: "#bfe5db" }
                        contentItem: Text { text: quickShunJin.text; font.pixelSize: 12; color: "#0b6b5e"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("开始顺筋")
                    }

                    Button {
                        id: quickPause
                        Layout.preferredWidth: 84
                        Layout.preferredHeight: 30
                        text: "暂停"
                        background: Rectangle { color: quickPause.pressed ? "#fff0d9" : (quickPause.hovered ? "#fff6e9" : "#fff8ed"); radius: 8; border.color: "#f4d4a1" }
                        contentItem: Text { text: quickPause.text; font.pixelSize: 12; color: "#9a5b12"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("暂停按摩")
                    }

                    Button {
                        id: quickResume
                        Layout.preferredWidth: 84
                        Layout.preferredHeight: 30
                        text: "继续"
                        background: Rectangle { color: quickResume.pressed ? "#d7f0eb" : (quickResume.hovered ? "#e8f8f4" : "#eef8f5"); radius: 8; border.color: "#bfe5db" }
                        contentItem: Text { text: quickResume.text; font.pixelSize: 12; color: "#0b6b5e"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("继续按摩")
                    }

                    Button {
                        id: quickStop
                        Layout.preferredWidth: 84
                        Layout.preferredHeight: 30
                        text: "停止"
                        background: Rectangle { color: quickStop.pressed ? "#fde2e2" : (quickStop.hovered ? "#fff0f0" : "#fff5f5"); radius: 8; border.color: "#f4b8b8" }
                        contentItem: Text { text: quickStop.text; font.pixelSize: 12; color: "#b42323"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("停止按摩")
                    }

                    Button {
                        id: quickStatus
                        Layout.preferredWidth: 96
                        Layout.preferredHeight: 30
                        text: "查询状态"
                        background: Rectangle { color: quickStatus.pressed ? "#dfe5ec" : (quickStatus.hovered ? "#edf2f7" : "#f3f7fb"); radius: 8; border.color: "#d8e2ed" }
                        contentItem: Text { text: quickStatus.text; font.pixelSize: 12; color: "#334155"; font.family: "PingFang SC, Microsoft YaHei UI"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                        onClicked: root.quickCommand("检测状态")
                    }

                    Item { Layout.fillWidth: true }
                }
            }
        }
    }
}
