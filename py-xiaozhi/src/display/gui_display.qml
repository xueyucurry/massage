import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    width: 920
    height: 560
    color: "#eef4f8"

    signal manualButtonPressed()
    signal manualButtonReleased()
    signal autoButtonClicked()
    signal abortButtonClicked()
    signal modeButtonClicked()
    signal sendButtonClicked(string text)
    signal settingsButtonClicked()
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
                Layout.preferredWidth: 260
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
