import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

RowLayout {
    id: panel
    property var healthModel: null
    spacing: 10

    Rectangle {
        Layout.preferredWidth: 210
        Layout.fillHeight: true
        radius: 8; color: "#ffffff"; border.color: "#dbe5ef"
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 10; spacing: 8
            RowLayout {
                Layout.fillWidth: true
                Text { text: "健康监测"; font.pixelSize: 13; font.weight: Font.Bold; color: "#1f2a37" }
                Item { Layout.fillWidth: true }
                Rectangle { Layout.preferredWidth: 58; Layout.preferredHeight: 20; radius: 10; color: healthModel ? healthModel.badgeColor : "#8A94A6"
                    Text { anchors.centerIn: parent; text: healthModel ? healthModel.badgeLabel : "等待接收"; font.pixelSize: 9; color: "white" }
                }
            }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: healthModel && healthModel.massageRecommendationText.length > 0 ? 145 : 105; radius: 6; color: "#f8fafc"; border.color: "#e6edf4"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 6; spacing: 3
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "个人资料"; font.pixelSize: 10; color: "#64748b" }
                        Item { Layout.fillWidth: true }
                        Text { text: healthModel ? healthModel.bodySourceText : "手动录入"; font.pixelSize: 9; color: "#8a94a6" }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 6
                        ColumnLayout {
                            Layout.fillWidth: true; spacing: 0
                            Text { text: "身高"; font.pixelSize: 9; color: "#64748b" }
                            Text { text: healthModel ? healthModel.heightText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                        }
                        ColumnLayout {
                            Layout.fillWidth: true; spacing: 0
                            Text { text: "体重"; font.pixelSize: 9; color: "#64748b" }
                            Text { text: healthModel ? healthModel.weightText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                        }
                        ColumnLayout {
                            Layout.fillWidth: true; spacing: 0
                            Text { text: "BMI"; font.pixelSize: 9; color: "#64748b" }
                            Text { text: healthModel ? healthModel.bmiText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                        }
                    }
                    Text { text: healthModel ? healthModel.massageRecommendationText : ""; visible: text.length > 0; font.pixelSize: 9; color: "#0b6b5e"; Layout.fillWidth: true; Layout.preferredHeight: visible ? 40 : 0; wrapMode: Text.WordWrap; verticalAlignment: Text.AlignVCenter }
                    Rectangle {
                        Layout.fillWidth: true; Layout.preferredHeight: 24; radius: 5; color: "#ffffff"; border.color: "#d8e2ed"
                        RowLayout {
                            anchors.fill: parent; anchors.leftMargin: 4; anchors.rightMargin: 3; spacing: 3
                            Text { text: "身高"; font.pixelSize: 8; color: "#64748b" }
                            TextField { id: heightInput; Layout.preferredWidth: 43; Layout.preferredHeight: 22; padding: 0; placeholderText: "cm"; font.pixelSize: 9; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; selectByMouse: true; validator: DoubleValidator { bottom: 50; top: 250; decimals: 1 }
                                background: Rectangle { radius: 3; color: "transparent"; border.color: "#d8e2ed" }
                            }
                            Text { text: "体重"; font.pixelSize: 8; color: "#64748b" }
                            TextField { id: weightInput; Layout.preferredWidth: 43; Layout.preferredHeight: 22; padding: 0; placeholderText: "kg"; font.pixelSize: 9; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter; selectByMouse: true; validator: DoubleValidator { bottom: 10; top: 500; decimals: 1 }
                                background: Rectangle { radius: 3; color: "transparent"; border.color: "#d8e2ed" }
                            }
                            Item { Layout.fillWidth: true }
                            Button { id: saveMetrics; text: "保存"; Layout.preferredWidth: 34; Layout.preferredHeight: 20; enabled: heightInput.acceptableInput && weightInput.acceptableInput
                                contentItem: Text { text: saveMetrics.text; font.pixelSize: 8; color: "#0b6b5e"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                background: Rectangle { radius: 4; color: "#f2f8f6"; border.color: "#bfe5db" }
                                onClicked: { healthModel.saveManualMetrics(heightInput.text, weightInput.text); heightInput.text = ""; weightInput.text = "" }
                            }
                        }
                    }
                }
            }
            GridLayout {
                Layout.fillWidth: true; columns: 2; rowSpacing: 5; columnSpacing: 8
                Text { text: "心率"; font.pixelSize: 10; color: "#64748b" }
                Text { text: healthModel ? healthModel.heartRateText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                Text { text: "血氧"; font.pixelSize: 10; color: "#64748b" }
                Text { text: healthModel ? healthModel.spo2Text : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                Text { text: "体温"; font.pixelSize: 10; color: "#64748b" }
                Text { text: healthModel ? healthModel.temperatureText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
                Text { text: "电量"; font.pixelSize: 10; color: "#64748b" }
                Text { text: healthModel ? healthModel.batteryText : "--"; font.pixelSize: 12; font.weight: Font.DemiBold; color: "#334155" }
            }
            Rectangle { Layout.fillWidth: true; Layout.preferredHeight: 1; color: "#e1eaf3" }
            Text { text: healthModel ? ((healthModel.simulatedSource ? "模拟数据库 · " : "") + healthModel.receiverText) : "接收器状态未知"; font.pixelSize: 10; color: healthModel && healthModel.simulatedSource ? "#B76E00" : (healthModel && healthModel.receiverRunning ? "#0B8F77" : "#D9822B"); Layout.fillWidth: true; elide: Text.ElideRight }
            RowLayout {
                Layout.fillWidth: true
                Button { id: startReceiver; text: "启动接收"; Layout.fillWidth: true; Layout.preferredHeight: 26; enabled: healthModel ? !healthModel.receiverRunning : true; onClicked: healthModel.startReceiver()
                    contentItem: Text { text: startReceiver.text; font.pixelSize: 10; color: "#0b6b5e"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    background: Rectangle { radius: 5; color: "#f2f8f6"; border.color: "#bfe5db" }
                }
                Button { id: stopReceiver; text: "停止接收"; Layout.fillWidth: true; Layout.preferredHeight: 26; enabled: healthModel ? healthModel.receiverRunning : false; onClicked: healthModel.stopReceiver()
                    contentItem: Text { text: stopReceiver.text; font.pixelSize: 10; color: "#b42323"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                    background: Rectangle { radius: 5; color: "#fff5f5"; border.color: "#f4b8b8" }
                }
            }
            Text { text: healthModel ? healthModel.message : "等待首条有效体征数据"; font.pixelSize: 10; color: "#64748b"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
        }
    }

    Rectangle {
        Layout.preferredWidth: 280
        Layout.fillHeight: true
        radius: 8; color: "#ffffff"; border.color: "#dbe5ef"
        ColumnLayout {
            anchors.fill: parent; anchors.margins: 10; spacing: 7
            RowLayout {
                Layout.fillWidth: true
                Text { text: "体征趋势"; font.pixelSize: 13; font.weight: Font.Bold; color: "#1f2a37" }
                Item { Layout.fillWidth: true }
                Text { text: "每"; font.pixelSize: 9; color: "#64748b" }
                TextField {
                    id: bucketInput; Layout.preferredWidth: 36; Layout.preferredHeight: 26; text: "30"; font.pixelSize: 10; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                    validator: IntValidator { bottom: 5; top: 300 }
                    selectByMouse: true
                    Component.onCompleted: text = healthModel ? String(healthModel.bucketSeconds) : "30"
                    background: Rectangle { radius: 3; color: "transparent"; border.color: "#dbe5ef" }
                }
                Text { text: "秒/点"; font.pixelSize: 9; color: "#64748b" }
                Button { id: applyBucket; text: "应用"; Layout.preferredWidth: 32; Layout.preferredHeight: 24; onClicked: healthModel.setTrendBucketSeconds(bucketInput.text)
                    contentItem: Text { text: applyBucket.text; font.pixelSize: 8; color: "#0b6b5e"; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                }
            }
            Text { text: "最近 5 分钟 · 最多显示 12 点"; font.pixelSize: 9; color: "#8a94a6" }
            Repeater {
                model: [
                    { name: "心率", unit: "bpm", decimals: 0, values: healthModel ? healthModel.heartRateTrend : [], color: "#e85d75" },
                    { name: "血氧", unit: "%", decimals: 0, values: healthModel ? healthModel.spo2Trend : [], color: "#3d8bfd" },
                    { name: "体温", unit: "℃", decimals: 1, values: healthModel ? healthModel.temperatureTrend : [], color: "#f0a542" }
                ]
                delegate: Canvas {
                    id: trendChart; Layout.fillWidth: true; Layout.preferredHeight: healthModel && healthModel.alertMessage.length > 0 ? 80 : 91
                    property var values: modelData.values; property color lineColor: modelData.color; property int decimals: modelData.decimals
                    onValuesChanged: requestPaint(); onWidthChanged: requestPaint(); onHeightChanged: requestPaint()
                    Connections { target: healthModel; function onStateChanged() { trendChart.requestPaint() } }
                    onPaint: {
                        var ctx = getContext("2d"); ctx.clearRect(0, 0, width, height)
                        var points = values || [], left = 7, right = 7, top = 25, bottom = height - 10
                        ctx.font = "10px sans-serif"; ctx.fillStyle = "#334155"; ctx.fillText(modelData.name + "（" + modelData.unit + "）", 0, 12)
                        if (points.length < 2) { ctx.fillStyle = "#8a94a6"; ctx.fillText("等待更多有效数据", left, Math.round(height / 2)); return }
                        var low = points[0], high = points[0]
                        for (var i = 1; i < points.length; i++) { low = Math.min(low, points[i]); high = Math.max(high, points[i]) }
                        var pad = Math.max((high - low) * 0.15, Math.abs(high) * 0.03, 0.5); low -= pad; high += pad
                        ctx.strokeStyle = "#e7edf4"; ctx.lineWidth = 1
                        for (var grid = 0; grid < 3; grid++) { var gy = top + grid * (bottom - top) / 2; ctx.beginPath(); ctx.moveTo(left, gy); ctx.lineTo(width-right, gy); ctx.stroke() }
                        ctx.strokeStyle = lineColor; ctx.lineWidth = 1.8; ctx.lineJoin = "round"; ctx.beginPath()
                        for (var p = 0; p < points.length; p++) { var x = left + p * (width-left-right)/(points.length-1); var y = bottom - (points[p]-low)*(bottom-top)/(high-low); if (p === 0) ctx.moveTo(x,y); else ctx.lineTo(x,y) }
                        ctx.stroke(); ctx.fillStyle = lineColor; ctx.font = "8px sans-serif"; ctx.textAlign = "center"
                        for (var dot = 0; dot < points.length; dot++) {
                            var dx = left + dot*(width-left-right)/(points.length-1)
                            var dy = bottom-(points[dot]-low)*(bottom-top)/(high-low)
                            ctx.beginPath(); ctx.arc(dx,dy,2.2,0,Math.PI*2); ctx.fill()
                            ctx.textAlign = dot === 0 ? "left" : (dot === points.length-1 ? "right" : "center")
                            ctx.fillText(points[dot].toFixed(decimals), dx, Math.max(top+8,dy-5))
                        }
                        ctx.textAlign = "left"
                    }
                }
            }
            Item { Layout.fillHeight: true }
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: visible ? 54 : 0
                visible: healthModel && healthModel.alertMessage.length > 0
                radius: 5
                color: healthModel && healthModel.alertColor === "#B42323" ? "#fff1ef" : "#fff8e8"
                border.color: healthModel ? healthModel.alertColor : "#B76E00"
                Text { anchors.fill: parent; anchors.margins: 5; text: healthModel ? healthModel.alertMessage : ""; font.pixelSize: 11; font.weight: Font.DemiBold; color: healthModel ? healthModel.alertColor : "#B76E00"; wrapMode: Text.WordWrap; verticalAlignment: Text.AlignVCenter }
            }
        }
    }
}
