import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import jp.co.flagship 1.0

ApplicationWindow {
    id: root
    visible: true
    width: 1280
    height: 720
    minimumWidth: 800
    minimumHeight: 600
    title: qsTr("FlagShip")

    background: Rectangle { color: theme.winBg }

    header: Rectangle {
        height: 50
        color: theme.headerBg
        Text {
            text: qsTr("FlagShip: Robot Map Generator")
            color: theme.textCol
            font.pixelSize: 20
            font.bold: true
            anchors.verticalCenter: parent.verticalCenter
            anchors.left: parent.left
            anchors.leftMargin: 15
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 5

        MapView {
            id: map
            objectName: "mapViewItem"
            Layout.fillWidth: true
            Layout.fillHeight: true
            
            mapBackgroundColor: theme.mapBg
            gridLineColor: theme.gridCol
            
            resolution: parseInt(inpRes.text)
            mapWidth: parseInt(inpMapW.text)
            mapHeight: parseInt(inpMapH.text)
            showCenterCrosshair: swCross.checked
            showSafetyZone: swSafe.checked
            
            editMode: btnModeWp.checked ? MapView.Waypoint : 
                      btnModeObs.checked ? MapView.Obstacle : 
                      btnModeMove.checked ? MapView.Move : MapView.Erase

            focus: true
            
            Keys.onPressed: (event) => {
                if (event.key === Qt.Key_Return || event.key === Qt.Key_Enter) {
                    if (!inpObsW.activeFocus && !inpObsH.activeFocus) {
                        map.confirmObstaclePlacement();
                        event.accepted = true;
                    }
                } else if (event.key === Qt.Key_Escape) {
                    map.cancelObstaclePlacement();
                    event.accepted = true;
                } else if (event.key === Qt.Key_Delete || event.key === Qt.Key_Backspace) {
                    map.deleteSelectedObstacle();
                    map.deleteSelectedWaypoint();
                    event.accepted = true;
                } else if (event.modifiers & Qt.ControlModifier && event.key === Qt.Key_Z) {
                    if (event.modifiers & Qt.ShiftModifier) map.redo();
                    else map.undo();
                    event.accepted = true;
                } else if (event.modifiers & Qt.ControlModifier && event.key === Qt.Key_Y) {
                    map.redo();
                    event.accepted = true;
                }
            }

            onRequestDimensionInputFocus: {
                inpObsW.forceActiveFocus();
            }

            SequentialAnimation {
                loops: Animation.Infinite
                running: map.obstacleDrawingState === MapView.Confirming
                NumberAnimation {
                    target: map
                    property: "previewOpacity"
                    from: 1.0;
                    to: 0.4; duration: 700
                    easing.type: Easing.InOutQuad
                }
                NumberAnimation {
                    target: map
                    property: "previewOpacity"
                    from: 0.4;
                    to: 1.0; duration: 700
                    easing.type: Easing.InOutQuad
                }
            }

            MouseArea {
                anchors.fill: parent
                acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
                hoverEnabled: true
                property point pressPos: Qt.point(0,0)

                onPositionChanged: (mouse) => {
                    if (mouse.buttons & (Qt.RightButton | Qt.MiddleButton)) {
                        const dx = mouse.x - pressPos.x;
                        const dy = mouse.y - pressPos.y;
                        map.pan(-dx, -dy);
                    }
                    
                    map.updateMousePosition(Qt.point(mouse.x, mouse.y));
                    
                    if (mouse.buttons & Qt.LeftButton) {
                         if (map.editMode === MapView.Move) {
                             map.updateMoving(Qt.point(mouse.x, mouse.y));
                         }
                    }
                    
                    pressPos = Qt.point(mouse.x, mouse.y);
                }
                
                onEntered: map.updateMousePosition(Qt.point(mouseX, mouseY))
                onExited: map.mouseExited()
                
                onPressed: (mouse) => {
                    pressPos = Qt.point(mouse.x, mouse.y)
                    if (mouse.button === Qt.LeftButton) {
                        if (map.editMode === MapView.Move) {
                            map.startMoving(Qt.point(mouse.x, mouse.y));
                        }
                    }
                }

                onReleased: (mouse) => {
                    if (mouse.button === Qt.LeftButton) {
                        if (map.editMode === MapView.Move) {
                            map.finishMoving(Qt.point(mouse.x, mouse.y));
                        } else {
                            const d = Qt.point(mouse.x - pressPos.x, mouse.y - pressPos.y);
                            if (d.x*d.x + d.y*d.y < 25) {
                                map.handleMapClick(Qt.point(mouse.x, mouse.y), !!(mouse.modifiers & Qt.ControlModifier));
                                map.forceActiveFocus();
                            }
                        }
                    }
                }

                onWheel: (wheel) => {
                    const f = Math.pow(1.2, wheel.angleDelta.y / 120.0);
                    map.zoom(f, Qt.point(wheel.x, wheel.y));
                }
            }

            Connections {
                target: map
                function onPathfindingFailed(reason) {
                    popErr.text = reason;
                    popErr.open();
                }
                function onRequestLoopModeConfirmation() {
                    dlgLoop.open();
                }
                function onRequestNonLoopModeConfirmation() {
                    dlgNonLoop.open();
                }
            }

            TextField {
                id: inpObsW
                visible: map.dimensionInputsVisible
                x: map.widthInputPos.x - width / 2
                y: map.widthInputPos.y
                text: map.previewWidth.toLocaleString(Qt.locale(), 'f', 1)
                font.pixelSize: 12
                selectByMouse: true
                background: Rectangle { color: "#333"; border.color: "#999"; radius: 2 }
                color: "white"
                validator: DoubleValidator { bottom: 0; decimals: 1; notation: DoubleValidator.StandardNotation }
                onActiveFocusChanged: if (activeFocus) selectAll()
                onAccepted: {
                    map.setObstacleDimension(true, parseFloat(text));
                    inpObsH.forceActiveFocus();
                }
                Keys.onPressed: (e) => {
                    if (e.key === Qt.Key_Tab) {
                        map.setObstacleDimension(true, parseFloat(text));
                        inpObsH.forceActiveFocus();
                        e.accepted = true;
                    }
                }
            }

            TextField {
                id: inpObsH
                visible: map.dimensionInputsVisible
                x: map.heightInputPos.x
                y: map.heightInputPos.y - height / 2
                text: map.previewHeight.toLocaleString(Qt.locale(), 'f', 1)
                font.pixelSize: 12
                selectByMouse: true
                background: Rectangle { color: "#333"; border.color: "#999"; radius: 2 }
                color: "white"
                validator: DoubleValidator { bottom: 0; decimals: 1; notation: DoubleValidator.StandardNotation }
                onActiveFocusChanged: if (activeFocus) selectAll()
                onAccepted: {
                    map.setObstacleDimension(false, parseFloat(text));
                    map.confirmObstaclePlacement();
                }
                Keys.onPressed: (e) => {
                    if (e.key === Qt.Key_Backtab) {
                        map.setObstacleDimension(false, parseFloat(text));
                        inpObsW.forceActiveFocus();
                        e.accepted = true;
                    }
                }
            }
            
            Button {
                id: btnReset
                text: qsTr("Reset View")
                anchors.left: parent.left;
                anchors.top: parent.top; anchors.margins: 10
                onClicked: map.resetView()
                background: Rectangle {
                    color: parent.pressed ? Qt.darker(theme.btnSecBg) : theme.btnSecBg
                    radius: 5
                }
                contentItem: Text {
                    text: parent.text;
                    color: theme.btnTextCol
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter;
                    verticalAlignment: Text.AlignVCenter
                }
            }

            Button {
                id: btnClear
                text: qsTr("Clear All")
                anchors.left: parent.left;
                anchors.top: btnReset.bottom; anchors.margins: 10
                onClicked: map.clearWaypoints()
                background: Rectangle {
                    color: parent.pressed ? Qt.darker(theme.btnSecBg) : theme.btnSecBg
                    radius: 5
                }
                contentItem: Text {
                    text: parent.text;
                    color: theme.btnTextCol
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter;
                    verticalAlignment: Text.AlignVCenter
                }
            }

            Button {
                id: btnGen
                text: qsTr("Generate .hpp")
                font.bold: true
                anchors.right: parent.right; anchors.bottom: parent.bottom;
                anchors.margins: 10
                onClicked: backend.generateHppFile(
                    inpSpd.text, inpAng.text, inpRes.text, inpMapW.text, inpMapH.text
                )
                background: Rectangle {
                    color: parent.highlighted ? theme.btnPrimBg : Qt.darker(theme.btnPrimBg)
                    radius: 5
                }
                contentItem: Text {
                    text: parent.text;
                    color: theme.btnTextCol
                    font: parent.font
                    horizontalAlignment: Text.AlignHCenter;
                    verticalAlignment: Text.AlignVCenter
                }
                highlighted: true
            }
        }

        Rectangle {
            id: panel
            color: theme.panelBg
            radius: 8
            Layout.preferredWidth: 300
            Layout.fillHeight: true

            Flickable {
                anchors.fill: parent
                contentHeight: mainCol.height
                clip: true
                flickableDirection: Flickable.VerticalFlick

                ColumnLayout {
                    id: mainCol
                    width: panel.width
                    anchors.margins: 15
                    spacing: 10
                     
                    RowLayout {
                        Label { 
                            text: qsTr("Parameters");
                            color: theme.textCol
                            font { pixelSize: 18; bold: true }
                        }
                        Item { Layout.fillWidth: true }
                        Switch { 
                            text: qsTr("Light")
                            onToggled: theme.toggleTheme() 
                        }
                    }
                    
                    Rectangle { height: 1; color: theme.inpBorder; Layout.fillWidth: true; Layout.topMargin: 5; Layout.bottomMargin: 10 }

                    Label { text: qsTr("Pathfinding Mode"); color: theme.textCol; font.pixelSize: 16; }
                    ComboBox {
                        id: cmbAlgo
                        Layout.fillWidth: true
                        model: [qsTr("Direct"), qsTr("Waypoint-Strict"), qsTr("Waypoint-Guided")]
                        currentIndex: map.pathfindingMode
                        onCurrentIndexChanged: if (map.pathfindingMode !== currentIndex) map.pathfindingMode = currentIndex
                        Connections {
                            target: map
                            function onPathfindingModeChanged() {
                                if (cmbAlgo.currentIndex !== map.pathfindingMode) cmbAlgo.currentIndex = map.pathfindingMode
                            }
                        }
                    }
                    
                    CheckBox {
                        id: chkLoop
                        text: qsTr("Loop Path")
                        checked: map.loopPath
                        onCheckedChanged: map.loopPath = checked
                        Layout.topMargin: 5
                        contentItem: Text {
                            text: parent.text;
                            font: parent.font; color: theme.textCol
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: parent.indicator.width + parent.spacing
                        }
                    }

                    Rectangle { height: 1; color: theme.inpBorder; Layout.fillWidth: true; Layout.topMargin: 10; Layout.bottomMargin: 5 }

                    Label { text: qsTr("Display & Edit"); color: theme.textCol; font.pixelSize: 16; }
                    ColumnLayout {
                        Switch {
                            id: swSafe
                            text: qsTr("Visualize Safety Zone")
                            contentItem: Text {
                                text: parent.text
                                font: parent.font
                                color: theme.textCol
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: parent.indicator.width + parent.spacing
                            }
                        }
                    }

                    GridLayout {
                        columns: 2
                        ToolButton {
                            id: btnModeWp
                            text: qsTr("Waypoint")
                            checkable: true; checked: true
                            onClicked: { 
                                if(checked) { btnModeObs.checked = false; btnModeErase.checked = false; btnModeMove.checked = false; } else checked = true 
                            }
                        }
                        ToolButton {
                            id: btnModeObs
                            text: qsTr("Obstacle")
                            checkable: true
                            onClicked: { 
                                if(checked) { btnModeWp.checked = false; btnModeErase.checked = false; btnModeMove.checked = false; } else checked = true 
                            }
                        }
                        ToolButton {
                            id: btnModeMove
                            text: qsTr("Move")
                            checkable: true
                            onClicked: { 
                                if(checked) { btnModeWp.checked = false; btnModeObs.checked = false; btnModeErase.checked = false; } else checked = true 
                            }
                        }
                        ToolButton {
                            id: btnModeErase
                            text: qsTr("Eraser")
                            checkable: true
                            onClicked: { 
                                if(checked) { btnModeWp.checked = false; btnModeObs.checked = false; btnModeMove.checked = false; } else checked = true 
                            }
                        }
                    }
                   
                    ColumnLayout {
                        visible: map.selectedWaypointIndex !== -1 && btnModeWp.checked
                        Layout.topMargin: 10
                        
                        Label { 
                            text: qsTr("Mode for Waypoint ") + map.selectedWaypointIndex
                            color: theme.textCol; font.bold: true 
                        }
                        ComboBox {
                            id: cmbWpMode
                            model: [qsTr("Safe"), qsTr("Aggressive")]
                            currentIndex: map.selectedWaypointMode
                            onCurrentIndexChanged: if (map.selectedWaypointMode !== currentIndex) map.selectedWaypointMode = currentIndex
                            Connections {
                                target: map
                                function onSelectedWaypointModeChanged() {
                                    if (cmbWpMode.currentIndex !== map.selectedWaypointMode) cmbWpMode.currentIndex = map.selectedWaypointMode
                                }
                            }
                        }
                    }

                    Rectangle { height: 1; color: theme.inpBorder; Layout.fillWidth: true; Layout.topMargin: 10; Layout.bottomMargin: 5 }

                    Label { text: qsTr("Robot & Path Parameters"); color: theme.textCol; font.pixelSize: 16; }
                    
                    ColumnLayout {
                        Layout.fillWidth: true
                        visible: cmbAlgo.currentIndex === 1 || cmbAlgo.currentIndex === 2
                        Label { text: qsTr("Curve Tension"); color: theme.textCol }
                        RowLayout {
                            Slider {
                                id: sldTension
                                from: 0.0; to: 1.0; stepSize: 0.05
                                value: map.smoothingTension
                                onMoved: map.smoothingTension = value
                                Layout.fillWidth: true
                            }
                            Label { text: sldTension.value.toLocaleString(Qt.locale(), 'f', 2); color: theme.textCol; Layout.minimumWidth: 30 }
                        }
                    }

                    Label { text: qsTr("Safety Threshold (Multiplier)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    RowLayout {
                        Slider {
                            id: sldSafe
                            from: 1.0; to: 3.0; stepSize: 0.1
                            value: map.safetyThreshold
                            onMoved: map.safetyThreshold = value
                            Layout.fillWidth: true
                            Connections {
                                target: map
                                function onSafetyThresholdChanged() { if (sldSafe.value !== map.safetyThreshold) sldSafe.value = map.safetyThreshold }
                            }
                        }
                        Label { text: sldSafe.value.toLocaleString(Qt.locale(), 'f', 1); color: theme.textCol; Layout.minimumWidth: 30 }
                    }

                    Label { text: qsTr("Robot Width (mm)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 5 }
                    TextField {
                        id: inpRobW
                        Layout.fillWidth: true
                        color: theme.textCol
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: DoubleValidator { bottom: 0; top: 10000; decimals: 1; notation: DoubleValidator.StandardNotation }
                        onEditingFinished: if (text) map.robotWidth = parseFloat(text)
                        Component.onCompleted: text = map.robotWidth.toFixed(1)
                        Connections {
                            target: map
                            function onRobotSizeChanged() { if (parseFloat(inpRobW.text) !== map.robotWidth) inpRobW.text = map.robotWidth.toFixed(1) }
                        }
                    }

                    Label { text: qsTr("Robot Height (mm)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    TextField {
                        id: inpRobH
                        Layout.fillWidth: true
                        color: theme.textCol
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: DoubleValidator { bottom: 0; top: 10000; decimals: 1; notation: DoubleValidator.StandardNotation }
                        onEditingFinished: if (text) map.robotHeight = parseFloat(text)
                        Component.onCompleted: text = map.robotHeight.toFixed(1)
                        Connections {
                            target: map
                            function onRobotSizeChanged() { if (parseFloat(inpRobH.text) !== map.robotHeight) inpRobH.text = map.robotHeight.toFixed(1) }
                        }
                    }
                    
                    RowLayout {
                        Layout.topMargin: 10
                        Button {
                            text: qsTr("Set as Start")
                            onClicked: map.setStartPoint()
                            visible: !chkLoop.checked 
                        }
                        Button {
                            text: qsTr("Set as Goal")
                            onClicked: map.setGoalPoint()
                            visible: !chkLoop.checked
                        }
                        Button {
                            text: qsTr("Set as Loop Point")
                            onClicked: map.setLoopStartPoint()
                            visible: chkLoop.checked
                        }
                    }
      
                    Button {
                        id: btnFind
                        text: qsTr("Find Path")
                        onClicked: map.findPath()
                        Layout.topMargin: 5
                        font.bold: true
                    }

                    Rectangle { height: 1; color: theme.inpBorder; Layout.fillWidth: true; Layout.topMargin: 10; Layout.bottomMargin: 5 }
                    
                    Label { text: qsTr("Map Parameters"); color: theme.textCol; font.pixelSize: 16; }
                    
                    Label { text: qsTr("Map Resolution (mm/cell)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 5 }
                    TextField {
                        id: inpRes
                        placeholderText: "e.g., 10"
                        Layout.fillWidth: true
                        color: theme.textCol; text: "10"
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: IntValidator { bottom: 1; top: 100; }
                        onEditingFinished: if (text) map.resolution = parseInt(text)
                    }

                    Label { text: qsTr("Map Width (cells)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    TextField {
                        id: inpMapW
                        placeholderText: "e.g., 150"
                        Layout.fillWidth: true
                        color: theme.textCol; text: "150"
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: IntValidator { bottom: 1; }
                        onEditingFinished: if (text) map.mapWidth = parseInt(text)
                    }
                    
                    Label { text: qsTr("Map Height (cells)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    TextField {
                        id: inpMapH
                        placeholderText: "e.g., 150"
                        Layout.fillWidth: true
                        color: theme.textCol; text: "150"
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: IntValidator { bottom: 1; }
                        onEditingFinished: if (text) map.mapHeight = parseInt(text)
                    }
                    
                    Label {
                        text: qsTr("Field Edge Threshold (mm)")
                        color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10
                    }
                    RowLayout {
                        Slider {
                            id: sldEdge
                            from: 0; to: 300; stepSize: 5
                            Layout.fillWidth: true
                            value: map.fieldEdgeThreshold
                            onMoved: map.fieldEdgeThreshold = value
                        }
                        Label {
                            text: sldEdge.value.toFixed(0) + " mm"
                            color: theme.textCol; Layout.minimumWidth: 50
                        }
                    }

                    Label { text: qsTr("Default Speed (mm/s)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    TextField {
                        id: inpSpd
                        placeholderText: "e.g., 500"
                        Layout.fillWidth: true
                        color: theme.textCol
                        placeholderTextColor: theme.textMuted
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                    }
                    
                    Label { text: qsTr("Default Angle (degree)"); color: theme.textCol; font.pixelSize: 14; Layout.topMargin: 10 }
                    TextField {
                        id: inpAng
                        placeholderText: "e.g., 90"
                        Layout.fillWidth: true
                        color: theme.textCol
                        background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        validator: DoubleValidator { bottom: -360; top: 360; decimals: 1; notation: DoubleValidator.StandardNotation }
                        onEditingFinished: if (text) map.robotAngle = parseFloat(text)
                        Component.onCompleted: text = map.robotAngle.toFixed(1)
                        Connections {
                            target: map
                            function onRobotAngleChanged() { if (parseFloat(inpAng.text) !== map.robotAngle) inpAng.text = map.robotAngle.toFixed(1) }
                        }
                    }
                    
                    Item { Layout.fillHeight: true }

                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 8
                        Label {
                            text: qsTr("Namespace")
                            color: theme.textCol
                            font.pixelSize: 14
                            Layout.alignment: Qt.AlignVCenter
                        }
                        TextField {
                            id: inpNs
                            Layout.fillWidth: true
                            color: theme.textCol
                            placeholderText: "PathData"
                            text: backend.namespaceName
                            onTextChanged: backend.namespaceName = text
                            background: Rectangle { color: theme.inpBg; radius: 4; border.color: theme.inpBorder }
                        }
                    }
                }
            }
        }
    }

    Popup {
        id: popErr
        y: root.height - (height + 20)
        x: (root.width - width) / 2
        width: 400
        height: errTxt.implicitHeight + 20
        modal: false
        dim: false
        closePolicy: Popup.CloseOnEscape

        property var autoClose: Timer {
            interval: 4000
            onTriggered: popErr.close()
        }
        onOpened: autoClose.start()
        onClosed: autoClose.stop()

        property alias text: errTxt.text

        background: Rectangle {
            color: "#D32F2F"
            radius: 8
            border.color: Qt.darker("#D32F2F"); border.width: 1
        }

        contentItem: Text {
            id: errTxt
            text: "Error"
            color: "white"
            font.bold: true
            wrapMode: Text.WordWrap
            padding: 10
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    Dialog {
        id: dlgLoop
        title: qsTr("Confirm Loop Mode")
        standardButtons: Dialog.Yes | Dialog.No
        modal: true
        width: 400
        
        contentItem: Text {
            text: qsTr("Enabling loop mode will clear all existing Waypoints, Start, and Goal points. Continue?")
            wrapMode: Text.WordWrap
            padding: 10
        }
        onAccepted: map.confirmLoopModeActivation();
        onRejected: {} 
    }

    Dialog {
        id: dlgNonLoop
        title: qsTr("Confirm Non-Loop Mode")
        standardButtons: Dialog.Yes | Dialog.No
        modal: true
        width: 400
        
        contentItem: Text {
            text: qsTr("Disabling loop mode will clear all existing Waypoints. Continue?")
            wrapMode: Text.WordWrap
            padding: 10
        }
        onAccepted: map.confirmNonLoopModeActivation();
        onRejected: {}
    }
}
