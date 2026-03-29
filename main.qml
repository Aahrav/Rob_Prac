import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Window {
    width: 300
    height: 200
    visible: true
    title: "Hello World"
    readonly property list<string> hello: ["Hallo Welt", "Hei maailma", "Hola Mundo", "Привет мир"]
    function setText() {
        text.text = hello[Math.floor(Math.random() * hello.length)]
    }
    ColumnLayout {
        anchors.fill: parent
        Text{
            id:text
            text: "Hello World"
            Layout.alignment: Qt.AlignCenter
        }
        Button{
            text: "Click me!"
            Layout.alignment: Qt.AlignCenter
            onClicked: setText()
        }
    }
}
