import QtQuick 2.9
import QtQuick.Shapes 1.12

Shape {
    id: arrowShape
    width: 30
    height: 44
    ShapePath {
        fillColor: "white"
        fillRule: ShapePath.WindingFill
        strokeWidth: 0
        strokeColor: "white"
        startX: 0
        startY: 0
        PathLine {
            x: 30
            y: 22
        }
        PathLine {
            x: 0
            y: 44
        }
        PathLine {
            x: 0
            y: 0
        }
    }
}
