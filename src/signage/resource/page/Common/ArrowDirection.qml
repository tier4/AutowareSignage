import QtQuick 2.9
import QtQuick.Shapes 1.12

Shape {
    id: arrowShape
    width: 30*viewController.size_ratio
    height: 44*viewController.size_ratio
    ShapePath {
        fillColor: "white"
        fillRule: ShapePath.WindingFill
        strokeWidth: 0
        strokeColor: "white"
        startX: 0
        startY: 0
        PathLine {
            x: 30*viewController.size_ratio
            y: 22*viewController.size_ratio
        }
        PathLine {
            x: 0
            y: 44*viewController.size_ratio
        }
        PathLine {
            x: 0
            y: 0
        }
    }
}
