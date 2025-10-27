namespace Robust.Shared.NewPhysics;

/// A begin touch event is generated when a shape starts to overlap a sensor shape.
internal record struct SensorBeginTouchEvent
{
    /// The id of the sensor shape
    ShapeId sensorShapeId;

    /// The id of the dynamic shape that began touching the sensor shape
    ShapeId visitorShapeId;
}
