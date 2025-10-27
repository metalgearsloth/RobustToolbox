namespace Robust.Shared.NewPhysics;

/// An end touch event is generated when a shape stops overlapping a sensor shape.
///	These include things like setting the transform, destroying a body or shape, or changing
///	a filter. You will also get an end event if the sensor or visitor are destroyed.
///	Therefore you should always confirm the shape id is valid using b2Shape_IsValid.
public record struct SensorEndTouchEvent
{
    /// The id of the sensor shape
    ///	@warning this shape may have been destroyed
    ///	@see b2Shape_IsValid
    ShapeId sensorShapeId;

    /// The id of the shape that stopped touching the sensor shape
    ///	@warning this shape may have been destroyed
    ///	@see b2Shape_IsValid
    ShapeId visitorShapeId;

}
