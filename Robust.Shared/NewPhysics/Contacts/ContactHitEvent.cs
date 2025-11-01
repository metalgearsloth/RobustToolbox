using System.Numerics;

namespace Robust.Shared.NewPhysics;

/// A hit touch event is generated when two shapes collide with a speed faster than the hit speed threshold.
public record struct ContactHitEvent
{
    /// Id of the first shape
    ShapeId shapeIdA;

    /// Id of the second shape
    ShapeId shapeIdB;

    /// Point where the shapes hit
    Vector2 point;

    /// Normal vector pointing from shape A to shape B
    Vector2 normal;

    /// The speed the shapes are approaching. Always positive. Typically in meters per second.
    float approachSpeed;
}
