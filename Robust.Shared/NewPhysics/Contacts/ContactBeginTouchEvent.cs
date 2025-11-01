namespace Robust.Shared.NewPhysics;

/// <summary>
/// A begin touch event is generated when two shapes begin touching.
/// </summary>
internal record struct ContactBeginTouchEvent
{
    /// <summary>
    /// Id of the first shape
    /// </summary>
    ShapeId shapeIdA;

    /// <summary>
    /// Id of the second shape
    /// </summary>
    ShapeId shapeIdB;

    /// The initial contact manifold. This is recorded before the solver is called,
    /// so all the impulses will be zero.
    b2Manifold manifold;
}
