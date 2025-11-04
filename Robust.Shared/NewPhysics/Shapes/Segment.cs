using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Shapes;

public record struct Segment : IPhysShape
{
    /// The first point
    [DataField]
    public Vector2 Point1;

    /// The second point
    [DataField]
    public Vector2 Point2;

    public int ChildCount => 1;
    public float Radius { get; set; } = 0f;
    public ShapeType ShapeType { get; }

    public Segment()
    {

    }

    public bool Equals(IPhysShape? other)
    {
        throw new System.NotImplementedException();
    }

    public Box2 ComputeAABB(Transform transform, int childIndex)
    {
        throw new System.NotImplementedException();
    }
}
