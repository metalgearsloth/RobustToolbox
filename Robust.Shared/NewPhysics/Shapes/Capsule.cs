using System;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Shapes;

/// A solid capsule can be viewed as two semicircles connected
/// by a rectangle.
public record struct Capsule : IPhysShape
{
    /// Local center of the first semicircle
    [DataField]
    public Vector2 Center1;

    /// Local center of the second semicircle
    [DataField]
    public Vector2 Center2;

    public int ChildCount => 1;

    /// The radius of the semicircles
    public float Radius { get; set; }

    public ShapeType ShapeType => ShapeType.Capsule;

    public Capsule(Vector2 center1, Vector2 center2, float radius)
    {
        Center1 = center1;
        Center2 = center2;
        Radius = radius;
    }

    public bool Equals(IPhysShape? other)
    {
        if (other is not Capsule otherCapsule)
            return false;

        return Equals(otherCapsule);
    }

    public bool Equals(Capsule other)
    {
        return Center1.Equals(other.Center1) &&
               Center2.Equals(other.Center2) &&
               Radius.Equals(other.Radius);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Center1, Center2, Radius);
    }

    public Box2 ComputeAABB(Transform transform, int childIndex)
    {
        DebugTools.Assert(childIndex == 0);

        var v1 = Transform.TransformPoint(transform, Center1 );
        var v2 = Transform.TransformPoint( transform, Center2 );

        var r = new Vector2(Radius, Radius);
        var lower = Vector2.Min( v1, v2 ) - r;
        var upper = Vector2.Max( v1, v2 ) + r;

        var aabb = new Box2(lower, upper);
        return aabb;
    }
}
