using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;

namespace Robust.Shared.NewPhysics.Shapes;

public record struct ChainSegment : IPhysShape
{
    public bool Equals(IPhysShape? other)
    {
        throw new System.NotImplementedException();
    }

    public int ChildCount { get; }
    public float Radius { get; set; }
    public ShapeType ShapeType { get; }
    public Box2 ComputeAABB(Transform transform, int childIndex)
    {
        throw new System.NotImplementedException();
    }
}
