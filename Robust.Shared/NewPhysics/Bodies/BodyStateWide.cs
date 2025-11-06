using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Bodies;

internal record struct BodyStateWide
{
    public FixedArray8<Vector2> v;
    public FixedArray8<float> w;
    public FixedArray8<float> flags;
    public FixedArray8<Vector2> dp;
    public FixedArray8<Quaternion2D> dq;
}
