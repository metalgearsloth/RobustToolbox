using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Math;

internal record struct Vector2Wide
{
    // We have this and not floatWide so we can do the X / Y access more easily
    public FixedArray8<float> X;
    public FixedArray8<float> Y;
}
