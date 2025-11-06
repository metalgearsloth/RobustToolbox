using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Math;

internal record struct Vector2Wide
{
    public static readonly Vector2Wide Zero = new()
    {
        X = new FixedArray8<float>(),
        Y = new FixedArray8<float>()
    };

    public FixedArray8<float> X;
    public FixedArray8<float> Y;
}
