using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Bodies;

internal record struct BodyStateWide
{
    public FixedArray8<float> vX;
    public FixedArray8<float> vY;

    // Angular
    public FixedArray8<float> w;

    public FixedArray8<float> flags;

    // Transform pos
    public FixedArray8<float> dpX;
    public FixedArray8<float> dpY;

    // Transform rot
    public FixedArray8<float> dqC;
    public FixedArray8<float> dqS;
}
