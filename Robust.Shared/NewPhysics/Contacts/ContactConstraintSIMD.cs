using System.Numerics;
using System.Runtime.Intrinsics;
using Robust.Shared.Maths;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Contacts;

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

internal record struct b2ContactConstraintSIMD
{
    // Even if no AVX we'll just use 8 lane.
    // TODO: Use Vector256?

    public FixedArray8<int> indexA;
    public FixedArray8<int> indexB;

    // You'll float too

    public FixedArray8<float> invMassA, invMassB;
    public FixedArray8<float> invIA, invIB;
    public FixedArray8<Vector2> normal;
    public FixedArray8<float> friction;
    public FixedArray8<float> tangentSpeed;
    public FixedArray8<float> rollingResistance;
    public FixedArray8<float> rollingMass;
    public FixedArray8<float> rollingImpulse;
    public FixedArray8<float> biasRate;
    public FixedArray8<float> massScale;
    public FixedArray8<float> impulseScale;
    public FixedArray8<Vector2> anchorA1, anchorB1;
    public FixedArray8<float> normalMass1, tangentMass1;
    public FixedArray8<float> baseSeparation1;
    public FixedArray8<float> normalImpulse1;
    public FixedArray8<float> totalNormalImpulse1;
    public FixedArray8<float> tangentImpulse1;
    public FixedArray8<Vector2> anchorA2, anchorB2;
    public FixedArray8<float> baseSeparation2;
    public FixedArray8<float> normalImpulse2;
    public FixedArray8<float> totalNormalImpulse2;
    public FixedArray8<float> tangentImpulse2;
    public FixedArray8<float> normalMass2, tangentMass2;
    public FixedArray8<float> restitution;
    public FixedArray8<float> relativeVelocity1, relativeVelocity2;
}
