using System.Numerics;
using System.Runtime.Intrinsics;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Math;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Contacts;

// Soft contact constraints with sub-stepping support
// Uses fixed anchors for Jacobians for better behavior on rolling shapes (circles & capsules)
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

internal record struct b2ContactConstraintSIMD
{
    // Even if no AVX we'll just use 8 lane.
    public FixedArray8<int> indexA;
    public FixedArray8<int> indexB;

    // You'll float too

    public FloatWide invMassA, invMassB;
    public FloatWide invIA, invIB;
    public Vector2Wide normal;
    public FloatWide friction;
    public FloatWide tangentSpeed;
    public FloatWide rollingResistance;
    public FloatWide rollingMass;
    public FloatWide rollingImpulse;
    public FloatWide biasRate;
    public FloatWide massScale;
    public FloatWide impulseScale;
    public Vector2Wide anchorA1, anchorB1;
    public FloatWide normalMass1, tangentMass1;
    public FloatWide baseSeparation1;
    public FloatWide normalImpulse1;
    public FloatWide totalNormalImpulse1;
    public FloatWide tangentImpulse1;
    public Vector2Wide anchorA2, anchorB2;
    public FloatWide baseSeparation2;
    public FloatWide normalImpulse2;
    public FloatWide totalNormalImpulse2;
    public FloatWide tangentImpulse2;
    public FloatWide normalMass2, tangentMass2;
    public FloatWide restitution;
    public FloatWide relativeVelocity1, relativeVelocity2;
}
