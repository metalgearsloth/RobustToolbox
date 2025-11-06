using System;
using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
/// @ingroup distance_joint
public sealed partial class DistanceJoint : BaseJoint
{
    [NonSerialized]
    internal int IndexA;

    [NonSerialized]
    internal int IndexB;

    /// The rest length of this joint. Clamped to a stable minimum value.
    [DataField]
    public float length { get; internal set; } = 1f;

    /// Enable the distance constraint to behave like a spring. If false
    /// then the distance joint will be rigid, overriding the limit and motor.
    [DataField]
    public bool enableSpring { get; internal set; }

    /// The lower spring force controls how much tension it can sustain
    [DataField]
    public float lowerSpringForce { get; internal set; } = float.MinValue;

    /// The upper spring force controls how much compression it an sustain
    [DataField]

    public float upperSpringForce { get; internal set; } = float.MaxValue;

    /// The spring linear stiffness Hertz, cycles per second
    [DataField]
    public float hertz { get; internal set; }

    /// The spring linear damping ratio, non-dimensional
    [DataField]
    public float dampingRatio { get; internal set; }

    /// Enable/disable the joint limit
    [DataField]
    public bool enableLimit { get; internal set; }

    /// Minimum length. Clamped to a stable minimum value.
    [DataField]
    public float minLength { get; internal set; }

    /// Maximum length. Must be greater than or equal to the minimum length.
    [DataField]
    public float maxLength { get; internal set; } = PhysicsConstants.Huge;

    /// Enable/disable the joint motor
    [DataField]
    public bool enableMotor { get; internal set; }

    /// The maximum motor force, usually in newtons
    [DataField]
    public float maxMotorForce { get; internal set; }

    /// The desired motor speed, usually in meters per second
    [DataField]
    public float motorSpeed { get; internal set; }

    internal float Impulse;
    internal float LowerImpulse;
    internal float UpperImpulse;
    internal float MotorImpulse;

    internal Vector2 AnchorA;
    internal Vector2 AnchorB;
    internal Vector2 DeltaCenter;
    internal Softness DistanceSoftness;
    internal float AxialMass;
}
