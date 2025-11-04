using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Distance joint definition
/// Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
/// @ingroup distance_joint
public sealed partial class DistanceJoint : BaseJoint
{
    /// The rest length of this joint. Clamped to a stable minimum value.
    [DataField]
    public float length;

    /// Enable the distance constraint to behave like a spring. If false
    /// then the distance joint will be rigid, overriding the limit and motor.
    [DataField]
    public bool enableSpring;

    /// The lower spring force controls how much tension it can sustain
    [DataField]
    public float lowerSpringForce;

    /// The upper spring force controls how much compression it an sustain
    [DataField]
    public float upperSpringForce;

    /// The spring linear stiffness Hertz, cycles per second
    [DataField]
    public float hertz;

    /// The spring linear damping ratio, non-dimensional
    [DataField]
    public float dampingRatio;

    /// Enable/disable the joint limit
    [DataField]
    public bool enableLimit;

    /// Minimum length. Clamped to a stable minimum value.
    [DataField]
    public float minLength;

    /// Maximum length. Must be greater than or equal to the minimum length.
    [DataField]
    public float maxLength;

    /// Enable/disable the joint motor
    [DataField]
    public bool enableMotor;

    /// The maximum motor force, usually in newtons
    [DataField]
    public float maxMotorForce;

    /// The desired motor speed, usually in meters per second
    [DataField]
    public float motorSpeed;

}
