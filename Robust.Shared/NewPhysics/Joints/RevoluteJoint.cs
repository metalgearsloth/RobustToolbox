using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation.
/// @ingroup revolute_joint
public sealed partial class RevoluteJoint : BaseJoint
{
    /// The target angle for the joint in radians. The spring-damper will drive
    /// to this angle.
    [DataField]
    public float targetAngle { get; internal set; }

    /// Enable a rotational spring on the revolute hinge axis
    [DataField]
    public bool enableSpring { get; internal set; }

    /// The spring stiffness Hertz, cycles per second
    [DataField]
    public float hertz { get; internal set; }

    /// The spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio { get; internal set; }

    /// A flag to enable joint limits
    [DataField]
    public bool enableLimit { get; internal set; }

    /// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
    [DataField]
    public float lowerAngle { get; internal set; }

    /// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
    [DataField]
    public float upperAngle { get; internal set; }

    /// A flag to enable the joint motor
    [DataField]
    public bool enableMotor { get; internal set; }

    /// The maximum motor torque, typically in newton-meters
    [DataField]
    public float maxMotorTorque { get; internal set; }

    /// The desired motor speed in radians per second
    [DataField]
    public float motorSpeed { get; internal set; }

    internal Vector2 linearImpulse;
    internal float springImpulse;
    internal float motorImpulse;
    internal float lowerImpulse;
    internal float upperImpulse;

    internal int IndexA;
    internal int IndexB;
    internal Transform frameA;
    internal Transform frameB;
    internal Vector2 deltaCenter;
    internal float axialMass;
    internal Softness springSoftness;
}
