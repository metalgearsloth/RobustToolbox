using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// A motor joint is used to control the relative velocity and or transform between two bodies.
/// With a velocity of zero this acts like top-down friction.
/// @ingroup motor_joint
public sealed partial class MotorJoint : BaseJoint
{
    /// The desired linear velocity
    [DataField]
    public Vector2 linearVelocity { get; internal set; }

    /// The maximum motor force in newtons
    [DataField]
    public float maxVelocityForce { get; internal set; }

    /// The desired angular velocity
    [DataField]
    public float angularVelocity { get; internal set; }

    /// The maximum motor torque in newton-meters
    [DataField]
    public float maxVelocityTorque { get; internal set; }

    /// Linear spring hertz for position control
    [DataField]
    public float linearHertz { get; internal set; }

    /// Linear spring damping ratio
    [DataField]
    public float linearDampingRatio { get; internal set; }

    /// Maximum spring force in newtons
    [DataField]
    public float maxSpringForce { get; internal set; }

    /// Angular spring hertz for position control
    [DataField]
    public float angularHertz { get; internal set; }

    /// Angular spring damping ratio
    [DataField]
    public float angularDampingRatio { get; internal set; }

    /// Maximum spring torque in newton-meters
    [DataField]
    public float maxSpringTorque { get; internal set; }

    internal Vector2 linearVelocityImpulse;
    internal float angularVelocityImpulse;
    internal Vector2 linearSpringImpulse;
    internal float angularSpringImpulse;

    internal Softness linearSpring;
    internal Softness angularSpring;

    internal int IndexA;
    internal int IndexB;
    internal Transform frameA;
    internal Transform frameB;
    internal Vector2 deltaCenter;
    internal Matrix22 linearMass;
    internal float angularMass;
}
