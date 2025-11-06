using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Wheel joint definition
/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup wheel_joint
public sealed partial class WheelJoint : BaseJoint
{
    /// Enable a linear spring along the local axis
    [DataField]
    public bool enableSpring { get; internal set; }

    /// Spring stiffness in Hertz
    [DataField]
    public float hertz { get; internal set; }

    /// Spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio { get; internal set; }

    /// Enable/disable the joint linear limit
    [DataField]
    public bool enableLimit { get; internal set; }

    /// The lower translation limit
    [DataField]
    public float lowerTranslation { get; internal set; }

    /// The upper translation limit
    [DataField]
    public float upperTranslation { get; internal set; }

    /// Enable/disable the joint rotational motor
    [DataField]
    public bool enableMotor { get; internal set; }

    /// The maximum motor torque, typically in newton-meters
    [DataField]
    public float maxMotorTorque { get; internal set; }

    /// The desired motor speed in radians per second
    [DataField]
    public float motorSpeed { get; internal set; }

    internal float perpImpulse;
    internal float motorImpulse;
    internal float springImpulse;
    internal float lowerImpulse;
    internal float upperImpulse;

    internal int IndexA;
    internal int IndexB;
    internal Transform frameA;
    internal Transform frameB;
    internal Vector2 deltaCenter;
    internal float perpMass;
    internal float motorMass;
    internal float axialMass;
    internal Softness springSoftness;
}
