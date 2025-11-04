using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Weld joint definition
/// Connects two bodies together rigidly. This constraint provides springs to mimic
/// soft-body simulation.
/// @note The approximate solver in Box2D cannot hold many bodies together rigidly
/// @ingroup weld_joint
public sealed partial class WeldJoint : BaseJoint
{
    /// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
    [DataField]
    public float linearHertz;

    /// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
    [DataField]
    public float angularHertz;

    /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
    [DataField]
    public float linearDampingRatio;

    /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
    [DataField]
    public float angularDampingRatio;
}
