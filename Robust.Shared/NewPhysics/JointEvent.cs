namespace Robust.Shared.NewPhysics;

/// Joint events report joints that are awake and have a force and/or torque exceeding the threshold
/// The observed forces and torques are not returned for efficiency reasons.
public record struct JointEvent
{
    /// The joint id
    JointId jointId;
}
