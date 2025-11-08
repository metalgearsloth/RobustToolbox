using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Joints;

// Map from b2JointId to b2Joint in the solver sets
[ImplicitDataDefinitionForInheritors]
public abstract partial class BaseJoint
{
    [DataField]
    public bool CollideConnected;

    // index of simulation set stored in b2World
    // PhysicsConstants.NullIndex when slot is free
    internal int SetIndex;

    // index into the constraint graph color array, may be PhysicsConstants.NullIndex for sleeping/disabled joints
    // PhysicsConstants.NullIndex when slot is free
    internal int ColorIndex;

    // joint index within set or graph color
    // PhysicsConstants.NullIndex when slot is free
    internal int LocalIndex;

    internal FixedArray2<JointEdge> Edges;

    internal int JointId;
    internal int IslandId;
    internal int IslandPrev;
    internal int IslandNext;

    internal float DrawScale;

    internal b2JointType Type;

    // This is monotonically advanced when a body is allocated in this slot
    // Used to check for invalid b2JointId
    internal ushort Generation;
}
