using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Joints;

// Map from b2JointId to b2Joint in the solver sets
internal sealed class b2Joint
{
    // index of simulation set stored in b2World
    // B2_NULL_INDEX when slot is free
    int setIndex;

    // index into the constraint graph color array, may be B2_NULL_INDEX for sleeping/disabled joints
    // B2_NULL_INDEX when slot is free
    int colorIndex;

    // joint index within set or graph color
    // B2_NULL_INDEX when slot is free
    int localIndex;

    FixedArray2<b2JointEdge> edges;

    int jointId;
    int islandId;
    int islandPrev;
    int islandNext;

    float drawScale;

    b2JointType type;

    // This is monotonically advanced when a body is allocated in this slot
    // Used to check for invalid b2JointId
    ushort generation;

    bool collideConnected;

}
