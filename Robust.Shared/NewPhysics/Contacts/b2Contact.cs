using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
public sealed class b2Contact
{
    // index of simulation set stored in b2World
    // B2_NULL_INDEX when slot is free
    public SetType setIndex;

    // index into the constraint graph color array
    // B2_NULL_INDEX for non-touching or sleeping contacts
    // B2_NULL_INDEX when slot is free
    internal int colorIndex;

    // contact index within set or graph color
    // B2_NULL_INDEX when slot is free
    internal int localIndex;

    internal int shapeIdA;
    internal int shapeIdB;
    int contactId;

    // A contact only belongs to an island if touching, otherwise B2_NULL_INDEX.
    FixedArray2<b2ContactEdge> edges;
    int islandPrev;
    int islandNext;
    internal int islandId;

    internal ContactFlags flags;
}
