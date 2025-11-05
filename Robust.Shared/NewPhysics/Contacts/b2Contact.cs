using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

// Cold contact data. Used as a persistent handle and for persistent island
// connectivity.
public sealed class b2Contact
{
    // index of simulation set stored in b2World
    // PhysicsConstants.NullIndex when slot is free
    public int setIndex;

    // index into the constraint graph color array
    // PhysicsConstants.NullIndex for non-touching or sleeping contacts
    // PhysicsConstants.NullIndex when slot is free
    internal int colorIndex;

    // contact index within set or graph color
    // PhysicsConstants.NullIndex when slot is free
    internal int localIndex;

    internal int shapeIdA;
    internal int shapeIdB;
    internal int contactId;

    // A contact only belongs to an island if touching, otherwise PhysicsConstants.NullIndex.
    internal FixedArray2<b2ContactEdge> edges;
    internal int islandPrev;
    internal int islandNext;
    internal int islandId;

    internal ContactFlags flags;
}
