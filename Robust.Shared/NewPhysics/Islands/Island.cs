namespace Robust.Shared.NewPhysics.Islands;

// Deterministic solver
//
// Collide all awake contacts
// Use bit array to emit start/stop touching events in defined order, per thread. Try using contact index, assuming contacts are
// created in a deterministic order. bit-wise OR together bit arrays and issue changes:
// - start touching: merge islands - temporary linked list - mark root island dirty - wake all - largest island is root
// - stop touching: increment constraintRemoveCount

// Persistent island for awake bodies, joints, and contacts
// https://en.wikipedia.org/wiki/Component_(graph_theory)
// https://en.wikipedia.org/wiki/Dynamic_connectivity
// map from int to solver set and index
public sealed class Island
{
    // index of solver set stored in b2World
    // may be B2_NULL_INDEX
    int setIndex;

    // island index within set
    // may be B2_NULL_INDEX
    int localIndex;

    int islandId;

    int headBody;
    int tailBody;
    int bodyCount;

    int headContact;
    int tailContact;
    int contactCount;

    int headJoint;
    int tailJoint;
    int jointCount;

    // Keeps track of how many contacts have been removed from this island.
    // This is used to determine if an island is a candidate for splitting.
    int constraintRemoveCount;
}
