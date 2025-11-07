using Robust.Shared.GameObjects;

namespace Robust.Shared.NewPhysics;

/// Body id references a body instance. This should be treated as an opaque handle.
internal record struct BodyId
{
    internal int index1;

    // Rather than use generation we just check the uid in the slot
    internal EntityUid Uid;
}
