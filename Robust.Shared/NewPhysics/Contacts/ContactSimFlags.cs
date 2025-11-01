using System;

namespace Robust.Shared.NewPhysics;

// Shifted to be distinct from b2ContactFlags
[Flags]
internal enum ContactSimFlags
{
    // Set when the shapes are touching
    SimTouchingFlag = 0x00010000,

    // This contact no longer has overlapping AABBs
    SimDisjoint = 0x00020000,

    // This contact started touching
    SimStartedTouching = 0x00040000,

    // This contact stopped touching
    SimStoppedTouching = 0x00080000,

    // This contact has a hit event
    SimEnableHitEvent = 0x00100000,

    // This contact wants pre-solve events
    SimEnablePreSolveEvents = 0x00200000,
};
