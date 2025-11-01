using System;

namespace Robust.Shared.NewPhysics;

[Flags]
public enum ContactFlags : byte
{
    // Set when the solid shapes are touching.
    ContactTouchingFlag = 0x00000001,

    // Contact has a hit event
    ContactHitEventFlag = 0x00000002,

    // This contact wants contact events
    ContactEnableContactEvents = 0x00000004,
}
