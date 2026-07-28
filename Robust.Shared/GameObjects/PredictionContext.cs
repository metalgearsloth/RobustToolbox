using System;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Identifies the input prediction pass responsible for deferred work.
/// </summary>
/// <param name="Tick">The originating input tick.</param>
/// <param name="Owner">The user that originated the input, if any.</param>
[Serializable, NetSerializable]
public readonly record struct PredictionContext(GameTick Tick, NetUserId? Owner);
