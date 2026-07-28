using System;
using Robust.Shared.GameStates;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Indicates the attached entity was spawn predicted and should be reconciled when the server states comes in.
/// </summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class PredictedSpawnComponent : Component
{
    public GameTick SpawnTick;

    public ushort SpawnIndex;

    public string? SpawnId;

    public NetUserId? SpawnOwner;
}

[Serializable, NetSerializable]
public sealed class PredictedSpawnComponentState(
    GameTick spawnTick,
    ushort spawnIndex,
    string? spawnId,
    NetUserId? spawnOwner) : ComponentState
{
    public readonly GameTick SpawnTick = spawnTick;
    public readonly ushort SpawnIndex = spawnIndex;
    public readonly string? SpawnId = spawnId;
    public readonly NetUserId? SpawnOwner = spawnOwner;
}
