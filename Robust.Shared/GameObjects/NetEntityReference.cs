using System;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Network reference to an entity that may not have had its authoritative
/// <see cref="NetEntity"/> replicated to the client yet.
/// </summary>
[Serializable, NetSerializable]
public readonly record struct NetEntityReference(
    NetEntity NetEntity,
    GameTick PredictedSpawnTick,
    ushort PredictedSpawnIndex,
    string? PredictedSpawnId,
    NetUserId? PredictedSpawnOwner = null)
{
    public static readonly NetEntityReference Invalid = new(NetEntity.Invalid, GameTick.Zero, 0, null);

    public NetEntityReference(NetEntity netEntity) : this(netEntity, GameTick.Zero, 0, null)
    {
    }

    public bool IsValid => NetEntity.IsValid() || IsPredicted;

    public bool IsPredicted => !NetEntity.IsValid() && PredictedSpawnTick != GameTick.Zero;

    public static implicit operator NetEntityReference(NetEntity netEntity)
    {
        return new NetEntityReference(netEntity);
    }

    public static explicit operator NetEntity(NetEntityReference reference)
    {
        return reference.NetEntity;
    }

    public override string ToString()
    {
        if (!IsPredicted)
            return NetEntity.ToString();

        var id = PredictedSpawnId == null ? string.Empty : $", id={PredictedSpawnId}";
        var owner = PredictedSpawnOwner == null ? string.Empty : $", owner={PredictedSpawnOwner}";
        return $"predicted(tick={PredictedSpawnTick}, index={PredictedSpawnIndex}{id}{owner})";
    }
}
