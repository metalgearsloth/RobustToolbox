using System;
using Robust.Shared.Input;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Stable handle for an entity spawned by a predicted input command.
/// The reference is scoped to the client session that produced the input.
/// </summary>
[Serializable, NetSerializable, CopyByRef]
public readonly struct PredictedEntityReference : IEquatable<PredictedEntityReference>
{
    public readonly GameTick Tick;
    public readonly ushort SubTick;
    public readonly KeyFunctionId InputFunctionId;
    public readonly ushort SpawnIndex;

    public PredictedEntityReference(GameTick tick, ushort subTick, KeyFunctionId inputFunctionId, ushort spawnIndex)
    {
        Tick = tick;
        SubTick = subTick;
        InputFunctionId = inputFunctionId;
        SpawnIndex = spawnIndex;
    }

    public bool Equals(PredictedEntityReference other)
    {
        return Tick == other.Tick
               && SubTick == other.SubTick
               && InputFunctionId == other.InputFunctionId
               && SpawnIndex == other.SpawnIndex;
    }

    public override bool Equals(object? obj)
    {
        return obj is PredictedEntityReference other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Tick, SubTick, InputFunctionId, SpawnIndex);
    }

    public static bool operator ==(PredictedEntityReference left, PredictedEntityReference right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(PredictedEntityReference left, PredictedEntityReference right)
    {
        return !left.Equals(right);
    }

    public override string ToString()
    {
        return $"{Tick}:{SubTick}:{InputFunctionId}:{SpawnIndex}";
    }
}

/// <summary>
/// A network entity reference that may point at a normal server entity or an entity
/// spawned by the receiver's own predicted input stream.
/// </summary>
[Serializable, NetSerializable, CopyByRef]
public readonly struct EntityNetReference : IEquatable<EntityNetReference>
{
    public readonly NetEntity NetEntity;
    public readonly PredictedEntityReference Predicted;
    public readonly bool IsPredicted;

    public static readonly EntityNetReference Invalid = new(NetEntity.Invalid);

    public EntityNetReference(NetEntity netEntity)
    {
        NetEntity = netEntity;
        Predicted = default;
        IsPredicted = false;
    }

    public EntityNetReference(PredictedEntityReference predicted)
    {
        NetEntity = NetEntity.Invalid;
        Predicted = predicted;
        IsPredicted = true;
    }

    public bool Equals(EntityNetReference other)
    {
        return IsPredicted == other.IsPredicted
               && NetEntity == other.NetEntity
               && Predicted == other.Predicted;
    }

    public override bool Equals(object? obj)
    {
        return obj is EntityNetReference other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(NetEntity, Predicted, IsPredicted);
    }

    public static bool operator ==(EntityNetReference left, EntityNetReference right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(EntityNetReference left, EntityNetReference right)
    {
        return !left.Equals(right);
    }

    public override string ToString()
    {
        return IsPredicted ? $"p:{Predicted}" : NetEntity.ToString();
    }
}
