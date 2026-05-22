using System;
using System.Collections.Generic;
using Robust.Shared.Input;
using Robust.Shared.Player;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

public partial class EntityManager
{
    private readonly Dictionary<PredictedEntityReference, EntityUid> _localPredictedEntities = new();
    private readonly Dictionary<PredictedSpawnOwnerKey, EntityUid> _ownedPredictedEntities = new();
    private PredictedSpawnContext? _predictedSpawnContext;

    public IDisposable PushPredictedSpawnContext(
        ICommonSession? session,
        GameTick tick,
        ushort subTick,
        KeyFunctionId inputFunctionId)
    {
        var old = _predictedSpawnContext;
        _predictedSpawnContext = new PredictedSpawnContext(session, tick, subTick, inputFunctionId, 0);
        return new PredictedSpawnContextPop(this, old);
    }

    public bool TryGetPredictedEntityReference(EntityUid uid, out PredictedEntityReference reference)
    {
        if (TryGetComponent(uid, out PredictedSpawnComponent? predicted) && predicted.HasReference)
        {
            reference = predicted.Reference;
            return true;
        }

        reference = default;
        return false;
    }

    public EntityNetReference GetNetEntityReference(EntityUid uid, MetaDataComponent? metadata = null)
    {
        return TryGetPredictedEntityReference(uid, out var predicted)
            ? new EntityNetReference(predicted)
            : new EntityNetReference(GetNetEntity(uid, metadata));
    }

    public EntityNetReference? GetNetEntityReference(EntityUid? uid, MetaDataComponent? metadata = null)
    {
        return uid == null ? null : GetNetEntityReference(uid.Value, metadata);
    }

    public bool TryGetEntity(EntityNetReference reference, ICommonSession? session, out EntityUid entity)
    {
        if (!reference.IsPredicted)
        {
            if (TryGetEntity(reference.NetEntity, out var netEntity))
            {
                entity = netEntity.Value;
                return true;
            }

            entity = EntityUid.Invalid;
            return false;
        }

        if (session != null &&
            _ownedPredictedEntities.TryGetValue(new PredictedSpawnOwnerKey(session, reference.Predicted), out entity))
        {
            return true;
        }

        if (_netMan.IsClient && _localPredictedEntities.TryGetValue(reference.Predicted, out entity))
            return true;

        entity = EntityUid.Invalid;
        return false;
    }

    public EntityUid GetEntity(EntityNetReference reference, ICommonSession? session = null)
    {
        return TryGetEntity(reference, session, out var entity)
            ? entity
            : EntityUid.Invalid;
    }

    protected void RegisterPredictedSpawn(EntityUid uid, PredictedSpawnComponent? comp = null)
    {
        if (_predictedSpawnContext is not { } context)
            return;

        var reference = new PredictedEntityReference(
            context.Tick,
            context.SubTick,
            context.InputFunctionId,
            context.SpawnIndex);

        context.SpawnIndex++;
        _predictedSpawnContext = context;

        if (comp != null || TryGetComponent(uid, out comp))
        {
            comp.Reference = reference;
            comp.HasReference = true;
        }

        _localPredictedEntities[reference] = uid;

        if (context.Session != null)
            _ownedPredictedEntities[new PredictedSpawnOwnerKey(context.Session, reference)] = uid;
    }

    private readonly record struct PredictedSpawnOwnerKey(ICommonSession Session, PredictedEntityReference Reference);

    private struct PredictedSpawnContext
    {
        public readonly ICommonSession? Session;
        public readonly GameTick Tick;
        public readonly ushort SubTick;
        public readonly KeyFunctionId InputFunctionId;
        public ushort SpawnIndex;

        public PredictedSpawnContext(
            ICommonSession? session,
            GameTick tick,
            ushort subTick,
            KeyFunctionId inputFunctionId,
            ushort spawnIndex)
        {
            Session = session;
            Tick = tick;
            SubTick = subTick;
            InputFunctionId = inputFunctionId;
            SpawnIndex = spawnIndex;
        }
    }

    private sealed class PredictedSpawnContextPop : IDisposable
    {
        private readonly EntityManager _manager;
        private readonly PredictedSpawnContext? _old;
        private bool _disposed;

        public PredictedSpawnContextPop(EntityManager manager, PredictedSpawnContext? old)
        {
            _manager = manager;
            _old = old;
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            _manager._predictedSpawnContext = _old;
            _disposed = true;
        }
    }
}
