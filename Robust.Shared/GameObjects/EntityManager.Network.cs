using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using Robust.Shared.Map;
using Robust.Shared.Network;
using Robust.Shared.Timing;
using Robust.Shared.Utility;

namespace Robust.Shared.GameObjects;

public partial class EntityManager
{
    // TODO POOLING
    // Just add overrides that take in an existing collection.

    /// <summary>
    /// Inverse lookup for net entities.
    /// Regular lookup uses MetadataComponent.
    /// </summary>
    protected readonly Dictionary<NetEntity, (EntityUid, MetaDataComponent)> NetEntityLookup = new(EntityCapacity);

    private readonly Dictionary<PredictedSpawnKey, EntityUid> _predictedSpawnLookup = new();
    private readonly Dictionary<EntityUid, PredictedSpawnKey> _predictedSpawnKeys = new();
    private GameTick? _predictedSpawnTickOverride;
    private NetUserId? _predictedSpawnOwnerOverride;

    /// <summary>
    /// Clears an old inverse lookup for a particular entityuid.
    /// Do not call this unless you are sure of what you're doing.
    /// </summary>
    internal void ClearNetEntity(NetEntity netEntity)
    {
        NetEntityLookup.Remove(netEntity);
    }

    /// <summary>
    /// Set the inverse lookup for a particular entityuid.
    /// Do not call this unless you are sure of what you're doing.
    /// </summary>
    internal void SetNetEntity(EntityUid uid, NetEntity netEntity, MetaDataComponent component)
    {
        DebugTools.Assert(component.NetEntity == NetEntity.Invalid || _netMan.IsClient);
        DebugTools.Assert(!NetEntityLookup.ContainsKey(netEntity));
        NetEntityLookup[netEntity] = (uid, component);
        component.NetEntity = netEntity;
    }

    /// <inheritdoc />
    public virtual bool IsClientSide(EntityUid uid, MetaDataComponent? metadata = null)
    {
        return false;
    }

    #region NetEntity

    /// <inheritdoc />
    public bool TryParseNetEntity(string arg, [NotNullWhen(true)] out EntityUid? entity)
    {
        if (!NetEntity.TryParse(arg, out var netEntity) ||
            !TryGetEntity(netEntity, out entity))
        {
            entity = null;
            return false;
        }

        return true;
    }

    /// <inheritdoc />
    public bool TryGetEntity(NetEntity nEntity, [NotNullWhen(true)] out EntityUid? entity)
    {
        if (NetEntityLookup.TryGetValue(nEntity, out var went))
        {
            entity = went.Item1;
            return true;
        }

        entity = null;
        return false;
    }

    /// <inheritdoc />
    public bool TryGetEntityData(NetEntity nEntity, [NotNullWhen(true)] out EntityUid? entity, [NotNullWhen(true)] out MetaDataComponent? meta)
    {
        if (NetEntityLookup.TryGetValue(nEntity, out var went))
        {
            entity = went.Item1;
            meta = went.Item2;
            return true;
        }

        entity = null;
        meta = null;
        return false;
    }

    /// <inheritdoc />
    public bool TryGetEntity(NetEntity? nEntity, [NotNullWhen(true)] out EntityUid? entity)
    {
        if (nEntity == null)
        {
            entity = null;
            return false;
        }

        return TryGetEntity(nEntity.Value, out entity);
    }

    /// <inheritdoc />
    public bool TryGetEntity(NetEntityReference reference, [NotNullWhen(true)] out EntityUid? entity)
    {
        if (!reference.IsPredicted)
            return TryGetEntity(reference.NetEntity, out entity);

        if (TryGetPredictedEntity(reference, out var uid))
        {
            entity = uid;
            return true;
        }

        entity = null;
        return false;
    }

    /// <inheritdoc />
    public bool TryGetNetEntity(EntityUid uid, [NotNullWhen(true)] out NetEntity? netEntity, MetaDataComponent? metadata = null)
    {
        if (uid == EntityUid.Invalid)
        {
            netEntity = null;
            return false;
        }

        // TODO NetEntity figure out why this happens
        // I wanted this to logMissing but it seems to break a loootttt of dodgy stuff on content.
        if (MetaQuery.Resolve(uid, ref metadata, false))
        {
            netEntity = metadata.NetEntity;
            return true;
        }

        netEntity = NetEntity.Invalid;
        return false;
    }

    /// <inheritdoc />
    public bool TryGetNetEntity(EntityUid? uid, [NotNullWhen(true)] out NetEntity? netEntity, MetaDataComponent? metadata = null)
    {
        if (uid == null)
        {
            netEntity = null;
            return false;
        }

        return TryGetNetEntity(uid.Value, out netEntity, metadata);
    }

    /// <inheritdoc />
    public virtual EntityUid EnsureEntity<T>(NetEntity nEntity, EntityUid callerEntity)
    {
        // On server we don't want to ensure any reserved entities for later or flag for comp state handling
        // so this is just GetEntity. Client-side code overrides this method.
        return GetEntity(nEntity);
    }

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public EntityUid? EnsureEntity<T>(NetEntity? nEntity, EntityUid callerEntity)
    {
        if (nEntity == null)
            return null;

        return EnsureEntity<T>(nEntity.Value, callerEntity);
    }

    /// <inheritdoc />
    public EntityUid GetEntity(NetEntity nEntity)
    {
        if (nEntity == NetEntity.Invalid)
            return EntityUid.Invalid;

        if (!NetEntityLookup.TryGetValue(nEntity, out var tuple))
            return EntityUid.Invalid;

        return tuple.Item1;
    }

    /// <inheritdoc />
    public EntityUid GetEntity(NetEntityReference reference)
    {
        if (!reference.IsPredicted)
            return GetEntity(reference.NetEntity);

        return TryGetPredictedEntity(reference, out var uid)
            ? uid.Value
            : EntityUid.Invalid;
    }

    public (EntityUid, MetaDataComponent) GetEntityData(NetEntity nEntity)
    {
        return NetEntityLookup[nEntity];
    }

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public EntityUid? GetEntity(NetEntity? nEntity)
    {
        if (nEntity == null)
            return null;

        return GetEntity(nEntity.Value);
    }

    /// <inheritdoc />
    public NetEntity GetNetEntity(EntityUid uid, MetaDataComponent? metadata = null)
    {
        if (uid == EntityUid.Invalid)
            return NetEntity.Invalid;

        if (!MetaQuery.Resolve(uid, ref metadata))
            return NetEntity.Invalid;

        return metadata.NetEntity;
    }

    /// <inheritdoc />
    public NetEntityReference GetNetEntityReference(EntityUid uid, MetaDataComponent? metadata = null)
    {
        var netEntity = GetNetEntity(uid, metadata);

        if (!netEntity.IsClientSide())
            return new NetEntityReference(netEntity);

        if (TryGetComponent(uid, out PredictedSpawnComponent? predicted))
        {
            return new NetEntityReference(
                NetEntity.Invalid,
                predicted.SpawnTick,
                predicted.SpawnIndex,
                predicted.SpawnId,
                predicted.SpawnOwner);
        }

        return new NetEntityReference(netEntity);
    }

    /// <inheritdoc />
    public PredictedSpawnTickScope WithPredictedSpawnTick(GameTick tick, NetUserId? owner = null)
    {
        var previousTick = _predictedSpawnTickOverride;
        var previousOwner = _predictedSpawnOwnerOverride;
        _predictedSpawnTickOverride = tick;
        _predictedSpawnOwnerOverride = owner;
        return new PredictedSpawnTickScope(this, previousTick, previousOwner);
    }

    /// <inheritdoc />
    public bool TryGetPredictedSpawnContext(out GameTick tick, out NetUserId? owner)
    {
        if (_predictedSpawnTickOverride is not { } predictedTick)
        {
            tick = default;
            owner = null;
            return false;
        }

        tick = predictedTick;
        owner = _predictedSpawnOwnerOverride;
        return true;
    }

    /// <inheritdoc />
    public NetEntity? GetNetEntity(EntityUid? uid, MetaDataComponent? metadata = null)
    {
        if (uid == null)
            return null;

        return GetNetEntity(uid.Value, metadata);
    }

    internal void RegisterPredictedSpawn(Entity<MetaDataComponent?> ent, string? id)
    {
        if (!MetaQuery.Resolve(ent.Owner, ref ent.Comp))
            return;

        var tick = _predictedSpawnTickOverride ?? _gameTiming.CurTick;
        var key = new PredictedSpawnKey(tick, 0, id, _predictedSpawnOwnerOverride);
        while (_predictedSpawnLookup.ContainsKey(key))
            key = key with { Index = checked((ushort) (key.Index + 1)) };

        if (EnsureComponent<PredictedSpawnComponent>(ent.Owner) is { } predicted)
            SetPredictedSpawnReference(ent.Owner, predicted, key.Tick, key.Index, key.Id, key.Owner);
    }

    internal void SetPredictedSpawnReference(
        EntityUid uid,
        PredictedSpawnComponent predicted,
        GameTick tick,
        ushort index,
        string? id,
        NetUserId? owner,
        bool dirty = true)
    {
        UnregisterPredictedSpawn(uid);

        predicted.SpawnTick = tick;
        predicted.SpawnIndex = index;
        predicted.SpawnId = id;
        predicted.SpawnOwner = owner;
        var key = new PredictedSpawnKey(tick, index, id, owner);
        _predictedSpawnLookup[key] = uid;
        _predictedSpawnKeys[uid] = key;

        if (dirty)
            Dirty(uid, predicted);
    }

    internal void RestorePredictedSpawnTickScope(GameTick? tick, NetUserId? owner)
    {
        _predictedSpawnTickOverride = tick;
        _predictedSpawnOwnerOverride = owner;
    }

    internal void UnregisterPredictedSpawn(EntityUid uid)
    {
        if (!_predictedSpawnKeys.Remove(uid, out var key))
            return;

        if (_predictedSpawnLookup.TryGetValue(key, out var mapped) && mapped == uid)
            _predictedSpawnLookup.Remove(key);
    }

    #endregion

    private bool TryGetPredictedEntity(NetEntityReference reference, [NotNullWhen(true)] out EntityUid? entity)
    {
        if (_predictedSpawnLookup.TryGetValue(PredictedSpawnKey.FromReference(reference), out var uid) &&
            EntityExists(uid))
        {
            entity = uid;
            return true;
        }

        entity = null;
        return false;
    }

    private readonly record struct PredictedSpawnKey(
        GameTick Tick,
        ushort Index,
        string? Id,
        NetUserId? Owner)
    {
        public static PredictedSpawnKey FromReference(NetEntityReference reference)
        {
            return new PredictedSpawnKey(
                reference.PredictedSpawnTick,
                reference.PredictedSpawnIndex,
                reference.PredictedSpawnId,
                reference.PredictedSpawnOwner);
        }
    }

    #region NetCoordinates

    /// <inheritdoc />
    public NetCoordinates GetNetCoordinates(EntityCoordinates coordinates, MetaDataComponent? metadata = null)
    {
        return new NetCoordinates(GetNetEntity(coordinates.EntityId, metadata), coordinates.Position);
    }

    /// <inheritdoc />
    public NetCoordinates? GetNetCoordinates(EntityCoordinates? coordinates, MetaDataComponent? metadata = null)
    {
        if (coordinates == null)
            return null;

        return new NetCoordinates(GetNetEntity(coordinates.Value.EntityId, metadata), coordinates.Value.Position);
    }

    /// <inheritdoc />
    public EntityCoordinates GetCoordinates(NetCoordinates coordinates)
    {
        return new EntityCoordinates(GetEntity(coordinates.NetEntity), coordinates.Position);
    }

    /// <inheritdoc />
    public EntityCoordinates? GetCoordinates(NetCoordinates? coordinates)
    {
        if (coordinates == null)
            return null;

        return new EntityCoordinates(GetEntity(coordinates.Value.NetEntity), coordinates.Value.Position);
    }

    /// <inheritdoc />
    public virtual EntityCoordinates EnsureCoordinates<T>(NetCoordinates netCoordinates, EntityUid callerEntity)
    {
        // See EnsureEntity
        return GetCoordinates(netCoordinates);
    }

    /// <inheritdoc />
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public EntityCoordinates? EnsureCoordinates<T>(NetCoordinates? netCoordinates, EntityUid callerEntity)
    {
        if (netCoordinates == null)
            return null;

        return EnsureCoordinates<T>(netCoordinates.Value, callerEntity);
    }

    #endregion

    #region Collection helpers

    /// <inheritdoc />
    public HashSet<EntityUid> GetEntitySet(HashSet<NetEntity> netEntities)
    {
        var entities = new HashSet<EntityUid>(netEntities.Count);

        foreach (var netEntity in netEntities)
        {
            entities.Add(GetEntity(netEntity));
        }

        return entities;
    }

    /// <inheritdoc />
    public List<EntityUid> GetEntityList(List<NetEntity> netEntities)
    {
        var entities = new List<EntityUid>(netEntities.Count);

        foreach (var netEntity in netEntities)
        {
            entities.Add(GetEntity(netEntity));
        }

        return entities;
    }

    public Dictionary<EntityUid, T> GetEntityDictionary<T>(Dictionary<NetEntity, T> netEntities)
    {
        var entities = new Dictionary<EntityUid, T>(netEntities.Count);

        foreach (var pair in netEntities)
        {
            entities.Add(GetEntity(pair.Key), pair.Value);
        }

        return entities;
    }

    public Dictionary<T, EntityUid> GetEntityDictionary<T>(Dictionary<T, NetEntity> netEntities) where T : notnull
    {
        var entities = new Dictionary<T, EntityUid>(netEntities.Count);

        foreach (var pair in netEntities)
        {
            entities.Add(pair.Key, GetEntity(pair.Value));
        }

        return entities;
    }

    public Dictionary<T, EntityUid?> GetEntityDictionary<T>(Dictionary<T, NetEntity?> netEntities) where T : notnull
    {
        var entities = new Dictionary<T, EntityUid?>(netEntities.Count);

        foreach (var pair in netEntities)
        {
            entities.Add(pair.Key, GetEntity(pair.Value));
        }

        return entities;
    }

    public Dictionary<EntityUid, EntityUid> GetEntityDictionary(Dictionary<NetEntity, NetEntity> netEntities)
    {
        var entities = new Dictionary<EntityUid, EntityUid>(netEntities.Count);

        foreach (var pair in netEntities)
        {
            entities.Add(GetEntity(pair.Key), GetEntity(pair.Value));
        }

        return entities;
    }

    public Dictionary<EntityUid, EntityUid?> GetEntityDictionary(Dictionary<NetEntity, NetEntity?> netEntities)
    {
        var entities = new Dictionary<EntityUid, EntityUid?>(netEntities.Count);

        foreach (var pair in netEntities)
        {
            entities.Add(GetEntity(pair.Key), GetEntity(pair.Value));
        }

        return entities;
    }

    public HashSet<EntityUid> EnsureEntitySet<T>(HashSet<NetEntity> netEntities, EntityUid callerEntity)
    {
        var entities = new HashSet<EntityUid>(netEntities.Count);

        foreach (var netEntity in netEntities)
        {
            entities.Add(EnsureEntity<T>(netEntity, callerEntity));
        }

        return entities;
    }

    public void EnsureEntitySet<T>(HashSet<NetEntity> netEntities, EntityUid callerEntity, HashSet<EntityUid> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var netEntity in netEntities)
        {
            entities.Add(EnsureEntity<T>(netEntity, callerEntity));
        }
    }

    /// <inheritdoc />
    public List<EntityUid> EnsureEntityList<T>(List<NetEntity> netEntities, EntityUid callerEntity)
    {
        var entities = new List<EntityUid>(netEntities.Count);

        foreach (var netEntity in netEntities)
        {
            entities.Add(EnsureEntity<T>(netEntity, callerEntity));
        }

        return entities;
    }

    public void EnsureEntityList<T>(List<NetEntity> netEntities, EntityUid callerEntity, List<EntityUid> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var netEntity in netEntities)
        {
            entities.Add(EnsureEntity<T>(netEntity, callerEntity));
        }
    }

    public void EnsureEntityDictionary<TComp, TValue>(Dictionary<NetEntity, TValue> netEntities, EntityUid callerEntity,
        Dictionary<EntityUid, TValue> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(EnsureEntity<TComp>(pair.Key, callerEntity), pair.Value);
        }
    }

    public void EnsureEntityDictionaryNullableValue<TComp, TValue>(Dictionary<NetEntity, TValue?> netEntities, EntityUid callerEntity,
        Dictionary<EntityUid, TValue?> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(EnsureEntity<TComp>(pair.Key, callerEntity), pair.Value);
        }
    }

    public void EnsureEntityDictionary<TComp, TKey>(Dictionary<TKey, NetEntity> netEntities, EntityUid callerEntity,
        Dictionary<TKey, EntityUid> entities) where TKey : notnull
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(pair.Key, EnsureEntity<TComp>(pair.Value, callerEntity));
        }
    }

    public void EnsureEntityDictionary<TComp, TKey>(Dictionary<TKey, NetEntity?> netEntities, EntityUid callerEntity,
        Dictionary<TKey, EntityUid?> entities) where TKey : notnull
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(pair.Key, EnsureEntity<TComp>(pair.Value, callerEntity));
        }
    }

    public void EnsureEntityDictionary<TComp>(Dictionary<NetEntity, NetEntity> netEntities, EntityUid callerEntity,
        Dictionary<EntityUid, EntityUid> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(EnsureEntity<TComp>(pair.Key, callerEntity), EnsureEntity<TComp>(pair.Value, callerEntity));
        }
    }

    public void EnsureEntityDictionary<TComp>(Dictionary<NetEntity, NetEntity?> netEntities, EntityUid callerEntity,
        Dictionary<EntityUid, EntityUid?> entities)
    {
        entities.Clear();
        entities.EnsureCapacity(netEntities.Count);
        foreach (var pair in netEntities)
        {
            entities.TryAdd(EnsureEntity<TComp>(pair.Key, callerEntity), EnsureEntity<TComp>(pair.Value, callerEntity));
        }
    }

    /// <inheritdoc />
    public List<EntityUid> GetEntityList(ICollection<NetEntity> netEntities)
    {
        var entities = new List<EntityUid>(netEntities.Count);
        foreach (var netEntity in netEntities)
        {
            entities.Add(GetEntity(netEntity));
        }

        return entities;
    }

    /// <inheritdoc />
    public List<EntityUid?> GetEntityList(List<NetEntity?> netEntities)
    {
        var entities = new List<EntityUid?>(netEntities.Count);

        foreach (var netEntity in netEntities)
        {
            entities.Add(GetEntity(netEntity));
        }

        return entities;
    }

    /// <inheritdoc />
    public EntityUid[] GetEntityArray(NetEntity[] netEntities)
    {
        var entities = new EntityUid[netEntities.Length];

        for (var i = 0; i < netEntities.Length; i++)
        {
            entities[i] = GetEntity(netEntities[i]);
        }

        return entities;
    }

    /// <inheritdoc />
    public EntityUid?[] GetEntityArray(NetEntity?[] netEntities)
    {
        var entities = new EntityUid?[netEntities.Length];

        for (var i = 0; i < netEntities.Length; i++)
        {
            entities[i] = GetEntity(netEntities[i]);
        }

        return entities;
    }

    /// <inheritdoc />
    public HashSet<NetEntity> GetNetEntitySet(HashSet<EntityUid> entities)
    {
        var newSet = new HashSet<NetEntity>(entities.Count);

        foreach (var ent in entities)
        {
            MetaQuery.TryGetComponent(ent, out var metadata);
            newSet.Add(GetNetEntity(ent, metadata));
        }

        return newSet;
    }

    /// <inheritdoc />
    public List<NetEntity> GetNetEntityList(List<EntityUid> entities)
    {
        var netEntities = new List<NetEntity>(entities.Count);

        foreach (var netEntity in entities)
        {
            netEntities.Add(GetNetEntity(netEntity));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public List<NetEntity> GetNetEntityList(IReadOnlyList<EntityUid> entities)
    {
        var netEntities = new List<NetEntity>(entities.Count);

        foreach (var netEntity in entities)
        {
            netEntities.Add(GetNetEntity(netEntity));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public List<NetEntity> GetNetEntityList(ICollection<EntityUid> entities)
    {
        var netEntities = new List<NetEntity>(entities.Count);

        foreach (var netEntity in entities)
        {
            netEntities.Add(GetNetEntity(netEntity));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public List<NetEntity?> GetNetEntityList(List<EntityUid?> entities)
    {
        var netEntities = new List<NetEntity?>(entities.Count);

        foreach (var netEntity in entities)
        {
            netEntities.Add(GetNetEntity(netEntity));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public NetEntity[] GetNetEntityArray(EntityUid[] entities)
    {
        var netEntities = new NetEntity[entities.Length];

        for (var i = 0; i < entities.Length; i++)
        {
            netEntities[i] = GetNetEntity(entities[i]);
        }

        return netEntities;
    }

    /// <inheritdoc />
    public NetEntity?[] GetNetEntityArray(EntityUid?[] entities)
    {
        var netEntities = new NetEntity?[entities.Length];

        for (var i = 0; i < entities.Length; i++)
        {
            netEntities[i] = GetNetEntity(entities[i]);
        }

        return netEntities;
    }

    /// <inheritdoc />
    public Dictionary<NetEntity, T> GetNetEntityDictionary<T>(Dictionary<EntityUid, T> entities)
    {
        var netEntities = new Dictionary<NetEntity, T>(entities.Count);

        foreach (var pair in entities)
        {
            netEntities.Add(GetNetEntity(pair.Key), pair.Value);
        }

        return netEntities;
    }

    /// <inheritdoc />
    public Dictionary<T, NetEntity> GetNetEntityDictionary<T>(Dictionary<T, EntityUid> entities) where T : notnull
    {
        var netEntities = new Dictionary<T, NetEntity>(entities.Count);

        foreach (var pair in entities)
        {
            netEntities.Add(pair.Key, GetNetEntity(pair.Value));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public Dictionary<T, NetEntity?> GetNetEntityDictionary<T>(Dictionary<T, EntityUid?> entities) where T : notnull
    {
        var netEntities = new Dictionary<T, NetEntity?>(entities.Count);

        foreach (var pair in entities)
        {
            netEntities.Add(pair.Key, GetNetEntity(pair.Value));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public Dictionary<NetEntity, NetEntity> GetNetEntityDictionary(Dictionary<EntityUid, EntityUid> entities)
    {
        var netEntities = new Dictionary<NetEntity, NetEntity>(entities.Count);

        foreach (var pair in entities)
        {
            netEntities.Add(GetNetEntity(pair.Key), GetNetEntity(pair.Value));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public Dictionary<NetEntity, NetEntity?> GetNetEntityDictionary(Dictionary<EntityUid, EntityUid?> entities)
    {
        var netEntities = new Dictionary<NetEntity, NetEntity?>(entities.Count);

        foreach (var pair in entities)
        {
            netEntities.Add(GetNetEntity(pair.Key), GetNetEntity(pair.Value));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public HashSet<EntityCoordinates> GetEntitySet(HashSet<NetCoordinates> netEntities)
    {
        var entities = new HashSet<EntityCoordinates>(netEntities.Count);

        foreach (var netCoordinates in netEntities)
        {
            entities.Add(GetCoordinates(netCoordinates));
        }

        return entities;
    }

    /// <inheritdoc />
    public List<EntityCoordinates> GetEntityList(List<NetCoordinates> netEntities)
    {
        var entities = new List<EntityCoordinates>(netEntities.Count);

        foreach (var netCoordinates in netEntities)
        {
            entities.Add(GetCoordinates(netCoordinates));
        }

        return entities;
    }

    /// <inheritdoc />
    public List<EntityCoordinates> GetEntityList(ICollection<NetCoordinates> netEntities)
    {
        var entities = new List<EntityCoordinates>(netEntities.Count);

        foreach (var netCoordinates in netEntities)
        {
            entities.Add(GetCoordinates(netCoordinates));
        }

        return entities;
    }

    /// <inheritdoc />
    public List<EntityCoordinates?> GetEntityList(List<NetCoordinates?> netEntities)
    {
        var entities = new List<EntityCoordinates?>(netEntities.Count);

        foreach (var netCoordinates in netEntities)
        {
            entities.Add(GetCoordinates(netCoordinates));
        }

        return entities;
    }

    /// <inheritdoc />
    public EntityCoordinates[] GetEntityArray(NetCoordinates[] netEntities)
    {
        var entities = new EntityCoordinates[netEntities.Length];

        for (var i = 0; i < netEntities.Length; i++)
        {
            entities[i] = GetCoordinates(netEntities[i]);
        }

        return entities;
    }

    /// <inheritdoc />
    public EntityCoordinates?[] GetEntityArray(NetCoordinates?[] netEntities)
    {
        var entities = new EntityCoordinates?[netEntities.Length];

        for (var i = 0; i < netEntities.Length; i++)
        {
            entities[i] = GetCoordinates(netEntities[i]);
        }

        return entities;
    }

    /// <inheritdoc />
    public HashSet<NetCoordinates> GetNetCoordinatesSet(HashSet<EntityCoordinates> entities)
    {
        var newSet = new HashSet<NetCoordinates>(entities.Count);

        foreach (var coordinates in entities)
        {
            newSet.Add(GetNetCoordinates(coordinates));
        }

        return newSet;
    }

    /// <inheritdoc />
    public List<NetCoordinates> GetNetCoordinatesList(List<EntityCoordinates> entities)
    {
        var netEntities = new List<NetCoordinates>(entities.Count);

        foreach (var netCoordinates in entities)
        {
            netEntities.Add(GetNetCoordinates(netCoordinates));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public List<NetCoordinates> GetNetCoordinatesList(ICollection<EntityCoordinates> entities)
    {
        var netEntities = new List<NetCoordinates>(entities.Count);

        foreach (var netCoordinates in entities)
        {
            netEntities.Add(GetNetCoordinates(netCoordinates));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public List<NetCoordinates?> GetNetCoordinatesList(List<EntityCoordinates?> entities)
    {
        var netEntities = new List<NetCoordinates?>(entities.Count);

        foreach (var netCoordinates in entities)
        {
            netEntities.Add(GetNetCoordinates(netCoordinates));
        }

        return netEntities;
    }

    /// <inheritdoc />
    public NetCoordinates[] GetNetCoordinatesArray(EntityCoordinates[] entities)
    {
        var netEntities = new NetCoordinates[entities.Length];

        for (var i = 0; i < entities.Length; i++)
        {
            netEntities[i] = GetNetCoordinates(entities[i]);
        }

        return netEntities;
    }

    /// <inheritdoc />
    public NetCoordinates?[] GetNetCoordinatesArray(EntityCoordinates?[] entities)
    {
        var netEntities = new NetCoordinates?[entities.Length];

        for (var i = 0; i < entities.Length; i++)
        {
            netEntities[i] = GetNetCoordinates(entities[i]);
        }

        return netEntities;
    }

    #endregion
}
