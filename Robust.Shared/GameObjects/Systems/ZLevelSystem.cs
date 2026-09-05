using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Numerics;
using Robust.Shared.EntitySerialization;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Map.Events;
using Robust.Shared.Maths;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Shared queries and mutation helpers for ordered z-map networks.
/// </summary>
public sealed partial class ZLevelSystem : EntitySystem
{
    private const string ZLevelMapNetworkPrototype = "ZLevelMapNetwork";

    [Dependency] private readonly SharedMapSystem _mapSystem = default!;
    [Dependency] private readonly SharedTransformSystem _transform = default!;
    [Dependency] private readonly SharedPvsOverrideSystem _pvsOverride = default!;
    [Dependency] private readonly ITileDefinitionManager _tileDefinitions = default!;

    [Dependency] private EntityQuery<MapComponent> _mapQuery = default!;
    [Dependency] private EntityQuery<ZLevelMapComponent> _zMapQuery = default!;
    [Dependency] private EntityQuery<ZLevelMapNetworkComponent> _networkQuery = default!;

    private List<Entity<MapGridComponent>> _renderOccluderGrids = new();

    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<ZLevelMapComponent, ComponentShutdown>(OnZMapShutdown);
        SubscribeLocalEvent<ZLevelMapNetworkComponent, ComponentStartup>(OnNetworkStartup);
        SubscribeLocalEvent<ZLevelMapNetworkComponent, ComponentShutdown>(OnNetworkShutdown);
        SubscribeLocalEvent<BeforeSerializationEvent>(OnBeforeSerialization);
    }

    public Entity<ZLevelMapNetworkComponent> CreateMapNetwork()
    {
        var uid = Spawn(ZLevelMapNetworkPrototype);
        return (uid, Comp<ZLevelMapNetworkComponent>(uid));
    }

    /// <summary>
    /// Changes the replicated visual projection without changing any authored or physics coordinates.
    /// </summary>
    public void SetProjectionOffset(Entity<ZLevelMapNetworkComponent?> network, Vector2 offset)
    {
        if (!Resolve(network, ref network.Comp) || !float.IsFinite(offset.X) || !float.IsFinite(offset.Y))
            return;

        network.Comp.ProjectionOffset = offset;
        DirtyField(network.Owner, network.Comp, nameof(ZLevelMapNetworkComponent.ProjectionOffset));
    }

    /// <summary>
    /// Sets the replicated renderer and PVS visibility window for this network.
    /// </summary>
    public void SetVisibleLevels(Entity<ZLevelMapNetworkComponent?> network, int below, int above)
    {
        if (!Resolve(network, ref network.Comp))
            return;

        network.Comp.VisibleLevelsBelow = Math.Max(0, below);
        network.Comp.VisibleLevelsAbove = Math.Max(0, above);
        DirtyFields(
            network.Owner,
            network.Comp,
            null,
            nameof(ZLevelMapNetworkComponent.VisibleLevelsBelow),
            nameof(ZLevelMapNetworkComponent.VisibleLevelsAbove));
    }

    /// <summary>
    /// Sets replicated lower-level rendering effects. A null shader uses the engine default.
    /// </summary>
    public void SetLowerLevelEffects(
        Entity<ZLevelMapNetworkComponent?> network,
        bool enabled,
        string? shader,
        float blurRadius,
        float darkenStrength,
        Color tint,
        bool stopAtOpaque = true)
    {
        if (!Resolve(network, ref network.Comp))
            return;

        network.Comp.LowerPostShaderEnabled = enabled;
        network.Comp.LowerPostShader = string.IsNullOrWhiteSpace(shader) ? null : shader;
        network.Comp.LowerBlurRadius = Math.Max(0f, blurRadius);
        network.Comp.LowerDarkenStrength = Math.Clamp(darkenStrength, 0f, 1f);
        network.Comp.LowerTint = tint;
        network.Comp.StopAtOpaque = stopAtOpaque;
        DirtyFields(
            network.Owner,
            network.Comp,
            null,
            nameof(ZLevelMapNetworkComponent.LowerPostShaderEnabled),
            nameof(ZLevelMapNetworkComponent.LowerPostShader),
            nameof(ZLevelMapNetworkComponent.LowerBlurRadius),
            nameof(ZLevelMapNetworkComponent.LowerDarkenStrength),
            nameof(ZLevelMapNetworkComponent.LowerTint),
            nameof(ZLevelMapNetworkComponent.StopAtOpaque));
    }

    /// <summary>
    /// Creates a network from maps ordered from lowest to highest.
    /// </summary>
    public bool TryCreateMapNetwork(
        IReadOnlyList<EntityUid> maps,
        [NotNullWhen(true)] out Entity<ZLevelMapNetworkComponent>? network)
    {
        network = null;
        if (maps.Count == 0 || maps.Distinct().Count() != maps.Count)
            return false;

        var created = CreateMapNetwork();
        var depths = new Dictionary<EntityUid, int>(maps.Count);
        for (var i = 0; i < maps.Count; i++)
            depths.Add(maps[i], i);

        if (!TryAddMaps(created, depths))
        {
            PredictedQueueDel(created);
            return false;
        }

        network = created;
        return true;
    }

    /// <summary>
    /// Adds maps to an empty network. The supplied depths establish their ordering.
    /// </summary>
    public bool TryAddMaps(Entity<ZLevelMapNetworkComponent> network, IReadOnlyDictionary<EntityUid, int> maps)
    {
        if (maps.Count == 0 || maps.Values.Distinct().Count() != maps.Count)
            return false;

        foreach (var map in maps.Keys)
        {
            if (!_mapQuery.HasComp(map) || _zMapQuery.HasComp(map))
                return false;
        }

        var sorted = maps.OrderBy(pair => pair.Value).Select(pair => pair.Key).ToList();
        if (network.Comp.SortedZLevelsInternal.Count == 0)
        {
            SetNetworkLevels(network, sorted);
            return true;
        }

        return TryInsertMaps(network, network.Comp.SortedZLevelsInternal.Count, sorted);
    }

    public bool TryInsertMap(Entity<ZLevelMapNetworkComponent> network, int index, EntityUid map)
        => TryInsertMaps(network, index, [map]);

    public bool TryInsertMaps(Entity<ZLevelMapNetworkComponent> network, int index, IReadOnlyList<EntityUid> maps)
    {
        if (maps.Count == 0 ||
            index < 0 ||
            index > network.Comp.SortedZLevelsInternal.Count ||
            maps.Distinct().Count() != maps.Count ||
            !HasLinearDepths(network))
        {
            return false;
        }

        foreach (var map in maps)
        {
            if (!_mapQuery.HasComp(map) || _zMapQuery.HasComp(map))
                return false;
        }

        var levels = new List<EntityUid>(network.Comp.SortedZLevelsInternal);
        levels.InsertRange(index, maps);
        SetNetworkLevels(network, levels);
        return true;
    }

    [Pure]
    public bool TryGetMapNetwork(EntityUid map, [NotNullWhen(true)] out Entity<ZLevelMapNetworkComponent>? network)
    {
        network = null;
        if (!_zMapQuery.TryComp(map, out var zMap) || !_networkQuery.TryComp(zMap.Network, out var comp))
            return false;

        network = (zMap.Network, comp);
        return true;
    }

    [Pure]
    public bool TryGetMapData(
        EntityUid map,
        [NotNullWhen(true)] out ZLevelMapComponent? zMap,
        [NotNullWhen(true)] out ZLevelMapNetworkComponent? network)
    {
        network = null;
        return _zMapQuery.TryComp(map, out zMap) && _networkQuery.TryComp(zMap.Network, out network);
    }

    [Pure]
    public bool TryGetMapDepthOffset(EntityUid fromMap, EntityUid toMap, out int offset)
    {
        offset = 0;
        if (!_zMapQuery.TryComp(fromMap, out var from) ||
            !_zMapQuery.TryComp(toMap, out var to) ||
            from.Network != to.Network)
        {
            return false;
        }

        offset = to.Depth - from.Depth;
        return true;
    }

    public void CollectMapNetworks(List<Entity<ZLevelMapNetworkComponent>> networks)
    {
        networks.Clear();
        var query = AllEntityQuery<ZLevelMapNetworkComponent>();
        while (query.MoveNext(out var uid, out var comp))
            networks.Add((uid, comp));
    }

    [Pure]
    public IReadOnlyList<EntityUid> GetNetworkMaps(Entity<ZLevelMapNetworkComponent> network)
        => network.Comp.SortedZLevels;

    [Pure]
    public bool TryGetMapAtDepth(EntityUid networkUid, int depth, [NotNullWhen(true)] out EntityUid? map)
    {
        map = null;
        if (!_networkQuery.TryComp(networkUid, out var network) ||
            depth < 0 ||
            depth >= network.SortedZLevelsInternal.Count)
        {
            return false;
        }

        map = network.SortedZLevelsInternal[depth];
        return true;
    }

    [Pure]
    public bool TryMapOffset(
        Entity<ZLevelMapComponent?> map,
        int offset,
        [NotNullWhen(true)] out Entity<ZLevelMapComponent>? other)
    {
        other = null;
        if (!Resolve(map, ref map.Comp, false) ||
            !_networkQuery.TryComp(map.Comp.Network, out _) ||
            !TryGetMapAtDepth(map.Comp.Network, map.Comp.Depth + offset, out var otherUid) ||
            !_zMapQuery.TryComp(otherUid.Value, out var otherComp))
        {
            return false;
        }

        other = (otherUid.Value, otherComp);
        return true;
    }

    [Pure]
    public bool TryGetMapOffset(EntityUid map, int offset, [NotNullWhen(true)] out EntityUid? other)
    {
        other = null;
        if (!_zMapQuery.TryComp(map, out var zMap) || !TryMapOffset((map, zMap), offset, out var result))
            return false;

        other = result.Value.Owner;
        return true;
    }

    [Pure]
    public bool TryGetMapAbove(EntityUid map, [NotNullWhen(true)] out EntityUid? above)
        => TryGetMapOffset(map, 1, out above);

    [Pure]
    public bool TryGetMapBelow(EntityUid map, [NotNullWhen(true)] out EntityUid? below)
        => TryGetMapOffset(map, -1, out below);

    /// <summary>
    /// Moves an entity to another map while preserving canonical world XY and rotation.
    /// </summary>
    public bool TryMoveEntityToMapOffset(EntityUid entity, int offset, [NotNullWhen(true)] out EntityUid? targetMap)
    {
        targetMap = null;
        var xform = Transform(entity);
        if (offset == 0 ||
            xform.MapUid is not { } currentMap ||
            !TryGetMapOffset(currentMap, offset, out targetMap) ||
            targetMap is not { } target ||
            !_mapQuery.TryComp(target, out var targetMapComp))
        {
            targetMap = null;
            return false;
        }

        var (position, rotation) = _transform.GetWorldPositionRotation(xform);
        _transform.SetMapCoordinates((entity, xform), new MapCoordinates(position, targetMapComp.MapId), rotation);
        return true;
    }

    public bool TryMoveEntityToMapOffset(EntityUid entity, int offset)
        => TryMoveEntityToMapOffset(entity, offset, out _);

    public void CollectMapOffsets(EntityUid map, int below, int above, List<EntityUid> maps)
    {
        maps.Clear();
        if (!TryGetMapData(map, out var zMap, out var network))
            return;

        for (var offset = -Math.Max(0, below); offset <= Math.Max(0, above); offset++)
        {
            if (offset == 0)
                continue;

            var depth = zMap.Depth + offset;
            if (depth >= 0 && depth < network.SortedZLevels.Count)
                maps.Add(network.SortedZLevels[depth]);
        }
    }

    [Pure]
    public bool TryGetMapDepth(EntityUid map, [NotNullWhen(true)] out int? depth)
    {
        if (_zMapQuery.TryComp(map, out var zMap))
        {
            depth = zMap.Depth;
            return true;
        }

        depth = null;
        return false;
    }

    public void CollectRenderableMaps(
        EntityUid mapUid,
        MapId mapId,
        int below,
        int above,
        Box2 worldAABB,
        Box2Rotated worldBounds,
        List<MapId> belowMaps,
        List<MapId> aboveMaps)
    {
        belowMaps.Clear();
        aboveMaps.Clear();
        if (!_zMapQuery.HasComp(mapUid))
            return;

        CollectRenderableMapDirection(mapUid, -1, below, belowMaps);
        CollectRenderableMapDirection(mapUid, 1, above, aboveMaps);
    }

    private void CollectRenderableMapDirection(EntityUid mapUid, int direction, int count, List<MapId> maps)
    {
        for (var current = mapUid; maps.Count < Math.Max(0, count);)
        {
            if (!TryGetMapOffset(current, direction, out var next) ||
                !_mapQuery.TryComp(next.Value, out var map))
            {
                return;
            }

            maps.Add(map.MapId);
            current = next.Value;
        }
    }

    /// <summary>
    /// Checks whether holes or transparent tiles allow the next lower map to be seen.
    /// </summary>
    public bool NeedsLowerLevelRendered(MapId mapId, Box2 worldAABB, Box2Rotated worldBounds)
    {
        _renderOccluderGrids.Clear();
        _mapSystem.FindGridsIntersecting(mapId, worldAABB.Enlarged(1f), ref _renderOccluderGrids, approx: true);

        foreach (var grid in _renderOccluderGrids)
        {
            var matrix = _transform.GetWorldMatrix(grid.Owner);
            var gridAabb = matrix.TransformBox(grid.Comp.LocalAABB).Enlarged(grid.Comp.TileSize * 2f);
            if (gridAabb.Contains(worldAABB))
                return GridViewportContainsEmptyTile(grid, worldBounds);
        }

        return true;
    }

    [Pure]
    public bool GridViewportContainsEmptyTile(Entity<MapGridComponent> grid, Box2Rotated worldBounds)
    {
        var localAabb = _transform.GetInvWorldMatrix(grid.Owner).TransformBox(worldBounds).Enlarged(grid.Comp.TileSize);
        var tileSize = grid.Comp.TileSize;
        var min = new Vector2i(
            (int) MathF.Floor(localAabb.Left / tileSize) - 1,
            (int) MathF.Floor(localAabb.Bottom / tileSize) - 1);
        var max = new Vector2i(
            (int) MathF.Ceiling(localAabb.Right / tileSize) + 1,
            (int) MathF.Ceiling(localAabb.Top / tileSize) + 1);
        var chunkSize = grid.Comp.ChunkSize;
        var minChunk = _mapSystem.GridTileToChunkIndices(grid.Comp, min);
        var maxChunk = _mapSystem.GridTileToChunkIndices(grid.Comp, max);

        for (var chunkX = minChunk.X; chunkX <= maxChunk.X; chunkX++)
        {
            for (var chunkY = minChunk.Y; chunkY <= maxChunk.Y; chunkY++)
            {
                var chunkIndex = new Vector2i(chunkX, chunkY);
                if (!grid.Comp.Chunks.TryGetValue(chunkIndex, out var chunk))
                    return true;

                var origin = chunkIndex * chunkSize;
                var relativeMin = min - origin;
                var relativeMax = max - origin;
                var localMin = new Vector2i(Math.Max(0, relativeMin.X), Math.Max(0, relativeMin.Y));
                var localMax = new Vector2i(
                    Math.Min(chunkSize - 1, relativeMax.X),
                    Math.Min(chunkSize - 1, relativeMax.Y));
                for (var x = localMin.X; x <= localMax.X; x++)
                {
                    for (var y = localMin.Y; y <= localMax.Y; y++)
                    {
                        var tile = chunk.GetTile((ushort) x, (ushort) y);
                        if (tile.IsEmpty ||
                            _tileDefinitions.TryGetDefinition(tile.TypeId, out var definition) && definition.ZLevelTransparent)
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    [Pure]
    public bool HasLinearDepths(Entity<ZLevelMapNetworkComponent> network)
    {
        for (var i = 0; i < network.Comp.SortedZLevels.Count; i++)
        {
            var map = network.Comp.SortedZLevels[i];
            if (!_zMapQuery.TryComp(map, out var zMap) || zMap.Network != network.Owner || zMap.Depth != i)
                return false;
        }

        return true;
    }

    public bool TryRemoveMapFromNetwork(EntityUid map)
    {
        if (!TryGetMapNetwork(map, out var network) || network is not { } entity)
            return false;

        var levels = new List<EntityUid>(entity.Comp.SortedZLevels);
        if (!levels.Remove(map))
            return false;

        RemCompDeferred<ZLevelMapComponent>(map);
        if (levels.Count == 0)
            PredictedQueueDel(entity.Owner);
        else
            SetNetworkLevels(entity, levels);

        return true;
    }

    private void OnZMapShutdown(Entity<ZLevelMapComponent> entity, ref ComponentShutdown args)
    {
        if (!_networkQuery.TryComp(entity.Comp.Network, out var networkComp))
            return;

        Entity<ZLevelMapNetworkComponent> network = (entity.Comp.Network, networkComp);
        var levels = new List<EntityUid>(network.Comp.SortedZLevels);
        if (!levels.Remove(entity.Owner))
            return;

        if (levels.Count == 0)
        {
            if (!TerminatingOrDeleted(network.Owner))
                QueueDel(network.Owner);
        }
        else if (!TerminatingOrDeleted(network.Owner))
        {
            SetNetworkLevels(network, levels);
        }
    }

    private void OnNetworkStartup(Entity<ZLevelMapNetworkComponent> entity, ref ComponentStartup args)
    {
        // The network entity lives in nullspace, so normal spatial PVS cannot discover its replicated configuration.
        _pvsOverride.AddGlobalOverride(entity);
        if (entity.Comp.Maps.Count == 0)
            return;

        var seen = new HashSet<EntityUid>();
        if (entity.Comp.Maps.Any(map => !_mapQuery.HasComp(map) || _zMapQuery.HasComp(map) || !seen.Add(map)))
        {
            Log.Error($"Unable to restore z-level network {ToPrettyString(entity)}: its saved map list is invalid.");
            return;
        }

        SetNetworkLevels(entity, entity.Comp.Maps);
    }

    private void OnNetworkShutdown(Entity<ZLevelMapNetworkComponent> entity, ref ComponentShutdown args)
    {
        _pvsOverride.RemoveGlobalOverride(entity);
        foreach (var map in entity.Comp.SortedZLevels)
        {
            if (_zMapQuery.TryComp(map, out var zMap) && zMap.Network == entity.Owner)
                RemCompDeferred<ZLevelMapComponent>(map);
        }
    }

    private void OnBeforeSerialization(BeforeSerializationEvent args)
    {
        if (args.Category != FileCategory.Save)
            return;

        var query = AllEntityQuery<ZLevelMapNetworkComponent>();
        while (query.MoveNext(out _, out var network))
        {
            network.Maps.Clear();
            network.Maps.AddRange(network.SortedZLevels);
        }
    }

    private void SetNetworkLevels(Entity<ZLevelMapNetworkComponent> network, IReadOnlyList<EntityUid> maps)
    {
        network.Comp.SortedZLevelsInternal.Clear();
        for (var depth = 0; depth < maps.Count; depth++)
        {
            var map = maps[depth];
            network.Comp.SortedZLevelsInternal.Add(map);
            var zMap = EnsureComp<ZLevelMapComponent>(map);
            zMap.Network = network.Owner;
            zMap.Depth = depth;
            DirtyFields(map, zMap, null, nameof(ZLevelMapComponent.Network), nameof(ZLevelMapComponent.Depth));
        }

        DirtyField(network.Owner, network.Comp, nameof(ZLevelMapNetworkComponent.SortedZLevelsInternal));
    }
}
