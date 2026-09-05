using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using JetBrains.Annotations;
using Robust.Shared.GameObjects;
using Robust.Shared.Configuration;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Shapes;
using PhysicsTransform = Robust.Shared.Physics.Transform;

namespace Robust.Shared.Physics.Systems;

public sealed partial class ZLevelPhysicsSystem
{
    [Dependency] private ZLevelSupportSystem _support = default!;

    private readonly Dictionary<EntityUid, HashSet<EntityUid>> _supportedBodiesByProvider = new();
    private readonly HashSet<EntityUid> _supportRefreshBodies = new();
    private readonly List<ZLevelSupportCandidateDebug> _supportDiagnosticsScratch = new();

    /// <summary>
    /// Recomputes the support state beneath a body's canonical support point.
    /// </summary>
    public void RefreshSupport(Entity<ZLevelPhysicsComponent> entity, bool wake = true)
    {
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            return;
        }

        if (!_xformQuery.TryComp(entity.Owner, out var xform) ||
            xform.MapUid is not { } currentMap ||
            !_zLevels.TryGetMapDepth(currentMap, out var currentDepth))
        {
            return;
        }

        var presentation = EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        var oldGround = entity.Comp.GroundState;
        var currentAbsoluteHeight = ZLevelProjection.GetAbsoluteZ(currentDepth.Value, presentation.LocalHeight);
        var maxRise = oldGround == ZLevelGroundState.Grounded && entity.Comp.AutoStep
            ? _maxStepUp
            : PositionEpsilon;

        _supportDiagnosticsScratch.Clear();
        var selected = _support.TryQuerySupport(
            entity,
            _transform.GetWorldPosition(xform),
            currentAbsoluteHeight,
            maxRise,
            out var support,
            _supportDiagnosticsScratch);
        _support.SetLastSupportCandidates(entity.Owner, _supportDiagnosticsScratch);

        if (!selected)
        {
            ApplySupportResult(entity, ZLevelSupportResult.None);
            if (oldGround == ZLevelGroundState.Grounded)
            {
                SetGroundState(entity, ZLevelGroundState.Airborne);
                if (wake)
                    WakeBody(entity);
            }

            return;
        }

        ApplySupportResult(entity, support);

        if (oldGround != ZLevelGroundState.Grounded)
            return;

        var rise = support.AbsoluteHeight - currentAbsoluteHeight;
        var maximumSnapDown = MathF.Min(_maxStepDown, _groundSnapDistance);
        if (rise <= _maxStepUp + PositionEpsilon &&
            rise >= -maximumSnapDown - PositionEpsilon)
        {
            SetLocalHeight(
                (entity.Owner, presentation),
                ZLevelProjection.GetLocalHeight(support.AbsoluteHeight, currentDepth.Value));
            SetGroundState(entity, ZLevelGroundState.Grounded);
            SetVerticalVelocity(entity, 0f);
            NormalizeEntityMapFromAbsoluteZ(entity, support.AbsoluteHeight);
            return;
        }

        SetGroundState(entity, ZLevelGroundState.Airborne);
        if (wake)
            WakeBody(entity);
    }

    /// <summary>
    /// Applies a selected support result without changing grounded state, velocity, z height, or map parent.
    /// </summary>
    private void ApplySupportResult(Entity<ZLevelPhysicsComponent> entity, ZLevelSupportResult support)
    {
        if (entity.Comp.SupportProvider == support.Provider &&
            entity.Comp.SupportSurface == support.Surface &&
            entity.Comp.SupportHeight.Equals(support.AbsoluteHeight))
        {
            return;
        }

        ReplaceSupportedBodyProvider(entity.Owner, entity.Comp.SupportProvider, support.Provider);
        entity.Comp.SupportProvider = support.Provider;
        entity.Comp.SupportSurface = support.Surface;
        entity.Comp.SupportHeight = support.AbsoluteHeight;
        DirtyFields(
            entity.Owner,
            entity.Comp,
            null,
            nameof(ZLevelPhysicsComponent.SupportProvider),
            nameof(ZLevelPhysicsComponent.SupportSurface),
            nameof(ZLevelPhysicsComponent.SupportHeight));
    }

    private void SetGroundState(Entity<ZLevelPhysicsComponent> entity, ZLevelGroundState state)
    {
        if (entity.Comp.GroundState == state)
            return;

        entity.Comp.GroundState = state;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPhysicsComponent.GroundState));
    }

    private void SetVerticalVelocity(Entity<ZLevelPhysicsComponent> entity, float velocity)
    {
        if (entity.Comp.Velocity.Equals(velocity))
            return;

        entity.Comp.Velocity = velocity;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPhysicsComponent.Velocity));
    }

    [Pure]
    public float GetLocalSupportHeight(Entity<ZLevelPhysicsComponent> entity)
    {
        if (entity.Comp.SupportSurface == ZLevelSupportSurface.None ||
            Transform(entity).MapUid is not { } map ||
            !_zLevels.TryGetMapDepth(map, out var depth))
        {
            return 0f;
        }

        return ZLevelProjection.GetLocalHeight(entity.Comp.SupportHeight, depth.Value);
    }

    /// <summary>
    /// Reparents a grounded body to the z-map that owns its absolute support height.
    /// </summary>
    public bool NormalizeEntityMapFromAbsoluteZ(Entity<ZLevelPhysicsComponent> entity, float absoluteZ)
    {
        if (!float.IsFinite(absoluteZ) ||
            !_xformQuery.TryComp(entity.Owner, out var xform) ||
            xform.MapUid is not { } currentMap ||
            !_zLevels.TryGetMapData(currentMap, out var currentZMap, out var network))
        {
            return false;
        }

        var targetDepth = Math.Clamp(
            (int) MathF.Floor(absoluteZ + PositionEpsilon),
            0,
            network.SortedZLevels.Count - 1);
        var offset = targetDepth - currentZMap.Depth;
        if (offset == 0)
            return true;

        if (!_zLevels.TryMoveEntityToMapOffset(entity.Owner, offset, out var targetMap) ||
            targetMap is not { } targetMapUid ||
            !_zLevels.TryGetMapDepth(targetMapUid, out var targetMapDepth) ||
            !_zPresentationQuery.TryComp(entity.Owner, out var presentation))
        {
            return false;
        }

        SetLocalHeight(
            (entity.Owner, presentation),
            ZLevelProjection.GetLocalHeight(absoluteZ, targetMapDepth.Value));
        var moved = new ZLevelMapMoveEvent(offset, targetMapUid);
        RaiseLocalEvent(entity.Owner, ref moved);
        return true;
    }

    /// <summary>
    /// Replaces an anchored flat support height and refreshes nearby/supported bodies.
    /// </summary>
    public void SetSupportHeight(Entity<ZLevelHighGroundComponent> entity, float height)
    {
        if (!float.IsFinite(height) || entity.Comp.Height.Equals(height))
            return;

        entity.Comp.Height = height;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelHighGroundComponent.Height));
        RefreshBodiesAtHighGround(entity.Owner);
    }

    public bool TryGetSupportSurfacePoints(
        Entity<ZLevelHighGroundComponent> provider,
        Span<ZLevelSupportPoint> points,
        out int count)
        => _support.TryGetSupportSurfacePoints(provider, points, out count);

    public bool TrySampleSupportHeight(
        EntityUid provider,
        ZLevelSupportSurface surface,
        Vector2 worldPoint,
        out float absoluteHeight)
        => _support.TrySampleSurfaceHeight(provider, surface, worldPoint, out absoluteHeight);

    public bool TryGetSupportLocalBounds(Entity<ZLevelHighGroundComponent> provider, out Box2 bounds)
        => _support.TryGetSupportLocalBounds(provider, out bounds);

    internal void RefreshSupportsOnMovedGrid(EntityUid grid)
    {
        _supportRefreshBodies.Clear();
        AddSupportedBodies(grid, _supportRefreshBodies);

        foreach (var provider in _supportedBodiesByProvider.Keys.ToArray())
        {
            if (provider == grid ||
                !_xformQuery.TryComp(provider, out var providerXform) ||
                providerXform.GridUid != grid)
            {
                continue;
            }

            AddSupportedBodies(provider, _supportRefreshBodies);
        }

        RefreshSupportBodySet(wake: false);
    }

    private void RefreshSupportedBodiesAtProvider(EntityUid provider)
    {
        _supportRefreshBodies.Clear();
        AddSupportedBodies(provider, _supportRefreshBodies);
        RefreshSupportBodySet(wake: true);
    }

    private void RefreshSupportBodySet(bool wake)
    {
        foreach (var uid in _supportRefreshBodies)
        {
            if (!_zPhysicsQuery.TryComp(uid, out var physics) || !CanSimulateBody((uid, physics)))
                continue;

            RefreshSupport((uid, physics), wake);
            if (wake)
                WakeBody((uid, physics));
        }

        _supportRefreshBodies.Clear();
    }

    private void AddSupportedBodies(EntityUid provider, HashSet<EntityUid> bodies)
    {
        if (!_supportedBodiesByProvider.TryGetValue(provider, out var supported))
            return;

        foreach (var body in supported)
            bodies.Add(body);
    }

    private void ReplaceSupportedBodyProvider(EntityUid body, EntityUid? oldProvider, EntityUid? newProvider)
    {
        if (oldProvider == newProvider)
            return;

        if (oldProvider is { } oldUid &&
            _supportedBodiesByProvider.TryGetValue(oldUid, out var oldBodies))
        {
            oldBodies.Remove(body);
            if (oldBodies.Count == 0)
                _supportedBodiesByProvider.Remove(oldUid);
        }

        if (newProvider is not { } newUid)
            return;

        if (!_supportedBodiesByProvider.TryGetValue(newUid, out var newBodies))
        {
            newBodies = new HashSet<EntityUid>();
            _supportedBodiesByProvider.Add(newUid, newBodies);
        }

        newBodies.Add(body);
    }

    private void ClearSupportTracking(Entity<ZLevelPhysicsComponent> entity)
    {
        ReplaceSupportedBodyProvider(entity.Owner, entity.Comp.SupportProvider, null);
        _support.ClearLastSupportCandidates(entity.Owner);
    }
}

/// <summary>
/// Pure support-surface queries for flat z-level floors, platforms, and wall tops.
/// </summary>
public sealed partial class ZLevelSupportSystem : EntitySystem
{
    private const int MaxSupportDiagnostics = 64;
    private const float PositionEpsilon = 0.00001f;

    [Dependency] private IConfigurationManager _configuration = default!;
    [Dependency] private IManifoldManager _manifold = default!;
    [Dependency] private EntityLookupSystem _lookup = default!;
    [Dependency] private SharedMapSystem _map = default!;
    [Dependency] private SharedPhysicsSystem _physics = default!;
    [Dependency] private SharedTransformSystem _transform = default!;
    [Dependency] private ZLevelSystem _zLevels = default!;

    [Dependency] private EntityQuery<FixturesComponent> _fixturesQuery = default!;
    [Dependency] private EntityQuery<MapComponent> _mapQuery = default!;
    [Dependency] private EntityQuery<MapGridComponent> _gridQuery = default!;
    [Dependency] private EntityQuery<TransformComponent> _xformQuery = default!;
    [Dependency] private EntityQuery<ZLevelHighGroundComponent> _highGroundQuery = default!;
    [Dependency] private EntityQuery<ZLevelMapComponent> _zMapQuery = default!;

    private PhysShapeCircle _supportProbe = new();
    private float _supportHysteresis;
    private readonly HashSet<Entity<ZLevelHighGroundComponent>> _supportProviders = new();
    private List<Entity<MapGridComponent>> _supportGrids = new();
    private readonly List<ZLevelSupportResult> _supportCandidates = new();
    private readonly Dictionary<EntityUid, List<ZLevelSupportCandidateDebug>> _lastSupportCandidates = new();

    public float SupportHysteresis => _supportHysteresis;

    public override void Initialize()
    {
        base.Initialize();
        Subs.CVar(_configuration, CVars.PhysicsZLevelSupportHysteresis, SetSupportHysteresis, true);
    }

    private void SetSupportHysteresis(float value)
    {
        _supportHysteresis = MathF.Max(0f, value);
        _supportProbe = new PhysShapeCircle(_supportHysteresis);
    }

    [Pure]
    public bool TryQuerySupport(
        Entity<ZLevelPhysicsComponent> body,
        Vector2 proposedWorldPosition,
        float currentAbsoluteHeight,
        float maximumRise,
        out ZLevelSupportResult selected,
        List<ZLevelSupportCandidateDebug>? diagnostics = null)
    {
        selected = ZLevelSupportResult.None;
        _supportCandidates.Clear();
        diagnostics?.Clear();

        if (!float.IsFinite(currentAbsoluteHeight) ||
            !_xformQuery.TryComp(body.Owner, out var xform) ||
            xform.MapUid is not { } currentMap ||
            !_zMapQuery.TryComp(currentMap, out _) ||
            !_mapQuery.TryComp(currentMap, out _))
        {
            return false;
        }

        maximumRise = MathF.Max(0f, maximumRise);
        var sample = new SupportSample(
            proposedWorldPosition,
            Box2.CenteredAround(proposedWorldPosition, new Vector2(_supportHysteresis * 2f)),
            new PhysicsTransform(proposedWorldPosition, Angle.Zero));

        for (var floor = 0; floor <= 1; floor++)
        {
            var checkingMap = currentMap;
            if (floor != 0)
            {
                if (!_zLevels.TryGetMapOffset(currentMap, -floor, out var below) || below is not { } belowMap)
                    continue;

                checkingMap = belowMap;
            }

            if (!_zMapQuery.TryComp(checkingMap, out var checkingZMap) ||
                !_mapQuery.TryComp(checkingMap, out var checkingMapComp))
            {
                continue;
            }

            CollectHighGroundCandidates(
                body.Owner,
                checkingMap,
                checkingMapComp.MapId,
                checkingZMap.Depth,
                sample,
                currentAbsoluteHeight,
                maximumRise,
                diagnostics);
            CollectTileCandidates(
                checkingMapComp.MapId,
                checkingZMap.Depth,
                sample,
                currentAbsoluteHeight,
                maximumRise,
                diagnostics);
        }

        if (_supportCandidates.Count == 0)
            return false;

        _supportCandidates.Sort(CompareSupportCandidates);
        selected = _supportCandidates[0];
        return true;
    }

    [Pure]
    public bool TrySampleSurfaceHeight(
        EntityUid provider,
        ZLevelSupportSurface surface,
        Vector2 worldPoint,
        out float absoluteHeight)
    {
        absoluteHeight = 0f;
        switch (surface)
        {
            case ZLevelSupportSurface.Tile:
                if (!_gridQuery.TryComp(provider, out _) ||
                    !_xformQuery.TryComp(provider, out var gridXform) ||
                    gridXform.MapUid is not { } gridMap ||
                    !_zLevels.TryGetMapDepth(gridMap, out var gridDepth))
                {
                    return false;
                }

                absoluteHeight = gridDepth.Value;
                return true;
            case ZLevelSupportSurface.HighGround:
                if (!_highGroundQuery.TryComp(provider, out var highGround) ||
                    !_xformQuery.TryComp(provider, out var xform) ||
                    xform.MapUid is not { } map ||
                    !_zLevels.TryGetMapDepth(map, out var depth) ||
                    !float.IsFinite(highGround.Height))
                {
                    return false;
                }

                absoluteHeight = depth.Value + highGround.Height;
                return true;
            case ZLevelSupportSurface.NetworkBoundary:
                if (!_zLevels.TryGetMapDepth(provider, out var mapDepth))
                    return false;

                absoluteHeight = mapDepth.Value;
                return true;
            default:
                return false;
        }
    }

    public bool TryGetSupportSurfacePoints(
        Entity<ZLevelHighGroundComponent> provider,
        Span<ZLevelSupportPoint> points,
        out int count)
    {
        count = 0;
        if (points.Length < 4 ||
            !TryGetSupportLocalBounds(provider, out var bounds) ||
            !_xformQuery.TryComp(provider.Owner, out var xform) ||
            xform.MapUid is not { } map ||
            !_zLevels.TryGetMapDepth(map, out var depth) ||
            !float.IsFinite(provider.Comp.Height))
        {
            return false;
        }

        var matrix = _transform.GetWorldMatrix(xform);
        var absoluteHeight = depth.Value + provider.Comp.Height;
        Span<Vector2> local = stackalloc Vector2[4]
        {
            bounds.BottomLeft,
            bounds.BottomRight,
            bounds.TopRight,
            bounds.TopLeft,
        };

        for (var i = 0; i < local.Length; i++)
            points[i] = new ZLevelSupportPoint(Vector2.Transform(local[i], matrix), absoluteHeight);

        count = 4;
        return true;
    }

    /// <summary>
    /// Gets the explicit authored support fixture bounds in provider-local coordinates.
    /// </summary>
    [Pure]
    public bool TryGetSupportLocalBounds(Entity<ZLevelHighGroundComponent> provider, out Box2 bounds)
    {
        bounds = default;
        if (!TryGetSupportFixture(provider, out var fixture))
            return false;

        var found = false;
        for (var child = 0; child < fixture.Shape.ChildCount; child++)
        {
            var childBounds = fixture.Shape.ComputeAABB(PhysicsTransform.Empty, child);
            if (fixture.Shape is PolygonShape or PhysShapeAabb)
            {
                var radius = fixture.Shape.Radius;
                if (childBounds.Width > radius * 2f && childBounds.Height > radius * 2f)
                {
                    childBounds = new Box2(
                        childBounds.Left + radius,
                        childBounds.Bottom + radius,
                        childBounds.Right - radius,
                        childBounds.Top - radius);
                }
            }

            bounds = found ? Union(bounds, childBounds) : childBounds;
            found = true;
        }

        return found;
    }

    public IReadOnlyList<ZLevelSupportCandidateDebug> GetLastSupportCandidates(EntityUid body)
        => _lastSupportCandidates.TryGetValue(body, out var candidates)
            ? candidates
            : Array.Empty<ZLevelSupportCandidateDebug>();

    internal void SetLastSupportCandidates(EntityUid body, IReadOnlyList<ZLevelSupportCandidateDebug> candidates)
    {
        if (candidates.Count == 0)
        {
            _lastSupportCandidates.Remove(body);
            return;
        }

        if (!_lastSupportCandidates.TryGetValue(body, out var stored))
        {
            stored = new List<ZLevelSupportCandidateDebug>(Math.Min(candidates.Count, MaxSupportDiagnostics));
            _lastSupportCandidates.Add(body, stored);
        }

        stored.Clear();
        var count = Math.Min(candidates.Count, MaxSupportDiagnostics);
        for (var i = 0; i < count; i++)
            stored.Add(candidates[i]);
    }

    internal void ClearLastSupportCandidates(EntityUid body)
    {
        _lastSupportCandidates.Remove(body);
    }

    private void CollectHighGroundCandidates(
        EntityUid body,
        EntityUid checkingMap,
        MapId mapId,
        int mapDepth,
        in SupportSample sample,
        float currentAbsoluteHeight,
        float maximumRise,
        List<ZLevelSupportCandidateDebug>? diagnostics)
    {
        _supportProviders.Clear();
        _lookup.GetEntitiesIntersecting(
            mapId,
            sample.Bounds,
            _supportProviders,
            LookupFlags.Static | LookupFlags.Sundries | LookupFlags.Sensors);

        foreach (var provider in _supportProviders.OrderBy(entry => entry.Owner))
        {
            if (provider.Owner == body)
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, 0f, sample.Point,
                    ZLevelSupportRejection.Self, diagnostics);
                continue;
            }

            if (!_xformQuery.TryComp(provider.Owner, out var providerXform) || providerXform.MapUid != checkingMap)
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, 0f, sample.Point,
                    ZLevelSupportRejection.WrongMap, diagnostics);
                continue;
            }

            if (string.IsNullOrWhiteSpace(provider.Comp.SurfaceFixture))
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, 0f, sample.Point,
                    ZLevelSupportRejection.NoSurface, diagnostics);
                continue;
            }

            if (!TryGetSupportFixture(provider, out var fixture))
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, 0f, sample.Point,
                    ZLevelSupportRejection.NoFixture, diagnostics);
                continue;
            }

            if (!TryGetHighGroundHeight(
                    provider,
                    providerXform,
                    fixture,
                    sample.Point,
                    out var height))
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, provider.Comp.Height, sample.Point,
                    ZLevelSupportRejection.NonFiniteHeight, diagnostics);
                continue;
            }

            if (!SupportSampleOverlapsFixture(sample, provider.Owner, providerXform, fixture))
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, 0f, sample.Point,
                    ZLevelSupportRejection.OutsideFootprint, diagnostics);
                continue;
            }

            var absoluteHeight = mapDepth + height;
            if (absoluteHeight > currentAbsoluteHeight + maximumRise + PositionEpsilon)
            {
                AddDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, absoluteHeight, sample.Point,
                    ZLevelSupportRejection.AboveStepLimit, diagnostics);
                continue;
            }

            var candidate = new ZLevelSupportResult(
                provider.Owner,
                ZLevelSupportSurface.HighGround,
                absoluteHeight,
                sample.Point);
            _supportCandidates.Add(candidate);
            AddDiagnostic(candidate, ZLevelSupportRejection.None, diagnostics);
        }
    }

    private void CollectTileCandidates(
        MapId mapId,
        int mapDepth,
        in SupportSample sample,
        float currentAbsoluteHeight,
        float maximumRise,
        List<ZLevelSupportCandidateDebug>? diagnostics)
    {
        if (mapDepth > currentAbsoluteHeight + maximumRise + PositionEpsilon)
            return;

        _supportGrids.Clear();
        _map.FindGridsIntersecting(mapId, sample.Bounds, ref _supportGrids, approx: false, includeMap: true);
        _supportGrids.Sort((left, right) => left.Owner.CompareTo(right.Owner));

        EntityUid previous = default;
        foreach (var grid in _supportGrids)
        {
            if (grid.Owner == previous)
                continue;
            previous = grid.Owner;

            if (TileSupportOverlapsSample(grid, sample))
            {
                var candidate = new ZLevelSupportResult(
                    grid.Owner,
                    ZLevelSupportSurface.Tile,
                    mapDepth,
                    sample.Point);
                _supportCandidates.Add(candidate);
                AddDiagnostic(candidate, ZLevelSupportRejection.None, diagnostics);
            }
        }
    }

    private bool TileSupportOverlapsSample(Entity<MapGridComponent> grid, in SupportSample sample)
    {
        var inverse = _transform.GetInvWorldMatrix(grid.Owner);
        var matrix = _transform.GetWorldMatrix(grid.Owner);
        var localBounds = inverse.TransformBox(sample.Bounds);
        var tileSize = grid.Comp.TileSize;
        var min = new Vector2i(
            (int) MathF.Floor(localBounds.Left / tileSize),
            (int) MathF.Floor(localBounds.Bottom / tileSize));
        var max = new Vector2i(
            (int) MathF.Floor(localBounds.Right / tileSize),
            (int) MathF.Floor(localBounds.Top / tileSize));

        for (var x = min.X; x <= max.X; x++)
        {
            for (var y = min.Y; y <= max.Y; y++)
            {
                var tileIndex = new Vector2i(x, y);
                if (!_map.TryGetTileRef(grid.Owner, grid.Comp, tileIndex, out var tile) || tile.Tile.IsEmpty)
                    continue;

                var tileBounds = new Box2(
                    new Vector2(x * tileSize, y * tileSize),
                    new Vector2((x + 1) * tileSize, (y + 1) * tileSize));
                var tileShape = new SlimPolygon(tileBounds, matrix, out var tileWorldBounds);
                if (!tileWorldBounds.Intersects(sample.Bounds))
                    continue;

                if (_manifold.TestOverlap(
                        _supportProbe,
                        0,
                        tileShape,
                        0,
                        sample.Transform,
                        PhysicsTransform.Empty,
                        ignoreShapeSkin: true))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private bool SupportSampleOverlapsFixture(
        in SupportSample sample,
        EntityUid provider,
        TransformComponent providerXform,
        Fixture fixture)
    {
        var providerTransform = _physics.GetPhysicsTransform(provider, providerXform);
        for (var child = 0; child < fixture.Shape.ChildCount; child++)
        {
            if (_manifold.TestOverlap(
                    _supportProbe,
                    0,
                    fixture.Shape,
                    child,
                    sample.Transform,
                    providerTransform,
                    ignoreShapeSkin: true))
            {
                return true;
            }
        }

        return false;
    }

    private bool TryGetHighGroundHeight(
        Entity<ZLevelHighGroundComponent> provider,
        TransformComponent providerXform,
        Fixture fixture,
        Vector2 samplePoint,
        out float height)
    {
        if (provider.Comp.HeightCurve.Count == 0)
        {
            height = provider.Comp.Height;
            return float.IsFinite(height);
        }

        if (!TryGetFixtureLocalBounds(fixture, out var bounds))
        {
            height = 0f;
            return false;
        }

        var localPoint = Vector2.Transform(samplePoint, _transform.GetInvWorldMatrix(provider.Owner));
        var local = new Vector2(
            Normalize(localPoint.X, bounds.Left, bounds.Right),
            Normalize(localPoint.Y, bounds.Bottom, bounds.Top));

        var direction = providerXform.LocalRotation.GetCardinalDir();
        var t = direction switch
        {
            Direction.East => provider.Comp.Corner ? (local.X + 1f - local.Y) / 2f : local.X,
            Direction.West => provider.Comp.Corner ? (1f - local.X + local.Y) / 2f : 1f - local.X,
            Direction.North => provider.Comp.Corner ? (local.X + local.Y) / 2f : local.Y,
            Direction.South => provider.Comp.Corner ? (1f - local.X + 1f - local.Y) / 2f : 1f - local.Y,
            _ => 0.5f,
        };

        height = InterpolateHeight(provider.Comp.HeightCurve, Math.Clamp(t, 0f, 1f));
        return float.IsFinite(height);
    }

    private static float Normalize(float value, float min, float max)
    {
        var range = max - min;
        return MathF.Abs(range) <= PositionEpsilon
            ? 0.5f
            : (value - min) / range;
    }

    private static float InterpolateHeight(IReadOnlyList<float> curve, float t)
    {
        if (curve.Count == 1)
            return curve[0];

        var scaled = t * (curve.Count - 1);
        var index = Math.Min((int) scaled, curve.Count - 2);
        return MathHelper.Lerp(curve[index], curve[index + 1], scaled - index);
    }

    private bool TryGetSupportFixture(Entity<ZLevelHighGroundComponent> provider, out Fixture fixture)
    {
        fixture = default!;
        if (string.IsNullOrWhiteSpace(provider.Comp.SurfaceFixture) ||
            !_fixturesQuery.TryComp(provider.Owner, out var fixtures) ||
            !fixtures.Fixtures.TryGetValue(provider.Comp.SurfaceFixture, out var found))
        {
            return false;
        }

        fixture = found;
        return true;
    }

    private static bool TryGetFixtureLocalBounds(Fixture fixture, out Box2 bounds)
    {
        bounds = default;
        var any = false;
        for (var child = 0; child < fixture.Shape.ChildCount; child++)
        {
            var childBounds = fixture.Shape.ComputeAABB(PhysicsTransform.Empty, child);
            if (fixture.Shape is PolygonShape or PhysShapeAabb)
            {
                var radius = fixture.Shape.Radius;
                childBounds = new Box2(
                    childBounds.Left + radius,
                    childBounds.Bottom + radius,
                    childBounds.Right - radius,
                    childBounds.Top - radius);
            }

            bounds = any ? Union(bounds, childBounds) : childBounds;
            any = true;
        }

        return any;
    }

    private static void AddDiagnostic(
        ZLevelSupportResult candidate,
        ZLevelSupportRejection rejection,
        List<ZLevelSupportCandidateDebug>? diagnostics)
        => AddDiagnostic(
            candidate.Provider ?? EntityUid.Invalid,
            candidate.Surface,
            candidate.AbsoluteHeight,
            candidate.SamplePoint,
            rejection,
            diagnostics);

    private static void AddDiagnostic(
        EntityUid provider,
        ZLevelSupportSurface surface,
        float height,
        Vector2 point,
        ZLevelSupportRejection rejection,
        List<ZLevelSupportCandidateDebug>? diagnostics)
    {
        if (diagnostics == null || diagnostics.Count >= MaxSupportDiagnostics)
            return;

        diagnostics.Add(new(provider, surface, height, point, rejection));
    }

    private static int CompareSupportCandidates(ZLevelSupportResult left, ZLevelSupportResult right)
    {
        var comparison = right.AbsoluteHeight.CompareTo(left.AbsoluteHeight);
        if (comparison != 0)
            return comparison;

        comparison = right.Surface.CompareTo(left.Surface);
        if (comparison != 0)
            return comparison;

        var leftProvider = left.Provider ?? EntityUid.Invalid;
        var rightProvider = right.Provider ?? EntityUid.Invalid;
        return leftProvider.CompareTo(rightProvider);
    }

    private static Box2 Union(Box2 left, Box2 right)
        => new(Vector2.Min(left.BottomLeft, right.BottomLeft), Vector2.Max(left.TopRight, right.TopRight));

    private readonly record struct SupportSample(
        Vector2 Point,
        Box2 Bounds,
        PhysicsTransform Transform);
}

public readonly record struct ZLevelSupportResult(
    EntityUid? Provider,
    ZLevelSupportSurface Surface,
    float AbsoluteHeight,
    Vector2 SamplePoint)
{
    public static readonly ZLevelSupportResult None = new(null, ZLevelSupportSurface.None, 0f, default);
}

public enum ZLevelSupportRejection : byte
{
    None,
    Self,
    WrongMap,
    NoSurface,
    NoFixture,
    OutsideFootprint,
    NonFiniteHeight,
    AboveStepLimit,
}

/// <summary>
/// One accepted or rejected result from the latest deterministic support query.
/// </summary>
public readonly record struct ZLevelSupportCandidateDebug(
    EntityUid Provider,
    ZLevelSupportSurface Surface,
    float AbsoluteHeight,
    Vector2 SamplePoint,
    ZLevelSupportRejection Rejection);

public readonly record struct ZLevelSupportPoint(Vector2 CanonicalPosition, float AbsoluteHeight);
