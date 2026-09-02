using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using JetBrains.Annotations;
using Robust.Shared.GameObjects;
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
    private const int MaxSupportDiagnostics = 64;
    private const float DefaultFootprintRadius = 0.05f;

    [Dependency] private IManifoldManager _manifold = null!;
    [Dependency] private EntityQuery<FixturesComponent> _fixturesQuery;
    [Dependency] private EntityQuery<MapGridComponent> _gridQuery;

    private readonly HashSet<Entity<ZLevelHighGroundComponent>> _supportProviders = new();
    private List<Entity<MapGridComponent>> _supportGrids = new();
    private readonly List<SupportCandidate> _supportCandidates = new();

    /// <summary>
    /// Recomputes the deterministic support state beneath a body's complete fixture footprint.
    /// </summary>
    public void RefreshSupport(Entity<ZLevelPhysicsComponent> entity, bool wake = true)
    {
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            return;
        }

        var hadSupport = entity.Comp.SupportSurface != ZLevelSupportSurface.None;
        var oldProvider = entity.Comp.SupportProvider;
        var oldSurface = entity.Comp.SupportSurface;
        var oldTile = entity.Comp.SupportTile;
        var oldState = entity.Comp.GroundState;

        var selected = TrySelectSupport(entity, out var candidate);
        if (!selected)
        {
            SetSupportState(entity, null, ZLevelSupportSurface.None, default, 0f, default);
            if (oldState == ZLevelGroundState.Grounded)
            {
                SetGroundState(entity, ZLevelGroundState.Airborne, ZLevelReconciliationState.AuthoritativeFall);
                if (wake)
                    WakeBody(entity);
            }

            return;
        }

        var sameSupport = hadSupport &&
                          oldProvider == candidate.Provider &&
                          oldSurface == candidate.Surface &&
                          oldTile == candidate.Tile;
        SetSupportState(
            entity,
            candidate.Provider,
            candidate.Surface,
            candidate.Tile,
            candidate.AbsoluteHeight,
            candidate.ContactPoint);

        if (!_net.IsClient &&
            oldState == ZLevelGroundState.Grounded &&
            sameSupport &&
            candidate.Surface == ZLevelSupportSurface.HighGround)
        {
            NormalizeGroundedSupportMap(entity, candidate.AbsoluteHeight);
        }

        if (!_zPresentationQuery.TryComp(entity.Owner, out var presentation) ||
            Transform(entity).MapUid is not { } currentMap ||
            !_zLevels.TryGetMapDepth(currentMap, out var currentDepth))
        {
            return;
        }

        var currentAbsoluteHeight = ZLevelProjection.GetAbsoluteZ(currentDepth.Value, presentation.LocalHeight);
        if (oldState != ZLevelGroundState.Grounded)
            return;

        var rise = candidate.AbsoluteHeight - currentAbsoluteHeight;
        var maximumSnapDown = MathF.Min(_maxStepDown, _groundSnapDistance);
        if (sameSupport || rise <= _maxStepUp + PositionEpsilon && rise >= -maximumSnapDown - PositionEpsilon)
        {
            SetLocalHeight(
                (entity.Owner, presentation),
                ZLevelProjection.GetLocalHeight(candidate.AbsoluteHeight, currentDepth.Value));
            SetGroundState(
                entity,
                ZLevelGroundState.Grounded,
                sameSupport
                    ? ZLevelReconciliationState.Confirmed
                    : ZLevelReconciliationState.AuthoritativeSupportChange);
            entity.Comp.Velocity = 0f;
            return;
        }

        SetGroundState(entity, ZLevelGroundState.Airborne, ZLevelReconciliationState.AuthoritativeFall);
        if (wake)
            WakeBody(entity);
    }

    /// <summary>
    /// Predicts only the continuous height of an already-confirmed support provider. Candidate changes, walking off,
    /// falls, landings, and map reparenting remain server-authoritative.
    /// </summary>
    private void PredictSameSupportHeight(Entity<ZLevelPhysicsComponent> entity)
    {
        if (entity.Comp.GroundState != ZLevelGroundState.Grounded ||
            entity.Comp.SupportSurface != ZLevelSupportSurface.HighGround ||
            entity.Comp.SupportProvider is not { } providerUid ||
            !_highGroundQuery.TryComp(providerUid, out var provider) ||
            !_xformQuery.TryComp(providerUid, out var providerXform) ||
            !_xformQuery.TryComp(entity.Owner, out var bodyXform) ||
            bodyXform.MapUid is not { } bodyMap ||
            providerXform.MapUid is not { } providerMap ||
            !_zLevels.TryGetMapDepth(bodyMap, out var bodyDepth) ||
            !_zLevels.TryGetMapDepth(providerMap, out var providerDepth) ||
            !_zLevels.TryGetMapDepthOffset(providerMap, bodyMap, out var mapOffset) ||
            Math.Abs(mapOffset) > 1 ||
            _container.IsEntityOrParentInContainer(entity.Owner) ||
            bodyXform.Anchored ||
            (bodyXform.ParentUid != bodyXform.GridUid && bodyXform.ParentUid != bodyXform.MapUid) ||
            !_physicsQuery.TryComp(entity.Owner, out var body) ||
            (body.BodyType & (BodyType.Dynamic | BodyType.KinematicController)) == 0 ||
            !TryGetSupportLocalBounds((providerUid, provider), out var localBounds))
        {
            return;
        }

        var footprint = GetFootprint(entity.Owner, bodyXform);
        if (!FootprintOverlapsProvider(footprint, (providerUid, provider), providerXform))
            return;

        var inverse = _transform.GetInvWorldMatrix(providerXform);
        var localCenter = Vector2.Transform(footprint.Center, inverse);
        var localContact = Vector2.Clamp(localCenter, localBounds.BottomLeft, localBounds.TopRight);
        var absoluteHeight = providerDepth.Value + EvaluateSurfaceHeight(provider, localBounds, localContact);
        if (!float.IsFinite(absoluteHeight) ||
            !_zPresentationQuery.TryComp(entity.Owner, out var presentation))
        {
            return;
        }

        var currentAbsoluteHeight = ZLevelProjection.GetAbsoluteZ(bodyDepth.Value, presentation.LocalHeight);
        var rise = absoluteHeight - currentAbsoluteHeight;
        var maximumSnapDown = MathF.Min(_maxStepDown, _groundSnapDistance);
        if (rise > _maxStepUp + PositionEpsilon || rise < -maximumSnapDown - PositionEpsilon)
            return;

        SetLocalHeight(
            (entity.Owner, presentation),
            ZLevelProjection.GetLocalHeight(absoluteHeight, bodyDepth.Value));
    }

    /// <summary>
    /// Keeps a grounded continuous surface on the nearest canonical map plane. A hysteresis band around each plane
    /// prevents direction reversals at ramp edges from repeatedly reparenting the body.
    /// </summary>
    private void NormalizeGroundedSupportMap(Entity<ZLevelPhysicsComponent> entity, float absoluteHeight)
    {
        if (_supportMapTransitions.Contains(entity.Owner) ||
            Transform(entity).MapUid is not { } currentMap ||
            !_zLevels.TryGetMapDepth(currentMap, out var currentDepth))
        {
            return;
        }

        var offset = 0;
        if (absoluteHeight >= currentDepth.Value + 1f - PositionEpsilon)
            offset = 1;
        else if (absoluteHeight < currentDepth.Value - _supportHysteresis - PositionEpsilon)
            offset = -1;

        if (offset == 0)
            return;

        _supportMapTransitions.Add(entity.Owner);
        try
        {
            TryMove(entity.Owner, offset);
        }
        finally
        {
            _supportMapTransitions.Remove(entity.Owner);
        }
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

    private bool TrySelectSupport(Entity<ZLevelPhysicsComponent> entity, out SupportCandidate selected)
    {
        selected = default;
        _supportCandidates.Clear();
        entity.Comp.LastSupportCandidates.Clear();

        if (!_xformQuery.TryComp(entity.Owner, out var xform) ||
            xform.MapUid is not { } currentMap ||
            !_zMapQuery.TryComp(currentMap, out var currentZMap) ||
            !_mapQuery.TryComp(currentMap, out _) ||
            !_zPresentationQuery.TryComp(entity.Owner, out var presentation))
        {
            return false;
        }

        var footprint = GetFootprint(entity.Owner, xform);
        var currentAbsoluteHeight = ZLevelProjection.GetAbsoluteZ(currentZMap.Depth, presentation.LocalHeight);

        // One adjacent level is enough: farther floors are reconsidered after each authoritative map transition.
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
                entity,
                checkingMap,
                checkingMapComp.MapId,
                checkingZMap.Depth,
                footprint,
                currentAbsoluteHeight);
            CollectTileCandidates(
                entity,
                checkingMapComp.MapId,
                checkingZMap.Depth,
                footprint,
                currentAbsoluteHeight);
        }

        if (_supportCandidates.Count == 0)
            return false;

        _supportCandidates.Sort(CompareSupportCandidates);
        selected = _supportCandidates[0];

        if (entity.Comp.SupportProvider is { } currentProvider)
        {
            foreach (var candidate in _supportCandidates)
            {
                if (candidate.Provider != currentProvider ||
                    candidate.Surface != entity.Comp.SupportSurface ||
                    candidate.Tile != entity.Comp.SupportTile)
                {
                    continue;
                }

                if (selected.AbsoluteHeight - candidate.AbsoluteHeight <= _supportHysteresis)
                    selected = candidate;
                break;
            }
        }

        for (var i = 0; i < entity.Comp.LastSupportCandidates.Count; i++)
        {
            var diagnostic = entity.Comp.LastSupportCandidates[i];
            if (diagnostic.Rejection != ZLevelSupportRejection.None ||
                diagnostic.Provider == selected.Provider &&
                diagnostic.Surface == selected.Surface &&
                diagnostic.Tile == selected.Tile)
            {
                continue;
            }

            entity.Comp.LastSupportCandidates[i] = diagnostic with { Rejection = ZLevelSupportRejection.Occluded };
        }

        return true;
    }

    internal void RefreshSupportsOnMovedGrid(EntityUid grid)
    {
        var query = EntityQueryEnumerator<ZLevelPhysicsComponent, TransformComponent>();
        while (query.MoveNext(out var uid, out var physics, out var xform))
        {
            if (physics.GroundState != ZLevelGroundState.Grounded)
                continue;

            var providerOnGrid = physics.SupportProvider is { } provider &&
                                 _xformQuery.TryComp(provider, out var providerXform) &&
                                 providerXform.GridUid == grid;
            if (xform.GridUid != grid && !providerOnGrid)
                continue;

            RefreshSupport((uid, physics), wake: false);
        }
    }

    private void CollectHighGroundCandidates(
        Entity<ZLevelPhysicsComponent> body,
        EntityUid checkingMap,
        MapId mapId,
        int mapDepth,
        in BodyFootprint footprint,
        float currentAbsoluteHeight)
    {
        _supportProviders.Clear();
        _lookup.GetEntitiesIntersecting(
            mapId,
            footprint.Bounds,
            _supportProviders,
            LookupFlags.Static | LookupFlags.Sundries | LookupFlags.Sensors);

        foreach (var provider in _supportProviders.OrderBy(entry => entry.Owner))
        {
            if (provider.Owner == body.Owner)
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, 0f, default,
                    ZLevelSupportRejection.Self, body.Comp);
                continue;
            }

            if (!_xformQuery.TryComp(provider.Owner, out var providerXform) || providerXform.MapUid != checkingMap)
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, 0f, default,
                    ZLevelSupportRejection.WrongMap, body.Comp);
                continue;
            }

            if (provider.Comp.HeightCurve.Count == 0)
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, 0f, default,
                    ZLevelSupportRejection.NoSurface, body.Comp);
                continue;
            }

            if (!TryGetSupportLocalBounds(provider, out var localBounds))
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, 0f, default,
                    ZLevelSupportRejection.NoFixture, body.Comp);
                continue;
            }

            if (!FootprintOverlapsProvider(footprint, provider, providerXform))
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, 0f, default,
                    ZLevelSupportRejection.OutsideFootprint, body.Comp);
                continue;
            }

            var inverse = _transform.GetInvWorldMatrix(providerXform);
            var localCenter = Vector2.Transform(footprint.Center, inverse);
            var localContact = Vector2.Clamp(localCenter, localBounds.BottomLeft, localBounds.TopRight);
            var contactPoint = Vector2.Transform(localContact, _transform.GetWorldMatrix(providerXform));
            var surfaceHeight = EvaluateSurfaceHeight(provider.Comp, localBounds, localContact);
            var absoluteHeight = mapDepth + surfaceHeight;
            if (!float.IsFinite(absoluteHeight))
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, absoluteHeight,
                    contactPoint, ZLevelSupportRejection.NonFiniteHeight, body.Comp);
                continue;
            }

            var allowedRise = body.Comp.GroundState == ZLevelGroundState.Grounded && body.Comp.AutoStep
                ? _maxStepUp
                : PositionEpsilon;
            if (absoluteHeight > currentAbsoluteHeight + allowedRise + PositionEpsilon)
            {
                AddSupportDiagnostic(provider.Owner, ZLevelSupportSurface.HighGround, default, absoluteHeight,
                    contactPoint, ZLevelSupportRejection.AboveStepLimit, body.Comp);
                continue;
            }

            var candidate = new SupportCandidate(
                provider.Owner,
                ZLevelSupportSurface.HighGround,
                default,
                absoluteHeight,
                contactPoint,
                provider.Comp.Priority);
            _supportCandidates.Add(candidate);
            AddSupportDiagnostic(candidate, ZLevelSupportRejection.None, body.Comp);
        }
    }

    private void CollectTileCandidates(
        Entity<ZLevelPhysicsComponent> body,
        MapId mapId,
        int mapDepth,
        in BodyFootprint footprint,
        float currentAbsoluteHeight)
    {
        if (mapDepth > currentAbsoluteHeight + PositionEpsilon)
            return;

        _supportGrids.Clear();
        _map.FindGridsIntersecting(mapId, footprint.Bounds, ref _supportGrids, approx: false, includeMap: true);
        _supportGrids.Sort((left, right) => left.Owner.CompareTo(right.Owner));

        EntityUid previous = default;
        foreach (var grid in _supportGrids)
        {
            if (grid.Owner == previous)
                continue;
            previous = grid.Owner;

            var inverse = _transform.GetInvWorldMatrix(grid.Owner);
            var matrix = _transform.GetWorldMatrix(grid.Owner);
            var localBounds = inverse.TransformBox(footprint.Bounds);
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
                    if (!FootprintOverlapsShape(footprint, tileShape, PhysicsTransform.Empty, tileWorldBounds))
                        continue;

                    var localCenter = Vector2.Transform(footprint.Center, inverse);
                    var localContact = Vector2.Clamp(localCenter, tileBounds.BottomLeft, tileBounds.TopRight);
                    var contactPoint = Vector2.Transform(localContact, matrix);
                    var candidate = new SupportCandidate(
                        grid.Owner,
                        ZLevelSupportSurface.Tile,
                        tileIndex,
                        mapDepth,
                        contactPoint,
                        0);
                    _supportCandidates.Add(candidate);
                    AddSupportDiagnostic(candidate, ZLevelSupportRejection.None, body.Comp);
                }
            }
        }
    }

    private BodyFootprint GetFootprint(EntityUid uid, TransformComponent xform)
    {
        var center = _transform.GetWorldPosition(xform);
        var bounds = Box2.CenteredAround(center, new Vector2(DefaultFootprintRadius * 2f));
        var hasFixtures = false;
        var physicsTransform = _physics.GetPhysicsTransform(uid, xform);

        if (_fixturesQuery.TryComp(uid, out var fixtures))
        {
            foreach (var (_, fixture) in fixtures.Fixtures.OrderBy(entry => entry.Key, StringComparer.Ordinal))
            {
                if (!fixture.Hard)
                    continue;

                for (var child = 0; child < fixture.Shape.ChildCount; child++)
                {
                    var fixtureBounds = fixture.Shape.ComputeAABB(physicsTransform, child);
                    bounds = hasFixtures ? Union(bounds, fixtureBounds) : fixtureBounds;
                    hasFixtures = true;
                }
            }
        }

        return new BodyFootprint(uid, center, bounds, physicsTransform, hasFixtures);
    }

    private bool FootprintOverlapsProvider(
        in BodyFootprint footprint,
        Entity<ZLevelHighGroundComponent> provider,
        TransformComponent providerXform)
    {
        if (!_fixturesQuery.TryComp(provider.Owner, out var providerFixtures))
            return false;

        var providerTransform = _physics.GetPhysicsTransform(provider.Owner, providerXform);
        foreach (var (bodyId, bodyFixture) in EnumerateFootprintFixtures(footprint))
        {
            foreach (var (providerId, providerFixture) in providerFixtures.Fixtures.OrderBy(entry => entry.Key, StringComparer.Ordinal))
            {
                if (provider.Comp.SurfaceFixture.Length != 0 && provider.Comp.SurfaceFixture != providerId)
                    continue;

                for (var bodyChild = 0; bodyChild < bodyFixture.Shape.ChildCount; bodyChild++)
                {
                    for (var providerChild = 0; providerChild < providerFixture.Shape.ChildCount; providerChild++)
                    {
                        if (_manifold.TestOverlap(
                                bodyFixture.Shape,
                                bodyChild,
                                providerFixture.Shape,
                                providerChild,
                                footprint.Transform,
                                providerTransform,
                                ignoreShapeSkin: true))
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return !footprint.HasFixtures && _lookup.GetWorldAABB(provider.Owner, providerXform).Intersects(footprint.Bounds);
    }

    private bool FootprintOverlapsShape<TShape>(
        in BodyFootprint footprint,
        TShape shape,
        PhysicsTransform shapeTransform,
        Box2 shapeBounds)
        where TShape : Robust.Shared.Physics.Collision.Shapes.IPhysShape
    {
        foreach (var (_, bodyFixture) in EnumerateFootprintFixtures(footprint))
        {
            for (var child = 0; child < bodyFixture.Shape.ChildCount; child++)
            {
                if (_manifold.TestOverlap(
                        bodyFixture.Shape,
                        child,
                        shape,
                        0,
                        footprint.Transform,
                        shapeTransform,
                        ignoreShapeSkin: true))
                {
                    return true;
                }
            }
        }

        return !footprint.HasFixtures && footprint.Bounds.Intersects(shapeBounds);
    }

    private IEnumerable<KeyValuePair<string, Fixture>> EnumerateFootprintFixtures(in BodyFootprint footprint)
    {
        if (!footprint.HasFixtures || !_fixturesQuery.TryComp(footprint.Owner, out var fixtures))
            return Array.Empty<KeyValuePair<string, Fixture>>();

        return fixtures.Fixtures
            .Where(entry => entry.Value.Hard)
            .OrderBy(entry => entry.Key, StringComparer.Ordinal);
    }

    /// <summary>
    /// Gets the authored fixture bounds of a provider's walkable surface in provider-local coordinates.
    /// </summary>
    public bool TryGetSupportLocalBounds(Entity<ZLevelHighGroundComponent> provider, out Box2 bounds)
    {
        bounds = default;
        if (!_fixturesQuery.TryComp(provider.Owner, out var fixtures))
            return false;

        var found = false;
        foreach (var (id, fixture) in fixtures.Fixtures.OrderBy(entry => entry.Key, StringComparer.Ordinal))
        {
            if (provider.Comp.SurfaceFixture.Length != 0 && provider.Comp.SurfaceFixture != id)
                continue;

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
        }

        return found;
    }

    /// <summary>
    /// Gets the projected-top source polygon as canonical XY/absolute-Z samples for content presentation.
    /// </summary>
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
            !_zLevels.TryGetMapDepth(map, out var depth))
        {
            return false;
        }

        var matrix = _transform.GetWorldMatrix(xform);
        Span<Vector2> local = stackalloc Vector2[4]
        {
            bounds.BottomLeft,
            bounds.BottomRight,
            bounds.TopRight,
            bounds.TopLeft,
        };

        for (var i = 0; i < local.Length; i++)
        {
            points[i] = new ZLevelSupportPoint(
                Vector2.Transform(local[i], matrix),
                depth.Value + EvaluateSurfaceHeight(provider.Comp, bounds, local[i]));
        }

        count = 4;
        return true;
    }

    private static float EvaluateSurfaceHeight(ZLevelHighGroundComponent component, Box2 bounds, Vector2 localPoint)
    {
        if (component.HeightCurve.Count == 0)
            return float.NaN;

        var width = MathF.Max(bounds.Width, PositionEpsilon);
        var height = MathF.Max(bounds.Height, PositionEpsilon);
        var u = Math.Clamp((localPoint.X - bounds.Left) / width, 0f, 1f);
        var v = Math.Clamp((localPoint.Y - bounds.Bottom) / height, 0f, 1f);
        // Angle zero is south in Robust. Evaluating in provider-local space makes rotation and moving grids exact.
        var t = component.Corner ? (1f - u + 1f - v) * 0.5f : 1f - v;
        return InterpolateHeight(component.HeightCurve, Math.Clamp(t, 0f, 1f));
    }

    private static float InterpolateHeight(IReadOnlyList<float> curve, float t)
    {
        if (curve.Count == 1)
            return curve[0];

        var scaled = t * (curve.Count - 1);
        var index = Math.Min((int) scaled, curve.Count - 2);
        return MathHelper.Lerp(curve[index], curve[index + 1], scaled - index);
    }

    private void SetSupportState(
        Entity<ZLevelPhysicsComponent> entity,
        EntityUid? provider,
        ZLevelSupportSurface surface,
        Vector2i tile,
        float absoluteHeight,
        Vector2 contactPoint)
    {
        if (entity.Comp.SupportProvider == provider &&
            entity.Comp.SupportSurface == surface &&
            entity.Comp.SupportTile == tile &&
            entity.Comp.SupportHeight.Equals(absoluteHeight) &&
            entity.Comp.SupportPoint.Equals(contactPoint))
        {
            return;
        }

        entity.Comp.SupportProvider = provider;
        entity.Comp.SupportSurface = surface;
        entity.Comp.SupportTile = tile;
        entity.Comp.SupportHeight = absoluteHeight;
        entity.Comp.SupportPoint = contactPoint;
        DirtyFields(
            entity.Owner,
            entity.Comp,
            null,
            nameof(ZLevelPhysicsComponent.SupportProvider),
            nameof(ZLevelPhysicsComponent.SupportSurface),
            nameof(ZLevelPhysicsComponent.SupportTile),
            nameof(ZLevelPhysicsComponent.SupportHeight),
            nameof(ZLevelPhysicsComponent.SupportPoint));
    }

    private void SetGroundState(
        Entity<ZLevelPhysicsComponent> entity,
        ZLevelGroundState state,
        ZLevelReconciliationState reconciliation)
    {
        if (entity.Comp.GroundState == state && entity.Comp.ReconciliationState == reconciliation)
            return;

        entity.Comp.GroundState = state;
        entity.Comp.ReconciliationState = reconciliation;
        DirtyFields(
            entity.Owner,
            entity.Comp,
            null,
            nameof(ZLevelPhysicsComponent.GroundState),
            nameof(ZLevelPhysicsComponent.ReconciliationState));
    }

    private void AddSupportDiagnostic(
        SupportCandidate candidate,
        ZLevelSupportRejection rejection,
        ZLevelPhysicsComponent component)
        => AddSupportDiagnostic(
            candidate.Provider,
            candidate.Surface,
            candidate.Tile,
            candidate.AbsoluteHeight,
            candidate.ContactPoint,
            rejection,
            component);

    private static void AddSupportDiagnostic(
        EntityUid provider,
        ZLevelSupportSurface surface,
        Vector2i tile,
        float height,
        Vector2 point,
        ZLevelSupportRejection rejection,
        ZLevelPhysicsComponent component)
    {
        if (component.LastSupportCandidates.Count >= MaxSupportDiagnostics)
            return;

        component.LastSupportCandidates.Add(new(provider, surface, tile, height, point, rejection));
    }

    private static int CompareSupportCandidates(SupportCandidate left, SupportCandidate right)
    {
        var comparison = right.AbsoluteHeight.CompareTo(left.AbsoluteHeight);
        if (comparison != 0)
            return comparison;

        comparison = right.Priority.CompareTo(left.Priority);
        if (comparison != 0)
            return comparison;

        comparison = left.Surface.CompareTo(right.Surface);
        if (comparison != 0)
            return comparison;

        comparison = left.Provider.CompareTo(right.Provider);
        if (comparison != 0)
            return comparison;

        comparison = left.Tile.X.CompareTo(right.Tile.X);
        return comparison != 0 ? comparison : left.Tile.Y.CompareTo(right.Tile.Y);
    }

    private static Box2 Union(Box2 left, Box2 right)
        => new(Vector2.Min(left.BottomLeft, right.BottomLeft), Vector2.Max(left.TopRight, right.TopRight));

    private readonly record struct BodyFootprint(
        EntityUid Owner,
        Vector2 Center,
        Box2 Bounds,
        PhysicsTransform Transform,
        bool HasFixtures);

    private readonly record struct SupportCandidate(
        EntityUid Provider,
        ZLevelSupportSurface Surface,
        Vector2i Tile,
        float AbsoluteHeight,
        Vector2 ContactPoint,
        int Priority);
}

public readonly record struct ZLevelSupportPoint(Vector2 CanonicalPosition, float AbsoluteHeight);
