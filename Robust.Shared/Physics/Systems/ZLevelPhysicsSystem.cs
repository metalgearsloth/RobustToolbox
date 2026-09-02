using System;
using System.Collections.Generic;
using System.Numerics;
using JetBrains.Annotations;
using Robust.Shared.Analyzers;
using Robust.Shared.Containers;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Network;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Events;

namespace Robust.Shared.Physics.Systems;

/// <summary>
/// Simulates continuous vertical motion across linked z-level maps.
/// </summary>
public sealed partial class ZLevelPhysicsSystem : SharedZLevelPresentationSystem
{
    public const float DefaultGravity = 9.8f;
    public const float DefaultVelocityLimit = 20f;
    public const float DefaultImpactVelocity = 3.5f;
    public const float DefaultAirborneHeight = 0.15f;
    public const float DefaultMaxStepUp = 0.25f;
    public const float DefaultMaxStepDown = 0.25f;
    public const float DefaultGroundSnapDistance = 0.25f;

    private const int MaxStepsPerUpdate = 240;
    private const int MaxEventsPerStep = 256;
    private const float TimeEpsilon = 0.000001f;
    private const float PositionEpsilon = 0.00001f;
    private const float VelocityEpsilon = 0.00001f;

    [Dependency] private IConfigurationManager _configuration = null!;
    [Dependency] private SharedContainerSystem _container = null!;
    [Dependency] private EntityLookupSystem _lookup = null!;
    [Dependency] private INetManager _net = null!;
    [Dependency] private SharedMapSystem _map = null!;
    [Dependency] private SharedPhysicsSystem _physics = null!;
    [Dependency] private SharedTransformSystem _transform = null!;
    [Dependency] private ZLevelSystem _zLevels = null!;

    [Dependency] private EntityQuery<PhysicsComponent> _physicsQuery;
    [Dependency] private EntityQuery<TransformComponent> _xformQuery;
    [Dependency] private EntityQuery<MapComponent> _mapQuery;
    [Dependency] private EntityQuery<ZLevelHighGroundComponent> _highGroundQuery;
    [Dependency] private EntityQuery<ZLevelMapComponent> _zMapQuery;
    [Dependency] private EntityQuery<ZLevelPresentationComponent> _zPresentationQuery;
    [Dependency] private EntityQuery<ZLevelPhysicsComponent> _zPhysicsQuery;

    private readonly List<EntityUid> _activeBodies = new();
    private readonly HashSet<EntityUid> _activeBodySet = new();
    private readonly HashSet<EntityUid> _dirtyMovementBodies = new();
    private readonly HashSet<EntityUid> _pendingBodyRefresh = new();
    private readonly HashSet<Entity<ZLevelPhysicsComponent>> _nearbyBodies = new();

    private TimeSpan _fixedTimestep;
    private TimeSpan _accumulatedTime = TimeSpan.Zero;
    private float _gravity = DefaultGravity;
    private float _velocityLimit = DefaultVelocityLimit;
    private float _impactVelocity = DefaultImpactVelocity;
    private float _airborneHeight = DefaultAirborneHeight;
    private float _maxStepUp = DefaultMaxStepUp;
    private float _maxStepDown = DefaultMaxStepDown;
    private float _groundSnapDistance = DefaultGroundSnapDistance;

    /// <summary>
    /// Bodies currently participating in vertical simulation.
    /// </summary>
    public IReadOnlyList<EntityUid> ActiveBodies => _activeBodies;

    /// <summary>
    /// Number of bodies processed during the most recent update.
    /// </summary>
    public int UpdateCalls { get; private set; }

    /// <summary>
    /// Current configured vertical gravity acceleration.
    /// </summary>
    public float Gravity => _gravity;

    /// <summary>
    /// Current absolute vertical speed limit.
    /// </summary>
    public float VelocityLimit => _velocityLimit;

    /// <summary>
    /// Minimum vertical speed needed for an impact event.
    /// </summary>
    public float ImpactVelocity => _impactVelocity;

    /// <summary>
    /// Distance above support where a body counts as airborne.
    /// </summary>
    public float AirborneHeight => _airborneHeight;

    public float MaxStepUp => _maxStepUp;
    public float MaxStepDown => _maxStepDown;
    public float GroundSnapDistance => _groundSnapDistance;

    /// <inheritdoc/>
    public override void Initialize()
    {
        base.Initialize();

        UpdatesAfter.Add(typeof(SharedPhysicsSystem));

        Subs.CVar(_configuration, CVars.PhysicsZLevelTickRate, SetPhysicsTickRate, true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelGravity, value => _gravity = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelVelocityLimit, value => _velocityLimit = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelImpactVelocity, value => _impactVelocity = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelAirborneHeight, value => _airborneHeight = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelMaxStepUp, value => _maxStepUp = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelMaxStepDown, value => _maxStepDown = MathF.Max(0f, value), true);
        Subs.CVar(_configuration, CVars.PhysicsZLevelGroundSnapDistance, value => _groundSnapDistance = MathF.Max(0f, value), true);

        SubscribeLocalEvent<ZLevelPhysicsComponent, EntGotInsertedIntoContainerMessage>(OnInsertedIntoContainer);
        SubscribeLocalEvent<ZLevelPhysicsComponent, EntGotRemovedFromContainerMessage>(OnRemovedFromContainer);
        SubscribeLocalEvent<ZLevelPhysicsComponent, PreventCollideEvent>(OnPreventCollide);
        SubscribeLocalEvent<MapGridComponent, MapGridMovedEvent>(OnSupportGridMoved);
    }

    private void OnSupportGridMoved(Entity<MapGridComponent> grid, ref MapGridMovedEvent args)
        => RefreshSupportsOnMovedGrid(grid.Owner);

    private void SetPhysicsTickRate(int tickRate)
    {
        tickRate = Math.Max(1, tickRate);
        _fixedTimestep = TimeSpan.FromSeconds(1d / tickRate);
        _accumulatedTime = TimeSpan.Zero;
    }

    [SubscribeLocalEvent]
    private void OnStartup(Entity<ZLevelPhysicsComponent> entity, ref ComponentStartup args)
    {
        EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        InitializeBody(entity);
    }

    [SubscribeLocalEvent]
    private void OnMapInit(Entity<ZLevelPhysicsComponent> entity, ref MapInitEvent args)
    {
        InitializeBody(entity);
    }

    [SubscribeLocalEvent]
    private void OnAfterHandleState(Entity<ZLevelPhysicsComponent> entity, ref AfterAutoHandleStateEvent args)
    {
        if (_net.IsClient)
            return;

        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            _pendingBodyRefresh.Remove(entity.Owner);
            return;
        }

        RefreshSupport(entity);
        RefreshBody(entity);

        if (!_activeBodySet.Contains(entity.Owner) && HasActiveZState(entity.Comp))
            _pendingBodyRefresh.Add(entity.Owner);

    }

    [SubscribeLocalEvent]
    private void OnShutdown(Entity<ZLevelPhysicsComponent> entity, ref ComponentShutdown args)
    {
        ClearSupportTracking(entity);
        SleepBody(entity);
        _dirtyMovementBodies.Remove(entity.Owner);
        _pendingBodyRefresh.Remove(entity.Owner);
    }

    [SubscribeLocalEvent]
    private void OnAnchorStateChanged(Entity<ZLevelPhysicsComponent> entity, ref AnchorStateChangedEvent args)
    {
        InitializeBody(entity);
    }

    [SubscribeLocalEvent]
    private void OnPhysicsBodyTypeChanged(Entity<ZLevelPhysicsComponent> entity, ref PhysicsBodyTypeChangedEvent args)
    {
        InitializeBody(entity);
    }

    [SubscribeLocalEvent]
    private void OnParentChanged(Entity<ZLevelPhysicsComponent> entity, ref EntParentChangedMessage args)
    {
        // Contained entities follow their container owner's z pose. They must never preserve or simulate an
        // independent absolute height when a containing mob, hand, inventory, or storage owner changes maps.
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            _dirtyMovementBodies.Remove(entity.Owner);
            _pendingBodyRefresh.Remove(entity.Owner);
            return;
        }

        // Local height is relative to the current map. Re-express it against the new map depth so a reparent does
        // not change absolute z. This is also the normalization step for physics-driven map crossings.
        if (args.OldMapId is { } oldMap &&
            args.Transform.MapUid is { } newMap &&
            oldMap != newMap &&
            _zLevels.TryGetMapDepth(oldMap, out var oldDepth) &&
            _zLevels.TryGetMapDepth(newMap, out var newDepth) &&
            _zPresentationQuery.TryComp(entity.Owner, out var presentation))
        {
            var absoluteZ = ZLevelProjection.GetAbsoluteZ(oldDepth.Value, presentation.LocalHeight);
            SetLocalHeight((entity.Owner, presentation), ZLevelProjection.GetLocalHeight(absoluteZ, newDepth.Value));
        }

        _pendingBodyRefresh.Add(entity.Owner);
        RefreshBody(entity);
    }

    private void OnInsertedIntoContainer(
        EntityUid uid,
        ZLevelPhysicsComponent component,
        EntGotInsertedIntoContainerMessage args)
    {
        Entity<ZLevelPhysicsComponent> entity = (uid, component);
        // Contained entities inherit the carrier's vertical pose and never simulate independently. Capturing that
        // pose also prevents an old airborne/high-ground height from resurfacing the next time the item is dropped.
        CopyCarrierHeight(entity, args.Container.Owner);
        ResetContainedMotion(entity);
        SleepBody(entity);
        _dirtyMovementBodies.Remove(entity.Owner);
        _pendingBodyRefresh.Remove(entity.Owner);
    }

    private void OnRemovedFromContainer(
        EntityUid uid,
        ZLevelPhysicsComponent component,
        EntGotRemovedFromContainerMessage args)
    {
        Entity<ZLevelPhysicsComponent> entity = (uid, component);
        // Parent-change notification is raised before the container removal notification. Re-sample the outermost
        // carrier here so the newly released body starts at the carrier's current absolute z. The client performs
        // this deterministic copy for ordinary hands prediction, but vertical simulation remains server-only.
        CopyCarrierHeight(entity, args.Container.Owner);

        if (_net.IsClient)
            return;

        // A hand/storage removal that places the item directly on support is initialization, not a fall. Mark it
        // grounded immediately so the first gravity tick cannot manufacture a zero-distance landing event.
        InitializeBody(entity);
    }

    private void CopyCarrierHeight(Entity<ZLevelPhysicsComponent> entity, EntityUid containerOwner)
    {
        if (!_xformQuery.TryComp(entity.Owner, out var entityXform) ||
            entityXform.MapUid is not { } entityMap ||
            !_zLevels.TryGetMapDepth(entityMap, out var entityDepth))
        {
            return;
        }

        ZLevelPresentationComponent? carrierPresentation = null;
        EntityUid carrierMap = EntityUid.Invalid;
        var current = containerOwner;
        for (var depth = 0; depth < 128 && _xformQuery.TryComp(current, out var currentXform); depth++)
        {
            if (_zPresentationQuery.TryComp(current, out var presentation))
            {
                carrierPresentation = presentation;
                carrierMap = currentXform.MapUid ?? EntityUid.Invalid;
            }

            if (!currentXform.ParentUid.IsValid() || currentXform.ParentUid == currentXform.MapUid)
                break;

            current = currentXform.ParentUid;
        }

        if (carrierPresentation == null ||
            !carrierMap.IsValid() ||
            !_zLevels.TryGetMapDepth(carrierMap, out var carrierDepth))
        {
            return;
        }

        var absoluteZ = ZLevelProjection.GetAbsoluteZ(carrierDepth.Value, carrierPresentation.LocalHeight);
        var itemPresentation = EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        SetLocalHeight((entity.Owner, itemPresentation), ZLevelProjection.GetLocalHeight(absoluteZ, entityDepth.Value));
    }

    private void ResetContainedMotion(Entity<ZLevelPhysicsComponent> entity)
    {
        ApplySupportResult(entity, ZLevelSupportResult.None);
        SetGroundState(entity, ZLevelGroundState.Airborne);
        entity.Comp.LastStep = default;
        if (entity.Comp.Velocity.Equals(0f))
            return;

        entity.Comp.Velocity = 0f;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPhysicsComponent.Velocity));
    }

    [SubscribeLocalEvent]
    private void OnMove(Entity<ZLevelPhysicsComponent> entity, ref MoveEvent args)
    {
        if (_net.IsClient)
            return;

        if (!CanSimulateBody(entity))
            return;

        _dirtyMovementBodies.Add(entity.Owner);
        WakeBody(entity);
    }

    [SubscribeLocalEvent]
    private void OnTileChanged(Entity<MapGridComponent> grid, ref TileChangedEvent args)
    {
        var mapUid = Transform(grid).MapUid;
        if (mapUid == null)
            return;

        foreach (var change in args.Changes)
        {
            var worldPosition = _map.GridTileToWorldPos(grid.Owner, grid.Comp, change.GridIndices);
            RefreshBodiesNearMapPosition(mapUid.Value, worldPosition, grid.Comp.TileSize);

        }
    }

    [SubscribeLocalEvent]
    private void OnHighGroundChanged(Entity<ZLevelHighGroundComponent> entity, ref ComponentStartup args)
    {
        RefreshBodiesAtHighGround(entity.Owner);
    }

    [SubscribeLocalEvent]
    private void OnZLevelMapStartup(Entity<ZLevelMapComponent> map, ref ComponentStartup args)
    {
        var query = EntityQueryEnumerator<ZLevelPhysicsComponent, TransformComponent>();
        while (query.MoveNext(out var uid, out var zPhysics, out var xform))
        {
            if (xform.MapUid != map.Owner || !CanSimulateBody((uid, zPhysics), xform: xform))
                continue;

            InitializeBody((uid, zPhysics));
        }
    }

    [SubscribeLocalEvent]
    private void OnHighGroundChanged(Entity<ZLevelHighGroundComponent> entity, ref ComponentShutdown args)
    {
        RefreshBodiesAtHighGround(entity.Owner);
    }

    private void RefreshBodiesAtHighGround(EntityUid highGround)
    {
        RefreshSupportedBodiesAtProvider(highGround);

        if (!_xformQuery.TryComp(highGround, out var groundXform) || groundXform.MapUid is not { } mapUid)
            return;

        var worldPosition = _transform.GetWorldPosition(groundXform);
        RefreshBodiesNearMapPosition(mapUid, worldPosition, 2f);

        if (_zLevels.TryGetMapAbove(mapUid, out var aboveMap) &&
            aboveMap is { } aboveMapUid)
            RefreshBodiesNearMapPosition(aboveMapUid, worldPosition, 2f);
    }

    private void RefreshBodiesNearMapPosition(EntityUid mapUid, Vector2 worldPosition, float range)
    {
        if (!_mapQuery.TryComp(mapUid, out var map))
            return;

        _nearbyBodies.Clear();
        _lookup.GetEntitiesInRange(map.MapId, worldPosition, range, _nearbyBodies);

        foreach (var body in _nearbyBodies)
        {
            if (!_xformQuery.TryComp(body.Owner, out var xform) ||
                xform.MapUid != mapUid ||
                !CanSimulateBody(body, xform: xform))
                continue;

            RefreshSupport(body);
            WakeBody(body);
        }
    }

    /// <inheritdoc/>
    public override void Update(float frameTime)
    {
        base.Update(frameTime);

        UpdateCalls = 0;
        // Vertical motion, map transitions, impacts, and landing are authoritative. Clients retain ordinary XY
        // prediction and interpolate received LocalHeight/map changes through the render-pose system.
        if (_net.IsClient || frameTime <= 0f)
            return;

        _accumulatedTime += TimeSpan.FromSeconds(frameTime);

        var steps = 0;
        while (_accumulatedTime >= _fixedTimestep && steps < MaxStepsPerUpdate)
        {
            UpdateZPhysics((float) _fixedTimestep.TotalSeconds);
            _accumulatedTime -= _fixedTimestep;
            steps++;
        }
    }

    private void UpdateZPhysics(float frameTime)
    {
        UpdatePendingBodyRefreshes();
        UpdateDirtyMovement();

        for (var i = _activeBodies.Count - 1; i >= 0; i--)
        {
            var uid = _activeBodies[i];
            if (!_zPhysicsQuery.TryComp(uid, out var zPhysics) ||
                !_physicsQuery.TryComp(uid, out var physics) ||
                !_xformQuery.TryComp(uid, out var xform) ||
                !_zMapQuery.HasComp(xform.MapUid) ||
                !CanSimulateBody((uid, zPhysics), physics, xform))
            {
                RemoveActiveAt(i, uid);
                continue;
            }

            if (!_zPresentationQuery.TryComp(uid, out var presentation))
            {
                presentation = EnsureComp<ZLevelPresentationComponent>(uid);
            }

            ProcessZPhysics((uid, zPhysics, physics), presentation, frameTime);
        }
    }

    private void ProcessZPhysics(
        Entity<ZLevelPhysicsComponent, PhysicsComponent> entity,
        ZLevelPresentationComponent presentation,
        float frameTime)
    {
        var zPhysics = entity.Comp1;
        UpdateCalls++;

        var oldVelocity = zPhysics.Velocity;
        var acceleration = zPhysics.VelocityGravity
            ? -_gravity * zPhysics.GravityMultiplier
            : 0f;

        if (zPhysics.VelocityRaiseEvent)
        {
            var velocityEvent = new ZLevelGetVelocityEvent(entity.Owner, zPhysics);
            RaiseLocalEvent(entity.Owner, ref velocityEvent);
            acceleration += velocityEvent.VelocityDelta;
        }

        zPhysics.Velocity = Math.Clamp(zPhysics.Velocity, -_velocityLimit, _velocityLimit);

        var remaining = frameTime;
        var crossings = 0;
        var events = 0;
        var nextEvent = ZLevelStepEvent.None;
        var limitReached = false;

        var localSupportHeight = GetLocalSupportHeight((entity.Owner, zPhysics));
        var initialGroundDistance = zPhysics.SupportSurface == ZLevelSupportSurface.None
            ? float.PositiveInfinity
            : presentation.LocalHeight - localSupportHeight;
        if (zPhysics.GroundState == ZLevelGroundState.Grounded &&
            MathF.Abs(initialGroundDistance) <= PositionEpsilon &&
            zPhysics.Velocity <= VelocityEpsilon &&
            acceleration <= 0f)
        {
            zPhysics.Velocity = 0f;
            remaining = 0f;
        }
        else if (initialGroundDistance > PositionEpsilon || zPhysics.Velocity > VelocityEpsilon)
        {
            SetGroundState((entity.Owner, zPhysics), ZLevelGroundState.Airborne);
        }

        while (remaining > TimeEpsilon)
        {
            if (events >= MaxEventsPerStep)
            {
                limitReached = true;
                break;
            }

            var segment = GetMotionSegment(zPhysics.Velocity, acceleration, remaining);
            var motionEvent = FindNextEvent(entity.Owner, zPhysics, presentation.LocalHeight, segment);
            nextEvent = motionEvent.Type;

            if (motionEvent.Time > segment.Duration + TimeEpsilon)
            {
                AdvanceMotion(zPhysics, (entity.Owner, presentation), segment.Duration, segment.Acceleration);
                remaining -= segment.Duration;

                if (segment.EndEvent != ZLevelStepEvent.None)
                {
                    events++;
                    nextEvent = segment.EndEvent;
                    if (segment.EndEvent == ZLevelStepEvent.TerminalVelocity)
                        zPhysics.Velocity = MathF.Sign(zPhysics.Velocity) * _velocityLimit;
                    else if (MathF.Abs(zPhysics.Velocity) <= VelocityEpsilon)
                        zPhysics.Velocity = 0f;
                }

                continue;
            }

            var eventTime = Math.Clamp(motionEvent.Time, 0f, segment.Duration);
            AdvanceMotion(zPhysics, (entity.Owner, presentation), eventTime, segment.Acceleration);
            remaining -= eventTime;
            SetLocalHeight((entity.Owner, presentation), motionEvent.Position);
            events++;

            switch (motionEvent.Type)
            {
                case ZLevelStepEvent.Floor:
                    ResolveImpact(entity.Owner, zPhysics, presentation, ZLevelImpactSurface.Floor);
                    if (zPhysics.Velocity <= VelocityEpsilon && acceleration <= 0f)
                    {
                        zPhysics.Velocity = 0f;
                        remaining = 0f;
                    }
                    break;
                case ZLevelStepEvent.Ceiling:
                    ResolveImpact(entity.Owner, zPhysics, presentation, ZLevelImpactSurface.Ceiling);
                    if (zPhysics.Velocity >= -VelocityEpsilon && acceleration >= 0f)
                    {
                        zPhysics.Velocity = 0f;
                        remaining = 0f;
                    }
                    break;
                case ZLevelStepEvent.MapBoundaryDown:
                    if (TryMove(entity.Owner, -1))
                    {
                        SetGroundState((entity.Owner, zPhysics), ZLevelGroundState.Airborne);
                        crossings++;
                        if (zPhysics.Fallable)
                        {
                            var fall = new ZLevelFallMapEvent();
                            RaiseLocalEvent(entity.Owner, ref fall);
                        }
                    }
                    else
                    {
                        var handled = ResolveBlockedBoundary(entity.Owner, zPhysics, presentation, -1);
                        if (handled || zPhysics.Velocity <= VelocityEpsilon && acceleration <= 0f)
                            remaining = 0f;
                    }
                    break;
                case ZLevelStepEvent.MapBoundaryUp:
                    if (TryMove(entity.Owner, 1))
                    {
                        SetGroundState((entity.Owner, zPhysics), ZLevelGroundState.Airborne);
                        crossings++;
                    }
                    else
                    {
                        var handled = ResolveBlockedBoundary(entity.Owner, zPhysics, presentation, 1);
                        if (handled || zPhysics.Velocity >= -VelocityEpsilon && acceleration >= 0f)
                            remaining = 0f;
                    }
                    break;
                default:
                    throw new InvalidOperationException($"Unexpected z-level motion event {motionEvent.Type}.");
            }
        }

        zPhysics.Velocity = Math.Clamp(zPhysics.Velocity, -_velocityLimit, _velocityLimit);
        zPhysics.LastStep = new ZLevelStepDebug(
            oldVelocity,
            zPhysics.Velocity,
            remaining,
            crossings,
            events,
            nextEvent,
            limitReached);

        if (MathF.Abs(oldVelocity - zPhysics.Velocity) > 0.001f)
            DirtyField(entity.Owner, zPhysics, nameof(ZLevelPhysicsComponent.Velocity));

        if (zPhysics.VelocityGravity)
        {
            var distanceToGround = zPhysics.SupportSurface == ZLevelSupportSurface.None
                ? float.PositiveInfinity
                : presentation.LocalHeight - GetLocalSupportHeight((entity.Owner, zPhysics));
            var targetStatus = zPhysics.GroundState == ZLevelGroundState.Airborne && distanceToGround > _airborneHeight
                ? BodyStatus.InAir
                : BodyStatus.OnGround;
            if (entity.Comp2.BodyStatus != targetStatus)
            {
                _physics.SetBodyStatus(entity.Owner, entity.Comp2, targetStatus);
                var status = new ZLevelBodyStatusChangedEvent(targetStatus);
                RaiseLocalEvent(entity.Owner, ref status);
            }
        }

        SleepUpdate((entity.Owner, zPhysics), presentation, frameTime);
    }

    private MotionSegment GetMotionSegment(float velocity, float acceleration, float remaining)
    {
        if (MathF.Abs(acceleration) <= VelocityEpsilon || _velocityLimit <= VelocityEpsilon)
            return new MotionSegment(remaining, 0f, ZLevelStepEvent.None);

        var terminal = MathF.Sign(acceleration) * _velocityLimit;
        if (MathF.Abs(velocity - terminal) <= VelocityEpsilon)
            return new MotionSegment(remaining, 0f, ZLevelStepEvent.None);

        var endEvent = ZLevelStepEvent.TerminalVelocity;
        var duration = (terminal - velocity) / acceleration;

        if (velocity * acceleration < 0f)
        {
            var turnTime = -velocity / acceleration;
            if (turnTime >= 0f && turnTime < duration)
            {
                duration = turnTime;
                endEvent = ZLevelStepEvent.VelocityTurn;
            }
        }

        if (duration <= TimeEpsilon || duration >= remaining)
            return new MotionSegment(remaining, acceleration, ZLevelStepEvent.None);

        return new MotionSegment(duration, acceleration, endEvent);
    }

    private MotionEvent FindNextEvent(
        EntityUid uid,
        ZLevelPhysicsComponent component,
        float position,
        MotionSegment segment)
    {
        var endVelocity = component.Velocity + segment.Acceleration * segment.Duration;
        var direction = MathF.Abs(component.Velocity) > VelocityEpsilon
            ? Math.Sign(component.Velocity)
            : Math.Sign(endVelocity);

        if (direction == 0)
            return MotionEvent.None;

        if (direction < 0)
        {
            if (component.Fallable &&
                component.SupportSurface != ZLevelSupportSurface.None)
            {
                var localSupportHeight = GetLocalSupportHeight((uid, component));
                if (localSupportHeight < -PositionEpsilon || localSupportHeight > position + PositionEpsilon)
                    goto CheckDownBoundary;

                var time = SolveTimeToPosition(
                    position,
                    localSupportHeight,
                    component.Velocity,
                    segment.Acceleration,
                    segment.Duration);
                if (time != null)
                    return new MotionEvent(ZLevelStepEvent.Floor, time.Value, localSupportHeight);
            }

            CheckDownBoundary:
            var boundaryTime = SolveTimeToPosition(
                position,
                0f,
                component.Velocity,
                segment.Acceleration,
                segment.Duration);
            return boundaryTime is { } downTime
                ? new MotionEvent(ZLevelStepEvent.MapBoundaryDown, downTime, 0f)
                : MotionEvent.None;
        }

        var upperTime = SolveTimeToPosition(
            position,
            1f,
            component.Velocity,
            segment.Acceleration,
            segment.Duration);
        if (upperTime == null)
            return MotionEvent.None;

        return new MotionEvent(
            HasTransitionObstruction(uid, 1) ? ZLevelStepEvent.Ceiling : ZLevelStepEvent.MapBoundaryUp,
            upperTime.Value,
            1f);
    }

    private static float? SolveTimeToPosition(
        float start,
        float target,
        float velocity,
        float acceleration,
        float maxTime)
    {
        var direction = MathF.Abs(velocity) > VelocityEpsilon
            ? Math.Sign(velocity)
            : Math.Sign(acceleration);
        var distance = (target - start) * direction;
        if (distance < -PositionEpsilon)
            return null;
        if (distance <= PositionEpsilon)
            return 0f;

        var speed = velocity * direction;
        var directedAcceleration = acceleration * direction;
        float time;
        if (MathF.Abs(directedAcceleration) <= VelocityEpsilon)
        {
            if (speed <= VelocityEpsilon)
                return null;
            time = distance / speed;
        }
        else
        {
            var discriminant = speed * speed + 2f * directedAcceleration * distance;
            if (discriminant < 0f)
                return null;

            time = (-speed + MathF.Sqrt(discriminant)) / directedAcceleration;
        }

        return time >= -TimeEpsilon && time <= maxTime + TimeEpsilon
            ? Math.Clamp(time, 0f, maxTime)
            : null;
    }

    private void AdvanceMotion(
        ZLevelPhysicsComponent component,
        Entity<ZLevelPresentationComponent?> presentation,
        float duration,
        float acceleration)
    {
        if (duration <= 0f)
            return;

        var position = presentation.Comp!.LocalHeight +
                       component.Velocity * duration +
                       0.5f * acceleration * duration * duration;
        component.Velocity += acceleration * duration;
        SetLocalHeight(presentation, position);
    }

    private void ResolveImpact(
        EntityUid uid,
        ZLevelPhysicsComponent component,
        ZLevelPresentationComponent presentation,
        ZLevelImpactSurface surface)
    {
        var impactSpeed = MathF.Abs(component.Velocity);
        if (impactSpeed >= _impactVelocity)
        {
            var impact = new ZLevelImpactEvent(impactSpeed, surface);
            RaiseLocalEvent(uid, ref impact);
        }

        var landing = new ZLevelLandingEvent(impactSpeed, surface);
        RaiseLocalEvent(uid, ref landing);

        component.Velocity = -component.Velocity * MathF.Max(0f, component.Bounciness);
        if (MathF.Abs(component.Velocity) < MathF.Max(0f, component.SleepThreshold))
            component.Velocity = 0f;

        if (surface == ZLevelImpactSurface.Floor)
        {
            SetGroundState((uid, component), ZLevelGroundState.Grounded);
            SetLocalHeight((uid, presentation), GetLocalSupportHeight((uid, component)));
            NormalizeEntityMapFromAbsoluteZ((uid, component), component.SupportHeight);
        }
    }

    private bool ResolveBlockedBoundary(
        EntityUid uid,
        ZLevelPhysicsComponent component,
        ZLevelPresentationComponent presentation,
        int direction)
    {
        var boundary = new ZLevelBoundaryEvent(direction, presentation.LocalHeight, component.Velocity);
        RaiseLocalEvent(uid, ref boundary);
        if (boundary.Handled)
            return true;

        if (direction < 0)
        {
            var xform = Transform(uid);
            if (xform.MapUid is { } map && _zLevels.TryGetMapDepth(map, out var depth))
            {
                ApplySupportResult(
                    (uid, component),
                    new ZLevelSupportResult(
                        map,
                        ZLevelSupportSurface.NetworkBoundary,
                        depth.Value,
                        _transform.GetWorldPosition(xform)));
            }
            ResolveImpact(uid, component, presentation, ZLevelImpactSurface.Floor);
        }
        else
        {
            ResolveImpact(uid, component, presentation, ZLevelImpactSurface.Ceiling);
        }

        return false;
    }

    private void SleepUpdate(
        Entity<ZLevelPhysicsComponent> entity,
        ZLevelPresentationComponent presentation,
        float frameTime)
    {
        var distance = entity.Comp.SupportSurface == ZLevelSupportSurface.None
            ? float.PositiveInfinity
            : presentation.LocalHeight - GetLocalSupportHeight(entity);
        var almostStopped = MathF.Abs(entity.Comp.Velocity) < MathF.Max(0f, entity.Comp.SleepThreshold) &&
                            MathF.Abs(distance) <= 0.01f;

        if (!almostStopped)
        {
            entity.Comp.SleepTimer = 0f;
            return;
        }

        entity.Comp.SleepTimer += frameTime;
        if (entity.Comp.SleepTimer >= MathF.Max(0f, entity.Comp.TimeToSleep))
            SleepBody(entity);
    }

    private void UpdatePendingBodyRefreshes()
    {
        if (_pendingBodyRefresh.Count == 0)
            return;

        _nearbyBodies.Clear();
        foreach (var uid in _pendingBodyRefresh)
        {
            if (!_zPhysicsQuery.TryComp(uid, out var component))
                continue;

            _nearbyBodies.Add((uid, component));
        }

        foreach (var entity in _nearbyBodies)
        {
            if (!HasActiveZState(entity.Comp) || !CanSimulateBody(entity))
            {
                _pendingBodyRefresh.Remove(entity.Owner);
                continue;
            }

            RefreshSupport(entity);
            RefreshBody(entity);

            if (_activeBodySet.Contains(entity.Owner))
                _pendingBodyRefresh.Remove(entity.Owner);
        }

        _nearbyBodies.Clear();
    }

    private static bool HasActiveZState(ZLevelPhysicsComponent component)
    {
        // A newly replicated component can arrive before its presentation component state. Keep it eligible for one
        // refresh; settled bodies are removed by SleepUpdate once both states are available.
        return MathF.Abs(component.Velocity) > 0.001f || !component.Sleeping;
    }

    private void UpdateDirtyMovement()
    {
        foreach (var uid in _dirtyMovementBodies)
        {
            if (!_zPhysicsQuery.TryComp(uid, out var component) || !CanSimulateBody((uid, component)))
                continue;

            // Actual XY movement may step onto, step off, or leave the current flat support.
            RefreshSupport((uid, component));
            RefreshBody((uid, component));
        }

        _dirtyMovementBodies.Clear();
    }

    /// <summary>
    /// Returns the cached distance from a body to support below it.
    /// </summary>
    [Pure]
    public float DistanceToGround(Entity<ZLevelPhysicsComponent> entity)
    {
        if (entity.Comp.SupportSurface == ZLevelSupportSurface.None)
            return float.PositiveInfinity;

        return _zPresentationQuery.TryComp(entity.Owner, out var presentation)
            ? presentation.LocalHeight - GetLocalSupportHeight(entity)
            : -GetLocalSupportHeight(entity);
    }

    /// <summary>
    /// Sets a body's local vertical position and wakes it.
    /// </summary>
    [PublicAPI]
    public void SetZPosition(Entity<ZLevelPhysicsComponent> entity, float position)
    {
        if (!float.IsFinite(position))
            return;

        var presentation = EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        SetGroundState(entity, ZLevelGroundState.Airborne);
        SetLocalHeight((entity.Owner, presentation), position);
        WakeBody(entity);
    }

    /// <summary>
    /// Sets a body's vertical velocity and wakes it.
    /// </summary>
    [PublicAPI]
    public void SetZVelocity(Entity<ZLevelPhysicsComponent> entity, float velocity)
    {
        if (!float.IsFinite(velocity))
            return;

        entity.Comp.Velocity = velocity;
        if (velocity > VelocityEpsilon)
            SetGroundState(entity, ZLevelGroundState.Airborne);
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPhysicsComponent.Velocity));
        WakeBody(entity);
    }

    /// <summary>
    /// Adds to a body's vertical velocity and wakes it.
    /// </summary>
    [PublicAPI]
    public void AddZVelocity(Entity<ZLevelPhysicsComponent> entity, float velocity)
    {
        SetZVelocity(entity, entity.Comp.Velocity + velocity);
    }

    /// <summary>
    /// Sets how a body uses floor and high-ground support.
    /// </summary>
    public void SetGroundSupport(Entity<ZLevelPhysicsComponent> entity, bool fallable, bool autoStep)
    {
        if (entity.Comp.Fallable == fallable && entity.Comp.AutoStep == autoStep)
            return;

        entity.Comp.Fallable = fallable;
        entity.Comp.AutoStep = autoStep;
        DirtyFields(
            entity.Owner,
            entity.Comp,
            null,
            nameof(ZLevelPhysicsComponent.Fallable),
            nameof(ZLevelPhysicsComponent.AutoStep));
        RefreshSupport(entity);
        RefreshBody(entity);
    }

    /// <summary>
    /// Re-evaluates a body's gravity multiplier through <see cref="ZLevelCheckGravityEvent"/>.
    /// </summary>
    public void UpdateGravityState(Entity<ZLevelPhysicsComponent> entity)
    {
        var gravity = new ZLevelCheckGravityEvent();
        RaiseLocalEvent(entity.Owner, ref gravity);
        entity.Comp.GravityMultiplier = gravity.Gravity;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPhysicsComponent.GravityMultiplier));
        WakeBody(entity);
    }

    /// <summary>
    /// Moves an entity to another map in the same z-level network while preserving map-relative position and rotation.
    /// </summary>
    public bool TryMove(EntityUid uid, int offset)
    {
        if (HasTransitionObstruction(uid, offset) ||
            !_zLevels.TryMoveEntityToMapOffset(uid, offset, out var targetMap) ||
            targetMap is not { } targetMapUid)
            return false;

        if (_zPhysicsQuery.TryComp(uid, out var zPhysics))
            RefreshSupport((uid, zPhysics));

        var moved = new ZLevelMapMoveEvent(offset, targetMapUid);
        RaiseLocalEvent(uid, ref moved);
        return true;
    }

    /// <summary>
    /// Moves an entity one z-level up.
    /// </summary>
    public bool TryMoveUp(EntityUid uid) => TryMove(uid, 1);

    /// <summary>
    /// Moves an entity one z-level down.
    /// </summary>
    public bool TryMoveDown(EntityUid uid) => TryMove(uid, -1);

    /// <summary>
    /// Wakes a body for vertical simulation.
    /// </summary>
    public void WakeBody(Entity<ZLevelPhysicsComponent> entity)
    {
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            return;
        }

        if (!_activeBodySet.Add(entity.Owner))
            return;

        entity.Comp.Sleeping = false;
        entity.Comp.SleepTimer = 0f;
        _activeBodies.Add(entity.Owner);
    }

    /// <summary>
    /// Stops simulating a settled or inactive body until it is woken.
    /// </summary>
    public void SleepBody(Entity<ZLevelPhysicsComponent> entity)
    {
        entity.Comp.Sleeping = true;
        entity.Comp.SleepTimer = 0f;
        if (!_activeBodySet.Remove(entity.Owner))
            return;

        _activeBodies.Remove(entity.Owner);
    }

    /// <summary>
    /// Re-evaluates whether a body can participate in vertical simulation.
    /// </summary>
    public void RefreshBody(Entity<ZLevelPhysicsComponent> entity)
    {
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            return;
        }

        WakeBody(entity);
    }

    /// <summary>
    /// Initializes an eligible body's support without manufacturing a fall or landing for an entity already resting
    /// on the current map. Unsupported or moving bodies are activated normally.
    /// </summary>
    private void InitializeBody(Entity<ZLevelPhysicsComponent> entity)
    {
        if (!CanSimulateBody(entity))
        {
            SleepBody(entity);
            return;
        }

        RefreshSupport(entity, wake: false);
        if (!_zPresentationQuery.TryComp(entity.Owner, out var presentation))
        {
            presentation = EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        }

        var localSupportHeight = GetLocalSupportHeight(entity);
        if (entity.Comp.SupportSurface != ZLevelSupportSurface.None &&
            entity.Comp.AutoStep &&
            presentation.LocalHeight < localSupportHeight &&
            localSupportHeight - presentation.LocalHeight <= _maxStepUp + PositionEpsilon)
        {
            SetLocalHeight((entity.Owner, presentation), localSupportHeight);
        }

        var onSupport = entity.Comp.SupportSurface != ZLevelSupportSurface.None &&
                        MathF.Abs(presentation.LocalHeight - localSupportHeight) <= PositionEpsilon;
        if (onSupport && entity.Comp.Velocity <= VelocityEpsilon)
        {
            entity.Comp.Velocity = 0f;
            SetGroundState(entity, ZLevelGroundState.Grounded);
            if (_physicsQuery.TryComp(entity.Owner, out var physics) &&
                entity.Comp.VelocityGravity &&
                physics.BodyStatus != BodyStatus.OnGround)
            {
                _physics.SetBodyStatus(entity.Owner, physics, BodyStatus.OnGround);
            }

            SleepBody(entity);
            return;
        }

        SetGroundState(entity, ZLevelGroundState.Airborne);
        WakeBody(entity);
    }

    private bool CanSimulateBody(
        Entity<ZLevelPhysicsComponent> entity,
        PhysicsComponent? physics = null,
        TransformComponent? xform = null)
    {
        if (_net.IsClient ||
            (!entity.Comp.VelocityGravity && !entity.Comp.Fallable && !entity.Comp.AutoStep) ||
            TerminatingOrDeleted(entity.Owner) ||
            !_physicsQuery.Resolve(entity.Owner, ref physics, false) ||
            !_xformQuery.Resolve(entity.Owner, ref xform, false))
        {
            return false;
        }

        if (_container.IsEntityOrParentInContainer(entity.Owner))
            return false;

        var parent = xform.ParentUid;
        var directlyOnMap = parent == xform.GridUid || parent == xform.MapUid;
        if (!directlyOnMap || xform.Anchored || physics.BodyType == BodyType.Static)
            return false;

        if (!_zMapQuery.HasComp(xform.MapUid))
            return false;

        return true;
    }

    private void RemoveActiveAt(int index, EntityUid uid)
    {
        _activeBodies.RemoveAt(index);
        _activeBodySet.Remove(uid);
    }
}

internal readonly record struct MotionSegment(
    float Duration,
    float Acceleration,
    ZLevelStepEvent EndEvent);

internal readonly record struct MotionEvent(
    ZLevelStepEvent Type,
    float Time,
    float Position)
{
    public static readonly MotionEvent None = new(ZLevelStepEvent.None, float.PositiveInfinity, 0f);
}

/// <summary>
/// Raised after an entity changes maps due to continuous z-axis motion.
/// </summary>
[ByRefEvent]
public readonly record struct ZLevelMapMoveEvent(int Offset, EntityUid TargetMap);

/// <summary>
/// Raised when gravity carries an entity to the map below.
/// </summary>
[ByRefEvent]
public struct ZLevelFallMapEvent;

public enum ZLevelImpactSurface : byte
{
    Floor,
    Ceiling,
}

/// <summary>
/// Raised when a body strikes vertical support or a ceiling with meaningful speed.
/// </summary>
[ByRefEvent]
public readonly record struct ZLevelImpactEvent(float ImpactSpeed, ZLevelImpactSurface Surface);

/// <summary>
/// Raised exactly when a vertical body reaches a blocking floor or ceiling, including low-speed contacts.
/// Content landing uses this instead of a timer or a thresholded damage impact.
/// </summary>
[ByRefEvent]
public readonly record struct ZLevelLandingEvent(float ImpactSpeed, ZLevelImpactSurface Surface);

/// <summary>
/// Raised each fixed step for bodies that request external vertical forces.
/// </summary>
[ByRefEvent]
public struct ZLevelGetVelocityEvent(EntityUid uid, ZLevelPhysicsComponent component)
{
    public Entity<ZLevelPhysicsComponent> Target = (uid, component);
    public float VelocityDelta;
}

/// <summary>
/// Raised when a body asks external systems to recompute its gravity multiplier.
/// </summary>
[ByRefEvent]
public struct ZLevelCheckGravityEvent
{
    public float Gravity;

    public ZLevelCheckGravityEvent()
    {
        Gravity = 1f;
    }
}

/// <summary>
/// Raised when vertical motion reaches the end of a z-level map network.
/// Set when supplying an alternative such as chasm behavior.
/// </summary>
[ByRefEvent]
public struct ZLevelBoundaryEvent(int direction, float localHeight, float velocity)
{
    public int Direction = direction;
    public float LocalHeight = localHeight;
    public float Velocity = velocity;
    public bool Handled;
}

/// <summary>
/// Raised when z-height synchronization changes the physics body status.
/// </summary>
[ByRefEvent]
public readonly record struct ZLevelBodyStatusChangedEvent(BodyStatus NewStatus);
