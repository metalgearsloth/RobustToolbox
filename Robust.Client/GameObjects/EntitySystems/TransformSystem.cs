using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using JetBrains.Annotations;
using Robust.Client.Timing;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Timing;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;
using Unsafe = System.Runtime.CompilerServices.Unsafe;

namespace Robust.Client.GameObjects;

/// <summary>
/// Maintains the client-only positions used for rendering transform interpolation.
/// Simulation, prediction, physics and networking always read the final pose from
/// <see cref="TransformComponent"/>.
/// </summary>
[UsedImplicitly]
public sealed partial class TransformSystem : SharedTransformSystem
{
    private const int MaxTransformDepth = 128;

    [Dependency] private IConfigurationManager _configuration = default!;
    [Dependency] private IClientGameTiming _timing = default!;
    [Dependency] private MapSystem _mapSystem = default!;
    [Dependency] private ZLevelSystem _zLevels = default!;

    [Dependency] private EntityQuery<MapGridComponent> _renderGridQuery;
    [Dependency] private EntityQuery<ZLevelPresentationComponent> _zPresentationQuery;

    [ViewVariables]
    private readonly Dictionary<EntityUid, RenderPoseState> _renderPoses = new();

    private readonly HashSet<EntityUid> _snapRenderRotations = new();
    private readonly Dictionary<MapId, float> _cullingMargins = new();
    private readonly HashSet<EntityUid> _cullingVisited = new();
    private readonly HashSet<EntityUid> _remove = new();
    private float _maxInterpolationDistanceSquared;
    private float _minInterpolationDistanceSquared;
    private float _correctionHalfLife;
    private float _minCorrectionTranslationSquared;
    private float _minCorrectionRotation;

    /// <summary>
    /// Invoked when interpolation crosses render spaces. The default policy only permits the same map.
    /// A client feature such as z-level rendering can mark other map pairs as compatible.
    /// </summary>
    public event RenderSpaceCompatibilityHandler? RenderSpaceCompatibility;

    public delegate void RenderSpaceCompatibilityHandler(ref RenderSpaceCompatibilityEvent args);

    public override void Initialize()
    {
        base.Initialize();
        OnGlobalMoveEvent += OnTransformMoved;
        SubscribeLocalEvent<ZLevelPresentationComponent, ZLevelPresentationChangedEvent>(OnPresentationChanged);

        _configuration.OnValueChanged(CVars.NetInterpMaxDistance, SetMaxInterpolationDistance, true);
        _configuration.OnValueChanged(CVars.NetInterpMinDistance, SetMinInterpolationDistance, true);
        _configuration.OnValueChanged(CVars.NetInterpCorrectionHalfLife, SetCorrectionHalfLife, true);
        _configuration.OnValueChanged(CVars.NetInterpCorrectionMinTranslation, SetMinCorrectionTranslation, true);
        _configuration.OnValueChanged(CVars.NetInterpCorrectionMinRotation, SetMinCorrectionRotation, true);
    }

    public override void Shutdown()
    {
        OnGlobalMoveEvent -= OnTransformMoved;
        _configuration.UnsubValueChanged(CVars.NetInterpMaxDistance, SetMaxInterpolationDistance);
        _configuration.UnsubValueChanged(CVars.NetInterpMinDistance, SetMinInterpolationDistance);
        _configuration.UnsubValueChanged(CVars.NetInterpCorrectionHalfLife, SetCorrectionHalfLife);
        _configuration.UnsubValueChanged(CVars.NetInterpCorrectionMinTranslation, SetMinCorrectionTranslation);
        _configuration.UnsubValueChanged(CVars.NetInterpCorrectionMinRotation, SetMinCorrectionRotation);
        _renderPoses.Clear();
        _snapRenderRotations.Clear();
        _cullingMargins.Clear();
        base.Shutdown();
    }

    /// <summary>
    /// Clears all cached render state.
    /// </summary>
    public void ResetRenderPoses()
    {
        _renderPoses.Clear();
        _snapRenderRotations.Clear();
        _cullingMargins.Clear();
        _cullingVisited.Clear();
        _remove.Clear();
    }

    [SubscribeLocalEvent]
    private void OnTransformShutdown(EntityUid uid, TransformComponent component, ComponentShutdown args)
    {
        _renderPoses.Remove(uid);
        _snapRenderRotations.Remove(uid);
    }

    private void SetMaxInterpolationDistance(float value)
    {
        value = Math.Max(0f, value);
        _maxInterpolationDistanceSquared = value * value;
    }

    private void SetMinInterpolationDistance(float value)
    {
        value = Math.Max(0f, value);
        _minInterpolationDistanceSquared = value * value;
    }

    private void SetCorrectionHalfLife(float value)
        => _correctionHalfLife = Math.Max(0f, value);

    private void SetMinCorrectionTranslation(float value)
    {
        value = Math.Max(0f, value);
        _minCorrectionTranslationSquared = value * value;
    }

    private void SetMinCorrectionRotation(float value)
        => _minCorrectionRotation = Math.Max(0f, value);

    private void OnTransformMoved(ref MoveEvent args)
    {
        var uid = args.Sender;
        var xform = args.Component;

        if (xform.Deleted || !TryCreateEndpoint(uid, args.NewPosition, args.NewRotation, out var target))
        {
            _renderPoses.Remove(uid);
            return;
        }

        if (!TryCreateEndpoint(uid, args.OldPosition, args.OldRotation, out var oldEndpoint))
        {
            _renderPoses.Remove(uid);
            return;
        }

        HandlePoseChange(uid, xform, oldEndpoint, target);
    }

    private void OnPresentationChanged(
        Entity<ZLevelPresentationComponent> entity,
        ref ZLevelPresentationChangedEvent args)
    {
        if (!XformQuery.TryGetComponent(entity.Owner, out var xform) || xform.Deleted)
            return;

        var coordinates = new EntityCoordinates(xform.ParentUid, xform.LocalPosition);
        if (!TryCreateEndpoint(entity.Owner, coordinates, xform.LocalRotation, out var source, args.OldHeight) ||
            !TryCreateEndpoint(entity.Owner, coordinates, xform.LocalRotation, out var target, args.NewHeight))
        {
            _renderPoses.Remove(entity.Owner);
            return;
        }

        HandlePoseChange(entity.Owner, xform, source, target);
    }

    private void HandlePoseChange(
        EntityUid uid,
        TransformComponent xform,
        in RenderPoseEndpoint oldEndpoint,
        in RenderPoseEndpoint target)
    {
        ref var existing = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
        var hasExisting = !Unsafe.IsNullRef(ref existing);

        var source = oldEndpoint;
        RenderPose rendered;

        if (hasExisting)
        {
            // Use the last pose that reached the screen.
            rendered = existing.LastRendered;
            var parentOrRenderSpaceChanged = oldEndpoint.Parent != target.Parent
                                             || oldEndpoint.RenderSpace != target.RenderSpace;
            var sameTickPredictionReplay = existing.Type == RenderInterpolationType.PredictionInterpolation
                                           && existing.ChangeTick == _timing.CurTick
                                           && (existing.PendingPredictionReplay || _timing.ApplyingState);
            var predictionRollbackOrCorrection = existing.Type == RenderInterpolationType.PredictionCorrection
                                                 || sameTickPredictionReplay;

            // Multiple changes in one simulation tick retain the original source: A -> B -> C is A -> C.
            if (existing.Type != RenderInterpolationType.PredictionCorrection
                && existing.ChangeTick == _timing.CurTick
                && existing.Alpha <= existing.InterpolationStartAlpha
                && !existing.PendingPredictionReplay)
            {
                source = existing.Source;
            }
            else if (parentOrRenderSpaceChanged || predictionRollbackOrCorrection)
            {
                // Rebase from the pose that was displayed, without binding it to the old parent.
                source = new RenderPoseEndpoint(
                    EntityUid.Invalid,
                    rendered.CanonicalPosition,
                    rendered.Rotation,
                    rendered.CoordinateSpace,
                    rendered.AbsoluteZ);
            }
        }
        else
        {
            rendered = ResolveLastRenderedEndpoint(oldEndpoint, 0);
        }

        if (!TryGetCommonRenderSpace(source.RenderSpace, target.RenderSpace, out var coordinateSpace)
            || !ShouldInterpolate(source, target))
        {
            _renderPoses.Remove(uid);
            return;
        }

        if (!_timing.ApplyingState)
        {
            if (_timing.IsFirstTimePredicted)
            {
                // First-time prediction renders over the current tick. Replay retains the last rendered pose below.
                if (hasExisting
                    && existing.Type == RenderInterpolationType.PredictionInterpolation
                    && existing.ChangeTick == _timing.CurTick
                    && existing.Alpha <= existing.InterpolationStartAlpha
                    && !existing.PendingPredictionReplay)
                {
                    existing.Target = target;
                    existing.CoordinateSpace = coordinateSpace;
                    return;
                }

                ref var tickState = ref CollectionsMarshal.GetValueRefOrAddDefault(_renderPoses, uid, out _);
                StartTickInterpolation(
                    ref tickState,
                    source,
                    target,
                    rendered,
                    coordinateSpace,
                    RenderInterpolationType.PredictionInterpolation);
                return;
            }

            if (hasExisting && existing.Type == RenderInterpolationType.PredictionCorrection)
            {
                existing.Target = target;
                existing.CoordinateSpace = coordinateSpace;

                if (existing.PendingPredictionReplay && _timing.InPrediction)
                    RebaseCorrection(ref existing, target);

                return;
            }

            if (hasExisting)
            {
                RebaseTickInterpolation(ref existing, source, target, rendered, coordinateSpace);
                return;
            }

            // Replay without render state must not interpolate from a rollback position.
            _renderPoses.Remove(uid);
            return;
        }

        if (hasExisting && existing.Type == RenderInterpolationType.PredictionInterpolation)
        {
            RebaseTickInterpolation(ref existing, source, target, rendered, coordinateSpace);
            return;
        }

        // ResetPredictedEntities restores the cached server state before ApplyGameState applies the newly
        // arrived state. The reset creates the correction below while LastModifiedTick is still in the future,
        // but ResetPredictedEntities normalizes that tick before the new state is applied. Preserve the correction
        // across both state applications so prediction replay can rebase it against the final predicted pose.
        if (hasExisting && existing.Type == RenderInterpolationType.PredictionCorrection)
        {
            if (!existing.PendingPredictionReplay)
                existing.CorrectionAnchor = rendered;

            existing.Source = source;
            existing.Target = target;
            existing.LastRendered = rendered;
            existing.CoordinateSpace = coordinateSpace;
            existing.ChangeTick = _timing.CurTick;
            existing.PendingPredictionReplay = true;
            RebaseCorrection(ref existing, target);
            return;
        }

        // A transform that was dirty in a future prediction tick is being authoritatively reset.
        // Keep the last render pose on screen and decay the difference from the replayed simulation pose.
        if (xform.LastModifiedTick > _timing.LastRealTick)
        {
            ref var correction = ref CollectionsMarshal.GetValueRefOrAddDefault(_renderPoses, uid, out _);
            correction.Source = source;
            correction.Target = target;
            correction.LastRendered = rendered;
            correction.CorrectionAnchor = rendered;
            correction.CoordinateSpace = coordinateSpace;
            correction.ChangeTick = _timing.CurTick;
            correction.Type = RenderInterpolationType.PredictionCorrection;
            correction.PendingPredictionReplay = true;
            correction.CorrectionRemaining = 1f;
            RebaseCorrection(ref correction, target);
            return;
        }

        ref var networkState = ref CollectionsMarshal.GetValueRefOrAddDefault(_renderPoses, uid, out _);
        networkState.Source = source;
        networkState.Target = target;
        networkState.LastRendered = rendered;
        networkState.CoordinateSpace = coordinateSpace;
        networkState.ChangeTick = _timing.CurTick;
        networkState.Type = RenderInterpolationType.NetworkInterpolation;
        networkState.Alpha = 0f;
        networkState.InterpolationStartAlpha = 0f;
        networkState.LastFramePhase = -1f;
        networkState.PendingPredictionReplay = false;
        networkState.CorrectionTranslation = Vector2.Zero;
        networkState.CorrectionRotation = Angle.Zero;
        networkState.CorrectionZ = 0f;
        networkState.CorrectionRemaining = 0f;
    }

    private void StartTickInterpolation(
        ref RenderPoseState state,
        in RenderPoseEndpoint source,
        in RenderPoseEndpoint target,
        in RenderPose rendered,
        EntityUid coordinateSpace,
        RenderInterpolationType type)
    {
        state.Source = source;
        state.Target = target;
        state.LastRendered = rendered;
        state.CoordinateSpace = coordinateSpace;
        state.ChangeTick = _timing.CurTick;
        state.Type = type;
        state.Alpha = 0f;
        state.InterpolationStartAlpha = 0f;
        state.LastFramePhase = -1f;
        state.PendingPredictionReplay = false;
        state.CorrectionTranslation = Vector2.Zero;
        state.CorrectionRotation = Angle.Zero;
        state.CorrectionZ = 0f;
        state.CorrectionRemaining = 0f;
    }

    private void RebaseTickInterpolation(
        ref RenderPoseState state,
        in RenderPoseEndpoint source,
        in RenderPoseEndpoint target,
        in RenderPose rendered,
        EntityUid coordinateSpace)
    {
        var newPredictionTick = _timing.InPrediction && _timing.CurTick > state.ChangeTick;

        if (newPredictionTick)
        {
            state.CorrectionAnchor = rendered;
            state.InterpolationStartAlpha = 0f;
            state.Alpha = 0f;
            state.LastFramePhase = -1f;
            state.ChangeTick = _timing.CurTick;
            state.PendingPredictionReplay = true;
        }

        if (!state.PendingPredictionReplay)
        {
            state.CorrectionAnchor = rendered;
            state.InterpolationStartAlpha = state.Alpha;
            state.PendingPredictionReplay = true;
        }

        state.Source = source;
        state.Target = target;
        state.CoordinateSpace = coordinateSpace;
        state.LastRendered = state.CorrectionAnchor;
        state.Type = RenderInterpolationType.PredictionInterpolation;
    }

    private bool ShouldInterpolate(in RenderPoseEndpoint source, in RenderPoseEndpoint target)
    {
        var sourcePose = ResolveEndpoint(source, 0);
        var targetPose = ResolveEndpoint(target, 0);
        var distance = Vector2.DistanceSquared(sourcePose.Position, targetPose.Position);

        if (distance >= _maxInterpolationDistanceSquared)
            return false;

        return distance > _minInterpolationDistanceSquared
               || !sourcePose.Rotation.EqualsApprox(targetPose.Rotation)
               || !MathHelper.CloseTo(sourcePose.AbsoluteZ, targetPose.AbsoluteZ);
    }

    private void RebaseCorrection(ref RenderPoseState state, in RenderPoseEndpoint target)
    {
        var targetPose = ResolveEndpoint(target, 0);
        state.CorrectionTranslation = state.CorrectionAnchor.CanonicalPosition - targetPose.CanonicalPosition;
        state.CorrectionRotation = state.SnapRotation
            ? Angle.Zero
            : Angle.ShortestDistance(targetPose.Rotation, state.CorrectionAnchor.Rotation);
        state.CorrectionZ = state.CorrectionAnchor.AbsoluteZ - targetPose.AbsoluteZ;
        state.LastRendered = state.CorrectionAnchor;
        state.Alpha = 0f;
    }

    public override void FrameUpdate(float frameTime)
    {
        base.FrameUpdate(frameTime);

        foreach (var uid in _snapRenderRotations)
        {
            ref var state = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
            if (!Unsafe.IsNullRef(ref state))
                SnapRenderRotation(ref state);
        }

        _snapRenderRotations.Clear();
        _remove.Clear();
        _cullingMargins.Clear();
        _cullingVisited.Clear();

        foreach (var (uid, _) in _renderPoses)
        {
            ref var state = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
            if (Unsafe.IsNullRef(ref state))
                continue;

            if (!XformQuery.TryGetComponent(uid, out var xform) || xform.Deleted)
            {
                _remove.Add(uid);
                continue;
            }

            if (state.Type == RenderInterpolationType.PredictionCorrection)
            {
                state.PendingPredictionReplay = false;

                if (frameTime > 0f)
                {
                    var decay = _correctionHalfLife <= 0f
                        ? 0f
                        : MathF.Pow(0.5f, frameTime / _correctionHalfLife);
                    state.CorrectionTranslation *= decay;
                    state.CorrectionRotation *= decay;
                    state.CorrectionZ *= decay;
                    state.CorrectionRemaining *= decay;
                }

                state.Alpha = 1f - state.CorrectionRemaining;
            }
            else
            {
                state.Alpha = GetInterpolationAlpha(ref state);
            }
        }

        // Resolve poses after interpolation has advanced so moving parents are cached for children.
        foreach (var (uid, _) in _renderPoses)
        {
            ref var state = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
            if (Unsafe.IsNullRef(ref state))
                continue;

            if (_remove.Contains(uid))
                continue;

            state.LastRendered = GetRenderPoseInternal(uid, 0);

            if (state.Type == RenderInterpolationType.PredictionInterpolation)
                state.PendingPredictionReplay = false;

            if (state.Type != RenderInterpolationType.PredictionCorrection && state.Alpha >= 1f)
            {
                _remove.Add(uid);
                continue;
            }

            if (state.Type == RenderInterpolationType.PredictionCorrection
                && state.CorrectionTranslation.LengthSquared() < _minCorrectionTranslationSquared
                && Math.Abs(state.CorrectionRotation.Theta) < _minCorrectionRotation
                && Math.Abs(state.CorrectionZ) < ZLevelProjection.BoundaryEpsilon)
            {
                _remove.Add(uid);
            }
        }

        // Component trees are indexed by simulation positions, so include render-position error in their queries.
        foreach (var uid in _renderPoses.Keys)
        {
            if (!_remove.Contains(uid))
                UpdateCullingMargin(uid, 0);
        }

        foreach (var uid in _remove)
        {
            _renderPoses.Remove(uid);
        }
    }

    private void UpdateCullingMargin(EntityUid uid, int depth)
    {
        if (depth >= MaxTransformDepth
            || !_cullingVisited.Add(uid)
            || !XformQuery.TryGetComponent(uid, out var xform)
            || xform.Deleted)
        {
            return;
        }

        var simulation = base.GetWorldPositionRotation(xform);
        var rendered = GetRenderPoseInternal(uid, 0, xform);
        var errorSquared = Vector2.DistanceSquared(simulation.WorldPosition, rendered.Position);
        var isGrid = _renderGridQuery.TryGetComponent(uid, out var grid);
        if (isGrid && grid != null)
        {
            var simulationMatrix = Matrix3Helpers.CreateTransform(simulation.WorldPosition, simulation.WorldRotation);
            var renderMatrix = Matrix3Helpers.CreateTransform(rendered.Position, rendered.Rotation);
            errorSquared = Math.Max(errorSquared,
                GetPointTransformErrorSquared(grid.LocalAABB.BottomLeft, simulationMatrix, renderMatrix));
            errorSquared = Math.Max(errorSquared,
                GetPointTransformErrorSquared(grid.LocalAABB.BottomRight, simulationMatrix, renderMatrix));
            errorSquared = Math.Max(errorSquared,
                GetPointTransformErrorSquared(grid.LocalAABB.TopLeft, simulationMatrix, renderMatrix));
            errorSquared = Math.Max(errorSquared,
                GetPointTransformErrorSquared(grid.LocalAABB.TopRight, simulationMatrix, renderMatrix));
        }

        // A crossing entity is queried from its simulation tree but can be drawn in either adjacent map pass.
        // Include every renderer sample, not just the stable coordinate-space pose, in the conservative margin.
        Span<RenderLayerSample> layerSamples = stackalloc RenderLayerSample[2];
        var sampleCount = GetRenderLayerSamples(uid, layerSamples, xform);
        for (var i = 0; i < sampleCount; i++)
        {
            var sample = layerSamples[i];
            var sampleErrorSquared = Vector2.DistanceSquared(simulation.WorldPosition, sample.Position);
            if (isGrid && grid != null)
            {
                var simulationMatrix = Matrix3Helpers.CreateTransform(simulation.WorldPosition, simulation.WorldRotation);
                var sampleMatrix = Matrix3Helpers.CreateTransform(sample.Position, sample.Rotation);
                sampleErrorSquared = Math.Max(sampleErrorSquared,
                    GetPointTransformErrorSquared(grid.LocalAABB.BottomLeft, simulationMatrix, sampleMatrix));
                sampleErrorSquared = Math.Max(sampleErrorSquared,
                    GetPointTransformErrorSquared(grid.LocalAABB.BottomRight, simulationMatrix, sampleMatrix));
                sampleErrorSquared = Math.Max(sampleErrorSquared,
                    GetPointTransformErrorSquared(grid.LocalAABB.TopLeft, simulationMatrix, sampleMatrix));
                sampleErrorSquared = Math.Max(sampleErrorSquared,
                    GetPointTransformErrorSquared(grid.LocalAABB.TopRight, simulationMatrix, sampleMatrix));
            }

            var sampleError = MathF.Sqrt(sampleErrorSquared);
            errorSquared = Math.Max(errorSquared, sampleErrorSquared);
            UpdateCullingMargin(sample.Map, sampleError);
        }

        var error = MathF.Sqrt(errorSquared);
        UpdateCullingMargin(xform.MapID, error);
        UpdateCullingMargin(rendered.CoordinateSpace, error);
        if (rendered.SourceRenderSpace != rendered.CoordinateSpace)
            UpdateCullingMargin(rendered.SourceRenderSpace, error);
        if (rendered.TargetRenderSpace != rendered.CoordinateSpace
            && rendered.TargetRenderSpace != rendered.SourceRenderSpace)
        {
            UpdateCullingMargin(rendered.TargetRenderSpace, error);
        }

        // Grid bounds conservatively cover descendant movement without walking every grid entity each frame.
        if (isGrid)
            return;

        var children = xform.ChildEnumerator;
        while (children.MoveNext(out var child))
        {
            UpdateCullingMargin(child, depth + 1);
        }
    }

    private static float GetPointTransformErrorSquared(
        Vector2 point,
        in Matrix3x2 simulation,
        in Matrix3x2 rendered)
    {
        return Vector2.DistanceSquared(Vector2.Transform(point, simulation), Vector2.Transform(point, rendered));
    }

    private void UpdateCullingMargin(MapId mapId, float error)
    {
        if (mapId == MapId.Nullspace)
            return;

        ref var margin = ref CollectionsMarshal.GetValueRefOrAddDefault(_cullingMargins, mapId, out _);
        margin = Math.Max(margin, error);
    }

    private void UpdateCullingMargin(EntityUid renderSpace, float error)
    {
        if (TryGetRenderSpaceMapId(renderSpace, out var mapId))
            UpdateCullingMargin(mapId, error);
    }

    private float GetInterpolationAlpha(ref RenderPoseState state)
    {
        if (_timing.TickPeriod <= TimeSpan.Zero)
            return 1f;

        // Prediction runs ahead of ChangeTick; clamp old segments when the phase wraps.
        var phase = _timing.TickPhase;

        if (state.LastFramePhase >= 0f && phase < state.LastFramePhase)
            return 1f;

        state.LastFramePhase = phase;
        return Math.Max(state.Alpha, phase);
    }

    /// <summary>
    /// Gets the transform used for drawing.
    /// </summary>
    [Pure]
    public RenderPose GetRenderWorldPose(EntityUid uid, TransformComponent? xform = null)
    {
        if (!XformQuery.Resolve(uid, ref xform, false))
            return default;

        return GetRenderPoseInternal(uid, 0, xform);
    }

    /// <summary>
    /// Gets an entity's rendered world position and rotation.
    /// </summary>
    [Pure]
    public (Vector2 WorldPosition, Angle WorldRotation) GetRenderWorldPositionRotation(
        EntityUid uid,
        TransformComponent? xform = null)
    {
        var pose = GetRenderWorldPose(uid, xform);
        return (pose.Position, pose.Rotation);
    }

    /// <summary>
    /// Gets an entity's rendered world position.
    /// </summary>
    [Pure]
    public Vector2 GetRenderWorldPosition(EntityUid uid, TransformComponent? xform = null)
        => GetRenderWorldPose(uid, xform).Position;

    /// <summary>
    /// Gets an entity's rendered world rotation.
    /// </summary>
    [Pure]
    public Angle GetRenderWorldRotation(EntityUid uid, TransformComponent? xform = null)
        => GetRenderWorldPose(uid, xform).Rotation;

    /// <summary>
    /// Gets an entity's rendered world matrix.
    /// </summary>
    [Pure]
    public Matrix3x2 GetRenderWorldMatrix(EntityUid uid, TransformComponent? xform = null)
    {
        var pose = GetRenderWorldPose(uid, xform);
        return Matrix3Helpers.CreateTransform(pose.Position, pose.Rotation);
    }

    /// <summary>
    /// Gets the inverse of an entity's rendered world matrix.
    /// </summary>
    [Pure]
    public Matrix3x2 GetInvRenderWorldMatrix(EntityUid uid, TransformComponent? xform = null)
    {
        var pose = GetRenderWorldPose(uid, xform);
        return Matrix3Helpers.CreateInverseTransform(pose.Position, pose.Rotation);
    }

    /// <summary>
    /// Gets an entity's rendered coordinates in its stable coordinate space.
    /// </summary>
    [Pure]
    public MapCoordinates GetRenderMapCoordinates(EntityUid uid, TransformComponent? xform = null)
    {
        if (!XformQuery.Resolve(uid, ref xform, false))
            return MapCoordinates.Nullspace;

        var pose = GetRenderWorldPose(uid, xform);
        return TryGetRenderSpaceMapId(pose.CoordinateSpace, out var mapId)
            ? new MapCoordinates(pose.Position, mapId)
            : MapCoordinates.Nullspace;
    }

    /// <summary>
    /// Produces the layer samples actually consumed by the renderer. Between integer absolute-z boundaries this
    /// returns two samples at the same continuous projected position with complementary opacity.
    /// </summary>
    public int GetRenderLayerSamples(
        EntityUid uid,
        Span<RenderLayerSample> samples,
        TransformComponent? xform = null,
        IReadOnlySet<EntityUid>? visibleMaps = null)
    {
        if (samples.Length == 0 || !XformQuery.Resolve(uid, ref xform, false))
            return 0;

        var pose = GetRenderWorldPose(uid, xform);
        if (!_zLevels.TryGetMapData(pose.CoordinateSpace, out var coordinateMap, out var network))
        {
            samples[0] = new RenderLayerSample(pose.CoordinateSpace, pose.Position, pose.Rotation, 1f, 0);
            return 1;
        }

        var weights = ZLevelProjection.GetLayerWeights(pose.AbsoluteZ);
        var count = 0;
        if (weights.LowerWeight > 0f &&
            weights.LowerDepth >= 0 &&
            weights.LowerDepth < network.SortedZLevels.Count &&
            (visibleMaps == null || visibleMaps.Contains(network.SortedZLevels[weights.LowerDepth])))
        {
            var map = network.SortedZLevels[weights.LowerDepth];
            var position = ZLevelProjection.Reproject(
                pose.Position,
                coordinateMap.Depth,
                weights.LowerDepth,
                network.ProjectionOffset);
            samples[count++] = new RenderLayerSample(
                map,
                position,
                pose.Rotation,
                weights.LowerWeight,
                weights.LowerDepth);
        }

        if (weights.UpperWeight > 0f &&
            samples.Length > count &&
            weights.UpperDepth >= 0 &&
            weights.UpperDepth < network.SortedZLevels.Count &&
            (visibleMaps == null || visibleMaps.Contains(network.SortedZLevels[weights.UpperDepth])))
        {
            var map = network.SortedZLevels[weights.UpperDepth];
            var position = ZLevelProjection.Reproject(
                pose.Position,
                coordinateMap.Depth,
                weights.UpperDepth,
                network.ProjectionOffset);
            samples[count++] = new RenderLayerSample(
                map,
                position,
                pose.Rotation,
                weights.UpperWeight,
                weights.UpperDepth);
        }

        if (visibleMaps != null && count > 0)
        {
            var totalOpacity = 0f;
            for (var i = 0; i < count; i++)
                totalOpacity += samples[i].Opacity;

            // A z transition is a crossfade only while both samples are actually composited. If one adjacent map is
            // hidden by the viewport stack, normalize the surviving renderer samples so the entity never fades out.
            if (totalOpacity > ZLevelProjection.BoundaryEpsilon)
            {
                for (var i = 0; i < count; i++)
                    samples[i] = samples[i] with { Opacity = samples[i].Opacity / totalOpacity };
            }
        }
        else if (count == 1 && samples[0].Opacity < 1f)
        {
            samples[0] = samples[0] with { Opacity = 1f };
        }

        return count;
    }

    /// <summary>
    /// Gets this frame's sample for a particular map render pass.
    /// </summary>
    public bool TryGetRenderLayerSample(
        EntityUid uid,
        EntityUid layerMap,
        out RenderLayerSample sample,
        TransformComponent? xform = null,
        IReadOnlySet<EntityUid>? visibleMaps = null)
    {
        Span<RenderLayerSample> samples = stackalloc RenderLayerSample[2];
        var count = GetRenderLayerSamples(uid, samples, xform, visibleMaps);
        for (var i = 0; i < count; i++)
        {
            if (samples[i].Map == layerMap)
            {
                sample = samples[i];
                return true;
            }
        }

        sample = default;
        return false;
    }

    /// <summary>
    /// Produces the single presented sample used by an attachment drawn after the z-level stack has been
    /// composited. Only renderer layers that are visible in the viewport contribute to its opacity.
    /// </summary>
    /// <remarks>
    /// World-space attachments should use <see cref="TryGetRenderLayerSample"/> in each layer pass instead.
    /// This helper is intended for screen-space labels and similar visuals that are drawn once after the stack.
    /// </remarks>
    public bool TryGetPresentedViewSample(
        EntityUid uid,
        EntityUid viewedMap,
        IReadOnlySet<EntityUid> visibleMaps,
        out PresentedViewSample sample,
        TransformComponent? xform = null)
    {
        sample = default;
        if (viewedMap == EntityUid.Invalid ||
            visibleMaps.Count == 0 ||
            !XformQuery.Resolve(uid, ref xform, false))
        {
            return false;
        }

        Span<RenderLayerSample> layerSamples = stackalloc RenderLayerSample[2];
        var count = GetRenderLayerSamples(uid, layerSamples, xform, visibleMaps);
        var opacity = 0f;
        for (var i = 0; i < count; i++)
        {
            if (visibleMaps.Contains(layerSamples[i].Map))
                opacity += layerSamples[i].Opacity;
        }

        if (opacity <= ZLevelProjection.BoundaryEpsilon)
            return false;

        var pose = GetRenderWorldPoseForLayer(uid, viewedMap, xform);
        if (!AreRenderSpacesCompatible(pose.CoordinateSpace, viewedMap))
            return false;

        sample = new PresentedViewSample(
            pose.Position,
            pose.Rotation,
            Math.Clamp(opacity, 0f, 1f),
            pose.AbsoluteZ);
        return true;
    }

    /// <summary>
    /// Projects a point on one map's surface into another map render pass.
    /// </summary>
    /// <remarks>
    /// This is intended for map-owned visuals such as overlays and placement previews. Physics and gameplay
    /// code should continue to use the unprojected <see cref="MapCoordinates.Position"/>.
    /// </remarks>
    public bool TryProjectMapCoordinatesForLayer(
        MapCoordinates coordinates,
        EntityUid layerMap,
        out Vector2 position)
    {
        position = coordinates.Position;
        var sourceMap = _mapSystem.GetMapOrInvalid(coordinates.MapId);
        if (sourceMap == EntityUid.Invalid || layerMap == EntityUid.Invalid)
            return false;

        if (sourceMap == layerMap)
            return true;

        if (!_zLevels.TryGetMapData(sourceMap, out var source, out var network) ||
            !_zLevels.TryGetMapData(layerMap, out var target, out var targetNetwork) ||
            source.Network != target.Network)
        {
            return false;
        }

        position = ZLevelProjection.Reproject(
            coordinates.Position,
            source.Depth,
            target.Depth,
            network.ProjectionOffset);
        return true;
    }

    /// <summary>
    /// Re-expresses a render layer's visible bounds in a compatible source map.
    /// </summary>
    /// <remarks>
    /// Broadphase and component-tree data use unprojected map coordinates. Overlay renderers that gather map-owned
    /// geometry from visible z maps should use this before querying a source map.
    /// </remarks>
    public bool TryGetMapRenderBoundsForLayer(
        EntityUid sourceMap,
        EntityUid layerMap,
        in Box2Rotated layerBounds,
        out MapId sourceMapId,
        out Box2Rotated sourceBounds)
    {
        sourceMapId = MapId.Nullspace;
        sourceBounds = layerBounds;
        if (!TryComp(sourceMap, out MapComponent? map) ||
            !TryProjectMapCoordinatesForLayer(
                new MapCoordinates(Vector2.Zero, map.MapId),
                layerMap,
                out var projectedOrigin))
        {
            return false;
        }

        sourceMapId = map.MapId;
        sourceBounds.Box = sourceBounds.Box.Translated(-projectedOrigin);
        sourceBounds.Origin -= projectedOrigin;
        return true;
    }

    /// <summary>
    /// Gets the z projection and renderer layer selection for diagnostics.
    /// </summary>
    public bool TryGetZLevelRenderDebugData(EntityUid uid, out ZLevelRenderDebugData data)
    {
        if (!XformQuery.TryGetComponent(uid, out var xform))
        {
            data = default;
            return false;
        }

        var pose = GetRenderWorldPose(uid, xform);
        var simulationMap = xform.MapUid ?? EntityUid.Invalid;
        var mapDepth = _zLevels.TryGetMapDepth(simulationMap, out var depth) ? depth.Value : 0;
        var authoredLocalHeight = _zPresentationQuery.TryComp(uid, out var presentation)
            ? presentation.LocalHeight
            : pose.AbsoluteZ - mapDepth;
        Span<RenderLayerSample> samples = stackalloc RenderLayerSample[2];
        var count = GetRenderLayerSamples(uid, samples, xform);
        data = new ZLevelRenderDebugData(
            mapDepth,
            authoredLocalHeight,
            pose.AbsoluteZ - pose.ReferenceDepth,
            pose.AbsoluteZ,
            pose.Position,
            pose.CanonicalPosition,
            count,
            count > 0 ? samples[0] : default,
            count > 1 ? samples[1] : default);
        return true;
    }

    /// <summary>
    /// Re-expresses a presented pose in another map pass of the same network.
    /// </summary>
    public RenderPose GetRenderWorldPoseForLayer(EntityUid uid, EntityUid layerMap, TransformComponent? xform = null)
    {
        var pose = GetRenderWorldPose(uid, xform);
        if (!_zLevels.TryGetMapData(pose.CoordinateSpace, out var from, out var network) ||
            !_zLevels.TryGetMapData(layerMap, out var to, out var targetNetwork) ||
            from.Network != to.Network)
        {
            return pose;
        }

        return pose with
        {
            Position = ZLevelProjection.Reproject(pose.Position, from.Depth, to.Depth, network.ProjectionOffset),
            ReferenceDepth = to.Depth,
        };
    }

    private RenderPose GetRenderPoseInternal(EntityUid uid, int depth, TransformComponent? xform = null)
    {
        if (depth >= MaxTransformDepth || !XformQuery.Resolve(uid, ref xform, false))
            return default;

        if (_renderPoses.TryGetValue(uid, out var state))
        {
            var snapRotation = state.SnapRotation || _snapRenderRotations.Contains(uid);
            if (state.Type == RenderInterpolationType.PredictionCorrection)
            {
                var correctionTarget = ResolveEndpoint(state.Target, depth + 1);
                return CreateRenderPose(
                    correctionTarget.CanonicalPosition + state.CorrectionTranslation,
                    snapRotation
                        ? correctionTarget.Rotation
                        : correctionTarget.Rotation + state.CorrectionRotation,
                    state.CoordinateSpace,
                    state.Source.RenderSpace,
                    state.Target.RenderSpace,
                    state.Alpha,
                    correctionTarget.AbsoluteZ + state.CorrectionZ);
            }

            var source = ResolveEndpoint(state.Source, depth + 1);
            var target = ResolveEndpoint(state.Target, depth + 1);
            var alpha = GetSegmentAlpha(state);
            return CreateRenderPose(
                Vector2.Lerp(source.CanonicalPosition, target.CanonicalPosition, alpha),
                snapRotation
                    ? target.Rotation
                    : Angle.Lerp(source.Rotation, target.Rotation, alpha),
                state.CoordinateSpace,
                state.Source.RenderSpace,
                state.Target.RenderSpace,
                alpha,
                MathHelper.Lerp(source.AbsoluteZ, target.AbsoluteZ, alpha));
        }

        var renderSpace = xform.MapUid ?? EntityUid.Invalid;
        if (!xform.ParentUid.IsValid())
        {
            var absoluteZ = GetOwnAbsoluteZ(uid, renderSpace) ?? GetMapDepth(renderSpace);
            return CreateRenderPose(xform.LocalPosition, xform.LocalRotation, renderSpace, renderSpace, renderSpace, 1f, absoluteZ);
        }

        var parent = GetRenderPoseInternal(xform.ParentUid, depth + 1);
        var canonicalPosition = parent.CanonicalPosition + parent.Rotation.RotateVec(xform.LocalPosition);
        var childAbsoluteZ = GetOwnAbsoluteZ(uid, renderSpace) ?? parent.AbsoluteZ;
        return CreateRenderPose(
            canonicalPosition,
            parent.Rotation + xform.LocalRotation,
            parent.CoordinateSpace,
            parent.SourceRenderSpace,
            parent.TargetRenderSpace,
            parent.RenderSpaceAlpha,
            childAbsoluteZ);
    }

    private RenderPose ResolveEndpoint(in RenderPoseEndpoint endpoint, int depth)
    {
        if (!endpoint.Parent.IsValid() || depth >= MaxTransformDepth)
        {
            var absoluteZ = endpoint.AbsoluteZ ?? GetMapDepth(endpoint.RenderSpace);
            return CreateRenderPose(
                endpoint.LocalPosition,
                endpoint.LocalRotation,
                endpoint.RenderSpace,
                endpoint.RenderSpace,
                endpoint.RenderSpace,
                1f,
                absoluteZ);
        }

        var parent = GetRenderPoseInternal(endpoint.Parent, depth + 1);
        return CreateRenderPose(
            parent.CanonicalPosition + parent.Rotation.RotateVec(endpoint.LocalPosition),
            parent.Rotation + endpoint.LocalRotation,
            parent.CoordinateSpace,
            endpoint.RenderSpace,
            endpoint.RenderSpace,
            1f,
            endpoint.AbsoluteZ ?? parent.AbsoluteZ);
    }

    private static float GetSegmentAlpha(RenderPoseState state)
    {
        var remaining = 1f - state.InterpolationStartAlpha;
        if (remaining <= float.Epsilon)
            return 1f;

        return Math.Clamp((state.Alpha - state.InterpolationStartAlpha) / remaining, 0f, 1f);
    }

    private RenderPose ResolveLastRenderedEndpoint(in RenderPoseEndpoint endpoint, int depth)
    {
        if (!endpoint.Parent.IsValid() || depth >= MaxTransformDepth)
        {
            var absoluteZ = endpoint.AbsoluteZ ?? GetMapDepth(endpoint.RenderSpace);
            return CreateRenderPose(
                endpoint.LocalPosition,
                endpoint.LocalRotation,
                endpoint.RenderSpace,
                endpoint.RenderSpace,
                endpoint.RenderSpace,
                1f,
                absoluteZ);
        }

        var parent = GetLastRenderedPoseInternal(endpoint.Parent, depth + 1);
        return CreateRenderPose(
            parent.CanonicalPosition + parent.Rotation.RotateVec(endpoint.LocalPosition),
            parent.Rotation + endpoint.LocalRotation,
            parent.CoordinateSpace,
            endpoint.RenderSpace,
            endpoint.RenderSpace,
            1f,
            endpoint.AbsoluteZ ?? parent.AbsoluteZ);
    }

    private RenderPose GetLastRenderedPoseInternal(EntityUid uid, int depth, TransformComponent? xform = null)
    {
        if (depth >= MaxTransformDepth || !XformQuery.Resolve(uid, ref xform, false))
            return default;

        if (_renderPoses.TryGetValue(uid, out var state))
            return state.LastRendered;

        var renderSpace = xform.MapUid ?? EntityUid.Invalid;
        if (!xform.ParentUid.IsValid())
        {
            var absoluteZ = GetOwnAbsoluteZ(uid, renderSpace) ?? GetMapDepth(renderSpace);
            return CreateRenderPose(xform.LocalPosition, xform.LocalRotation, renderSpace, renderSpace, renderSpace, 1f, absoluteZ);
        }

        var parent = GetLastRenderedPoseInternal(xform.ParentUid, depth + 1);
        return CreateRenderPose(
            parent.CanonicalPosition + parent.Rotation.RotateVec(xform.LocalPosition),
            parent.Rotation + xform.LocalRotation,
            parent.CoordinateSpace,
            parent.SourceRenderSpace,
            parent.TargetRenderSpace,
            parent.RenderSpaceAlpha,
            GetOwnAbsoluteZ(uid, renderSpace) ?? parent.AbsoluteZ);
    }

    private bool TryCreateEndpoint(
        EntityUid uid,
        in EntityCoordinates coordinates,
        Angle rotation,
        out RenderPoseEndpoint endpoint,
        float? localHeightOverride = null)
    {
        if (!coordinates.EntityId.IsValid()
            || !XformQuery.TryGetComponent(coordinates.EntityId, out var parent)
            || parent.MapUid is not { } renderSpace)
        {
            endpoint = default;
            return false;
        }

        float? absoluteZ = null;
        if (_zLevels.TryGetMapDepth(renderSpace, out var depth))
        {
            var localHeight = localHeightOverride;
            if (localHeight == null && _zPresentationQuery.TryComp(uid, out var presentation))
                localHeight = presentation.LocalHeight + presentation.VisualHeight;

            if (localHeight is { } height)
                absoluteZ = ZLevelProjection.GetAbsoluteZ(depth.Value, height);
        }

        endpoint = new RenderPoseEndpoint(coordinates.EntityId, coordinates.Position, rotation, renderSpace, absoluteZ);
        return true;
    }

    private float? GetOwnAbsoluteZ(EntityUid uid, EntityUid renderSpace)
    {
        if (!_zPresentationQuery.TryComp(uid, out var presentation) ||
            !_zLevels.TryGetMapDepth(renderSpace, out var depth))
        {
            return null;
        }

        return ZLevelProjection.GetAbsoluteZ(depth.Value, presentation.LocalHeight + presentation.VisualHeight);
    }

    private float GetMapDepth(EntityUid renderSpace)
        => _zLevels.TryGetMapDepth(renderSpace, out var depth) ? depth.Value : 0f;

    private RenderPose CreateRenderPose(
        Vector2 canonicalPosition,
        Angle rotation,
        EntityUid coordinateSpace,
        EntityUid sourceRenderSpace,
        EntityUid targetRenderSpace,
        float renderSpaceAlpha,
        float absoluteZ)
    {
        var position = canonicalPosition;
        var referenceDepth = 0;
        var projectionOffset = Vector2.Zero;
        if (_zLevels.TryGetMapData(coordinateSpace, out var zMap, out var network))
        {
            referenceDepth = zMap.Depth;
            projectionOffset = network.ProjectionOffset;
            position = ZLevelProjection.Project(canonicalPosition, absoluteZ, referenceDepth, projectionOffset);
        }

        return new RenderPose(
            position,
            canonicalPosition,
            rotation,
            coordinateSpace,
            sourceRenderSpace,
            targetRenderSpace,
            renderSpaceAlpha,
            absoluteZ,
            referenceDepth,
            projectionOffset);
    }

    /// <summary>
    /// Gets the stable coordinate space used to render between two render spaces.
    /// </summary>
    public bool TryGetCommonRenderSpace(EntityUid first, EntityUid second, out EntityUid commonSpace)
    {
        if (!first.IsValid() || !second.IsValid())
        {
            commonSpace = EntityUid.Invalid;
            return false;
        }

        if (first == second)
        {
            commonSpace = first;
            return true;
        }

        var args = new RenderSpaceCompatibilityEvent(first, second);
        RenderSpaceCompatibility?.Invoke(ref args);
        commonSpace = args.CommonSpace;
        return commonSpace.IsValid();
    }

    /// <summary>
    /// Checks whether two render spaces share a common coordinate space.
    /// </summary>
    public bool AreRenderSpacesCompatible(EntityUid first, EntityUid second)
        => TryGetCommonRenderSpace(first, second, out _);

    private bool TryGetRenderSpaceMapId(EntityUid renderSpace, out MapId mapId)
    {
        if (renderSpace.IsValid() && XformQuery.TryGetComponent(renderSpace, out var xform))
        {
            mapId = xform.MapID;
            return mapId != MapId.Nullspace;
        }

        mapId = MapId.Nullspace;
        return false;
    }

    /// <summary>
    /// Discards active render interpolation for an entity and optionally its descendants.
    /// </summary>
    public void SnapRenderPose(EntityUid uid, bool recursive = false)
    {
        _renderPoses.Remove(uid);
        _snapRenderRotations.Remove(uid);

        if (!recursive || !XformQuery.TryGetComponent(uid, out var xform))
            return;

        var children = xform.ChildEnumerator;
        while (children.MoveNext(out var child))
        {
            SnapRenderPose(child, true);
        }
    }

    /// <summary>
    /// Snaps rendered rotation to its simulation target without interrupting position interpolation.
    /// </summary>
    public void SnapRenderRotation(EntityUid uid)
    {
        ref var state = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
        if (Unsafe.IsNullRef(ref state))
        {
            _snapRenderRotations.Add(uid);
            return;
        }

        SnapRenderRotation(ref state);
    }

    private void SnapRenderRotation(ref RenderPoseState state)
    {
        var target = ResolveEndpoint(state.Target, 0);
        state.SnapRotation = true;
        state.CorrectionRotation = Angle.Zero;
        state.CorrectionAnchor = state.CorrectionAnchor with { Rotation = target.Rotation };
        state.LastRendered = state.LastRendered with { Rotation = target.Rotation };
    }

    /// <summary>
    /// Enlarges a render query enough to include sparse poses whose simulation target lies outside the viewport.
    /// </summary>
    [Pure]
    public Box2 GetRenderCullingBounds(MapId mapId, in Box2Rotated bounds)
        => GetRenderCullingBounds(mapId, bounds.CalcBoundingBox());

    /// <summary>
    /// Enlarges a render query to include active render offsets.
    /// </summary>
    [Pure]
    public Box2 GetRenderCullingBounds(MapId mapId, in Box2 bounds)
    {
        return _cullingMargins.TryGetValue(mapId, out var margin) && margin > 0f
            ? bounds.Enlarged(margin)
            : bounds;
    }

    /// <summary>
    /// Enumerates active render interpolation state for debugging.
    /// </summary>
    public IEnumerable<RenderPoseDebugData> GetRenderPoseDebugData()
    {
        foreach (var uid in _renderPoses.Keys)
        {
            if (TryGetRenderPoseDebugData(uid, out var data))
                yield return data;
        }
    }

    /// <summary>
    /// Gets active render interpolation state for an entity.
    /// </summary>
    public bool TryGetRenderPoseDebugData(EntityUid uid, out RenderPoseDebugData data)
    {
        if (!_renderPoses.TryGetValue(uid, out var state)
            || !XformQuery.TryGetComponent(uid, out var xform))
        {
            data = default;
            return false;
        }

        var simulation = GetWorldPositionRotation(xform);
        var source = ResolveEndpoint(state.Source, 0);
        var target = ResolveEndpoint(state.Target, 0);
        var rendered = GetRenderPoseInternal(uid, 0, xform);
        var renderSpace = xform.MapUid ?? EntityUid.Invalid;
        var simulationZ = GetOwnAbsoluteZ(uid, renderSpace) ?? GetMapDepth(renderSpace);
        data = new RenderPoseDebugData(
            uid,
            state.Source.Parent,
            CreateRenderPose(
                simulation.WorldPosition,
                simulation.WorldRotation,
                renderSpace,
                renderSpace,
                renderSpace,
                1f,
                simulationZ),
            rendered,
            source,
            target,
            state.Target.Parent,
            state.CoordinateSpace,
            state.Source.RenderSpace,
            state.Target.RenderSpace,
            state.Type,
            state.Alpha,
            state.CorrectionTranslation,
            state.CorrectionRotation,
            state.CorrectionZ);
        return true;
    }

    private struct RenderPoseState
    {
        public RenderPoseEndpoint Source;
        public RenderPoseEndpoint Target;
        public RenderPose LastRendered;
        public RenderPose CorrectionAnchor;
        public EntityUid CoordinateSpace;
        public GameTick ChangeTick;
        public RenderInterpolationType Type;
        public float Alpha;
        public float InterpolationStartAlpha;
        public float LastFramePhase;
        public Vector2 CorrectionTranslation;
        public Angle CorrectionRotation;
        public float CorrectionZ;
        public float CorrectionRemaining;
        public bool PendingPredictionReplay;
        public bool SnapRotation;
    }
}

/// <summary>
/// A rendered transform and its coordinate and layer ownership.
/// </summary>
public readonly record struct RenderPose(
    Vector2 Position,
    Vector2 CanonicalPosition,
    Angle Rotation,
    EntityUid CoordinateSpace,
    EntityUid SourceRenderSpace,
    EntityUid TargetRenderSpace,
    float RenderSpaceAlpha,
    float AbsoluteZ,
    int ReferenceDepth,
    Vector2 ProjectionOffset)
{
    public RenderPose(Vector2 position, Angle rotation, EntityUid renderSpace)
        : this(position, position, rotation, renderSpace, renderSpace, renderSpace, 1f, 0f, 0, Vector2.Zero)
    {
    }
}

internal readonly record struct RenderPoseEndpoint(
    EntityUid Parent,
    Vector2 LocalPosition,
    Angle LocalRotation,
    EntityUid RenderSpace,
    float? AbsoluteZ);

/// <summary>
/// One renderer-facing sample of an entity on an integer z layer.
/// </summary>
public readonly record struct RenderLayerSample(
    EntityUid Map,
    Vector2 Position,
    Angle Rotation,
    float Opacity,
    int Depth);

/// <summary>
/// A presented entity pose expressed in the controlling eye's map after combining the renderer layers visible
/// in that viewport.
/// </summary>
public readonly record struct PresentedViewSample(
    Vector2 Position,
    Angle Rotation,
    float Opacity,
    float AbsoluteZ);

/// <summary>
/// Renderer-selected z presentation values for debug overlays and commands.
/// </summary>
public readonly record struct ZLevelRenderDebugData(
    int MapDepth,
    float AuthoredLocalHeight,
    float PresentedLocalHeight,
    float AbsoluteZ,
    Vector2 ProjectedPosition,
    Vector2 CanonicalPosition,
    int LayerCount,
    RenderLayerSample FirstLayer,
    RenderLayerSample SecondLayer);

/// <summary>
/// The source of an active render interpolation.
/// </summary>
public enum RenderInterpolationType : byte
{
    NetworkInterpolation,
    PredictionInterpolation,
    PredictionCorrection,
}

/// <summary>
/// Allows a renderer to supply a common coordinate space for two render spaces.
/// </summary>
public struct RenderSpaceCompatibilityEvent
{
    /// <summary>
    /// The source render space.
    /// </summary>
    public readonly EntityUid First;

    /// <summary>
    /// The destination render space.
    /// </summary>
    public readonly EntityUid Second;

    /// <summary>
    /// A common coordinate space, or invalid if these spaces are incompatible.
    /// </summary>
    public EntityUid CommonSpace;

    public RenderSpaceCompatibilityEvent(EntityUid first, EntityUid second)
    {
        First = first;
        Second = second;
        CommonSpace = EntityUid.Invalid;
    }
}

/// <summary>
/// Diagnostic data for an active render interpolation.
/// </summary>
public readonly record struct RenderPoseDebugData(
    EntityUid Entity,
    EntityUid SourceParent,
    RenderPose Simulation,
    RenderPose Rendered,
    RenderPose Source,
    RenderPose Target,
    EntityUid Parent,
    EntityUid CoordinateSpace,
    EntityUid SourceRenderSpace,
    EntityUid TargetRenderSpace,
    RenderInterpolationType Type,
    float Alpha,
    Vector2 CorrectionTranslation,
    Angle CorrectionRotation,
    float CorrectionZ);
