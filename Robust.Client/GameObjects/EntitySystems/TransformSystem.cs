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

    [Dependency] private EntityQuery<MapGridComponent> _renderGridQuery;

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

        if (xform.Deleted || !TryCreateEndpoint(args.NewPosition, args.NewRotation, out var target))
        {
            _renderPoses.Remove(uid);
            return;
        }

        ref var existing = ref CollectionsMarshal.GetValueRefOrNullRef(_renderPoses, uid);
        var hasExisting = !Unsafe.IsNullRef(ref existing);

        if (!TryCreateEndpoint(args.OldPosition, args.OldRotation, out var oldEndpoint))
        {
            _renderPoses.Remove(uid);
            return;
        }

        var source = oldEndpoint;
        RenderPose rendered;

        if (hasExisting)
        {
            // Use the last pose that reached the screen.
            rendered = existing.LastRendered;
            var parentOrRenderSpaceChanged = oldEndpoint.Parent != target.Parent
                                             || oldEndpoint.RenderSpace != target.RenderSpace;
            var predictionRollbackOrCorrection = existing.Type == RenderInterpolationType.PredictionCorrection
                                                 || existing.PendingPredictionReplay
                                                 || (_timing.ApplyingState && existing.Type == RenderInterpolationType.PredictionInterpolation);

            // Multiple changes in one simulation tick retain the original source: A -> B -> C is A -> C.
            if (existing.Type != RenderInterpolationType.PredictionCorrection
                && existing.ChangeTick == _timing.CurTick
                && existing.Alpha <= existing.InterpolationStartAlpha
                && !existing.PendingPredictionReplay)
            {
                source = existing.Source;
            }
            else if ((parentOrRenderSpaceChanged || predictionRollbackOrCorrection)
                     && !TryBindRenderPoseToParent(rendered, args.OldPosition.EntityId, out source))
            {
                source = oldEndpoint;
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

            // Replay without render state must not interpolate from a temporary rollback position.
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
        state.CorrectionRemaining = 0f;
    }

    private void RebaseTickInterpolation(
        ref RenderPoseState state,
        in RenderPoseEndpoint source,
        in RenderPoseEndpoint target,
        in RenderPose rendered,
        EntityUid coordinateSpace)
    {
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
               || !sourcePose.Rotation.EqualsApprox(targetPose.Rotation);
    }

    private void RebaseCorrection(ref RenderPoseState state, in RenderPoseEndpoint target)
    {
        var targetPose = ResolveEndpoint(target, 0);
        state.CorrectionTranslation = state.CorrectionAnchor.Position - targetPose.Position;
        state.CorrectionRotation = state.SnapRotation
            ? Angle.Zero
            : Angle.ShortestDistance(targetPose.Rotation, state.CorrectionAnchor.Rotation);
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
                && Math.Abs(state.CorrectionRotation.Theta) < _minCorrectionRotation)
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
        var phase = Math.Clamp(
            (float)(_timing.TickRemainder.TotalSeconds / _timing.TickPeriod.TotalSeconds),
            0f,
            1f);

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
                return new RenderPose(
                    correctionTarget.Position + state.CorrectionTranslation,
                    snapRotation
                        ? correctionTarget.Rotation
                        : correctionTarget.Rotation + state.CorrectionRotation,
                    state.CoordinateSpace,
                    state.Source.RenderSpace,
                    state.Target.RenderSpace,
                    state.Alpha);
            }

            var source = ResolveEndpoint(state.Source, depth + 1);
            var target = ResolveEndpoint(state.Target, depth + 1);
            var alpha = GetSegmentAlpha(state);
            return new RenderPose(
                Vector2.Lerp(source.Position, target.Position, alpha),
                snapRotation
                    ? target.Rotation
                    : Angle.Lerp(source.Rotation, target.Rotation, alpha),
                state.CoordinateSpace,
                state.Source.RenderSpace,
                state.Target.RenderSpace,
                alpha);
        }

        var renderSpace = xform.MapUid ?? EntityUid.Invalid;
        if (!xform.ParentUid.IsValid())
            return new RenderPose(xform.LocalPosition, xform.LocalRotation, renderSpace);

        var parent = GetRenderPoseInternal(xform.ParentUid, depth + 1);
        return new RenderPose(
            parent.Position + parent.Rotation.RotateVec(xform.LocalPosition),
            parent.Rotation + xform.LocalRotation,
            parent.CoordinateSpace,
            parent.SourceRenderSpace,
            parent.TargetRenderSpace,
            parent.RenderSpaceAlpha);
    }

    private RenderPose ResolveEndpoint(in RenderPoseEndpoint endpoint, int depth)
    {
        if (!endpoint.Parent.IsValid() || depth >= MaxTransformDepth)
            return new RenderPose(endpoint.LocalPosition, endpoint.LocalRotation, endpoint.RenderSpace);

        var parent = GetRenderPoseInternal(endpoint.Parent, depth + 1);
        return new RenderPose(
            parent.Position + parent.Rotation.RotateVec(endpoint.LocalPosition),
            parent.Rotation + endpoint.LocalRotation,
            parent.CoordinateSpace,
            endpoint.RenderSpace,
            endpoint.RenderSpace,
            1f);
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
            return new RenderPose(endpoint.LocalPosition, endpoint.LocalRotation, endpoint.RenderSpace);

        var parent = GetLastRenderedPoseInternal(endpoint.Parent, depth + 1);
        return new RenderPose(
            parent.Position + parent.Rotation.RotateVec(endpoint.LocalPosition),
            parent.Rotation + endpoint.LocalRotation,
            parent.CoordinateSpace,
            endpoint.RenderSpace,
            endpoint.RenderSpace,
            1f);
    }

    private RenderPose GetLastRenderedPoseInternal(EntityUid uid, int depth, TransformComponent? xform = null)
    {
        if (depth >= MaxTransformDepth || !XformQuery.Resolve(uid, ref xform, false))
            return default;

        if (_renderPoses.TryGetValue(uid, out var state))
            return state.LastRendered;

        var renderSpace = xform.MapUid ?? EntityUid.Invalid;
        if (!xform.ParentUid.IsValid())
            return new RenderPose(xform.LocalPosition, xform.LocalRotation, renderSpace);

        var parent = GetLastRenderedPoseInternal(xform.ParentUid, depth + 1);
        return parent with { Position = parent.Position + parent.Rotation.RotateVec(xform.LocalPosition), Rotation = parent.Rotation + xform.LocalRotation };
    }

    private bool TryCreateEndpoint(
        in EntityCoordinates coordinates,
        Angle rotation,
        out RenderPoseEndpoint endpoint)
    {
        if (!coordinates.EntityId.IsValid()
            || !XformQuery.TryGetComponent(coordinates.EntityId, out var parent)
            || parent.MapUid is not { } renderSpace)
        {
            endpoint = default;
            return false;
        }

        endpoint = new RenderPoseEndpoint(coordinates.EntityId, coordinates.Position, rotation, renderSpace);
        return true;
    }

    private bool TryBindRenderPoseToParent(in RenderPose pose, EntityUid parentUid, out RenderPoseEndpoint endpoint)
    {
        if (!parentUid.IsValid()
            || !XformQuery.TryGetComponent(parentUid, out var parentXform)
            || parentXform.MapUid is not { } renderSpace)
        {
            endpoint = default;
            return false;
        }

        var parent = GetLastRenderedPoseInternal(parentUid, 0, parentXform);
        var localPosition = (-parent.Rotation).RotateVec(pose.Position - parent.Position);
        var localRotation = pose.Rotation - parent.Rotation;
        endpoint = new RenderPoseEndpoint(parentUid, localPosition, localRotation, renderSpace);
        return true;
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
    internal IEnumerable<RenderPoseDebugData> GetRenderPoseDebugData()
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
    internal bool TryGetRenderPoseDebugData(EntityUid uid, out RenderPoseDebugData data)
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
        data = new RenderPoseDebugData(
            uid,
            new RenderPose(simulation.WorldPosition, simulation.WorldRotation, xform.MapUid ?? EntityUid.Invalid),
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
            state.CorrectionRotation);
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
    Angle Rotation,
    EntityUid CoordinateSpace,
    EntityUid SourceRenderSpace,
    EntityUid TargetRenderSpace,
    float RenderSpaceAlpha)
{
    public RenderPose(Vector2 position, Angle rotation, EntityUid renderSpace)
        : this(position, rotation, renderSpace, renderSpace, renderSpace, 1f)
    {
    }
}

internal readonly record struct RenderPoseEndpoint(
    EntityUid Parent,
    Vector2 LocalPosition,
    Angle LocalRotation,
    EntityUid RenderSpace);

/// <summary>
/// The source of an active render interpolation.
/// </summary>
internal enum RenderInterpolationType : byte
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
internal readonly record struct RenderPoseDebugData(
    EntityUid Entity,
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
    Angle CorrectionRotation);
