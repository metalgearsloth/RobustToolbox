using System;
using System.Collections.Generic;
using System.Numerics;
using JetBrains.Annotations;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Timing;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;

namespace Robust.Client.GameObjects
{
    /// <summary>
    ///     Handles interpolation of transform positions.
    /// </summary>
    [UsedImplicitly]
    public sealed partial class TransformSystem : SharedTransformSystem
    {
        // Max distance per tick how far an entity can move before it is considered teleporting.
        // TODO: Make these values somehow dependent on server TPS.
        private const float MaxInterpolationDistance = 2.0f;
        private const float MaxInterpolationDistanceSquared = MaxInterpolationDistance * MaxInterpolationDistance;

        private const float MinInterpolationDistance = 0.001f;
        private const float MinInterpolationDistanceSquared = MinInterpolationDistance * MinInterpolationDistance;

        [Dependency] private IGameTiming _gameTiming = default!;

        // Only keep track of transforms actively lerping.
        // Much faster than iterating 3000+ transforms every frame.
        [ViewVariables] private readonly List<Entity<TransformComponent>> _lerpingTransforms = new();

        private readonly Dictionary<EntityUid, RenderCorrection> _renderCorrections = new();
        private readonly Dictionary<EntityUid, RenderPose> _stateRenderOrigins = new();
        private readonly Dictionary<EntityUid, PredictionPose> _predictionOrigins = new();
        private readonly HashSet<EntityUid> _predictedTransforms = new();

        private GameTick _predictionComparisonTick;
        private bool _capturingPredictionCorrection;
        private bool _predictionResetOccurred;
        private bool _stateLossCorrection;

        private struct RenderCorrection
        {
            public Vector2 Position;
            public Angle Rotation;
            public float TimeRemaining;
        }

        private readonly record struct RenderPose(Vector2 Position, Angle Rotation);
        private readonly record struct PredictionPose(EntityUid Parent, Vector2 Position, Angle Rotation);

        public void Reset()
        {
            foreach (var (_, xform) in _lerpingTransforms)
            {
                xform.ActivelyLerping = false;
                xform.NextPosition = null;
                xform.NextRotation = null;
                xform.LerpParent = EntityUid.Invalid;
            }

            _lerpingTransforms.Clear();

            if (_capturingPredictionCorrection)
            {
                _predictionResetOccurred = true;
                return;
            }

            _renderCorrections.Clear();
            _predictionOrigins.Clear();
            _predictedTransforms.Clear();
            _predictionResetOccurred = false;
        }

        public void BeginPredictionCorrection()
        {
            _capturingPredictionCorrection = true;
            _predictionResetOccurred = false;
            _predictionComparisonTick = _gameTiming.CurTick;
            _predictionOrigins.Clear();

            foreach (var uid in _predictedTransforms)
            {
                if (!XformQuery.TryGetComponent(uid, out var xform)
                    || xform.Deleted
                    || !xform.Initialized
                    || !xform.ParentUid.IsValid())
                {
                    continue;
                }

                _predictionOrigins[uid] = new PredictionPose(
                    xform.ParentUid,
                    GetRenderLocalPosition(xform),
                    GetRenderLocalRotation(xform));
            }

            _predictedTransforms.Clear();
        }

        public void CapturePredictionReplayTick()
        {
            if (!_capturingPredictionCorrection
                || !_predictionResetOccurred
                || _predictionOrigins.Count == 0
                || _gameTiming.CurTick != _predictionComparisonTick)
            {
                return;
            }

            foreach (var (uid, origin) in _predictionOrigins)
            {
                if (!XformQuery.TryGetComponent(uid, out var xform)
                    || xform.Deleted
                    || !xform.Initialized
                    || xform.ParentUid != origin.Parent)
                {
                    continue;
                }

                AddRenderCorrection(
                    uid,
                    origin.Position - GetRenderLocalPosition(xform, false),
                    Angle.ShortestDistance(GetRenderLocalRotation(xform, false), origin.Rotation),
                    GetCorrectionTime());
            }

            _predictionOrigins.Clear();
        }

        public void EndPredictionCorrection()
        {
            _capturingPredictionCorrection = false;
            _predictionResetOccurred = false;
            _predictionOrigins.Clear();
        }

        public void NotifyStateMissing()
        {
            for (var i = 0; i < _lerpingTransforms.Count; i++)
            {
                var (uid, xform) = _lerpingTransforms[i];

                if (xform.PredictedLerp
                    || !CanRenderLerp(xform)
                    || xform.Deleted
                    || !xform.Initialized)
                {
                    continue;
                }

                var renderPosition = GetRenderLerpEndPosition(xform) + GetRenderCorrection(uid).Position;
                var renderRotation = GetRenderLerpEndRotation(xform) + GetRenderCorrection(uid).Rotation;

                SetRenderCorrection(
                    uid,
                    renderPosition - xform._localPosition,
                    Angle.ShortestDistance(xform._localRotation, renderRotation),
                    float.PositiveInfinity);

                xform.ActivelyLerping = false;
                xform.NextPosition = null;
                xform.NextRotation = null;
                xform.LerpParent = EntityUid.Invalid;
                _lerpingTransforms.RemoveSwap(i);
                i -= 1;
            }
        }

        public void SetStateLossCorrection(bool value)
        {
            _stateLossCorrection = value;
        }

        protected override void BeforeHandleState(EntityUid uid, TransformComponent xform, EntityUid newParent)
        {
            if (!_gameTiming.ApplyingState
                || xform.Deleted
                || !xform.Initialized
                || !xform.ParentUid.IsValid()
                || !newParent.IsValid()
                || (!_stateLossCorrection && xform.ParentUid == newParent))
            {
                return;
            }

            var (position, rotation) = GetRenderWorldPositionRotation(xform);
            _stateRenderOrigins[uid] = new RenderPose(position, rotation);
        }

        protected override void AfterHandleState(EntityUid uid, TransformComponent xform)
        {
            if (!_stateRenderOrigins.Remove(uid, out var origin))
                return;

            SetRenderCorrectionFromWorldPose(uid, xform, origin, GetCorrectionTime());
        }

        public override void ActivateLerp(EntityUid uid, TransformComponent xform)
        {
            // This lerping logic is pretty convoluted and generally assumes that the client does not mispredict.
            // A more foolproof solution would be to just cache the coordinates at which any given entity was most
            // recently rendered and using that as the lerp origin. However that'd require enumerating over all entities
            // every tick which is pretty icky.

            // The general considerations are:
            // - If the client receives a server state for an entity moving from a->b and predicts nothing else, then it
            //   should show the entity lerping.
            // - If the client predicts an entity will move while already lerping due to a state-application, it should
            //   clear the state's lerp, under the assumption that the client predicted the state and already rendered
            //   the entity in the state's final position.
            // - If the client predicts that an entity moves, then we only lerp if this is the first time that the tick
            //   was predicted. I.e., we assume the entity was already rendered in the final position that was
            //   previously predicted.
            // - If the client predicts that an entity should lerp twice in the same tick, then we need to combine them.
            //   I.e. moving from a->b then b->c, the client should lerp from a->c.

            // If the client predicts an entity moves while already lerping, it should clear the
            // predict a->b, lerp a->b
            // predicted a->b, then predict b->c. Lerp b->c
            // predicted a->b, then predict b->c. Lerp b->c
            // predicted a->b, predicted b->c, then predict c->d. Lerp c->d
            // server state a->b, then predicted b->c, lerp b->c
            // server state a->b, then predicted b->c, then predict d, lerp b->c

            if (_gameTiming.ApplyingState)
            {
                if (xform.ActivelyLerping)
                {
                    // This should not happen, but can happen if some bad component state application code modifies an entity's coordinates.
                    Log.Error($"Entity {(ToPrettyString(uid))} tried to lerp twice while applying component states.");
                    return;
                }

                _lerpingTransforms.Add((uid, xform));
                xform.ActivelyLerping = true;
                xform.PredictedLerp = false;
                xform.LerpParent = xform.ParentUid;
                xform.PrevRotation = xform._localRotation;
                xform.PrevPosition = xform._localPosition;
                xform.LastLerp = _gameTiming.CurTick;
                return;
            }

            _predictedTransforms.Add(uid);
            xform.LastLerp = _gameTiming.CurTick;
            if (!_gameTiming.IsFirstTimePredicted)
            {
                xform.ActivelyLerping = false;
                return;
            }

            if (!xform.ActivelyLerping)
            {
                _lerpingTransforms.Add((uid, xform));
                xform.ActivelyLerping = true;
                xform.PredictedLerp = true;
                xform.PrevRotation = xform._localRotation;
                xform.PrevPosition = xform._localPosition;
                xform.LerpParent = xform.ParentUid;
                return;
            }

            if (!xform.PredictedLerp || xform.LerpParent != xform.ParentUid)
            {
                // Existing lerp was not due to prediction, but due to state application. That lerp should already
                // have been rendered, so we will start a new lerp from the current position.
                xform.PrevRotation = xform._localRotation;
                xform.PrevPosition = xform._localPosition;
                xform.LerpParent = xform.ParentUid;
            }
        }

        public override void FrameUpdate(float frameTime)
        {
            base.FrameUpdate(frameTime);

            for (var i = 0; i < _lerpingTransforms.Count; i++)
            {
                var (_, transform) = _lerpingTransforms[i];
                var found = false;

                // Only lerp if parent didn't change.
                // E.g. entering lockers would do it.
                if (CanRenderLerp(transform))
                {
                    if (transform.NextPosition != null)
                    {
                        var lerpDest = transform.NextPosition.Value;
                        var lerpSource = transform.PrevPosition;
                        var distance = (lerpDest - lerpSource).LengthSquared();

                        if (distance is > MinInterpolationDistanceSquared and < MaxInterpolationDistanceSquared)
                            found = true;
                    }

                    if (transform.NextRotation != null)
                        found = true;
                }

                // Transforms only get removed from the lerp list if they no longer are in here.
                // This is much easier than having the transform itself tell us to remove it.
                if (!found)
                {
                    // Transform is no longer lerping, remove.
                    transform.ActivelyLerping = false;
                    _lerpingTransforms.RemoveSwap(i);
                    i -= 1;
                }
            }

            DecayRenderCorrections(frameTime);
        }

        public Vector2 GetRenderWorldPosition(EntityUid uid)
        {
            return GetRenderWorldPositionRotation(XformQuery.GetComponent(uid)).WorldPosition;
        }

        public Vector2 GetRenderWorldPosition(TransformComponent component)
        {
            return GetRenderWorldPositionRotation(component).WorldPosition;
        }

        public MapCoordinates GetRenderMapCoordinates(EntityUid uid, TransformComponent? xform = null)
        {
            if (!XformQuery.Resolve(uid, ref xform))
                return MapCoordinates.Nullspace;

            return GetRenderMapCoordinates(xform);
        }

        public MapCoordinates GetRenderMapCoordinates(TransformComponent xform)
        {
            return new MapCoordinates(GetRenderWorldPosition(xform), xform.MapID);
        }

        public (Vector2 WorldPosition, Angle WorldRotation) GetRenderWorldPositionRotation(
            TransformComponent component)
        {
            return GetRenderWorldPositionRotation(component, XformQuery, true);
        }

        public (Vector2 WorldPosition, Angle WorldRotation) GetRenderWorldPositionRotation(
            TransformComponent component,
            EntityQuery<TransformComponent> xformQuery)
        {
            return GetRenderWorldPositionRotation(component, xformQuery, true);
        }

        public (Vector2 Position, Angle Rotation) GetRenderRelativePositionRotation(
            TransformComponent component,
            EntityUid relative,
            EntityQuery<TransformComponent> xformQuery)
        {
            var rot = GetRenderLocalRotation(component);
            var pos = GetRenderLocalPosition(component);
            var xform = component;

            while (xform.ParentUid != relative)
            {
                xform = xformQuery.GetComponent(xform.ParentUid);
                var parentRot = GetRenderLocalRotation(xform);
                pos = parentRot.RotateVec(pos) + GetRenderLocalPosition(xform);
                rot += parentRot;
            }

            return (pos, rot);
        }

        public Vector2 GetRenderLocalPosition(TransformComponent component)
        {
            return GetRenderLocalPosition(component, true);
        }

        public Angle GetRenderLocalRotation(TransformComponent component)
        {
            return GetRenderLocalRotation(component, true);
        }

        private (Vector2 WorldPosition, Angle WorldRotation) GetRenderWorldPositionRotation(
            TransformComponent component,
            EntityQuery<TransformComponent> xformQuery,
            bool includeOwnCorrection)
        {
            var pos = GetRenderLocalPosition(component, includeOwnCorrection);
            var angle = GetRenderLocalRotation(component, includeOwnCorrection);

            while (component.ParentUid != component.MapUid && component.ParentUid.IsValid())
            {
                component = xformQuery.GetComponent(component.ParentUid);

                var parentRot = GetRenderLocalRotation(component);
                pos = parentRot.RotateVec(pos) + GetRenderLocalPosition(component);
                angle += parentRot;
            }

            return (pos, angle);
        }

        private Vector2 GetRenderLocalPosition(TransformComponent component, bool includeCorrection)
        {
            var position = component._localPosition;

            if (CanRenderLerp(component) && component.NextPosition != null)
            {
                var lerpDest = component.NextPosition.Value;
                var lerpSource = component.PrevPosition;
                var distance = (lerpDest - lerpSource).LengthSquared();

                if (distance is > MinInterpolationDistanceSquared and < MaxInterpolationDistanceSquared)
                    position = Vector2.Lerp(lerpSource, lerpDest, GetTickStep());
            }

            if (includeCorrection)
                position += GetRenderCorrection(component.Owner).Position;

            return position;
        }

        private Angle GetRenderLocalRotation(TransformComponent component, bool includeCorrection)
        {
            var rotation = component._localRotation;

            if (CanRenderLerp(component) && component.NextRotation != null)
                rotation = Angle.Lerp(component.PrevRotation, component.NextRotation.Value, GetTickStep());

            if (includeCorrection)
                rotation += GetRenderCorrection(component.Owner).Rotation;

            return rotation;
        }

        private Vector2 GetRenderLerpEndPosition(TransformComponent component)
        {
            if (!CanRenderLerp(component) || component.NextPosition == null)
                return component._localPosition;

            var lerpDest = component.NextPosition.Value;
            var lerpSource = component.PrevPosition;
            var distance = (lerpDest - lerpSource).LengthSquared();

            if (distance is <= MinInterpolationDistanceSquared or >= MaxInterpolationDistanceSquared)
                return component._localPosition;

            return lerpDest;
        }

        private Angle GetRenderLerpEndRotation(TransformComponent component)
        {
            return CanRenderLerp(component) && component.NextRotation != null
                ? component.NextRotation.Value
                : component._localRotation;
        }

        private bool CanRenderLerp(TransformComponent component)
        {
            return component.ActivelyLerping
                   && component.LerpParent == component.ParentUid
                   && component.ParentUid.IsValid()
                   && !component.Deleted;
        }

        private float GetTickStep()
        {
            return (float) (_gameTiming.TickRemainder.TotalSeconds / _gameTiming.TickPeriod.TotalSeconds);
        }

        private float GetCorrectionTime()
        {
            return (float) _gameTiming.TickPeriod.TotalSeconds;
        }

        private RenderCorrection GetRenderCorrection(EntityUid uid)
        {
            return _renderCorrections.GetValueOrDefault(uid);
        }

        private void AddRenderCorrection(EntityUid uid, Vector2 position, Angle rotation, float time)
        {
            var current = GetRenderCorrection(uid);
            SetRenderCorrection(uid, current.Position + position, current.Rotation + rotation, time);
        }

        private void SetRenderCorrection(EntityUid uid, Vector2 position, Angle rotation, float time)
        {
            if (position.LengthSquared() <= MinInterpolationDistanceSquared
                && Math.Abs(rotation.Theta) <= MinInterpolationDistance)
            {
                _renderCorrections.Remove(uid);
                return;
            }

            if (position.LengthSquared() >= MaxInterpolationDistanceSquared)
            {
                _renderCorrections.Remove(uid);
                return;
            }

            _renderCorrections[uid] = new RenderCorrection
            {
                Position = position,
                Rotation = rotation,
                TimeRemaining = time,
            };
        }

        private void SetRenderCorrectionFromWorldPose(
            EntityUid uid,
            TransformComponent xform,
            RenderPose origin,
            float time)
        {
            var target = GetRenderWorldPositionRotation(xform, XformQuery, false);
            var position = origin.Position - target.WorldPosition;

            if (xform.ParentUid != xform.MapUid
                && xform.ParentUid.IsValid()
                && XformQuery.TryGetComponent(xform.ParentUid, out var parent))
            {
                var parentRotation = GetRenderWorldPositionRotation(parent).WorldRotation;
                position = (-parentRotation).RotateVec(position);
            }

            SetRenderCorrection(
                uid,
                position,
                Angle.ShortestDistance(target.WorldRotation, origin.Rotation),
                time);
        }

        private void DecayRenderCorrections(float frameTime)
        {
            List<EntityUid>? staleCorrections = null;

            foreach (var (uid, correction) in _renderCorrections)
            {
                if (!XformQuery.TryGetComponent(uid, out var xform) || xform.Deleted || !xform.Initialized)
                {
                    (staleCorrections ??= new List<EntityUid>()).Add(uid);
                    continue;
                }

                if (float.IsPositiveInfinity(correction.TimeRemaining))
                    continue;

                var oldTime = correction.TimeRemaining;
                var newTime = MathF.Max(0f, oldTime - frameTime);
                var scale = oldTime > 0f ? newTime / oldTime : 0f;

                var updated = correction;
                updated.TimeRemaining = newTime;
                updated.Position *= scale;
                updated.Rotation *= scale;

                if (updated.Position.LengthSquared() <= MinInterpolationDistanceSquared
                    && Math.Abs(updated.Rotation.Theta) <= MinInterpolationDistance)
                {
                    (staleCorrections ??= new List<EntityUid>()).Add(uid);
                    continue;
                }

                _renderCorrections[uid] = updated;
            }

            if (staleCorrections == null)
                return;

            foreach (var uid in staleCorrections)
                _renderCorrections.Remove(uid);
        }
    }
}
