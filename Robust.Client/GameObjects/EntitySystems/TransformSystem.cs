using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Robust.Client.Physics;
using Robust.Shared.GameObjects.Components.Transform;
using Robust.Shared.GameObjects.EntitySystemMessages;
using Robust.Shared.GameObjects.Systems;
using Robust.Shared.Interfaces.Timing;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;

namespace Robust.Client.GameObjects.EntitySystems
{
    /// <summary>
    ///     Handles interpolation of transform positions.
    /// </summary>
    [UsedImplicitly]
    internal sealed class TransformSystem : EntitySystem
    {
        // Max distance per tick how far an entity can move before it is considered teleporting.
        // TODO: Make these values somehow dependent on server TPS.
        private const float MaxInterpolationDistance = 2.0f;
        private const double MaxInterpolationAngle = Math.PI / 4; // 45 degrees.

        [Dependency] private readonly IGameTiming _gameTiming = default!;

        // Only keep track of transforms actively lerping.
        // Much faster than iterating 3000+ transforms every frame.
        [ViewVariables] private readonly List<TransformComponent> _lerpingTransforms = new List<TransformComponent>();

        public override void Initialize()
        {
            base.Initialize();

            SubscribeLocalEvent<TransformStartLerpMessage>(TransformStartLerpHandler);
            UpdatesAfter.Add(typeof(PhysicsSystem));
        }

        private void TransformStartLerpHandler(TransformStartLerpMessage ev)
        {
            _lerpingTransforms.Add(ev.Transform);
        }

        public override void FrameUpdate(float frameTime)
        {
            base.FrameUpdate(frameTime);

            var step = (float) (_gameTiming.TickRemainder.TotalSeconds / _gameTiming.TickPeriod.TotalSeconds);

            for (var i = 0; i < _lerpingTransforms.Count; i++)
            {
                var transform = _lerpingTransforms[i];
                var found = false;

                // Only lerp if parent didn't change.
                // E.g. entering lockers would do it.
                if (transform.LerpParent == transform.ParentUid)
                {
                    if (transform.LerpDestination != null)
                    {
                        var lerpDest = transform.LerpDestination.Value;
                        var lerpSource = transform.LerpSource;
                        if ((lerpDest - lerpSource).LengthSquared < MaxInterpolationDistance * MaxInterpolationDistance)
                        {
                            transform.LocalPosition = Vector2.Lerp(lerpSource, lerpDest, step);
                            // Setting LocalPosition clears LerpPosition so fix that.
                            transform.LerpDestination = lerpDest;
                            found = true;
                        }
                    }

                    if (transform.LerpAngle != null)
                    {
                        var lerpDest = transform.LerpAngle.Value;
                        var lerpSource = transform.LerpSourceAngle;
                        if (lerpDest.Theta - lerpSource.Theta < MaxInterpolationAngle)
                        {
                            transform.LocalRotation = Angle.Lerp(lerpSource, lerpDest, step);
                            // Setting LocalRotation clears LerpAngle so fix that.
                            transform.LerpAngle = lerpDest;
                            found = true;
                        }
                    }
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
        }
    }
}
