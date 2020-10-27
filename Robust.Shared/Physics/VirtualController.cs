using System;
using Robust.Shared.GameObjects.Components;
using Robust.Shared.Maths;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Physics
{
    /// <summary>
    ///     The VirtualController allows dynamic changes in the properties of a physics component, usually to simulate a complex physical interaction (such as player movement).
    /// </summary>
    public abstract class VirtualController
    {
        private Vector2 _linearVelocity;

        /// <summary>
        ///     Current contribution to the linear velocity of the entity in meters per second.
        /// </summary>
        [ViewVariables(VVAccess.ReadWrite)]
        [Obsolete("Not used at all")]
        public virtual Vector2 LinearVelocity
        {
            get => _linearVelocity;
            set
            {
                if (_linearVelocity == value)
                    return;

                _linearVelocity = value;

                if (ControlledComponent != null)
                {
                    ControlledComponent.WakeBody();
                    ControlledComponent.Dirty();
                }
            }
        }

        public virtual Vector2 Force
        {
            get => _force;
            set
            {
                if (value != Vector2.Zero)
                    ControlledComponent?.WakeBody();

                if (_force == value)
                    return;

                _force = value;
                ControlledComponent?.Dirty();
            }
        }

        private Vector2 _force;

        public virtual IPhysicsComponent? ControlledComponent { protected get; set; }

        public void ApplyAcceleration(Vector2 acceleration)
        {
            var mass = ControlledComponent?.Mass;
            mass ??= 0.0f;
            Force = acceleration * mass.Value;
        }

        /// <summary>
        ///     Tries to set this controller's linear velocity to zero.
        /// </summary>
        /// <returns>True if successful, false otherwise.</returns>
        public virtual bool Stop()
        {
            Force = Vector2.Zero;
            return true;
        }

        /// <summary>
        ///     Modify a physics component before processing impulses
        /// </summary>
        public virtual void UpdateBeforeProcessing() { }

        /// <summary>
        ///     Modify a physics component after processing impulses
        /// </summary>
        public virtual void UpdateAfterProcessing() { }
    }
}
