using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Events;
using Robust.Shared.Physics.Systems;

namespace Robust.Shared.GameObjects
{
    public sealed class CollisionWakeSystem : EntitySystem
    {
        [Dependency] private readonly SharedPhysicsSystem _physics = default!;

        public override void Initialize()
        {
            base.Initialize();

            SubscribeLocalEvent<CollisionWakeComponent, StartCollideEvent>(OnWakeStartCollide);
            SubscribeLocalEvent<CollisionWakeComponent, EndCollideEvent>(OnWakeEndCollide);
            SubscribeLocalEvent<CollisionWakeComponent, ComponentShutdown>(OnRemove);

            SubscribeLocalEvent<CollisionWakeComponent, PhysicsWakeEvent>(OnWake);
            SubscribeLocalEvent<CollisionWakeComponent, PhysicsSleepEvent>(OnSleep);

            SubscribeLocalEvent<CollisionWakeComponent, JointAddedEvent>(OnJointAdd);
            SubscribeLocalEvent<CollisionWakeComponent, JointRemovedEvent>(OnJointRemove);

            SubscribeLocalEvent<CollisionWakeComponent, EntParentChangedMessage>(OnParentChange);
        }

        private void OnWakeStartCollide(Entity<CollisionWakeComponent> ent, ref StartCollideEvent args)
        {
            // Disable CollisionWake if there's a contact.
            if (!ent.Comp.Enabled)
                return;

            ent.Comp.Enabled = false;
            Dirty(ent);
        }

        private void OnWakeEndCollide(Entity<CollisionWakeComponent> entity, ref EndCollideEvent args)
        {
            // If collision ended check if we can re-apply CollisionWake.
            if (entity.Comp.Enabled || !CanSleep(entity))
                return;

            entity.Comp.Enabled = true;
            Dirty(entity);
            UpdateCanCollide(entity, args.OurBody);
        }

        public void SetEnabled(EntityUid uid, bool enabled, CollisionWakeComponent? component = null)
        {
            if (!Resolve(uid, ref component) || component.Enabled == enabled)
                return;

            component.Enabled = enabled;

            if (component.Enabled)
                UpdateCanCollide((uid, component));
            else if (TryComp(uid, out PhysicsComponent? physics))
                _physics.SetCanCollide(uid, true, body: physics);

            Dirty(uid, component);
        }

        /// <summary>
        /// Returns whether CollisionWake can be applied.
        /// </summary>
        private bool CanSleep(Entity<CollisionWakeComponent> entity, PhysicsComponent? body = null, JointComponent? joints = null)
        {
            if (Resolve(entity, ref body))
            {
                if (body.ContactCount > 0)
                {
                    return false;
                }
            }

            if (Resolve(entity, ref joints, false))
            {
                if (joints.JointCount > 0)
                {
                    return false;
                }
            }

            return true;
        }

        private void OnRemove(EntityUid uid, CollisionWakeComponent component, ComponentShutdown args)
        {
            if (component.Enabled
                && !Terminating(uid)
                && TryComp(uid, out PhysicsComponent? physics))
            {
                _physics.SetCanCollide(uid, true, body: physics);
            }
        }

        private void OnParentChange(Entity<CollisionWakeComponent> entity, ref EntParentChangedMessage args)
        {
            if (entity.Comp.LifeStage < ComponentLifeStage.Initialized)
                return;

            UpdateCanCollide(entity, xform: args.Transform);
        }

        private void OnJointRemove(EntityUid uid, CollisionWakeComponent component, JointRemovedEvent args)
        {
            Entity<CollisionWakeComponent> entity = (uid, component);

            if (!CanSleep(entity, args.OurBody))
                return;

            UpdateCanCollide(entity);
        }

        private void OnJointAdd(EntityUid uid, CollisionWakeComponent component, JointAddedEvent args)
        {
            // Bypass UpdateCanCollide() as joint count will always be bigger than 0:
            if (component.Enabled)
                _physics.SetCanCollide(uid, true);
        }

        private void OnWake(Entity<CollisionWakeComponent> entity, ref PhysicsWakeEvent args)
        {
            UpdateCanCollide(entity, args.Body, checkTerminating: false);
        }

        private void OnSleep(Entity<CollisionWakeComponent> entity, ref PhysicsSleepEvent args)
        {
            UpdateCanCollide(entity, args.Body);
        }

        private void UpdateCanCollide(
            Entity<CollisionWakeComponent> entity,
            PhysicsComponent? body = null,
            TransformComponent? xform = null,
            bool checkTerminating = true,
            bool dirty = true)
        {
            if (!entity.Comp.Enabled)
                return;

            if (checkTerminating && Terminating(entity))
                return;

            if (!Resolve(entity, ref body, ref xform, false) ||
                xform.MapID == MapId.Nullspace)
            {
                return;
            }

            // If we're attached to the map we'll also just never disable collision due to how grid movement works.
            var canCollide = body.Awake ||
                             body.ContactCount > 0 ||
                              (TryComp(entity, out JointComponent? jointComponent) && jointComponent.JointCount > 0) ||
                              xform.GridUid == null;

            _physics.SetCanCollide(entity, canCollide, dirty, body: body);
        }
    }
}
