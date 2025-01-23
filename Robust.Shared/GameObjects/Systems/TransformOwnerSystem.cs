using System;
using System.Numerics;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

public abstract class SharedOwnerTransformSystem : EntitySystem
{
    [Dependency] protected readonly IGameTiming Timing = default!;
    [Dependency] private readonly SharedTransformSystem XformSystem = default!;

    public override void Initialize()
    {
        base.Initialize();
        SubscribeAllEvent<TransformOwnerMessage>(OnTransformMessage);
        UpdatesAfter.Add(typeof(SharedPhysicsSystem));
        UpdatesAfter.Add(typeof(SharedTransformSystem));
    }

    private void OnTransformMessage(TransformOwnerMessage msg, EntitySessionEventArgs args)
    {
        var player = args.SenderSession.AttachedEntity;

        if (!HasComp<OwnerTransformComponent>(player))
            return;

        var xform = Transform(player.Value);
        XformSystem.SetCoordinates(player.Value, xform, new EntityCoordinates(xform.ParentUid, msg.LocalPosition), rotation: msg.LocalRotation);
    }

    public void SetPlayerOwner(EntityUid uid, bool value)
    {
        if (!value)
        {
            if (!RemComp<OwnerTransformComponent>(uid))
                return;
        }
        else
        {
            if (!EnsureComp<OwnerTransformComponent>(uid, out var comp))
                return;
        }

        if (TryComp(uid, out PhysicsComponent? physics))
        {
            physics.ServerIgnored = value;
        }

        if (TryComp(uid, out TransformComponent? transform))
        {
            transform.IgnoreState = value;
        }
    }

    [Serializable, NetSerializable]
    protected sealed class TransformOwnerMessage : EntityEventArgs
    {
        public NetEntity Parent;
        public Vector2 LocalPosition;
        public Angle LocalRotation;
    }
}
