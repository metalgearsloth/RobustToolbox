using System;
using System.Numerics;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

public abstract class SharedOwnerTransformSystem : EntitySystem
{
    [Dependency] protected readonly IGameTiming Timing = default!;
    [Dependency] private readonly SharedTransformSystem XformSystem = default!;

    public virtual bool Enabled { get; protected set; } = false;

    public override void Initialize()
    {
        base.Initialize();
        SubscribeNetworkEvent<TransformOwnerMessage>(OnTransformMessage);
    }

    private void OnTransformMessage(TransformOwnerMessage msg, EntitySessionEventArgs args)
    {
        var player = args.SenderSession.AttachedEntity;

        if (!HasComp<OwnerTransformComponent>(player))
            return;

        XformSystem.SetLocalPositionRotation(player.Value, msg.LocalPosition, msg.LocalRotation);
    }

    public void SetPlayerOwner(EntityUid uid)
    {
        if (!EnsureComp<OwnerTransformComponent>(uid, out var comp))
            return;

        // TODO: Disable transform states (except teleports probably)
        if (TryComp(uid, out PhysicsComponent? physics))
        {
            physics.ServerIgnored = true;
        }
    }

    [Serializable, NetSerializable]
    protected sealed class TransformOwnerMessage : EntityEventArgs
    {
        public Vector2 LocalPosition;
        public Angle LocalRotation;
    }
}
