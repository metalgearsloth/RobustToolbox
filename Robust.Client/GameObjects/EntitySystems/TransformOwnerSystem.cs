using Robust.Client.Player;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Physics.Systems;

namespace Robust.Client.GameObjects;

public sealed class OwnerTransformSystem : SharedOwnerTransformSystem
{
    [Dependency] private readonly IPlayerManager _playerManager = default!;

    public override void Initialize()
    {
        base.Initialize();
        UpdatesAfter.Add(typeof(SharedPhysicsSystem));
        UpdatesAfter.Add(typeof(SharedTransformSystem));
    }

    public override void Update(float frameTime)
    {
        base.Update(frameTime);

        if (!Timing.IsFirstTimePredicted)
            return;

        var player = _playerManager.LocalEntity;

        if (!TryComp(player, out OwnerTransformComponent? ownerXform))
        {
            Enabled = false;
            return;
        }

        Enabled = true;
        var xform = Transform(player.Value);

        if (xform.LocalPosition.Equals(ownerXform.LastPosition) && xform.LocalRotation.Equals(ownerXform.LastRotation))
            return;

        ownerXform.LastPosition = xform.LocalPosition;
        ownerXform.LastRotation = xform.LocalRotation;
        RaiseNetworkEvent(new TransformOwnerMessage()
        {
            LocalPosition = xform.LocalPosition,
            LocalRotation = xform.LocalRotation,
        });
    }
}
