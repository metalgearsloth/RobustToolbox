using System;
using Robust.Shared.GameObjects;
using Robust.Shared.Prototypes;

namespace Robust.Client.GameObjects;

public sealed class UserInterfaceSystem : SharedUserInterfaceSystem
{
    public override void Initialize()
    {
        base.Initialize();
        ProtoManager.PrototypesReloaded += OnProtoReload;
    }

    public override void Shutdown()
    {
        base.Shutdown();
        ProtoManager.PrototypesReloaded -= OnProtoReload;
    }

    /// <inheritdoc />
    public override void SendUiMessage(Entity<UserInterfaceComponent?> entity, Enum key, BoundUserInterfaceMessage message)
    {
        var player = Player.LocalEntity;

        if (player == null)
        {
            return;
        }

        // This adds overhead but it lets us more neatly share the code between the 2 and not bypass validation checks.
        OnMessageReceived(new BoundUIWrapMessage(GetNetEntity(entity.Owner), message, key), player.Value);
    }

    /// <inheritdoc />
    public override void ClientReceiveUiMessage(Entity<UserInterfaceComponent?> entity, Enum key, BoundUserInterfaceMessage message)
    {
        var player = Player.LocalEntity;

        if (player == null ||
            !Resolve(entity.Owner, ref entity.Comp, false) ||
            !entity.Comp.ClientOpenInterfaces.ContainsKey(key))
        {
            return;
        }

        // This adds overhead but it lets us more neatly share the code between the 2 and not bypass validation checks.
        OnMessageReceived(new BoundUIWrapMessage(GetNetEntity(entity.Owner), message, key), player.Value);
    }

    private void OnProtoReload(PrototypesReloadedEventArgs obj)
    {
        var player = Player.LocalEntity;

        if (!UserQuery.TryComp(player, out var userComp))
            return;

        foreach (var uid in userComp.OpenInterfaces.Keys)
        {
            if (!UIQuery.TryComp(uid, out var uiComp))
                continue;

            foreach (var bui in uiComp.ClientOpenInterfaces.Values)
            {
                bui.OnProtoReload(obj);
            }
        }
    }
}
