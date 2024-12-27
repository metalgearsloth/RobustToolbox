using System;
using System.Collections.Generic;
using Robust.Server.GameStates;
using Robust.Shared.GameObjects;
using Robust.Shared.Utility;

namespace Robust.Server.GameObjects;

public sealed class UserInterfaceSystem : SharedUserInterfaceSystem
{
    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<UserInterfaceUserComponent, ExpandPvsEvent>(OnBuiUserExpand);
    }

    /// <inheritdoc />
    public override void SendUiMessage(Entity<UserInterfaceComponent?> entity, Enum key, BoundUserInterfaceMessage message)
    {
        ServerSendUiMessage(entity, key, message);
    }

    /// <inheritdoc />
    public override void ClientReceiveUiMessage(Entity<UserInterfaceComponent?> entity, Enum key, BoundUserInterfaceMessage message)
    {
        // NOOP
    }

    private void OnBuiUserExpand(Entity<UserInterfaceUserComponent> ent, ref ExpandPvsEvent args)
    {
        var buis = ent.Comp.OpenInterfaces.Keys;

        if (buis.Count == 0)
            return;

        args.Entities ??= new List<EntityUid>(buis.Count);

        foreach (var ui in buis)
        {
            DebugTools.Assert(ent.Comp.OpenInterfaces[ui].Count > 0);
            DebugTools.Assert(HasComp<UserInterfaceComponent>(ui));
            args.Entities.Add(ui);
        }
    }
}
