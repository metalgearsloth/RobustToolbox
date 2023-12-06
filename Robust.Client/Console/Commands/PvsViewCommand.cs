using System.Collections.Generic;
using Robust.Shared.Console;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Utility;

namespace Robust.Client.Console.Commands;

/// <summary>
/// Debug command that dumps data about how many entities are in Pvs range.
/// </summary>
public sealed class PvsViewCommand : IConsoleCommand
{
    [Dependency] private readonly IEntityManager _entManager = default!;

    public string Command => "pvsview";
    public string Description => "Debug command that dumps data about how many entities are in Pvs range.";
    public string Help => $"{Command}";
    public void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        var ent = shell.Player?.AttachedEntity;

        if (ent == null || !_entManager.TryGetComponent(ent, out TransformComponent? entXform))
        {
            return;
        }

        var playerMapId = entXform.MapID;
        var query = _entManager.EntityQueryEnumerator<TransformComponent, MetaDataComponent>();
        var anchoredCount = 0;
        var dynamicCount = 0;
        var anchoredChildCount = 0;
        var protoCounts = new Dictionary<string, int>();

        while (query.MoveNext(out var uid, out var comp, out var metadata))
        {
            if (comp.MapID != playerMapId)
                continue;

            if (metadata.EntityPrototype != null)
            {
                var weh = protoCounts.GetOrNew(metadata.EntityPrototype.ID);
                weh++;
                protoCounts[metadata.EntityPrototype.ID] = weh;
            }

            if (comp.Anchored)
            {
                anchoredCount++;
            }
            else
            {
                if (_entManager.TryGetComponent(comp.ParentUid, out TransformComponent? parentXform) &&
                    parentXform.Anchored)
                {
                    anchoredChildCount++;
                    continue;
                }

                dynamicCount++;
                continue;
            }
        }

        shell.WriteLine($"Anchored count: {anchoredCount}, dynamic count: {dynamicCount}, anchored child count: {anchoredChildCount}");

        foreach (var proto in protoCounts)
        {
            if (proto.Value > 20)
            {
                shell.WriteLine($"Found {proto.Value} of {proto.Key}");
            }
        }
    }
}
