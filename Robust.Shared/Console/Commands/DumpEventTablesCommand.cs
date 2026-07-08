using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Localization;

namespace Robust.Shared.Console.Commands;

internal sealed partial class DumpEventTablesCommand : LocalizedCommands
{
    [Dependency] private EntityManager _entities = default!;
    [Dependency] private IComponentFactory _componentFactory = default!;

    public override string Command => "dump_event_tables";

    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length < 1)
        {
            shell.WriteError(Loc.GetString("cmd-dump_event_tables-missing-arg-entity"));
            return;
        }

        if (!NetEntity.TryParse(args[0], out var entityNet) ||
            !_entities.TryGetEntity(entityNet, out var entity) ||
            !_entities.EntityExists(entity))
        {
            shell.WriteError(Loc.GetString("cmd-dump_event_tables-error-entity"));
            return;
        }

        var eventBus = (EntityEventBus)_entities.EventBus;

        if (eventBus._entEventTables.Get(entity.Value) is not { } table)
        {
            shell.WriteError(Loc.GetString("cmd-dump_event_tables-error-entity"));
            return;
        }

        for (var eventId = 0; eventId < table.EventIndices.Length; eventId++)
        {
            ref var comps = ref table.EventIndices[eventId];
            if (comps.Start < 0)
                continue;

            var evType = eventBus._directedEventTypes[eventId];
            shell.WriteLine($"{evType}:");

            var idx = comps.Start;
            while (idx != -1)
            {
                ref var entry = ref table.ComponentLists[idx];
                idx = entry.Next;

                var reg = _componentFactory.IdxToType(entry.Component);
                shell.WriteLine($"    {reg.Name}");
            }
        }
    }

    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        if (args.Length == 1)
            return CompletionResult.FromHint(Loc.GetString("cmd-dump_event_tables-arg-entity"));

        return CompletionResult.Empty;
    }
}
