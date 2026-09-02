using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Systems;

namespace Robust.Shared.Console.Commands;

/// <summary>
/// Development commands for building and inspecting small z-level stacks without map-loading content code.
/// Maps passed to <c>zlevels_stack</c> are ordered from bottom to top.
/// </summary>
public sealed partial class ZLevelsStackCommand : LocalizedEntityCommands
{
    [Dependency] private SharedMapSystem _mapSystem = default!;
    [Dependency] private ZLevelSystem _zLevels = default!;

    /// <inheritdoc/>
    public override string Command => "zlevels_stack";

    /// <inheritdoc/>
    public override bool RequireServerOrSingleplayer => true;

    /// <inheritdoc/>
    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length < 2)
        {
            shell.WriteError(Help);
            return;
        }

        var maps = new List<EntityUid>(args.Length);
        var seen = new HashSet<MapId>();
        foreach (var arg in args)
        {
            if (!int.TryParse(arg, out var value) ||
                !seen.Add(new MapId(value)) ||
                !_mapSystem.TryGetMap(new MapId(value), out var map))
            {
                shell.WriteError(Loc.GetString("cmd-zlevels_stack-error-map", ("map", arg)));
                return;
            }

            if (EntityManager.HasComponent<ZLevelMapComponent>(map.Value))
            {
                shell.WriteError(Loc.GetString("cmd-zlevels_stack-error-member", ("map", value)));
                return;
            }

            maps.Add(map.Value);
        }

        if (!_zLevels.TryCreateMapNetwork(maps, out var network))
        {
            shell.WriteError(Loc.GetString("cmd-zlevels_stack-error-create"));
            return;
        }

        shell.WriteLine(Loc.GetString("cmd-zlevels_stack-success", ("network", network.Value.Owner), ("maps", string.Join(", ", args))));
    }

    /// <inheritdoc/>
    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        return CompletionResult.FromHintOptions(
            CompletionHelper.MapIds(args.Length == 0 ? string.Empty : args[^1], entManager: EntityManager),
            Loc.GetString("cmd-zlevels_stack-hint"));
    }
}

/// <summary>
/// Removes a map from its z-level stack.
/// </summary>
public sealed partial class ZLevelsUnstackCommand : LocalizedEntityCommands
{
    [Dependency] private SharedMapSystem _mapSystem = default!;
    [Dependency] private ZLevelSystem _zLevels = default!;

    /// <inheritdoc/>
    public override string Command => "zlevels_unstack";

    /// <inheritdoc/>
    public override bool RequireServerOrSingleplayer => true;

    /// <inheritdoc/>
    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length != 1 ||
            !int.TryParse(args[0], out var value) ||
            !_mapSystem.TryGetMap(new MapId(value), out var map) ||
            !_zLevels.TryRemoveMapFromNetwork(map.Value))
        {
            shell.WriteError(Help);
            return;
        }

        shell.WriteLine(Loc.GetString("cmd-zlevels_unstack-success", ("map", value)));
    }

    /// <inheritdoc/>
    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        return args.Length == 1
            ? CompletionResult.FromHintOptions(CompletionHelper.MapIds(args[0], entManager: EntityManager), Loc.GetString("cmd-zlevels_unstack-hint"))
            : CompletionResult.Empty;
    }
}

/// <summary>
/// Links two grids on adjacent z-level maps.
/// </summary>
public sealed partial class ZLevelsLinkGridsCommand : LocalizedEntityCommands
{
    [Dependency] private ZLevelSystem _zLevels = default!;

    public override string Command => "zlevels_link_grids";

    public override bool RequireServerOrSingleplayer => true;

    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length != 2 ||
            !NetEntity.TryParse(args[0], out var lowerNet) ||
            !NetEntity.TryParse(args[1], out var upperNet) ||
            !EntityManager.TryGetEntity(lowerNet, out var lower) ||
            !EntityManager.TryGetEntity(upperNet, out var upper) ||
            !_zLevels.TryLinkGrids(lower.Value, upper.Value))
        {
            shell.WriteError(Help);
            return;
        }

        shell.WriteLine(Loc.GetString("cmd-zlevels_link_grids-success", ("lower", lower), ("upper", upper)));
    }

    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        return args.Length is 1 or 2
            ? CompletionResult.FromHintOptions(
                CompletionHelper.Components<MapGridComponent>(args[^1], EntityManager),
                Loc.GetString("cmd-zlevels_link_grids-hint"))
            : CompletionResult.Empty;
    }
}

/// <summary>
/// Removes all z-level links connected to a grid.
/// </summary>
public sealed partial class ZLevelsUnlinkGridCommand : LocalizedEntityCommands
{
    [Dependency] private ZLevelSystem _zLevels = default!;

    public override string Command => "zlevels_unlink_grid";

    public override bool RequireServerOrSingleplayer => true;

    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length != 1 ||
            !NetEntity.TryParse(args[0], out var gridNet) ||
            !EntityManager.TryGetEntity(gridNet, out var grid) ||
            !_zLevels.TryUnlinkGrid(grid.Value))
        {
            shell.WriteError(Help);
            return;
        }

        shell.WriteLine(Loc.GetString("cmd-zlevels_unlink_grid-success", ("grid", grid)));
    }

    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        return args.Length == 1
            ? CompletionResult.FromHintOptions(
                CompletionHelper.Components<MapGridComponent>(args[0], EntityManager),
                Loc.GetString("cmd-zlevels_unlink_grid-hint"))
            : CompletionResult.Empty;
    }
}

/// <summary>
/// Lists all active z-level map stacks.
/// </summary>
public sealed partial class ZLevelsListCommand : LocalizedEntityCommands
{
    [Dependency] private ZLevelSystem _zLevels = default!;
    [Dependency] private EntityQuery<MapComponent> _mapQuery = default!;

    private readonly List<Entity<ZLevelMapNetworkComponent>> _networks = new();
    private readonly List<string> _maps = new();

    /// <inheritdoc/>
    public override string Command => "zlevels_list";

    /// <inheritdoc/>
    public override bool RequireServerOrSingleplayer => true;

    /// <inheritdoc/>
    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length != 0)
        {
            shell.WriteError(Help);
            return;
        }

        _zLevels.CollectMapNetworks(_networks);
        if (_networks.Count == 0)
        {
            shell.WriteLine(Loc.GetString("cmd-zlevels_list-empty"));
            return;
        }

        foreach (var network in _networks)
        {
            _maps.Clear();

            foreach (var uid in network.Comp.SortedZLevels)
            {
                _maps.Add(_mapQuery.TryComp(uid, out var map)
                    ? map.MapId.Value.ToString()
                    : uid.ToString());
            }

            shell.WriteLine(Loc.GetString("cmd-zlevels_list-entry", ("network", network.Owner), ("maps", string.Join(", ", _maps))));
        }
    }
}

/// <summary>
/// Prints the last bounded z-physics step for an entity.
/// </summary>
public sealed partial class ZPhysicsDebugCommand : LocalizedEntityCommands
{
    [Dependency] private ZLevelSystem _zLevels = default!;
    [Dependency] private ZLevelSupportSystem _support = default!;
    [Dependency] private SharedTransformSystem _transform = default!;

    public override string Command => "zphysics_debug";

    public override bool RequireServerOrSingleplayer => true;

    public override void Execute(IConsoleShell shell, string argStr, string[] args)
    {
        if (args.Length != 1 ||
            !NetEntity.TryParse(args[0], out var netEntity) ||
            !EntityManager.TryGetEntity(netEntity, out var uid) ||
            !EntityManager.TryGetComponent(uid.Value, out ZLevelPhysicsComponent? physics))
        {
            shell.WriteError(Help);
            return;
        }

        var step = physics.LastStep;
        shell.WriteLine(Loc.GetString(
            "cmd-zphysics_debug-result",
            ("entity", uid.Value),
            ("startVelocity", step.StartVelocity),
            ("endVelocity", step.EndVelocity),
            ("nextEvent", step.NextEvent),
            ("remaining", step.RemainingTime),
            ("crossings", step.Crossings),
            ("events", step.Events),
            ("limited", step.IterationLimitReached)));

        var samplePoint = "unavailable";
        var projected = "unavailable";
        if (physics.SupportSurface != ZLevelSupportSurface.None &&
            EntityManager.TryGetComponent(uid.Value, out TransformComponent? xform) &&
            xform.MapUid is { } viewedMap)
        {
            var point = _transform.GetWorldPosition(xform);
            samplePoint = point.ToString();
            if (_zLevels.TryProjectAbsolutePosition(
                    viewedMap,
                    point,
                    physics.SupportHeight,
                    out var projectedPoint))
            {
                projected = projectedPoint.ToString();
            }
        }

        shell.WriteLine(Loc.GetString(
            "cmd-zphysics_debug-support",
            ("ground", physics.GroundState),
            ("provider", physics.SupportProvider?.ToString() ?? "none"),
            ("surface", physics.SupportSurface),
            ("height", physics.SupportHeight),
            ("sample", samplePoint),
            ("projected", projected)));

        foreach (var candidate in _support.GetLastSupportCandidates(uid.Value))
        {
            shell.WriteLine(Loc.GetString(
                "cmd-zphysics_debug-candidate",
                ("provider", candidate.Provider),
                ("surface", candidate.Surface),
                ("height", candidate.AbsoluteHeight),
                ("sample", candidate.SamplePoint),
                ("rejection", candidate.Rejection)));
        }
    }

    public override CompletionResult GetCompletion(IConsoleShell shell, string[] args)
    {
        return args.Length == 1
            ? CompletionResult.FromHintOptions(
                CompletionHelper.Components<ZLevelPhysicsComponent>(args[0], EntityManager),
                Loc.GetString("cmd-zphysics_debug-hint"))
            : CompletionResult.Empty;
    }
}
