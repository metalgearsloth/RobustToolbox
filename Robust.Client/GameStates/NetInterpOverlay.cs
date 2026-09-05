using System;
using System.Numerics;
using Robust.Client.GameObjects;
using Robust.Client.Graphics;
using Robust.Client.Player;
using Robust.Client.ResourceManagement;
using Robust.Shared.Console;
using Robust.Shared.Enums;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Maths;

namespace Robust.Client.GameStates;

/// <summary>
/// Disabled-by-default diagnostic view for the client-only render-pose cache.
/// </summary>
internal sealed partial class NetInterpOverlay : Overlay
{
    [Dependency] private IEntityManager _entityManager = default!;
    [Dependency] private IResourceCache _resourceCache = default!;

    private readonly TransformSystem _transforms;
    private readonly Font _font;

    private EntityUid? _filter;

    public override OverlaySpace Space => OverlaySpace.ScreenSpace;

    public NetInterpOverlay()
    {
        IoCManager.InjectDependencies(this);
        _transforms = _entityManager.System<TransformSystem>();
        _font = new VectorFont(
            _resourceCache.GetResource<FontResource>("/Fonts/NotoSans/NotoSans-Regular.ttf"),
            10);
    }

    protected internal override void Draw(in OverlayDrawArgs args)
    {
        if (args.ViewportControl == null)
            return;

        var handle = args.ScreenHandle;
        foreach (var data in _transforms.GetRenderPoseDebugData())
        {
            if ((_filter != null && data.Entity != _filter)
                || data.CoordinateSpace != args.MapUid)
                continue;

            var simulation = args.ViewportControl.WorldToScreen(data.Simulation.Position);
            var rendered = args.ViewportControl.WorldToScreen(data.Rendered.Position);
            var source = args.ViewportControl.WorldToScreen(data.Source.Position);
            var target = args.ViewportControl.WorldToScreen(data.Target.Position);

            handle.DrawLine(source, target, Color.Yellow);
            handle.DrawLine(simulation, rendered, Color.Cyan);
            DrawMarker(handle, source, Color.Yellow);
            DrawMarker(handle, target, Color.Green);
            DrawMarker(handle, simulation, Color.Red);
            DrawMarker(handle, rendered, Color.Cyan);

            var correction =
                $"{data.CorrectionTranslation.X:0.000},{data.CorrectionTranslation.Y:0.000}, {data.CorrectionRotation.Degrees:0.00}deg";
            var text = $"{data.Entity} {data.Type} a={data.Alpha:0.000}\n" +
                       $"sim {Format(data.Simulation)} render {Format(data.Rendered)}\n" +
                       $"source {Format(data.Source)} target {Format(data.Target)}\n" +
                       $"parent {data.Parent} coords {data.CoordinateSpace}\n" +
                       $"spaces {data.SourceRenderSpace}->{data.TargetRenderSpace} " +
                       $"a={data.Rendered.RenderSpaceAlpha:0.000} error {correction}";
            var dimensions = handle.GetDimensions(_font, text, 1f);
            var labelPos = rendered + new Vector2(8f, 8f);
            handle.DrawRect(UIBox2.FromDimensions(labelPos - new Vector2(2f), dimensions + new Vector2(4f)),
                new Color(20, 20, 24, 220));
            handle.DrawString(_font, labelPos, text);
        }
    }

    private static string Format(in RenderPose pose)
        => $"({pose.Position.X:0.00},{pose.Position.Y:0.00},{pose.Rotation.Degrees:0.0}deg)";

    private static void DrawMarker(DrawingHandleScreen handle, Vector2 position, Color color)
    {
        handle.DrawRect(UIBox2.FromDimensions(position - new Vector2(2f), new Vector2(4f)), color);
    }

    private sealed partial class NetShowInterpCommand : LocalizedCommands
    {
        [Dependency] private IOverlayManager _overlay = default!;
        [Dependency] private IPlayerManager _players = default!;

        public override string Command => "net_draw_interp";

        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            if (args.Length > 1)
            {
                shell.WriteError(Help);
                return;
            }

            if (args.Length == 0)
            {
                if (_overlay.HasOverlay<NetInterpOverlay>())
                {
                    _overlay.RemoveOverlay<NetInterpOverlay>();
                    shell.WriteLine("Disabled render interpolation overlay.");
                }
                else
                {
                    _overlay.AddOverlay(new NetInterpOverlay());
                    shell.WriteLine("Enabled render interpolation overlay.");
                }

                return;
            }

            if (args[0] == "0")
            {
                _overlay.RemoveOverlay<NetInterpOverlay>();
                shell.WriteLine("Disabled render interpolation overlay.");
                return;
            }

            EntityUid? filter;
            if (args[0].Equals("all", StringComparison.OrdinalIgnoreCase))
            {
                filter = null;
            }
            else if (args[0].Equals("self", StringComparison.OrdinalIgnoreCase))
            {
                if (_players.LocalEntity is not { } player)
                {
                    shell.WriteError("No controlled entity.");
                    return;
                }

                filter = player;
            }
            else if (EntityUid.TryParse(args[0], out var uid))
            {
                filter = uid;
            }
            else
            {
                shell.WriteError(Help);
                return;
            }

            if (!_overlay.TryGetOverlay<NetInterpOverlay>(out var overlay))
            {
                overlay = new NetInterpOverlay();
                _overlay.AddOverlay(overlay);
            }

            overlay._filter = filter;
            shell.WriteLine(filter == null
                ? "Enabled render interpolation overlay for all entities."
                : $"Enabled render interpolation overlay for entity {filter}.");
        }
    }

}
