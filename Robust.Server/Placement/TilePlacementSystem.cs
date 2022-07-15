using Robust.Server.Console;
using Robust.Server.Player;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Placement;
using Robust.Shared.Players;

namespace Robust.Server.Placement;

public sealed class TilePlacementSystem : SharedTilePlacementSystem
{
    [Dependency] private readonly IConGroupController _controller = default!;

    public override void Initialize()
    {
        base.Initialize();
        SubscribeNetworkEvent<TileSinglePlacementMessage>(OnTilePlacement);
        SubscribeNetworkEvent<TileGridPlacementMessage>(OnTileGridPlacement);
    }

    private void OnTileGridPlacement(TileGridPlacementMessage msg, EntitySessionEventArgs args)
    {
        if (!CanPlace(args.SenderSession)) return;


    }

    private void OnTilePlacement(TileSinglePlacementMessage msg, EntitySessionEventArgs args)
    {
        if (!CanPlace(args.SenderSession)) return;


    }

    public override bool CanPlace(ICommonSession session)
    {
        return base.CanPlace(session) && _controller.CanAdminPlace((IPlayerSession) session);
    }
}
