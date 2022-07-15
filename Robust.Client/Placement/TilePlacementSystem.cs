using Robust.Client.ResourceManagement;
using Robust.Client.UserInterface.CustomControls;
using Robust.Shared.IoC;
using Robust.Shared.Placement;

namespace Robust.Client.Placement;

public sealed class TilePlacementSystem : SharedTilePlacementSystem
{
    [Dependency] private readonly IResourceCache _resource = default!;

    private TileSpawnWindow? _tilesSpawnWindow;

    protected override void OnEnable()
    {
        base.OnEnable();
        _tilesSpawnWindow = new TileSpawnWindow(DefManager, _resource);
        _tilesSpawnWindow.OnClose += () => Enabled = false;
    }

    protected override void OnDisable()
    {
        base.OnDisable();
        _tilesSpawnWindow = null;
    }
}
