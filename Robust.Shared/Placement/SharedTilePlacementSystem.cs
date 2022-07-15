using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;

namespace Robust.Shared.Placement;

public abstract class SharedTilePlacementSystem : PlacementSystem
{
    [Dependency] protected readonly ITileDefinitionManager DefManager = default!;
}
