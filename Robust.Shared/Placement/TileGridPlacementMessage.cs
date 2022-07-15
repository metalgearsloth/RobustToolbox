using System;
using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;

namespace Robust.Shared.Placement;

[Serializable, NetSerializable]
public sealed class TileGridPlacementMessage : GridPlacementMessage
{
    public Tile Tile;

    public EntityUid Grid;
    public List<Vector2i> Indices = new();
}
