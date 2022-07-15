using System;
using Robust.Shared.Map;
using Robust.Shared.Serialization;

namespace Robust.Shared.Placement;

[Serializable, NetSerializable]
public sealed class TileSinglePlacementMessage : SinglePlacementMessage
{
    public TileRef TileRef;
}
