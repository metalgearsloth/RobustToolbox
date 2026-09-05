using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics;

/// <summary>
/// Raised by the engine if content wishes to override the default clear color.
/// </summary>
[ByRefEvent]
public record struct GetClearColorEvent
{
    public readonly EntityUid MapUid;
    public readonly MapId MapId;
    public readonly int ZLevelOffset;
    public Color? Color;

    public GetClearColorEvent(EntityUid mapUid, MapId mapId = default, int zLevelOffset = 0)
    {
        MapUid = mapUid;
        MapId = mapId;
        ZLevelOffset = zLevelOffset;
        Color = null;
    }
}
