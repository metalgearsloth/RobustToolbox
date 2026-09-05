using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics;

/// <summary>
/// Raised while applying the post-shader for a rendered z-level layer.
/// The engine provides default values; content can replace them or set <see cref="PostShader"/> to null.
/// </summary>
[ByRefEvent]
public record struct ZLevelPostShaderEvent
{
    public readonly EntityUid MapUid;
    public readonly EntityUid NetworkUid;
    public readonly MapId MapId;
    public readonly int ZLevelOffset;
    /// <summary>
    /// Continuous 0..1 depth-effect strength for this layer relative to the presented eye z.
    /// </summary>
    public readonly float EffectStrength;
    public ShaderInstance? PostShader;
    public Color Tint;

    public ZLevelPostShaderEvent(
        EntityUid mapUid,
        EntityUid networkUid,
        MapId mapId,
        int zLevelOffset,
        float effectStrength,
        ShaderInstance? postShader,
        Color tint)
    {
        MapUid = mapUid;
        NetworkUid = networkUid;
        MapId = mapId;
        ZLevelOffset = zLevelOffset;
        EffectStrength = effectStrength;
        PostShader = postShader;
        Tint = tint;
    }
}
