using Robust.Client.Interfaces.Graphics;
using Robust.Shared.Interfaces.GameObjects;
using Robust.Shared.Map;

namespace Robust.Client.Audio
{
    /// <summary>
    ///     Implement content-side to update audio effects in specific instances
    /// </summary>
    public interface IAudioEffect
    {
        bool TrySetEntityEffect(IClydeAudioSource source, IEntity entity);
        bool TrySetCoordsEffect(IClydeAudioSource source, EntityCoordinates coordinates);
    }
}
