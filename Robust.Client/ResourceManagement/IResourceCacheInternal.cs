using Robust.Client.Graphics;
using Robust.LoaderApi;
using Robust.Shared.ContentPack;
using Robust.Shared.Utility;

namespace Robust.Client.ResourceManagement;

/// <inheritdoc />
internal interface IResourceCacheInternal : IResourceCache
{
    void TextureLoaded(TextureLoadedEventArgs eventArgs);
    void TextureUnloaded(Texture texture);
    void RsiLoaded(RsiLoadedEventArgs eventArgs);
    void RsiUnloaded(RSI rsi);
    void PreloadTextures();

    void MountLoaderApi(IResourceManager manager, IFileApi api, string apiPrefix, ResPath? prefix = null);
}
