using System.Collections.Generic;
using Robust.Client.Graphics;
using Robust.Shared.Maths;
using Robust.Shared.Utility;
using SixLabors.ImageSharp;

namespace Robust.Client.ResourceManagement
{
    public readonly struct RsiLoadedEventArgs
    {
        internal RsiLoadedEventArgs(ResPath path, RSIResource resource, Image atlas, Dictionary<RSI.StateId, Vector2i[][]> atlasOffsets, Vector2i atlasOffset)
        {
            Path = path;
            Resource = resource;
            Atlas = atlas;
            AtlasOffsets = atlasOffsets;
            AtlasOffset = atlasOffset;
        }

        public ResPath Path { get; }
        public RSIResource Resource { get; }
        public Image Atlas { get; }
        public Dictionary<RSI.StateId, Vector2i[][]> AtlasOffsets { get; }
        public Vector2i AtlasOffset { get; }
    }
}
