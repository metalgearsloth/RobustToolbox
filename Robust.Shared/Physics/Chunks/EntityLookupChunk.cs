using System.Collections.Generic;
using Robust.Shared.Interfaces.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Shared.Physics.Chunks
{
    internal sealed class EntityLookupChunk
    {
        internal const byte ChunkSize = 16;

        /// <summary>
        ///     Parent MapId for this chunk
        /// </summary>
        internal MapId MapId { get; }

        /// <summary>
        ///     Parent GridId for this chunk
        /// </summary>
        internal GridId GridId { get; }

        /// <summary>
        ///     Bottom-left corner of the chunk
        /// </summary>
        internal Vector2i Origin { get; }

        private EntityLookupNode[,] _nodes = new EntityLookupNode[ChunkSize,ChunkSize];

        internal EntityLookupChunk(MapId mapId, GridId gridId, Vector2i origin)
        {
            MapId = mapId;
            GridId = gridId;
            Origin = origin;

            for (var x = 0; x < ChunkSize; x++)
            {
                for (var y = 0; y < ChunkSize; y++)
                {
                    _nodes[x, y] = new EntityLookupNode(this, new Vector2i(Origin.X + x, Origin.Y + y));
                }
            }
        }

        public IEnumerable<EntityLookupNode> GetNodes()
        {
            for (var x = 0; x < ChunkSize; x++)
            {
                for (var y = 0; y < ChunkSize; y++)
                {
                    yield return _nodes[x, y];
                }
            }
        }

        internal EntityLookupNode GetNode(Vector2i nodeIndices)
        {
            return _nodes[nodeIndices.X - Origin.X, nodeIndices.Y - Origin.Y];
        }

        public IEnumerable<IEntity> GetEntities()
        {
            for (var x = 0; x < _nodes.Length; x++)
            {
                for (var y = 0; y < _nodes.Length; y++)
                {
                    var node = _nodes[x, y];
                    foreach (var entity in node.Entities)
                    {
                        yield return entity;
                    }
                }
            }
        }
    }
}
