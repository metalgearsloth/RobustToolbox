using System;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Annotations;
using Robust.Shared.GameObjects.Components;
using Robust.Shared.GameObjects.Components.Transform;
using Robust.Shared.GameObjects.EntitySystemMessages;
using Robust.Shared.GameObjects.Systems;
using Robust.Shared.Interfaces.GameObjects;
using Robust.Shared.Interfaces.Map;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Shared.Physics.Chunks
{
    /// <summary>
    ///     Stores what entities intersect a particular tile.
    /// </summary>
    [UsedImplicitly]
    public sealed class EntityLookupSystem : EntitySystem
    {
        [Dependency] private readonly IMapManager _mapManager = default!;

        private readonly Dictionary<MapId, Dictionary<GridId, Dictionary<Vector2i, EntityLookupChunk>>> _graph =
                     new Dictionary<MapId, Dictionary<GridId, Dictionary<Vector2i, EntityLookupChunk>>>();

        /// <summary>
        ///     Need to store the nodes for each entity because if the entity is deleted its transform is no longer valid.
        /// </summary>
        private readonly Dictionary<IEntity, HashSet<EntityLookupNode>> _lastKnownNodes =
                     new Dictionary<IEntity, HashSet<EntityLookupNode>>();

        /// <summary>
        ///     Yields all of the entities intersecting a particular entity's tiles.
        /// </summary>
        /// <param name="entity"></param>
        /// <returns></returns>
        public IEnumerable<IEntity> GetEntitiesIntersecting(IEntity entity)
        {
            foreach (var node in GetOrCreateNodes(entity))
            {
                foreach (var ent in node.Entities)
                {
                    yield return ent;
                }
            }
        }

        /// <summary>
        ///     Yields all of the entities intersecting a particular Vector2i
        /// </summary>
        /// <param name="mapId"></param>
        /// <param name="gridId"></param>
        /// <param name="gridIndices"></param>
        /// <returns></returns>
        public IEnumerable<IEntity> GetEntitiesIntersecting(MapId mapId, GridId gridId, Vector2i gridIndices)
        {
            var grids = _graph[mapId];
            var chunks = grids[gridId];

            var chunkIndices = GetChunkIndices(gridIndices);
            if (!chunks.TryGetValue(chunkIndices, out var chunk))
            {
                yield break;
            }

            foreach (var entity in chunk.GetNode(gridIndices).Entities)
            {
                yield return entity;
            }
        }

        public IEnumerable<IEntity> GetEntitiesIntersecting(MapId mapId, Box2 worldBox)
        {
            var grids = _graph[mapId];

            foreach (var grid in _mapManager.FindGridsIntersecting(mapId, worldBox))
            {
                foreach (var (_, chunk) in grids[grid.Index])
                {
                    // TODO: Need to check node is in box.
                    foreach (var node in chunk.GetNodes())
                    {
                        foreach (var entity in node.Entities)
                        {
                            yield return entity;
                        }
                    }
                }
            }
        }

        public IEnumerable<IEntity> GetEntitiesIntersecting(MapId mapId, Vector2 position)
        {
            var grids = _graph[mapId];

            if (_mapManager.TryFindGridAt(mapId, position, out var grid))
            {
                var chunkIndices = GetChunkIndices(position);
                var offsetIndices = new Vector2i((int) (Math.Floor(position.X)), (int) (Math.Floor(position.Y)));
                var node = grids[grid.Index][chunkIndices].GetNode(offsetIndices - chunkIndices);

                foreach (var entity in node.Entities)
                {
                    yield return entity;
                }
            }
        }

        public List<Vector2i> GetIndices(IEntity entity)
        {
            var results = new List<Vector2i>();

            if (!_lastKnownNodes.TryGetValue(entity, out var nodes))
            {
                return results;
            }

            foreach (var node in nodes)
            {
                results.Add(node.Indices);
            }

            return results;
        }

        private EntityLookupChunk GetOrCreateChunk(MapId mapId, GridId gridId, Vector2i indices)
        {
            var chunkIndices = GetChunkIndices(indices);

            if (!_graph.TryGetValue(mapId, out var grids))
            {
                grids = new Dictionary<GridId, Dictionary<Vector2i, EntityLookupChunk>>();
                _graph[mapId] = grids;
            }

            if (!grids.TryGetValue(gridId, out var gridChunks))
            {
                gridChunks = new Dictionary<Vector2i, EntityLookupChunk>();
                grids[gridId] = gridChunks;
            }

            if (!gridChunks.TryGetValue(chunkIndices, out var chunk))
            {
                chunk = new EntityLookupChunk(mapId, gridId, chunkIndices);
                gridChunks[chunkIndices] = chunk;
            }

            return chunk;
        }

        private Vector2i GetChunkIndices(Vector2i indices)
        {
            return new Vector2i(
                (int) (Math.Floor((float) indices.X / EntityLookupChunk.ChunkSize) * EntityLookupChunk.ChunkSize),
                (int) (Math.Floor((float) indices.Y / EntityLookupChunk.ChunkSize) * EntityLookupChunk.ChunkSize));
        }

        private Vector2i GetChunkIndices(Vector2 indices)
        {
            return new Vector2i(
                (int) (Math.Floor(indices.X / EntityLookupChunk.ChunkSize) * EntityLookupChunk.ChunkSize),
                (int) (Math.Floor(indices.Y / EntityLookupChunk.ChunkSize) * EntityLookupChunk.ChunkSize));
        }

        private HashSet<EntityLookupNode> GetOrCreateNodes(IEntity entity)
        {
            if (_lastKnownNodes.TryGetValue(entity, out var nodes))
            {
                return nodes;
            }

            var grids = GetEntityIndices(entity);
            var results = new HashSet<EntityLookupNode>();
            var mapId = entity.Transform.MapID;

            foreach (var (grid, indices) in grids)
            {
                foreach (var index in indices)
                {
                    results.Add(GetOrCreateNode(mapId, grid, index));
                }
            }

            _lastKnownNodes[entity] = results;
            return results;
        }

        /// <summary>
        ///     Return the corresponding TileLookupNode for these indices
        /// </summary>
        /// <param name="mapId"></param>
        /// <param name="gridId"></param>
        /// <param name="indices"></param>
        /// <returns></returns>
        private EntityLookupNode GetOrCreateNode(MapId mapId, GridId gridId, Vector2i indices)
        {
            var chunk = GetOrCreateChunk(mapId, gridId, indices);

            return chunk.GetNode(indices);
        }

        /// <summary>
        ///     Get the relevant GridId and Vector2i for this entity for lookup.
        /// </summary>
        /// <param name="entity"></param>
        /// <returns></returns>
        private Dictionary<GridId, List<Vector2i>> GetEntityIndices(IEntity entity)
        {
            var entityBounds = GetEntityBox(entity);
            var results = new Dictionary<GridId, List<Vector2i>>();
            var onlyOnGrid = false;

            foreach (var grid in _mapManager.FindGridsIntersecting(entity.Transform.MapID, GetEntityBox(entity)))
            {
                var indices = new List<Vector2i>();

                foreach (var tile in grid.GetTilesIntersecting(entityBounds))
                {
                    indices.Add(tile.GridIndices);
                }

                results[grid.Index] = indices;

                if (grid.WorldBounds.Encloses(entityBounds))
                    onlyOnGrid = true;
            }

            if (!onlyOnGrid)
            {
                var gridlessIndices = new List<Vector2i>();
                var leftFloor = (int) Math.Floor(entityBounds.Left);
                var bottomFloor = (int) Math.Floor(entityBounds.Bottom);

                for (var x = 0; x < Math.Ceiling(entityBounds.Width); x++)
                {
                    for (var y = 0; y < Math.Ceiling(entityBounds.Height); y++)
                    {
                        gridlessIndices.Add(new Vector2i(x + leftFloor, y + bottomFloor));
                    }
                }

                results[GridId.Invalid] = gridlessIndices;
            }

            return results;
        }

        private Box2 GetEntityBox(IEntity entity)
        {
            // Need to clip the aabb as anything with an edge intersecting another tile might be picked up, such as walls.
            // TODO: Check if we still need this, also try using 0.001 instead
            if (entity.TryGetComponent(out IPhysicsComponent? physics))
                return new Box2(physics.WorldAABB.BottomLeft + 0.01f, physics.WorldAABB.TopRight - 0.01f);

            // Don't want to accidentally get neighboring tiles unless we're near an edge
            return Box2.CenteredAround(entity.Transform.Coordinates.ToMapPos(EntityManager), Vector2.One / 2);
        }

        public override void Initialize()
        {
            SubscribeLocalEvent<MoveEvent>(HandleEntityMove);
            SubscribeLocalEvent<EntityInitializedMessage>(HandleEntityInitialized);
            _mapManager.OnGridCreated += HandleGridCreated;
            _mapManager.OnGridRemoved += HandleGridRemoval;
            _mapManager.TileChanged += HandleTileChanged;
            _mapManager.MapCreated += HandleMapCreated;
        }

        public override void Shutdown()
        {
            base.Shutdown();
            _mapManager.OnGridCreated -= HandleGridCreated;
            _mapManager.OnGridRemoved -= HandleGridRemoval;
            _mapManager.TileChanged -= HandleTileChanged;
            _mapManager.MapCreated -= HandleMapCreated;
        }

        private void HandleEntityInitialized(EntityInitializedMessage message)
        {
            HandleEntityAdd(message.Entity);
        }

        /*
        private void HandleEntityDeleted(EntityDeletedMessage message)
        {
            HandleEntityRemove(message.Entity);
        }
        */

        private void HandleTileChanged(object? sender, TileChangedEventArgs eventArgs)
        {
            GetOrCreateNode(eventArgs.NewTile.MapIndex, eventArgs.NewTile.GridIndex, eventArgs.NewTile.GridIndices);
        }

        private void HandleGridCreated(GridId gridId)
        {
            var mapId = _mapManager.GetGrid(gridId).ParentMapId;

            if (!_graph.TryGetValue(mapId, out var grids))
            {
                grids = new Dictionary<GridId, Dictionary<Vector2i, EntityLookupChunk>>();
                _graph[mapId] = grids;
            }

            grids[gridId] = new Dictionary<Vector2i, EntityLookupChunk>();
        }

        private void HandleMapCreated(object? sender, MapEventArgs eventArgs)
        {
            _graph[eventArgs.Map] = new Dictionary<GridId, Dictionary<Vector2i, EntityLookupChunk>>();
        }

        private void HandleGridRemoval(GridId gridId)
        {
            var toRemove = new List<IEntity>();

            foreach (var (entity, _) in _lastKnownNodes)
            {
                if (entity.Deleted || entity.Transform.GridID == gridId)
                    toRemove.Add(entity);
            }

            foreach (var entity in toRemove)
            {
                _lastKnownNodes.Remove(entity);
            }

            var mapId = _mapManager.GetGrid(gridId).ParentMapId;
            _graph[mapId].Remove(gridId);
        }

        /// <summary>
        ///     Tries to add the entity to the relevant TileLookupNode
        /// </summary>
        /// The node will filter it to the correct category (if possible)
        /// <param name="entity"></param>
        private void HandleEntityAdd(IEntity entity)
        {
            if (entity.Deleted || entity.Transform.GridID == GridId.Invalid)
            {
                return;
            }

            var entityNodes = GetOrCreateNodes(entity);
            var newIndices = new Dictionary<GridId, List<Vector2i>>();

            foreach (var node in entityNodes)
            {
                node.AddEntity(entity);
                if (!newIndices.TryGetValue(node.ParentChunk.GridId, out var existing))
                {
                    existing = new List<Vector2i>();
                    newIndices[node.ParentChunk.GridId] = existing;
                }

                existing.Add(node.Indices);
            }

            _lastKnownNodes[entity] = entityNodes;
            //EntityManager.EventBus.RaiseEvent(EventSource.Local, new TileLookupUpdateMessage(newIndices));
        }

        /// <summary>
        ///     Removes this entity from all of the applicable nodes.
        /// </summary>
        /// <param name="entity"></param>
        private void HandleEntityRemove(IEntity entity)
        {
            if (_lastKnownNodes.TryGetValue(entity, out var nodes))
            {
                foreach (var node in nodes)
                {
                    node.RemoveEntity(entity);
                }
            }

            _lastKnownNodes.Remove(entity);
            //EntityManager.EventBus.RaiseEvent(EventSource.Local, new TileLookupUpdateMessage(null));
        }

        /// <summary>
        ///     When an entity moves around we'll remove it from its old node and add it to its new node (if applicable)
        /// </summary>
        /// <param name="moveEvent"></param>
        private void HandleEntityMove(MoveEvent moveEvent)
        {
            if (moveEvent.Sender.Deleted || !moveEvent.NewPosition.IsValid(EntityManager))
            {
                HandleEntityRemove(moveEvent.Sender);
                return;
            }

            if (!_lastKnownNodes.TryGetValue(moveEvent.Sender, out var oldNodes))
            {
                return;
            }

            // Memory leak protection
            var gridBounds = _mapManager.GetGrid(moveEvent.Sender.Transform.GridID).WorldBounds;
            if (!gridBounds.Contains(moveEvent.Sender.Transform.WorldPosition))
            {
                HandleEntityRemove(moveEvent.Sender);
                return;
            }

            var newNodes = GetOrCreateNodes(moveEvent.Sender);

            if (oldNodes.Count == newNodes.Count && oldNodes.SetEquals(newNodes))
            {
                return;
            }

            var toRemove = oldNodes.Where(oldNode => !newNodes.Contains(oldNode));
            var toAdd = newNodes.Where(newNode => !oldNodes.Contains(newNode));

            foreach (var node in toRemove)
            {
                node.RemoveEntity(moveEvent.Sender);
            }

            foreach (var node in toAdd)
            {
                node.AddEntity(moveEvent.Sender);
            }

            var newIndices = new Dictionary<GridId, List<Vector2i>>();
            foreach (var node in newNodes)
            {
                if (!newIndices.TryGetValue(node.ParentChunk.GridId, out var existing))
                {
                    existing = new List<Vector2i>();
                    newIndices[node.ParentChunk.GridId] = existing;
                }

                existing.Add(node.Indices);
            }

            _lastKnownNodes[moveEvent.Sender] = newNodes;
            //EntityManager.EventBus.RaiseEvent(EventSource.Local, new TileLookupUpdateMessage(newIndices));
        }
    }
}
