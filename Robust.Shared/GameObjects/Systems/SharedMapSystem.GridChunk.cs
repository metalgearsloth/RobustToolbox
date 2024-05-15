using System;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Utility;

namespace Robust.Shared.GameObjects;

public abstract partial class SharedMapSystem
{
    /// <summary>
    ///     Replaces a single tile inside of the chunk.
    /// </summary>
    /// <param name="xIndex">The X tile index relative to the chunk.</param>
    /// <param name="yIndex">The Y tile index relative to the chunk.</param>
    /// <param name="tile">The new tile to insert.</param>
    internal bool SetChunkTile(EntityUid uid, MapGridComponent grid, MapChunk chunk, ushort xIndex, ushort yIndex, Tile tile)
    {
        if (!chunk.TrySetTile(xIndex, yIndex, tile, out var oldTile, out var shapeChanged))
            return false;

        var tileIndices = new Vector2i(xIndex, yIndex);
        OnTileModified(uid, grid, chunk, tileIndices, tile, oldTile, shapeChanged);
        return true;
    }

    private void OnTileModified(EntityUid uid, MapGridComponent grid, MapChunk mapChunk, Vector2i tileIndices, Tile newTile, Tile oldTile,
        bool shapeChanged)
    {
        // As the collision regeneration can potentially delete the chunk we'll notify of the tile changed first.
        var gridTile = mapChunk.ChunkTileToGridTile(tileIndices);
        mapChunk.LastTileModifiedTick = _timing.CurTick;
        grid.LastTileModifiedTick = _timing.CurTick;
        Dirty(uid, grid);

        // The map serializer currently sets tiles of unbound grids as part of the deserialization process
        // It properly sets SuppressOnTileChanged so that the event isn't spammed for every tile on the grid.
        // ParentMapId is not able to be accessed on unbound grids, so we can't even call this function for unbound grids.
        if (!MapManager.SuppressOnTileChanged)
        {
            var newTileRef = new TileRef(uid, gridTile, newTile);
            _mapInternal.RaiseOnTileChanged(newTileRef, oldTile, mapChunk.Indices);
        }

        if (shapeChanged && !mapChunk.SuppressCollisionRegeneration)
        {
            RegenerateCollision(uid, grid, mapChunk);
        }
    }
}
