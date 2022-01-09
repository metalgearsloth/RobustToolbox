using System;
using System.Collections.Generic;
using Robust.Shared.Console;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Utility;

namespace Robust.Shared.GameObjects;

public class MapManagerSystem : EntitySystem
{
    public sealed class MergeGridsCommand : IConsoleCommand
    {
        public string Command => "mergegrids";
        public string Description => "Merges the two specified grids together";
        public string Help => $"{Command}";
        public void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            throw new System.NotImplementedException();
        }
    }

    /// <summary>
    /// Combine two grids into the larger one.
    /// </summary>
    /// <remarks>
    /// Where the 2 grids overlap (tile or entity data) it assumes the larger one is the priority.
    /// </remarks>
    /// <param name="gridOne"></param>
    /// <param name="gridTwo"></param>
    /// <param name="origin">Where grid 2's origin is positioned in grid 1 terms.</param>
    /// <param name="direction">Cardinal direction of grid two in grid one terms. If they have the same rotation then it's East.</param>
    /// <returns>The larger grid that is retained</returns>
    public GridId MergeGrids(IMapGrid gridOne, IMapGrid gridTwo, Vector2i origin = new(), DirectionFlag direction = DirectionFlag.East)
    {
        DebugTools.Assert(direction != DirectionFlag.NorthEast &&
                          direction != DirectionFlag.NorthWest &&
                          direction != DirectionFlag.SouthEast &&
                          direction != DirectionFlag.SouthWest);

        var bodyOne = EntityManager.GetComponent<PhysicsComponent>(gridOne.GridEntityId);
        var bodyTwo = EntityManager.GetComponent<PhysicsComponent>(gridTwo.GridEntityId);
        GridId gridId;

        // Find larger grid
        if (bodyOne.Mass > bodyTwo.Mass)
        {
            gridId = gridOne.Index;
        }
        else
        {
            gridId = gridTwo.Index;
            var grid = gridTwo;
            gridOne = grid;
            gridTwo = gridOne;
        }

        // Update tiles on grid one.
        var gridTwoTiles = new List<(Vector2i, Tile)>();
        // TODO: Maps can easily store how many tiles they have to pre-allocate.
        foreach (var tile in gridTwo.GetAllTiles())
        {
            var adjustedIndex = tile.GridIndices + origin;

            gridTwoTiles.Add((adjustedIndex, tile.Tile));
        }

        gridOne.SetTiles(gridTwoTiles);

        DebugTools.Assert(EntityManager.GetComponent<TransformComponent>(gridTwo.GridEntityId).ChildCount == 0);
        EntityManager.DeleteEntity(gridTwo.GridEntityId);

        return gridId;
    }

    private Vector2i GetAdjustedIndex(Vector2i index, Direction direction)
    {
        switch (direction)
        {
            case Direction.East:
                return index;
            case Direction.South:
                return
            default:
                throw new ArgumentOutOfRangeException();
        }
    }
}
