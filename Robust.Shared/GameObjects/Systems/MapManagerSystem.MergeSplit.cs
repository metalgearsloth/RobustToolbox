using System;
using System.Collections.Generic;
using Robust.Shared.Console;
using Robust.Shared.IoC;
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

        // TODO: Tile count would be way better.
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
        // TODO: Probably need a struct enumerator
        foreach (var tile in gridTwo.GetAllTiles())
        {
            var adjustedIndex = GetAdjustedIndex(tile.GridIndices, direction);

            gridTwoTiles.Add((adjustedIndex, tile.Tile));
        }

        // Do the tiles all at once to avoid continuously regenerating collision
        gridOne.SetTiles(gridTwoTiles);

        // Now transfer entities across
        var gridOneComp = EntityManager.GetComponent<MapGridComponent>(gridOne.GridEntityId);
        var gridTwoXform = EntityManager.GetComponent<TransformComponent>(gridTwo.GridEntityId);

        // TODO: This is probably going to be the most expensive bit with slamming entity events all over the place.
        // and general code quality issues.
        foreach (var child in gridTwoXform.ChildEntities)
        {
            var childXform = EntityManager.GetComponent<TransformComponent>(child);

            // Need to do local position relative to the specified grid-one origin PLUS consider rotation too
            var newLocalPos = GetNewLocalPos(childXform.LocalPosition, origin, direction);
            childXform.LocalPosition = newLocalPos;

            if (childXform.Anchored)
            {
                childXform.Anchored = false;
                gridOneComp.AnchorEntity(childXform);
            }
            else
            {
                childXform.AttachParent(gridOne.GridEntityId);
            }
        }

        DebugTools.Assert(gridTwoXform.ChildCount == 0);

        DebugTools.Assert(EntityManager.GetComponent<TransformComponent>(gridTwo.GridEntityId).ChildCount == 0);
        EntityManager.DeleteEntity(gridTwo.GridEntityId);

        return gridId;
    }

    private Vector2i GetAdjustedIndex(Vector2i index, DirectionFlag direction)
    {
        return direction switch
        {
            DirectionFlag.East => index,
            DirectionFlag.South => new Vector2i(index.Y, -index.X),
            DirectionFlag.West => new Vector2i(-index.X, -index.Y),
            DirectionFlag.North => new Vector2i(-index.Y, index.X),
            _ => throw new ArgumentOutOfRangeException()
        };
    }

    private Vector2 GetNewLocalPos(Vector2 oldLocalPos, Vector2i origin, DirectionFlag direction)
    {
        throw new NotImplementedException();
    }
}
