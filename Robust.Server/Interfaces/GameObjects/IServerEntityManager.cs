using Robust.Shared.GameObjects;
using Robust.Shared.Interfaces.GameObjects;
using System.Collections.Generic;
using JetBrains.Annotations;
using Robust.Server.Interfaces.Player;
using Robust.Shared.Timing;

namespace Robust.Server.Interfaces.GameObjects
{
    public interface IServerEntityManager : IEntityManager
    {
        /// <summary>
        ///     Gets all entity states that have been modified after and including the provided tick.
        /// </summary>
        List<EntityState>? GetEntityStates(GameTick fromTick);

        /// <summary>
        ///     Gets all entity states that have been modified after and including the provided tick for a particular session.
        /// </summary>
        List<EntityState>? GetEntityStates(GameTick fromTick, IPlayerSession session, float range);

        // Keep track of deleted entities so we can sync deletions with the client.
        /// <summary>
        ///     Gets a list of all entity UIDs that were deleted between <paramref name="fromTick" /> and now.
        /// </summary>
        List<EntityUid>? GetDeletedEntities(GameTick fromTick);

        /// <summary>
        ///     Remove deletion history.
        /// </summary>
        /// <param name="toTick">The last tick to delete the history for. Inclusive.</param>
        void CullDeletionHistory(GameTick toTick);

        float MaxUpdateRange { get; }

    }
}
