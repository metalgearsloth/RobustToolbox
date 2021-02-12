﻿using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Utility;

namespace Robust.Shared.Containers
{
    /// <summary>
    /// Helper functions for the container system.
    /// </summary>
    public static class ContainerHelpers
    {
        /// <summary>
        /// Am I inside a container?
        /// </summary>
        /// <param name="entity">Entity that might be inside a container.</param>
        /// <returns>If the entity is inside of a container.</returns>
        public static bool IsInContainer(this IEntity entity)
        {
            DebugTools.AssertNotNull(entity);
            DebugTools.Assert(!entity.Deleted);

            // Notice the recursion starts at the Owner of the passed in entity, this
            // allows containers inside containers (toolboxes in lockers).
            if (entity.Transform.Parent != null)
                if (TryGetManagerComp(entity.Transform.Parent.Owner, out var containerComp))
                    return containerComp.ContainsEntity(entity);

            return false;
        }

        /// <summary>
        /// Tries to find the container manager that this entity is inside (if any).
        /// </summary>
        /// <param name="entity">Entity that might be inside a container.</param>
        /// <param name="manager">The container manager that this entity is inside of.</param>
        /// <returns>If a container manager was found.</returns>
        public static bool TryGetContainerMan(this IEntity entity, [NotNullWhen(true)] out IContainerManager? manager)
        {
            DebugTools.AssertNotNull(entity);
            DebugTools.Assert(!entity.Deleted);

            if (entity.Transform.Parent != null && TryGetManagerComp(entity.Transform.Parent.Owner, out manager) && manager.ContainsEntity(entity))
                return true;

            manager = default;
            return false;
        }

        /// <summary>
        /// Tries to find the container that this entity is inside (if any).
        /// </summary>
        /// <param name="entity">Entity that might be inside a container.</param>
        /// <param name="container">The container that this entity is inside of.</param>
        /// <returns>If a container was found.</returns>
        public static bool TryGetContainer(this IEntity entity, [NotNullWhen(true)] out IContainer? container)
        {
            DebugTools.AssertNotNull(entity);
            DebugTools.Assert(!entity.Deleted);

            if (TryGetContainerMan(entity, out var manager))
                return manager.TryGetContainer(entity, out container);

            container = default;
            return false;
        }

        /// <summary>
        ///     Attempts to remove an entity from its container, if any.
        /// </summary>
        /// <param name="entity">Entity that might be inside a container.</param>
        /// <param name="force">Whether to forcibly remove the entity from the container.</param>
        /// <param name="wasInContainer">Whether the entity was actually inside a container or not.</param>
        /// <returns>If the entity could be removed. Also returns false if it wasn't inside a container.</returns>
        public static bool TryRemoveFromContainer(this IEntity entity, bool force, out bool wasInContainer)
        {
            DebugTools.AssertNotNull(entity);
            DebugTools.Assert(!entity.Deleted);

            if (TryGetContainer(entity, out var container))
            {
                wasInContainer = true;

                if (!force)
                    return container.Remove(entity);

                container.ForceRemove(entity);
                return true;

            }

            wasInContainer = false;
            return false;
        }

        /// <summary>
        ///     Attempts to remove an entity from its container, if any.
        /// </summary>
        /// <param name="entity">Entity that might be inside a container.</param>
        /// <param name="force">Whether to forcibly remove the entity from the container.</param>
        /// <returns>If the entity could be removed. Also returns false if it wasn't inside a container.</returns>
        public static bool TryRemoveFromContainer(this IEntity entity, bool force = false)
        {
            return TryRemoveFromContainer(entity, force, out _);
        }

        /// <summary>
        ///     Attempts to remove all entities in a container.
        /// </summary>
        public static void EmptyContainer(this IContainer container, bool force = false, EntityCoordinates? moveTo = null)
        {
            foreach (var entity in container.ContainedEntities.ToArray())
            {
                if (entity.Deleted) continue;

                if (force)
                    container.ForceRemove(entity);
                else
                    container.Remove(entity);

                if (moveTo.HasValue)
                    entity.Transform.Coordinates = moveTo.Value;
            }
        }

        /// <summary>
        ///     Attempts to remove and delete all entities in a container.
        /// </summary>
        public static void CleanContainer(this IContainer container)
        {
            foreach (var ent in container.ContainedEntities.ToArray())
            {
                if (ent.Deleted) continue;
                container.ForceRemove(ent);
                ent.Delete();
            }
        }

        public static void AttachParentToContainerOrGrid(this ITransformComponent transform)
        {
            if (transform.Parent == null
                || !TryGetContainer(transform.Parent.Owner, out var container)
                || !TryInsertIntoContainer(transform, container))
            {
                transform.AttachToGridOrMap();
            }
        }

        private static bool TryInsertIntoContainer(this ITransformComponent transform, IContainer container)
        {
            if (container.Insert(transform.Owner))
            {
                return true;
            }

            if (container.Owner.Transform.Parent != null
                && TryGetContainer(container.Owner, out var newContainer))
            {
                return TryInsertIntoContainer(transform, newContainer);
            }

            return false;
        }

        private static bool TryGetManagerComp(this IEntity entity, [NotNullWhen(true)] out IContainerManager? manager)
        {
            DebugTools.AssertNotNull(entity);
            DebugTools.Assert(!entity.Deleted);

            if (entity.TryGetComponent(out manager))
                return true;

            // RECURSION ALERT
            if (entity.Transform.Parent != null)
                return TryGetManagerComp(entity.Transform.Parent.Owner, out manager);

            return false;
        }

        public static bool IsInSameOrNoContainer(this IEntity user, IEntity other)
        {
            DebugTools.AssertNotNull(user);
            DebugTools.AssertNotNull(other);

            var isUserContained = TryGetContainer(user, out var userContainer);
            var isOtherContained = TryGetContainer(other, out var otherContainer);

            // Both entities are not in a container
            if (!isUserContained && !isOtherContained)
            {
                return true;
            }

            // Both entities are in different contained states
            if (isUserContained != isOtherContained)
            {
                return false;
            }

            // Both entities are in the same container
            return userContainer == otherContainer;
        }
    }
}
