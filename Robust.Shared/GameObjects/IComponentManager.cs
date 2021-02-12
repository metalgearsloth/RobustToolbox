﻿using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using JetBrains.Annotations;

namespace Robust.Shared.GameObjects
{
    /// <summary>
    ///     Holds a collection of ECS components that are attached to entities.
    /// </summary>
    [PublicAPI]
    public interface IComponentManager
    {
        /// <summary>
        ///     A component was added to the manager.
        /// </summary>
        event EventHandler<ComponentEventArgs>? ComponentAdded;

        /// <summary>
        ///     A component was removed from the manager.
        /// </summary>
        event EventHandler<ComponentEventArgs>? ComponentRemoved;

        /// <summary>
        ///     A component was deleted. This is usually deferred until some time after it was removed.
        ///     Usually you will want to subscribe to <see cref="ComponentRemoved"/>.
        /// </summary>
        event EventHandler<ComponentEventArgs>? ComponentDeleted;

        /// <summary>
        ///     Instantly clears all components from the manager. This will NOT shut them down gracefully.
        ///     Any entities relying on existing components will be broken.
        /// </summary>
        void Clear();

        void Initialize();

        /// <summary>
        ///     Adds a Component type to an entity. If the entity is already Initialized, the component will
        ///     automatically be Initialized and Started.
        /// </summary>
        /// <typeparam name="T">Concrete component type to add.</typeparam>
        /// <returns>The newly added component.</returns>
        T AddComponent<T>(IEntity entity) where T : Component, new();

        /// <summary>
        ///     Adds a Component to an entity. If the entity is already Initialized, the component will
        ///     automatically be Initialized and Started.
        /// </summary>
        /// <param name="entity">Entity being modified.</param>
        /// <param name="component">Component to add.</param>
        /// <param name="overwrite">Should it overwrite existing components?</param>
        void AddComponent<T>(IEntity entity, T component, bool overwrite = false) where T : Component;

        /// <summary>
        ///     Removes the component with the specified reference type,
        ///     Without needing to have the component itself.
        /// </summary>
        /// <typeparam name="T">The component reference type to remove.</typeparam>
        /// <param name="uid">Entity UID to modify.</param>
        void RemoveComponent<T>(EntityUid uid);

        /// <summary>
        ///     Removes the component with a specified type.
        /// </summary>
        /// <param name="uid">Entity UID to modify.</param>
        /// <param name="type">A trait or component type to check for.</param>
        void RemoveComponent(EntityUid uid, Type type);

        /// <summary>
        ///     Removes the component with a specified network ID.
        /// </summary>
        /// <param name="uid">Entity UID to modify.</param>
        /// <param name="netID">Network ID of the component to remove.</param>
        void RemoveComponent(EntityUid uid, uint netID);

        /// <summary>
        ///     Removes the specified component.
        /// </summary>
        /// <param name="uid">Entity UID to modify.</param>
        /// <param name="component">Component to remove.</param>
        void RemoveComponent(EntityUid uid, IComponent component);

        /// <summary>
        ///     Removes all components from an entity, except the required components.
        /// </summary>
        /// <param name="uid">Entity UID to modify.</param>
        void RemoveComponents(EntityUid uid);

        /// <summary>
        ///     Removes ALL components from an entity. This includes the required components,
        ///     <see cref="TransformComponent"/> and <see cref="MetaDataComponent"/>. This should ONLY be
        ///     used when deleting an entity.
        /// </summary>
        /// <param name="uid">Entity UID to modify.</param>
        void DisposeComponents(EntityUid uid);

        /// <summary>
        ///     Checks if the entity has a component type.
        /// </summary>
        /// <typeparam name="T">Component reference type to check for.</typeparam>
        /// <param name="uid">Entity UID to check.</param>
        /// <returns>True if the entity has the component type, otherwise false.</returns>
        bool HasComponent<T>(EntityUid uid);

        /// <summary>
        ///     Checks if the entity has a component type.
        /// </summary>
        /// <param name="uid">Entity UID to check.</param>
        /// <param name="type">A trait or component type to check for.</param>
        /// <returns>True if the entity has the component type, otherwise false.</returns>
        bool HasComponent(EntityUid uid, Type type);

        /// <summary>
        ///     Checks if the entity has a component with a given network ID.
        /// </summary>
        /// <param name="uid">Entity UID to check.</param>
        /// <param name="netId">Network ID to check for.</param>
        /// <returns>True if the entity has a component with the given network ID, otherwise false.</returns>
        bool HasComponent(EntityUid uid, uint netId);

        /// <summary>
        ///     Returns the component of a specific type.
        /// </summary>
        /// <typeparam name="T">A trait or type of a component to retrieve.</typeparam>
        /// <param name="uid">Entity UID to look on.</param>
        /// <returns>The component of Type from the Entity.</returns>
        T GetComponent<T>(EntityUid uid);

        /// <summary>
        ///     Returns the component of a specific type.
        /// </summary>
        /// <param name="uid">Entity UID to look on.</param>
        /// <param name="type">A trait or component type to check for.</param>
        /// <returns>The component of Type from the Entity.</returns>
        IComponent GetComponent(EntityUid uid, Type type);

        /// <summary>
        ///     Returns the component with a specific network ID.
        /// </summary>
        /// <param name="uid">Entity UID to look on.</param>
        /// <param name="netId">Network ID of the component to retrieve.</param>
        /// <returns>The component with the specified network id.</returns>
        IComponent GetComponent(EntityUid uid, uint netId);

        /// <summary>
        ///     Returns the component of a specific type.
        /// </summary>
        /// <typeparam name="T">A trait or type of a component to retrieve.</typeparam>
        /// <param name="uid">Entity UID to check.</param>
        /// <param name="component">Component of the specified type (if exists).</param>
        /// <returns>If the component existed in the entity.</returns>
        bool TryGetComponent<T>(EntityUid uid, [NotNullWhen(true)] out T component);

        /// <summary>
        ///     Returns the component of a specific type.
        /// </summary>
        /// <param name="uid">Entity UID to check.</param>
        /// <param name="type">A trait or component type to check for.</param>
        /// <param name="component">Component of the specified type (if exists).</param>
        /// <returns>If the component existed in the entity.</returns>
        bool TryGetComponent(EntityUid uid, Type type, [NotNullWhen(true)] out IComponent? component);

        /// <summary>
        ///     Returns the component with a specified network ID.
        /// </summary>
        /// <param name="uid">Entity UID to check.</param>
        /// <param name="netId">Component Network ID to check for.</param>
        /// <param name="component">Component with the specified network id.</param>
        /// <returns>If the component existed in the entity.</returns>
        bool TryGetComponent(EntityUid uid, uint netId, [NotNullWhen(true)] out IComponent? component);

        /// <summary>
        ///     Returns ALL component type instances on an entity. A single component instance
        ///     can have multiple component types.
        /// </summary>
        /// <param name="uid">Entity UID to look on.</param>
        /// <returns>All component types on the Entity.</returns>
        IEnumerable<IComponent> GetComponents(EntityUid uid);

        /// <summary>
        ///     Returns ALL component type instances that are assignable to the specified type.
        ///     A single component instance can have multiple component type instances.
        /// </summary>
        /// <typeparam name="T">A trait or type of a component to retrieve.</typeparam>
        /// <param name="uid">Entity UID to look on.</param>
        /// <returns>All components that are assignable to the specified type.</returns>
        IEnumerable<T> GetComponents<T>(EntityUid uid);

        /// <summary>
        ///     Returns ALL networked components on an entity, including deleted ones.
        /// </summary>
        /// <param name="uid">Entity UID to look on.</param>
        /// <returns>All components that have a network ID.</returns>
        IEnumerable<IComponent> GetNetComponents(EntityUid uid);

        /// <summary>
        ///     Returns ALL component instances of a specified type.
        /// </summary>
        /// <typeparam name="T">A trait or type of a component to retrieve.</typeparam>
        /// <returns>All components that have the specified type.</returns>
        IEnumerable<T> EntityQuery<T>(bool includePaused = false);

        /// <summary>
        /// Returns the relevant components from all entities that contain the two required components.
        /// </summary>
        /// <typeparam name="TComp1">First required component.</typeparam>
        /// <typeparam name="TComp2">Second required component.</typeparam>
        /// <returns>The pairs of components from each entity that has the two required components.</returns>
        IEnumerable<(TComp1, TComp2)> EntityQuery<TComp1, TComp2>(bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent;

        /// <summary>
        /// Returns the relevant components from all entities that contain the three required components.
        /// </summary>
        /// <typeparam name="TComp1">First required component.</typeparam>
        /// <typeparam name="TComp2">Second required component.</typeparam>
        /// <typeparam name="TComp3">Third required component.</typeparam>
        /// <returns>The pairs of components from each entity that has the three required components.</returns>
        IEnumerable<(TComp1, TComp2, TComp3)> EntityQuery<TComp1, TComp2, TComp3>(bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent;

        /// <summary>
        /// Returns the relevant components from all entities that contain the four required components.
        /// </summary>
        /// <typeparam name="TComp1">First required component.</typeparam>
        /// <typeparam name="TComp2">Second required component.</typeparam>
        /// <typeparam name="TComp3">Third required component.</typeparam>
        /// <typeparam name="TComp4">Fourth required component.</typeparam>
        /// <returns>The pairs of components from each entity that has the four required components.</returns>
        IEnumerable<(TComp1, TComp2, TComp3, TComp4)> EntityQuery<TComp1, TComp2, TComp3, TComp4>(bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
            where TComp4 : IComponent;

        /// <summary>
        ///      Returns ALL component instances of a specified type.
        /// </summary>
        /// <param name="type">A trait or component type to check for.</param>
        /// <param name="includePaused"></param>
        /// <returns>All components that are the specified type.</returns>
        IEnumerable<IComponent> GetAllComponents(Type type, bool includePaused = false);

        /// <summary>
        ///     Culls all components from the collection that are marked as deleted. This needs to be called often.
        /// </summary>
        void CullRemovedComponents();
    }
}
