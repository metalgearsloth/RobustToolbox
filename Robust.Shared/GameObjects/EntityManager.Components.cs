using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using Arch.Core;
using Arch.Core.Utils;
using JetBrains.Annotations;
using Robust.Shared.GameStates;
using Robust.Shared.Log;
using Robust.Shared.Physics.Components;
using Robust.Shared.Player;
using Robust.Shared.Prototypes;
using Robust.Shared.Timing;
using Robust.Shared.Utility;
using ComponentRegistry = Robust.Shared.Prototypes.ComponentRegistry;
#if EXCEPTION_TOLERANCE
using Robust.Shared.Exceptions;
#endif

namespace Robust.Shared.GameObjects
{
    public partial class EntityManager
    {
        [IoC.Dependency] private IComponentFactory _componentFactory = default!;

#if EXCEPTION_TOLERANCE
        [IoC.Dependency] private readonly IRuntimeLog _runtimeLog = default!;
#endif

        public IComponentFactory ComponentFactory => _componentFactory;

        private const int TypeCapacity = 32;
        private const int EntityCapacity = 1024;

        private readonly HashSet<IComponent> _deleteSet = new(TypeCapacity);

        /// <inheritdoc />
        public event Action<AddedComponentEventArgs>? ComponentAdded;

        /// <inheritdoc />
        public event Action<RemovedComponentEventArgs>? ComponentRemoved;

        public void InitializeComponents()
        {
            if (Initialized)
                throw new InvalidOperationException("Already initialized.");

            FillComponentDict();
            _componentFactory.ComponentsAdded += OnComponentsAdded;
        }

        private void OnComponentsAdded(ComponentRegistration[] components)
        {
            RegisterComponents(components);
        }

        /// <summary>
        ///     Instantly clears all components from the manager. This will NOT shut them down gracefully.
        ///     Any entities relying on existing components will be broken.
        /// </summary>
        public void ClearComponents()
        {
            _deleteSet.Clear();
        }

        private void RegisterComponents(IEnumerable<ComponentRegistration> components)
        {
            // NOOP
        }

        #region Component Management

        /// <inheritdoc />
        public int Count<T>() where T : IComponent
        {
            return _world.CountEntities(AllEntityQueryDescription<T>.Value);
        }

        /// <inheritdoc />
        public int Count(Type type)
        {
            DebugTools.Assert(type.IsAssignableTo(typeof(IComponent)));
            return _world.CountEntities(new QueryDescription(all: [type]));
        }

        [Obsolete("Use InitializeEntity")]
        public void InitializeComponents(EntityUid uid, MetaDataComponent? metadata = null)
        {
            DebugTools.AssertOwner(uid, metadata);
            metadata ??= MetaQuery.GetComponent(uid);
            DebugTools.Assert(metadata.EntityLifeStage == EntityLifeStage.PreInit);
            SetLifeStage(metadata, EntityLifeStage.Initializing);

            // Initialize() can modify the collection of components. Copy them.
            FixedArray32<IComponent?> compsFixed = default;

            var comps = compsFixed.AsSpan;
            CopyComponentsInto(ref comps, uid);

            foreach (var comp in comps)
            {
                if (comp is {LifeStage: ComponentLifeStage.Added})
                    LifeInitialize(uid, comp, _componentFactory.GetIndex(comp.GetType()));
            }

            DebugTools.Assert(metadata.EntityLifeStage == EntityLifeStage.Initializing);
            SetLifeStage(metadata, EntityLifeStage.Initialized);
        }

        [Obsolete("Use StartEntity")]
        public void StartComponents(EntityUid uid)
        {
            // Startup() can modify _components
            // This code can only handle additions to the list. Is there a better way? Probably not.
            FixedArray32<IComponent?> compsFixed = default;

            var comps = compsFixed.AsSpan;
            CopyComponentsInto(ref comps, uid);

            // TODO: please for the love of god remove these initialization order hacks.

            // Init transform first, we always have it.
            var transform = TransformQuery.GetComponent(uid);
            if (transform.LifeStage == ComponentLifeStage.Initialized)
                LifeStartup(uid, transform, CompIdx.Index<TransformComponent>());

            // Init physics second if it exists.
            if (_physicsQuery.TryComp(uid, out var phys) && phys.LifeStage == ComponentLifeStage.Initialized)
            {
                LifeStartup(uid, phys, CompIdx.Index<PhysicsComponent>());
            }

            // Do rest of components.
            foreach (var comp in comps)
            {
                if (comp is { LifeStage: ComponentLifeStage.Initialized })
                    LifeStartup(uid, comp, _componentFactory.GetIndex(comp.GetType()));
            }
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void AddComponents(EntityUid target, EntityPrototype prototype, bool removeExisting = true)
        {
            AddComponents(target, prototype.Components, removeExisting);
        }

        /// <inheritdoc />
        public void AddComponents(EntityUid target, ComponentRegistry registry, bool removeExisting = true)
        {
            if (registry.Count == 0)
                return;

            var metadata = MetaQuery.GetComponent(target);

            foreach (var (name, entry) in registry)
            {
                var reg = _componentFactory.GetRegistration(name);

                if (removeExisting)
                {
                    var comp = _componentFactory.GetComponent(reg);
                    _serManager.CopyTo(entry.Component, ref comp, notNullableOverride: true);
                    comp.Owner = target;
                    AddComponentInternal(target, comp, reg, skipInit: false, overwrite: true, metadata: metadata);
                }
                else
                {
                    if (HasComponent(target, reg))
                    {
                        continue;
                    }

                    var comp = _componentFactory.GetComponent(reg);
                    _serManager.CopyTo(entry.Component, ref comp, notNullableOverride: true);
                    comp.Owner = target;
                    AddComponentInternal(target, comp, reg, skipInit: false, overwrite: false, metadata: metadata);
                }
            }
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveComponents(EntityUid target, EntityPrototype prototype)
        {
            RemoveComponents(target, prototype.Components);
        }

        /// <inheritdoc />
        public void RemoveComponents(EntityUid target, ComponentRegistry registry)
        {
            if (registry.Count == 0)
                return;

            var metadata = MetaQuery.GetComponent(target);

            foreach (var entry in registry.Values)
            {
                RemoveComponent(target, entry.Component.GetType(), metadata);
            }
        }

        public IComponent AddComponent(EntityUid uid, ushort netId, MetaDataComponent? meta = null)
        {
            var newComponent = _componentFactory.GetComponent(netId);
            AddComponent(uid, newComponent, metadata: meta);
            return newComponent;
        }

        public T AddComponent<T>(EntityUid uid) where T : IComponent, new()
        {
            var newComponent = _componentFactory.GetComponent<T>();
            AddComponent(uid, newComponent);
            return newComponent;
        }

        public readonly struct CompInitializeHandle<T> : IDisposable
            where T : IComponent
        {
            private readonly IEntityManager _entMan;
            private readonly EntityUid _owner;
            public readonly CompIdx CompType;
            public readonly T Comp;

            public CompInitializeHandle(IEntityManager entityManager, EntityUid owner, T comp, CompIdx compType)
            {
                _entMan = entityManager;
                _owner = owner;
                Comp = comp;
                CompType = compType;
            }

            public void Dispose()
            {
                var metadata = _entMan.GetComponent<MetaDataComponent>(_owner);

                if (!metadata.EntityInitialized && !metadata.EntityInitializing)
                    return;

                if (!Comp.Initialized)
                    ((EntityManager) _entMan).LifeInitialize(_owner, Comp, CompType);

                if (metadata.EntityInitialized && !Comp.Running)
                    ((EntityManager) _entMan).LifeStartup(_owner, Comp, CompType);
            }

            public static implicit operator T(CompInitializeHandle<T> handle)
            {
                return handle.Comp;
            }
        }

        public void AddComponent(
            EntityUid uid,
            EntityPrototype.ComponentRegistryEntry entry,
            MetaDataComponent? metadata = null)
        {
            var compReg = _componentFactory.GetRegistration(entry.Component.GetType());
            var copy = _componentFactory.GetComponent(compReg);
            AddComponentInternal(uid, copy, compReg, skipInit: false, overwrite: true, metadata);
        }

        /// <inheritdoc />
        public void AddComponent<T>(EntityUid uid, T component, MetaDataComponent? metadata = null) where T : IComponent
        {
            if (!MetaQuery.Resolve(uid, ref metadata, false))
                throw new ArgumentException($"Entity {uid} is not valid.", nameof(uid));

            if (component == null)
                throw new ArgumentNullException(nameof(component));

#pragma warning disable CS0618 // Type or member is obsolete
            if (component.Owner == default)
            {
                component.Owner = uid;
            }
            else if (component.Owner != uid)
            {
                throw new InvalidOperationException("Component is not owned by entity.");
            }
#pragma warning restore CS0618 // Type or member is obsolete
            var compReg = _componentFactory.GetRegistration(component);

            AddComponentInternal(uid, component, compReg, false, overwrite: false, metadata);
        }

        internal void AddComponentInternalOnly<T>(
            EntityUid uid, T component,
            ComponentRegistration reg,
            bool overwrite = false,
            MetaDataComponent? metadata = null) where T : IComponent
        {
            ThreadCheck();

            NormalizeComponentOwner(uid, component);
            AssertCanAddComponent(uid, component, reg, metadata);

            var archUid = ToArch(uid);
            if (overwrite && _world.Has(archUid, reg.ArchType))
            {
                RemoveExistingComponentForOverwrite(uid, archUid, reg, metadata);
                SetComponentInternalOnly(uid, component, reg, metadata);
                return;
            }

            DebugTools.Assert(!_world.Has(archUid, reg.ArchType));
            if (_world.Has(archUid, reg.ArchType))
                return;

            _world.Add(archUid, (object)component);

            metadata ??= MetaQuery.GetComponentInternal(uid);
            FinishComponentStorage(component, reg, metadata);
        }

        internal void SetComponentInternalOnly<T>(
            EntityUid uid, T component,
            ComponentRegistration reg,
            MetaDataComponent? metadata = null) where T : IComponent
        {
            ThreadCheck();

            NormalizeComponentOwner(uid, component);
            AssertCanAddComponent(uid, component, reg, metadata);

            var archUid = ToArch(uid);
            DebugTools.Assert(_world.Has(archUid, reg.ArchType));
            _world.Set(archUid, (object)component);

            metadata ??= MetaQuery.GetComponentInternal(uid);
            FinishComponentStorage(component, reg, metadata);
        }

        private void RemoveExistingComponentForOverwrite(
            EntityUid uid,
            Entity archUid,
            ComponentRegistration reg,
            MetaDataComponent? metadata)
        {
            if (!_world.TryGet(archUid, reg.ArchType, out var existing) || existing is not IComponent existingComp)
                return;

            var existingOwnerMatches = false;
#pragma warning disable CS0618
            existingOwnerMatches = existingComp.Owner == uid;
#pragma warning restore CS0618

            if (existingOwnerMatches && existingComp.LifeStage != ComponentLifeStage.PreAdd)
            {
                DebugTools.AssertOwner(uid, existingComp);
                RemoveComponentImmediate(uid, existingComp, reg.Idx, false, false, meta: metadata);
            }
        }

        private void NormalizeComponentOwner<T>(EntityUid uid, T component)
            where T : IComponent
        {
#pragma warning disable CS0618
            component.Owner = uid;
#pragma warning restore CS0618
            DebugTools.AssertOwner(uid, component);
        }

        private void AssertCanAddComponent<T>(EntityUid uid, T component, ComponentRegistration reg, MetaDataComponent? metadata)
            where T : IComponent
        {
            // We can't use typeof(T) here in case T is just Component
            DebugTools.Assert(component is MetaDataComponent ||
                              (metadata ?? MetaQuery.GetComponent(uid)).EntityLifeStage < EntityLifeStage.Terminating,
                $"Attempted to add a {component.GetType().Name} component to an entity ({ToPrettyString(uid)}) while it is terminating");

            // We can't use typeof(T) here in case T is just Component
            DebugTools.Assert(component is MetaDataComponent ||
                              (metadata ?? MetaQuery.GetComponent(uid)).EntityLifeStage < EntityLifeStage.Terminating,
                $"Attempted to add a {reg.Name} component to an entity ({ToPrettyString(uid)}) while it is terminating");
        }

        private void FinishComponentStorage<T>(T component, ComponentRegistration reg, MetaDataComponent metadata)
            where T : IComponent
        {
            // add the component to the netId grid
            if (reg.NetID != null && component.NetSyncEnabled)
                metadata.NetComponents.Add(reg.NetID.Value, component);

            if (component is IComponentDelta delta)
            {
                var curTick = _gameTiming.CurTick;
                delta.LastModifiedFields = new GameTick[reg.NetworkedFields.Length];
                Array.Fill(delta.LastModifiedFields, curTick);
            }

            component.Networked = reg.NetID != null;
        }

        internal void AddComponentEvents<T>(EntityUid uid, T component, ComponentRegistration reg, bool skipInit,
            MetaDataComponent? metadata = null) where T : IComponent
        {
            var eventArgs = new AddedComponentEventArgs(new ComponentEventArgs(component, uid), reg);
            ComponentAdded?.Invoke(eventArgs);
            EventBusInternal.OnComponentAdded(eventArgs);

            LifeAddToEntity(uid, component, reg.Idx);

            if (skipInit)
                return;

            metadata ??= MetaQuery.GetComponentInternal(uid);

            // Bur this overhead sucks.
            if (metadata.EntityLifeStage < EntityLifeStage.Initializing)
                return;

            if (component.Networked)
                DirtyEntity(uid, metadata);

            LifeInitialize(uid, component, reg.Idx);

            if (metadata.EntityInitialized)
                LifeStartup(uid, component, reg.Idx);

            if (metadata.EntityLifeStage >= EntityLifeStage.MapInitialized)
                EventBusInternal.RaiseComponentEvent(uid, component, reg.Idx, MapInitEventInstance);
        }

        internal void AddComponentInternal<T>(EntityUid uid, T component, ComponentRegistration reg, bool skipInit, bool overwrite = false, MetaDataComponent? metadata = null) where T : IComponent
        {
            AddComponentInternalOnly(uid, component, reg, overwrite: overwrite, metadata);
            AddComponentEvents(uid, component, reg, skipInit, metadata);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveComponent<T>(EntityUid uid, MetaDataComponent? meta = null) where T : IComponent
        {
            if (!TryGetComponent(uid, out T? comp))
                return false;

            RemoveComponentImmediate(uid, comp, CompIdx.Index<T>(), terminating: false, archetypeChange: true, meta: meta);
            return true;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveComponent(EntityUid uid, Type type, MetaDataComponent? meta = null)
        {
            if (!TryGetComponent(uid, type, out var comp))
                return false;

            RemoveComponentImmediate(uid, comp, _componentFactory.GetIndex(type), false, true, meta);
            return true;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveComponent(EntityUid uid, ushort netId, MetaDataComponent? meta = null)
        {
            if (!MetaQuery.Resolve(uid, ref meta))
                return false;

            if (!TryGetComponent(uid, netId, out var comp, meta))
                return false;

            RemoveComponentImmediate(uid, comp, _componentFactory.GetIndex(comp.GetType()), false, true, meta);
            return true;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveComponent(EntityUid uid, IComponent component, MetaDataComponent? meta = null)
        {
            RemoveComponentImmediate(uid, component, _componentFactory.GetIndex(component.GetType()), false, true, meta);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveComponentDeferred<T>(EntityUid uid)
        {
            return RemoveComponentDeferred(uid, typeof(T));
        }

        /// <inheritdoc />
        public bool RemoveComponentDeferred(EntityUid uid, Type type)
        {
            if (!TryGetComponent(uid, type, out var comp))
                return false;

            RemoveComponentDeferred(comp, uid, false);
            return true;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool RemoveComponentDeferred(EntityUid uid, ushort netId, MetaDataComponent? meta = null)
        {
            if (!MetaQuery.Resolve(uid, ref meta))
                return false;

            if (!TryGetComponent(uid, netId, out var comp, meta))
                return false;

            RemoveComponentDeferred(comp, uid, false);
            return true;
        }

        /// <inheritdoc />
        public void RemoveComponentDeferred(EntityUid owner, IComponent component)
        {
            RemoveComponentDeferred(component, owner, false);
        }

        /// <inheritdoc />
        public void RemoveComponents(EntityUid uid, MetaDataComponent? meta = null)
        {
            var objComps = _world.GetAllComponents(ToArch(uid));
            // Reverse order
            for (var i = objComps.Length - 1; i >= 0; i--)
            {
                var comp = (IComponent) objComps[i]!;
                RemoveComponentImmediate(uid, comp, _componentFactory.GetIndex(comp.GetType()), terminating: false, archetypeChange: false, meta);
            }
        }

        /// <inheritdoc />
        public void DisposeComponents(EntityUid uid, MetaDataComponent? meta = null)
        {
            var objComps = _world.GetAllComponents(ToArch(uid));

            // Reverse order
            for (var i = objComps.Length - 1; i >= 0; i--)
            {
                var comp = (IComponent) objComps[i]!;

                try
                {
#pragma warning disable CS0618
                    comp.Owner = uid;
#pragma warning restore CS0618
                    RemoveComponentImmediate(uid, comp, _componentFactory.GetIndex(comp.GetType()), terminating: true, archetypeChange: false, meta);
                }
                catch (Exception exc)
                {
                    _sawmill.Error($"Caught exception while trying to remove component {_componentFactory.GetComponentName(comp.GetType())} from entity '{ToPrettyString(uid)}'\n{exc.StackTrace}");
                }
            }
        }

        private void RemoveComponentDeferred(IComponent component, EntityUid uid, bool terminating)
        {
            if (component == null)
                throw new ArgumentNullException(nameof(component));

#pragma warning disable CS0618 // Type or member is obsolete
            if (component.Owner != uid)
#pragma warning restore CS0618 // Type or member is obsolete
                throw new InvalidOperationException("Component is not owned by entity.");

            if (component.Deleted)
                return;

#if EXCEPTION_TOLERANCE
            try
            {
#endif
            // these two components are required on all entities and cannot be removed normally.
            if (!terminating && component is TransformComponent or MetaDataComponent)
            {
                DebugTools.Assert("Tried to remove a protected component.");
                return;
            }

            if (!_deleteSet.Add(component))
            {
                // Already deferring deletion
                DebugTools.Assert(component.LifeStage >= ComponentLifeStage.Stopped);
                return;
            }

            DebugTools.Assert(component.LifeStage >= ComponentLifeStage.Added);

            if (component.LifeStage is >= ComponentLifeStage.Initialized and < ComponentLifeStage.Stopping)
                LifeShutdown(uid, component, _componentFactory.GetIndex(component.GetType()));
            else if (component.LifeStage == ComponentLifeStage.Added)
            {
                // The component was added, but never initialized or started. It's kinda weird to add and then
                // immediately defer-remove a component, but oh well. Let's just set the life stage directly and not
                // raise shutdown events? The removal events will still get called later.
                // This is also what LifeShutdown() would also do, albeit behind a DebugAssert.
                component.LifeStage = ComponentLifeStage.Stopped;
            }
#if EXCEPTION_TOLERANCE
            }
            catch (Exception e)
            {
                _sawmill.Error($"Caught exception while queuing deferred component removal. Entity={ToPrettyString(component.Owner)}, type={component.GetType()}");
                _runtimeLog.LogException(e, nameof(RemoveComponentDeferred));
            }
#endif
        }

        private void ThrowPreAddRemovalException(EntityUid target, IComponent component)
        {
            throw new InvalidOperationException(
                $"Removing a component, {component.GetType()} before it has been added is probably not what you wanted to do. Target entity was {ToPrettyString(target)}.");
        }

        private void RemoveComponentImmediate(
            EntityUid uid,
            IComponent component,
            CompIdx idx,
            bool terminating,
            bool archetypeChange,
            MetaDataComponent? meta = null)
        {
            ThreadCheck();
            DebugTools.AssertOwner(uid, component);

            if (component.LifeStage == ComponentLifeStage.PreAdd)
            {
                ThrowPreAddRemovalException(uid, component);
            }

            if (component.Deleted)
            {
                _sawmill.Warning($"Deleting an already deleted component. Entity: {ToPrettyString(uid)}, Component: {_componentFactory.GetComponentName(component.GetType())}.");
                return;
            }

#if EXCEPTION_TOLERANCE
            try
            {
#endif
            // these two components are required on all entities and cannot be removed.
            if (!terminating && component is TransformComponent or MetaDataComponent)
            {
                DebugTools.Assert("Tried to remove a protected component.");
                return;
            }

            if (component.Running)
                LifeShutdown(uid, component, idx);

            if (component.LifeStage != ComponentLifeStage.PreAdd)
                LifeRemoveFromEntity(uid, component, idx); // Sets delete

#if EXCEPTION_TOLERANCE
            }
            catch (Exception e)
            {
                _sawmill.Error($"Caught exception during immediate component removal. Entity={ToPrettyString(component.Owner)}, type={component.GetType()}");
                _runtimeLog.LogException(e, nameof(RemoveComponentImmediate));
            }
#endif
            DeleteComponent(uid, component, idx, terminating: terminating, archetypeChange: archetypeChange, meta);
        }

        /// <inheritdoc />
        public void CullRemovedComponents()
        {
            foreach (var component in _deleteSet)
            {
                if (component.Deleted)
                    continue;
                var uid = component.Owner;
                var idx = _componentFactory.GetIndex(component.GetType());

#if EXCEPTION_TOLERANCE
            try
            {
#endif
                // The component may have been restarted sometime after removal was deferred.
                if (component.Running)
                {
                    // TODO add options to cancel deferred deletion?
                    _sawmill.Warning($"Found a running component while culling deferred deletions, owner={ToPrettyString(uid)}, type={component.GetType()}");
                    LifeShutdown(uid, component, idx);
                }

                if (component.LifeStage != ComponentLifeStage.PreAdd)
                    LifeRemoveFromEntity(uid, component, idx);

#if EXCEPTION_TOLERANCE
            }
            catch (Exception e)
            {
                _sawmill.Error($"Caught exception  while processing deferred component removal. Entity={ToPrettyString(component.Owner)}, type={component.GetType()}");
                _runtimeLog.LogException(e, nameof(CullRemovedComponents));
            }
#endif
                DeleteComponent(uid, component, _componentFactory.GetIndex(component.GetType()), terminating: false, archetypeChange: true);
            }

            _deleteSet.Clear();
        }

        private void DeleteComponent(EntityUid entityUid, IComponent component, CompIdx idx, bool terminating, bool archetypeChange, MetaDataComponent? metadata = null)
        {
            if (!MetaQuery.ResolveInternal(entityUid, ref metadata))
                return;

            var eventArgs = new RemovedComponentEventArgs(new ComponentEventArgs(component, entityUid), false, metadata, idx);
            ComponentRemoved?.Invoke(eventArgs);
            EventBusInternal.OnComponentRemoved(eventArgs);

            if (!terminating)
            {
                var reg = _componentFactory.GetRegistration(component);
                DebugTools.Assert(component.Networked == (reg.NetID != null));
                if (reg.NetID != null)
                {
                    if (!metadata.NetComponents.Remove(reg.NetID.Value))
                        _sawmill.Error($"Entity {ToPrettyString(entityUid, metadata)} did not have {component.GetType().Name} in its networked component dictionary during component deletion.");

                    if (component.NetSyncEnabled)
                    {
                        DirtyEntity(entityUid, metadata);
                        metadata.LastComponentRemoved = _gameTiming.CurTick;
                    }
                }
            }

            if (archetypeChange)
            {
                var archUid = ToArch(entityUid);
                DebugTools.Assert(_world.Has(archUid, idx.Type));
                if (_world.Has(archUid, idx.Type))
                    _world.Remove(archUid, idx.Type);
            }

            DebugTools.Assert(_netMan.IsClient // Client side prediction can set LastComponentRemoved to some future tick,
                              || metadata.EntityLastModifiedTick >= metadata.LastComponentRemoved);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent<T>(EntityUid uid) where T : IComponent
        {
            return TryAlive(uid, out var archUid) && _world.TryGet(archUid, out T? comp) && comp != null && !comp.Deleted;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent<T>([NotNullWhen(true)] EntityUid? uid) where T : IComponent
        {
            return uid.HasValue && HasComponent<T>(uid.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent(EntityUid uid, ComponentRegistration reg)
        {
            return TryAlive(uid, out var archUid) && _world.TryGet(archUid, reg.ArchType, out var comp) && comp != null && !((IComponent) comp).Deleted;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent(EntityUid uid, Type type)
        {
            return TryAlive(uid, out var archUid) && _world.TryGet(archUid, type, out var comp) && comp != null && !((IComponent) comp).Deleted;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent([NotNullWhen(true)] EntityUid? uid, Type type)
        {
            if (!uid.HasValue)
            {
                return false;
            }

            return HasComponent(uid.Value, type);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent(EntityUid uid, ushort netId, MetaDataComponent? meta = null)
        {
            if (!MetaQuery.Resolve(uid, ref meta))
                return false;

            return meta.NetComponents.ContainsKey(netId);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent([NotNullWhen(true)] EntityUid? uid, ushort netId, MetaDataComponent? meta = null)
        {
            if (!uid.HasValue)
            {
                DebugTools.AssertNull(meta);
                return false;
            }

            return HasComponent(uid.Value, netId, meta);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T EnsureComponent<T>(EntityUid uid) where T : IComponent, new()
        {
            if (TryGetComponent<T>(uid, out var component))
            {
                // Check for deferred component removal.
                if (component.LifeStage <= ComponentLifeStage.Running)
                    return component;
                RemoveComponent(uid, component);
            }

            return AddComponent<T>(uid);
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool EnsureComponent<T>(ref Entity<T?> entity) where T : IComponent, new()
        {
            if (entity.Comp == null)
                return EnsureComponent<T>(entity.Owner, out entity.Comp);

            DebugTools.AssertOwner(entity, entity.Comp);

            // Check for deferred component removal.
            if (entity.Comp.LifeStage <= ComponentLifeStage.Running)
                return true;

            RemoveComponent(entity, entity.Comp);
            entity.Comp = AddComponent<T>(entity);
            return false;
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool EnsureComponent<T>(EntityUid entity, out T component) where T : IComponent, new()
        {
            if (TryGetComponent<T>(entity, out var comp))
            {
                // Check for deferred component removal.
                if (comp.LifeStage <= ComponentLifeStage.Running)
                {
                    component = comp;
                    return true;
                }

                RemoveComponent(entity, comp);
            }

            component = AddComponent<T>(entity);
            return false;
        }

        /// <inheritdoc />
        [Pure]
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public T GetComponent<T>(EntityUid uid) where T : IComponent
        {
            if (!uid.Valid || !_world.TryGet(ToArch(uid), out T? comp) || comp == null)
            {
                throw new KeyNotFoundException($"Entity {uid} does not have a component of type {typeof(T)}");
            }

            return comp;
        }

        [Pure]
        public IComponent GetComponent(EntityUid uid, CompIdx type)
        {
            if (!uid.Valid || !_world.TryGet(ToArch(uid), type.Type, out var comp) || comp == null)
            {
                throw new KeyNotFoundException($"Entity {uid} does not have a component of type {type.Type}");
            }

            return (IComponent) comp;
        }

        /// <inheritdoc />
        [Pure]
        public IComponent GetComponent(EntityUid uid, Type type)
        {
            if (!uid.Valid || !_world.TryGet(ToArch(uid), type, out var comp) || comp == null)
            {
                throw new KeyNotFoundException($"Entity {uid} does not have a component of type {type}");
            }

            return (IComponent) comp;
        }

        /// <inheritdoc />
        [Pure]
        public IComponent GetComponent(EntityUid uid, ushort netId, MetaDataComponent? meta = null)
        {
            return (meta ?? MetaQuery.GetComponentInternal(uid)).NetComponents[netId];
        }

        [Pure]
        public IComponent GetComponentInternal(EntityUid uid, CompIdx type)
        {
            if (TryGetComponent(uid, type, out var component))
                return component;

            throw new KeyNotFoundException($"Entity {uid} does not have a component of type {type}");
        }

        /// <inheritdoc />
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetComponent<T>(EntityUid uid, [NotNullWhen(true)] out T? component) where T : IComponent?
        {
            if (!uid.Valid)
            {
                component = default;
                return false;
            }

            if (_world.TryGet(ToArch(uid), out component) && component != null)
            {
                if (!component.Deleted)
                {
                    return true;
                }
            }

            component = default;
            return false;
        }

        /// <inheritdoc />
        public bool TryGetComponent<T>([NotNullWhen(true)] EntityUid? uid, [NotNullWhen(true)] out T? component) where T : IComponent?
        {
            if (!uid.HasValue)
            {
                component = default!;
                return false;
            }

            if (TryGetComponent(uid.Value, typeof(T), out var comp))
            {
                if (!comp.Deleted)
                {
                    component = (T)comp;
                    return true;
                }
            }

            component = default;
            return false;
        }

        /// <inheritdoc />
        public bool TryGetComponent(EntityUid uid, ComponentRegistration reg, [NotNullWhen(true)] out IComponent? component)
        {
            return TryGetComponent(uid, reg.ArchType, out component);
        }

        internal bool TryGetComponent<T>(EntityUid uid, ComponentType type, [NotNullWhen(true)] out IComponent? component)
        {
            if (TryAlive(uid, out var archUid) && _world.TryGet(archUid, type, out var comp))
            {
                component = (IComponent) comp;

                if (!component.Deleted)
                {
                    return true;
                }
            }

            component = null;
            return false;
        }

        internal bool TryGetComponent<T>(EntityUid uid, ComponentType type, [NotNullWhen(true)] out T? component) where T : IComponent?
        {
            if (TryAlive(uid, out var archUid) && _world.TryGet(archUid, type, out var comp))
            {
                component = (T) comp;

                if (!component.Deleted)
                {
                    return true;
                }
            }

            component = default;
            return false;
        }

        internal bool HasComponentWithoutLifeCheck(EntityUid uid, ComponentRegistration reg)
        {
            return _world.TryGet(ToArch(uid), reg.ArchType, out var comp)
                   && comp is IComponent component
                   && !component.Deleted;
        }

        internal bool TryGetComponentWithoutLifeCheck(
            EntityUid uid,
            CompIdx idx,
            [NotNullWhen(true)] out IComponent? component)
        {
            if (_world.TryGet(ToArch(uid), idx.Type, out var comp) && comp is IComponent value)
            {
                component = value;
                return !component.Deleted;
            }

            component = null;
            return false;
        }

        /// <inheritdoc />
        public bool TryGetComponent(EntityUid uid, Type type, [NotNullWhen(true)] out IComponent? component)
        {
            return TryGetComponent(uid, (ComponentType) type, out component);
        }

        public bool TryGetComponent<T>(EntityUid uid, CompIdx type, [NotNullWhen(true)] out T? component) where T : IComponent?
        {
            return TryGetComponent(uid, type.Type, out component);
        }

        public bool TryGetComponent(EntityUid uid, CompIdx type, [NotNullWhen(true)] out IComponent? component)
        {
            return TryGetComponent(uid, type.Type, out component);
        }

        /// <inheritdoc />
        public bool TryGetComponent([NotNullWhen(true)] EntityUid? uid, Type type,
            [NotNullWhen(true)] out IComponent? component)
        {
            if (!uid.HasValue)
            {
                component = null;
                return false;
            }

            return TryGetComponent(uid.Value, type, out component);
        }

        /// <inheritdoc />
        public bool TryGetComponent(EntityUid uid, ushort netId, [MaybeNullWhen(false)] out IComponent component, MetaDataComponent? meta = null)
        {
            if (MetaQuery.TryGetComponentInternal(uid, out var metadata)
                && metadata.NetComponents.TryGetValue(netId, out var comp))
            {
                component = comp;
                return true;
            }

            component = default;
            return false;
        }

        /// <inheritdoc />
        public bool TryGetComponent([NotNullWhen(true)] EntityUid? uid, ushort netId,
            [MaybeNullWhen(false)] out IComponent component, MetaDataComponent? meta = null)
        {
            if (!uid.HasValue)
            {
                DebugTools.AssertNull(meta);
                component = default;
                return false;
            }

            return TryGetComponent(uid.Value, netId, out component, meta);
        }

        /// <inheritdoc/>
        public bool TryCopyComponent<T>(EntityUid source, EntityUid target, ref T? sourceComponent, [NotNullWhen(true)] out T? targetComp, MetaDataComponent? meta = null) where T : IComponent
        {
            if (!MetaQuery.Resolve(target, ref meta))
            {
                targetComp = default;
                return false;
            }

            if (sourceComponent == null && !TryGetComponent(source, out sourceComponent))
            {
                targetComp = default;
                return false;
            }

            targetComp = CopyComponentInternal(source, target, sourceComponent, meta);
            return true;
        }

        /// <inheritdoc/>
        public bool TryCopyComponents(
            EntityUid source,
            EntityUid target,
            MetaDataComponent? meta = null,
            params Type[] sourceComponents)
        {
            if (!MetaQuery.TryGetComponent(target, out meta))
                return false;

            var allCopied = true;

            foreach (var type in sourceComponents)
            {
                if (!TryGetComponent(source, type, out var srcComp))
                {
                    allCopied = false;
                    continue;
                }

                CopyComponent(source, target, srcComp, meta: meta);
            }

            return allCopied;
        }

        /// <inheritdoc/>
        public IComponent CopyComponent(EntityUid source, EntityUid target, IComponent sourceComponent, MetaDataComponent? meta = null)
        {
            if (!MetaQuery.Resolve(target, ref meta))
            {
                throw new InvalidOperationException();
            }

            return CopyComponentInternal(source, target, sourceComponent, meta);
        }

        /// <inheritdoc/>
        public T CopyComponent<T>(EntityUid source, EntityUid target, T sourceComponent,MetaDataComponent? meta = null) where T : IComponent
        {
            if (!MetaQuery.Resolve(target, ref meta))
            {
                throw new InvalidOperationException();
            }

            return CopyComponentInternal(source, target, sourceComponent, meta);
        }

        /// <inheritdoc/>
        public void CopyComponents(EntityUid source, EntityUid target, MetaDataComponent? meta = null, params IComponent[] sourceComponents)
        {
            if (!MetaQuery.Resolve(target, ref meta))
                return;

            // TODO: DO bulk changes for arch.
            foreach (var comp in sourceComponents)
            {
                CopyComponentInternal(source, target, comp, meta);
            }
        }

        private T CopyComponentInternal<T>(EntityUid source, EntityUid target, T sourceComponent, MetaDataComponent meta) where T : IComponent
        {
            var compReg = ComponentFactory.GetRegistration(sourceComponent.GetType());
            var component = (T)ComponentFactory.GetComponent(compReg);

            _serManager.CopyTo(sourceComponent, ref component, notNullableOverride: true);
            component.Owner = target;

            AddComponentInternal(target, component, compReg, skipInit: false, overwrite: true, meta);
            return component;
        }

        public EntityQuery<TComp1> GetEntityQuery<TComp1>() where TComp1 : IComponent
        {
            return new EntityQuery<TComp1>(this, ResolveSawmill);
        }

        public EntityQuery<IComponent> GetEntityQuery(Type type)
        {
            return new EntityQuery<IComponent>(this, ResolveSawmill);
        }

        /// <inheritdoc />
        public IEnumerable<IComponent> GetComponents(EntityUid uid)
        {
            foreach (var obj in _world.GetAllComponents(ToArch(uid)))
            {
                var comp = (IComponent)obj!;

                if (comp.Deleted) continue;

                yield return comp;
            }
        }

        /// <inheritdoc />
        public int ComponentCount(EntityUid uid)
        {
            return _world.GetArchetype(ToArch(uid)).Signature.Count;
        }

        /// <summary>
        /// Copy the components for an entity into the given span,
        /// or re-allocate the span as an array if there's not enough space.º
        /// </summary>
        private void CopyComponentsInto(ref Span<IComponent?> comps, EntityUid uid)
        {
            var set = _world.GetAllComponents(ToArch(uid));

            if (set.Length > comps.Length)
            {
                comps = new IComponent?[set.Length];
            }

            var i = 0;
            foreach (var c in set)
            {
                comps[i++] = (IComponent)c!;
            }
        }

        /// <inheritdoc />
        public IEnumerable<T> GetComponents<T>(EntityUid uid)
        {
            var comps = _world.GetAllComponents(ToArch(uid));

            foreach (var comp in comps)
            {
                var component = (IComponent)comp!;
                if (component.Deleted || component is not T tComp) continue;

                yield return tComp;
            }
        }

        /// <inheritdoc />
        public NetComponentEnumerable GetNetComponents(EntityUid uid, MetaDataComponent? meta = null)
        {
            meta ??= MetaQuery.GetComponentInternal(uid);
            return new NetComponentEnumerable(meta.NetComponents);
        }

        /// <inheritdoc />
        public NetComponentEnumerable? GetNetComponentsOrNull(EntityUid uid, MetaDataComponent? meta = null)
        {
            return MetaQuery.Resolve(uid, ref meta)
                ? new NetComponentEnumerable(meta.NetComponents)
                : null;
        }

        #region Join Functions

        public (EntityUid Uid, T Component)[] AllComponents<T>() where T : IComponent
        {
            var query = AllEntityQueryEnumerator<T>();
            var comps = new (EntityUid Uid, T Component)[Count<T>()];
            var i = 0;

            while (query.MoveNext(out var uid, out var comp))
            {
                comps[i] = (uid, comp);
                i++;
            }

            // Count<T> includes "deleted" components that are not returned by MoveNext()
            // This ensures that we dont return an array with empty/invalid entries
            Array.Resize(ref comps, i);
            return comps;
        }

        public Entity<T>[] AllEntities<T>() where T : IComponent
        {
            var query = AllEntityQueryEnumerator<T>();
            var comps = new Entity<T>[Count<T>()];
            var i = 0;

            while (query.MoveNext(out var uid, out var comp))
            {
                comps[i++] = (uid, comp);
            }

            // Count<T> includes "deleted" components that are not returned by MoveNext()
            // This ensures that we dont return an array with empty/invalid entries
            Array.Resize(ref comps, i);
            return comps;
        }

        public Entity<IComponent>[] AllEntities(Type tComp)
        {
            var query = AllEntityQueryEnumerator(tComp);
            var comps = new Entity<IComponent>[Count(tComp)];
            var i = 0;

            while (query.MoveNext(out var uid, out var comp))
            {
                comps[i++] = (uid, comp);
            }

            // Count() includes "deleted" components that are not returned by MoveNext()
            // This ensures that we dont return an array with empty/invalid entries
            Array.Resize(ref comps, i);
            return comps;
        }


        public EntityUid[] AllEntityUids<T>() where T : IComponent
        {
            var query = AllEntityQueryEnumerator<T>();
            var comps = new EntityUid[Count<T>()];
            var i = 0;

            while (query.MoveNext(out var uid, out _))
            {
                comps[i++] = uid;
            }

            // Count<T> includes "deleted" components that are not returned by MoveNext()
            // This ensures that we dont return an array with empty/invalid entries
            Array.Resize(ref comps, i);
            return comps;
        }

        public EntityUid[] AllEntityUids(Type tComp)
        {
            var query = AllEntityQueryEnumerator(tComp);
            var comps = new EntityUid[Count(tComp)];
            var i = 0;

            while (query.MoveNext(out var uid, out _))
            {
                comps[i++] = uid;
            }

            // Count() includes "deleted" components that are not returned by MoveNext()
            // This ensures that we dont return an array with empty/invalid entries
            Array.Resize(ref comps, i);
            return comps;
        }

        public List<(EntityUid Uid, T Component)> AllComponentsList<T>() where T : IComponent
        {
            var query = AllEntityQueryEnumerator<T>();
            var comps = new List<(EntityUid Uid, T Component)>(Count<T>());

            while (query.MoveNext(out var uid, out var comp))
            {
                comps.Add((uid, comp));
            }

            return comps;
        }

        /// <inheritdoc />
        public ComponentQueryEnumerator ComponentQueryEnumerator(ComponentRegistry registry)
        {
            if (registry.Count == 0)
            {
                return new ComponentQueryEnumerator(_world, QueryDescription.Null);
            }

            var query = new QueryDescription(registry.GetTypes());
            return new ComponentQueryEnumerator(_world, query);
        }

        public AllEntityQueryEnumerator<IComponent> AllEntityQueryEnumerator(Type comp)
        {
            DebugTools.Assert(comp.IsAssignableTo(typeof(IComponent)));
            return new AllEntityQueryEnumerator<IComponent>(_world);
        }

        public AllEntityQueryEnumerator<TComp1> AllEntityQueryEnumerator<TComp1>()
        where TComp1 : IComponent
        {
            return new AllEntityQueryEnumerator<TComp1>(_world);
        }

        public AllEntityQueryEnumerator<TComp1, TComp2> AllEntityQueryEnumerator<TComp1, TComp2>()
            where TComp1 : IComponent
            where TComp2 : IComponent
        {
            return new AllEntityQueryEnumerator<TComp1, TComp2>(_world);
        }

        public AllEntityQueryEnumerator<TComp1, TComp2, TComp3> AllEntityQueryEnumerator<TComp1, TComp2, TComp3>()
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
        {
            return new AllEntityQueryEnumerator<TComp1, TComp2, TComp3>(_world);
        }

        public AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>()
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
            where TComp4 : IComponent
        {
            return new AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>(_world);
        }

        public EntityQueryEnumerator<TComp1> EntityQueryEnumerator<TComp1>()
            where TComp1 : IComponent
        {
            return new EntityQueryEnumerator<TComp1>(_world);
        }

        public EntityQueryEnumerator<TComp1, TComp2> EntityQueryEnumerator<TComp1, TComp2>()
            where TComp1 : IComponent
            where TComp2 : IComponent
        {
            return new EntityQueryEnumerator<TComp1, TComp2>(_world);
        }

        public EntityQueryEnumerator<TComp1, TComp2, TComp3> EntityQueryEnumerator<TComp1, TComp2, TComp3>()
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
        {
            return new EntityQueryEnumerator<TComp1, TComp2, TComp3>(_world);
        }

        public EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>()
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
            where TComp4 : IComponent
        {
            return new EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>(_world);
        }

        /// <inheritdoc />
        public IEnumerable<T> EntityQuery<T>(bool includePaused = false) where T : IComponent
        {
            if (includePaused)
            {
                var query = new AllEntityQueryEnumerator<T>(_world);
                while (query.MoveNext(out var comp))
                {
                    yield return comp;
                }
            }
            else
            {
                var query = new EntityQueryEnumerator<T>(_world);
                while (query.MoveNext(out var comp))
                {
                    yield return comp;
                }
            }
        }

        /// <inheritdoc />
        public IEnumerable<(TComp1, TComp2)> EntityQuery<TComp1, TComp2>(bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent
        {
            if (includePaused)
            {
                var query = new AllEntityQueryEnumerator<TComp1, TComp2>(_world);
                while (query.MoveNext(out var comp1, out var comp2))
                {
                    yield return (comp1, comp2);
                }
            }
            else
            {
                var query = new EntityQueryEnumerator<TComp1, TComp2>(_world);
                while (query.MoveNext(out var comp1, out var comp2))
                {
                    yield return (comp1, comp2);
                }
            }
        }

        /// <inheritdoc />
        public IEnumerable<(TComp1, TComp2, TComp3)> EntityQuery<TComp1, TComp2, TComp3>(bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
        {
            if (includePaused)
            {
                var query = new AllEntityQueryEnumerator<TComp1, TComp2, TComp3>(_world);
                while (query.MoveNext(out var comp1, out var comp2, out var comp3))
                {
                    yield return (comp1, comp2, comp3);
                }
            }
            else
            {
                var query = new EntityQueryEnumerator<TComp1, TComp2, TComp3>(_world);
                while (query.MoveNext(out var comp1, out var comp2, out var comp3))
                {
                    yield return (comp1, comp2, comp3);
                }
            }
        }

        /// <inheritdoc />
        public IEnumerable<(TComp1, TComp2, TComp3, TComp4)> EntityQuery<TComp1, TComp2, TComp3, TComp4>(
            bool includePaused = false)
            where TComp1 : IComponent
            where TComp2 : IComponent
            where TComp3 : IComponent
            where TComp4 : IComponent
        {
            if (includePaused)
            {
                var query = new AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>(_world);
                while (query.MoveNext(out var comp1, out var comp2, out var comp3, out var comp4))
                {
                    yield return (comp1, comp2, comp3, comp4);
                }
            }
            else
            {
                var query = new EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>(_world);
                while (query.MoveNext(out var comp1, out var comp2, out var comp3, out var comp4))
                {
                    yield return (comp1, comp2, comp3, comp4);
                }
            }
        }

        #endregion

        /// <inheritdoc />
        public IEnumerable<(EntityUid Uid, IComponent Component)> GetAllComponents(Type type, bool includePaused = false)
        {
            QueryDescription query;

            // TODO arch paused component
            if (includePaused)
            {
                // TODO arch pool
                query = new (new ComponentType[] { type });
            }
            else
            {
                query = new(new ComponentType[] { type, typeof(MetaDataComponent) });
            }

            foreach (var chunk in _world.ChunkIterator(query))
            {
                var components = chunk.GetArray(type);
                var metas = includePaused ? default : chunk.GetArray<MetaDataComponent>();

                for (var i = 0; i < chunk.Count; i++)
                {
                    var comp = (IComponent)components.GetValue(i)!;
                    if (comp.Deleted)
                        continue;

                    if (!includePaused && metas![i].EntityPaused)
                        continue;

                    yield return (chunk.Entity(i), comp);
                }
            }
        }

        /// <inheritdoc />
        [Pure]
        public IComponentState? GetComponentState(IEventBus eventBus, IComponent component, ICommonSession? session, GameTick fromTick)
        {
            DebugTools.Assert(component.NetSyncEnabled, $"Attempting to get component state for an un-synced component: {component.GetType()}");
            var getState = new ComponentGetState(session, fromTick);
            eventBus.RaiseComponentEvent(component.Owner, component, ref getState);

            return getState.State;
        }

        public bool CanGetComponentState(IEventBus eventBus, IComponent component, ICommonSession player)
            => CanGetComponentState(component, player);

        public bool CanGetComponentState(IComponent component, ICommonSession player)
        {
            var attempt = new ComponentGetStateAttemptEvent(player);
            EventBusInternal.RaiseComponentEvent(component.Owner, component, ref attempt);
            return !attempt.Cancelled;
        }

        #endregion

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void FillComponentDict()
        {
            RegisterComponents(_componentFactory.GetAllRegistrations());
        }
    }

    public readonly struct NetComponentEnumerable
    {
        private readonly Dictionary<ushort, IComponent> _dictionary;

        public NetComponentEnumerable(Dictionary<ushort, IComponent> dictionary) => _dictionary = dictionary;
        public NetComponentEnumerator GetEnumerator() => new(_dictionary);
    }

    public struct NetComponentEnumerator
    {
        // DO NOT MAKE THIS READONLY
        private Dictionary<ushort, IComponent>.Enumerator _dictEnum;

        public NetComponentEnumerator(Dictionary<ushort, IComponent> dictionary) =>
            _dictEnum = dictionary.GetEnumerator();

        public bool MoveNext() => _dictEnum.MoveNext();

        public (ushort netId, IComponent component) Current
        {
            get
            {
                var val = _dictEnum.Current;
                return (val.Key, val.Value);
            }
        }
    }

    /// <summary>
    ///     An index of all entities with a given component, avoiding looking up the component's storage every time.
    ///     Using these saves on dictionary lookups, making your code slightly more efficient, and ties in nicely with
    ///     <see cref="Entity{T}"/>.
    /// </summary>
    /// <typeparam name="TComp1">Any component type.</typeparam>
    /// <example>
    ///     <code>
    ///         public sealed class MySystem : EntitySystem
    ///         {
    ///             private EntityQuery&lt;TransformComponent&gt; _transforms = default!;
    ///             <br/>
    ///             public override void Initialize()
    ///             {
    ///                 _transforms = GetEntityQuery&lt;TransformComponent&gt;();
    ///             }
    ///             <br/>
    ///             public void DoThings(EntityUid myEnt)
    ///             {
    ///                 var ent = _transforms.Get(myEnt);
    ///                 // ...
    ///             }
    ///         }
    ///     </code>
    /// </example>
    /// <remarks>
    ///     Queries hold references to <see cref="IEntityManager"/> internals, and are always up to date with the world.
    ///     They can not however perform mutation, if you need to add or remove components you must use
    ///     <see cref="EntitySystem"/> or <see cref="IEntityManager"/> methods.
    /// </remarks>
    /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.GetEntityQuery``1">EntitySystem.GetEntityQuery()</seealso>
    /// <seealso cref="M:Robust.Shared.GameObjects.EntityManager.GetEntityQuery``1">EntityManager.GetEntityQuery()</seealso>
    public readonly struct EntityQuery<TComp1> where TComp1 : IComponent
    {
        private readonly EntityManager _entManager;
        private readonly ComponentType _type;
        private readonly ISawmill _sawmill;

        internal EntityQuery(EntityManager entManager, ISawmill sawmill)
        {
            _entManager = entManager;
            _type = Component<TComp1>.ComponentType;
            _sawmill = sawmill;
        }

        /// <summary>
        ///     Gets <typeparamref name="TComp1"/> for an entity, throwing if it can't find it.
        /// </summary>
        /// <param name="uid">The entity to do a lookup for.</param>
        /// <returns>The located component.</returns>
        /// <exception cref="KeyNotFoundException">Thrown if the entity does not have a component of type <typeparamref name="TComp1"/>.</exception>
        /// <seealso cref="M:Robust.Shared.GameObjects.IEntityManager.GetComponent``1(Robust.Shared.GameObjects.EntityUid)">
        ///     IEntityManager.GetComponent&lt;T&gt;(EntityUid)
        /// </seealso>
        /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.Comp``1(Robust.Shared.GameObjects.EntityUid)">
        ///     EntitySystem.Comp&lt;T&gt;(EntityUid)
        /// </seealso>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public TComp1 GetComponent(EntityUid uid)
        {
            if (_entManager.TryGetComponent(uid, _type, out var comp) && !comp.Deleted)
                return (TComp1) comp;

            throw new KeyNotFoundException($"Entity {uid} does not have a component of type {typeof(TComp1)}");
        }

        /// <inheritdoc cref="GetComponent"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining), Pure]
        public Entity<TComp1> Get(EntityUid uid)
        {
            if (_entManager.TryGetComponent(uid, _type, out var comp) && !comp.Deleted)
                return new Entity<TComp1>(uid, (TComp1) comp);

            throw new KeyNotFoundException($"Entity {uid} does not have a component of type {typeof(TComp1)}");
        }

        /// <summary>
        ///     Gets <typeparamref name="TComp1"/> for an entity, if it's present.
        /// </summary>
        /// <remarks>
        ///     If it is strictly errorenous for a component to not be present, you may want to use
        ///     <see cref="Resolve(Robust.Shared.GameObjects.EntityUid,ref TComp1?,bool)"/> instead.
        /// </remarks>
        /// <param name="uid">The entity to do a lookup for.</param>
        /// <param name="component">The located component, if any.</param>
        /// <returns>Whether the component was found.</returns>
        /// <seealso cref="M:Robust.Shared.GameObjects.IEntityManager.TryGetComponent``1(Robust.Shared.GameObjects.EntityUid,``0@)">
        ///     IEntityManager.TryGetComponent&lt;T&gt;(EntityUid, out T?)
        /// </seealso>
        /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.TryComp``1(Robust.Shared.GameObjects.EntityUid,``0@)">
        ///     EntitySystem.TryComp&lt;T&gt;(EntityUid, out T?)
        /// </seealso>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool TryGetComponent([NotNullWhen(true)] EntityUid? uid, [NotNullWhen(true)] out TComp1? component)
        {
            if (uid == null)
            {
                component = default;
                return false;
            }

            return TryGetComponent(uid.Value, out component);
        }

        /// <inheritdoc cref="TryGetComponent(Robust.Shared.GameObjects.EntityUid?,out TComp1?)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool TryGetComponent(EntityUid uid, [NotNullWhen(true)] out TComp1? component)
        {
            if (_entManager.TryGetComponent(uid, _type, out var comp) && !comp.Deleted)
            {
                component = (TComp1) comp;
                return true;
            }

            component = default;
            return false;
        }

        /// <inheritdoc cref="TryGetComponent(Robust.Shared.GameObjects.EntityUid?,out TComp1?)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool TryComp(EntityUid uid, [NotNullWhen(true)] out TComp1? component)
            => TryGetComponent(uid, out component);

        /// <inheritdoc cref="TryGetComponent(Robust.Shared.GameObjects.EntityUid?,out TComp1?)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool TryComp([NotNullWhen(true)] EntityUid? uid, [NotNullWhen(true)] out TComp1? component)
            => TryGetComponent(uid, out component);

        /// <summary>
        ///     Tests if the given entity has <typeparamref name="TComp1"/>.
        /// </summary>
        /// <param name="uid">The entity to do a lookup for.</param>
        /// <returns>Whether the component exists for that entity.</returns>
        /// <remarks>If you immediately need to then look up that component, it's more efficient to use <see cref="TryComp(Robust.Shared.GameObjects.EntityUid,out TComp1?)"/>.</remarks>
        /// <seealso cref="M:Robust.Shared.GameObjects.IEntityManager.HasComponent``1(Robust.Shared.GameObjects.EntityUid)">
        ///     IEntityManager.HasComponent&lt;T&gt;(EntityUid)
        /// </seealso>
        /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.HasComp``1(Robust.Shared.GameObjects.EntityUid)">
        ///     EntitySystem.HasComp&lt;T&gt;(EntityUid)
        /// </seealso>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComp(EntityUid uid) => HasComponent(uid);

        /// <inheritdoc cref="HasComp(Robust.Shared.GameObjects.EntityUid)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComp([NotNullWhen(true)] EntityUid? uid) => HasComponent(uid);

        /// <inheritdoc cref="HasComp(Robust.Shared.GameObjects.EntityUid)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent(EntityUid uid)
        {
            return _entManager.TryGetComponent(uid, _type, out var comp) && !comp.Deleted;
        }

        /// <inheritdoc cref="HasComp(Robust.Shared.GameObjects.EntityUid)"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public bool HasComponent([NotNullWhen(true)] EntityUid? uid)
        {
            return uid != null && HasComponent(uid.Value);
        }

        /// <include file='Docs.xml' path='entries/entry[@name="EntityQueryResolve"]/*'/>
        /// <param name="uid">The entity to do a lookup for.</param>
        /// <param name="component">The space to write the component into if found.</param>
        /// <param name="logMissing">Whether to log if the component is missing, for diagnostics.</param>
        /// <returns>Whether the component was found.</returns>
        /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.Resolve``1(Robust.Shared.GameObjects.EntityUid,``0@,System.Boolean)">
        ///     EntitySystem.Resolve&lt;T&gt;(EntityUid, out T?)
        /// </seealso>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Resolve(EntityUid uid, [NotNullWhen(true)] ref TComp1? component, bool logMissing = true)
        {
            if (component != null)
            {
                DebugTools.AssertOwner(uid, component);
                return true;
            }

            if (_entManager.TryGetComponent(uid, _type, out var comp) && !comp.Deleted)
            {
                component = (TComp1)comp;
                return true;
            }

            if (logMissing)
                _sawmill.Error($"Can't resolve \"{typeof(TComp1)}\" on entity {_entManager.ToPrettyString(uid)}!\n{Environment.StackTrace}");

            return false;
        }

        /// <include file='Docs.xml' path='entries/entry[@name="EntityQueryResolve"]/*'/>
        /// <param name="entity">The space to write the component into if found.</param>
        /// <param name="logMissing">Whether to log if the component is missing, for diagnostics.</param>
        /// <returns>Whether the component was found.</returns>
        /// <seealso cref="M:Robust.Shared.GameObjects.EntitySystem.Resolve``1(Robust.Shared.GameObjects.EntityUid,``0@,System.Boolean)">
        ///     EntitySystem.Resolve&lt;T&gt;(EntityUid, out T?)
        /// </seealso>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Resolve(ref Entity<TComp1?> entity, bool logMissing = true)
        {
            return Resolve(entity.Owner, ref entity.Comp, logMissing);
        }

        /// <summary>
        ///     Gets <typeparamref name="TComp1"/> for an entity if it's present, or null if it's not.
        /// </summary>
        /// <param name="uid">The entity to do the lookup on.</param>
        /// <returns>The component, if it exists.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public TComp1? CompOrNull(EntityUid uid)
        {
            if (TryGetComponent(uid, out var comp))
                return comp;

            return default;
        }

        /// <inheritdoc cref="GetComponent"/>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        public TComp1 Comp(EntityUid uid)
        {
            return GetComponent(uid);
        }

        #region Internal

        /// <summary>
        /// Elides the component.Deleted check of <see cref="GetComponent"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal TComp1 GetComponentInternal(EntityUid uid)
        {
            if (_entManager.TryGetComponent(uid, _type, out var comp))
                return (TComp1) comp;

            throw new KeyNotFoundException($"Entity {uid} does not have a component of type {typeof(TComp1)}");
        }

        /// <summary>
        /// Elides the component.Deleted check of <see cref="TryGetComponent(System.Nullable{Robust.Shared.GameObjects.EntityUid},out TComp1?)"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal bool TryGetComponentInternal([NotNullWhen(true)] EntityUid? uid, [NotNullWhen(true)] out TComp1? component)
        {
            if (uid == null)
            {
                component = default;
                return false;
            }

            return TryGetComponentInternal(uid.Value, out component);
        }

        /// <summary>
        /// Elides the component.Deleted check of <see cref="TryGetComponent(System.Nullable{Robust.Shared.GameObjects.EntityUid},out TComp1?)"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal bool TryGetComponentInternal(EntityUid uid, [NotNullWhen(true)] out TComp1? component)
        {
            if (_entManager.TryGetComponent(uid, _type, out var comp))
            {
                component = (TComp1) comp;
                return true;
            }

            component = default;
            return false;
        }

        /// <summary>
        /// Elides the component.Deleted check of <see cref="HasComponent(EntityUid)"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal bool HasComponentInternal(EntityUid uid)
        {
            return uid.Valid && _entManager.HasComponent(uid, _type);
        }

        /// <summary>
        /// Elides the component.Deleted check of <see cref="Resolve"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal bool ResolveInternal(EntityUid uid, [NotNullWhen(true)] ref TComp1? component, bool logMissing = true)
        {
            if (component != null)
            {
                DebugTools.AssertOwner(uid, component);
                return true;
            }

            if (_entManager.TryGetComponent(uid, _type, out var comp))
            {
                component = (TComp1)comp;
                return true;
            }

            if (logMissing)
                _sawmill.Error($"Can't resolve \"{typeof(TComp1)}\" on entity {_entManager.ToPrettyString(uid)}!\n{new StackTrace(1, true)}");

            return false;
        }
        /// <summary>
        /// Elides the component.Deleted check of <see cref="CompOrNull"/>
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        [Pure]
        internal TComp1? CompOrNullInternal(EntityUid uid)
        {
            if (TryGetComponent(uid, out var comp))
                return comp;

            return default;
        }

        #endregion
    }

    internal static class EntityQueryDescription<TComp1>
        where TComp1 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, MetaDataComponent>();
    }

    internal static class EntityQueryDescription<TComp1, TComp2>
        where TComp1 : IComponent
        where TComp2 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2, MetaDataComponent>();
    }

    internal static class EntityQueryDescription<TComp1, TComp2, TComp3>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2, TComp3, MetaDataComponent>();
    }

    internal static class EntityQueryDescription<TComp1, TComp2, TComp3, TComp4>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
        where TComp4 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2, TComp3, TComp4, MetaDataComponent>();
    }

    internal static class AllEntityQueryDescription<TComp1>
        where TComp1 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1>();
    }

    internal static class AllEntityQueryDescription<TComp1, TComp2>
        where TComp1 : IComponent
        where TComp2 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2>();
    }

    internal static class AllEntityQueryDescription<TComp1, TComp2, TComp3>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2, TComp3>();
    }

    internal static class AllEntityQueryDescription<TComp1, TComp2, TComp3, TComp4>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
        where TComp4 : IComponent
    {
        public static readonly QueryDescription Value = new QueryDescription().WithAll<TComp1, TComp2, TComp3, TComp4>();
    }

    #region ComponentRegistry Query

    /// <summary>
    /// Non-generic version of <see cref="AllEntityQueryEnumerator{TComp1}"/>
    /// </summary>
    public struct ComponentQueryEnumerator
    {
        private QueryDescription _desc;
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;

        public ComponentQueryEnumerator(
            World world,
            QueryDescription desc)
        {
            _desc = desc;
            _chunkEnumerator = world.ChunkIterator(desc).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
            }
        }

        public bool MoveNext(out EntityUid uid)
        {
            while (true)
            {
                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        uid = EntityUid.Invalid;
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                }

                // Deletion check moment
                foreach (var comp in _desc.All)
                {
                    if (((IComponent)_chunkEnumerator.Current.Get(_index, comp)!).Deleted)
                        return MoveNext(out uid);
                }

                uid = _chunkEnumerator.Current.Entity(_index);
                return true;
            }
        }
    }
    #endregion

    #region Query

    /// <summary>
    ///     Iterates all entities that have the given components, including the components themselves, but only if
    ///     the entity they're on is not <see cref="EntitySystem.Paused">Paused</see>.
    /// </summary>
    public struct EntityQueryEnumerator<TComp1>
        where TComp1 : IComponent
    {
        private readonly World _world;
        private readonly Query _query;

        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private MetaDataComponent[] _metaArray = default!;

        public EntityQueryEnumerator(World world)
        {
            _world = world;
            _query = world.Query(EntityQueryDescription<TComp1>.Value);
            Reset();
        }

        public void Reset()
        {
            _chunkEnumerator = _world.ChunkIterator(EntityQueryDescription<TComp1>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _metaArray);
            }
        }

        /// <summary>
        ///     Provides the next entity and component in the enumerator, if there are still more to iterate through.
        /// </summary>
        /// <param name="uid">The found entity, if any.</param>
        /// <param name="comp1">A component on the found entity.</param>
        /// <returns>Whether the enumerator was empty (and as such no entity nor component were returned)</returns>
        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)] out TComp1? comp1)
        {
            if (MoveNext(out comp1))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext([NotNullWhen(true)] out TComp1? comp1)
        {
            while (true)
            {
                comp1 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _metaArray);
                }

                comp1 = _comp1Array[_index];
                var meta = _metaArray[_index];

                if (comp1.Deleted || meta.EntityPaused) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(EntityQueryEnumerator<TComp1> enumerator)
        {
            private EntityQueryEnumerator<TComp1> _enumerator = enumerator;

            public Entity<TComp1> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp))
                    return false;

                Current = (id, comp);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching unpaused components.
    /// </summary>
    public struct EntityQueryEnumerator<TComp1, TComp2>
        where TComp1 : IComponent
        where TComp2 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;
        private MetaDataComponent[] _metaArray = default!;

        public EntityQueryEnumerator(World world)
        {
            Unsafe.SkipInit(out this);
            _chunkEnumerator = world.ChunkIterator(EntityQueryDescription<TComp1, TComp2>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _metaArray);
            }
        }

        /// <inheritdoc cref="M:Robust.Shared.GameObjects.EntityQueryEnumerator`1.MoveNext(Robust.Shared.GameObjects.EntityUid@,`0@)"/>
        /// <param name="comp2">A component on the found entity.</param>
        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)] out TComp1? comp1, [NotNullWhen(true)] out TComp2? comp2)
        {
            if (MoveNext(out comp1, out comp2))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext([NotNullWhen(true)] out TComp1? comp1, [NotNullWhen(true)] out TComp2? comp2)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _metaArray);
                }

                comp1 = _comp1Array[_index];
                comp2 = _comp2Array[_index];
                var meta = _metaArray[_index];

                if (comp1.Deleted || comp2.Deleted || meta.EntityPaused) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(EntityQueryEnumerator<TComp1, TComp2> enumerator)
        {
            private EntityQueryEnumerator<TComp1, TComp2> _enumerator = enumerator;

            public Entity<TComp1, TComp2> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2))
                    return false;

                Current = (id, comp1, comp2);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching unpaused components.
    /// </summary>
    public struct EntityQueryEnumerator<TComp1, TComp2, TComp3>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;
        private TComp3[] _comp3Array = default!;
        private MetaDataComponent[] _metaArray = default!;

        public EntityQueryEnumerator(World world)
        {
            _chunkEnumerator = world.ChunkIterator(EntityQueryDescription<TComp1, TComp2, TComp3>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _metaArray);
            }
        }

        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3)
        {
            if (MoveNext(out comp1, out comp2, out comp3))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                DebugTools.AssertOwner(uid, comp3);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext(
            [NotNullWhen(true)] out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;
                comp3 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _metaArray);
                }

                comp1 = _comp1Array[_index];
                comp2 = _comp2Array[_index];
                comp3 = _comp3Array[_index];
                var meta = _metaArray[_index];

                if (comp1.Deleted || comp2.Deleted || comp3.Deleted || meta.EntityPaused) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(EntityQueryEnumerator<TComp1, TComp2, TComp3> enumerator)
        {
            private EntityQueryEnumerator<TComp1, TComp2, TComp3> _enumerator = enumerator;

            public Entity<TComp1, TComp2, TComp3> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2, out var comp3))
                    return false;

                Current = (id, comp1, comp2, comp3);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching unpaused components.
    /// </summary>
    public struct EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
        where TComp4 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;
        private TComp3[] _comp3Array = default!;
        private TComp4[] _comp4Array = default!;
        private MetaDataComponent[] _metaArray = default!;

        public EntityQueryEnumerator(World world)
        {
            _chunkEnumerator = world.ChunkIterator(EntityQueryDescription<TComp1, TComp2, TComp3, TComp4>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _comp4Array, out _metaArray);
            }
        }

        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3,
            [NotNullWhen(true)] out TComp4? comp4)
        {
            if (MoveNext(out comp1, out comp2, out comp3, out comp4))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                DebugTools.AssertOwner(uid, comp3);
                DebugTools.AssertOwner(uid, comp4);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext(
            [NotNullWhen(true)] out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3,
            [NotNullWhen(true)] out TComp4? comp4)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;
                comp3 = default;
                comp4 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _comp4Array, out _metaArray);
                }

                comp1 = _comp1Array[_index];
                comp2 = _comp2Array[_index];
                comp3 = _comp3Array[_index];
                comp4 = _comp4Array[_index];
                var meta = _metaArray[_index];

                if (comp1.Deleted || comp2.Deleted || comp3.Deleted || comp4.Deleted || meta.EntityPaused) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> enumerator)
        {
            private EntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> _enumerator = enumerator;

            public Entity<TComp1, TComp2, TComp3, TComp4> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2, out var comp3, out var comp4))
                    return false;

                Current = (id, comp1, comp2, comp3, comp4);
                return true;
            }
        }
    }

    #endregion

    #region All query

    /// <summary>
    ///     Iterates all entities that have the given components, including the components themselves, regardless
    ///     of if the entity is <see cref="EntitySystem.Paused">Paused</see>.
    /// </summary>
    public struct AllEntityQueryEnumerator<TComp1>
        where TComp1 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array;

        internal AllEntityQueryEnumerator(World world)
        {
            Unsafe.SkipInit(out this);
            _chunkEnumerator = world.ChunkIterator(AllEntityQueryDescription<TComp1>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _comp1Array = _chunkEnumerator.Current.GetArray<TComp1>();
            }
            else
            {
                _index = 0;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1)
        {
            if (MoveNext(out comp1))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext([NotNullWhen(true)] out TComp1? comp1)
        {
            while (true)
            {
                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        comp1 = default;
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _comp1Array = _chunkEnumerator.Current.GetArray<TComp1>();
                }

                comp1 = _comp1Array[_index];

                if (comp1.Deleted) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(AllEntityQueryEnumerator<TComp1> enumerator)
        {
            private AllEntityQueryEnumerator<TComp1> _enumerator = enumerator;

            public Entity<TComp1> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp))
                    return false;

                Current = (id, comp);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching components, paused or not.
    /// </summary>
    public struct AllEntityQueryEnumerator<TComp1, TComp2>
        where TComp1 : IComponent
        where TComp2 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;

        public AllEntityQueryEnumerator(World world)
        {
            Unsafe.SkipInit(out this);
            _chunkEnumerator = world.ChunkIterator(AllEntityQueryDescription<TComp1, TComp2>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array);
            }
            else
            {
                _index = 0;
            }
        }

        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2)
        {
            if (MoveNext(out comp1, out comp2))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext([NotNullWhen(true)] out TComp1? comp1, [NotNullWhen(true)] out TComp2? comp2)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array);
                }

                comp1 = _comp1Array[_index];

                if (comp1.Deleted) continue;

                comp2 = _comp2Array[_index];

                if (comp2.Deleted) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(AllEntityQueryEnumerator<TComp1, TComp2> enumerator)
        {
            private AllEntityQueryEnumerator<TComp1, TComp2> _enumerator = enumerator;

            public Entity<TComp1, TComp2> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2))
                    return false;

                Current = (id, comp1, comp2);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching components, paused or not.
    /// </summary>
    public struct AllEntityQueryEnumerator<TComp1, TComp2, TComp3>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;
        private TComp3[] _comp3Array = default!;

        public AllEntityQueryEnumerator(World world)
        {
            _chunkEnumerator = world.ChunkIterator(AllEntityQueryDescription<TComp1, TComp2, TComp3>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array);
            }
        }

        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3)
        {
            if (MoveNext(out comp1, out comp2, out comp3))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                DebugTools.AssertOwner(uid, comp3);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext(
            [NotNullWhen(true)] out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;
                comp3 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array);
                }

                comp1 = _comp1Array[_index];
                comp2 = _comp2Array[_index];
                comp3 = _comp3Array[_index];

                if (comp1.Deleted || comp2.Deleted || comp3.Deleted) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(AllEntityQueryEnumerator<TComp1, TComp2, TComp3> enumerator)
        {
            private AllEntityQueryEnumerator<TComp1, TComp2, TComp3> _enumerator = enumerator;

            public Entity<TComp1, TComp2, TComp3> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2, out var comp3))
                    return false;

                Current = (id, comp1, comp2, comp3);
                return true;
            }
        }
    }

    /// <summary>
    /// Returns all matching components, paused or not.
    /// </summary>
    public struct AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4>
        where TComp1 : IComponent
        where TComp2 : IComponent
        where TComp3 : IComponent
        where TComp4 : IComponent
    {
        private ArchChunkEnumerator _chunkEnumerator;
        private int _index;
        private TComp1[] _comp1Array = default!;
        private TComp2[] _comp2Array = default!;
        private TComp3[] _comp3Array = default!;
        private TComp4[] _comp4Array = default!;

        public AllEntityQueryEnumerator(World world)
        {
            _chunkEnumerator = world.ChunkIterator(AllEntityQueryDescription<TComp1, TComp2, TComp3, TComp4>.Value).GetEnumerator();
            if (_chunkEnumerator.MoveNext())
            {
                _index = _chunkEnumerator.Current.Count;
                _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _comp4Array);
            }
        }

        public bool MoveNext(out EntityUid uid, [NotNullWhen(true)]
            out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3,
            [NotNullWhen(true)] out TComp4? comp4)
        {
            if (MoveNext(out comp1, out comp2, out comp3, out comp4))
            {
                uid = _chunkEnumerator.Current.Entity(_index);
                DebugTools.AssertOwner(uid, comp1);
                DebugTools.AssertOwner(uid, comp2);
                DebugTools.AssertOwner(uid, comp3);
                DebugTools.AssertOwner(uid, comp4);
                return true;
            }

            uid = EntityUid.Invalid;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool MoveNext(
            [NotNullWhen(true)] out TComp1? comp1,
            [NotNullWhen(true)] out TComp2? comp2,
            [NotNullWhen(true)] out TComp3? comp3,
            [NotNullWhen(true)] out TComp4? comp4)
        {
            while (true)
            {
                comp1 = default;
                comp2 = default;
                comp3 = default;
                comp4 = default;

                if (--_index < 0)
                {
                    if (!_chunkEnumerator.MoveNext())
                    {
                        return false;
                    }

                    _index = _chunkEnumerator.Current.Count - 1;
                    _chunkEnumerator.Current.GetArray(out _comp1Array, out _comp2Array, out _comp3Array, out _comp4Array);
                }

                comp1 = _comp1Array[_index];
                comp2 = _comp2Array[_index];
                comp3 = _comp3Array[_index];
                comp4 = _comp4Array[_index];

                if (comp1.Deleted || comp2.Deleted || comp3.Deleted || comp4.Deleted) continue;

                return true;
            }
        }

        public Enumerator GetEnumerator()
        {
            return new Enumerator(this);
        }

        public struct Enumerator(AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> enumerator)
        {
            private AllEntityQueryEnumerator<TComp1, TComp2, TComp3, TComp4> _enumerator = enumerator;

            public Entity<TComp1, TComp2, TComp3, TComp4> Current { get; private set; }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public bool MoveNext()
            {
                if (!_enumerator.MoveNext(out var id, out var comp1, out var comp2, out var comp3, out var comp4))
                    return false;

                Current = (id, comp1, comp2, comp3, comp4);
                return true;
            }
        }
    }

    #endregion
}
