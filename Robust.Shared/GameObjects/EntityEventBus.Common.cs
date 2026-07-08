using System;
using System.Collections.Frozen;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using JetBrains.Annotations;
using Robust.Shared.Collections;
using Robust.Shared.Reflection;
using Robust.Shared.Utility;

namespace Robust.Shared.GameObjects;

internal sealed partial class EntityEventBus : IEventBus
{
    private EntityManager _entMan;
    private IComponentFactory _comFac;
    private IReflectionManager _reflection;

    // Data on individual events. Used to check ordering info and fire broadcast events.
    private FrozenDictionary<Type, EventData> _eventData = FrozenDictionary<Type, EventData>.Empty;
    private readonly Dictionary<Type, EventData> _eventDataUnfrozen = new();

    // Inverse subscriptions to be able to unsubscribe an IEntityEventSubscriber.
    private readonly Dictionary<IEntityEventSubscriber, Dictionary<Type, BroadcastRegistration>> _inverseEventSubscriptions
        = new();

    // For queued message broadcast.
    private readonly Queue<(EventSource source, object args)> _eventQueue = new();

    // eUid.Id -> EventType -> { CompType1, ... CompTypeN }
    // See EventTable declaration for layout details
    internal EntityEventTableStorage _entEventTables = new();

    /// <summary>
    /// Array of component events and their handlers. The array is indexed by a component's
    /// <see cref="CompIdx.Value"/>, while the dictionary is indexed by the event type. This does not include events
    /// with the <see cref="ComponentEventAttribute"/>, unless <see cref="ComponentEventAttribute.Exclusive"/> is false.
    /// </summary>
    private FrozenDictionary<Type, DirectedRegistration>[] _eventSubs = default!;

    /// <summary>
    /// Variant of <see cref="_eventSubs"/> that only includes events with the <see cref="ComponentEventAttribute"/>
    /// </summary>
    private FrozenDictionary<Type, GeneratedDirectedEventHandler>[] _compEventSubs = default!;

    // pre-freeze event subscription data
    private Dictionary<Type, DirectedRegistration>?[] _eventSubsUnfrozen = [];
    private Dictionary<Type, GeneratedDirectedEventHandler>?[] _compEventSubsUnfrozen = [];

    /// <summary>
    /// Inverse of <see cref="_eventSubs"/>, mapping event types to sets of components.
    /// </summary>
    private Dictionary<Type, HashSet<CompIdx>> _eventSubsInv = new();
    // Only required to sort ordered subscriptions, which only happens during initialization
    // so doesn't need to be a frozen dictionary.

    // prevents shitcode, get your subscriptions figured out before you start spawning entities
    private bool _subscriptionLock;

    private int _subscriptionVersion;
    private int _directedEventCount;
#if DEBUG
    private int _directedDispatchDepth;
#endif
    internal Type[] _directedEventTypes = [];

    public bool IgnoreUnregisteredComponents;

    private readonly List<Type> _childrenTypesTemp = [];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void EnterDirectedDispatch()
    {
#if DEBUG
        _directedDispatchDepth++;
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ExitDirectedDispatch()
    {
#if DEBUG
        _directedDispatchDepth--;
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AssertNotMutatingDuringDirectedDispatch()
    {
#if DEBUG
        DebugTools.Assert(_directedDispatchDepth == 0, "Cannot mutate directed event tables during directed event dispatch.");
#endif
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ref EntityEventBusUnit ExtractUnitRef(ref object obj, Type objType)
    {
        // If it's a boxed value type we have to do some trickery to return the INTERIOR reference,
        // not the reference to the boxed object.
        // Otherwise the EntityEventBusUnit points to the reference to the reference type.
        return ref objType.IsValueType
            ? ref Unsafe.As<object, UnitBox>(ref obj).Value
            : ref Unsafe.As<object, EntityEventBusUnit>(ref obj);
    }

    private void RegisterCommon(Type eventType, OrderingData? data, out EventData subs)
    {
        if (_subscriptionLock)
            throw new InvalidOperationException("Subscription locked.");

        subs = _eventDataUnfrozen.GetOrNew(eventType);

        if (data == null)
            return;

        if (data.Before.Length > 0 || data.After.Length > 0)
        {
            subs.IsOrdered = true;
            subs.OrderingUpToDate = false;
        }
    }

    /// <summary>
    /// Information for a single event type handled by EventBus. Not specific to broadcast registrations.
    /// </summary>
    private sealed class EventData
    {
        public int DirectedEventId = -1;
        public bool IsOrdered;
        public bool OrderingUpToDate;
        public ValueList<BroadcastRegistration> BroadcastRegistrations;
    }

    private static class EventCache<TEvent>
        where TEvent : notnull
    {
        public static EntityEventBus? Bus;
        public static int Version;
        public static bool HasEventData;
        public static EventData? EventData;
        public static GeneratedDirectedEventHandler?[]? ComponentHandlers;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static EventData? GetEventData(EntityEventBus bus)
        {
            if (ReferenceEquals(Bus, bus) && Version == bus._subscriptionVersion)
                return HasEventData ? EventData : null;

            return UpdateEventData(bus);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static EventData? UpdateEventData(EntityEventBus bus)
        {
            Bus = bus;
            Version = bus._subscriptionVersion;
            HasEventData = bus._eventData.TryGetValue(typeof(TEvent), out var data);
            EventData = data;
            ComponentHandlers = null;
            return HasEventData ? data : null;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static GeneratedDirectedEventHandler?[] GetComponentHandlers(EntityEventBus bus)
        {
            if (ReferenceEquals(Bus, bus)
                && Version == bus._subscriptionVersion
                && ComponentHandlers is { } handlers)
                return handlers;

            return UpdateComponentHandlers(bus);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private static GeneratedDirectedEventHandler?[] UpdateComponentHandlers(EntityEventBus bus)
        {
            if (!ReferenceEquals(Bus, bus) || Version != bus._subscriptionVersion)
            {
                Bus = bus;
                Version = bus._subscriptionVersion;
                HasEventData = bus._eventData.TryGetValue(typeof(TEvent), out var data);
                EventData = data;
            }

            var handlers = GC.AllocateUninitializedArray<GeneratedDirectedEventHandler?>(bus._compEventSubs.Length);
            for (var i = 0; i < handlers.Length; i++)
            {
                handlers[i] = bus._compEventSubs[i]?.GetValueOrDefault(typeof(TEvent));
            }

            ComponentHandlers = handlers;
            return handlers;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    private sealed class UnitBox
    {
        [UsedImplicitly] public EntityEventBusUnit Value;
    }
}
