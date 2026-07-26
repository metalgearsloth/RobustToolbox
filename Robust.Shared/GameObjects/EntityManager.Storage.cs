using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using Arch.Core.Utils;

namespace Robust.Shared.GameObjects;

public partial class EntityManager
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int ToArchId(EntityUid uid)
    {
        return uid.Id - EntityUid.ArchUidOffset;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int ToArchVersion(EntityUid uid)
    {
        return uid.Version - EntityUid.ArchVersionOffset;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorage<T>(EntityUid uid, [NotNullWhen(true)] out T? component)
        where T : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), out component) &&
            component != null &&
            !component.Deleted)
        {
            return true;
        }

        component = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorage<T>(
        EntityUid uid,
        ComponentType type,
        [NotNullWhen(true)] out T? component)
        where T : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), type, out var obj) &&
            obj is T value &&
            !value.Deleted)
        {
            component = value;
            return true;
        }

        component = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorage(
        EntityUid uid,
        ComponentType type,
        [NotNullWhen(true)] out IComponent? component)
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), type, out var obj) &&
            obj is IComponent value &&
            !value.Deleted)
        {
            component = value;
            return true;
        }

        component = null;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorageInternal<T>(EntityUid uid, [NotNullWhen(true)] out T? component)
        where T : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), out component) &&
            component != null)
        {
            return true;
        }

        component = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorageInternal<T>(
        EntityUid uid,
        ComponentType type,
        [NotNullWhen(true)] out T? component)
        where T : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), type, out var obj) &&
            obj is T value)
        {
            component = value;
            return true;
        }

        component = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool TryGetComponentStorageInternal(
        EntityUid uid,
        ComponentType type,
        [NotNullWhen(true)] out IComponent? component)
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), type, out var obj) &&
            obj is IComponent value)
        {
            component = value;
            return true;
        }

        component = null;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetComponents<T1, T2>(
        EntityUid uid,
        [NotNullWhen(true)] out T1? component1,
        [NotNullWhen(true)] out T2? component2)
        where T1 : IComponent?
        where T2 : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), out component1, out component2) &&
            component1 != null &&
            component2 != null &&
            !component1.Deleted &&
            !component2.Deleted)
        {
            return true;
        }

        component1 = default;
        component2 = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetComponents<T1, T2, T3>(
        EntityUid uid,
        [NotNullWhen(true)] out T1? component1,
        [NotNullWhen(true)] out T2? component2,
        [NotNullWhen(true)] out T3? component3)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), out component1, out component2, out component3) &&
            component1 != null &&
            component2 != null &&
            component3 != null &&
            !component1.Deleted &&
            !component2.Deleted &&
            !component3.Deleted)
        {
            return true;
        }

        component1 = default;
        component2 = default;
        component3 = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetComponents<T1, T2, T3, T4>(
        EntityUid uid,
        [NotNullWhen(true)] out T1? component1,
        [NotNullWhen(true)] out T2? component2,
        [NotNullWhen(true)] out T3? component3,
        [NotNullWhen(true)] out T4? component4)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
        where T4 : IComponent?
    {
        if (uid.Valid &&
            _world.TryGet(ToArchId(uid), ToArchVersion(uid), out component1, out component2, out component3, out component4) &&
            component1 != null &&
            component2 != null &&
            component3 != null &&
            component4 != null &&
            !component1.Deleted &&
            !component2.Deleted &&
            !component3.Deleted &&
            !component4.Deleted)
        {
            return true;
        }

        component1 = default;
        component2 = default;
        component3 = default;
        component4 = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetEntity<T1, T2>(EntityUid uid, [NotNullWhen(true)] out Entity<T1, T2>? entity)
        where T1 : IComponent?
        where T2 : IComponent?
    {
        if (TryGetComponents<T1, T2>(uid, out var component1, out var component2))
        {
            entity = new Entity<T1, T2>(uid, component1, component2);
            return true;
        }

        entity = null;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetEntity<T1, T2, T3>(EntityUid uid, [NotNullWhen(true)] out Entity<T1, T2, T3>? entity)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
    {
        if (TryGetComponents<T1, T2, T3>(uid, out var component1, out var component2, out var component3))
        {
            entity = new Entity<T1, T2, T3>(uid, component1, component2, component3);
            return true;
        }

        entity = null;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryGetEntity<T1, T2, T3, T4>(EntityUid uid, [NotNullWhen(true)] out Entity<T1, T2, T3, T4>? entity)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
        where T4 : IComponent?
    {
        if (TryGetComponents<T1, T2, T3, T4>(uid, out var component1, out var component2, out var component3, out var component4))
        {
            entity = new Entity<T1, T2, T3, T4>(uid, component1, component2, component3, component4);
            return true;
        }

        entity = null;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasComponents<T1, T2>(EntityUid uid)
        where T1 : IComponent?
        where T2 : IComponent?
    {
        return TryGetComponents<T1, T2>(uid, out _, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasComponents<T1, T2, T3>(EntityUid uid)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
    {
        return TryGetComponents<T1, T2, T3>(uid, out _, out _, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasComponents<T1, T2, T3, T4>(EntityUid uid)
        where T1 : IComponent?
        where T2 : IComponent?
        where T3 : IComponent?
        where T4 : IComponent?
    {
        return TryGetComponents<T1, T2, T3, T4>(uid, out _, out _, out _, out _);
    }
}
