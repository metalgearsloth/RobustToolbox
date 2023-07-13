using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.Collections;
using Robust.Shared.Containers;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.GameObjects;

public sealed partial class EntityLookupSystem
{
    #region Private

    private void AddComponentsIntersecting<T>(
        EntityUid lookupUid,
        ComponentQueryCallback<T> callback,
        Box2 worldAABB,
        LookupFlags flags,
        EntityQuery<T> query) where T : Component
    {
        var lookup = _broadQuery.GetComponent(lookupUid);
        var invMatrix = _transform.GetInvWorldMatrix(lookupUid);
        var localAABB = invMatrix.TransformBox(worldAABB);
        var state = (callback, query);

        if ((flags & LookupFlags.Dynamic) != 0x0)
        {
            lookup.DynamicTree.QueryAabb(ref state, static (ref (ComponentQueryCallback<T> callback, EntityQuery<T> query) tuple, in FixtureProxy value) =>
            {
                if (!tuple.query.TryGetComponent(value.Entity, out var comp))
                    return true;

                tuple.callback(value.Entity, comp);
                return true;
            }, localAABB, (flags & LookupFlags.Approximate) != 0x0);
        }

        if ((flags & (LookupFlags.Static)) != 0x0)
        {
            lookup.StaticTree.QueryAabb(ref state, static (ref (ComponentQueryCallback<T> callback, EntityQuery<T> query) tuple, in FixtureProxy value) =>
            {
                if (!tuple.query.TryGetComponent(value.Entity, out var comp))
                    return true;

                tuple.callback(value.Entity, comp);
                return true;
            }, localAABB, (flags & LookupFlags.Approximate) != 0x0);
        }

        if ((flags & LookupFlags.StaticSundries) == LookupFlags.StaticSundries)
        {
            lookup.StaticSundriesTree.QueryAabb(ref state, static (ref (ComponentQueryCallback<T> callback, EntityQuery<T> query) tuple, in EntityUid value) =>
            {
                if (!tuple.query.TryGetComponent(value, out var comp))
                    return true;

                tuple.callback(value, comp);
                return true;
            }, localAABB, (flags & LookupFlags.Approximate) != 0x0);
        }

        if ((flags & LookupFlags.Sundries) != 0x0)
        {
            lookup.SundriesTree.QueryAabb(ref state, static (ref (ComponentQueryCallback<T> callback, EntityQuery<T> query) tuple, in EntityUid value) =>
            {
                if (!tuple.query.TryGetComponent(value, out var comp))
                    return true;

                tuple.callback(value, comp);
                return true;
            }, localAABB, (flags & LookupFlags.Approximate) != 0x0);
        }
    }

    private void RecursiveAdd<T>(EntityUid uid, ComponentQueryCallback<T> callback, EntityQuery<T> query) where T : Component
    {
        var childEnumerator = _xformQuery.GetComponent(uid).ChildEnumerator;

        while (childEnumerator.MoveNext(out var child))
        {
            if (query.TryGetComponent(child.Value, out var compies))
            {
                callback(child.Value, compies);
            }

            RecursiveAdd(child.Value, callback, query);
        }
    }

    private void AddContained<T>(EntityUid uid, ComponentQueryCallback<T> callback, LookupFlags flags, EntityQuery<T> query) where T : Component
    {
        if ((flags & LookupFlags.Contained) == 0x0)
            return;

        if (!_containerQuery.TryGetComponent(uid, out var conManager))
            return;

        foreach (var con in conManager.GetAllContainers())
        {
            foreach (var contained in con.ContainedEntities)
            {
                if (query.TryGetComponent(contained, out var compies))
                {
                    callback(contained, compies);
                }

                RecursiveAdd(contained, callback, query);
            }
        }
    }

    /// <summary>
    /// Should we just iterate every component and check position or do bounds checks.
    /// </summary>
    private bool UseBoundsQuery(Type type, float area)
    {
        return Count(type) > area;
    }

    /// <summary>
    /// Should we just iterate every component and check position or do bounds checks.
    /// </summary>
    private bool UseBoundsQuery<T>(float area) where T : Component
    {
        // If the component has a low count we'll just do an estimate if it's faster to iterate every comp directly
        // Might be useful to have some way to expose this to content?
        // For now we'll assume 1 entity per metre.
        return Count<T>() > area;
    }

    #endregion

    // Like .Queries but works with components
    #region Box2

    public bool AnyComponentsIntersecting(Type type, MapId mapId, Box2 worldAABB, LookupFlags flags = DefaultFlags)
    {
        DebugTools.Assert(typeof(Component).IsAssignableFrom(type));

        if (mapId == MapId.Nullspace)
            return false;

        if (!UseBoundsQuery(type, worldAABB.Height * worldAABB.Width))
        {
            foreach (var comp in EntityManager.GetAllComponents(type, true))
            {
                var uid = comp.Owner;
                var xform = _xformQuery.GetComponent(uid);

                if (xform.MapID != mapId ||
                    !worldAABB.Contains(_transform.GetWorldPosition(uid)) ||
                    ((flags & LookupFlags.Contained) == 0x0 &&
                    _container.IsEntityOrParentInContainer(uid, _metaQuery.GetComponent(uid), xform)))
                {
                    continue;
                }

                return true;
            }
        }
        else
        {
            var query = EntityManager.GetEntityQuery(type);

            // Get grid entities
            ComponentQueryCallback callback = (uid, component) =>
            {

            };

            var state = (this, callback, worldAABB, flags, query);

            _mapManager.FindGridsIntersecting(mapId, worldAABB, ref state, static (EntityUid gridUid, MapGridComponent grid, ref
                (EntityLookupSystem lookup,
                ComponentQueryCallback callback,
                Box2 worldAABB,
                LookupFlags flags,
                EntityQuery<Component> query) tuple) =>
            {
                tuple.lookup.AddComponentsIntersecting(gridUid, tuple.callback, tuple.worldAABB, tuple.flags, tuple.query);

                return true;
            });

            // Get map entities
            var mapUid = _mapManager.GetMapEntityId(mapId);
            AddComponentsIntersecting(mapUid, intersecting, worldAABB, flags, query);
            AddContained(intersecting, flags, query);
        }

        return intersecting.Count > 0;
    }

    public void GetComponentsIntersecting(Type type, MapId mapId, Box2 worldAABB, ComponentQueryCallback callback, LookupFlags flags = DefaultFlags)
    {
        DebugTools.Assert(typeof(Component).IsAssignableFrom(type));
        if (mapId == MapId.Nullspace)
            return;

        if (!UseBoundsQuery(type, worldAABB.Height * worldAABB.Width))
        {
            foreach (var comp in EntityManager.GetAllComponents(type, true))
            {
                var xform = _xformQuery.GetComponent(comp.Owner);

                if (xform.MapID != mapId ||
                    !worldAABB.Contains(_transform.GetWorldPosition(comp.Owner)) ||
                    ((flags & LookupFlags.Contained) == 0x0 &&
                     _container.IsEntityOrParentInContainer(comp.Owner, _metaQuery.GetComponent(comp.Owner), xform)))
                {
                    continue;
                }

                intersecting.Add((Component) comp);
            }
        }
        else
        {
            var query = EntityManager.GetEntityQuery(type);

            // Get grid entities
            foreach (var grid in _mapManager.FindGridsIntersecting(mapId, worldAABB))
            {
                AddComponentsIntersecting(grid.Owner, intersecting, worldAABB, flags, query);
            }

            // Get map entities
            var mapUid = _mapManager.GetMapEntityId(mapId);
            AddComponentsIntersecting(mapUid, intersecting, worldAABB, flags, query);
            AddContained(intersecting, flags, query);
        }
    }

    public void GetComponentsIntersecting<T>(MapId mapId, Box2 worldAABB, ComponentQueryCallback<T> callback, LookupFlags flags = DefaultFlags) where T : Component
    {
        if (mapId == MapId.Nullspace)
            return;

        if (!UseBoundsQuery<T>(worldAABB.Height * worldAABB.Width))
        {
            var query = AllEntityQuery<T, TransformComponent>();

            while (query.MoveNext(out var comp, out var xform))
            {
                if (xform.MapID != mapId || !worldAABB.Contains(_transform.GetWorldPosition(xform))) continue;
                intersecting.Add(comp);
            }
        }
        else
        {
            var query = GetEntityQuery<T>();

            // Get grid entities
            foreach (var grid in _mapManager.FindGridsIntersecting(mapId, worldAABB))
            {
                AddComponentsIntersecting(grid.Owner, intersecting, worldAABB, flags, query);
            }

            // Get map entities
            var mapUid = _mapManager.GetMapEntityId(mapId);
            AddComponentsIntersecting(mapUid, intersecting, worldAABB, flags, query);
            AddContained(intersecting, flags, query, callback);
        }

        return;
    }

    #endregion

    #region EntityCoordinates

    public void GetComponentsInRange<T>(EntityCoordinates coordinates, float range, ComponentQueryCallback<T> callback) where T : Component
    {
        var mapPos = coordinates.ToMap(EntityManager, _transform);
        GetComponentsInRange(mapPos, range, callback);
    }

    #endregion

    #region MapCoordinates

    public void GetComponentsInRange(Type type, MapCoordinates coordinates, float range, ComponentQueryCallback callback)
    {
        DebugTools.Assert(typeof(Component).IsAssignableFrom(type));
        GetComponentsInRange(type, coordinates.MapId, coordinates.Position, range, callback);
    }

    public void GetComponentsInRange<T>(MapCoordinates coordinates, float range, ComponentQueryCallback<T> callback) where T : Component
    {
        GetComponentsInRange(coordinates.MapId, coordinates.Position, range, callback);
    }

    #endregion

    #region MapId

    public bool AnyComponentsInRange(Type type, MapId mapId, Vector2 worldPos, float range)
    {
        DebugTools.Assert(typeof(Component).IsAssignableFrom(type));
        DebugTools.Assert(range > 0, "Range must be a positive float");

        if (mapId == MapId.Nullspace) return false;

        // TODO: Actual circles
        var rangeVec = new Vector2(range, range);

        var worldAABB = new Box2(worldPos - rangeVec, worldPos + rangeVec);
        return AnyComponentsIntersecting(type, mapId, worldAABB);
    }

    public void GetComponentsInRange(Type type, MapId mapId, Vector2 worldPos, float range, ComponentQueryCallback callback)
    {
        DebugTools.Assert(typeof(Component).IsAssignableFrom(type));
        DebugTools.Assert(range > 0, "Range must be a positive float");

        if (mapId == MapId.Nullspace)
            return;

        // TODO: Actual circles
        var rangeVec = new Vector2(range, range);

        var worldAABB = new Box2(worldPos - rangeVec, worldPos + rangeVec);
        GetComponentsIntersecting(type, mapId, worldAABB, callback);
    }

    public void GetComponentsInRange<T>(MapId mapId, Vector2 worldPos, float range, ComponentQueryCallback<T> callback) where T : Component
    {
        DebugTools.Assert(range > 0, "Range must be a positive float");

        if (mapId == MapId.Nullspace)
            return;

        // TODO: Actual circles
        var rangeVec = new Vector2(range, range);

        var worldAABB = new Box2(worldPos - rangeVec, worldPos + rangeVec);
        GetComponentsIntersecting<T>(mapId, worldAABB, callback);
    }

    #endregion
}
