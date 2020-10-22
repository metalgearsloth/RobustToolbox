using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Robust.Shared.GameObjects.Components.Transform;
using Robust.Shared.GameObjects.EntitySystemMessages;
using Robust.Shared.Interfaces.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Physics;

namespace Robust.Shared.GameObjects.Systems
{
    public abstract class OccluderSystem : EntitySystem
    {

        public IEnumerable<RayCastResults> IntersectRayWithPredicate(MapId originMapId, in Ray ray, float maxLength,
            Func<IEntity, bool>? predicate = null, bool returnOnFirstHit = true)
        {

            //var mapTree = _mapTrees[originMapId];
            var list = new List<RayCastResults>();

            /*
            mapTree.QueryRay(ref list,
                (ref List<RayCastResults> state, in OccluderComponent value, in Vector2 point, float distFromOrigin) =>
                {
                    if (distFromOrigin > maxLength)
                    {
                        return true;
                    }

                    if (!value.Enabled)
                    {
                        return true;
                    }

                    if (predicate != null && predicate.Invoke(value.Owner))
                    {
                        return true;
                    }

                    var result = new RayCastResults(distFromOrigin, point, value.Owner);
                    state.Add(result);
                    return !returnOnFirstHit;
                }, ray);
*/
            return list;
        }
    }

    internal readonly struct OccluderTreeRemoveOccluderMessage
    {
        public readonly OccluderComponent Occluder;
        public readonly MapId Map;

        public OccluderTreeRemoveOccluderMessage(OccluderComponent occluder, MapId map)
        {
            Occluder = occluder;
            Map = map;
        }
    }
}
