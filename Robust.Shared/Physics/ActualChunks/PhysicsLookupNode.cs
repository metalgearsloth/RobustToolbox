using System.Collections.Generic;
using Robust.Shared.GameObjects.Components;
using Robust.Shared.Interfaces.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Shared.Physics.Chunks
{
    internal sealed class PhysicsLookupNode
    {
        internal PhysicsLookupChunk ParentChunk { get; }

        internal Vector2i Indices { get; }

        internal IEnumerable<IPhysShape> PhysicsShapes
        {
            get
            {
                foreach (var comp in _entities)
                {
                    if (comp.Deleted)
                        continue;

                    foreach (var shape in comp.PhysicsShapes)
                    {
                        yield return shape;
                    }
                }
            }
        }

        private readonly HashSet<IPhysicsComponent> _entities = new HashSet<IPhysicsComponent>();

        internal PhysicsLookupNode(PhysicsLookupChunk parentChunk, Vector2i indices)
        {
            ParentChunk = parentChunk;
            Indices = indices;
        }

        internal void AddPhysics(IPhysicsComponent comp)
        {
            _entities.Add(comp);
        }

        internal void RemovePhysics(IPhysicsComponent comp)
        {
            _entities.Remove(comp);
        }
    }
}
