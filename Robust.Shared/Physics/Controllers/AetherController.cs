using System;
using JetBrains.Annotations;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Physics.Dynamics;

namespace Robust.Shared.Physics.Controllers
{
    [MeansImplicitUse]
    public abstract class AetherController
    {
        [Dependency] protected readonly IComponentManager ComponentManager = default!;
        [Dependency] protected readonly IEntityManager EntityManager = default!;

        public virtual void Initialize()
        {
            IoCManager.InjectDependencies(this);
        }

        /// <summary>
        ///     Run before any map processing starts.
        /// </summary>
        /// <param name="prediction"></param>
        /// <param name="frameTime"></param>
        public virtual void UpdateBeforeSolve(bool prediction, float frameTime) {}

        /// <summary>
        ///     Run after all map processing has finished.
        /// </summary>
        /// <param name="prediction"></param>
        /// <param name="frameTime"></param>
        public virtual void UpdateAfterSolve(bool prediction, float frameTime) {}

        /// <summary>
        ///     Run before a particular map starts.
        /// </summary>
        /// <param name="prediction"></param>
        /// <param name="map"></param>
        /// <param name="frameTime"></param>
        public virtual void UpdateBeforeMapSolve(bool prediction, PhysicsMap map, float frameTime) {}

        /// <summary>
        ///     Run after a particular map finishes.
        /// </summary>
        /// <param name="prediction"></param>
        /// <param name="map"></param>
        /// <param name="frameTime"></param>
        public virtual void UpdateAfterMapSolve(bool prediction, PhysicsMap map, float frameTime) {}
    }
}
