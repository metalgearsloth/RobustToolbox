using JetBrains.Annotations;
using Robust.Client.Interfaces.Graphics.ClientEye;
using Robust.Shared.GameObjects.Systems;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Physics.Chunks;

namespace Robust.Client.GameObjects.EntitySystems
{
    /// <summary>
    /// Updates the layer animation for every visible sprite.
    /// </summary>
    [UsedImplicitly]
    public class SpriteSystem : EntitySystem
    {
        [Dependency] private readonly IEyeManager _eyeManager = default!;

        private SharedEntityLookupSystem _lookupSystem = default!;

        public override void Initialize()
        {
            base.Initialize();
            _lookupSystem = Get<SharedEntityLookupSystem>();
            UpdatesAfter.Add(typeof(TransformSystem));
        }

        /// <inheritdoc />
        public override void FrameUpdate(float frameTime)
        {
            // So we could calculate the correct size of the entities based on the contents of their sprite...
            // Or we can just assume that no entity is larger than 10x10 and get a stupid easy check.
            var pvsBounds = _eyeManager.GetWorldViewport().Enlarged(5);

            // TODO: Each viewport
            var currentMap = _eyeManager.CurrentMap;
            if (currentMap == MapId.Nullspace)
                return;

            foreach (var entity in _lookupSystem.GetEntitiesIntersecting(currentMap, pvsBounds, includeGrids: false))
            {
                if (!entity.TryGetComponent(out SpriteComponent? spriteComponent))
                    continue;

                if (spriteComponent.IsInert)
                    continue;

                spriteComponent.FrameUpdate(frameTime);
            }
        }
    }
}
