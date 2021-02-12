using Robust.Shared.Input.Binding;

namespace Robust.Shared.GameObjects
{
    public abstract class SharedInputSystem : EntitySystem
    {
        private readonly CommandBindRegistry _bindRegistry = new();

        /// <summary>
        ///     Holds the keyFunction -> handler bindings for the simulation.
        /// </summary>
        public ICommandBindRegistry BindRegistry => _bindRegistry;
    }
}
