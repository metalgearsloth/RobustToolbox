using JetBrains.Annotations;
using Robust.Shared.Console;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;

namespace Robust.Client.Console.Commands
{
    [UsedImplicitly]
    internal sealed class AddCompCommand : IConsoleCommand
    {
        public string Command => "addcompc";
        public string Description => "Adds a component to an entity on the client";
        public string Help => "addcompc <uid> <componentName>";

        public void Execute(IConsoleShell shell, string argStr, string[] args)
        {

            if (args.Length != 2)
            {
                shell.WriteLine("Wrong number of arguments");
                return;
            }

            var entityUid = EntityUid.Parse(args[0]);
            var componentName = args[1];

            var compManager = IoCManager.Resolve<IComponentManager>();
            var compFactory = IoCManager.Resolve<IComponentFactory>();
            var entityManager = IoCManager.Resolve<IEntityManager>();

            var entity = entityManager.GetEntity(entityUid);
            var component = (Component) compFactory.GetComponent(componentName);

            component.Owner = entity;

            compManager.AddComponent(entity, component);
        }
    }

    [UsedImplicitly]
    internal sealed class RemoveCompCommand : IConsoleCommand
    {
        public string Command => "rmcompc";
        public string Description => "Removes a component from an entity.";
        public string Help => "rmcompc <uid> <componentName>";

        public void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            if (args.Length != 2)
            {
                shell.WriteLine("Wrong number of arguments");
                return;
            }

            var entityUid = EntityUid.Parse(args[0]);
            var componentName = args[1];

            var compManager = IoCManager.Resolve<IComponentManager>();
            var compFactory = IoCManager.Resolve<IComponentFactory>();

            var registration = compFactory.GetRegistration(componentName);

            compManager.RemoveComponent(entityUid, registration.Type);
        }
    }
}
