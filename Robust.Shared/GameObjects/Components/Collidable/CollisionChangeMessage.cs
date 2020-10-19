namespace Robust.Shared.GameObjects.Components
{
    public class CollisionChangeMessage : EntitySystemMessage
    {
        public EntityUid Owner { get; }
        public bool CanCollide { get; }

        // TODO: Optimise a bit
        public IPhysicsComponent PhysicsComponent { get; }

        public CollisionChangeMessage(EntityUid owner, bool canCollide, IPhysicsComponent physicsComponent)
        {
            Owner = owner;
            CanCollide = canCollide;
            PhysicsComponent = physicsComponent;
        }
    }
}
