using Robust.Shared.GameObjects;

namespace Robust.Client.GameObjects
{
    internal interface IClientEntityManagerInternal : IClientEntityManager
    {
        // These methods are used by the Game State Manager.

        EntityUid CreateEntity(string? prototypeName, out MetaDataComponent metadata);
        EntityUid CreateEntity(string? prototypeName, NetEntity netEntity, out MetaDataComponent metadata);
    }
}
