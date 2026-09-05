using Robust.Client.Player;
using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;

namespace Robust.Client.GameObjects;

/// <summary>
/// Raises client-only z-level notifications for the entity controlled by the local player.
/// </summary>
public sealed partial class ClientZLevelSystem : EntitySystem
{
    [Dependency] private IPlayerManager _playerManager = default!;
    [Dependency] private ZLevelSystem _zLevels = default!;
    [Dependency] private TransformSystem _renderTransforms = default!;

    /// <inheritdoc/>
    public override void Initialize()
    {
        base.Initialize();
        _renderTransforms.RenderSpaceCompatibility += OnRenderSpaceCompatibility;
    }

    public override void Shutdown()
    {
        _renderTransforms.RenderSpaceCompatibility -= OnRenderSpaceCompatibility;
        base.Shutdown();
    }

    private void OnRenderSpaceCompatibility(ref RenderSpaceCompatibilityEvent args)
    {
        if (_zLevels.TryGetMapDepthOffset(args.First, args.Second, out _))
            args.CommonSpace = args.Second;
    }

    [SubscribeLocalEvent]
    private void OnParentChanged(ref EntParentChangedMessage args)
    {
        if (_playerManager.LocalEntity != args.Entity ||
            args.OldMapId is not { } oldMap ||
            args.Transform.MapUid is not { } newMap ||
            oldMap == newMap ||
            !_zLevels.TryGetMapDepthOffset(oldMap, newMap, out var offset) ||
            offset == 0)
        {
            return;
        }

        var ev = new LocalEntityZLevelChangedEvent(args.Entity, oldMap, newMap, offset);
        RaiseLocalEvent(ev);
    }
}

/// <summary>
/// Raised locally on the client when its controlled entity moves between maps in the same z-level network.
/// </summary>
/// <param name="Entity">The locally controlled entity that moved.</param>
/// <param name="OldMap">The map the entity moved from.</param>
/// <param name="NewMap">The map the entity moved to.</param>
/// <param name="Offset">The signed z-level depth change; positive is up and negative is down.</param>
public sealed class LocalEntityZLevelChangedEvent(
    EntityUid entity,
    EntityUid oldMap,
    EntityUid newMap,
    int offset) : EntityEventArgs
{
    public EntityUid Entity { get; } = entity;
    public EntityUid OldMap { get; } = oldMap;
    public EntityUid NewMap { get; } = newMap;
    public int Offset { get; } = offset;
}
