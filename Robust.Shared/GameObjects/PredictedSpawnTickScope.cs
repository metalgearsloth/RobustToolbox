using Robust.Shared.Network;
using Robust.Shared.Timing;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Temporarily registers predicted spawns against a specific prediction tick and owner.
/// </summary>
public ref struct PredictedSpawnTickScope
{
    private EntityManager? _entityManager;
    private readonly GameTick? _previousTick;
    private readonly NetUserId? _previousOwner;

    internal PredictedSpawnTickScope(
        EntityManager entityManager,
        GameTick? previousTick,
        NetUserId? previousOwner)
    {
        _entityManager = entityManager;
        _previousTick = previousTick;
        _previousOwner = previousOwner;
    }

    public void Dispose()
    {
        var entityManager = _entityManager;
        if (entityManager == null)
            return;

        _entityManager = null;
        entityManager.RestorePredictedSpawnTickScope(_previousTick, _previousOwner);
    }
}
