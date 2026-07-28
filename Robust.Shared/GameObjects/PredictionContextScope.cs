namespace Robust.Shared.GameObjects;

/// <summary>
/// Restores the previous prediction context when disposed.
/// </summary>
public ref struct PredictionContextScope
{
    private EntityManager? _entityManager;
    private readonly PredictionContext? _previousContext;

    internal PredictionContextScope(EntityManager entityManager, PredictionContext? previousContext)
    {
        _entityManager = entityManager;
        _previousContext = previousContext;
    }

    /// <summary>
    /// Restores the context that was active before this scope.
    /// </summary>
    public void Dispose()
    {
        var entityManager = _entityManager;
        if (entityManager == null)
            return;

        _entityManager = null;
        entityManager.RestorePredictionContext(_previousContext);
    }
}
