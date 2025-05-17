using System;
using System.Collections.Generic;

namespace Robust.Shared.Threading;

/// <summary>
/// Robust implementation of object pooling.
/// </summary>
[Virtual]
public class RobustObjectPool<T> where T : new()
{
    /// <summary>
    /// Limit on how many pooled entries we can store.
    /// </summary>
    private static int Capacity = Environment.ProcessorCount * 2;

    [ThreadStatic]
    private static readonly List<T> Pool;

    static RobustObjectPool()
    {
        Pool = new();
    }

    public virtual T Get()
    {
        if (Pool.Count > 0)
        {
            var entry = Pool[^1];
            Pool.RemoveAt(Pool.Count - 1);
            return entry;
        }

        return new T();
    }

    public virtual bool Return(T entry)
    {
        if (Pool.Count >= Capacity)
        {
            return false;
        }

        Pool.Add(entry);
        return true;
    }
}
