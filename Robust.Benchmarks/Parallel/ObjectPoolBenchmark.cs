using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using JetBrains.Annotations;
using Microsoft.Extensions.ObjectPool;
using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.Threading;

namespace Robust.Benchmarks.Parallel;

[Virtual]
public class ObjectPoolBenchmark
{
    private ObjectPool<TransformComponent> _defaultPool = default!;

    private RobustObjectPool<TransformComponent> _robustPool = default!;

    [UsedImplicitly]
    [Params(10, 100, 1000, 10000, 100000)]
    public int N;

    private Consumer _consume = new();

    [GlobalSetup]
    public void Setup()
    {
        _defaultPool =
        new DefaultObjectPool<TransformComponent>(new DefaultPooledObjectPolicy<TransformComponent>());

        _robustPool = new RobustObjectPool<TransformComponent>();
    }

    [Benchmark]
    public void GetDefault()
    {
        for (var i = 0; i < N; i++)
        {
            var entry = _defaultPool.Get();
            _consume.Consume(entry);
        }
    }

    [Benchmark]
    public void GetReturnDefault()
    {
        var entries = new List<TransformComponent>(N);

        for (var i = 0; i < N; i++)
        {
            entries.Add(_defaultPool.Get());
        }

        for (var i = 0; i < N; i++)
        {
            _defaultPool.Return(entries[i]);
        }
    }

    [Benchmark]
    public void GetRobust()
    {
        for (var i = 0; i < N; i++)
        {
            var entry = _robustPool.Get();
            _consume.Consume(entry);
        }
    }

    [Benchmark]
    public void GetReturnRobust()
    {
        var entries = new List<TransformComponent>(N);

        for (var i = 0; i < N; i++)
        {
            entries.Add(_robustPool.Get());
        }

        for (var i = 0; i < N; i++)
        {
            _robustPool.Return(entries[i]);
        }
    }
}
