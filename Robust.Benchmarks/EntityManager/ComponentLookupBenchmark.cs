using System;
using Arch.Core;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Engines;
using JetBrains.Annotations;
using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.UnitTesting.Server;

namespace Robust.Benchmarks.EntityManager;

[MemoryDiagnoser]
[Virtual]
public partial class ComponentLookupBenchmark
{
    private static readonly Consumer Consumer = new();

    private ISimulation _simulation = default!;
    private Robust.Shared.GameObjects.EntityManager _entityManager = default!;
    private EntityQuery<A> _queryA = default!;
    private EntityUid[] _uids = default!;

    private A[] _denseA = default!;
    private A?[][] _bucketedA = default!;

    private World _archWorld = default!;
    private Entity[] _archEntities = default!;

    [UsedImplicitly]
    [Params(1, 1000)]
    public int N;

    [GlobalSetup]
    public void GlobalSetup()
    {
        _simulation = RobustServerSimulation
            .NewSimulation()
            .RegisterComponents(f =>
            {
                f.RegisterClass<A>();
                f.RegisterClass<B>();
                f.RegisterClass<C>();
                f.RegisterClass<D>();
            })
            .InitializeInstance();

        _entityManager = _simulation.Resolve<Robust.Shared.GameObjects.EntityManager>();
        _queryA = _entityManager.GetEntityQuery<A>();
        _uids = new EntityUid[N];
        _denseA = new A[16];
        _bucketedA = [];

        var map = _simulation.CreateMap().Uid;
        var coords = new EntityCoordinates(map, default);

        for (var i = 0; i < N; i++)
        {
            var uid = _entityManager.SpawnEntity(null, coords);
            _uids[i] = uid;

            var a = _entityManager.AddComponent<A>(uid);
            _entityManager.AddComponent<B>(uid);
            _entityManager.AddComponent<C>(uid);
            _entityManager.AddComponent<D>(uid);

            if (uid.Id >= _denseA.Length)
                EnsureDenseCapacity(uid.Id + 1);

            _denseA[uid.Id] = a;
            SetBucketed(uid.Id, a);
        }

        _archWorld = World.Create();
        _archEntities = new Entity[N];
        for (var i = 0; i < N; i++)
        {
            var entity = _archWorld.Create();
            _archWorld.Add(entity, new A(), new B(), new C(), new D());
            _archEntities[i] = entity;
        }
    }

    [GlobalCleanup]
    public void GlobalCleanup()
    {
        World.Destroy(_archWorld);
    }

    [Benchmark(Baseline = true)]
    public void TryGetComponent()
    {
        foreach (var uid in _uids)
        {
            _entityManager.TryGetComponent<A>(uid, out var a);
            Consumer.Consume(a!);
        }
    }

    [Benchmark]
    public void EntityQueryTryGetComponent()
    {
        foreach (var uid in _uids)
        {
            _queryA.TryGetComponent(uid, out var a);
            Consumer.Consume(a!);
        }
    }

    [Benchmark]
    public void FourSeparateTryGetComponent()
    {
        foreach (var uid in _uids)
        {
            _entityManager.TryGetComponent<A>(uid, out var a);
            _entityManager.TryGetComponent<B>(uid, out var b);
            _entityManager.TryGetComponent<C>(uid, out var c);
            _entityManager.TryGetComponent<D>(uid, out var d);
            Consumer.Consume((a, b, c, d));
        }
    }

    [Benchmark]
    public void BulkTryGetComponents4()
    {
        foreach (var uid in _uids)
        {
            _entityManager.TryGetComponents<A, B, C, D>(uid, out var a, out var b, out var c, out var d);
            Consumer.Consume((a, b, c, d));
        }
    }

    [Benchmark]
    public void HasComponent()
    {
        foreach (var uid in _uids)
        {
            Consumer.Consume(_entityManager.HasComponent<A>(uid));
        }
    }

    [Benchmark]
    public void HasComponents4()
    {
        foreach (var uid in _uids)
        {
            Consumer.Consume(_entityManager.HasComponents<A, B, C, D>(uid));
        }
    }

    [Benchmark]
    public void DenseArrayGet()
    {
        foreach (var uid in _uids)
        {
            Consumer.Consume(_denseA[uid.Id]);
        }
    }

    [Benchmark]
    public void BucketedArrayGet()
    {
        foreach (var uid in _uids)
        {
            Consumer.Consume(GetBucketed(uid.Id)!);
        }
    }

    [Benchmark]
    public void RawArchTryGet4()
    {
        foreach (var entity in _archEntities)
        {
            _archWorld.TryGet(entity.Id, entity.Version, out A? a, out B? b, out C? c, out D? d);
            Consumer.Consume(((object?) a, (object?) b, (object?) c, (object?) d));
        }
    }

    private void EnsureDenseCapacity(int capacity)
    {
        if (capacity > _denseA.Length)
            Array.Resize(ref _denseA, capacity);
    }

    private void SetBucketed(int id, A component)
    {
        const int bucketBits = 10;
        const int bucketSize = 1 << bucketBits;
        const int bucketMask = bucketSize - 1;

        var bucket = id >> bucketBits;
        if (bucket >= _bucketedA.Length)
        {
            Array.Resize(ref _bucketedA, bucket + 1);
        }

        _bucketedA[bucket] ??= new A[bucketSize];
        _bucketedA[bucket][id & bucketMask] = component;
    }

    private A? GetBucketed(int id)
    {
        const int bucketBits = 10;
        const int bucketMask = (1 << bucketBits) - 1;

        var bucket = _bucketedA[id >> bucketBits];
        return bucket?[id & bucketMask];
    }

    [ComponentProtoName("A")]
    public sealed partial class A : Component
    {
    }

    [ComponentProtoName("B")]
    public sealed partial class B : Component
    {
    }

    [ComponentProtoName("C")]
    public sealed partial class C : Component
    {
    }

    [ComponentProtoName("D")]
    public sealed partial class D : Component
    {
    }
}
