using System;
using System.Runtime.CompilerServices;
using BenchmarkDotNet.Attributes;
using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.UnitTesting.Server;

namespace Robust.Benchmarks.EntityManager;

[MemoryDiagnoser]
[Virtual]
public partial class EntityEventBusBenchmark
{
    private ISimulation _simulation = default!;
    private IEntityManager _entityManager = default!;
    private ManualBenchSystem _manualSystem = default!;
    private AttributeBenchSystem _attributeSystem = default!;
    private GeneratedBenchSystem _generatedSystem = default!;

    [GlobalSetup]
    public void GlobalSetup()
    {
        _simulation = RobustServerSimulation
            .NewSimulation()
            .RegisterComponents(f =>
            {
                f.RegisterClass<ManualBenchComponent>();
                f.RegisterClass<AttributeBenchComponent>();
                f.RegisterClass<GeneratedBenchComponent>();
            })
            .RegisterEntitySystems(f =>
            {
                f.LoadExtraSystemType<ManualBenchSystem>();
                f.LoadExtraSystemType<AttributeBenchSystem>();
                f.LoadExtraSystemType<GeneratedBenchSystem>();
            })
            .InitializeInstance();

        _entityManager = _simulation.Resolve<IEntityManager>();
        _manualSystem = _entityManager.System<ManualBenchSystem>();
        _attributeSystem = _entityManager.System<AttributeBenchSystem>();
        _generatedSystem = _entityManager.System<GeneratedBenchSystem>();

        var map = _simulation.CreateMap().MapId;

        var manualEntity = _simulation.SpawnEntity(null, new MapCoordinates(0, 0, map));
        var manualComponent = _entityManager.AddComponent<ManualBenchComponent>(manualEntity);
        _manualSystem.Entity = manualEntity;
        _manualSystem.Component = manualComponent;
        _manualSystem.OnCSharpEvent += _manualSystem.OnEvent;

        _attributeSystem.Entity = _simulation.SpawnEntity(null, new MapCoordinates(0, 0, map));
        _entityManager.AddComponent<AttributeBenchComponent>(_attributeSystem.Entity);

        _generatedSystem.Entity = _simulation.SpawnEntity(null, new MapCoordinates(0, 0, map));
        _entityManager.AddComponent<GeneratedBenchComponent>(_generatedSystem.Entity);

        ValidateSetup();
    }

    private void ValidateSetup()
    {
        if (_manualSystem.RaiseEventBus() != 1)
            throw new InvalidOperationException("Manual event bus benchmark is not dispatching.");

        if (_attributeSystem.RaiseEventBus() != 1)
            throw new InvalidOperationException("Attribute event bus benchmark is not dispatching.");

        if (_generatedSystem.RaiseEventBus() != 1)
            throw new InvalidOperationException("Generated raw thunk event bus benchmark is not dispatching.");

        if (_manualSystem.RaiseComponentBus() != 1)
            throw new InvalidOperationException("Component event bus benchmark is not dispatching.");
    }

    [Benchmark(Baseline = true)]
    public int CSharpEvent()
    {
        return _manualSystem.CSharpEvent();
    }

    [Benchmark]
    public int RaiseLocalEventManualSubscription()
    {
        return _manualSystem.RaiseEventBus();
    }

    [Benchmark]
    public int RaiseLocalEventAttributeSubscription()
    {
        return _attributeSystem.RaiseEventBus();
    }

    [Benchmark]
    public int RaiseLocalEventGeneratedRawThunk()
    {
        return _generatedSystem.RaiseEventBus();
    }

    [Benchmark]
    public int RaiseComponentEvent()
    {
        return _manualSystem.RaiseComponentBus();
    }

    public sealed partial class ManualBenchComponent : Component
    {
    }

    public sealed partial class AttributeBenchComponent : Component
    {
    }

    public sealed partial class GeneratedBenchComponent : Component
    {
    }

    public sealed class ManualBenchSystem : EntitySystem
    {
        public delegate void EventHandler(EntityUid uid, ManualBenchComponent component, ref ManualBenchEvent ev);

        public event EventHandler? OnCSharpEvent;
        public EntityUid Entity;
        public ManualBenchComponent Component = default!;

        public override void Initialize()
        {
            SubscribeLocalEvent<ManualBenchComponent, ManualBenchEvent>(OnEvent);
            SubscribeLocalEvent<ManualBenchComponent, ManualComponentBenchEvent>(OnComponentEvent);
        }

        public int CSharpEvent()
        {
            var ev = new ManualBenchEvent();
            OnCSharpEvent?.Invoke(Entity, Component, ref ev);
            return ev.Value;
        }

        public int RaiseEventBus()
        {
            var ev = new ManualBenchEvent();
            RaiseLocalEvent(Entity, ref ev);
            return ev.Value;
        }

        public int RaiseComponentBus()
        {
            var ev = new ManualComponentBenchEvent();
            RaiseComponentEvent(Entity, Component, ref ev);
            return ev.Value;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public void OnEvent(EntityUid uid, ManualBenchComponent component, ref ManualBenchEvent ev)
        {
            ev.Value = 1;
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        public void OnComponentEvent(EntityUid uid, ManualBenchComponent component, ref ManualComponentBenchEvent ev)
        {
            ev.Value = 1;
        }
    }

    public sealed partial class AttributeBenchSystem : EntitySystem
    {
        public EntityUid Entity;

        public int RaiseEventBus()
        {
            var ev = new AttributeBenchEvent();
            RaiseLocalEvent(Entity, ref ev);
            return ev.Value;
        }

        [SubscribeLocalEvent]
        [MethodImpl(MethodImplOptions.NoInlining)]
        private void OnEvent(EntityUid uid, AttributeBenchComponent component, ref AttributeBenchEvent ev)
        {
            ev.Value = 1;
        }
    }

    public sealed class GeneratedBenchSystem : EntitySystem
    {
        public EntityUid Entity;

        public override void Initialize()
        {
            SubscribeGeneratedLocalEvent<GeneratedBenchComponent, GeneratedBenchEvent>(OnEventRaw);
        }

        public int RaiseEventBus()
        {
            var ev = new GeneratedBenchEvent();
            RaiseLocalEvent(Entity, ref ev);
            return ev.Value;
        }

        private void OnEventRaw(EntityUid uid, IComponent component, ref EntityEventBusUnit ev)
        {
            ref var typed = ref Unsafe.As<EntityEventBusUnit, GeneratedBenchEvent>(ref ev);
            OnEvent(uid, (GeneratedBenchComponent) component, ref typed);
        }

        [MethodImpl(MethodImplOptions.NoInlining)]
        private void OnEvent(EntityUid uid, GeneratedBenchComponent component, ref GeneratedBenchEvent ev)
        {
            ev.Value = 1;
        }
    }

    [ByRefEvent]
    public struct ManualBenchEvent
    {
        public int Value;
    }

    [ByRefEvent]
    [ComponentEvent]
    public struct ManualComponentBenchEvent
    {
        public int Value;
    }

    [ByRefEvent]
    public struct AttributeBenchEvent
    {
        public int Value;
    }

    [ByRefEvent]
    public struct GeneratedBenchEvent
    {
        public int Value;
    }
}
