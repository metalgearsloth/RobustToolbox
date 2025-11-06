using System;
using System.Collections.Generic;
using System.Threading;
using Robust.Shared.Collections;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.NewPhysics.Islands;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.NewPhysics.Solver;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem : EntitySystem
{
    /*
     * This is derived from Box2d v3 with several changes made for RT convenience and backwards compatability.
     * Most notable:
     * - Fixtures are what box2d calls shapes, and RT shapes are the lightweight structures that can be used for queries etc.
     * - Where applicable rather than using Id references we just use actual references for the cold data (e.g. components). Any sim / hot data still uses Ids to reduce box2d differences and to keep them as structs in arrays.
     * - Worlds don't exist as we use EntityManager to handle the same concept.
     */

    // TODO: Access valuelists by ref
    // TODO: Check generations on contacts + bodies + ids.

    [Dependency] private readonly IParallelManager _parallel = default!;

    private CollideJob _collideJob = default!;
    private RebuildJob _rebuildJob = default!;
    private WaitHandle _rebuildHandle = default!;
    private SolveStageJob _solveJob = default!;
    private SplitIslandJob _splitJob = default!;
    private WaitHandle _splitHandle = default!;
    private FinalizeBodiesJob _finalizeJob = default!;

    private ConstraintGraph _constraintGraph = new();

    private readonly SolverSet[] _solverSets = new SolverSet[4];

    /*
     * Physics data
     */

    // Identify islands for splitting as follows:
    // - I want to split islands so smaller islands can sleep
    // - when a body comes to rest and its sleep timer trips, I can look at the island and flag it for splitting
    //   if it has removed constraints
    // - islands that have removed constraints must be put split first because I don't want to wake bodies incorrectly
    // - otherwise I can use the awake islands that have bodies wanting to sleep as the splitting candidates
    // - if no bodies want to sleep then there is no reason to perform island splitting
    private int _splitIslandId;

    // Box2D cheats with joints because it uses a union.

    private readonly List<Entity<PhysicsComponent>> _bodies = new();
    private readonly List<b2Contact> _contacts = new();
    private readonly List<Island> _islands = new();
    private readonly List<Fixture> _shapes = new();
    private readonly List<BaseJoint> _joints = new();

    /// <summary>
    /// Contact pairs
    /// </summary>
    private readonly HashSet<ulong> _pairSet = new();

    /*
     * Pools
     */

    private readonly IdPool _bodyIdPool = new();
    private readonly IdPool _contactIdPool = new();
    private readonly IdPool _islandIdPool = new();
    private readonly IdPool _jointIdPool = new();
    private readonly IdPool _solverSetPool = new();

    /*
     * Event buffer
     */

    private readonly List<BodyMoveEvent> _bodyMoveEvents = new(4);
    private readonly List<SensorBeginTouchEvent> _sensorBeginEvents = new (4);
    private readonly List<ContactBeginTouchEvent> _contactBeginEvents = new(4);
    private readonly List<ContactHitEvent> _contactHitEvents = new(4);
    private readonly List<JointEvent> _jointEvents = new(4);

    // End events are double buffered so that the user doesn't need to flush events
    private readonly List<SensorEndTouchEvent>[] _sensorEndEvents = new List<SensorEndTouchEvent>[2];
    private readonly List<ContactEndTouchEvent>[] _contactEndEvents = new List<ContactEndTouchEvent>[2];
    private int _endEventArrayIndex;

    /*
     * Step context
     */
    // Box2D has this as its own struct but in that case it makes sense as it has multi-world support
    // For us EntityManager is this (his) world
    // On Box2D some of the data uses pointers between the graph colors and a flat list so we just keep the flat lists instead.

    private float _dt;
    private float _invDt;
    private int _substepCount;
    private float _h;
    private float _invH;

    private Softness _contactSoftness;
    private Softness _staticSoftness;
    private bool _enableContactSoftening;

    private float _restitutionThreshold;
    private float _maxLinearSpeed;
    private bool _enableWarmStarting;

    private ValueList<SolverStage> _contextStages = new();
    private ValueList<SolverBlock> _contextBodyBlocks = new();
    private ValueList<SolverBlock> _contextContactBlocks = new();
    private ValueList<SolverBlock> _contextJointBlocks = new();
    private ValueList<SolverBlock> _contextGraphBlocks = new();

    private ValueList<BodySim> _contextSims;
    private ValueList<BodyState> _contextBodyStates = new();
    private ValueList<b2ContactConstraintSIMD> _contextSimdContactConstraints = new();
    private ValueList<ContactSim?> _contextContacts = new();
    private ValueList<JointSim> _contextJoints = new();

    private int _activeColorCount;
    private int _stageCount;

    private int _simdWidth;
    // todo_erin 4 seems good but more benchmarking would be good
    private const int blocksPerWorker = 4;
    private int _simdShift;

    private int _bulletBodyCount;
    private List<int> _bulletBodies = new();

    /*
     * CVars
     */

    // TODO: Cvar
    private bool _enableSpeculative;

    private int _substeps = 4;

    internal bool _locked;

    const int Iterations = 1;
    const int RelaxIterations = 1;

    public override void Initialize()
    {
        base.Initialize();

        _collideJob = new CollideJob();

        _rebuildJob = new RebuildJob()
        {
            Broadphase = _broadphase,
        };

        _solveJob = new();

        _splitJob = new()
        {
            System = this,
        };

        _finalizeJob = new();

        InitializeSolverSets();
    }

    private void InitializeSolverSets()
    {
        // add empty static, active, and disabled body sets
        var set = new SolverSet()
        {
            setIndex = _solverSetPool.AllocId(),
        };

        // static set
        _solverSets[0] = set;
        DebugTools.Assert(_solverSets[(int) SetType.StaticSet].setIndex == (int) SetType.StaticSet);

        // disabled set
        set = new SolverSet
        {
            setIndex = _solverSetPool.AllocId(),
        };

        _solverSets[1] = set;
        DebugTools.Assert(_solverSets[(int) SetType.DisabledSet].setIndex == (int) SetType.DisabledSet);

        // awake set
        set = new SolverSet
        {
            setIndex = _solverSetPool.AllocId(),
        };

        _solverSets[2] = set;
        DebugTools.Assert(_solverSets[(int) SetType.AwakeSet].setIndex == (int) SetType.AwakeSet);
    }

    public override void Update(float frameTime)
    {
        base.Update(frameTime);
        Step(frameTime, _substeps);
        // Run event dispatch as its own step
        // Box2d just fills up arrays and lets the caller deal with it.
        DispatchEvents(frameTime);
    }
}
