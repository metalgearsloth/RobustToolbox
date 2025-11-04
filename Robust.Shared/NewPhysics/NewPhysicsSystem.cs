using System.Collections.Generic;
using System.Threading;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
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

    // TODO: Check generations on contacts + bodies + ids.
    // TODO: Implement pairset for the broadphase checking rather than dictionary lookups.

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

    private readonly List<Entity<PhysicsComponent>> _bodies = new();
    private readonly List<b2Contact> _contacts = new();
    private readonly List<Island> _islands = new();
    private readonly List<Fixture> _shapes = new();
    private readonly List<BaseJoint> _joints = new();

    /// <summary>
    /// Contact pairs
    /// </summary>
    private HashSet<ulong> _pairSet = new();

    /*
     * Pools
     */

    private readonly IdPool _bodyIdPool = new();
    private readonly IdPool _contactIdPool = new();
    private readonly IdPool _islandIdPool = new();
    private readonly IdPool _jointIdPool = new();
    private readonly IdPool _solverSetPool = new();

    // TODO: Cvar
    private bool _enableSpeculative;

    private int _substeps = 4;

    internal bool _locked;

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
