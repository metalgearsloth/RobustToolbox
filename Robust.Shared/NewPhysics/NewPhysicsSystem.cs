using System.Collections.Generic;
using System.Threading;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
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
     */

    [Dependency] private readonly IParallelManager _parallel = default!;

    private CollideJob _collideJob = default!;
    private RebuildJob _rebuildJob = default!;
    private WaitHandle _rebuildHandle = default!;

    private ConstraintGraph _constraintGraph = new();

    private readonly SolverSet[] _solverSets = new SolverSet[4];

    /*
     * Physics data
     */

    private List<b2Contact> _contacts = new();
    private List<Fixture> _shapes = new();

    /*
     * Pools
     */

    private readonly IdPool _solverSetPool = new();
    private readonly IdPool _contactIdPool = new();
    private readonly IdPool _islandIdPool = new();

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
    }
}
