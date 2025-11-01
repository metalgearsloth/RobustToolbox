using System;
using Robust.Shared.IoC;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    [Dependency] private readonly SharedBroadphaseSystem _broadphase = default!;

    private readonly BodyMoveEvent[] _bodyMoveEvents = new BodyMoveEvent[4];
    private readonly SensorBeginTouchEvent[] _sensorBeginEvents = new SensorBeginTouchEvent[4];
    private readonly ContactBeginTouchEvent[] _contactBeginEvents = new ContactBeginTouchEvent[4];
    private readonly ContactHitEvent[] _contactHitEvents = new ContactHitEvent[4];
    private readonly JointEvent[] _jointEvents = new JointEvent[4];

    // End events are double buffered so that the user doesn't need to flush events
    private readonly SensorEndTouchEvent[][] _sensorEndEvents = new SensorEndTouchEvent[2][];
    private readonly ContactEndTouchEvent[][] _contactEndEvents = new ContactEndTouchEvent[2][];
    private int _endEventArrayIndex;

    private PhysicsProfile _profile = new();

    private float _invH;
    private float _invDt;

    private float _contactSpeed;
    private float _contactHertz;
    private float _contactDampingRatio;

    private float _restitutionThreshold;
    private float _maxLinearSpeed;
    private bool _enableWarmStarting;

    /// <summary>
    ///  Simulate a world for one time step. This performs collision detection, integration, and constraint solution.
    /// <param name="timeStep">The amount of time to simulate, this should be a fixed number. Usually 1/60.</param>
    /// <param name="subStepCount">The number of sub-steps, increasing the sub-step count can increase accuracy. Usually 4.</param>
    /// </summary>
    public void Step(float timeStep, int subStepCount)
    {
        DebugTools.Assert(!float.IsNaN(timeStep) && timeStep > 0f);
        DebugTools.Assert(subStepCount > 0);

        DebugTools.Assert(!_locked);

        if (_locked)
            return;

        // TODO: Add generations back for shapeids orrr alternatively just store the things directly probably.

        // Prepare event capture
        Array.Clear(_bodyMoveEvents);
        Array.Clear(_sensorBeginEvents);
        Array.Clear(_contactBeginEvents);
        Array.Clear(_contactHitEvents);
        Array.Clear(_jointEvents);

        // TODO: Clear profile

        if (timeStep == 0f)
        {
            _endEventArrayIndex = 1 - _endEventArrayIndex;
            Array.Clear(_sensorEndEvents[_endEventArrayIndex]);
            Array.Clear(_contactEndEvents[_endEventArrayIndex]);

            // todo_erin would be useful to still process collision while paused
            return;
        }

        _locked = true;
        _broadphase.FindNewContacts();

        var context = new StepContext
        {
            dt = timeStep,
            subStepCount = Math.Max(1, subStepCount)
        };

        if ( timeStep > 0.0f )
        {
            context.inv_dt = 1.0f / timeStep;
            context.h = timeStep / context.subStepCount;
            context.inv_h = context.subStepCount * context.inv_dt;
        }
        else
        {
            context.inv_dt = 0.0f;
            context.h = 0.0f;
            context.inv_h = 0.0f;
        }

        _invDt = context.inv_dt;
        _invH = context.inv_h;

        // Hertz values get reduced for large time steps
        float contactHertz = MathF.Min(_contactHertz, 0.125f * context.inv_h );
        context.contactSoftness = MakeSoft( contactHertz, _contactDampingRatio, context.h );
        context.staticSoftness = MakeSoft( 2.0f * contactHertz, _contactDampingRatio, context.h );

        context.restitutionThreshold = _restitutionThreshold;
        context.maxLinearVelocity = _maxLinearSpeed;
        context.enableWarmStarting = _enableWarmStarting;

        // Update contacts
        Collide(context);
    }
}
