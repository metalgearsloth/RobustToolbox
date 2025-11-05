using System;
using System.Collections.Generic;
using Robust.Shared.Collections;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    [Dependency] private readonly SharedBroadphaseSystem _broadphase = default!;

    private PhysicsProfile _profile = new();

    private float _contactSpeed;
    private float _contactHertz;
    private float _contactDampingRatio;

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
        _contactBeginEvents.Clear();

        _bodyMoveEvents.Clear();

        _sensorBeginEvents.Clear();
        _contactHitEvents.Clear();
        _contactHitEvents.Clear();
        _jointEvents.Clear();

        // Just because Box2D allocates these per tick we'll put them here.
        _sims = null;
        _states = null;

        if (NumericsHelpers.Vector256Enabled)
        {
            _simdWidth = 8;
        }
        else
        {
            // Yes Box2D defaults to 4 even if no SSE2
            _simdWidth = 4;
        }

        _simdShift = (int)Math.Log2(_simdWidth);

        // TODO: Clear profile

        if (timeStep == 0f)
        {
            _endEventArrayIndex = 1 - _endEventArrayIndex;
            _sensorEndEvents[_endEventArrayIndex].Clear();
            _contactEndEvents[_endEventArrayIndex].Clear();

            // todo_erin would be useful to still process collision while paused
            return;
        }

        _locked = true;
        _broadphase.FindNewContacts();

        _dt = timeStep;
        _substepCount = Math.Max(1, subStepCount);

        if (timeStep > 0f)
        {
            _invDt = 1f / timeStep;
            _h = timeStep / _substepCount;
            _invH = _substepCount * _invDt;
        }
        else
        {
            _invDt = 0f;
            _h = 0f;
            _invH = 0f;
        }

        // Hertz values get reduced for large time steps
        float contactHertz = MathF.Min(_contactHertz, 0.125f * _invH );
        _contactSoftness = MakeSoft( contactHertz, _contactDampingRatio, _h );
        _staticSoftness = MakeSoft( 2.0f * contactHertz, _contactDampingRatio, _h );

        // Update contacts
        Collide();

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (_dt > 0.0f)
        {
            Solve();
        }

        OverlapSensors();

        // Swap end event array buffers
        _endEventArrayIndex = 1 - _endEventArrayIndex;
        _sensorEndEvents[_endEventArrayIndex].Clear();
        _contactEndEvents[_endEventArrayIndex].Clear();
        _locked = false;
    }
}
