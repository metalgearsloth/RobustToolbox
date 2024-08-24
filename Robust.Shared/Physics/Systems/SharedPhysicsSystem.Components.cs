/*
 * Farseer Physics Engine:
 * Copyright (c) 2012 Ian Qvist
 *
 * Original source Box2D:
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * PhysicsComponent is heavily modified from Box2D.
 */

using System;
using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.Collections;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Events;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public partial class SharedPhysicsSystem
{
    internal const int StateFieldCount = 12;

    #region Lifetime

    private void OnPhysicsMapInit(Entity<PhysicsComponent> entity, MapInitEvent args)
    {
        entity.Comp.SleepTime = _cfg.GetCVar(CVars.TimeToSleep);

        if (entity.Comp.SleepTime > 0f)
        {
            if (!WakeBody(entity))
            {
                entity.Comp.SleepTime = 0f;
            }
        }

        Dirty(entity);
    }

    private void OnPhysicsInit(EntityUid uid, PhysicsComponent component, ComponentInit args)
    {
        var xform = Transform(uid);

        if (component.CanCollide && (_containerSystem.IsEntityOrParentInContainer(uid) || xform.MapID == MapId.Nullspace))
        {
            SetCanCollide(uid, false, false, body: component);
        }

        // TODO: Set sleeptime to

        if (component.CanCollide)
        {
            if (component.BodyType != BodyType.Static)
            {
                SetAwake((uid, component), true);
            }
        }

        // Can't ACTUALLY add it to the broadphase here because transform is still in a transient dimension on the 5th plane
        // hence we'll just make sure its body is set and SharedBroadphaseSystem will deal with it later.

        // Make sure all the right stuff is set on the body
        FixtureUpdate(uid, dirty: false, body: component);

        if (component.FixtureCount == 0)
            component.CanCollide = false;

        var ev = new CollisionChangeEvent(uid, component, component.CanCollide);
        RaiseLocalEvent(ref ev);

        if (component.Awake)
        {
            AddAwakeBody((uid, component));
        }
    }

    private void OnPhysicsGetState(EntityUid uid, PhysicsComponent component, ref ComponentGetState args)
    {
        if (args.FromTick > component.CreationTick && component.LastFieldUpdate >= args.FromTick)
        {
            var slowPath = false;

            for (var i = 0; i < _angularVelocityIndex; i++)
            {
                var field = component.LastModifiedFields[i];

                if (field < args.FromTick)
                    continue;

                slowPath = true;
                break;
            }

            // We can do a smaller delta with no list index overhead.
            if (!slowPath)
            {
                var angularDirty = component.LastModifiedFields[_angularVelocityIndex] >= args.FromTick;

                if (angularDirty)
                {
                    args.State = new PhysicsVelocityDeltaState()
                    {
                        AngularVelocity = component.AngularVelocity,
                        LinearVelocity = component.LinearVelocity,
                    };
                }
                else
                {
                    args.State = new PhysicsLinearVelocityDeltaState()
                    {
                        LinearVelocity = component.LinearVelocity,
                    };
                }

                return;
            }

            // Slowpath :(
            uint fields = 0;
            var data = new ValueList<object?>();

            for (byte i = 0; i < component.LastModifiedFields.Length; i++)
            {
                var lastUpdate = component.LastModifiedFields[i];

                if (lastUpdate < args.FromTick)
                    continue;

                fields |= (uint) (1 << i);

                switch (i)
                {
                    case 0:
                        data.Add(component.CanCollide);
                        break;
                    case 1:
                        data.Add(component.BodyType);
                        break;
                    case 2:
                        data.Add(component.SleepingAllowed);
                        break;
                    case 3:
                        data.Add(component.FixedRotation);
                        break;
                    case 4:
                        data.Add(component._friction);
                        break;
                    case 5:
                        data.Add(component.Force);
                        break;
                    case 6:
                        data.Add(component.Torque);
                        break;
                    case 7:
                        data.Add(component.LinearDamping);
                        break;
                    case 8:
                        data.Add(component.AngularDamping);
                        break;
                    case 9:
                        data.Add(GetFixturesCopy(component.Fixtures));
                        break;
                    case 10:
                        data.Add(component.AngularVelocity);
                        break;
                    case 11:
                        data.Add(component.LinearVelocity);
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }

            args.State = new PhysicsDeltaState()
            {
                ModifiedFields = fields,
                Fields = data.ToArray(),
            };
            return;
        }

        args.State = new PhysicsComponentState
        {
            CanCollide = component.CanCollide,
            SleepingAllowed = component.SleepingAllowed,
            FixedRotation = component.FixedRotation,
            LinearVelocity = component.LinearVelocity,
            AngularVelocity = component.AngularVelocity,
            BodyType = component.BodyType,
            Friction = component._friction,
            LinearDamping = component.LinearDamping,
            AngularDamping = component.AngularDamping,
            Force = component.Force,
            Torque = component.Torque,
        };
    }

    internal static Dictionary<string, Fixture> GetFixturesCopy(Dictionary<string, Fixture> fixtures)
    {
        var target = new Dictionary<string, Fixture>(fixtures.Count);

        foreach (var (id, fixture) in fixtures)
        {
            target[id] = new Fixture(fixture);
        }

        return target;
    }
    private void OnPhysicsHandleState(EntityUid uid, PhysicsComponent component, ref ComponentHandleState args)
    {
        if (args.Current == null)
            return;

        // So transform doesn't apply MapId in the HandleComponentState because ??? so MapId can still be 0.
        // Fucking kill me, please. You have no idea deep the rabbit hole of shitcode goes to make this work.
        if (args.Current is PhysicsLinearVelocityDeltaState linearState)
        {
            SetLinearVelocity(uid, linearState.LinearVelocity, body: component);
        }
        else if (args.Current is PhysicsVelocityDeltaState velocityState)
        {
            SetLinearVelocity(uid, velocityState.LinearVelocity, body: component);
            SetAngularVelocity(uid, velocityState.AngularVelocity, body: component);
        }
        else if (args.Current is PhysicsDeltaState deltaState)
        {
            byte index = 0;

            for (var i = 0; i < StateFieldCount; i++)
            {
                var field = 1 << i;

                // Field not dirty
                if ((deltaState.ModifiedFields & field) == 0x0)
                    continue;

                var value = deltaState.Fields[index];

                switch (i)
                {
                    case 0:
                        SetCanCollide(uid, (bool)value!, body: component);
                        break;
                    case 1:
                        component.BodyStatus = (BodyStatus)value!;
                        break;
                    case 2:
                        SetBodyType(uid, (BodyType)value!, component);
                        break;
                    case 3:
                        SetSleepingAllowed(uid, component, (bool)value!);
                        break;
                    case 4:
                        SetFixedRotation(uid, (bool)value!, body: component);
                        break;
                    case 5:
                        SetFriction(uid, component, (float)value!);
                        break;
                    case 6:
                        component.Force = (Vector2)value!;
                        break;
                    case 7:
                        component.Torque = (float)value!;
                        break;
                    case 8:
                        SetLinearDamping(uid, component, (float)value!);
                        break;
                    case 9:
                        SetAngularDamping(uid, component, (float)value!);
                        break;
                    case 10:
                        break;
                    case 11:
                        SetAngularVelocity(uid, (float)value!, body: component);
                        break;
                    case 12:
                        SetLinearVelocity(uid, (Vector2)value!, body: component);
                        break;
                    default:
                        throw new ArgumentOutOfRangeException();
                }

                index++;
            }
        }
        else if (args.Current is PhysicsComponentState newState)
        {
            SetSleepingAllowed(uid, component, newState.SleepingAllowed);
            SetFixedRotation(uid, newState.FixedRotation, body: component);
            SetCanCollide(uid, newState.CanCollide, body: component);
            component.BodyStatus = newState.Status;

            SetLinearVelocity(uid, newState.LinearVelocity, body: component);
            SetAngularVelocity(uid, newState.AngularVelocity, body: component);
            SetBodyType(uid, newState.BodyType, component);
            SetFriction(uid, component, newState.Friction);
            SetLinearDamping(uid, component, newState.LinearDamping);
            SetAngularDamping(uid, component, newState.AngularDamping);
            component.Force = newState.Force;
            component.Torque = newState.Torque;

            var toAddFixtures = new ValueList<(string Id, Fixture Fixture)>();
            var toRemoveFixtures = new ValueList<(string Id, Fixture Fixture)>();
            var computeProperties = false;

            // Given a bunch of data isn't serialized need to sort of re-initialise it
            var newFixtures = new Dictionary<string, Fixture>(newState.Fixtures.Count);

            foreach (var (id, fixture) in newState.Fixtures)
            {
                var newFixture = new Fixture();
                fixture.CopyTo(newFixture);
                newFixtures.Add(id, newFixture);
            }

            TransformComponent? xform = null;

            // Add / update new fixtures
            // FUTURE SLOTH
            // Do not touch this or I WILL GLASS YOU.
            // Updating fixtures in place causes prediction issues with contacts.
            // See PR #3431 for when this started.
            foreach (var (id, fixture) in newFixtures)
            {
                if (!component.Fixtures.TryGetValue(id, out var existing))
                {
                    toAddFixtures.Add((id, fixture));
                }
                else if (!existing.Equivalent(fixture))
                {
                    toRemoveFixtures.Add((id, existing));
                    toAddFixtures.Add((id, fixture));
                }
            }

            // Remove old fixtures
            foreach (var (existingId, existing) in component.Fixtures)
            {
                if (!newFixtures.ContainsKey(existingId))
                {
                    toRemoveFixtures.Add((existingId, existing));
                }
            }

            // TODO add a DestroyFixture() override that takes in a list.
            // reduced broadphase lookups
            foreach (var (id, fixture) in toRemoveFixtures.Span)
            {
                computeProperties = true;
                DestroyFixture(uid, id, fixture, false, component);
            }

            // TODO: We also still need event listeners for shapes (Probably need C# events)
            // Or we could just make it so shapes can only be updated via fixturesystem which handles it
            // automagically (friends or something?)
            foreach (var (id, fixture) in toAddFixtures.Span)
            {
                computeProperties = true;
                CreateFixture(uid, id, fixture, false, component, xform);
            }

            if (computeProperties)
            {
                FixtureUpdate(uid, body: component);
            }
        }
    }

    #endregion

    private bool IsMoveable(PhysicsComponent body)
    {
        return (body.BodyType & (BodyType.Dynamic | BodyType.KinematicController)) != 0x0;
    }

    #region Impulses

    public void ApplyAngularImpulse(EntityUid uid, float impulse, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        SetAngularVelocity(uid, body.AngularVelocity + impulse * body.InvI, body: body);
    }

    public void ApplyForce(EntityUid uid, Vector2 force, Vector2 point, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        body.Force += force;
        body.Torque += Vector2Helpers.Cross(point - body._localCenter, force);
    }

    public void ApplyForce(EntityUid uid, Vector2 force, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        body.Force += force;
    }

    public void ApplyTorque(EntityUid uid, float torque, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        body.Torque += torque;
        DirtyField(uid, body, nameof(PhysicsComponent.Torque));
    }

    public void ApplyLinearImpulse(EntityUid uid, Vector2 impulse, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        SetLinearVelocity(uid,body.LinearVelocity + impulse * body._invMass, body: body);
    }

    public void ApplyLinearImpulse(EntityUid uid, Vector2 impulse, Vector2 point, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || !IsMoveable(body) || !WakeBody(uid, body: body))
        {
            return;
        }

        SetLinearVelocity(uid, body.LinearVelocity + impulse * body._invMass, body: body);
        SetAngularVelocity(uid, body.AngularVelocity + body.InvI * Vector2Helpers.Cross(point - body._localCenter, impulse), body: body);
    }

    #endregion

    #region Setters

    public void DestroyContacts(PhysicsComponent body)
    {
        if (body.Contacts.Count == 0) return;

        // This variable is only used in edge-case scenario when contact flagged Deleting raises
        // EndCollideEvent which will QueueDelete contact's entity
        ushort contactsFlaggedDeleting = 0;
        var node = body.Contacts.First;

        while (node != null)
        {
            var contact = node.Value;
            node = node.Next;

            // Destroy last so the linked-list doesn't get touched.
            if (!DestroyContact(contact))
            {
                contactsFlaggedDeleting++;
            }
        }

        // This contact will be deleted before SimulateWorld runs since it is already set to be Deleted
        DebugTools.Assert(body.Contacts.Count == contactsFlaggedDeleting);
    }

    /// <summary>
    /// Completely resets a dynamic body.
    /// </summary>
    public void ResetDynamics(EntityUid uid, PhysicsComponent body, bool dirty = true)
    {
        if (body.Torque != 0f)
        {
            body.Torque = 0f;
            DirtyField(uid, body, nameof(PhysicsComponent.Torque));
        }

        if (body.AngularVelocity != 0f)
        {
            body.AngularVelocity = 0f;
            DirtyField(uid, body, nameof(PhysicsComponent.AngularVelocity));
        }

        if (body.Force != Vector2.Zero)
        {
            body.Force = Vector2.Zero;
            DirtyField(uid, body, nameof(PhysicsComponent.Force));
        }

        if (body.LinearVelocity != Vector2.Zero)
        {
            body.LinearVelocity = Vector2.Zero;
            DirtyField(uid, body, nameof(PhysicsComponent.LinearVelocity));
        }
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void ResetDynamics(PhysicsComponent body, bool dirty = true)
    {
        ResetDynamics(body.Owner, body, dirty);
    }

    public void ResetMassData(EntityUid uid, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return;

        var oldMass = body._mass;
        var oldInertia = body._inertia;

        body._mass = 0.0f;
        body._invMass = 0.0f;
        body._inertia = 0.0f;
        body.InvI = 0.0f;
        var localCenter = Vector2.Zero;

        foreach (var fixture in body.Fixtures.Values)
        {
            if (fixture.Density <= 0.0f) continue;

            var data = new MassData();
            GetMassData(fixture.Shape, ref data, fixture.Density);

            body._mass += data.Mass;
            localCenter += data.Center * data.Mass;
            body._inertia += data.I;
        }

        // Update this after re-calculating mass as content may want to use the sum of fixture masses instead.
        if (((int) body.BodyType & (int) (BodyType.Kinematic | BodyType.Static)) != 0)
        {
            body._localCenter = Vector2.Zero;
            return;
        }

        if (body._mass > 0.0f)
        {
            body._invMass = 1.0f / body._mass;
            localCenter *= body._invMass;
        }
        else
        {
            // Always need positive mass.
            body._mass = 1.0f;
            body._invMass = 1.0f;
        }

        if (body._inertia > 0.0f && !body.FixedRotation)
        {
            // Center inertia about center of mass.
            body._inertia -= body._mass * Vector2.Dot(localCenter, localCenter);

            DebugTools.Assert(body._inertia > 0.0f);
            body.InvI = 1.0f / body._inertia;
        }
        else
        {
            body._inertia = 0.0f;
            body.InvI = 0.0f;
        }

        var oldCenter = body._localCenter;
        body._localCenter = localCenter;

        // Update center of mass velocity.
        var comVelocityDiff = Vector2Helpers.Cross(body.AngularVelocity, localCenter - oldCenter);

        if (comVelocityDiff != Vector2.Zero)
       	{
       		body.LinearVelocity += comVelocityDiff;
        	DirtyField(uid, body, nameof(PhysicsComponent.LinearVelocity));
       	}

        if (body._mass == oldMass && body._inertia == oldInertia && oldCenter == localCenter)
            return;

        var ev = new MassDataChangedEvent((uid, body), oldMass, oldInertia, oldCenter);
        RaiseLocalEvent(uid, ref ev);
    }

    public bool SetAngularVelocity(EntityUid uid, float value, bool dirty = true, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return false;

        if (body.BodyType == BodyType.Static)
            return false;

        if (value * value > 0.0f)
        {
            if (!WakeBody(uid, body: body))
                return false;
        }

        // CloseToPercent tolerance needs to be small enough such that an angular velocity just above
        // sleep-tolerance can damp down to sleeping.

        if (MathHelper.CloseToPercent(body.AngularVelocity, value, 0.00001f))
            return false;

        body.AngularVelocity = value;
        DirtyField(uid, body, nameof(PhysicsComponent.AngularVelocity));

        return true;
    }

    /// <summary>
    /// Attempts to set the body to collidable, wake it, then move it.
    /// </summary>
    public bool SetLinearVelocity(EntityUid uid, Vector2 velocity, bool dirty = true, bool wakeBody = true, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return false;

        if (body.BodyType == BodyType.Static)
            return false;

        if (wakeBody && Vector2.Dot(velocity, velocity) > 0.0f)
        {
            if (!WakeBody(uid, body: body))
                return false;
        }

        if (body.LinearVelocity.EqualsApprox(velocity, 0.0000001f))
            return false;

        body.LinearVelocity = velocity;
        DirtyField(uid, body, nameof(PhysicsComponent.LinearVelocity));
        return true;
    }

    public void SetAngularDamping(EntityUid uid, PhysicsComponent body, float value, bool dirty = true)
    {
        if (MathHelper.CloseTo(body.AngularDamping, value))
            return;

        body.AngularDamping = value;
        DirtyField(uid, body, nameof(PhysicsComponent.AngularDamping));
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void SetAngularDamping(PhysicsComponent body, float value, bool dirty = true)
    {
        SetAngularDamping(body.Owner, body, value, dirty);
    }

    public void SetLinearDamping(EntityUid uid, PhysicsComponent body, float value, bool dirty = true)
    {
        if (MathHelper.CloseTo(body.LinearDamping, value))
            return;

        body.LinearDamping = value;
        DirtyField(uid, body, nameof(PhysicsComponent.LinearDamping));
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void SetLinearDamping(PhysicsComponent body, float value, bool dirty = true)
    {
        SetLinearDamping(body.Owner, body, value, dirty);
    }

    [Obsolete("Use SetAwake with EntityUid<PhysicsComponent>")]
    public void SetAwake(EntityUid uid, PhysicsComponent body, bool value, bool updateSleepTime = true)
    {
        SetAwake(new Entity<PhysicsComponent>(uid, body), value, updateSleepTime);
    }

    public void SetAwake(Entity<PhysicsComponent> ent, bool value, bool updateSleepTime = true)
    {
        var (uid, body) = ent;
        var canWake = body.BodyType != BodyType.Static && body.CanCollide;

        if (body.Awake == value)
        {
            DebugTools.Assert(!body.Awake || canWake);
            return;
        }

        if (value && !canWake)
            return;

        body.Awake = value;

        if (value)
        {
            var ev = new PhysicsWakeEvent(uid, body);
            RaiseLocalEvent(uid, ref ev, true);
        }
        else
        {
            var ev = new PhysicsSleepEvent(uid, body);
            RaiseLocalEvent(uid, ref ev, true);
            ResetDynamics(ent, body, dirty: false);
        }

        // Update wake system last, if sleeping but still colliding.
        if (!value && body.CanCollide)
            _wakeSystem.UpdateCanCollide(ent, checkTerminating: false, dirty: false);

        if (updateSleepTime)
            SetSleepTime(body, 0);

        if (body.Awake != value)
        {
            Log.Error($"Found a corrupted physics awake state for {ToPrettyString(ent)}! Did you forget to cancel the sleep subscription? Forcing body awake");
            DebugTools.Assert(false);
            body.Awake = true;
        }

        UpdateMapAwakeState(uid, body);
    }

    public void TrySetBodyType(EntityUid uid, BodyType value, PhysicsComponent? body = null, TransformComponent? xform = null)
    {
        if (PhysicsQuery.Resolve(uid, ref body, false) &&
            _xformQuery.Resolve(uid, ref xform, false))
        {
            SetBodyType(uid, value, body, xform);
        }
    }

    public void SetBodyType(EntityUid uid, BodyType value, PhysicsComponent? body = null, TransformComponent? xform = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return;

        if (body.BodyType == value)
            return;

        var oldType = body.BodyType;
        body.BodyType = value;
        ResetMassData(uid, body);

        if (body.BodyType == BodyType.Static)
        {
            SetAwake((uid, body), false);

            if (body.LinearVelocity != Vector2.Zero)
            {
                body.LinearVelocity = Vector2.Zero;
                DirtyField(uid, body, nameof(PhysicsComponent.LinearVelocity));
            }

            if (body.AngularVelocity != 0f)
            {
                body.AngularVelocity = 0f;
                DirtyField(uid, body, nameof(PhysicsComponent.AngularVelocity));
            }
        }
        // Even if it's dynamic if it can't collide then don't force it awake.
        else if (body.CanCollide)
        {
            SetAwake((uid, body), true);
        }

        if (body.Torque != 0f)
        {
            body.Torque = 0f;
            DirtyField(uid, body, nameof(PhysicsComponent.Torque));
        }

        _broadphase.RegenerateContacts(uid, body, xform);

        if (body.Initialized)
        {
            var ev = new PhysicsBodyTypeChangedEvent(uid, body.BodyType, oldType, body);
            RaiseLocalEvent(uid, ref ev, true);
        }
    }

    public void SetBodyStatus(EntityUid uid, PhysicsComponent body, BodyStatus status, bool dirty = true)
    {
        if (body.BodyStatus == status)
            return;

        body.BodyStatus = status;
        DirtyField(uid, body, nameof(PhysicsComponent.BodyStatus));
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void SetBodyStatus(PhysicsComponent body, BodyStatus status, bool dirty = true)
    {
        SetBodyStatus(body.Owner, body, status, dirty);
    }

    /// <summary>
    /// Sets the <see cref="PhysicsComponent.CanCollide"/> property; this handles whether the body is enabled.
    /// </summary>
    /// <returns>CanCollide</returns>
    /// <param name="force">Bypasses fixture and container checks</param>
    public bool SetCanCollide(
        EntityUid uid,
        bool value,
        bool dirty = true,
        bool force = false,
        PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return false;

        if (body.CanCollide == value)
            return value;

        if (value)
        {
            if (!force)
            {
                // If we're recursively in a container then never set this.
                if (_containerSystem.IsEntityOrParentInContainer(uid))
                    return false;

                if (body.FixtureCount == 0 && !_mapManager.IsGrid(uid))
                    return false;
            }
            else
            {
                DebugTools.Assert(!_containerSystem.IsEntityOrParentInContainer(uid));
                DebugTools.Assert(body.FixtureCount > 0 || _mapManager.IsGrid(uid));
            }
        }

        // Need to do this before SetAwake to avoid double-changing it.
        body.CanCollide = value;

        if (!value)
            SetAwake((uid, body), false);

        if (body.Initialized)
        {
            var ev = new CollisionChangeEvent(uid, body, value);
            RaiseLocalEvent(ref ev);
        }
        DirtyField(uid, body, nameof(PhysicsComponent.CanCollide));
        return value;
    }

    public void SetFixedRotation(EntityUid uid, bool value, bool dirty = true, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body) || body.FixedRotation == value)
            return;

        body.FixedRotation = value;
        DirtyField(uid, body, nameof(PhysicsComponent.FixedRotation));

        if (body.AngularVelocity != 0f)
        {
            body.AngularVelocity = 0.0f;
            DirtyField(uid, body, nameof(PhysicsComponent.AngularVelocity));
        }

        ResetMassData(uid, body: body);
    }

    public void SetFriction(EntityUid uid, PhysicsComponent body, float value, bool dirty = true)
    {
        if (MathHelper.CloseTo(body.Friction, value))
            return;

        body._friction = value;
        DirtyField(uid, body, nameof(PhysicsComponent.Friction));
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void SetFriction(PhysicsComponent body, float value, bool dirty = true)
    {
        SetFriction(body.Owner, body, value, dirty);
    }

    public void SetInertia(EntityUid uid, PhysicsComponent body, float value, bool dirty = true)
    {
        DebugTools.Assert(!float.IsNaN(value));

        if (body.BodyType != BodyType.Dynamic) return;

        if (MathHelper.CloseToPercent(body._inertia, value)) return;

        if (value > 0.0f && !body.FixedRotation)
        {
            body._inertia = value - body.Mass * Vector2.Dot(body._localCenter, body._localCenter);
            DebugTools.Assert(body._inertia > 0.0f);
            body.InvI = 1.0f / body._inertia;
            // Not networked
        }
    }

    [Obsolete("Use overload that takes EntityUid")]
    public void SetInertia(PhysicsComponent body, float value, bool dirty = true)
    {
        SetInertia(body.Owner, body, value, dirty);
    }

    public void SetLocalCenter(EntityUid uid, PhysicsComponent body, Vector2 value)
    {
        if (body.BodyType != BodyType.Dynamic) return;

        if (value.EqualsApprox(body._localCenter)) return;

        body._localCenter = value;
        // Not networked
    }

    public void SetSleepingAllowed(EntityUid uid, PhysicsComponent body, bool value, bool dirty = true)
    {
        if (body.SleepingAllowed == value)
            return;

        if (!value)
            SetAwake((uid, body), true);

        body.SleepingAllowed = value;
        DirtyField(uid, body, nameof(PhysicsComponent.SleepingAllowed));
    }

    public void SetSleepTime(PhysicsComponent body, float value)
    {
        DebugTools.Assert(!float.IsNaN(value));

        if (MathHelper.CloseToPercent(value, body.SleepTime))
            return;

        body.SleepTime = value;
    }

    internal void SleepBody(Entity<PhysicsComponent> entity)
    {
        SetAwake();
    }

    /// <summary>
    /// Tries to enable the body and also set it awake.
    /// </summary>
    /// <param name="force">Bypasses fixture and container checks</param>
    /// <returns>true if the body is collidable and awake</returns>
    public bool WakeBody(EntityUid uid, bool force = false, PhysicsComponent? body = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body))
            return false;

        if (!SetCanCollide(uid, true, body: body, force: force))
            return false;

        SetAwake((uid, body), true);
        return body.Awake;
    }

    #endregion

    public Transform GetPhysicsTransform(EntityUid uid, TransformComponent? xform = null)
    {
        if (!_xformQuery.Resolve(uid, ref xform))
            return Physics.Transform.Empty;

        var (worldPos, worldRot) = _transform.GetWorldPositionRotation(xform);

        return new Transform(worldPos, worldRot);
    }

    /// <summary>
    /// Gets the physics World AABB, only considering fixtures.
    /// </summary>
    public Box2 GetWorldAABB(EntityUid uid, PhysicsComponent? body = null, TransformComponent? xform = null)
    {
        if (!Resolve(uid, ref body, ref xform))
            return new Box2();

        var (worldPos, worldRot) = _transform.GetWorldPositionRotation(xform);

        var transform = new Transform(worldPos, (float) worldRot.Theta);

        var bounds = new Box2(transform.Position, transform.Position);

        foreach (var fixture in body.Fixtures.Values)
        {
            for (var i = 0; i < fixture.Shape.ChildCount; i++)
            {
                var boundy = fixture.Shape.ComputeAABB(transform, i);
                bounds = bounds.Union(boundy);
            }
        }

        return bounds;
    }

    public Box2 GetHardAABB(EntityUid uid, PhysicsComponent? body = null, TransformComponent? xform = null)
    {
        if (!PhysicsQuery.Resolve(uid, ref body)
            || !PhysicsQuery.Resolve(uid, ref body)
            || !Resolve(uid, ref xform))
        {
            return Box2.Empty;
        }

        var (worldPos, worldRot) = _transform.GetWorldPositionRotation(xform);

        var transform = new Transform(worldPos, (float) worldRot.Theta);

        var bounds = new Box2(transform.Position, transform.Position);

        foreach (var fixture in body.Fixtures.Values)
        {
            if (!fixture.Hard) continue;

            for (var i = 0; i < fixture.Shape.ChildCount; i++)
            {
                var boundy = fixture.Shape.ComputeAABB(transform, i);
                bounds = bounds.Union(boundy);
            }
        }

        return bounds;
    }

    public (int Layer, int Mask) GetHardCollision(Entity<PhysicsComponent?> entity)
    {
        if (!PhysicsQuery.Resolve(entity.Owner, ref entity.Comp, false))
        {
            return (0, 0);
        }

        var layer = 0;
        var mask = 0;

        foreach (var fixture in entity.Comp.Fixtures.Values)
        {
            if (!fixture.Hard)
                continue;

            layer |= fixture.CollisionLayer;
            mask |= fixture.CollisionMask;
        }

        return (layer, mask);
    }

    public virtual void UpdateIsPredicted(EntityUid? uid, PhysicsComponent? physics = null)
    {
        // See client-side system
    }
}
