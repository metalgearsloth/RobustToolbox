// Copyright (c) 2017 Kastellanos Nikolaos

/* Original source Farseer Physics Engine:
 * Copyright (c) 2014 Ian Qvist, http://farseerphysics.codeplex.com
 * Microsoft Permissive License (Ms-PL) v1.1
 */

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
*/

using System;
using Robust.Shared.GameObjects;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Serialization;

namespace Robust.Shared.Physics.Dynamics.Contacts;

[Serializable, NetSerializable]
public sealed class Contact : IEquatable<Contact>
{
    /*
     * Class so we can store a ref on each component.
     * Just means state networking needs to handle synchronising it across the 2 entities.
     * Joints also have this problem
     */

    #region Networked

    public EntityUid EntityA { get; internal set; }
    public EntityUid EntityB { get; internal set; }

    public string FixtureAId { get; internal set; } = string.Empty;
    public string FixtureBId { get; internal set; } = string.Empty;

    internal ContactType Type { get; set; }

    /// <summary>
    ///     Determines whether the contact is touching.
    /// </summary>
    public bool IsTouching { get; internal set; }

    /// <summary>
    ///     The mixed friction of the 2 fixtures.
    /// </summary>
    public float Friction { get; internal set; }

    /// <summary>
    ///     The mixed restitution of the 2 fixtures.
    /// </summary>
    public float Restitution { get; internal set; }

    /// <summary>
    ///     Used for conveyor belt behavior in m/s.
    /// </summary>
    public float TangentSpeed { get; internal set; }

    #endregion

    [NonSerialized]
    public Manifold Manifold;

    [NonSerialized]
    internal ContactFlags Flags = ContactFlags.None;

    /// Enable/disable this contact. This can be used inside the pre-solve
    /// contact listener. The contact is only disabled for the current
    /// time step (or sub-step in continuous collisions).
    [NonSerialized]
    public bool Enabled;

    internal Contact(IManifoldManager manifoldManager)
    {
    }

    public bool Equals(Contact? other)
    {
        if (ReferenceEquals(null, other)) return false;
        if (ReferenceEquals(this, other)) return true;
        return Equals(FixtureAId, other.FixtureAId) &&
               Equals(FixtureBId, other.FixtureBId) &&
               EntityA.Equals(other.EntityA) &&
               EntityB.Equals(other.EntityB) &&
               Type == other.Type;
    }

    public override bool Equals(object? obj)
    {
        return ReferenceEquals(this, obj) || obj is Contact other && Equals(other);
    }

    public override int GetHashCode()
    {
        var hashCode = new HashCode();
        hashCode.Add(EntityA);
        hashCode.Add(EntityB);
        hashCode.Add(FixtureAId);
        hashCode.Add(FixtureBId);
        hashCode.Add(Type);

        return hashCode.ToHashCode();
    }
}

[Serializable, NetSerializable]
public enum ContactType : byte
{
    NotSupported,
    Polygon,
    PolygonAndCircle,
    Circle,
    EdgeAndPolygon,
    EdgeAndCircle,
    ChainAndPolygon,
    ChainAndCircle,
}

[Flags]
internal enum ContactFlags : byte
{
    None = 0,

    /// <summary>
    /// Is the contact pending its first manifold generation.
    /// </summary>
    PreInit = 1 << 0,

    /// <summary>
    ///     Has this contact already been added to an island?
    /// </summary>
    Island = 1 << 1,

    /// <summary>
    ///     Does this contact need re-filtering?
    /// </summary>
    Filter = 1 << 2,

    /// <summary>
    /// Is this a special contact for grid-grid collisions
    /// </summary>
    Grid = 1 << 3,

    /// <summary>
    /// Set right before the contact is deleted
    /// </summary>
    Deleting = 1 << 4,
}
