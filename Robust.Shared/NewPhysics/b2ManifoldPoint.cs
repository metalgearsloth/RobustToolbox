using System.Numerics;

namespace Robust.Shared.NewPhysics;

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// You may use the totalNormalImpulse to determine if there was an interaction during
/// the time step.
public record struct b2ManifoldPoint
{
    /// Location of the contact point in world space. Subject to precision loss at large coordinates.
    /// @note Should only be used for debugging.
    Vector2 point;

    /// Location of the contact point relative to shapeA's origin in world space
    /// @note When used internally to the Box2D solver, this is relative to the body center of mass.
    Vector2 anchorA;

    /// Location of the contact point relative to shapeB's origin in world space
    /// @note When used internally to the Box2D solver, this is relative to the body center of mass.
    Vector2 anchorB;

    /// The separation of the contact point, negative if penetrating
    float separation;

    /// The impulse along the manifold normal vector.
    float normalImpulse;

    /// The friction impulse
    float tangentImpulse;

    /// The total normal impulse applied across sub-stepping and restitution. This is important
    /// to identify speculative contact points that had an interaction in the time step.
    float totalNormalImpulse;

    /// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
    /// zero then there was no hit. Negative means shapes are approaching.
    float normalVelocity;

    /// Uniquely identifies a contact point between two shapes
    short id;

    /// Did this contact point exist the previous step?
    bool persisted;
}
