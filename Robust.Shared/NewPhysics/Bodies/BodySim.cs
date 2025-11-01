using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;

namespace Robust.Shared.NewPhysics.Bodies;

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
internal record struct BodySim
{
    // transform for body origin
    public Transform transform;

    // center of mass position in world space
    public Vector2 center;

    // previous rotation and COM for TOI
    public Quaternion2D rotation0;
    public Vector2 center0;

    // location of center of mass relative to the body origin
    public Vector2 localCenter;

    public Vector2 force;
    public float torque;

    // inverse inertia
    public float invMass;
    public float invInertia;

    public float minExtent;
    public float maxExtent;
    public float linearDamping;
    public float angularDamping;
    public float gravityScale;

    public PhysicsComponent body;

    // b2BodyFlags
    public uint flags;
}
