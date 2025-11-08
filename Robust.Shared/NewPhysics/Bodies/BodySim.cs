using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;

namespace Robust.Shared.NewPhysics.Bodies;

// Body simulation data used for integration of position and velocity
// Transform data used for collision and solver preparation.
internal record struct BodySim
{
    // transform for body origin
    public Transform Transform;

    // center of mass position in world space
    public Vector2 Center;

    // previous rotation and COM for TOI
    public Quaternion2D Rotation0;
    public Vector2 Center0;

    // location of center of mass relative to the body origin
    public Vector2 LocalCenter;

    public Vector2 Force;
    public float Torque;

    // inverse inertia
    public float InvMass;
    public float InvInertia;

    public float MinExtent;
    public float MaxExtent;
    public float LinearDamping;
    public float AngularDamping;
    public float GravityScale;

    public PhysicsComponent Body;

    public BodyFlags Flags;
}
