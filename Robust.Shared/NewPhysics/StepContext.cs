using System.Collections.Generic;
using Robust.Shared.NewPhysics.Bodies;

namespace Robust.Shared.NewPhysics;

// Context for a time step. Recreated each time step.
internal sealed class StepContext
{
    // time step
    public float dt;

    // inverse time step (0 if dt == 0).
    public float inv_dt;

    // sub-step
    public float h;
    public float inv_h;

    public int subStepCount;

    public Softness contactSoftness;
    public Softness staticSoftness;

    public float restitutionThreshold;
    public float maxLinearVelocity;

    public bool enableWarmStarting;

    // contact pointers for simplified parallel-for access.
    // - parallel-for collide with no gaps
    // - parallel-for prepare and store contacts with NULL gaps for SIMD remainders
    // despite being an array of pointers, these are contiguous sub-arrays corresponding
    // to constraint graph colors
    public List<ContactSim> contacts;

    public int BulletBodyCount;

    public int[] BulletBodies;

    public List<BodySim> sims;
    public List<BodyState> states;
}
