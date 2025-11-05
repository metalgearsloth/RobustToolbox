using System.Collections.Generic;
using Robust.Shared.Collections;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Islands;
using Robust.Shared.NewPhysics.Joints;

namespace Robust.Shared.NewPhysics;

// This holds solver set data. The following sets are used:
// - static set for all static bodies and joints between static bodies
// - active set for all active bodies with body states (no contacts or joints)
// - disabled set for disabled bodies and their joints
// - all further sets are sleeping island sets along with their contacts and joints
// The purpose of solver sets is to achieve high memory locality.
// https://www.youtube.com/watch?v=nZNd5FjSquk
internal sealed class SolverSet
{
    // Bodies. Empty for unused set.
    public ValueList<BodySim> bodySims = new();

    // Body state only exists for active set
    public ValueList<BodyState> bodyStates = new();

    // This holds sleeping/disabled joints. Empty for static/active set.
    public ValueList<JointSim> jointSims = new();

    // This holds all contacts for sleeping sets.
    // This holds non-touching contacts for the awake set.
    public ValueList<ContactSim> contactSims = new();

    // The awake set has an array of islands. Sleeping sets normally have a single islands. However, joints
    // created between sleeping sets causes the sets to merge, leaving them with multiple islands. These sleeping
    // islands will be naturally merged with the set is woken.
    // The static and disabled sets have no islands.
    // Islands live in the solver sets to limit the number of islands that need to be considered for sleeping.
    public ValueList<IslandSim> islandSims = new();

    // Aligns with b2World::solverSetIdPool. Used to create a stable id for body/contact/joint/islands.
    public int setIndex;
}
