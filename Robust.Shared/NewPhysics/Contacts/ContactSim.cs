using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Dynamics;

namespace Robust.Shared.NewPhysics;

// Class because it needs to be combined together into a list for the parallel collision job.
internal sealed class ContactSim()
{
    // Don't use a ref here so we can use it for bitset operations.
    public int contactId;

	public int bodyIdA;
	public int bodyIdB;

    // Transient body indices
    public int bodySimIndexA;
    public int bodySimIndexB;

    public Fixture shapeA;
    public Fixture shapeB;

    public float invMassA;
    public float invIA;

    public float invMassB;
    public float invIB;

    public b2Manifold manifold;

    // Mixed friction and restitution
    public float friction;
    public float restitution;
    public float rollingResistance;
    public float tangentSpeed;

    public ContactSimFlags simFlags;

    internal SimplexCache cache = new();
}
