using System.Diagnostics.Contracts;
using Robust.Shared.Physics.Dynamics;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    /// <summary>
    /// Returns whether these 2 fixtures have a contact between them.
    /// </summary>
    [Pure]
    public bool HasContact(Fixture fixtureA, Fixture fixtureB)
    {
        if (fixtureB.Id < fixtureA.Id)
        {
            (fixtureA, fixtureB) = (fixtureB, fixtureA);
        }

        var pairKey = B2_SHAPE_PAIR_KEY(fixtureA.Id, fixtureB.Id);
        return _pairSet.Contains(pairKey);
    }
}
