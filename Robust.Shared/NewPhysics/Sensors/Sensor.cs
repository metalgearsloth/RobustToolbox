using Robust.Shared.Collections;

namespace Robust.Shared.NewPhysics.Sensors;

internal sealed class Sensor
{
    // todo find a way to pool these
    public ValueList<Visitor> hits;
    public ValueList<Visitor> overlaps1;
    public ValueList<Visitor> overlaps2;
    public int shapeId;
}
