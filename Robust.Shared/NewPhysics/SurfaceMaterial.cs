using System;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics;

/// Surface materials allow chain shapes to have per segment surface properties.
[DataDefinition]
public partial record struct SurfaceMaterial
{
    /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
    [DataField]
    public float Friction;

    /// The coefficient of restitution (bounce) usually in the range [0,1].
    /// https://en.wikipedia.org/wiki/Coefficient_of_restitution
    [DataField]
    public float Restitution;

    /// The rolling resistance usually in the range [0,1].
    [DataField]
    public float RollingResistance;

    /// The tangent speed for conveyor belts
    [DataField]
    public float TangentSpeed;

    /// User material identifier. This is passed with query results and to friction and restitution
    /// combining functions. It is not used internally.
    [DataField]
    public ulong UserMaterialId;

    /// Custom debug draw color.
    [DataField]
    public uint CustomColor;

    public SurfaceMaterial(SurfaceMaterial other)
    {
        Friction = other.Friction;
        Restitution = other.Restitution;
        RollingResistance = other.RollingResistance;
        TangentSpeed = other.TangentSpeed;
        UserMaterialId = other.UserMaterialId;
        CustomColor = other.CustomColor;
    }

    public bool Equals(SurfaceMaterial other)
    {
        return Friction.Equals(other.Friction) &&
               Restitution.Equals(other.Restitution) &&
               RollingResistance.Equals(other.RollingResistance) &&
               TangentSpeed.Equals(other.TangentSpeed) &&
               UserMaterialId.Equals(other.UserMaterialId) &&
               CustomColor.Equals(other.CustomColor);
    }

    public readonly override int GetHashCode()
    {
        return HashCode.Combine(Friction, Restitution, RollingResistance, TangentSpeed, UserMaterialId, CustomColor);
    }
}
