namespace Robust.Shared.NewPhysics;

/// Contact id references a contact instance. This should be treated as an opaque handled.
public record struct ContactId
{
    int index1;
    short padding;
}
