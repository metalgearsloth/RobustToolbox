using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Shapes;

/// <summary>
/// Like <see cref="FixedArray4{T}"/> but for vertices and physics purposes
/// </summary>
[Serializable, NetSerializable]
[DataRecord]
public record struct FixedVertArray4
{
    internal Vector2 _00;
    internal Vector2 _01;
    internal Vector2 _02;
    internal Vector2 _03;

    internal Span<Vector2> AsSpan => MemoryMarshal.CreateSpan(ref _00, 4);

    public bool Equals(FixedVertArray4 other)
    {
        return _00.Equals(other._00) &&
               _01.Equals(other._01) &&
               _02.Equals(other._02) &&
               _03.Equals(other._03);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(_00, _01, _02, _03);
    }
}

/// <summary>
/// Like <see cref="FixedArray8{T}"/> but for vertices and physics purposes
/// </summary>
[Serializable, NetSerializable]
[DataRecord]
public record struct FixedVertArray8
{
    internal Vector2 _00;
    internal Vector2 _01;
    internal Vector2 _02;
    internal Vector2 _03;
    internal Vector2 _04;
    internal Vector2 _05;
    internal Vector2 _06;
    internal Vector2 _07;

    internal Span<Vector2> AsSpan => MemoryMarshal.CreateSpan(ref _00, 8);

    public bool Equals(FixedVertArray8 other)
    {
        return _00.Equals(other._00) &&
               _01.Equals(other._01) &&
               _02.Equals(other._02) &&
               _03.Equals(other._03) &&
               _04.Equals(other._04) &&
               _05.Equals(other._05) &&
               _06.Equals(other._06) &&
               _07.Equals(other._07);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(_00, _01, _02, _03, _04, _05, _06, _07);
    }
}
