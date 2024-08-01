using System;
using CommunityToolkit.HighPerformance.Buffers;

namespace Robust.Shared.GameObjects;

public sealed class StringManager
{
    private StringPool _pool = new();

    internal string Intern(ReadOnlySpan<char> chars)
    {
        return _pool.GetOrAdd(chars);
    }

    public string Concat(params string[] strings)
    {
        return _pool.GetOrAdd(string.Concat(strings));
    }

    /// <summary>
    /// Tries to return the interned string for the specified text.
    /// </summary>
    public string Intern(string text)
    {
        return _pool.GetOrAdd(text);
    }
}
