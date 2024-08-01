using CommunityToolkit.HighPerformance.Buffers;

namespace Robust.Shared.GameObjects;

public sealed class StringManager
{
    private StringPool _pool = new();

    public static void Initialize()
    {
    }

    /// <summary>
    /// Tries to return the interned string for the specified text.
    /// </summary>
    public string Intern(string text)
    {
        return _pool.GetOrAdd(text);
    }
}
