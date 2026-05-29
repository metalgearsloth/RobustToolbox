using Arch.Core;

namespace Robust.Shared.GameObjects;

internal readonly struct ArchChunkIterator
{
    private readonly Chunk[] _chunks;

    internal ArchChunkIterator(Chunk[] chunks)
    {
        _chunks = chunks;
    }

    public ArchChunkEnumerator GetEnumerator()
    {
        return new ArchChunkEnumerator(_chunks);
    }
}

internal struct ArchChunkEnumerator
{
    private readonly Chunk[] _chunks;
    private int _index;

    public Chunk Current => _chunks[_index];

    internal ArchChunkEnumerator(Chunk[] chunks)
    {
        _chunks = chunks;
        _index = -1;
    }

    public bool MoveNext()
    {
        while (++_index < _chunks.Length)
        {
            if (Current.Count > 0)
                return true;
        }

        return false;
    }
}

internal static partial class QueryExtensions
{
    internal static ArchChunkIterator ChunkIterator(this World world, in QueryDescription queryDescription)
    {
        var query = world.Query(queryDescription);
        var count = 0;

        foreach (ref var chunk in query)
        {
            if (chunk.Count > 0)
                count++;
        }

        if (count == 0)
            return new ArchChunkIterator([]);

        var chunks = new Chunk[count];
        var index = 0;

        foreach (ref var chunk in query)
        {
            if (chunk.Count > 0)
                chunks[index++] = chunk;
        }

        return new ArchChunkIterator(chunks);
    }
}
