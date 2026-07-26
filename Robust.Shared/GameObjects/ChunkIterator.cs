using Arch.Core;

namespace Robust.Shared.GameObjects;

internal readonly struct ArchChunkIterator
{
    private readonly Query _query;

    internal ArchChunkIterator(Query query)
    {
        _query = query;
    }

    public ArchChunkEnumerator GetEnumerator()
    {
        return new ArchChunkEnumerator(_query);
    }
}

internal struct ArchChunkEnumerator
{
    private readonly Query _query;
    private int _archetypeIndex;
    private int _chunkIndex;
    private Archetype? _archetype;
    private Chunk _current;

    public Chunk Current => _current;
    public Archetype CurrentArchetype => _archetype!;

    internal ArchChunkEnumerator(Query query)
    {
        _query = query;
        _archetypeIndex = query.Matches.Count;
        _chunkIndex = 0;
        _archetype = null;
        _current = default;
    }

    public bool MoveNext()
    {
        while (true)
        {
            while (_archetype != null && --_chunkIndex >= 0)
            {
                var chunk = _archetype.GetChunk(_chunkIndex);
                if (chunk.Count == 0)
                    continue;

                _current = chunk;
                return true;
            }

            if (--_archetypeIndex < 0)
                return false;

            _archetype = _query.Matches.Span[_archetypeIndex];
            if (_archetype.EntityCount == 0)
                continue;

            _chunkIndex = _archetype.ChunkCount;
        }
    }
}

internal static partial class QueryExtensions
{
    internal static ArchChunkIterator ChunkIterator(this World world, in QueryDescription queryDescription)
    {
        var query = world.Query(queryDescription);
        return new ArchChunkIterator(query);
    }
}
