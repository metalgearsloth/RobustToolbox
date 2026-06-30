using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using BenchmarkDotNet.Attributes;
using Robust.Shared.Analyzers;
using Robust.Shared.Collections;

namespace Robust.Benchmarks.Collections;

[Virtual, MemoryDiagnoser]
public class ValueListBenchmarks
{
    [Params(4, 16, 64)]
    public int N { get; set; }

    private sealed class Data(int i)
    {
        public readonly int I = i;
    }

    private ValueList<Data> _valueList;
    private List<Data> _list = default!;
    private Data[] _source = default!;

    [GlobalSetup]
    public void Setup()
    {
        var list = new List<Data>(N);
        for (var i = 0; i < N; i++)
        {
            list.Add(new(i));
        }

        _source = list.ToArray();
        _list = list;
        _valueList = new(list);
    }

    [Benchmark(Baseline = true)]
    public int ListEnumerate()
    {
        var total = 0;
        foreach (var ev in _list)
        {
            total += ev.I;
        }

        return total;
    }

    [Benchmark]
    public int ValueListEnumerate()
    {
        var total = 0;
        foreach (var ev in _valueList)
        {
            total += ev.I;
        }

        return total;
    }

    [Benchmark]
    public int ListCollectionsMarshalAsSpanEnumerate()
    {
        var total = 0;
        foreach (var ev in CollectionsMarshal.AsSpan(_list))
        {
            total += ev.I;
        }

        return total;
    }

    [Benchmark]
    public int ValueListSpanEnumerate()
    {
        var total = 0;
        foreach (var ev in _valueList.Span)
        {
            total += ev.I;
        }

        return total;
    }

    [Benchmark]
    public int ListIndexed()
    {
        var total = 0;
        var list = _list;
        for (var i = 0; i < list.Count; i++)
        {
            total += list[i].I;
        }

        return total;
    }

    [Benchmark]
    public int ValueListIndexed()
    {
        var total = 0;
        var list = _valueList;
        for (var i = 0; i < list.Count; i++)
        {
            total += list[i].I;
        }

        return total;
    }

    [Benchmark]
    public int ListCollectionsMarshalAsSpanSum()
    {
        var total = 0;
        var span = CollectionsMarshal.AsSpan(_list);
        for (var i = 0; i < span.Length; i++)
        {
            total += span[i].I;
        }

        return total;
    }

    [Benchmark]
    public int ValueListAsSpanSum()
    {
        var total = 0;
        var span = _valueList.Span;
        for (var i = 0; i < span.Length; i++)
        {
            total += span[i].I;
        }

        return total;
    }

    [Benchmark]
    public int ListAddPreallocated()
    {
        var list = new List<Data>(N);
        foreach (var data in _source)
        {
            list.Add(data);
        }

        return list.Count;
    }

    [Benchmark]
    public int ValueListAddPreallocated()
    {
        var list = new ValueList<Data>(N);
        foreach (var data in _source)
        {
            list.Add(data);
        }

        return list.Count;
    }

    [Benchmark]
    public int ListAddGrow()
    {
        var list = new List<Data>();
        foreach (var data in _source)
        {
            list.Add(data);
        }

        return list.Count;
    }

    [Benchmark]
    public int ValueListAddGrow()
    {
        var list = new ValueList<Data>();
        foreach (var data in _source)
        {
            list.Add(data);
        }

        return list.Count;
    }

    [Benchmark]
    public int ListClear()
    {
        var list = new List<Data>(_source);
        list.Clear();
        return list.Count;
    }

    [Benchmark]
    public int ValueListClear()
    {
        var list = new ValueList<Data>(_source);
        list.Clear();
        return list.Count;
    }

    [Benchmark]
    public int ListAddRange()
    {
        var list = new List<Data>(N);
        list.AddRange(_source);
        return list.Count;
    }

    [Benchmark]
    public int ValueListAddRange()
    {
        var list = new ValueList<Data>(N);
        list.AddRange(_source);
        return list.Count;
    }
}
