using System;
using System.Runtime.CompilerServices;

namespace Robust.Shared.GameObjects;

internal sealed partial class EntityEventBus
{
    internal sealed class EntityEventTableStorage
    {
        private const int PageBits = 12;
        private const int PageSize = 1 << PageBits;
        private const int PageMask = PageSize - 1;

        private EventTable?[][] _pages = [];

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public EventTable? Get(EntityUid uid)
        {
            var id = uid.Id;
            var pageIndex = id >> PageBits;

            if ((uint) pageIndex >= (uint) _pages.Length)
                return null;

            var page = _pages[pageIndex];
            return page == null ? null : page[id & PageMask];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ref EventTable? GetOrCreateSlot(EntityUid uid)
        {
            var id = uid.Id;
            var pageIndex = id >> PageBits;

            if ((uint) pageIndex >= (uint) _pages.Length)
                GrowPages(pageIndex + 1);

            var page = _pages[pageIndex];
            if (page == null)
            {
                page = new EventTable?[PageSize];
                _pages[pageIndex] = page;
            }

            return ref page[id & PageMask];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Remove(EntityUid uid)
        {
            var id = uid.Id;
            var pageIndex = id >> PageBits;

            if ((uint) pageIndex >= (uint) _pages.Length)
                return;

            var page = _pages[pageIndex];
            if (page != null)
                page[id & PageMask] = null;
        }

        public void Clear()
        {
            _pages = [];
        }

        private void GrowPages(int minSize)
        {
            var oldLength = _pages.Length;
            var newLength = Math.Max(minSize, Math.Max(4, oldLength * 2));
            Array.Resize(ref _pages, newLength);
        }
    }
}
