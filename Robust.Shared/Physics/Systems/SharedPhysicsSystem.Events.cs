namespace Robust.Shared.Physics.Systems;

public abstract partial class SharedPhysicsSystem
{
    private void ClearEvents()
    {
        _startCollideEvents.Clear();
        _endCollideEvents[1 - _endEventIndex].Clear();
    }

    private void DispatchEvents()
    {
        // Raises all the buffered events once physics step is done.
        foreach (var ev in _startCollideEvents)
        {
            var elem = ev;
            RaiseLocalEvent(ev.OurEntity, ref elem);
        }

        foreach (var ev in _endCollideEvents[1 - _endEventIndex])
        {
            var elem = ev;
            RaiseLocalEvent(ev.OurEntity, ref elem);
        }

        _endEventIndex = 1 - _endEventIndex;
    }
}
