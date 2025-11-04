namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void DispatchEvents(float frameTime)
    {
        // Check grid traversals in parallel, this is so we don't need to give out a redundant MoveEvent.

        // Move events
        foreach (var ev in _bodyMoveEvents)
        {
            // Store InvWorldMatrix on event maybe?

            // Run traversal if relevant

            // Update broadphase

            // Dispatch event
            // REMEMBER TO APPLY THE OFFSET AND NOT ABSOLUTE VALUES
        }

        // Contact events
        // It's fine not to double-buffer these as contacts are only handled at the start of the physics step.
        foreach (var ev in _contactBeginEvents)
        {

        }

        // Where the double-buffer comes in handy because if a caller destroys the contact it doesn't mutate this list.
        foreach (var ev in _contactEndEvents[1 - _endEventArrayIndex])
        {

        }

        foreach (var ev in _sensorBeginEvents)
        {

        }

        foreach (var ev in _sensorEndEvents[1 - _endEventArrayIndex])
        {

        }
    }
}
