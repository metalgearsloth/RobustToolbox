using System;
using System.Collections.Generic;
using Robust.Shared.Input;
using Robust.Shared.Players;
using Robust.Shared.Timing;

namespace Robust.Server.GameObjects;

public sealed partial class InputSystem
{
    private readonly Dictionary<ICommonSession, InputValidator> _lastValidatedInputs = new();

    /// <summary>
    /// Raised if a session fails input validation.
    /// </summary>
    public event Action<ICommonSession>? InputValidationFailed;

    /// <summary>
    /// How many inputs we validate for.
    /// </summary>
    private int _validationLength;

    private bool ValidateInput(ICommonSession session, GameTick curTick, KeyFunctionId function)
    {
        // HUH
        if (!_lastValidatedInputs.TryGetValue(session, out var inputs))
            return false;

        // TODO: Check if there's too many over N period.
        // TODO: Check if the tick timing between all of them is equal.

        if (false)
        {
            InputValidationFailed?.Invoke(session);
        }

        return true;
    }

    private sealed class InputValidator
    {
        // Ring buffer
        public int CurrentIndex;

        public ValidatedInput[] _inputs = Array.Empty<ValidatedInput>();
    }

    private record struct ValidatedInput
    {
        public GameTick Tick;
        public List<KeyFunctionId> Functions;
    }
}
