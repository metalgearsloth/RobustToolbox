﻿using System;
using Robust.Client.Graphics;
using Robust.Client.UserInterface.Controls;
using Robust.Shared;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Client.UserInterface
{
    public interface IUserInterfaceManager
    {
        UITheme ThemeDefaults { get; }

        /// <summary>
        ///     Default style sheet that applies to all controls
        ///     that do not have a more specific style sheet via <see cref="Control.Stylesheet"/>.
        /// </summary>
        Stylesheet? Stylesheet { get; set; }

        /// <summary>
        /// A control can have "keyboard focus" separate from ControlFocused, obtained when calling
        /// Control.GrabKeyboardFocus. Corresponding events in Control are KeyboardFocusEntered/Exited
        /// </summary>
        Control? KeyboardFocused { get; }

        /// <summary>
        /// A control gets "ControlFocused" when a mouse button (or any KeyBinding which has CanFocus = true) is
        /// pressed down on the control. While it is focused, it will receive mouse hover events and the corresponding
        /// keyup event if it still has focus when that occurs (it will NOT receive the keyup if focus has
        /// been taken by another control). Focus is removed when a different control takes focus
        /// (such as by pressing a different mouse button down over a different control) or when the keyup event
        /// happens. When focus is lost on a control, it always fires Control.ControlFocusExited.
        /// </summary>
        Control? ControlFocused { get; }

        LayoutContainer StateRoot { get; }

        LayoutContainer WindowRoot { get; }

        LayoutContainer PopupRoot { get; }

        PopupContainer ModalRoot { get; }

        Control? CurrentlyHovered { get; }

        float UIScale { get; }

        /// <summary>
        ///     Gets the default UIScale that we will use if <see cref="CVars.DisplayUIScale"/> gets set to 0.
        ///     Based on the OS-assigned window scale factor.
        /// </summary>
        float DefaultUIScale { get; }

        /// <summary>
        ///     The "root" control to which all other controls are parented,
        ///     potentially indirectly.
        /// </summary>
        Control RootControl { get; }

        IDebugMonitors DebugMonitors { get; }

        void Popup(string contents, string title = "Alert!");

        Control? MouseGetControl(Vector2 coordinates);

        /// <summary>
        ///     Gets the mouse position in UI space, accounting for <see cref="UIScale"/>.
        /// </summary>
        Vector2 MousePositionScaled { get; }

        Vector2 ScreenToUIPosition(Vector2 position);
        Vector2 ScreenToUIPosition(ScreenCoordinates coordinates);

        /// <summary>
        ///     Give a control keyboard focus, releasing focus on the currently focused control (if any).
        /// </summary>
        /// <param name="control">The control to give keyboard focus to.</param>
        /// <exception cref="ArgumentNullException">Thrown if <see cref="control"/> is <c>null</c>.</exception>
        /// <exception cref="ArgumentException">
        ///     Thrown if <see cref="control"/> has <see cref="Control.CanKeyboardFocus"/> <c>false</c>.
        /// </exception>
        void GrabKeyboardFocus(Control control);

        /// <summary>
        ///     Release keyboard focus from the currently focused control, if any.
        /// </summary>
        void ReleaseKeyboardFocus();

        /// <summary>
        ///     Conditionally release keyboard focus if <see cref="ifControl"/> has keyboard focus.
        /// </summary>
        /// <exception cref="ArgumentNullException">
        ///     Thrown if <see cref="ifControl"/> is <c>null</c>.
        /// </exception>
        /// <seealso cref="ReleaseKeyboardFocus()"/>
        void ReleaseKeyboardFocus(Control ifControl);

        /// <summary>
        ///     Cursor automatically used when the mouse is not over any UI control.
        /// </summary>
        ICursor? WorldCursor { get; set; }

        void PushModal(Control modal);
    }
}