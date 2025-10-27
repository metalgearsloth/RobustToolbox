namespace Robust.Shared.NewPhysics;

/// An end touch event is generated when two shapes stop touching.
///	You will get an end event if you do anything that destroys contacts previous to the last
///	world step. These include things like setting the transform, destroying a body
///	or shape, or changing a filter or body type.
public record struct ContactEndTouchEvent
{
    /// Id of the first shape
    ///	@warning this shape may have been destroyed
    ///	@see b2Shape_IsValid
    ShapeId shapeIdA;

    /// Id of the second shape
    ///	@warning this shape may have been destroyed
    ///	@see b2Shape_IsValid
    ShapeId shapeIdB;

    /// Id of the contact.
    ///	@warning this contact may have been destroyed
    ///	@see b2Contact_IsValid
    ContactId contactId;
}
