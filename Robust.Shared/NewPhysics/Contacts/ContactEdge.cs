namespace Robust.Shared.NewPhysics;

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// edges, one for each attached body.
internal sealed class b2ContactEdge
{
    int bodyId;
    int prevKey;
    int nextKey;
}
