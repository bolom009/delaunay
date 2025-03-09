package delaunay

type Basin struct {
	leftNode    *Node
	bottomNode  *Node
	rightNode   *Node
	width       float32
	leftHighest bool
}
