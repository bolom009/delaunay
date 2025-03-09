package delaunay

type Node struct {
	point    *Point
	triangle *Triangle
	next     *Node
	prev     *Node
	value    float32
}

func NewNode(p *Point, t *Triangle) *Node {
	return &Node{
		point:    p,
		triangle: t,
		next:     nil,
		prev:     nil,
		value:    p.X,
	}
}
