package delaunay

type Edge struct {
	p *Point
	q *Point
}

func NewEdge(p, q *Point) *Edge {
	e := &Edge{}
	e.init(p, q)

	return e
}

func (e *Edge) init(p1, p2 *Point) {
	e.p = p1
	e.q = p2

	if p1.Y > p2.Y {
		e.q = p1
		e.p = p2
	} else if p1.Y == p2.Y {
		if p1.X > p2.X {
			e.q = p1
			e.p = p2
		} else if p1.X == p2.X {
			panic("poly2tri Invalid Edge constructor: repeated points!")
		}
	}

	if e.q.edgesList == nil {
		e.q.edgesList = make([]*Edge, 0)
	}
	e.q.edgesList = append(e.q.edgesList, e)
}
func (e *Edge) hasPoint(point *Point) bool {
	return (e.p.X == point.X && e.p.Y == point.Y) || (e.q.X == point.X && e.q.Y == point.Y)
}
