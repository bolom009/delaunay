package delaunay

type Triangle struct {
	points          []*Point
	neighbors       []*Triangle
	interior        bool
	constrainedEdge []bool
	delaunayEdge    []bool
}

func NewTriangle(p1, p2, p3 *Point) *Triangle {
	return &Triangle{
		points:          []*Point{p1, p2, p3},
		neighbors:       []*Triangle{nil, nil, nil},
		interior:        false,
		constrainedEdge: []bool{false, false, false},
		delaunayEdge:    []bool{false, false, false},
	}
}

func (t *Triangle) GetPoints() []*Point {
	return t.points
}

func (t *Triangle) containsPoint(point *Point) bool {
	return point == t.points[0] || point == t.points[1] || point == t.points[2]
}

func (t *Triangle) containsEdge(edge *Edge) bool {
	return t.containsPoint(edge.p) && t.containsPoint(edge.q)
}

func (t *Triangle) containsPoints(p1, p2 *Point) bool {
	return t.containsPoint(p1) && t.containsPoint(p2)
}

func (t *Triangle) markNeighborPointers(p1 *Point, p2 *Point, o *Triangle) {
	points := t.points
	// Here we are comparing point references, not values
	if (p1 == points[2] && p2 == points[1]) || (p1 == points[1] && p2 == points[2]) {
		t.neighbors[0] = o
	} else if (p1 == points[0] && p2 == points[2]) || (p1 == points[2] && p2 == points[0]) {
		t.neighbors[1] = o
	} else if (p1 == points[0] && p2 == points[1]) || (p1 == points[1] && p2 == points[0]) {
		t.neighbors[2] = o
	} else {
		panic("poly2tri Invalid Triangle.markNeighborPointers() call")
	}
}
func (t *Triangle) markNeighbor(o *Triangle) {
	points := t.points
	if o.containsPoints(points[1], points[2]) {
		t.neighbors[0] = o
		o.markNeighborPointers(points[1], points[2], t)
	} else if o.containsPoints(points[0], points[2]) {
		t.neighbors[1] = o
		o.markNeighborPointers(points[0], points[2], t)
	} else if o.containsPoints(points[0], points[1]) {
		t.neighbors[2] = o
		o.markNeighborPointers(points[0], points[1], t)
	}
}

func (t *Triangle) clearNeighbors() {
	t.neighbors[0] = nil
	t.neighbors[1] = nil
	t.neighbors[2] = nil
}

func (t *Triangle) clearDelaunayEdges() {
	t.delaunayEdge[0] = false
	t.delaunayEdge[1] = false
	t.delaunayEdge[2] = false
}

func (t *Triangle) pointCW(p *Point) *Point {
	points := t.points
	// Here we are comparing point references, not values
	if p == points[0] {
		return points[2]
	} else if p == points[1] {
		return points[0]
	} else if p == points[2] {
		return points[1]
	}

	return nil
}

func (t *Triangle) pointCCW(p *Point) *Point {
	points := t.points
	// Here we are comparing point references, not values
	if p == points[0] {
		return points[1]
	} else if p == points[1] {
		return points[2]
	} else if p == points[2] {
		return points[0]
	}

	return nil
}

func (t *Triangle) neighborCW(p *Point) *Triangle {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.neighbors[1]
	} else if p == t.points[1] {
		return t.neighbors[2]
	}

	return t.neighbors[0]
}

func (t *Triangle) neighborCCW(p *Point) *Triangle {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.neighbors[2]
	} else if p == t.points[1] {
		return t.neighbors[0]
	}

	return t.neighbors[1]
}

func (t *Triangle) getConstrainedEdgeCW(p *Point) bool {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.constrainedEdge[1]
	} else if p == t.points[1] {
		return t.constrainedEdge[2]
	}

	return t.constrainedEdge[0]
}

func (t *Triangle) getConstrainedEdgeCCW(p *Point) bool {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.constrainedEdge[2]
	} else if p == t.points[1] {
		return t.constrainedEdge[0]
	}

	return t.constrainedEdge[1]
}

func (t *Triangle) getConstrainedEdgeAcross(p *Point) bool {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.constrainedEdge[0]
	} else if p == t.points[1] {
		return t.constrainedEdge[1]
	}

	return t.constrainedEdge[2]
}

func (t *Triangle) setConstrainedEdgeCW(p *Point, ce bool) {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		t.constrainedEdge[1] = ce
	} else if p == t.points[1] {
		t.constrainedEdge[2] = ce
	} else {
		t.constrainedEdge[0] = ce
	}
}

func (t *Triangle) setConstrainedEdgeCCW(p *Point, ce bool) {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		t.constrainedEdge[2] = ce
	} else if p == t.points[1] {
		t.constrainedEdge[0] = ce
	} else {
		t.constrainedEdge[1] = ce
	}
}

func (t *Triangle) getDelaunayEdgeCW(p *Point) bool {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.delaunayEdge[1]
	} else if p == t.points[1] {
		return t.delaunayEdge[2]
	}

	return t.delaunayEdge[0]
}

func (t *Triangle) getDelaunayEdgeCCW(p *Point) bool {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.delaunayEdge[2]
	} else if p == t.points[1] {
		return t.delaunayEdge[0]
	}

	return t.delaunayEdge[1]
}

func (t *Triangle) setDelaunayEdgeCW(p *Point, e bool) {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		t.delaunayEdge[1] = e
	} else if p == t.points[1] {
		t.delaunayEdge[2] = e
	} else {
		t.delaunayEdge[0] = e
	}
}

func (t *Triangle) setDelaunayEdgeCCW(p *Point, e bool) {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		t.delaunayEdge[2] = e
	} else if p == t.points[1] {
		t.delaunayEdge[0] = e
	} else {
		t.delaunayEdge[1] = e
	}
}

func (t *Triangle) neighborAcross(p *Point) *Triangle {
	// Here we are comparing point references, not values
	if p == t.points[0] {
		return t.neighbors[0]
	} else if p == t.points[1] {
		return t.neighbors[1]
	}

	return t.neighbors[2]
}

func (t *Triangle) oppositePoint(o *Triangle, p *Point) *Point {
	cw := o.pointCW(p)
	return t.pointCW(cw)
}

func (t *Triangle) legalize(opoint, npoint *Point) {
	points := t.points
	// Here we are comparing point references, not values
	if opoint == points[0] {
		points[1] = points[0]
		points[0] = points[2]
		points[2] = npoint
	} else if opoint == points[1] {
		points[2] = points[1]
		points[1] = points[0]
		points[0] = npoint
	} else if opoint == points[2] {
		points[0] = points[2]
		points[2] = points[1]
		points[1] = npoint
	} else {
		panic("poly2tri Invalid Triangle.legalize() call")
	}
}

func (t *Triangle) index(p *Point) int {
	points := t.points
	// Here we are comparing point references, not values
	if p == points[0] {
		return 0
	} else if p == points[1] {
		return 1
	} else if p == points[2] {
		return 2
	}

	panic("poly2tri Invalid Triangle.index() call")
}

func (t *Triangle) edgeIndex(p1, p2 *Point) int {
	points := t.points
	// Here we are comparing point references, not values
	if p1 == points[0] {
		if p2 == points[1] {
			return 2
		} else if p2 == points[2] {
			return 1
		}
	} else if p1 == points[1] {
		if p2 == points[2] {
			return 0
		} else if p2 == points[0] {
			return 2
		}
	} else if p1 == points[2] {
		if p2 == points[0] {
			return 1
		} else if p2 == points[1] {
			return 0
		}
	}
	return -1
}

func (t *Triangle) markConstrainedEdgeByEdge(edge *Edge) {
	t.markConstrainedEdgeByPoints(edge.p, edge.q)
}

func (t *Triangle) markConstrainedEdgeByPoints(p, q *Point) {
	points := t.points
	// Here we are comparing point references, not values
	if (q == points[0] && p == points[1]) || (q == points[1] && p == points[0]) {
		t.constrainedEdge[2] = true
	} else if (q == points[0] && p == points[2]) || (q == points[2] && p == points[0]) {
		t.constrainedEdge[1] = true
	} else if (q == points[1] && p == points[2]) || (q == points[2] && p == points[1]) {
		t.constrainedEdge[0] = true
	}
}
