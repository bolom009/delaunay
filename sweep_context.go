package delaunay

const kAlpha float32 = 0.3

type SweepContext struct {
	triangles []*Triangle
	maps      []*Triangle
	points    []*Point
	edgeList  []*Edge
	pMin      *Point
	pMax      *Point
	front     *AdvancingFront
	head      *Point
	tail      *Point
	afHead    *Node
	afMiddle  *Node
	afTail    *Node
	basin     *Basin
	edgeEvent *EdgeEvent
	sweep     *Sweep
}

func NewSweepContext(polygon []*Point, holes [][]*Point) *SweepContext {
	s := &SweepContext{
		sweep:     &Sweep{},
		triangles: make([]*Triangle, 0),
		maps:      make([]*Triangle, 0),
		points:    polygon,
		edgeList:  make([]*Edge, len(polygon)),
		pMin:      nil,
		pMax:      nil,
		front:     nil,
		head:      nil,
		tail:      nil,
		afHead:    nil,
		afMiddle:  nil,
		afTail:    nil,
		basin:     &Basin{},
		edgeEvent: &EdgeEvent{},
	}

	s.initEdges(polygon)
	for _, hole := range holes {
		s.AddHole(hole)
	}

	return s
}

func (s *SweepContext) AddHole(polyline []*Point) {
	s.initEdges(polyline)
	s.points = append(s.points, polyline...)
}

func (s *SweepContext) Triangulate() []*Triangle {
	s.sweep.triangulate(s)
	return s.triangles
}

func (s *SweepContext) getBoundingBox() (*Point, *Point) {
	return s.pMin, s.pMax
}

func (s *SweepContext) initTriangulation() {
	xMax := s.points[0].X
	xMin := s.points[0].X
	yMax := s.points[0].Y
	yMin := s.points[0].Y
	// Calculate bounds
	length := len(s.points)
	for i := 1; i < length; i++ {
		p := s.points[i]
		/* jshint expr:true */
		if p.X > xMax {
			xMax = p.X
		}
		if p.X < xMin {
			xMin = p.X
		}
		if p.Y > yMax {
			yMax = p.Y
		}
		if p.Y < yMin {
			yMin = p.Y
		}
	}
	s.pMin = NewPoint(xMin, yMin)
	s.pMax = NewPoint(xMax, yMax)
	dx := kAlpha * (xMax - xMin)
	dy := kAlpha * (yMax - yMin)
	s.head = NewPoint(xMax+dx, yMin-dy)
	s.tail = NewPoint(xMin-dx, yMin-dy)

	// Sort points along y-axis
	shot := &ISort{}
	shot.Data = s.points
	shot.Call = func(a interface{}, b interface{}) bool {
		_a := a.(*Point)
		_b := b.(*Point)
		if _a.Y == _b.Y {
			return _a.X-_b.X < 0
		}

		return _a.Y-_b.Y < 0
	}
	shot.Sort()
	shot.Free()
}

func (s *SweepContext) initEdges(polyline []*Point) {
	l := len(polyline)
	for i := 0; i < l; i++ {
		s.edgeList = append(s.edgeList, NewEdge(polyline[i], polyline[(i+1)%l]))
	}
}

func (s *SweepContext) locateNode(point *Point) *Node {
	return s.front.locateNode(point.X)
}

func (s *SweepContext) createAdvancingFront() {
	// Initial triangle
	triangle := NewTriangle(s.points[0], s.tail, s.head)
	s.maps = append(s.maps, triangle)
	head := NewNode(triangle.points[1], triangle)
	middle := NewNode(triangle.points[0], triangle)
	tail := NewNode(triangle.points[2], nil)
	s.front = NewAdvancingFront(head, tail)
	head.next = middle
	middle.next = tail
	middle.prev = head
	tail.prev = middle
}

func (s *SweepContext) mapTriangleToNodes(t *Triangle) {
	for i := 0; i < 3; i++ {
		if t.neighbors[i] == nil {
			if n := s.front.locatePoint(t.pointCW(t.points[i])); n != nil {
				n.triangle = t
			}
		}
	}
}

func (s *SweepContext) removeFromMap(triangle *Triangle) {
	length := len(s.maps)
	for i := 0; i < length; i++ {
		if s.maps[i] == triangle {
			s.maps = append(s.maps[:i], s.maps[i+1:]...)
			break
		}
	}
}

func (s *SweepContext) meshClean(triangle *Triangle) {
	triangles := []*Triangle{triangle}
	for len(triangles) > 0 {
		t := triangles[0]
		triangles = triangles[1:]
		if !t.interior {
			t.interior = true
			s.triangles = append(s.triangles, t)
			for i := 0; i < 3; i++ {
				if !t.constrainedEdge[i] {
					triangles = append(triangles, t.neighbors[i])
				}
			}
		}
	}
}
