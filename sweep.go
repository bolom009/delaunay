package delaunay

import "math"

const (
	EPSILON   float32 = 1e-12
	CW        int     = 1
	CCW       int     = -1
	COLLINEAR int     = 0
)

type Sweep struct{}

func (s *Sweep) triangulate(tcx *SweepContext) {
	tcx.initTriangulation()
	tcx.createAdvancingFront()
	// Sweep points; build mesh
	s.sweepPoints(tcx)
	// Clean up
	s.finalizationPolygon(tcx)
}

func (s *Sweep) sweepPoints(tcx *SweepContext) {
	for i := 1; i < len(tcx.points); i++ {
		point := tcx.points[i]
		node := s.pointEvent(tcx, point)
		if edges := point.edgesList; edges != nil {
			for _, edge := range edges {
				s.edgeEventByEdge(tcx, edge, node)
			}
		}
	}
}

func (s *Sweep) finalizationPolygon(tcx *SweepContext) {
	// Get an Internal triangle to start with
	t := tcx.front.head.next.triangle
	p := tcx.front.head.next.point
	for !t.getConstrainedEdgeCW(p) {
		t = t.neighborCCW(p)
	}
	// Collect interior triangles constrained by edges
	tcx.meshClean(t)
}

func (s *Sweep) pointEvent(tcx *SweepContext, point *Point) *Node {
	node := tcx.locateNode(point)
	newNode := s.newFrontTriangle(tcx, point, node)

	// Only need to check +epsilon since point never have smaller
	// x value than node due to how we fetch nodes from the front
	if point.X <= node.point.X+(EPSILON) {
		s.fill(tcx, node)
	}
	//tcx.AddNode(new_node);
	s.fillAdvancingFront(tcx, newNode)
	return newNode
}

func (s *Sweep) edgeEventByEdge(tcx *SweepContext, edge *Edge, node *Node) {
	tcx.edgeEvent.constrainedEdge = edge
	tcx.edgeEvent.right = edge.p.X > edge.q.X
	if s.isEdgeSideOfTriangle(node.triangle, edge.p, edge.q) {
		return
	}

	// For now we will do all needed filling
	// TODO: integrate with flip process might give some better performance
	//       but for now s avoid the issue with cases that needs both flips and fills
	s.fillEdgeEvent(tcx, edge, node)
	s.edgeEventByPoints(tcx, edge.p, edge.q, node.triangle, edge.q)
}

func (s *Sweep) edgeEventByPoints(tcx *SweepContext, ep *Point, eq *Point, triangle *Triangle, point *Point) {
	if s.isEdgeSideOfTriangle(triangle, ep, eq) {
		return
	}

	p1 := triangle.pointCCW(point)
	o1 := s.orient2d(eq, p1, ep)
	if o1 == COLLINEAR {
		// TODO integrate here changes from C++ version
		// (C++ repo revision 09880a869095 dated March 8, 2011)
		panic("poly2tri EdgeEvent: Collinear not supported! " + eq.toString() + " " + p1.toString() + " " + ep.toString())
	}

	p2 := triangle.pointCW(point)
	o2 := s.orient2d(eq, p2, ep)
	if o2 == COLLINEAR {
		// TODO integrate here changes from C++ version
		// (C++ repo revision 09880a869095 dated March 8, 2011)
		panic("poly2tri EdgeEvent: Collinear not supported! " + eq.toString() + " " + p2.toString() + " " + ep.toString())
	}

	if o1 == o2 {
		// Need to decide if we are rotating CW or CCW to get to a triangle
		// that will cross edge
		if o1 == CW {
			triangle = triangle.neighborCCW(point)
		} else {
			triangle = triangle.neighborCW(point)
		}
		s.edgeEventByPoints(tcx, ep, eq, triangle, point)
	} else {
		// This triangle crosses constraint so lets flippin start!
		s.flipEdgeEvent(tcx, ep, eq, triangle, point)
	}
}

func (s *Sweep) isEdgeSideOfTriangle(triangle *Triangle, ep *Point, eq *Point) bool {
	index := triangle.edgeIndex(ep, eq)
	if index != -1 {
		triangle.constrainedEdge[index] = true
		if t := triangle.neighbors[index]; t != nil {
			t.markConstrainedEdgeByPoints(ep, eq)
		}
		return true
	}
	return false
}

func (s *Sweep) newFrontTriangle(tcx *SweepContext, point *Point, node *Node) *Node {
	triangle := NewTriangle(point, node.point, node.next.point)
	triangle.markNeighbor(node.triangle)
	tcx.maps = append(tcx.maps, triangle)
	newNode := NewNode(point, nil)
	newNode.next = node.next
	newNode.prev = node
	node.next.prev = newNode
	node.next = newNode
	if !s.legalize(tcx, triangle) {
		tcx.mapTriangleToNodes(triangle)
	}

	return newNode
}

func (s *Sweep) fill(tcx *SweepContext, node *Node) {
	triangle := NewTriangle(node.prev.point, node.point, node.next.point)
	triangle.markNeighbor(node.prev.triangle)
	triangle.markNeighbor(node.triangle)
	tcx.maps = append(tcx.maps, triangle)
	// Update the advancing front
	node.prev.next = node.next
	node.next.prev = node.prev
	// If it was legalized the triangle has already been mapped
	if !s.legalize(tcx, triangle) {
		tcx.mapTriangleToNodes(triangle)
	}
}

func (s *Sweep) fillAdvancingFront(tcx *SweepContext, n *Node) {
	node := n.next
	for node.next != nil {
		// TODO integrate here changes from C++ version
		// (C++ repo revision acf81f1f1764 dated April 7, 2012)
		if s.isAngleObtuse(node.point, node.next.point, node.prev.point) {
			break
		}

		s.fill(tcx, node)
		node = node.next
	}
	// Fill left holes
	node = n.prev
	for node.prev != nil {
		// TODO integrate here changes from C++ version
		// (C++ repo revision acf81f1f1764 dated April 7, 2012)
		if s.isAngleObtuse(node.point, node.next.point, node.prev.point) {
			break
		}
		s.fill(tcx, node)
		node = node.prev
	}
	// Fill right basins
	if n.next != nil && n.next.next != nil {
		if s.isBasinAngleRight(n) {
			s.fillBasin(tcx, n)
		}
	}
}

func (s *Sweep) isBasinAngleRight(node *Node) bool {
	ax := node.point.X - node.next.next.point.X
	ay := node.point.Y - node.next.next.point.Y
	if ay < 0 {
		panic("unordered y")
	}

	return ax >= 0 || float32(math.Abs(float64(ax))) < ay
}

func (s *Sweep) legalize(tcx *SweepContext, t *Triangle) bool {
	for i := 0; i < 3; i++ {
		if t.delaunayEdge[i] {
			continue
		}

		if ot := t.neighbors[i]; ot != nil {
			p := t.points[i]
			op := ot.oppositePoint(t, p)
			oi := ot.index(op)
			// If s is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
			// then we should not try to legalize
			if ot.constrainedEdge[oi] || ot.delaunayEdge[oi] {
				t.constrainedEdge[i] = ot.constrainedEdge[oi]
				continue
			}
			if s.inCircle(p, t.pointCCW(p), t.pointCW(p), op) {
				// Lets mark s shared edge as Delaunay
				t.delaunayEdge[i] = true
				ot.delaunayEdge[oi] = true
				// Lets rotate shared edge one vertex CW to legalize it
				s.rotateTrianglePair(t, p, ot, op)
				// We now got one valid Delaunay Edge shared by two triangles
				// This gives us 4 new edges to check for Delaunay
				// Make sure that triangle to node mapping is done only one time for a specific triangle
				if !s.legalize(tcx, t) {
					tcx.mapTriangleToNodes(t)
				}
				if !s.legalize(tcx, ot) {
					tcx.mapTriangleToNodes(ot)
				}
				// Reset the Delaunay edges, since they only are valid Delaunay edges
				// until we add a new triangle or point.
				// XXX: need to think about s. Can these edges be tried after we
				//      return to previous recursive level?
				t.delaunayEdge[i] = false
				ot.delaunayEdge[oi] = false
				// If triangle have been legalized no need to check the other edges since
				// the recursive legalization will handles those so we can end here.
				return true
			}
		}
	}
	return false
}

/**
 * <b>Requirement</b>:<br>
 * 1. a,b and c form a triangle.<br>
 * 2. a and d is know to be on opposite side of bc<br>
 * <pre>
 *                a
 *                +
 *               / \
 *              /   \
 *            b/     \c
 *            +-------+
 *           /    d    \
 *          /           \
 * </pre>
 * <b>Fact</b>: d has to be in area B to have a chance to be inside the circle formed by
 *  a,b and c<br>
 *  d is outside B if orient2d(a,b,d) or orient2d(c,a,d) is CW<br>
 *  This preknowledge gives us a way to optimize the incircle test
 * @param pa - triangle point, opposite d
 * @param pb - triangle point
 * @param pc - triangle point
 * @param pd - point opposite a
 * @return {boolean} true if d is inside circle, false if on circle edge
 */
func (s *Sweep) inCircle(pa, pb, pc, pd *Point) bool {
	adx := pa.X - pd.X
	ady := pa.Y - pd.Y
	bdx := pb.X - pd.X
	bdy := pb.Y - pd.Y
	adxbdy := adx * bdy
	bdxady := bdx * ady
	oabd := adxbdy - bdxady
	if oabd <= 0 {
		return false
	}
	cdx := pc.X - pd.X
	cdy := pc.Y - pd.Y
	cdxady := cdx * ady
	adxcdy := adx * cdy
	ocad := cdxady - adxcdy
	if ocad <= 0 {
		return false
	}
	bdxcdy := bdx * cdy
	cdxbdy := cdx * bdy
	alift := adx*adx + ady*ady
	blift := bdx*bdx + bdy*bdy
	clift := cdx*cdx + cdy*cdy
	det := alift*(bdxcdy-cdxbdy) + blift*ocad + clift*oabd
	return det > 0
}

/**
 * Rotates a triangle pair one vertex CW
 *<pre>
 *       n2                    n2
 *  P +-----+             P +-----+
 *    | t  /|               |\  t |
 *    |   / |               | \   |
 *  n1|  /  |n3           n1|  \  |n3
 *    | /   |    after CW   |   \ |
 *    |/ oT |               | oT \|
 *    +-----+ oP            +-----+
 *       n4                    n4
 * </pre>
 */
func (s *Sweep) rotateTrianglePair(t *Triangle, p *Point, ot *Triangle, op *Point) {
	n1 := t.neighborCCW(p)
	n2 := t.neighborCW(p)
	n3 := ot.neighborCCW(op)
	n4 := ot.neighborCW(op)
	ce1 := t.getConstrainedEdgeCCW(p)
	ce2 := t.getConstrainedEdgeCW(p)
	ce3 := ot.getConstrainedEdgeCCW(op)
	ce4 := ot.getConstrainedEdgeCW(op)
	de1 := t.getDelaunayEdgeCCW(p)
	de2 := t.getDelaunayEdgeCW(p)
	de3 := ot.getDelaunayEdgeCCW(op)
	de4 := ot.getDelaunayEdgeCW(op)
	t.legalize(p, op)
	ot.legalize(op, p)
	// Remap delaunayEdge
	ot.setDelaunayEdgeCCW(p, de1)
	t.setDelaunayEdgeCW(p, de2)
	t.setDelaunayEdgeCCW(op, de3)
	ot.setDelaunayEdgeCW(op, de4)
	// Remap constrainedEdge
	ot.setConstrainedEdgeCCW(p, ce1)
	t.setConstrainedEdgeCW(p, ce2)
	t.setConstrainedEdgeCCW(op, ce3)
	ot.setConstrainedEdgeCW(op, ce4)
	// Remap neighbors
	// XXX: might optimize the markNeighbor by keeping track of
	//      what side should be assigned to what neighbor after the
	//      rotation. Now mark neighbor does lots of testing to find
	//      the right side.
	t.clearNeighbors()
	ot.clearNeighbors()
	if n1 != nil {
		ot.markNeighbor(n1)
	}
	if n2 != nil {
		t.markNeighbor(n2)
	}
	if n3 != nil {
		t.markNeighbor(n3)
	}
	if n4 != nil {
		ot.markNeighbor(n4)
	}
	t.markNeighbor(ot)
}

func (s *Sweep) fillBasin(tcx *SweepContext, node *Node) {
	if s.orient2d(node.point, node.next.point, node.next.next.point) == CCW {
		tcx.basin.leftNode = node.next.next
	} else {
		tcx.basin.leftNode = node.next
	}

	// Find the bottom and right node
	tcx.basin.bottomNode = tcx.basin.leftNode
	for tcx.basin.bottomNode.next != nil && tcx.basin.bottomNode.point.Y >= tcx.basin.bottomNode.next.point.Y {
		tcx.basin.bottomNode = tcx.basin.bottomNode.next
	}

	if tcx.basin.bottomNode == tcx.basin.leftNode {
		// No valid basin
		return
	}

	tcx.basin.rightNode = tcx.basin.bottomNode
	for tcx.basin.rightNode.next != nil && tcx.basin.rightNode.point.Y < tcx.basin.rightNode.next.point.Y {
		tcx.basin.rightNode = tcx.basin.rightNode.next
	}

	if tcx.basin.rightNode == tcx.basin.bottomNode {
		// No valid basins
		return
	}
	tcx.basin.width = tcx.basin.rightNode.point.X - tcx.basin.leftNode.point.X
	tcx.basin.leftHighest = tcx.basin.leftNode.point.Y > tcx.basin.rightNode.point.Y
	s.fillBasinReq(tcx, tcx.basin.bottomNode)
}

func (s *Sweep) fillBasinReq(tcx *SweepContext, node *Node) {
	// if shallow stop filling
	if s.isShallow(tcx, node) {
		return
	}

	s.fill(tcx, node)
	if node.prev == tcx.basin.leftNode && node.next == tcx.basin.rightNode {
		return
	} else if node.prev == tcx.basin.leftNode {
		o := s.orient2d(node.point, node.next.point, node.next.next.point)
		if o == CW {
			return
		}
		node = node.next
	} else if node.next == tcx.basin.rightNode {
		o := s.orient2d(node.point, node.prev.point, node.prev.prev.point)
		if o == CCW {
			return
		}
		node = node.prev
	} else {
		// Continue with the neighbor node with lowest Y value
		if node.prev.point.Y < node.next.point.Y {
			node = node.prev
		} else {
			node = node.next
		}
	}
	s.fillBasinReq(tcx, node)
}
func (s *Sweep) isShallow(tcx *SweepContext, node *Node) bool {
	height := float32(0)
	if tcx.basin.leftHighest {
		height = tcx.basin.leftNode.point.Y - node.point.Y
	} else {
		height = tcx.basin.rightNode.point.Y - node.point.Y
	}

	// if shallow stop filling
	if tcx.basin.width > height {
		return true
	}
	return false
}

func (s *Sweep) fillEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if tcx.edgeEvent.right {
		s.fillRightAboveEdgeEvent(tcx, edge, node)
	} else {
		s.fillLeftAboveEdgeEvent(tcx, edge, node)
	}
}

func (s *Sweep) fillRightAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.next.point.X < edge.p.X {
		// Check if next node is below the edge
		if s.orient2d(edge.q, node.next.point, edge.p) == CCW {
			s.fillRightBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.next
		}
	}
}
func (s *Sweep) fillRightBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X < edge.p.X {
		if s.orient2d(node.point, node.next.point, node.next.next.point) == CCW {
			// Concave
			s.fillRightConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			s.fillRightConvexEdgeEvent(tcx, edge, node)
			// Retry
			s.fillRightBelowEdgeEvent(tcx, edge, node)
		}
	}
}
func (s *Sweep) fillRightConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	s.fill(tcx, node.next)
	if node.next.point != edge.p {
		// Next above or below edge?
		if s.orient2d(edge.q, node.next.point, edge.p) == CCW {
			// Below
			if s.orient2d(node.point, node.next.point, node.next.next.point) == CCW {
				// Next is concave
				s.fillRightConcaveEdgeEvent(tcx, edge, node)
			}
		}
	}
}
func (s *Sweep) fillRightConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if s.orient2d(node.next.point, node.next.next.point, node.next.next.next.point) == CCW {
		// Concave
		s.fillRightConcaveEdgeEvent(tcx, edge, node.next)
	} else {
		// Convex
		// Next above or below edge?
		if s.orient2d(edge.q, node.next.next.point, edge.p) == CCW {
			// Below
			s.fillRightConvexEdgeEvent(tcx, edge, node.next)
		}
	}
}
func (s *Sweep) fillLeftAboveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	for node.prev.point.X > edge.p.X {
		// Check if next node is below the edge
		if s.orient2d(edge.q, node.prev.point, edge.p) == CW {
			s.fillLeftBelowEdgeEvent(tcx, edge, node)
		} else {
			node = node.prev
		}
	}
}

func (s *Sweep) fillLeftBelowEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	if node.point.X > edge.p.X {
		if s.orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
			// Concave
			s.fillLeftConcaveEdgeEvent(tcx, edge, node)
		} else {
			// Convex
			s.fillLeftConvexEdgeEvent(tcx, edge, node)
			// Retry s one
			s.fillLeftBelowEdgeEvent(tcx, edge, node)
		}
	}
}

func (s *Sweep) fillLeftConvexEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	// Next concave or convex?
	if s.orient2d(node.prev.point, node.prev.prev.point, node.prev.prev.prev.point) == CW {
		// Concave
		s.fillLeftConcaveEdgeEvent(tcx, edge, node.prev)
	} else {
		// Convex
		// Next above or below edge?
		if s.orient2d(edge.q, node.prev.prev.point, edge.p) == CW {
			// Below
			s.fillLeftConvexEdgeEvent(tcx, edge, node.prev)
		}
	}
}

func (s *Sweep) fillLeftConcaveEdgeEvent(tcx *SweepContext, edge *Edge, node *Node) {
	s.fill(tcx, node.prev)
	if node.prev.point != edge.p {
		// Next above or below edge?
		if s.orient2d(edge.q, node.prev.point, edge.p) == CW {
			// Below
			if s.orient2d(node.point, node.prev.point, node.prev.prev.point) == CW {
				// Next is concave
				s.fillLeftConcaveEdgeEvent(tcx, edge, node)
			}
		}
	}
}

func (s *Sweep) flipEdgeEvent(tcx *SweepContext, ep *Point, eq *Point, t *Triangle, p *Point) {
	ot := t.neighborAcross(p)
	if ot == nil {
		panic("FLIP failed due to missing triangle!")
	}
	op := ot.oppositePoint(t, p)
	// Additional check from Java version (see issue #88)
	if t.getConstrainedEdgeAcross(p) {
		index := t.index(p)
		panic("poly2tri Intersecting Constraints " + p.toString() + " " + op.toString() + " " + t.points[(index+1)%3].toString() + " " + t.points[(index+2)%3].toString())
	}
	if s.inScanArea(p, t.pointCCW(p), t.pointCW(p), op) {
		// Lets rotate shared edge one vertex CW
		s.rotateTrianglePair(t, p, ot, op)
		tcx.mapTriangleToNodes(t)
		tcx.mapTriangleToNodes(ot)

		// XXX: in the original C++ code for the next 2 lines, we are
		// comparing point values (and not pointers). In s JavaScript
		// code, we are comparing point references (pointers). This works
		// because we can't have 2 different points with the same values.
		// But to be really equivalent, we should use "Point.equals" here.
		if p == eq && op == ep {
			if eq == tcx.edgeEvent.constrainedEdge.q && ep == tcx.edgeEvent.constrainedEdge.p {
				t.markConstrainedEdgeByPoints(ep, eq)
				ot.markConstrainedEdgeByPoints(ep, eq)
				s.legalize(tcx, t)
				s.legalize(tcx, ot)
			}
		} else {
			o := s.orient2d(eq, op, ep)
			t = s.nextFlipTriangle(tcx, o, t, ot, p, op)
			s.flipEdgeEvent(tcx, ep, eq, t, p)
		}
	} else {
		newP := s.nextFlipPoint(ep, eq, ot, op)
		s.flipScanEdgeEvent(tcx, ep, eq, t, ot, newP)
		s.edgeEventByPoints(tcx, ep, eq, t, p)
	}
}

func (s *Sweep) nextFlipTriangle(tcx *SweepContext, o int, t *Triangle, ot *Triangle, p *Point, op *Point) *Triangle {
	if o == CCW {
		// ot is not crossing edge after flip
		edgeIndex := ot.edgeIndex(p, op)
		ot.delaunayEdge[edgeIndex] = true
		s.legalize(tcx, ot)
		ot.clearDelaunayEdges()
		return t
	}
	// t is not crossing edge after flip
	edgeIndex := t.edgeIndex(p, op)

	t.delaunayEdge[edgeIndex] = true
	s.legalize(tcx, t)
	t.clearDelaunayEdges()
	return ot
}

func (s *Sweep) nextFlipPoint(ep *Point, eq *Point, ot *Triangle, op *Point) *Point {
	o2d := s.orient2d(eq, op, ep)
	if o2d == CW {
		// Right
		return ot.pointCCW(op)
	} else if o2d == CCW {
		// Left
		return ot.pointCW(op)
	}

	panic("poly2tri [Unsupported] nextFlipPoint: opposing point on constrained edge! " + eq.toString() + " " + op.toString() + " " + ep.toString())
}

func (s *Sweep) flipScanEdgeEvent(tcx *SweepContext, ep *Point, eq *Point, flipTriangle *Triangle, t *Triangle, p *Point) {
	ot := t.neighborAcross(p)
	if ot == nil {
		panic("FLIP failed due to missing triangle")
	}
	op := ot.oppositePoint(t, p)
	if s.inScanArea(eq, flipTriangle.pointCCW(eq), flipTriangle.pointCW(eq), op) {
		// flip with new edge op.eq
		s.flipEdgeEvent(tcx, eq, op, ot, op)
	} else {
		newP := s.nextFlipPoint(ep, eq, ot, op)
		s.flipScanEdgeEvent(tcx, ep, eq, flipTriangle, ot, newP)
	}
}

func (s *Sweep) orient2d(pa, pb, pc *Point) int {
	detleft := (pa.X - pc.X) * (pb.Y - pc.Y)
	detright := (pa.Y - pc.Y) * (pb.X - pc.X)
	val := detleft - detright
	if val > -(EPSILON) && val < (EPSILON) {
		return COLLINEAR
	} else if val > 0 {
		return CCW
	}

	return CW
}

func (s *Sweep) inScanArea(pa, pb, pc, pd *Point) bool {
	oadb := (pa.X-pb.X)*(pd.Y-pb.Y) - (pd.X-pb.X)*(pa.Y-pb.Y)
	if oadb >= -EPSILON {
		return false
	}

	oadc := (pa.X-pc.X)*(pd.Y-pc.Y) - (pd.X-pc.X)*(pa.Y-pc.Y)
	if oadc <= EPSILON {
		return false
	}
	return true
}

func (s *Sweep) isAngleObtuse(pa, pb, pc *Point) bool {
	ax := pb.X - pa.X
	ay := pb.Y - pa.Y
	bx := pc.X - pa.X
	by := pc.Y - pa.Y
	return (ax*bx + ay*by) < 0
}
