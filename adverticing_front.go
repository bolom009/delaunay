package delaunay

type AdvancingFront struct {
	head       *Node
	tail       *Node
	searchNode *Node
}

func NewAdvancingFront(head, tail *Node) *AdvancingFront {
	return &AdvancingFront{
		head:       head,
		tail:       tail,
		searchNode: head,
	}
}

func (a *AdvancingFront) locateNode(x float32) *Node {
	node := a.searchNode
	if x < node.value {
		for node = node.prev; node != nil; node = node.prev {
			if x >= node.value {
				a.searchNode = node
				return node
			}
		}
	} else {
		for node = node.next; node != nil; node = node.next {
			if x < node.value {
				a.searchNode = node.prev
				return node.prev
			}
		}
	}
	return nil
}
func (a *AdvancingFront) locatePoint(point *Point) *Node {
	px := point.X
	node := a.searchNode
	nx := node.point.X
	if px == nx {
		// Here we are comparing point references, not values
		if point != node.point {
			// We might have two nodes with same x value for a short time
			if point == node.prev.point {
				node = node.prev
			} else if point == node.next.point {
				node = node.next
			} else {
				panic("poly2tri Invalid AdvancingFront.locatePoint() call")
			}
		}
	} else if px < nx {
		for node = node.prev; node != nil; node = node.prev {
			if point == node.point {
				break
			}
		}
	} else {
		for node = node.next; node != nil; node = node.next {
			if point == node.point {
				break
			}
		}
	}
	if node != nil {
		a.searchNode = node
	}
	return node
}
