package delaunay

import (
	"strconv"
)

type Point struct {
	X         float32
	Y         float32
	edgesList []*Edge
}

func NewPoint(x, y float32) *Point {
	return &Point{
		X:         x,
		Y:         y,
		edgesList: nil,
	}
}

func (p *Point) toString() string {
	return strconv.FormatFloat(float64(p.X), 'f', 4, 32) + "," + strconv.FormatFloat(float64(p.Y), 'f', 4, 32)
}
