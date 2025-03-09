package main

import (
	"encoding/json"
	"fmt"
	"github.com/bolom009/delaunay"
	"strings"
)

const floorPlan = `{"canvas":{"w":800,"h":600},"polygons":[[{"x":656.4375,"y":86},{"x":664.4375,"y":495},{"x":116.4375,"y":494},{"x":117.4375,"y":86}],[{"x":301.4375,"y":154},{"x":299.4375,"y":247},{"x":192.4375,"y":246},{"x":192.4375,"y":157}],[{"x":602.4375,"y":245},{"x":480.4375,"y":242},{"x":550.4375,"y":149}],[{"x":335.4375,"y":340},{"x":365.4375,"y":310},{"x":429.4375,"y":303},{"x":484.4375,"y":325},{"x":492.4375,"y":378},{"x":457.4375,"y":422},{"x":379.4375,"y":432},{"x":333.4375,"y":400},{"x":324.4375,"y":367}]]}`

type Vector2 struct {
	X float32
	Y float32
}

type polygonToolJSON struct {
	Polygons [][]Vector2 `json:"polygons"`
}

func main() {
	polygon, holes, err := newPolygonsFromJSON([]byte(floorPlan))
	if err != nil {
		panic(err)
	}

	d := newDelaunay(polygon, holes)
	for _, triangle := range d.Triangulate() {
		arr := make([]string, 3)
		for i, point := range triangle.GetPoints() {
			arr[i] = fmt.Sprintf("(%v, %v)", point.X, point.Y)
		}

		fmt.Println("triangle =>", strings.Join(arr, ", "))
	}
}

func newDelaunay(polygon []Vector2, holes [][]Vector2) *delaunay.SweepContext {
	pp := make([]*delaunay.Point, 0)
	for _, p := range polygon {
		pp = append(pp, delaunay.NewPoint(p.X, p.Y))
	}

	hh := make([][]*delaunay.Point, 0)
	for _, holePoints := range holes {
		hole := make([]*delaunay.Point, len(holePoints))
		for i, p := range holePoints {
			hole[i] = delaunay.NewPoint(p.X, p.Y)
		}

		hh = append(hh, hole)
	}

	return delaunay.NewSweepContext(pp, hh)
}

// newPolygonsFromJSON loads a polygon set from JSON data as exported by the
// [Polygon Constructor]: https://alaricus.github.io/PolygonConstructor/
func newPolygonsFromJSON(jsonData []byte) (polygon []Vector2, holes [][]Vector2, err error) {
	var jsonStruct polygonToolJSON
	err = json.Unmarshal(jsonData, &jsonStruct)
	if err != nil {
		return nil, nil, fmt.Errorf("could not unmarshal polygon JSON: %w", err)
	}

	polygon = make([]Vector2, 0)
	for _, pp := range jsonStruct.Polygons[0] {
		polygon = append(polygon, Vector2{X: pp.X, Y: pp.Y})
	}

	// Example hole: a small square hole in the middle: (4,4), (6,4), (6,6), (4,6)
	holes = make([][]Vector2, 0)
	for _, pp := range jsonStruct.Polygons[1:] {
		hole := make([]Vector2, len(pp))
		for j, pp := range pp {
			hole[j] = Vector2{X: pp.X, Y: pp.Y}
		}

		holes = append(holes, hole)
	}

	return polygon, holes, nil
}
