package delaunay

import (
	"reflect"
	"sort"
)

type ISort struct {
	Data interface{}
	Call func(interface{}, interface{}) bool
}

func (s *ISort) Len() int {
	return reflect.ValueOf(s.Data).Len()
}
func (s *ISort) Swap(a, b int) {
	db := reflect.ValueOf(s.Data)
	A := db.Index(a)
	B := db.Index(b)
	AV := reflect.ValueOf(A.Interface())
	BV := reflect.ValueOf(B.Interface())
	A.Set(BV)
	B.Set(AV)
}
func (s *ISort) Less(a, b int) bool {
	db := reflect.ValueOf(s.Data)
	A := db.Index(a)
	B := db.Index(b)
	return s.Call(A.Interface(), B.Interface())
}
func (s *ISort) Sort() {
	sort.Sort(s)
}
func (s *ISort) Stable() {
	sort.Stable(s)
}
func (s *ISort) Free() {
	s.Data = nil
	s.Call = nil
}
