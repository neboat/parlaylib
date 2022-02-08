#include <parlay/primitives.h>
#include <parlay/random.h>
#include <parlay/io.h>
#include <parlay/internal/get_time.h>
#include "hashmap.h"

// **************************************************************
// Delaunay Triangulation
// **************************************************************

using real = float;
using point_id = int;

struct point {
  point_id id; real x; real y;
  const bool operator==(const point p) const {return id == p.id;}
  const bool operator<(const point p) const {return id < p.id;}
};

struct tri { point_id p1, p2, p3;
  const bool operator==(const tri e) const {return p1 == e.p1 && p2 == e.p2 && p3 == e.p3;}
  const bool operator<(const tri e) const {return
      ((p1 < e.p1) ||
       (p1 == e.p1 && (p2 < e.p2 || p2 == e.p2 && p3 < e.p3)));}
  const size_t hash() {return parlay::hash64(p1) + parlay::hash64(p2) + parlay::hash64(p3);}
};

struct triangle {
  tri t;
  parlay::sequence<point> conflicts;
  triangle();
  triangle(tri t, parlay::sequence<point> c) : t(t), conflicts(c) {}
};
using triangle_ptr = std::shared_ptr<triangle>;

struct edge { point_id p1, p2;
  const bool operator==(const edge e) const {return p1 == e.p1 && p2 == e.p2;}
  const bool operator<(const edge e) const {return (p1 < e.p1) || (p1 == e.p1 && p2 < e.p2);}
  const unsigned long hash() {return parlay::hash64(p1) + parlay::hash64(p2);}
};
struct edge_triangle {edge e; triangle_ptr t;};

bool in_circle (point a, point b, point c, point d) {
  double ax = a.x-d.x;  double ay = a.y-d.y;
  double bx = b.x-d.x;  double by = b.y-d.y;
  double cx = c.x-d.x;  double cy = c.y-d.y;
  double total = ((ax*ax + ay*ay) * (bx*cy - cx*by) -
		  (bx*bx + by*by) * (ax*cy - cx*ay) +
		  (ax*by - bx*ay) * (cx*cx + cy*cy));
  return total < 0.0;
}

auto in_circle2 (point a, point b, point d) {
  struct vect { double x,y,z;};
  auto project = [=] (point p) {
    double px = p.x-d.x; double py = p.y-d.y;
    return vect{px, py, px*px + py*py};};
  auto cross = [] (vect v1, vect v2) {
    return vect{v1.y*v2.z - v1.z*v2.y,
		v1.z*v2.x - v1.x*v2.z,
		v1.x*v2.y - v1.y*v2.x};};
  vect cp = cross(project(a), project(b));
  return [=] (point c) -> bool{
    auto dot = [] (vect v1, vect v2) {
      return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;};
    return dot(cp, project(c)) > 0.0;};
}

// **************************************************************
// The main body
// **************************************************************

struct Delaunay {
  using Points = parlay::sequence<point>;
  hashmap<tri,bool> mesh;
  hashmap<edge,triangle_ptr> edges;
  Points points;
  point_id n;

  point_id earliest(triangle_ptr& t) {
    return (t->conflicts.size() == 0) ? n : t->conflicts[0].id;
  }

  auto filter_points(triangle_ptr& t1, triangle_ptr& t2, tri t) {
    auto a = parlay::merge(t1->conflicts, t2->conflicts);
    auto in_circle = in_circle2(points[t.p1],points[t.p2],points[t.p3]);
    auto keep = parlay::tabulate(a.size(), [&] (long i) {
	return ((i != 0) && (a[i].id != a[i-1].id) &&
		((i+1 < a.size() && a[i].id == a[i+1].id) ||
		 in_circle(a[i])));},500);
    return parlay::pack(a, keep);
  }
      
  void process_edge(triangle_ptr& t1, edge e, triangle_ptr& t2) {
    if (t1->conflicts.size() == 0 && t2->conflicts.size() == 0) {
      mesh.insert(t1->t,true); mesh.insert(t2->t,true);
      t1 = t2 = nullptr;
    } else if (earliest(t2) == earliest(t1)) {
      t1 = t2 = nullptr;
    } else {
      if (earliest(t2) < earliest(t1)) {
	std::swap(t2, t1); std::swap(e.p1, e.p2);}
      point_id p = earliest(t1);
      tri t{e.p1, e.p2, p};
      t1 = std::make_shared<triangle>(t, filter_points(t1, t2, t));
      auto check_edge = [&] (edge e, triangle_ptr& tp) {
	auto key = (e.p1 < e.p2) ? e : edge{e.p2, e.p1};
	if (edges.insert(key, tp)) return;
	auto tt = std::move(edges.at(key));
	process_edge(tp, e, tt);};
      auto ta1 = t1; 
      auto tb1 = t1;
      parlay::par_do3([&] () {check_edge(edge{p, e.p1}, ta1);},
		      [&] () {check_edge(edge{e.p2, p}, tb1);},
		      [&] {process_edge(t1, e, t2);});
    }
  }

  Delaunay(const Points& P) :
    mesh(hashmap<tri,bool>(2*P.size())),
    edges(hashmap<edge,triangle_ptr>(6*P.size())), n(P.size()) {
    points = P;
    point p0{n,0.0,100.0};
    point p1{n+1,100.0,-100.0};
    point p2{n+2,-100.0,-100.0};
    points = parlay::append(P, Points({p0, p1, p2}));
    auto t = std::make_shared<triangle>(tri{n, n+1, n+2}, P);
    n += 3;
    auto te = std::make_shared<triangle>(tri{-1, -1, -1}, Points());
    std::shared_ptr<triangle> te2, te3, t2, t3;
    t2 = t3 = t;
    te2 = te3 = te;
    parlay::par_do3([&] () mutable {process_edge(t2, edge{p0.id,p1.id}, te2);},
		    [&] () mutable {process_edge(t3, edge{p1.id,p2.id}, te3);},
		    [&] {process_edge(t, edge{p2.id,p0.id}, te);});
    std::cout << mesh.size() << std::endl;
  }
};
   
// **************************************************************
// Driver code
// **************************************************************
int main(int argc, char* argv[]) {
  auto usage = "Usage: delaunay <n>";
  if (argc != 2) std::cout << usage << std::endl;
  else {
    point_id n;
    try {n = std::stoi(argv[1]);}
    catch (...) {std::cout << usage << std::endl; return 1;}
    parlay::random r;

    // generate n random points in a unit square
    auto points = parlay::tabulate(n, [&] (point_id i) -> point {
	return point{i, (r[2*i] % 1000000000)/1000000000.0,
	    (r[2*i+1] % 1000000000)/1000000000.0};});

    parlay::internal::timer t;
    for (int i=0; i < 3; i++) {
      Delaunay dd(points);
      t.next("delaunay");
    }
    //std::cout << "number of points in upper hull = " << results.size() << std::endl;
  }
}
