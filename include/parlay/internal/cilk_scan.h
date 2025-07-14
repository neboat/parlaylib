#ifndef _CILK_SCAN_
#define _CILK_SCAN_

#include <cilk/cilk_api.h>
#include <cilk/reducer>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <type_traits>
#include <unistd.h>

struct range_t {
  ssize_t start = 0;
  ssize_t end = 0;
};

template <typename V> __cilk_identity_fn id_default = [](void *v) -> void {
  static_assert(std::is_default_constructible_v<V>,
                "id_default only works with default constructible types");
  *reinterpret_cast<V *>(v) = V{};
//   new (v) V;
};

template <typename V>
std::function<V(V *, V *)> reduce_default =
    [](V *l, V *r) -> V { return *l + *r; };

template <typename T>
using container_value_t = std::__remove_cvref_t<decltype(std::declval<T>()[0])>;

template <typename T>
struct scanner {
  using V = container_value_t<T>;
  using IdFnTy = __cilk_identity_fn;
  using ReduceFnTy = std::function<V(V*,V*)>;
//   static_assert(std::is_trivially_copyable_v<V>,
//                 "scanner only works with trivially copyable types");

  IdFnTy &value_id;
  ReduceFnTy &value_reduce;

  T &array;
  V sum;
  bool inclusive = true;

  // TODO: With compiler support, I think the range can be maintained
  // automatically.
  range_t r{.start = -1, .end = 1};

  // These fields maintain the tree structure for the down-sweep phase.
  scanner *l_child = nullptr, *r_child = nullptr;
  bool is_leftmost = true;

  __attribute__((always_inline)) inline void
  value_reduce_to_right(void *left, void *right) {
    *reinterpret_cast<V *>(right) = value_reduce(reinterpret_cast<V *>(left), reinterpret_cast<V *>(right));
  }

  // Helper routine to perform down-sweep.
  void down_sweep(V &prefix) {
    if (!l_child && !r_child) {
      // At a leaf, broadcast the prefix over the range of the array.
      value_reduce_to_right(&prefix, &array[r.end]);
      cilk_for(size_t i = r.start; i < r.end; ++i) {
        value_reduce_to_right(&prefix, &array[i]);
      }
    } else {
      cilk_scope {
        // Both l_child and r_child should be non-null.
        // Add the prefix to the end of l_child's range, to compute the
        // prefix for r_child.
        V r_prefix = value_reduce(&prefix, &l_child->sum);
        // Recursively down-sweep l_child and r_child in parallel.
        cilk_spawn l_child->down_sweep(prefix);
        r_child->down_sweep(r_prefix);
      }
      delete l_child;
      delete r_child;
    }
  }

  void identity_fn(void *v) {
    auto *sr = new (v) scanner(array, value_id, value_reduce, inclusive);
    sr->r.start = -1;
    sr->r.end = -1;

    // No view created by the identity function is leftmost.
    sr->is_leftmost = false;
  }
  __cilk_identity_fn identity = [this](void *v) -> void { identity_fn(v); };

  void reduce_fn(void *l, void *r) {
    auto *lsr = static_cast<scanner *>(l);
    auto *rsr = static_cast<scanner *>(r);
    // Perform up-sweep.
    if (lsr->is_leftmost) {
      // Only trigger down-sweep when reducing with the leftmost view.
      // Otherwise this hyperobject does too much total work.
      rsr->down_sweep(lsr->sum);
    } else {
      // Create a tree node with lsr and rsr as children.

      // TODO: We create new scan_reducer views here to avoid problems
      // with the runtime system freeing views implicitly.  Find a better
      // solution than allocating new nodes here.
      auto *l_node = new scanner(*lsr);
      auto *r_node = new scanner(*rsr);
      lsr->l_child = l_node;
      lsr->r_child = r_node;
    }
    // The resulting left view covers the full range.
    lsr->r.end = rsr->r.end;
    lsr->sum = value_reduce(&lsr->sum, &rsr->sum);
  }
  __cilk_reduce_fn reduce = [this](void *l, void *r) -> void { reduce_fn(l, r); };

  explicit scanner(T &array, IdFnTy &value_id, ReduceFnTy &value_reduce, bool inclusive) :
      value_id(value_id), value_reduce(value_reduce), array(array), inclusive(inclusive) {
    if (std::is_destructible<V>::value) sum.~V();
    value_id(&sum);
  }

  struct view_proxy {
    size_t idx;
    scanner *sr;

    view_proxy(size_t idx, scanner *sr) : idx(idx), sr(sr) {
      if (!sr->inclusive) sr->array[idx] = sr->sum;
    }
    ~view_proxy() {
      if (sr->inclusive) sr->array[idx] = sr->sum;
      sr->r.end = idx;
    }
    view_proxy(const view_proxy &) = delete;
    view_proxy(view_proxy &&other) = delete;
    auto operator=(const view_proxy &) -> view_proxy & = delete;
    auto operator=(view_proxy &&other) -> view_proxy & = delete;
    auto operator*() -> V & { return sr->sum; }
    auto operator->() -> V * { return &sr->sum; }
    operator V() const { return sr->sum; }
  };

  auto view(size_t idx) -> view_proxy {
    if (r.start == -1) {
      this->r.start = static_cast<ssize_t>(idx);
      this->r.end = static_cast<ssize_t>(idx);
    }
    return {idx, this};
  }
};

#endif // _CILK_SCAN_