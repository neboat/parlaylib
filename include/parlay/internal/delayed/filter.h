#ifndef PARLAY_INTERNAL_DELAYED_FILTER_H_
#define PARLAY_INTERNAL_DELAYED_FILTER_H_

#include <type_traits>

#include "../../range.h"
#include "../../type_traits.h"
#include "../../utilities.h"

#include "../sequence_ops.h"

#include "common.h"
#include "flatten.h"
#include "map.h"

namespace parlay {
namespace internal {
namespace delayed {

template<typename UnderlyingView, typename UnaryPredicate>
struct block_delayed_filter_t :
    public block_iterable_view_base<UnderlyingView, block_delayed_filter_t<UnderlyingView, UnaryPredicate>> {

 private:
  using base = block_iterable_view_base<UnderlyingView, block_delayed_filter_t<UnderlyingView, UnaryPredicate>>;
  using base::base_view;

  static_assert(is_block_iterable_range_v<UnderlyingView>);
  static_assert(std::is_invocable_r_v<bool, UnaryPredicate, range_reference_type_t<UnderlyingView>>);

  using block_type = sequence<range_iterator_type_t<UnderlyingView>>;
  using block_result_type = sequence<block_type>;
  using flattener_type = block_delayed_flatten_t<block_result_type>;
  using mapper_type = block_delayed_map_t<flattener_type, dereference>;

 public:
  using value_type = range_value_type_t<UnderlyingView>;
  using reference = range_reference_type_t<UnderlyingView>;

  template<typename UV, typename UP>
  block_delayed_filter_t(UV&& v, UP&& p) : base(std::forward<UV>(v)),
    result(flattener_type{filter_blocks(base_view(), std::forward<UP>(p)), 0}, dereference{}) {

  }

  // Returns the number of blocks
  auto get_num_blocks() const { return result.get_num_blocks(); }

  // Return an iterator pointing to the beginning of block i
  auto get_begin_block(size_t i) { return result.get_begin_block(i); }
  auto get_begin_block(size_t i) const { return result.get_begin_block(i); }

  auto get_end_block(size_t i) { return get_begin_block(i+1); }
  auto get_end_block(size_t i) const { return get_begin_block(i+1); }

  auto begin() { return get_begin_block(0); }
  auto begin() const { return get_begin_block(0); }

  auto end() { return get_begin_block(get_num_blocks()); }
  auto end() const { return get_begin_block(get_num_blocks()); }

  [[nodiscard]] size_t size() const { return result.size(); }

 private:
  template<typename UV, typename UP>
  block_result_type filter_blocks(UV&& v, UP&& p) {
    return parlay::internal::tabulate(num_blocks(v), [&](size_t i) -> block_type {
      block_type res;
      for (auto it = begin_block(v, i); it != end_block(v, i); ++it) {
        if (p(*it)) {
          res.emplace_back(std::move(it));
        }
      }
      return res;
    });
  }

  mapper_type result;
};

template<typename UnderlyingView, typename UnaryPredicate>
auto filter(UnderlyingView&& v, UnaryPredicate&& p) {
  static_assert(is_block_iterable_range_v<UnderlyingView>);
  static_assert(std::is_invocable_r_v<bool, UnaryPredicate, range_reference_type_t<UnderlyingView>>);
  return block_delayed_filter_t<UnderlyingView, UnaryPredicate>(
      std::forward<UnderlyingView>(v), std::forward<UnaryPredicate>(p));
}

}  // namespace delayed
}  // namespace internal
}  // namespace parlay

#endif  // PARLAY_INTERNAL_DELAYED_FILTER_H_