#pragma once
#include <cstdint>
#include <tuple>

namespace mc_rtc
{
namespace internal
{

// Compile-time iteration over a typelist
// See https://github.com/lipk/cpp-typelist/blob/master/typelist.hpp and tmplbook for inspiration

template<typename... Args>
struct TypeList
{
  /**
   * Wraps adds a surrounding type around each element in a TypeList, e.g
   *
   * ```
   * using simple = TypeList<int, std::vector<float>, std::string>;
   * using ptrs = simple::wrap<std::shared_ptr>;
   * ```
   *
   * ptrs is now TypeList<std::shared_ptr<int>, std::shared_ptr<std::vector<float>>, std::shared_ptr<std::string>>;
   */
  template<template<typename> class W>
  using wrap = TypeList<W<Args>...>;

  /**
   * map is the generic form of wrap.
   * It takes some M template class as input, too, but it outputs M<T>::type for each T element,
   * instead of simply M<T>.
   * Example: convert a list of scalar types into a list of std::vectors.
   *
   * ```
   *   template <typename T>
   *   struct wrap_in_vector { using type = std::vector<T>; };
   *   using simple = type_list<double, int, float>;
   *   using vecs = simple::map<wrap_in_vector>;
   * ```
   *
   */
  template<template<typename> class M>
  using map = TypeList<typename M<Args>::type...>;
};

template<typename ta, typename tb>
struct type_cat;

// Concatenates two type_list
template<typename... a, typename... b>
struct type_cat<TypeList<a...>, TypeList<b...>>
{
  typedef TypeList<a..., b...> type;
};

/**
 * Applies a Visitor for each type T
 */
namespace Detail
{

// Forward declare, in order to pattern match via specialization
template<typename...>
struct TypeVisitor;

// Pattern match to extract the TypeLists types into Args
template<template<typename...> class Sequence, typename... Args>
struct TypeVisitor<Sequence<Args...>>
{

  template<typename F, typename Param>
  static constexpr void Visit(Param & arg)
  {
    // Start recursive visitation for each type
    DoVisit<F, Param, Args...>(arg);
  }

  // Allow empty sequence
  template<typename F, typename Param>
  static constexpr void DoVisit(Param & arg)
  {
  }

  // Base case: one type, invoke functor
  template<typename F, typename Param, typename Head>
  static constexpr void DoVisit(Param & arg)
  {
    F::template Visit<Head, Param>(arg);
  }

  // At least [Head, Next], Tail... can be empty
  template<typename F, typename Param, typename Head, typename Next, typename... Tail>
  static constexpr void DoVisit(Param & arg)
  {
    // Visit head and recurse visitation on rest
    DoVisit<F, Param, Head>(arg), DoVisit<F, Param, Next, Tail...>(arg);
  }
};
} // namespace Detail

// Invokes the functor with every type, this code generation is done at compile time
template<typename Sequence, typename F, typename Param>
constexpr void ForEach(Param & arg)
{
  Detail::TypeVisitor<Sequence>::template Visit<F, Param>(arg);
}

// User defined visitor that checks type sizes with specializations for int and double
// struct Visitor {
//   template <typename T> static constexpr void Visit(nb::module_ &) { }
// };
//
// template<> constexpr void Visitor::Visit<int>(nb::module_ & ) {
//   static_assert(sizeof(int) == 4, "");
// }
//
// template<> constexpr void Visitor::Visit<double>(nb::module_ & ) {
//   static_assert(sizeof(double) == 8, "");
// }

} // namespace internal
} // namespace mc_rtc
