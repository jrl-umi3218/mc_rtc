#pragma once

namespace mc_rtc
{
namespace internal
{
// Compile-time iteration over a typelist
// Applies a Visitor for each type T

template<typename... Args>
struct TypeList
{
};

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
