#pragma once

namespace mc_rtc
{
namespace internal
{
// Compile-time iteration over a typelist
// Applies a Visitor for each type T

template<typename... params>
struct TypeList
{
};

namespace Detail
{

// Forward declare, in order to pattern match via specialization
template<typename...>
struct TypeVisitor;

// Pattern match to extract the TypeLists types into params
template<template<typename...> class Sequence, typename... params>
struct TypeVisitor<Sequence<params...>>
{

  template<typename F, typename Param>
  static constexpr void Visit(Param & param)
  {
    // Start recursive visitation for each type
    DoVisit<F, Param, params...>(param);
  }

  // Allow empty sequence
  template<typename F, typename Param>
  static constexpr void DoVisit(Param & param)
  {
  }

  // Base case: one type, invoke functor
  template<typename F, typename Param, typename Head>
  static constexpr void DoVisit(Param & param)
  {
    F::template Visit<Head, Param>(param);
  }

  // At least [Head, Next], Tail... can be empty
  template<typename F, typename Param, typename Head, typename Next, typename... Tail>
  static constexpr void DoVisit(Param & param)
  {
    // Visit head and recurse visitation on rest
    DoVisit<F, Param, Head>(param), DoVisit<F, Param, Next, Tail...>(param);
  }
};
} // namespace Detail

// Invokes the functor with every type, this code generation is done at compile time
template<typename Sequence, typename F, typename Param>
constexpr void ForEach(Param & param)
{
  Detail::TypeVisitor<Sequence>::template Visit<F, Param>(param);
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
