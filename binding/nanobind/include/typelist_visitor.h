#pragma once

// XXX: Make it independent from nanobind and move to mc_rtc

#include <nanobind/nanobind.h>
#include <mc_rtc/type_name.h>
#include <mc_rtc/Configuration.h>

namespace nb = nanobind;

namespace mc_rtc
{
    namespace internal
    {
        // Compile-time iteration over a typelist
        // Applies a Visitor for each type T

        template <typename... Args> struct TypeList { };

        namespace Detail {

            // Forward declare, in order to pattern match via specialization
            template <typename ...> struct TypeVisitor;

            // Pattern match to extract the TypeLists types into Args
            template <template <typename ...> class Sequence, typename... Args>
                struct TypeVisitor<Sequence<Args...>> {

                    template <typename F>
                        static constexpr void Visit(nb::module_ & m) {
                            // Start recursive visitation for each type
                            DoVisit<F, Args...>(m);
                        }


                    // Allow empty sequence
                    template <typename F>
                        static constexpr void DoVisit(nb::module_ & m) { }

                    // Base case: one type, invoke functor
                    template <typename F, typename Head>
                        static constexpr void DoVisit(nb::module_ & m) {
                            F::template Visit<Head>(m);
                        }

                    // At least [Head, Next], Tail... can be empty
                    template <typename F, typename Head, typename Next, typename... Tail>
                        static constexpr void DoVisit(nb::module_ & m) {
                            // Visit head and recurse visitation on rest
                            DoVisit<F, Head>(m), DoVisit<F, Next, Tail...>(m);
                        }

                };
        } // End Detail


        // Invokes the functor with every type, this code generation is done at compile time
        template <typename Sequence, typename F>
            constexpr void ForEach(nb::module_ & m) { Detail::TypeVisitor<Sequence>::template Visit<F>(m); }



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
    }
}
