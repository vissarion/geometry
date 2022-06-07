// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2014 Barend Gehrels, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2014-2022.
// Modifications copyright (c) 2014-2022 Oracle and/or its affiliates.

// Contributed and/or modified by Menelaos Karavelas, on behalf of Oracle
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_UNION_HPP
#define BOOST_GEOMETRY_ALGORITHMS_UNION_HPP


#include <boost/range/value_type.hpp>

#include <boost/geometry/algorithms/detail/intersection/gc.hpp>
#include <boost/geometry/algorithms/detail/intersection/multi.hpp>
#include <boost/geometry/algorithms/detail/make_rtree.hpp>
#include <boost/geometry/algorithms/detail/overlay/intersection_insert.hpp>
#include <boost/geometry/algorithms/detail/overlay/linear_linear.hpp>
#include <boost/geometry/algorithms/detail/overlay/overlay.hpp>
#include <boost/geometry/algorithms/detail/overlay/pointlike_pointlike.hpp>
#include <boost/geometry/algorithms/not_implemented.hpp>
#include <boost/geometry/core/point_order.hpp>
#include <boost/geometry/core/reverse_dispatch.hpp>
#include <boost/geometry/geometries/adapted/boost_variant.hpp>
#include <boost/geometry/geometries/concepts/check.hpp>
#include <boost/geometry/policies/robustness/get_rescale_policy.hpp>
#include <boost/geometry/strategies/default_strategy.hpp>
#include <boost/geometry/strategies/detail.hpp>
#include <boost/geometry/strategies/relate/cartesian.hpp>
#include <boost/geometry/strategies/relate/geographic.hpp>
#include <boost/geometry/strategies/relate/spherical.hpp>
#include <boost/geometry/util/range.hpp>
#include <boost/geometry/util/type_traits_std.hpp>
#include <boost/geometry/views/detail/geometry_collection_view.hpp>
#include <boost/geometry/views/detail/random_access_view.hpp>


namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut,
    typename TagIn1 = typename tag<Geometry1>::type,
    typename TagIn2 = typename tag<Geometry2>::type,
    typename TagOut = typename detail::setop_insert_output_tag<GeometryOut>::type,
    typename CastedTagIn1 = typename geometry::tag_cast<TagIn1, areal_tag, linear_tag, pointlike_tag>::type,
    typename CastedTagIn2 = typename geometry::tag_cast<TagIn2, areal_tag, linear_tag, pointlike_tag>::type,
    typename CastedTagOut = typename geometry::tag_cast<TagOut, areal_tag, linear_tag, pointlike_tag>::type,
    bool Reverse = geometry::reverse_dispatch<Geometry1, Geometry2>::type::value
>
struct union_insert: not_implemented<TagIn1, TagIn2, TagOut>
{};


// If reversal is needed, perform it first

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut,
    typename TagIn1, typename TagIn2, typename TagOut,
    typename CastedTagIn1, typename CastedTagIn2, typename CastedTagOut
>
struct union_insert
    <
        Geometry1, Geometry2, GeometryOut,
        TagIn1, TagIn2, TagOut,
        CastedTagIn1, CastedTagIn2, CastedTagOut,
        true
    >
{
    template <typename RobustPolicy, typename OutputIterator, typename Strategy>
    static inline OutputIterator apply(Geometry1 const& g1,
                                       Geometry2 const& g2,
                                       RobustPolicy const& robust_policy,
                                       OutputIterator out,
                                       Strategy const& strategy)
    {
        return union_insert
            <
                Geometry2, Geometry1, GeometryOut
            >::apply(g2, g1, robust_policy, out, strategy);
    }
};


template
<
    typename Geometry1, typename Geometry2, typename GeometryOut,
    typename TagIn1, typename TagIn2, typename TagOut
>
struct union_insert
    <
        Geometry1, Geometry2, GeometryOut,
        TagIn1, TagIn2, TagOut,
        areal_tag, areal_tag, areal_tag,
        false
    > : detail::overlay::overlay
        <
            Geometry1, Geometry2,
            detail::overlay::do_reverse<geometry::point_order<Geometry1>::value>::value,
            detail::overlay::do_reverse<geometry::point_order<Geometry2>::value>::value,
            detail::overlay::do_reverse<geometry::point_order<GeometryOut>::value>::value,
            GeometryOut,
            overlay_union
        >
{};


// dispatch for union of linear geometries
template
<
    typename Linear1, typename Linear2, typename LineStringOut,
    typename TagIn1, typename TagIn2
>
struct union_insert
    <
        Linear1, Linear2, LineStringOut,
        TagIn1, TagIn2, linestring_tag,
        linear_tag, linear_tag, linear_tag,
        false
    > : detail::overlay::linear_linear_linestring
        <
            Linear1, Linear2, LineStringOut, overlay_union
        >
{};


// dispatch for point-like geometries
template
<
    typename PointLike1, typename PointLike2, typename PointOut,
    typename TagIn1, typename TagIn2
>
struct union_insert
    <
        PointLike1, PointLike2, PointOut,
        TagIn1, TagIn2, point_tag,
        pointlike_tag, pointlike_tag, pointlike_tag,
        false
    > : detail::overlay::union_pointlike_pointlike_point
        <
            PointLike1, PointLike2, PointOut
        >
{};


template
<
    typename Geometry1, typename Geometry2, typename SingleTupledOut,
    typename TagIn1, typename TagIn2,
    typename CastedTagIn
>
struct union_insert
    <
        Geometry1, Geometry2, SingleTupledOut,
        TagIn1, TagIn2, detail::tupled_output_tag,
        CastedTagIn, CastedTagIn, detail::tupled_output_tag,
        false
    >
{
    typedef typename geometry::detail::single_tag_from_base_tag
        <
            CastedTagIn
        >::type single_tag;

    typedef detail::expect_output
        <
            Geometry1, Geometry2, SingleTupledOut, single_tag
        > expect_check;

    typedef typename geometry::detail::output_geometry_access
        <
            SingleTupledOut, single_tag, single_tag
        > access;

    template <typename RobustPolicy, typename OutputIterator, typename Strategy>
    static inline OutputIterator apply(Geometry1 const& g1,
                                       Geometry2 const& g2,
                                       RobustPolicy const& robust_policy,
                                       OutputIterator out,
                                       Strategy const& strategy)
    {
        access::get(out) = union_insert
            <
                Geometry2, Geometry1, typename access::type
            >::apply(g2, g1, robust_policy, access::get(out), strategy);

        return out;
    }
};


template
<
    typename Geometry1, typename Geometry2, typename SingleTupledOut,
    typename SingleTag1, typename SingleTag2,
    bool Geometry1LesserTopoDim = (topological_dimension<Geometry1>::value
                                    < topological_dimension<Geometry2>::value)
>
struct union_insert_tupled_different
{
    typedef typename geometry::detail::output_geometry_access
        <
            SingleTupledOut, SingleTag1, SingleTag1
        > access1;

    typedef typename geometry::detail::output_geometry_access
        <
            SingleTupledOut, SingleTag2, SingleTag2
        > access2;

    template <typename RobustPolicy, typename OutputIterator, typename Strategy>
    static inline OutputIterator apply(Geometry1 const& g1,
                                       Geometry2 const& g2,
                                       RobustPolicy const& robust_policy,
                                       OutputIterator out,
                                       Strategy const& strategy)
    {
        access1::get(out) = geometry::dispatch::intersection_insert
            <
                Geometry1, Geometry2,
                typename access1::type,
                overlay_difference,
                geometry::detail::overlay::do_reverse<geometry::point_order<Geometry1>::value>::value,
                geometry::detail::overlay::do_reverse<geometry::point_order<Geometry2>::value, true>::value
            >::apply(g1, g2, robust_policy, access1::get(out), strategy);

        access2::get(out) = geometry::detail::convert_to_output
            <
                Geometry2,
                typename access2::type
            >::apply(g2, access2::get(out));

        return out;
    }
};


template
<
    typename Geometry1, typename Geometry2, typename SingleTupledOut,
    typename SingleTag1, typename SingleTag2
>
struct union_insert_tupled_different
    <
        Geometry1, Geometry2, SingleTupledOut, SingleTag1, SingleTag2, false
    >
{
    template <typename RobustPolicy, typename OutputIterator, typename Strategy>
    static inline OutputIterator apply(Geometry1 const& g1,
                                       Geometry2 const& g2,
                                       RobustPolicy const& robust_policy,
                                       OutputIterator out,
                                       Strategy const& strategy)
    {
        return union_insert_tupled_different
            <
                Geometry2, Geometry1, SingleTupledOut, SingleTag2, SingleTag1, true
            >::apply(g2, g1, robust_policy, out, strategy);
    }
};


template
<
    typename Geometry1, typename Geometry2, typename SingleTupledOut,
    typename TagIn1, typename TagIn2,
    typename CastedTagIn1, typename CastedTagIn2
>
struct union_insert
    <
        Geometry1, Geometry2, SingleTupledOut,
        TagIn1, TagIn2, detail::tupled_output_tag,
        CastedTagIn1, CastedTagIn2, detail::tupled_output_tag,
        false
    >
{
    typedef typename geometry::detail::single_tag_from_base_tag
        <
            CastedTagIn1
        >::type single_tag1;

    typedef detail::expect_output
        <
            Geometry1, Geometry2, SingleTupledOut, single_tag1
        > expect_check1;

    typedef typename geometry::detail::single_tag_from_base_tag
        <
            CastedTagIn2
        >::type single_tag2;

    typedef detail::expect_output
        <
            Geometry1, Geometry2, SingleTupledOut, single_tag2
        > expect_check2;

    template <typename RobustPolicy, typename OutputIterator, typename Strategy>
    static inline OutputIterator apply(Geometry1 const& g1,
                                       Geometry2 const& g2,
                                       RobustPolicy const& robust_policy,
                                       OutputIterator out,
                                       Strategy const& strategy)
    {
        return union_insert_tupled_different
            <
                Geometry1, Geometry2, SingleTupledOut, single_tag1, single_tag2
            >::apply(g1, g2, robust_policy, out, strategy);
    }
};


} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace union_
{

/*!
\brief_calc2{union}
\ingroup union
\details \details_calc2{union_insert, spatial set theoretic union}.
    \details_insert{union}
\tparam GeometryOut output geometry type, must be specified
\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\tparam OutputIterator output iterator
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\param out \param_out{union}
\return \return_out
*/
template
<
    typename GeometryOut,
    typename Geometry1,
    typename Geometry2,
    typename OutputIterator
>
inline OutputIterator union_insert(Geometry1 const& geometry1,
            Geometry2 const& geometry2,
            OutputIterator out)
{
    concepts::check<Geometry1 const>();
    concepts::check<Geometry2 const>();
    geometry::detail::output_geometry_concept_check<GeometryOut>::apply();

    typename strategies::relate::services::default_strategy
        <
            Geometry1, Geometry2
        >::type strategy;

    typedef typename geometry::rescale_overlay_policy_type
        <
            Geometry1,
            Geometry2
        >::type rescale_policy_type;

    rescale_policy_type robust_policy
            = geometry::get_rescale_policy<rescale_policy_type>(
                geometry1, geometry2, strategy);

    return dispatch::union_insert
           <
               Geometry1, Geometry2, GeometryOut
           >::apply(geometry1, geometry2, robust_policy, out, strategy);
}


struct gc_element_id
{
    gc_element_id(unsigned int source_id_, std::size_t gc_id_)
        : source_id(source_id_), gc_id(gc_id_)
    {}

    unsigned int source_id;
    std::size_t gc_id;

    friend bool operator<(gc_element_id const& left, gc_element_id const& right)
    {
        return left.source_id < right.source_id
            || (left.source_id == right.source_id && left.gc_id < right.gc_id);
    }
};

template <typename GC1View, typename GC2View, typename Strategy, typename IntersectingFun, typename DisjointFun>
inline void gc_group_elements(GC1View const& gc1_view, GC2View const& gc2_view, Strategy const& strategy,
                              IntersectingFun&& intersecting_fun,
                              DisjointFun&& disjoint_fun,
                              bool group_self = false)
{
    // NOTE: could be replaced with unordered_map and unordered_set
    std::map<gc_element_id, std::set<gc_element_id>> adjacent;

    // Create adjacency list based on intersecting envelopes of GC elements
    auto const rtree2 = detail::make_rtree_indexes(gc2_view, strategy);
    if (! group_self) // group only elements from the other GC?
    {
        for (std::size_t i = 0; i < boost::size(gc1_view); ++i)
        {
            traits::iter_visit<GC1View>::apply([&](auto const& g1)
            {
                using g1_t = util::remove_cref_t<decltype(g1)>;
                using box1_t = detail::make_rtree_box_t<g1_t>;
                box1_t b1 = geometry::return_envelope<box1_t>(g1, strategy);
                detail::expand_by_epsilon(b1);

                gc_element_id id1 = {0, i};
                for (auto qit = rtree2.qbegin(index::intersects(b1)); qit != rtree2.qend(); ++qit)
                {
                    gc_element_id id2 = {1, qit->second};
                    adjacent[id1].insert(id2);
                    adjacent[id2].insert(id1);
                }
            }, boost::begin(gc1_view) + i);
        }
    }
    else // group elements from the same GCs and the other GC
    {
        auto const rtree1 = detail::make_rtree_indexes(gc1_view, strategy);
        for (auto it1 = rtree1.begin() ; it1 != rtree1.end() ; ++it1)
        {
            auto const b1 = it1->first;
            gc_element_id id1 = {0, it1->second};
            for (auto qit2 = rtree2.qbegin(index::intersects(b1)); qit2 != rtree2.qend(); ++qit2)
            {
                gc_element_id id2 = {1, qit2->second};
                adjacent[id1].insert(id2);
                adjacent[id2].insert(id1);
            }
            for (auto qit1 = rtree1.qbegin(index::intersects(b1)); qit1 != rtree1.qend(); ++qit1)
            {
                if (id1.gc_id != qit1->second)
                {
                    gc_element_id id11 = {0, qit1->second};
                    adjacent[id1].insert(id11);
                    adjacent[id11].insert(id1);
                }
            }
        }
        for (auto it2 = rtree2.begin(); it2 != rtree2.end(); ++it2)
        {
            auto const b2 = it2->first;
            gc_element_id id2 = {1, it2->second};
            for (auto qit2 = rtree2.qbegin(index::intersects(b2)); qit2 != rtree2.qend(); ++qit2)
            {
                if (id2.gc_id != qit2->second)
                {
                    gc_element_id id22 = {1, qit2->second};
                    adjacent[id2].insert(id22);
                    adjacent[id22].insert(id2);
                }
            }
        }
    }

    // Traverse the graph and build connected groups i.e. groups of intersecting envelopes
    std::deque<gc_element_id> queue;
    std::array<std::vector<bool>, 2> visited = {
        std::vector<bool>(boost::size(gc1_view), false),
        std::vector<bool>(boost::size(gc2_view), false)
    };        
    for (auto const& elem : adjacent)
    {
        std::vector<gc_element_id> group;
        if (! visited[elem.first.source_id][elem.first.gc_id])
        {
            queue.push_back(elem.first);
            visited[elem.first.source_id][elem.first.gc_id] = true;
            group.push_back(elem.first);
            while (! queue.empty())
            {
                gc_element_id e = queue.front();
                queue.pop_front();
                for (auto const& n : adjacent[e])
                {
                    if (! visited[n.source_id][n.gc_id])
                    {
                        queue.push_back(n);
                        visited[n.source_id][n.gc_id] = true;
                        group.push_back(n);
                    }
                }
            }
        }
        if (! group.empty())
        {
            intersecting_fun(group);
        }
    }

    {
        std::vector<gc_element_id> group;
        for (std::size_t i = 0; i < visited[0].size(); ++i)
        {
            if (! visited[0][i])
            {
                group.emplace_back(0, i);
            }
        }
        for (std::size_t i = 0; i < visited[1].size(); ++i)
        {
            if (! visited[1][i])
            {
                group.emplace_back(1, i);
            }
        }
        if (! group.empty())
        {
            disjoint_fun(group);
        }
    }
}


}} // namespace detail::union_
#endif // DOXYGEN_NO_DETAIL


namespace resolve_collection
{

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut,
    typename Tag1 = typename geometry::tag<Geometry1>::type,
    typename Tag2 = typename geometry::tag<Geometry2>::type,
    typename TagOut = typename geometry::tag<GeometryOut>::type
>
struct union_
{
    template <typename Strategy>
    static void apply(Geometry1 const& geometry1, Geometry2 const& geometry2,
                      GeometryOut & geometry_out, Strategy const& strategy)
    {
        typedef typename geometry::detail::output_geometry_value
            <
            GeometryOut
            >::type single_out;

        typedef typename geometry::rescale_overlay_policy_type
            <
                Geometry1,
                Geometry2,
                typename Strategy::cs_tag
            >::type rescale_policy_type;

        rescale_policy_type robust_policy
                = geometry::get_rescale_policy<rescale_policy_type>(
                    geometry1, geometry2, strategy);

        dispatch::union_insert
           <
               Geometry1, Geometry2, single_out
           >::apply(geometry1, geometry2, robust_policy,
                    geometry::detail::output_geometry_back_inserter(geometry_out),
                    strategy);
    }
};

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut
>
struct union_
    <
        Geometry1, Geometry2, GeometryOut,
        geometry_collection_tag, geometry_collection_tag, geometry_collection_tag
    >
{
    // NOTE: for now require all of the possible output types
    //       technically only a subset could be needed.
    using multi_point_t = typename util::sequence_find_if
        <
            typename traits::geometry_types<GeometryOut>::type,
            util::is_multi_point
        >::type;
    using multi_linestring_t = typename util::sequence_find_if
        <
            typename traits::geometry_types<GeometryOut>::type,
            util::is_multi_linestring
        >::type;
    using multi_polygon_t = typename util::sequence_find_if
        <
            typename traits::geometry_types<GeometryOut>::type,
            util::is_multi_polygon
        >::type;
    using tuple_out_t = boost::tuple<multi_point_t, multi_linestring_t, multi_polygon_t>;

    template <typename Strategy>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             GeometryOut& geometry_out,
                             Strategy const& strategy)
    {
        detail::random_access_view<Geometry1 const> gc1_view(geometry1);
        detail::random_access_view<Geometry2 const> gc2_view(geometry2);
        
        detail::union_::gc_group_elements(gc1_view, gc2_view, strategy,
            [&](auto const& inters_group)
            {
                tuple_out_t out;
                merge_group(gc1_view, gc2_view, strategy, inters_group, out);
                detail::intersection::gc_move_multi_back(geometry_out, boost::get<0>(out));
                detail::intersection::gc_move_multi_back(geometry_out, boost::get<1>(out));
                detail::intersection::gc_move_multi_back(geometry_out, boost::get<2>(out));
            },
            [&](auto const& disjoint_group)
            {
                copy_disjoint(gc1_view, gc2_view, disjoint_group, geometry_out);
            });
    }

private:
    template <typename GC1View, typename GC2View, typename Strategy, typename Group>
    static inline void merge_group(GC1View const& gc1_view, GC2View const& gc2_view,
                                   Strategy const& strategy, Group const& inters_group,
                                   tuple_out_t& out)
    {
        for (auto const& id : inters_group)
        {
            if (id.source_id == 0)
            {
                traits::iter_visit<GC1View>::apply([&](auto const& g1)
                {
                    merge_one(out, g1, strategy);
                }, boost::begin(gc1_view) + id.gc_id);
            }
            else
            {
                traits::iter_visit<GC2View>::apply([&](auto const& g2)
                {
                    merge_one(out, g2, strategy);
                }, boost::begin(gc2_view) + id.gc_id);
            }
        }
        /*
        // L = L \ A
        {
            multi_linestring_t l;
            subtract_greater_topodim(boost::get<1>(out), boost::get<2>(out), l, strategy);
            boost::get<1>(out) = std::move(l);
        }
        // P = P \ A
        {
            multi_point_t p;
            subtract_greater_topodim(boost::get<0>(out), boost::get<2>(out), p, strategy);
            boost::get<0>(out) = std::move(p);
        }
        // P = P \ L
        {
            multi_point_t p;
            subtract_greater_topodim(boost::get<0>(out), boost::get<1>(out), p, strategy);
            boost::get<0>(out) = std::move(p);
        }
        */
    }

    template <typename G, typename Strategy, std::enable_if_t<util::is_pointlike<G>::value, int> = 0>
    static inline void merge_one(tuple_out_t& out, G const& g, Strategy const& strategy)
    {
        multi_point_t p;
        union_<multi_point_t, G, multi_point_t>::apply(boost::get<0>(out), g, p, strategy);
        boost::get<0>(out) = std::move(p);
    }

    template <typename G, typename Strategy, std::enable_if_t<util::is_linear<G>::value, int> = 0>
    static inline void merge_one(tuple_out_t& out, G const& g, Strategy const& strategy)
    {
        multi_linestring_t l;
        union_<multi_linestring_t, G, multi_linestring_t>::apply(boost::get<1>(out), g, l, strategy);
        boost::get<1>(out) = std::move(l);
    }

    template <typename G, typename Strategy, std::enable_if_t<util::is_areal<G>::value, int> = 0>
    static inline void merge_one(tuple_out_t& out, G const& g, Strategy const& strategy)
    {
        multi_polygon_t a;
        union_<multi_polygon_t, G, multi_polygon_t>::apply(boost::get<2>(out), g, a, strategy);
        boost::get<2>(out) = std::move(a);
    }

    template <typename GC1View, typename GC2View, typename Group>
    static inline void copy_disjoint(GC1View const& gc1_view, GC2View const& gc2_view,
                                     Group const& disjoint_group, GeometryOut& geometry_out)
    {
        for (auto const& id : disjoint_group)
        {
            if (id.source_id == 0)
            {
                traits::iter_visit<GC1View>::apply([&](auto const& g1)
                {
                    copy_one(g1, geometry_out);
                }, boost::begin(gc1_view) + id.gc_id);
            }
            else
            {
                traits::iter_visit<GC2View>::apply([&](auto const& g2)
                {
                    copy_one(g2, geometry_out);
                }, boost::begin(gc2_view) + id.gc_id);
            }
        }
    }

    template <typename G, std::enable_if_t<util::is_pointlike<G>::value, int> = 0>
    static inline void copy_one(G const& g, GeometryOut& geometry_out)
    {
        multi_point_t p;
        geometry::convert(g, p);
        detail::intersection::gc_move_multi_back(geometry_out, p);
    }

    template <typename G, std::enable_if_t<util::is_linear<G>::value, int> = 0>
    static inline void copy_one(G const& g, GeometryOut& geometry_out)
    {
        multi_linestring_t l;
        geometry::convert(g, l);
        detail::intersection::gc_move_multi_back(geometry_out, l);
    }

    template <typename G, std::enable_if_t<util::is_areal<G>::value, int> = 0>
    static inline void copy_one(G const& g, GeometryOut& geometry_out)
    {
        multi_polygon_t a;
        geometry::convert(g, a);
        detail::intersection::gc_move_multi_back(geometry_out, a);
    }
    /*
    template <typename Multi1, typename Multi2, typename Strategy>
    static inline void subtract_greater_topodim(Multi1 const& multi1, Multi2 const& multi2, Multi1& multi_out, Strategy const& strategy)
    {
        using rescale_policy_type = typename geometry::rescale_overlay_policy_type
            <
                Multi1, Multi2
            >::type;

        rescale_policy_type robust_policy
                = geometry::get_rescale_policy<rescale_policy_type>(
                    multi1, multi2, strategy);

        geometry::dispatch::intersection_insert
            <
                Multi1, Multi2,
                typename boost::range_value<Multi1>::type,
                overlay_difference,
                geometry::detail::overlay::do_reverse<geometry::point_order<Multi1>::value>::value,
                geometry::detail::overlay::do_reverse<geometry::point_order<Multi2>::value, true>::value
            >::apply(multi1, multi2, robust_policy, range::back_inserter(multi_out), strategy);
    }
    */
};

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut, typename Tag1
>
struct union_
    <
        Geometry1, Geometry2, GeometryOut,
        Tag1, geometry_collection_tag, geometry_collection_tag
    >
{
    template <typename Strategy>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             GeometryOut& geometry_out,
                             Strategy const& strategy)
    {
        using gc_view_t = geometry::detail::geometry_collection_view<Geometry1>;
        union_
            <
                gc_view_t, Geometry2, GeometryOut
            >::apply(gc_view_t(geometry1), geometry2, geometry_out, strategy);
    }
};

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut, typename Tag2
>
struct union_
    <
        Geometry1, Geometry2, GeometryOut,
        geometry_collection_tag, Tag2, geometry_collection_tag
    >
{
    template <typename Strategy>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             GeometryOut& geometry_out,
                             Strategy const& strategy)
    {
        using gc_view_t = geometry::detail::geometry_collection_view<Geometry2>;
        union_
            <
                Geometry1, gc_view_t, GeometryOut
            >::apply(geometry1, gc_view_t(geometry2), geometry_out, strategy);
    }
};

template
<
    typename Geometry1, typename Geometry2, typename GeometryOut, typename Tag1, typename Tag2
>
struct union_
    <
        Geometry1, Geometry2, GeometryOut,
        Tag1, Tag2, geometry_collection_tag
    >
{
    template <typename Strategy>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             GeometryOut& geometry_out,
                             Strategy const& strategy)
    {
        using gc1_view_t = geometry::detail::geometry_collection_view<Geometry1>;
        using gc2_view_t = geometry::detail::geometry_collection_view<Geometry2>;
        union_
            <
                gc1_view_t, gc2_view_t, GeometryOut
            >::apply(gc1_view_t(geometry1), gc2_view_t(geometry2), geometry_out, strategy);
    }
};

} // namespace resolve_collection


namespace resolve_strategy {

template
<
    typename Strategy,
    bool IsUmbrella = strategies::detail::is_umbrella_strategy<Strategy>::value
>
struct union_
{
    template <typename Geometry1, typename Geometry2, typename Collection>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             Collection & output_collection,
                             Strategy const& strategy)
    {
        resolve_collection::union_
            <
                Geometry1, Geometry2, Collection
            >::apply(geometry1, geometry2, output_collection, strategy);
    }
};

template <typename Strategy>
struct union_<Strategy, false>
{
    template <typename Geometry1, typename Geometry2, typename Collection>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             Collection & output_collection,
                             Strategy const& strategy)
    {
        using strategies::relate::services::strategy_converter;

        union_
            <
                decltype(strategy_converter<Strategy>::get(strategy))
            >::apply(geometry1, geometry2, output_collection,
                     strategy_converter<Strategy>::get(strategy));
    }
};

template <>
struct union_<default_strategy, false>
{
    template <typename Geometry1, typename Geometry2, typename Collection>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             Collection & output_collection,
                             default_strategy)
    {
        typedef typename strategies::relate::services::default_strategy
            <
                Geometry1,
                Geometry2
            >::type strategy_type;

        union_
            <
                strategy_type
            >::apply(geometry1, geometry2, output_collection, strategy_type());
    }
};

} // resolve_strategy


namespace resolve_dynamic
{
    
template
<
    typename Geometry1, typename Geometry2,
    typename Tag1 = typename geometry::tag<Geometry1>::type,
    typename Tag2 = typename geometry::tag<Geometry2>::type
>
struct union_
{
    template <typename Collection, typename Strategy>
    static inline void apply(Geometry1 const& geometry1,
                             Geometry2 const& geometry2,
                             Collection& output_collection,
                             Strategy const& strategy)
    {
        concepts::check<Geometry1 const>();
        concepts::check<Geometry2 const>();
        //concepts::check<typename boost::range_value<Collection>::type>();
        geometry::detail::output_geometry_concept_check
            <
                typename geometry::detail::output_geometry_value
                    <
                        Collection
                    >::type
            >::apply();

        resolve_strategy::union_
            <
                Strategy
            >::apply(geometry1, geometry2, output_collection, strategy);
    }
};


template <typename DynamicGeometry1, typename Geometry2, typename Tag2>
struct union_<DynamicGeometry1, Geometry2, dynamic_geometry_tag, Tag2>
{
    template <typename Collection, typename Strategy>
    static inline void apply(DynamicGeometry1 const& geometry1, Geometry2 const& geometry2,
                             Collection& output_collection, Strategy const& strategy)
    {
        traits::visit<DynamicGeometry1>::apply([&](auto const& g1)
        {
            union_
                <
                    util::remove_cref_t<decltype(g1)>,
                    Geometry2
                >::apply(g1, geometry2, output_collection, strategy);
        }, geometry1);
    }
};


template <typename Geometry1, typename DynamicGeometry2, typename Tag1>
struct union_<Geometry1, DynamicGeometry2, Tag1, dynamic_geometry_tag>
{
    template <typename Collection, typename Strategy>
    static inline void apply(Geometry1 const& geometry1, DynamicGeometry2 const& geometry2,
                             Collection& output_collection, Strategy const& strategy)
    {
        traits::visit<DynamicGeometry2>::apply([&](auto const& g2)
        {
            union_
                <
                    Geometry1,
                    util::remove_cref_t<decltype(g2)>
                >::apply(geometry1, g2, output_collection, strategy);
        }, geometry2);
    }
};


template <typename DynamicGeometry1, typename DynamicGeometry2>
struct union_<DynamicGeometry1, DynamicGeometry2, dynamic_geometry_tag, dynamic_geometry_tag>
{
    template <typename Collection, typename Strategy>
    static inline void apply(DynamicGeometry1 const& geometry1, DynamicGeometry2 const& geometry2,
                             Collection& output_collection, Strategy const& strategy)
    {
        traits::visit<DynamicGeometry1, DynamicGeometry2>::apply([&](auto const& g1, auto const& g2)
        {
            union_
                <
                    util::remove_cref_t<decltype(g1)>,
                    util::remove_cref_t<decltype(g2)>
                >::apply(g1, g2, output_collection, strategy);
        }, geometry1, geometry2);
    }
};

    
} // namespace resolve_dynamic


/*!
\brief Combines two geometries which each other
\ingroup union
\details \details_calc2{union, spatial set theoretic union}.
\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\tparam Collection output collection, either a multi-geometry,
    or a std::vector<Geometry> / std::deque<Geometry> etc
\tparam Strategy \tparam_strategy{Union_}
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\param output_collection the output collection
\param strategy \param_strategy{union_}
\note Called union_ because union is a reserved word.

\qbk{distinguish,with strategy}
\qbk{[include reference/algorithms/union.qbk]}
*/
template
<
    typename Geometry1,
    typename Geometry2,
    typename Collection,
    typename Strategy
>
inline void union_(Geometry1 const& geometry1,
                   Geometry2 const& geometry2,
                   Collection& output_collection,
                   Strategy const& strategy)
{
    resolve_dynamic::union_
        <
            Geometry1,
            Geometry2
        >::apply(geometry1, geometry2, output_collection, strategy);
}


/*!
\brief Combines two geometries which each other
\ingroup union
\details \details_calc2{union, spatial set theoretic union}.
\tparam Geometry1 \tparam_geometry
\tparam Geometry2 \tparam_geometry
\tparam Collection output collection, either a multi-geometry,
    or a std::vector<Geometry> / std::deque<Geometry> etc
\param geometry1 \param_geometry
\param geometry2 \param_geometry
\param output_collection the output collection
\note Called union_ because union is a reserved word.

\qbk{[include reference/algorithms/union.qbk]}
*/
template
<
    typename Geometry1,
    typename Geometry2,
    typename Collection
>
inline void union_(Geometry1 const& geometry1,
                   Geometry2 const& geometry2,
                   Collection& output_collection)
{
    resolve_dynamic::union_
        <
            Geometry1,
            Geometry2
        >::apply(geometry1, geometry2, output_collection, default_strategy());
}


}} // namespace boost::geometry


#endif // BOOST_GEOMETRY_ALGORITHMS_UNION_HPP
