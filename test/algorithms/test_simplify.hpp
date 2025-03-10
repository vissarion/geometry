// Boost.Geometry (aka GGL, Generic Geometry Library)
// Unit Test

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.

// This file was modified by Oracle on 2021-2023.
// Modifications copyright (c) 2021-2023 Oracle and/or its affiliates.
// Contributed and/or modified by Vissarion Fysikopoulos, on behalf of Oracle
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_TEST_SIMPLIFY_HPP
#define BOOST_GEOMETRY_TEST_SIMPLIFY_HPP

// Test-functionality, shared between single and multi tests

#include <iomanip>
#include <sstream>
#include <geometry_test_common.hpp>
#include <boost/geometry/algorithms/correct_closure.hpp>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/geometry_collection.hpp>
#include <boost/geometry/strategies/concepts/simplify_concept.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/variant/variant.hpp>


template <typename Geometry, typename Tag = typename bg::tag<Geometry>::type>
struct boost_variant_type
{
    using type = boost::variant<Geometry, typename bg::point_type<Geometry>::type>;
};

template <typename Geometry>
struct boost_variant_type<Geometry, bg::point_tag>
{
    using type = boost::variant<Geometry>;
};

template
<
    typename GeometryForTag,
    typename Tag = typename bg::tag<GeometryForTag>::type
>
struct test_equality
{
    template <typename Geometry, typename Expected>
    static void apply(Geometry const& geometry, Expected const& expected)
    {
        // Verify both spatially equal AND number of points, because several
        // of the tests only check explicitly on collinear points being
        // simplified away
        bool const result
                = bg::equals(geometry, expected)
                && bg::num_points(geometry) == bg::num_points(expected);

        BOOST_CHECK_MESSAGE(result,
                            " result: " << bg::wkt(geometry) << " " << bg::area(geometry)
                            << " expected: " << bg::wkt(expected) << " " << bg::area(expected));

    }
};

// Linestring does NOT yet have "geometry::equals" implemented
// Until then, WKT's are compared (which is acceptable for linestrings, but not
// for polygons, because simplify might rotate them)
template <typename GeometryForTag>
struct test_equality<GeometryForTag, bg::linestring_tag>
{
    template <typename Geometry, typename Expected>
    static void apply(Geometry const& geometry, Expected const& expected)
    {
        std::ostringstream out1, out2;
        out1 << bg::wkt(geometry);
        out2 << bg::wkt(expected);
        BOOST_CHECK_EQUAL(out1.str(), out2.str());
    }
};


template <typename Tag>
struct test_inserter
{
    template <typename Geometry, typename Expected>
    static void apply(Geometry& , Expected const& , double )
    {}
};

template <>
struct test_inserter<bg::linestring_tag>
{
    template <typename Geometry, typename Expected, typename DistanceMeasure>
    static void apply(Geometry& geometry,
            Expected const& expected,
            DistanceMeasure const& distance)
    {
        {
            Geometry simplified;
            bg::detail::simplify::simplify_insert(geometry,
                std::back_inserter(simplified), distance);

            test_equality<Geometry>::apply(simplified, expected);
        }
    }
};

template <typename Geometry, typename Expected, typename DistanceMeasure>
void check_geometry(Geometry const& geometry,
                    Expected const& expected,
                    DistanceMeasure const& distance)
{
    Geometry simplified;
    bg::simplify(geometry, simplified, distance);
    test_equality<Expected>::apply(simplified, expected);
}

template <typename Geometry, typename Expected, typename Strategy, typename DistanceMeasure>
void check_geometry(Geometry const& geometry,
                    Expected const& expected,
                    DistanceMeasure const& distance,
                    Strategy const& strategy)
{
    Geometry simplified;
    bg::simplify(geometry, simplified, distance, strategy);
    test_equality<Expected>::apply(simplified, expected);
}

template <typename Geometry, typename DistanceMeasure>
void check_geometry_with_area(Geometry const& geometry,
                    double expected_area,
                    DistanceMeasure const& distance)
{
    Geometry simplified;
    bg::simplify(geometry, simplified, distance);
    BOOST_CHECK_CLOSE(bg::area(simplified), expected_area, 0.01);
}


template <typename Geometry, typename DistanceMeasure>
void test_geometry(std::string const& wkt,
        std::string const& expected_wkt,
        DistanceMeasure distance)
{
    typedef typename bg::point_type<Geometry>::type point_type;

    Geometry geometry, expected;

    bg::read_wkt(wkt, geometry);
    bg::read_wkt(expected_wkt, expected);

    using variant_t = typename boost_variant_type<Geometry>::type;
    variant_t v(geometry);

    // Define default strategy for testing
    typedef bg::strategy::simplify::douglas_peucker
        <
            typename bg::point_type<Geometry>::type,
            bg::strategy::distance::projected_point<double>
        > dp;

    BOOST_CONCEPT_ASSERT((bg::concepts::SimplifyStrategy<dp, point_type>));

    check_geometry(geometry, expected, distance);
    check_geometry(v, expected, distance);

    check_geometry(geometry, expected, distance, dp());
    check_geometry(v, expected, distance, dp());

    // For now check GC here because it's not supported by equals()
    {
        using gc_t = bg::model::geometry_collection<variant_t>;
        gc_t gc{v};
        gc_t gc_simplified;
        bg::simplify(gc, gc_simplified, distance);
        bg::detail::visit_breadth_first([&](auto const& g)
        {
            test_equality<Geometry>::apply(g, expected);
            return false;
        }, gc_simplified);
    }

    // Check inserter (if applicable)
    test_inserter
        <
            typename bg::tag<Geometry>::type
        >::apply(geometry, expected, distance);
}

template <typename Geometry, typename Strategy, typename DistanceMeasure>
void test_geometry(std::string const& wkt,
        std::string const& expected_wkt,
        DistanceMeasure const& distance,
        Strategy const& strategy)
{
    Geometry geometry, expected;

    bg::read_wkt(wkt, geometry);
    bg::read_wkt(expected_wkt, expected);
    bg::correct_closure(geometry);
    bg::correct_closure(expected);

    typename boost_variant_type<Geometry>::type v(geometry);

    BOOST_CONCEPT_ASSERT( (bg::concepts::SimplifyStrategy<Strategy,
                           typename bg::point_type<Geometry>::type>) );

    check_geometry(geometry, expected, distance, strategy);
    check_geometry(v, expected, distance, strategy);
}

template <typename Geometry, typename DistanceMeasure>
void test_geometry(std::string const& wkt,
        double expected_area,
        DistanceMeasure const& distance)
{
    Geometry geometry;
    bg::read_wkt(wkt, geometry);
    bg::correct_closure(geometry);

    check_geometry_with_area(geometry, expected_area, distance);
}

#endif
