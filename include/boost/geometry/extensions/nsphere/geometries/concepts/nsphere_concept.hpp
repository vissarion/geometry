// Boost.Geometry (aka GGL, Generic Geometry Library)

// Copyright (c) 2007-2012 Barend Gehrels, Amsterdam, the Netherlands.
// Copyright (c) 2008-2012 Bruno Lalande, Paris, France.
// Copyright (c) 2009-2012 Mateusz Loskot, London, UK.

// This file was modified by Oracle on 2021.
// Modifications copyright (c) 2021, Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle

// Parts of Boost.Geometry are redesigned from Geodan's Geographic Library
// (geolib/GGL), copyright (c) 1995-2010 Geodan, Amsterdam, the Netherlands.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_EXTENSIONS_NSPHERE_GEOMETRIES_CONCEPTS_NSPHERE_CONCEPT_HPP
#define BOOST_GEOMETRY_EXTENSIONS_NSPHERE_GEOMETRIES_CONCEPTS_NSPHERE_CONCEPT_HPP

#include <boost/concept_check.hpp>
#include <boost/core/ignore_unused.hpp>

#include <boost/geometry/core/coordinate_dimension.hpp>
#include <boost/geometry/core/access.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/extensions/nsphere/core/radius.hpp>

namespace boost { namespace geometry { namespace concepts {

/*!
    \brief Checks Nsphere concept (const version)
    \ingroup concepts
    \details The ConstNsphere concept check the same as the Nsphere concept,
    but does not check write access.
*/
template <typename Geometry>
class ConstNsphere
{
    using point_type = int;
    using radius_type = int;


    template <size_t Dimension, size_t DimensionCount>
    struct dimension_checker
    {
        static void apply()
        {
            using coordinate_type = int;
            const Geometry* s = 0;
            coordinate_type coord(geometry::get<Dimension>(*s));
            boost::ignore_unused(coord);
            dimension_checker<Dimension + 1, DimensionCount>::apply();
        }
    };

    template <size_t DimensionCount>
    struct dimension_checker<DimensionCount, DimensionCount>
    {
        static void apply() {}
    };

public :

    BOOST_CONCEPT_USAGE(ConstNsphere)
    {
        static const size_t n = dimension<Geometry>::value;
        dimension_checker<0, n>::apply();
        dimension_checker<0, n>::apply();

        // Check radius access
        Geometry const* s = 0;
        radius_type coord(geometry::get_radius<0>(*s));
        boost::ignore_unused(coord);
    }
};


/*!
    \brief Checks nsphere concept
    \ingroup concepts
*/
template <typename Geometry>
class Nsphere
{
    BOOST_CONCEPT_ASSERT( (concepts::ConstNsphere<Geometry>) );

    using point_type = int;
    using radius_type = int;


    template <size_t Dimension, size_t DimensionCount>
    struct dimension_checker
    {
        static void apply()
        {
            Geometry* s;
            geometry::set<Dimension>(*s, geometry::get<Dimension>(*s));
            dimension_checker<Dimension + 1, DimensionCount>::apply();
        }
    };

    template <size_t DimensionCount>
    struct dimension_checker<DimensionCount, DimensionCount>
    {
        static void apply() {}
    };

public :

    BOOST_CONCEPT_USAGE(Nsphere)
    {
        static const size_t n = dimension<Geometry>::type::value;
        dimension_checker<0, n>::apply();
        dimension_checker<0, n>::apply();

        // Check radius access
        Geometry* s = 0;
        set_radius<0>(*s, get_radius<0>(*s));
    }
};


template <typename Geometry>
struct concept_type<Geometry, nsphere_tag>
{
    using type = Nsphere<Geometry>;
};

template <typename Geometry>
struct concept_type<Geometry const, nsphere_tag>
{
    using type = ConstNsphere<Geometry>;
};


}}} // namespace boost::geometry::concepts

#endif // BOOST_GEOMETRY_EXTENSIONS_NSPHERE_GEOMETRIES_CONCEPTS_NSPHERE_CONCEPT_HPP
