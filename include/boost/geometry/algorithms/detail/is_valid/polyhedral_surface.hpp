// Boost.Geometry

// Copyright (c) 2025 Oracle and/or its affiliates.
// Contributed and/or modified by Vissarion Fysikopoulos, on behalf of Oracle

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_ALGORITHMS_DETAIL_IS_VALID_POLYHEDRAL_SURFACE_HPP
#define BOOST_GEOMETRY_ALGORITHMS_DETAIL_IS_VALID_POLYHEDRAL_SURFACE_HPP

#include <boost/qvm/vec.hpp>
#include <boost/qvm/vec_operations.hpp>

#include <boost/geometry/algorithms/detail/is_valid/polygon.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

namespace boost { namespace geometry
{

#ifndef DOXYGEN_NO_DETAIL
namespace detail { namespace is_valid
{

template <typename PolyhedralSurface>
struct valid_intersection
{
    valid_intersection() = default;

private:

    enum class intersection_type
    {
        none,
        valid,
        invalid
    };

    using CT = typename coordinate_type<PolyhedralSurface>::type;
    using vec3 = boost::qvm::vec<CT, 3>;
    using polygon_type = typename boost::range_value<PolyhedralSurface>::type;
    using ring_type = typename boost::geometry::ring_type<polygon_type>::type;
    using point_type = typename boost::range_value<ring_type>::type;
    using segment_type = typename boost::geometry::model::segment<point_type>;
    using set_vec_from_point = typename strategy::transform::detail::matrix_transformer::
        set_vec_from_point<point_type, 0, 3>;

    using point_2d_type = boost::geometry::model::point
        <CT, 2, boost::geometry::cs::cartesian>;
    using linestring_2d = typename boost::geometry::model::linestring<point_2d_type>;
    using polygon_2d = typename std::conditional
        <
            boost::geometry::point_order<polygon_type>::value == boost::geometry::counterclockwise,
            typename boost::geometry::model::polygon<point_2d_type, true>,
            typename boost::geometry::model::polygon<point_2d_type, false>
        >::type;


    struct polygon_projection
    {
        polygon_2d projected_polygon;
        vec3 base;
        vec3 v1;
        vec3 v2;
        vec3 normal;
    };

    std::unordered_map<std::string, polygon_projection> m_polygon_projections;

    template <typename VisitPolicy>
    static bool has_correct_intersection_orientation(polygon_type const& polygon,
                                                     segment_type const& segment,
                                                     VisitPolicy& visitor)
    {
        auto ring = boost::geometry::exterior_ring(polygon);
        if (boost::geometry::closure<ring_type>::value == boost::geometry::closed)
        {
            ring.pop_back(); // Remove the closing point to make it open
        }

        for (auto it = boost::begin(ring); it != boost::end(ring); ++it)
        {
            if (boost::geometry::equals(*it, segment.first))
            {
                auto prev = (it == boost::begin(ring)) ? boost::end(ring) - 1 : boost::prior(it);
                if (boost::geometry::equals(segment.second, *prev))
                {
                    // valid intersection and inconsistent orientation
                    return true;
                }
                auto next = (it == boost::end(ring)) ? boost::begin(ring) : boost::next(it);
                if (boost::geometry::equals(segment.second, *next))
                {
                    // valid intersection but inconsistent orientation
                    visitor.template apply<failure_inconsistent_orientation>();
                    return false;
                }
            }
        }
        visitor.template apply<failure_invalid_intersection>();
        return false;
    }

    // Check if a segment intersects a polygon in 3D space
    template <typename VisitPolicy>
    intersection_type apply(segment_type const& segment,
                            polygon_type const& polygon,
                            VisitPolicy& visitor)
    {
        // Print the polygon and segment in WKT format
        std::cout << "Polygon: " << boost::geometry::wkt(polygon) << std::endl;
        std::cout << "Segment: " << boost::geometry::wkt(segment) << std::endl;
        vec3 p1;
        vec3 p2;
        set_vec_from_point::apply(segment.first, p1);
        set_vec_from_point::apply(segment.second, p2);

        vec3 base;
        vec3 v1;
        vec3 v2;
        vec3 normal;
        polygon_2d projected_polygon;

        // Check if the polygon has been projected before
        std::ostringstream oss;
        oss << boost::geometry::wkt(polygon);
        if (m_polygon_projections.find(oss.str()) != m_polygon_projections.end())
        {
            auto& projection = m_polygon_projections[oss.str()];
            base = projection.base;
            v1 = projection.v1;
            v2 = projection.v2;
            normal = projection.normal;
            projected_polygon = projection.projected_polygon;
        }
        else
        {
            auto const& ring = boost::geometry::exterior_ring(polygon);
            constexpr bool is_ring_open = boost::geometry::closure<ring_type>::value
                == boost::geometry::open;

            if ((boost::size(ring) < 3 && is_ring_open) ||
                (boost::size(ring) < 4 && !is_ring_open))
            {
                visitor.template apply<failure_few_points_on_face>();
                return intersection_type::invalid;
            }

            // Get three non-collinear points from the polygon to define the plane
            if (!find_non_collinear_points(polygon, base, v1, v2))
            {
                visitor.template apply<failure_collinear_points_on_face>();
                return intersection_type::invalid;
            }

            v1 -= base;
            v2 -= base;
            normal = cross(v1, v2);

            // Check for non-coplanar points
            for (auto pit = points_begin(polygon); pit != points_end(polygon); ++pit)
            {
                vec3 vi;
                set_vec_from_point::apply(*pit, vi);
                auto vol = qvm::dot(normal, vi)- dot(normal, base);
                if (std::abs(vol) > 1e-10)
                {
                    visitor.template apply<failure_non_coplanar_points_on_face>();
                    return intersection_type::invalid;
                }
            }

            for (auto& point : ring)
            {
                vec3 p;
                set_vec_from_point::apply(point, p);
                auto projected_p = project_point_to_plane_2d(v1, v2, p);
                projected_polygon.outer().push_back(projected_p);
            }
            std::cout << "Projected Polygon: " << boost::geometry::wkt(projected_polygon) << std::endl;
            // Compute the orientation of the polygon using the determinant of the first 3 points
            if (boost::size(ring) >= 3)
            {
                point_2d_type p0 = projected_polygon.outer()[0];
                point_2d_type p1 = projected_polygon.outer()[1];
                point_2d_type p2 = projected_polygon.outer()[2];

                auto side = boost::geometry::strategy::side::side_robust
                <
                    double,
                    boost::geometry::strategy::side::fp_equals_policy
                >::apply(p0, p1, p2);

                auto const& orientation = boost::geometry::point_order<polygon_2d>::value;
                if ((side > 0 && boost::geometry::point_order<polygon_2d>::value == boost::geometry::clockwise) ||
                    (side < 0 && boost::geometry::point_order<polygon_2d>::value == boost::geometry::counterclockwise))
                {
                    std::cout << "Orientation is wrong!!!" << std::endl;
                    boost::geometry::reverse(projected_polygon);
                }
                m_polygon_projections[oss.str()] = polygon_projection{projected_polygon, base, v1, v2, normal};
            }

            // Check if the projected polygon is valid
            if (!resolve_dynamic::is_valid<polygon_2d>::apply(projected_polygon, visitor,
                default_strategy()))
            {
                return intersection_type::invalid;
            }
        }

        // Check if the segment intersects the plane of the polygon
        CT sign1 = dot(normal, p1) - dot(normal, base);
        CT sign2 = dot(normal, p2) - dot(normal, base);
        if ((sign1 > 0 && sign2 > 0) || (sign1 < 0 && sign2 < 0))
        {
            return intersection_type::none; // No intersection, this is valid
        }

        // Check if the segment is on the same plane as the polygon
        if (sign1 == 0 && sign2 == 0)
        {
            auto projected_p1 = project_point_to_plane_2d(v1, v2, p1);
            auto projected_p2 = project_point_to_plane_2d(v1, v2, p2);

            linestring_2d projected_segment = {projected_p1, projected_p2};
            std::cout << "Projected Segment: " << boost::geometry::wkt(projected_segment) << std::endl;

            // We have to correct the orientation of the projected polygon to compute the correct intersection
            // Later on we will check consistency of the orientation using the 3d polygon
            //boost::geometry::correct(projected_polygon);

            // Check if the projected segment intersects the polygon
            std::tuple
            <
                boost::geometry::model::multi_point<point_2d_type>,
                boost::geometry::model::multi_linestring<linestring_2d>> intersections;
            boost::geometry::intersection(projected_segment, projected_polygon, intersections);

            auto& points = std::get<0>(intersections);
            auto& linestrings = std::get<1>(intersections);

            if (boost::geometry::is_empty(points) && boost::geometry::is_empty(linestrings))
            {
                return intersection_type::none; // No intersection, this is valid
            }
            if (!points.empty())
            {
                visitor.template apply<failure_invalid_intersection>();
                std::cout << "Intersection points: " << boost::geometry::wkt(points) << std::endl;
                return intersection_type::invalid;
            }
            if (linestrings.size() == 1)
            {
                std::cout << "Intersection linestring: " << boost::geometry::wkt(linestrings.front()) << std::endl;
                if (boost::geometry::equals(linestrings.front(), projected_segment))
                {
                    if (!has_correct_intersection_orientation(polygon, segment, visitor))
                    {
                        return intersection_type::invalid;
                    }
                    return intersection_type::valid; // Valid intersection
                }
            }
        }

        // Compute the intersection point
        vec3 seg_dir = p2 - p1;
        CT denom = dot(normal, seg_dir);
        CT t = dot(normal, base - p1) / denom;
        vec3 intersection = p1 + t * seg_dir;

        // Project the intersection point onto the plane of the polygon
        auto projected_intersection = project_point_to_plane_2d(v1, v2, intersection);

        // Check if the projected intersection point is inside the projected polygon
        if (boost::geometry::within(projected_intersection, projected_polygon))
        {
            visitor.template apply<failure_invalid_intersection>();
            return intersection_type::invalid;
        }
        return intersection_type::none;
    }

       // Check if two vectors are linearly dependent (collinear)
    static bool are_linearly_dependent(vec3 const& a, vec3 const& b, CT tol = 1e-10)
    {
        return qvm::mag_sqr(qvm::cross(a, b)) < tol * tol;
    }

    // Try to find 3 non-collinear points
    static bool find_non_collinear_points(polygon_type const& polygon, vec3& base, vec3& v1, vec3& v2)
    {
        for (auto pit1 = points_begin(polygon); pit1 != points_end(polygon); ++pit1)
        {
            for (auto pit2 = pit1; pit2 != points_end(polygon); ++pit2)
            {
                for (auto pit3 = pit2; pit3 != points_end(polygon); ++pit3)
                {
                    set_vec_from_point::apply(*pit1, base);
                    set_vec_from_point::apply(*pit2, v1);
                    set_vec_from_point::apply(*pit3, v2);
                    if (!are_linearly_dependent(v1 - base, v2 - base)) {
                        return true;
                    }
                }
            }
        }
        return false; // All points are collinear
    }

    static point_2d_type project_point_to_plane_2d(vec3 const& v1,
                                                   vec3 const& v2,
                                                   vec3 const& p)
    {
        return point_2d_type{qvm::dot(v1, p), qvm::dot(v2, p)};
    }

 public:
    template <typename VisitPolicy>
    static bool are_points_coplanar(polygon_type const& polygon,
                                    VisitPolicy& visitor,
                                    CT tol = 1e-10)
    {
        auto const& ring = boost::geometry::exterior_ring(polygon);
        constexpr bool is_ring_open = boost::geometry::closure<ring_type>::value
            == boost::geometry::open;

        if ((boost::size(ring) < 3 && is_ring_open) ||
            (boost::size(ring) < 4 && !is_ring_open))
        {
            return visitor.template apply<failure_few_points_on_face>();
        }

        vec3 base;
        vec3 v1;
        vec3 v2;

        if (!find_non_collinear_points(polygon, base, v1, v2))
        {
            return visitor.template apply<failure_collinear_points_on_face>();
        }

        v1 -= base;
        v2 -= base;
        vec3 normal = qvm::cross(v1, v2);

        for (auto pit = points_begin(polygon); pit != points_end(polygon); ++pit)
        {
            vec3 vi;
            set_vec_from_point::apply(*pit, vi);
            vi -= base;
            auto vol = qvm::dot(normal, vi);
            if (std::abs(vol) > tol)
            {
                return visitor.template apply<failure_non_coplanar_points_on_face>();
            }
        }
        return true;
    }

    template <typename VisitPolicy>
    intersection_type apply(polygon_type const& polygon1,
                            polygon_type const& polygon2,
                            VisitPolicy& visitor)
    {
        //using segment_type = typename boost::geometry::model::segment<point_type>;

        auto const& ring1 = boost::geometry::exterior_ring(polygon1);
        auto const& ring2 = boost::geometry::exterior_ring(polygon2);

        std::vector<segment_type> segments1, segments2;

        // Create segments for polygon1
        for (auto it = boost::begin(ring1); it != boost::end(ring1) - 1; ++it)
        {
            segments1.emplace_back(*it, *(it + 1));
        }

        // Create segments for polygon2
        for (auto it = boost::begin(ring2); it != boost::end(ring2) - 1; ++it)
        {
            segments2.emplace_back(*it, *(it + 1));
        }

        // Check if any segment of polygon1 intersects polygon2
        bool disconnected = true;
        for (auto const& seg1 : segments1)
        {
            auto result = apply(seg1, polygon2, visitor);
            if (result == intersection_type::invalid)
            {
                return result;
            }
            else if (result == intersection_type::valid)
            {
                disconnected = false;
            }
        }

        // Check if any segment of polygon2 intersects polygon1
        bool is_polygon1_disconnected = true;
        for (auto const& seg2 : segments2)
        {
            auto result = apply(seg2, polygon1, visitor);
            if (result == intersection_type::invalid)
            {
                return result;
            }
            if (result == intersection_type::valid)
            {
                disconnected = false;
            }
        }

        if (disconnected)
        {
            return intersection_type::none;
        }
        return intersection_type::valid;
    }
};


template <typename PolyhedralSurface>
struct is_valid_polyhedral_surface
{
protected:

    using CT = typename coordinate_type<PolyhedralSurface>::type;
    using vec3 = boost::qvm::vec<CT, 3>;
    using polygon_type = typename boost::range_value<PolyhedralSurface>::type;
    using ring_type = typename boost::geometry::ring_type<polygon_type>::type;
    using point_type = typename boost::range_value<ring_type>::type;
    using segment_type = typename boost::geometry::model::segment<point_type>;
    using set_vec_from_point = typename strategy::transform::detail::matrix_transformer::
        set_vec_from_point<point_type, 0, 3>;

    using point_2d_type = boost::geometry::model::point
        <CT, 2, boost::geometry::cs::cartesian>;
    using linestring_2d = typename boost::geometry::model::linestring<point_2d_type>;
    using polygon_2d = typename boost::geometry::model::polygon<point_2d_type>;


public:

    template <typename VisitPolicy, typename Strategy>
    static inline bool apply(PolyhedralSurface const& surface,
                             VisitPolicy& visitor,
                             Strategy const& strategy)
    {
        // Print the polyhedral surface in WKT format
        std::cout << boost::geometry::wkt(surface) << ")" << std::endl;
/*
        // Check if all faces (polygons) of the polyhedral surface are planar
        for (auto const& polygon : surface)
        {
            if (!valid_intersection<PolyhedralSurface>::are_points_coplanar(polygon, visitor))
            {
                return false;
            }
        }
*/
        // Check pairs of polygons for consistent orientation
        // For ech pair we check if they have a common edge
        // and if the normals of the polygons are in the same direction
        // to check intersection we loop over the edges of the polygons

        bool disconnected = true;
        for (auto it1 = boost::begin(surface); it1 != boost::end(surface); ++it1)
        {
            auto const& polygon1 = *it1;

            for (auto it2 = boost::begin(surface); it2 != boost::end(surface); ++it2)
            {
                auto const& polygon2 = *it2;

                if (std::distance(boost::begin(surface), it1) != std::distance(boost::begin(surface), it2))
                {
                    valid_intersection<PolyhedralSurface> valid_intersection;
                    auto intersection = valid_intersection.apply(polygon1, polygon2, visitor);
                    using intersection_type = decltype(intersection);
                    if (intersection == intersection_type::invalid)
                    {
                        return false; //  Invalid intersection found (including inconsistent orientation)
                    }
                    else if (intersection == intersection_type::valid)
                    {
                        disconnected = false;
                    }
                }
            }
        }

        if (disconnected)
        {
            visitor.template apply<failure_disconnected_surface>();
            return false;
        }
        return true;
    }
};

}} // namespace detail::is_valid
#endif // DOXYGEN_NO_DETAIL

#ifndef DOXYGEN_NO_DISPATCH
namespace dispatch
{


// Reference (for validity of Polyhedral Surfaces): OGC
// OpenGIS® Implementation Standard for Geographic
// information - Simple feature access - Part 1: Common
// architecture: 6.1.12
template <typename PolyhedralSurface, bool AllowEmptyMultiGeometries>
struct is_valid
    <
        PolyhedralSurface, polyhedral_surface_tag, AllowEmptyMultiGeometries
    > : detail::is_valid::is_valid_polyhedral_surface<PolyhedralSurface>
{};


} // namespace dispatch
#endif // DOXYGEN_NO_DISPATCH

}} // namespace boost::geometry

#endif // BOOST_GEOMETRY_ALGORITHMS_DETAIL_IS_VALID_POLYHEDRAL_SURFACE_HPP
