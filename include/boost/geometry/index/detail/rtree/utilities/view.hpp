// Boost.Geometry Index
//
// Rtree utilities view
//
// Copyright (c) 2011-2013 Adam Wulkiewicz, Lodz, Poland.
//
// This file was modified by Oracle on 2019.
// Modifications copyright (c) 2019 Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_RTREE_UTILITIES_VIEW_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_RTREE_UTILITIES_VIEW_HPP

namespace boost { namespace geometry { namespace index {

namespace detail { namespace rtree { namespace utilities {

template <typename Rtree>
class view
{
public:
    using members_holder = typename Rtree::members_holder;

    using size_type = typename Rtree::size_type;

    using translator_type = typename Rtree::translator_type;
    using value_type = typename Rtree::value_type;
    using options_type = typename Rtree::options_type;
    using box_type = typename Rtree::box_type;
    using allocators_type = typename Rtree::allocators_type;

    view(Rtree const& rt) : m_rtree(rt) {}

    template <typename Visitor>
    void apply_visitor(Visitor & vis) const
    {
        m_rtree.apply_visitor(vis);
    }

    // This will most certainly be removed in the future
    translator_type translator() const
    {
        return m_rtree.translator();
    }

    // This will probably be removed in the future
    size_type depth() const
    {
        return m_rtree.depth();
    }

private:
    view(view const&);
    view & operator=(view const&);

    Rtree const& m_rtree;
};

}}} // namespace detail::rtree::utilities

}}} // namespace boost::geometry::index

#endif // BOOST_GEOMETRY_INDEX_DETAIL_RTREE_UTILITIES_VIEW_HPP
