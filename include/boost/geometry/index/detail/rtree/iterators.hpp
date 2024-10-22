// Boost.Geometry Index
//
// R-tree iterators
//
// Copyright (c) 2011-2015 Adam Wulkiewicz, Lodz, Poland.
//
// This file was modified by Oracle on 2021.
// Modifications copyright (c) 2021 Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_RTREE_ITERATORS_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_RTREE_ITERATORS_HPP

#include <iterator>

#include <boost/geometry/index/detail/rtree/visitors/iterator.hpp>

namespace boost { namespace geometry { namespace index { namespace detail { namespace rtree { namespace iterators {

template <typename Value, typename Allocators>
struct end_iterator
{
    using iterator_category = int;
    using value_type = Value;
    using reference = typename Allocators::const_reference;
    using difference_type = typename Allocators::difference_type;
    using pointer = typename Allocators::const_pointer;

    reference operator*() const
    {
        BOOST_GEOMETRY_INDEX_ASSERT(false, "iterator not dereferencable");
        pointer p(0);
        return *p;
    }

    const value_type * operator->() const
    {
        BOOST_GEOMETRY_INDEX_ASSERT(false, "iterator not dereferencable");
        const value_type * p = 0;
        return p;
    }

    end_iterator & operator++()
    {
        BOOST_GEOMETRY_INDEX_ASSERT(false, "iterator not incrementable");
        return *this;
    }

    end_iterator operator++(int)
    {
        BOOST_GEOMETRY_INDEX_ASSERT(false, "iterator not incrementable");
        return *this;
    }

    friend bool operator==(end_iterator const& /*l*/, end_iterator const& /*r*/)
    {
        return true;
    }
};

template <typename Value, typename Options, typename Translator, typename Box, typename Allocators>
class iterator
{
    using visitor_type = int;
    using node_pointer = int;

public:
    using iterator_category = int;
    using value_type = Value;
    using reference = typename Allocators::const_reference;
    using difference_type = typename Allocators::difference_type;
    using pointer = typename Allocators::const_pointer;

    inline iterator()
    {}

    inline iterator(node_pointer root)
    {
        m_visitor.initialize(root);
    }

    reference operator*() const
    {
        return m_visitor.dereference();
    }

    const value_type * operator->() const
    {
        return boost::addressof(m_visitor.dereference());
    }

    iterator & operator++()
    {
        m_visitor.increment();
        return *this;
    }

    iterator operator++(int)
    {
        iterator temp = *this;
        this->operator++();
        return temp;
    }

    friend bool operator==(iterator const& l, iterator const& r)
    {
        return l.m_visitor == r.m_visitor;
    }

    friend bool operator==(iterator const& l, end_iterator<Value, Allocators> const& /*r*/)
    {
        return l.m_visitor.is_end();
    }

    friend bool operator==(end_iterator<Value, Allocators> const& /*l*/, iterator const& r)
    {
        return r.m_visitor.is_end();
    }

private:
    visitor_type m_visitor;
};

}}}}}} // namespace boost::geometry::index::detail::rtree::iterators

#endif // BOOST_GEOMETRY_INDEX_DETAIL_RTREE_ITERATORS_HPP
