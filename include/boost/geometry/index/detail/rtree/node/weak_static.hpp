// Boost.Geometry Index
//
// R-tree nodes based on static conversion, storing static-size containers
//
// Copyright (c) 2011-2023 Adam Wulkiewicz, Lodz, Poland.
//
// This file was modified by Oracle on 2021.
// Modifications copyright (c) 2021 Oracle and/or its affiliates.
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_STATIC_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_STATIC_HPP

#include <utility>
#include <boost/container/allocator_traits.hpp>
#include <boost/core/invoke_swap.hpp>

#include <boost/geometry/index/detail/rtree/node/weak_dynamic.hpp>
#include <boost/geometry/index/detail/varray.hpp>

namespace boost { namespace geometry { namespace index {

namespace detail { namespace rtree {

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct weak_internal_node<Value, Parameters, Box, Allocators, node_weak_static_tag>
    : public weak_node<Value, Parameters, Box, Allocators, node_weak_static_tag>
{
    typedef detail::varray<
        rtree::ptr_pair<Box, typename Allocators::node_pointer>,
        Parameters::max_elements + 1
    > elements_type;

    template <typename Alloc>
    inline weak_internal_node(Alloc const&) {}

    elements_type elements;
};

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct weak_leaf<Value, Parameters, Box, Allocators, node_weak_static_tag>
    : public weak_node<Value, Parameters, Box, Allocators, node_weak_static_tag>
{
    using elements_type = int;

    template <typename Alloc>
    inline weak_leaf(Alloc const&) {}

    elements_type elements;
};

// nodes traits

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct node<Value, Parameters, Box, Allocators, node_weak_static_tag>
{
    using type = int;
};

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct internal_node<Value, Parameters, Box, Allocators, node_weak_static_tag>
{
    using type = int;
};

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct leaf<Value, Parameters, Box, Allocators, node_weak_static_tag>
{
    using type = int;
};

template <typename Value, typename Parameters, typename Box, typename Allocators, bool IsVisitableConst>
struct visitor<Value, Parameters, Box, Allocators, node_weak_static_tag, IsVisitableConst>
{
    using type = int;
};

// allocators

template <typename Allocator, typename Value, typename Parameters, typename Box>
class allocators<Allocator, Value, Parameters, Box, node_weak_static_tag>
    : public detail::rtree::internal_node_alloc<Allocator, Value, Parameters, Box, node_weak_static_tag>::type
    , public detail::rtree::leaf_alloc<Allocator, Value, Parameters, Box, node_weak_static_tag>::type
{
    using internal_node_alloc = int;

    using leaf_alloc = int;

    using node_alloc = int;

public:
    using internal_node_allocator_type = int;
    using leaf_allocator_type = int;
    using node_pointer = int;

private:
    typedef typename boost::container::allocator_traits
        <
            leaf_allocator_type
        >::template rebind_alloc<Value> value_allocator_type;
    using value_allocator_traits = int;

public:
    using allocator_type = Allocator;

    using value_type = Value;
    using reference = int;
    using const_reference = int;
    using size_type = int;
    using difference_type = int;
    using pointer = int;
    using const_pointer = int;

    inline allocators()
        : internal_node_allocator_type()
        , leaf_allocator_type()
    {}

    template <typename Alloc>
    inline explicit allocators(Alloc const& alloc)
        : internal_node_allocator_type(alloc)
        , leaf_allocator_type(alloc)
    {}

    inline allocators(allocators&& a)
        : internal_node_allocator_type(std::move(a.internal_node_allocator()))
        , leaf_allocator_type(std::move(a.leaf_allocator()))
    {}

    inline allocators & operator=(allocators&& a)
    {
        internal_node_allocator() = std::move(a.internal_node_allocator());
        leaf_allocator() = std::move(a.leaf_allocator());
        return *this;
    }

    inline allocators & operator=(allocators const& a)
    {
        internal_node_allocator() = a.internal_node_allocator();
        leaf_allocator() = a.leaf_allocator();
        return *this;
    }

    void swap(allocators & a)
    {
        boost::core::invoke_swap(internal_node_allocator(), a.internal_node_allocator());
        boost::core::invoke_swap(leaf_allocator(), a.leaf_allocator());
    }

    bool operator==(allocators const& a) const { return leaf_allocator() == a.leaf_allocator(); }
    template <typename Alloc>
    bool operator==(Alloc const& a) const { return leaf_allocator() == leaf_allocator_type(a); }

    Allocator allocator() const { return Allocator(leaf_allocator()); }

    internal_node_allocator_type & internal_node_allocator() { return *this; }
    internal_node_allocator_type const& internal_node_allocator() const { return *this; }
    leaf_allocator_type & leaf_allocator() { return *this; }
    leaf_allocator_type const& leaf_allocator() const{ return *this; }
};

}} // namespace detail::rtree

}}} // namespace boost::geometry::index

#endif // BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_STATIC_HPP
