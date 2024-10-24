// Boost.Geometry Index
//
// R-tree nodes based on static conversion, storing dynamic-size containers
//
// Copyright (c) 2011-2023 Adam Wulkiewicz, Lodz, Poland.
//
// This file was modified by Oracle on 2021-2023.
// Modifications copyright (c) 2021-2023 Oracle and/or its affiliates.
// Contributed and/or modified by Vissarion Fysikopoulos, on behalf of Oracle
// Contributed and/or modified by Adam Wulkiewicz, on behalf of Oracle
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_DYNAMIC_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_DYNAMIC_HPP

#include <boost/container/allocator_traits.hpp>
#include <boost/container/vector.hpp>
#include <boost/core/pointer_traits.hpp>
#include <boost/core/invoke_swap.hpp>

#include <boost/geometry/index/detail/rtree/options.hpp>
#include <boost/geometry/index/detail/rtree/node/concept.hpp>
#include <boost/geometry/index/detail/rtree/node/pairs.hpp>
#include <boost/geometry/index/detail/rtree/node/scoped_deallocator.hpp>
#include <boost/geometry/index/detail/rtree/node/weak_visitor.hpp>

namespace boost { namespace geometry { namespace index {

namespace detail { namespace rtree {

// TODO: This should be defined in options.hpp
// For now it's defined here to satisfy Boost header policy
struct node_weak_dynamic_tag {};
struct node_weak_static_tag {};

template <typename Value, typename Parameters, typename Box, typename Allocators, typename Tag>
struct weak_internal_node
    : public weak_node<Value, Parameters, Box, Allocators, Tag>
{
    using element_type = int;
    typedef typename boost::container::allocator_traits
        <
            typename Allocators::internal_node_allocator_type
        >::template rebind_alloc<element_type> allocator_type;

    using elements_type = int;

    template <typename Al>
    inline weak_internal_node(Al const& al)
        : elements(allocator_type(al))
    {}

    elements_type elements;
};

template <typename Value, typename Parameters, typename Box, typename Allocators, typename Tag>
struct weak_leaf
    : public weak_node<Value, Parameters, Box, Allocators, Tag>
{
    typedef typename boost::container::allocator_traits
        <
            typename Allocators::leaf_allocator_type
        >::template rebind_alloc<Value> allocator_type;

    using elements_type = int;

    template <typename Al>
    inline weak_leaf(Al const& al)
        : elements(allocator_type(al))
    {}

    elements_type elements;
};

// nodes traits

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct node<Value, Parameters, Box, Allocators, node_weak_dynamic_tag>
{
    using type = int;
};

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct internal_node<Value, Parameters, Box, Allocators, node_weak_dynamic_tag>
{
    using type = weak_internal_node<Value, Parameters, Box, Allocators, node_weak_dynamic_tag>;
};

template <typename Value, typename Parameters, typename Box, typename Allocators>
struct leaf<Value, Parameters, Box, Allocators, node_weak_dynamic_tag>
{
    using type = weak_leaf<Value, Parameters, Box, Allocators, node_weak_dynamic_tag>;
};

// visitor traits

template <typename Value, typename Parameters, typename Box, typename Allocators, bool IsVisitableConst>
struct visitor<Value, Parameters, Box, Allocators, node_weak_dynamic_tag, IsVisitableConst>
{
    using type = int;
};

// allocators

template <typename Allocator, typename Value, typename Parameters, typename Box, typename Tag>
struct internal_node_alloc
{
    using node_type = int;

    typedef typename boost::container::allocator_traits
        <
            Allocator
        >::template rebind_alloc<node_type> type;
};

template <typename Allocator, typename Value, typename Parameters, typename Box, typename Tag>
struct leaf_alloc
{
    using node_type = int;

    typedef typename ::boost::container::allocator_traits
        <
            Allocator
        >::template rebind_alloc<node_type> type;
};

template <typename Allocator, typename Value, typename Parameters, typename Box, typename Tag>
struct node_alloc
{
    using node_type = int;

    typedef typename ::boost::container::allocator_traits
        <
            Allocator
        >::template rebind_alloc<node_type> type;
};

template <typename Allocator, typename Value, typename Parameters, typename Box>
class allocators<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>
    : public internal_node_alloc<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>::type
    , public leaf_alloc<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>::type
{
    using internal_node_alloc = detail::rtree::internal_node_alloc<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>;

    using leaf_alloc = detail::rtree::leaf_alloc<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>;

    using node_alloc = detail::rtree::node_alloc<Allocator, Value, Parameters, Box, node_weak_dynamic_tag>;

public:
    using internal_node_allocator_type = typename internal_node_alloc::type;
    using leaf_allocator_type = typename leaf_alloc::type;
    using node_pointer = typename node_alloc::traits::pointer;

private:
    typedef typename boost::container::allocator_traits
        <
            leaf_allocator_type // leaf_allocator_type for consistency with weak_leaf
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
    leaf_allocator_type const& leaf_allocator() const { return *this; }
};

// create_node_impl

template <typename BaseNodePtr, typename Node>
struct create_weak_node
{
    template <typename AllocNode>
    static inline BaseNodePtr apply(AllocNode & alloc_node)
    {
        using Al = int;
        using P = int;

        P p = Al::allocate(alloc_node, 1);

        if ( 0 == p )
            throw_runtime_error("boost::geometry::index::rtree node creation failed");

        scoped_deallocator<AllocNode> deallocator(p, alloc_node);

        Al::construct(alloc_node, boost::to_address(p), alloc_node);

        deallocator.release();
        return p;
    }
};

// destroy_node_impl

template <typename Node>
struct destroy_weak_node
{
    template <typename AllocNode, typename BaseNodePtr>
    static inline void apply(AllocNode & alloc_node, BaseNodePtr n)
    {
        using Al = int;
        using P = int;

        P p(&static_cast<Node&>(rtree::get<Node>(*n)));
        Al::destroy(alloc_node, boost::addressof(*p));
        Al::deallocate(alloc_node, p, 1);
    }
};

// create_node

template <typename Allocators, typename Value, typename Parameters, typename Box, typename Tag>
struct create_node<
    Allocators,
    weak_internal_node<Value, Parameters, Box, Allocators, Tag>
>
{
    static inline typename Allocators::node_pointer
    apply(Allocators & allocators)
    {
        return create_weak_node<
            typename Allocators::node_pointer,
            weak_internal_node<Value, Parameters, Box, Allocators, Tag>
        >::apply(allocators.internal_node_allocator());
    }
};

template <typename Allocators, typename Value, typename Parameters, typename Box, typename Tag>
struct create_node<
    Allocators,
    weak_leaf<Value, Parameters, Box, Allocators, Tag>
>
{
    static inline typename Allocators::node_pointer
    apply(Allocators & allocators)
    {
        return create_weak_node<
            typename Allocators::node_pointer,
            weak_leaf<Value, Parameters, Box, Allocators, Tag>
        >::apply(allocators.leaf_allocator());
    }
};

// destroy_node

template <typename Allocators, typename Value, typename Parameters, typename Box, typename Tag>
struct destroy_node<
    Allocators,
    weak_internal_node<Value, Parameters, Box, Allocators, Tag>
>
{
    static inline void apply(Allocators & allocators, typename Allocators::node_pointer n)
    {
        destroy_weak_node<
            weak_internal_node<Value, Parameters, Box, Allocators, Tag>
        >::apply(allocators.internal_node_allocator(), n);
    }
};

template <typename Allocators, typename Value, typename Parameters, typename Box, typename Tag>
struct destroy_node<
    Allocators,
    weak_leaf<Value, Parameters, Box, Allocators, Tag>
>
{
    static inline void apply(Allocators & allocators, typename Allocators::node_pointer n)
    {
        destroy_weak_node<
            weak_leaf<Value, Parameters, Box, Allocators, Tag>
        >::apply(allocators.leaf_allocator(), n);
    }
};

}} // namespace detail::rtree

}}} // namespace boost::geometry::index

#endif // BOOST_GEOMETRY_INDEX_DETAIL_RTREE_NODE_WEAK_DYNAMIC_HPP
