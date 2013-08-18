// Boost.Geometry Index
//
// R-tree storage manager
//
// Copyright (c) 2011-2013 Adam Wulkiewicz, Lodz, Poland.
//
// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_RTREE_STORAGE_MANAGER_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_RTREE_STORAGE_MANAGER_HPP

namespace boost { namespace geometry { namespace index { namespace detail { namespace rtree {

template <typename Val, typename Ptr, typename Id>
class storage_ptr
{
public:
    typedef Val value_type;

    storage_ptr() {}

    storage_ptr(Ptr p, Id i)
        : m_ptr(p), m_id(i) {}

    template<typename P>
    storage_ptr(P p, Id i)
        : m_ptr(p), m_id(i) {}

    storage_ptr(storage_ptr const& p)
        : m_ptr(p.m_ptr), m_id(p.m_id) {}

    template <typename V, typename P>
    storage_ptr(storage_ptr<V, P, Id> const& other)
        : m_ptr(static_cast<Ptr>(other.m_ptr)), m_id(other.m_id) {}

    //Val & operator*() { BOOST_ASSERT_MSG(m_ptr, "null ptr"); return *m_ptr; }
    //const Val & operator*() const { BOOST_ASSERT_MSG(m_ptr, "null ptr"); return *m_ptr; }
    //Val * operator->() { BOOST_ASSERT_MSG(m_ptr, "null ptr"); return boost::addressof(*m_ptr); }
    //const Val * operator->() const { BOOST_ASSERT_MSG(m_ptr, "null ptr"); return boost::addressof(*m_ptr); }

    Ptr get() const { return m_ptr; }
    Id id() const { return m_id; }

    friend bool operator< (storage_ptr const& l, storage_ptr const& r) { return l.id() < r.id(); }
    friend bool operator== (storage_ptr const& l, storage_ptr const& r) { return l.id() == r.id(); }

private:
    Ptr m_ptr;
    Id m_id;
};

// TODO: optimization
// if node_id is unsigned integer and has the same number of bytes as node_pointer
// and node_pointer is convertible to node_id
// use node_pointer as node_id and as storage_node_pointer
// THIS MAY BE ENCLOSED IN STORAGE_PTR
// SIGNED TYPES HAS LESS DIGITS THAN UNSIGNED - THIS STRUCT IS WRONG - LEAVE IT FOR NOW, OPTIMIZE LATER
//template <typename MemId, typename StorageId>
//struct is_directly_convertible_to_id
//    : boost::mpl::and_<
//        boost::is_integral<StorageId>,
//        boost::mpl::bool_<std::numeric_limits<MemId>::digits == std::numeric_limits<StorageId>::digits>,
//        boost::is_convertible<MemId, StorageId>
//    >
//{};

template <typename Value, typename Options, typename Translator, typename Box, typename Allocators, typename Storage>
class storage_manager
    : public Allocators
    , public Storage
{
public:
    typedef typename Allocators::allocator_type allocator_type;
    typedef typename Allocators::node node;
    typedef typename Allocators::node_pointer node_pointer;
    //typedef typename Allocators::size_type size_type;

    typedef typename Storage::node_id node_id;
    typedef typename Storage::size_type storage_size_type; // different name

    typedef storage_ptr<node, node_pointer, node_id> storage_node_pointer;

    // allocator_type
    storage_manager(Allocators const& allocators, Storage const& storage)
        : Allocators(allocators), Storage(storage)
        , m_current_operation(none)
    {}

    template <typename Node>
    storage_node_pointer create()
    {
// TODO - not thread safe

        node_pointer n = rtree::create_node<Allocators, Node>(allocators());                // may throw

        node_id id = storage().new_id(static_cast<uintptr_t>(boost::addressof(*n)));        // may throw

        storage_node_pointer ptr(n, id);

        create_node<Node>(ptr);                                                             // may throw

        return ptr;
    }

    template <typename Node>
    void destroy(storage_node_pointer ptr)
    {
// TODO - thread safe in the current form (code commented out)

        //destroy_node<Node>(ptr);

        rtree::destroy_node<Allocators, Node>(allocators(), ptr.get());                     // shouldn't throw

        destroy_node<Node>(storage_node_pointer(node_pointer(0), ptr.id()));
    }

    template <typename Node>
    void modify(storage_node_pointer ptr)
    {
        modify_node<Node>(ptr);
    }

    // for serialization of nodes use wrapers, different than the ones used in whole-tree serialization.

    // header will be stored separately:
    // tree height
    // root id
    // algorithms
    // other?

    // in unbalanced tree nodes probably the type of the node will be stored

    node & dereference(storage_node_pointer ptr)
    {
        BOOST_GEOMETRY_INDEX_ASSERT(ptr.get(), "node should be acquired before dereference");
        return *ptr.get();
    }

// INFO
// Nodes aren't stored during acquire - dismiss

    template <typename Node>
    void acquire(storage_node_pointer & ptr)
    {
// TODO - not thread safe

        BOOST_GEOMETRY_INDEX_ASSERT(!ptr.get(), "can't acquire already acquired node");

        node_pointer n = rtree::create_node<Allocators, Node>(allocators());                    // may throw
        storage().load_node(ptr.id(), rtree::get<Node>(*n));                                    // may throw

        ptr = storage_node_pointer(n, ptr.id());
    }

// WARNING!
// Can't destroy node in dismiss if it were modified

// Dismiss will probably be used only in nonmodyfying operations, like queries
// If cacheing were used, acquire and dismiss should also store nodes in the map

    template <typename Node>
    void dismiss(storage_node_pointer & ptr)
    {
// TODO - not thread safe

        BOOST_GEOMETRY_INDEX_ASSERT(ptr.get(), "node should be acquired before dismiss");
// TODO - Later disable this assertion?
        BOOST_GEOMETRY_INDEX_ASSERT(m_nodes.find(ptr) == m_nodes.end(), "can't dismiss modified node");

        rtree::destroy_node<Allocators, Node>(allocators(), ptr.get());                         // shouldn't throw
        storage().unload_node(ptr.id());                                                        // may throw?

        ptr = storage_node_pointer(node_pointer(0), ptr.id());
    }

    enum operation { none, nonmodifying, incremental, immediate };

    void operation_begin(operation o) {}
    //void operation_end(operation o) {} - not needed since flush will be used

    void flush()
    {
        typedef typename nodes_map::iterator iter;

        // apply all changes / save all modified, created nodes / delete nodes
        if ( m_current_operation == incremental )
        {
            for ( iter it = m_nodes.begin() ; m_nodes.end() ; ++it )
            {
                if ( it->second == modified )
                    storage().save_node(it->first);
                else if ( it->second == destroyed )
                    storage().destroy_node(it->first);

// TODO
// Do something with other nodes?
// call storage().unload(it->first.id()) ?
            }
        }

        // TODO - cacheing
        for ( iter it = m_nodes.begin() ; it != m_nodes.end() ; ++it )
        {
            rtree::node_auto_ptr<Value, Options, Translator, Box, Allocators>
                auto_remover(it->first.get());
        }

        m_nodes.clear();
        m_current_operation = none;
    }

    storage_node_pointer root() { return null(); }

    storage_node_pointer null() { return storage_node_pointer(node_pointer(0), storage().null_id()); }

private:
    enum flag { unloaded, loaded, modified, destroyed };
    typedef std::map<storage_node_pointer, flag> nodes_map;

    template <typename Node>
    void create_node(storage_node_pointer ptr)
    {
        if ( m_current_operation == incremental )
        {
            //BOOST_GEOMETRY_INDEX_ASSERT(m_nodes.find(ptr) == m_nodes.end(), "node already created");
            m_nodes.insert(std::make_pair(ptr, modified));                              // may throw
        }
        else if ( m_current_operation == immediate )
        {
            storage().save_node(ptr.id(), rtree::get<Node>(dereference(ptr)));          // may throw
        }
        else
        {
            BOOST_GEOMETRY_INDEX_ASSERT(false, "valid operation wasn't begined");
        }
    }

    template <typename Node>
    void modify_node(storage_node_pointer ptr)
    {
        if ( m_current_operation == incremental )
        {
            typename nodes_map::iterator it = m_nodes.find(ptr);                        // shouldn't throw
            BOOST_GEOMETRY_INDEX_ASSERT(it != m_nodes.end() && it->second < deleted, "node should be stored and not deleted");
            it->second = modified;
        }
        else if ( m_current_operation == immediate )
        {
            storage().save_node(ptr.id(), rtree::get<Node>(dereference(ptr)));          // may throw
        }
        else
        {
            BOOST_GEOMETRY_INDEX_ASSERT(false, "valid operation wasn't begined");
        }
    }

    template <typename Node>
    void destroy_node(storage_node_pointer ptr)
    {
        if ( m_current_operation == incremental )
        {
            typename nodes_map::iterator it = m_nodes.find(ptr);
            BOOST_GEOMETRY_INDEX_ASSERT(it != m_nodes.end() && it->second < deleted, "node should be stored and not deleted");
            // really? or maybe
//            if ( it == m_nodes.end() )
//                m_nodes.insert(std::make_pair(ptr, deleted));                         // may throw if it will be here, otherwise it will be thread safe as it is
//            else
                it->second = destroyed;
        }
        else if ( m_current_operation == immediate )
        {
            storage().destroy_node(ptr.id());                                           // may throw
        }
        else
        {
            BOOST_GEOMETRY_INDEX_ASSERT(false, "valid operation wasn't begined");
        }
    }

    Allocators & allocators() { return *this; }
    Storage & storage() { return *this; }

    nodes_map m_nodes;
    operation m_current_operation;
};

}}}}} // namespace boost::geometry::index::detail::rtree

namespace boost { namespace geometry { namespace index {

struct none {};

// rtree empty storage
class rtree_storage
{
    // TODO
    // there must be a way to load the header and retrieve needed data

public:
    typedef uintptr_t node_id;
    typedef std::size_t size_type;

    node_id null_id() { return 0; }

    // or maybe return header?
    node_id root_id() { return null_id(); }

    template <typename T>
    node_id new_id(T seed)
    {
        BOOST_STATIC_ASSERT(sizeof(T) <= sizeof(node_id));
        return static_cast<node_id>(seed);
    }

    void destroy_node(node_id id) {}

    template <typename Node> // leaf or internal_node
    void save_node(node_id id, Node const& n) {}

    template <typename Node> // leaf or internal_node
    void load_node(node_id id, Node & n) {}

    void save_begin() {}
    void save_end() {}
    // ?
    void load_begin() {}
    void load_end() {}
    // or operation_begin(nonmodifying|incremental|immediate)s
};

}}} // namespace boost::geometry::index

// Also the header must be loaded/saved

// rename to serializing_storage
//template <typename Value>
//class storage
//{
//public:
//    typedef unsigned long long node_id;
//    typedef unsigned long long size_type;

//    storage(std::string const& dir)
//    {
//        // try to open header
//        // if ok, load header data and probably root as well
//    }

//    node_id null_id() { return 0; }

//    // or maybe return header?
//    node_id root_id() { return 0; }

//    template <typename T>
//    node_id new_id(T seed) {}
//    void delete_id(node_id id) {}

//    template <typename Node> // leaf or internal_node
//    void save_node(node_id id, Node const& n)
//    {
//        // if file isn't created, create it
//        // save node
////        typedef typename rtree::elements_type<Node>::type elements_type;
////        typedef typename elements_type::value_type element_type;
////        BOOST_FOREACH(element_type const& el, rtree::elements(n))
////            save_element(el);

//        // Bedzie problem z elementami movable-only
//        // Bedzie problem z tym, ze second pary w internal node bedzie wskaznikiem+id a nie tylko id
//        //   mozna to roznie rozwiazac, np. wrapper<Node> i udostepnienie tego co trzeba
//        //   lub ustandaryzowanie wskaznika przechowywanego jako second,
//        // lepiej byloby nie udostepniac wskaznikow
//    }

//    template <typename Node> // leaf or internal_node
//    void load_node(node_id id, Node & n) {}

//    // Node concept:
//    // elements_type<Node>::type
//    // elements(node)
//    // elements are stored in random access container
//    // internal nodes elements are stored in objects similar to pairs
//    //   - this may cause a problem if movable-only pairs will be used in the future
//    //   - values and pairs shouldn't be copied?

//    // This isn't fully ok
//    // It's possible that those won't be raw nodes but some wrappers

//    // Header concept:
//    // ?
//    // root, size, leafs_level

//    // a co jesli ktos bedzie chcial sobie np przechowac wskazniki do wezlow, ktore beda mialy byc zapisane w przyszlosci?
//    // bo nie powinien kopiowac wartosci skoro przewidywane jest zrobienie move-only
//    // chyba zeby udostepnic node_pointer do przechowywania

//    // lipa bedzie jesli najpierw wezel zostanie zmieniony a potem usuniety
//    // nie, ok poprostu status wszystkich zmienianych wezlow bedzie przechowywany w menedzerze - CREATED, MODIFIED, DELETED

//    void save_begin() {}
//    void save_end() {}

//private:
////    template <typename Element>
////    void save_element(Element const& el) {}
////    void save_element(Value const& el) {}
//};

//struct none {};

//template <typename Allocators>
//class storage_manager<Allocators, none>
//  : public Allocators
//{
//public:
//    typedef typename Allocators::allocator_type allocator_type;
//    typedef typename Allocators::node node;
//    typedef typename Allocators::node_pointer node_pointer;
//    typedef typename Allocators::size_type size_type;

//    typedef node_pointer storage_node_pointer;
//    typedef size_type storage_size_type;

//    storage_manager(Allocators const& allocators)
//        : Allocators(allocators)
//    {}

//    node & dereference(storage_node_pointer ptr) { return *ptr; }

//    storage_node_pointer load(storage_node_pointer ptr) { return ptr; }
//    void unload(storage_node_pointer /*ptr*/) {}

//    void modified(storage_node_pointer /*ptr*/, size_type /*child_index*/) {}
//    void save() {}

//    storage_node_pointer root() { return storage_node_pointer(0); }

//private:
//    Allocators & allocators() { return *this; }
//};

// allocator    storage
//     v          v
// allocators     v
//     v          v
// some_nodes_data_manager

#endif // BOOST_GEOMETRY_INDEX_DETAIL_RTREE_STORAGE_MANAGER_HPP
