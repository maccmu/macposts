// Adjacency list based graph implementation.
//
// This is a port of the graph library in the discontinued Rust version of
// macposts. The design is heavily inspired by the Rust `petgraph'[0] library,
// with simplifications and modifications specifically tailored for
// transportation networks.
//
// A note on the memory model: the graph owns all nodes and links. Users may use
// the returned references/pointers by graph methods, but should only do that
// when the graph is alive. Also, for memory safety, it is not allowed to delete
// nodes or links once added to the graph.
//
// [0]: https://github.com/petgraph/petgraph

// TODO: Move graph related functions (e.g., shortest path) to this library.
//
// TODO: Expose the graph class to Python.
//
// TODO: Mark functions as `noexcept' properly (maybe not very necessary?).
//
// NOTE: This library use smart pointers to manage resources and does not check
// the validity of pointers. However, it is still possible to trigger unsafe
// behaviors and lead to subtle bugs, e.g., deferencing an `end' iterator. Maybe
// add some basic checks?

#pragma once

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

namespace macposts
{
namespace graph
{
enum class Direction
{
  Incoming = 0,
  Outgoing = 1,
};

inline Direction
invert (Direction direction)
{
  switch (direction)
    {
    case Direction::Incoming:
      return Direction::Outgoing;
    case Direction::Outgoing:
      return Direction::Incoming;
    }
  throw std::runtime_error ("invalid direction");
}

// FIXME: If we do not want to attach data to a node/link, there is a waste of
// space in the current implementation.
template <class NId, class LId, class NData, class LData, bool directed>
class Graph
{
private:
  // Template for implementing various iterators on the graph
  template <class Value, class State, bool readonly,
            template <bool c> class Type>
  class Iterator
  {
  public:
    using iterator_category = std::input_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type =
      typename std::conditional<readonly, const Value, Value>::type;
    using pointer =
      typename std::conditional<readonly, const Value *, Value *>::type;
    using reference =
      typename std::conditional<readonly, const Value &, Value &>::type;

    explicit Iterator (State state, Direction direction)
        : current (state), direction (direction)
    {
    }

    void step () { ++current; }
    Type<readonly> &operator++ ()
    {
      auto self = static_cast<Type<readonly> *> (this);
      self->step ();
      return *self;
    }
    Type<readonly> operator++ (int)
    {
      auto self = static_cast<Type<readonly> *> (this);
      auto r = *self;
      ++(*self);
      return r;
    }

    bool operator== (const Type<readonly> &other) const
    {
      return (current == other.current) && (direction == other.direction);
    }
    bool operator!= (const Type<readonly> &other) const
    {
      auto self = static_cast<const Type<readonly> *> (this);
      return !(*self == other);
    }
    reference operator* () const { return *current; }

    // Convert from a non-const iterator to a const one
    //
    // NOTE: Implicit conversion is fine here because the two types of iterators
    // are almost the same, and such conversions are quite common and somewhat
    // expected (all STL containers support implicit conversion from `iterator'
    // to `const_iterator').
    //
    // HACK: This is really ugly, and relies on some (maybe false) assumptions
    // on the compiler and the implementation of the concrete iterator type --
    // the memory layout of a const iterator is the same as the non-const
    // counter part. Perhaps we should define this method individually.
    template <bool c = readonly, typename std::enable_if<!c, int>::type = 0>
    operator Type<true> () const
    {
      auto r = reinterpret_cast<const Type<true> *> (this);
      // HACK: This is to circumvent strict aliasing violation. We should really
      // use `std::launder' but that requires C++17.
      asm ("" : "+r"(r));
      return *r;
    }

  protected:
    State current;
    const Direction direction;
  };

public:
  // Basics: node and link
  class Link;
  class Node
  {
  public:
    NData data;

  private:
    friend class Graph;
    std::array<Link *, 2> next;
    // I originally planned to go with pointers/references only but in the
    // existing code base IDs are heavily used.
    const NId id;

  public:
    Node (const Node &) = delete;
    Node &operator= (const Node &) = delete;

  private:
    explicit Node (NId id, NData data)
        : data (std::move (data)), next ({ nullptr, nullptr }), id (id)
    {
    }
  };

  class Link
  {
  public:
    LData data;

  private:
    friend class Graph;
    std::array<Node *, 2> endpoints;
    std::array<Link *, 2> next;
    // I originally planned to go with pointers/references only but in the
    // existing code base IDs are heavily used.
    const LId id;

  public:
    Link (const Link &) = delete;
    Link &operator= (const Link &) = delete;

  private:
    explicit Link (LId id, LData data)
        : data (std::move (data)), endpoints ({ nullptr, nullptr }),
          next ({ nullptr, nullptr }), id (id)
    {
    }
  };

public:
  // Collections of nodes/links
  template <bool readonly> class Nodes
  {
  private:
    // NOTE: Because we use `unique_ptr' with a non-const pointer type,
    // `const_iterator' is good for non-const case as well.
    using State =
      typename std::unordered_map<NId, std::unique_ptr<Node>>::const_iterator;
    template <bool readonly_>
    class Iterator_ : public Iterator<Node, State, readonly_, Iterator_>
    {
    private:
      using Base = Iterator<Node, State, readonly_, Iterator_>;

    public:
      explicit Iterator_ (State state) : Base (state, Direction::Incoming) {}

      typename Base::reference operator* () const
      {
        return *this->current->second;
      }
    };

    const State head;
    const State tail;

  public:
    using iterator = Iterator_<readonly>;
    using const_iterator = Iterator_<true>;

    explicit Nodes (State head, State tail) : head (head), tail (tail) {}

    iterator begin () const { return iterator (head); }
    iterator end () const { return iterator (tail); }

    const_iterator cbegin () const
    {
      return static_cast<const_iterator> (begin ());
    }
    const_iterator cend () const
    {
      return static_cast<const_iterator> (end ());
    }

    bool empty () const { return begin () == end (); };
  };

  template <bool readonly> class Neighbors
  {
  private:
    template <bool readonly_>
    class Iterator_ : public Iterator<Node, Link *, readonly_, Iterator_>
    {
    private:
      using Base = Iterator<Node, Link *, readonly_, Iterator_>;
      std::unordered_set<const Node *> seen;
      Link *refill;

    public:
      explicit Iterator_ (Link *state, Direction direction, Link *refill)
          : Base (state, direction), seen (), refill (refill)
      {
      }

      void step ()
      {
        Link *current = this->current;
        int direction = static_cast<int> (this->direction);

        // Invariant: before stepping to the next neighbor, the current one
        // should have never been seen.
        //
        // TODO: Switch to `assert'.
#ifndef NDEBUG
        if (current && seen.count (current->endpoints[direction]))
          __builtin_trap ();
#endif

        while (current || refill)
          {
            if (!current)
              {
                current = refill;
                refill = nullptr;
                const_cast<Direction &> (this->direction)
                  = invert (this->direction);
                direction = static_cast<int> (this->direction);
              }
            else
              {
                seen.insert (current->endpoints[direction]);
              }

            if (!seen.count (current->endpoints[direction]))
              break;
            else
              current = current->next[direction];
          }
        this->current = current;
      }

      typename Base::reference operator* () const
      {
        return *this->current->endpoints[static_cast<int> (this->direction)];
      }

      bool operator== (const Iterator_ &other) const
      {
        if (directed)
          {
            // Ignore `refill' if directed
            return Base::operator== (other);
          }
        else
          {
            // Ignore direction if empty and directed
            bool empty
              = !(this->current || other.current || refill || other.refill);
            return empty
                   || (Base::operator== (other) && (refill == other.refill));
          }
      }
    };

    const Node &start;
    const Direction direction;

  public:
    using iterator = Iterator_<readonly>;
    using const_iterator = Iterator_<true>;

    explicit Neighbors (const Node &start, Direction direction)
        : start (start),
          direction (directed || start.next[static_cast<int> (direction)]
                       ? direction
                       : invert (direction))
    {
    }

    iterator begin () const
    {
      Link *next = start.next[static_cast<int> (direction)];
      Link *refill = directed
                       ? nullptr
                       : start.next[static_cast<int> (invert (direction))];
      return iterator (next, direction, refill);
    }
    iterator end () const { return iterator (nullptr, direction, nullptr); }

    const_iterator cbegin () const
    {
      return static_cast<const_iterator> (begin ());
    }
    const_iterator cend () const
    {
      return static_cast<const_iterator> (end ());
    }

    bool empty () const { return begin () == end (); };
  };

  template <bool readonly> class Links
  {
  private:
    // NOTE: Because we use `unique_ptr' with a non-const pointer type,
    // `const_iterator' is good for non-const case as well.
    using State =
      typename std::unordered_map<LId, std::unique_ptr<Link>>::const_iterator;
    template <bool readonly_>
    class Iterator_ : public Iterator<Link, State, readonly_, Iterator_>
    {
    private:
      using Base = Iterator<Link, State, readonly_, Iterator_>;

    public:
      explicit Iterator_ (State state) : Base (state, Direction::Incoming) {}

      typename Base::reference operator* () const
      {
        return *this->current->second;
      }
    };

    const State head;
    const State tail;

  public:
    using iterator = Iterator_<readonly>;
    using const_iterator = Iterator_<true>;

    explicit Links (State head, State tail) : head (head), tail (tail) {}

    iterator begin () const { return iterator (head); }
    iterator end () const { return iterator (tail); }

    const_iterator cbegin () const
    {
      return static_cast<const_iterator> (begin ());
    }
    const_iterator cend () const
    {
      return static_cast<const_iterator> (end ());
    }

    bool empty () const { return begin () == end (); };
  };

  template <bool readonly> class Connections
  {
  private:
    template <bool readonly_>
    class Iterator_ : public Iterator<Link, Link *, readonly_, Iterator_>
    {
    private:
      using Base = Iterator<Link, Link *, readonly_, Iterator_>;
      Link *refill;

    public:
      explicit Iterator_ (Link *state, Direction direction, Link *refill)
          : Base (state, direction), refill (refill)
      {
      }

      void step ()
      {
        Link *current = this->current;
        if (current)
          current = current->next[static_cast<int> (this->direction)];
        if (!current && refill)
          {
            current = refill;
            refill = nullptr;
            const_cast<Direction &> (this->direction)
              = invert (this->direction);
          }
        this->current = current;
      }

      bool operator== (const Iterator_ &other) const
      {
        if (directed)
          {
            // Ignore `refill' if directed
            return Base::operator== (other);
          }
        else
          {
            // Ignore direction if empty and undirected
            bool empty
              = !(this->current || other.current || refill || other.refill);
            return empty
                   || (Base::operator== (other) && (refill == other.refill));
          }
      }
    };

    const Node &start;
    const Direction direction;

  public:
    using iterator = Iterator_<readonly>;
    using const_iterator = Iterator_<true>;

    explicit Connections (const Node &start, Direction direction)
        : start (start),
          direction (directed || start.next[static_cast<int> (direction)]
                       ? direction
                       : invert (direction))
    {
    }

    iterator begin () const
    {
      Link *next = start.next[static_cast<int> (direction)];
      Link *refill = directed
                       ? nullptr
                       : start.next[static_cast<int> (invert (direction))];
      return iterator (next, direction, refill);
    }
    iterator end () const { return iterator (nullptr, direction, nullptr); }

    const_iterator cbegin () const
    {
      return static_cast<const_iterator> (begin ());
    }
    const_iterator cend () const
    {
      return static_cast<const_iterator> (end ());
    }

    bool empty () const { return begin () == end (); };
  };

  // Graph methods
  explicit Graph () : nodes_ (), links_ () {}

  // Add a node
  Node &add_node (NId id) { return add_node (id, NData ()); }
  Node &add_node (NId id, NData data)
  {
    if (nodes_.count (id))
      throw std::runtime_error ("node " + std::to_string (id)
                                + " already in graph");
    auto node = std::unique_ptr<Node> (new Node (id, std::move (data)));
    auto &r = *node;
    nodes_.insert ({ id, std::move (node) });
    return r;
  }
  Node &get_node (NId id) { return *nodes_.at (id); }
  const Node &get_node (NId id) const { return *nodes_.at (id); }

  // Add a link
  Link &add_link (Node &from, Node &to, LId id)
  {
    return add_link (from, to, id, LData ());
  }
  Link &add_link (Node &from, Node &to, LId id, LData data)
  {
    if (links_.count (id))
      throw std::runtime_error ("link " + std::to_string (id)
                                + " already in graph");
    if (&to == &from)
      throw std::runtime_error ("self-loops are not allowed");

    static const int incoming = static_cast<int> (Direction::Incoming);
    static const int outgoing = static_cast<int> (Direction::Outgoing);

    auto link = std::unique_ptr<Link> (new Link (id, std::move (data)));
    link->endpoints[incoming] = &from;
    link->endpoints[outgoing] = &to;
    link->next[incoming] = to.next[incoming];
    link->next[outgoing] = from.next[outgoing];
    from.next[outgoing] = link.get ();
    to.next[incoming] = link.get ();

    auto &r = *link;
    links_.insert ({ id, std::move (link) });
    return r;
  }
  Link &add_link (NId from, NId to, LId id)
  {
    return add_link (from, to, id, LData ());
  }
  Link &add_link (NId from, NId to, LId id, LData data)
  {
    auto &&from_ = get_node (from);
    auto &&to_ = get_node (to);
    return add_link (from_, to_, id, std::move (data));
  }
  Link &get_link (LId id) { return *links_.at (id); }
  const Link &get_link (LId id) const { return *links_.at (id); }

  NId get_id (const Node &node) const { return node.id; }
  LId get_id (const Link &link) const { return link.id; }

  std::size_t size_nodes () const { return nodes_.size (); }
  std::size_t size_links () const { return links_.size (); }

  Nodes<false> nodes ()
  {
    return Nodes<false> (nodes_.begin (), nodes_.end ());
  }
  Nodes<true> nodes () const
  {
    return Nodes<true> (nodes_.begin (), nodes_.end ());
  }
  Links<false> links ()
  {
    return Links<false> (links_.begin (), links_.end ());
  }
  Links<true> links () const
  {
    return Links<true> (links_.begin (), links_.end ());
  }

  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Neighbors<false> neighbors (const Node &node, Direction direction)
  {
    return Neighbors<false> (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Neighbors<false> neighbors (const Node &node)
  {
    return Neighbors<false> (node, Direction::Outgoing);
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Neighbors<true> neighbors (const Node &node, Direction direction) const
  {
    return Neighbors<true> (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Neighbors<true> neighbors (const Node &node) const
  {
    return Neighbors<true> (node, Direction::Outgoing);
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Neighbors<false> neighbors (NId id, Direction direction)
  {
    return neighbors (get_node (id), direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Neighbors<false> neighbors (NId id)
  {
    return neighbors (get_node (id));
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Neighbors<true> neighbors (NId id, Direction direction) const
  {
    return neighbors (get_node (id), direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Neighbors<true> neighbors (NId id) const
  {
    return neighbors (get_node (id));
  }

  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Connections<false> connections (const Node &node, Direction direction)
  {
    return Connections<false> (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Connections<false> connections (const Node &node)
  {
    return Connections<false> (node, Direction::Incoming);
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Connections<true> connections (const Node &node, Direction direction) const
  {
    return Connections<true> (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Connections<true> connections (const Node &node) const
  {
    return Connections<true> (node, Direction::Incoming);
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Connections<false> connections (NId id, Direction direction)
  {
    return connections (get_node (id), direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Connections<false> connections (NId id)
  {
    return connections (get_node (id));
  }
  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Connections<true> connections (NId id, Direction direction) const
  {
    return connections (get_node (id));
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Connections<true> connections (NId id) const
  {
    return connections (get_node (id));
  }

  std::pair<Node &, Node &> get_endpoints (const Link &link)
  {
    auto r = static_cast<const Graph *> (this)->get_endpoints (link);
    return { const_cast<Node &> (r.first), const_cast<Node &> (r.second) };
  }
  std::pair<const Node &, const Node &> get_endpoints (const Link &link) const
  {
    auto from = link.endpoints[static_cast<int> (Direction::Incoming)];
    auto to = link.endpoints[static_cast<int> (Direction::Outgoing)];
    if (!from || !to)
      {
        // This should be unreachable if the graph, links, and nodes are created
        // and managed through the designated interface.
        throw std::runtime_error ("invalid link");
      }
    return { *from, *to };
  }
  std::pair<Node &, Node &> get_endpoints (LId id)
  {
    const auto &link = get_link (id);
    return get_endpoints (link);
  }
  std::pair<const Node &, const Node &> get_endpoints (LId id) const
  {
    const auto &link = get_link (id);
    return get_endpoints (link);
  }

protected:
  std::unordered_map<NId, std::unique_ptr<Node>> nodes_;
  std::unordered_map<LId, std::unique_ptr<Link>> links_;
};

template <class NData, class LData>
using DiGraph = Graph<int, int, NData, LData, true>;
template <class NData, class LData>
using UnGraph = Graph<int, int, NData, LData, false>;
}
}
