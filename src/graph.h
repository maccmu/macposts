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
template <class NIdx, class LIdx, class NData, class LData, bool directed>
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
    template <bool c = readonly, typename std::enable_if<!c, int>::type = 0>
    operator Type<true> () const
    {
      // HACK: This is really ugly, and is very prone to error. This works
      // safely because the difference between a non-const iterator and a const
      // one *only* lies on the `const' specifier on the value, pointer, and
      // reference types. When modifying this template, please be sure to not
      // violate that contract.
      auto r = reinterpret_cast<const Type<true> *> (this);
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

  public:
    Node (const Node &) = delete;
    Node &operator= (const Node &) = delete;

  private:
    explicit Node (NData data)
        : data (std::move (data)), next ({ nullptr, nullptr })
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

  public:
    Link (const Link &) = delete;
    Link &operator= (const Link &) = delete;

  private:
    explicit Link (LData data)
        : data (std::move (data)), endpoints ({ nullptr, nullptr }),
          next ({ nullptr, nullptr })
    {
    }
  };

public:
  // Collections of nodes/links
  class Nodes
  {
  private:
    using State =
      typename std::unordered_map<NIdx, std::unique_ptr<Node>>::iterator;
    template <bool readonly>
    class Iterator_ : public Iterator<Node, State, readonly, Iterator_>
    {
    private:
      using Base = Iterator<Node, State, readonly, Iterator_>;

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
    using iterator = Iterator_<false>;
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
  };

  class Neighbors
  {
  private:
    template <bool readonly>
    class Iterator_ : public Iterator<Node, Link *, readonly, Iterator_>
    {
    private:
      using Base = Iterator<Node, Link *, readonly, Iterator_>;
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
        bool wrapped = false;

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
                wrapped = true;
              }

            Node *node = current->endpoints[direction];
            if (!wrapped)
              seen.insert (node);
            if (!seen.count (node))
              break;
            else
              current = current->next[direction];
          }
        this->current = current;
      }

      typename Base::reference operator* ()
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
    using iterator = Iterator_<false>;
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
  };

  class Links
  {
  private:
    using State =
      typename std::unordered_map<LIdx, std::unique_ptr<Link>>::iterator;
    template <bool readonly>
    class Iterator_ : public Iterator<Link, State, readonly, Iterator_>
    {
    private:
      using Base = Iterator<Link, State, readonly, Iterator_>;

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
    using iterator = Iterator_<false>;
    using const_iterator = Iterator_<true>;

    explicit Links (State head, State tail) : head (head), tail (tail) {}

    iterator begin () const { return iterator (head); }
    iterator end () const { return iterator (tail); }

    const_iterator cbegin () const { return begin (); }
    const_iterator cend () const { return end (); }
  };

  class Connections
  {
  private:
    template <bool readonly>
    class Iterator_ : public Iterator<Link, Link *, readonly, Iterator_>
    {
    private:
      using Base = Iterator<Link, Link *, readonly, Iterator_>;
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
    using iterator = Iterator_<false>;
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
  };

  // Graph methods
  explicit Graph () : nodes_ (), links_ () {}

  // Add a node
  Node &add_node (NIdx idx) { return add_node (idx, NData ()); }
  Node &add_node (NIdx idx, NData data)
  {
    if (nodes_.count (idx))
      throw std::runtime_error ("node " + std::to_string (idx)
                                + " already in graph");
    auto node = std::unique_ptr<Node> (new Node (std::move (data)));
    auto &r = *node;
    nodes_.insert ({ idx, std::move (node) });
    return r;
  }
  Node &get_node (NIdx idx) const { return *nodes_.at (idx); }

  // Add a link
  Link &add_link (Node &from, Node &to, LIdx idx)
  {
    return add_link (from, to, idx, LData ());
  }
  Link &add_link (Node &from, Node &to, LIdx idx, LData data)
  {
    if (links_.count (idx))
      throw std::runtime_error ("link " + std::to_string (idx)
                                + " already in graph");
    if (&to == &from)
      throw std::runtime_error ("self-loops are not allowed");

    static const int incoming = static_cast<int> (Direction::Incoming);
    static const int outgoing = static_cast<int> (Direction::Outgoing);

    auto link = std::unique_ptr<Link> (new Link (std::move (data)));
    link->endpoints[incoming] = &from;
    link->endpoints[outgoing] = &to;
    link->next[incoming] = to.next[incoming];
    link->next[outgoing] = from.next[outgoing];
    from.next[outgoing] = link.get ();
    to.next[incoming] = link.get ();

    auto &r = *link;
    links_.insert ({ idx, std::move (link) });
    return r;
  }
  Link &add_link (NIdx from, NIdx to, LIdx idx)
  {
    return add_link (from, to, idx, LData ());
  }
  Link &add_link (NIdx from, NIdx to, LIdx idx, LData data)
  {
    auto &&from_ = get_node (from);
    auto &&to_ = get_node (to);
    return add_link (from_, to_, idx, std::move (data));
  }
  Link &get_link (LIdx idx) const { return *links_.at (idx); }

  std::size_t size_nodes () const { return nodes_.size (); }
  std::size_t size_links () const { return links_.size (); }
  Nodes nodes () { return Nodes (nodes_.begin (), nodes_.end ()); }
  Links links () { return Links (links_.begin (), links_.end ()); }

  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Neighbors neighbors (const Node &node, Direction direction)
  {
    return Neighbors (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Neighbors neighbors (const Node &node)
  {
    return Neighbors (node, Direction::Outgoing);
  }

  template <bool d = directed, typename std::enable_if<d, int>::type = 0>
  Connections connections (const Node &node, Direction direction)
  {
    return Connections (node, direction);
  }
  template <bool d = directed, typename std::enable_if<!d, int>::type = 0>
  Connections connections (const Node &node)
  {
    return Connections (node, Direction::Incoming);
  }

  std::pair<Node &, Node &> get_endpoints (const Link &link)
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

protected:
  std::unordered_map<NIdx, std::unique_ptr<Node>> nodes_;
  std::unordered_map<LIdx, std::unique_ptr<Link>> links_;
};

template <class NData, class LData>
using DiGraph = Graph<int, int, NData, LData, true>;
template <class NData, class LData>
using UnGraph = Graph<int, int, NData, LData, false>;
}
}
