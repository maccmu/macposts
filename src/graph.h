// Adjacency list based directed graph implementation.
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
// TODO: Add undirected graph.
//
// TODO: Expose the graph class to Python.
//
// TODO: Mark functions as `noexcept' properly (maybe not very necessary?).
//
// NOTE: This library use smart pointers to manage resources and does not check
// the validaty of pointers. However, it is still possible to trigger unsafe
// behaviors and lead to subtle bugs, e.g., deferencing an `end' iterator. Maybe
// add some basic checks?

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace macposts
{
namespace graph
{
enum class Direction
{
  Incoming = 0,
  Outgoing = 1,
};

template <typename NData, typename LData> class DiGraph
{
protected:
  // Internal hash map type used to store nodes and links. Creating an alias
  // here to avoid typing the overlong name everywhere.
  //
  // NOTE: It is generally a bad idea to use references as hash map keys due to
  // the potential mismatch of lifetimes between keys and values. However, in
  // this case it is okay because value (node or link) owns the key (data).
  //
  // FIXME: `unordered_map' does not provide any guarantee on the order of
  // elements when enumerating, which is not ideal for reproducibility of
  // experiment results. However, `map' is an overkill in this case because we
  // only need a deterministic ordering of elements and sorting elements is too
  // much for this purpose. One solution would be using a vector to store the
  // insertion order of elements.
  template <typename Key, typename Value>
  using hashmap =
    typename std::unordered_map<std::reference_wrapper<const Key>,
                                std::unique_ptr<Value>, std::hash<Key>,
                                std::equal_to<Key>>;

public:
  class Link;
  class Node
  {
  public:
    NData data;

  private:
    friend class DiGraph;
    std::array<Link *, 2> next;

  public:
    Node (const Node &) = delete;
    Node &operator= (const Node &) = delete;

  private:
    explicit Node (NData data);
  };

  class Link
  {
  public:
    LData data;

  private:
    friend class DiGraph;
    std::array<Node *, 2> endpoints;
    std::array<Link *, 2> next;

  public:
    Link (const Link &) = delete;
    Link &operator= (const Link &) = delete;

  private:
    explicit Link (LData data);
  };

  class Nodes
  {
  public:
    class const_iterator
    {
    public:
      using iterator_category = std::input_iterator_tag;
      using value_type = Node;
      using difference_type = std::ptrdiff_t;
      using pointer = const Node *;
      using reference = const Node &;

    private:
      Link *current;
      const Direction direction;
      std::unordered_set<const Node *> seen;

    public:
      explicit const_iterator (Link *start, Direction direction);
      const_iterator &operator++ ();
      const_iterator operator++ (int);

      inline bool operator== (const const_iterator &other) const
      {
        return (current == other.current) && (direction == other.direction);
      }
      inline bool operator!= (const const_iterator &other) const
      {
        return (current != other.current) || (direction != other.direction);
      }
      inline reference operator* () const
      {
        return *current->endpoints[static_cast<int> (direction)];
      }
    };

  private:
    const Node &start;
    const Direction direction;

  public:
    explicit Nodes (Node &start, Direction direction);
    const_iterator begin () const;
    const_iterator end () const;

    inline const_iterator cbegin () const { return begin (); }
    inline const_iterator cend () const { return end (); }
  };

  class Links
  {
  public:
    class const_iterator
    {
    public:
      using iterator_category = std::input_iterator_tag;
      using value_type = Link;
      using difference_type = std::ptrdiff_t;
      using pointer = const Link *;
      using reference = const Link &;

    private:
      Link *current;
      const Direction direction;

    public:
      explicit const_iterator (Link *start, Direction direction);
      const_iterator &operator++ ();
      const_iterator operator++ (int);

      inline bool operator== (const const_iterator &other) const
      {
        return (current == other.current) && (direction == other.direction);
      }
      inline bool operator!= (const const_iterator &other) const
      {
        return (current != other.current) || (direction != other.direction);
      }
      inline reference operator* () const { return *current; }
    };

  private:
    const Node &start;
    const Direction direction;

  public:
    explicit Links (Node &start, Direction direction);
    const_iterator begin () const;
    const_iterator end () const;

    inline const_iterator cbegin () const { return begin (); }
    inline const_iterator cend () const { return end (); }
  };

  class const_iterator
  {
  public:
    using iterator_category = std::input_iterator_tag;
    using value_type = Node;
    using difference_type = std::ptrdiff_t;
    using pointer = const Node *;
    using reference = const Node &;

  private:
    typename hashmap<NData, Node>::const_iterator current;

  public:
    explicit const_iterator (
      typename hashmap<NData, Node>::const_iterator start);
    const_iterator &operator++ ();
    const_iterator operator++ (int);

    inline bool operator== (const const_iterator &other) const
    {
      return current == other.current;
    }
    inline bool operator!= (const const_iterator &other) const
    {
      return current != other.current;
    }
    inline reference operator* () const { return *current->second; }
  };

protected:
  hashmap<NData, Node> nodes;
  hashmap<LData, Link> links;

public:
  explicit DiGraph ();
  Node &add_node (NData data);
  Link &add_link (Node &from, Node &to, LData data);
  Link &add_link (const NData &from, const NData &to, LData data);
  Nodes neighbors (Node &node, Direction direction);
  Links connections (Node &node, Direction direction);
  Node &get_endpoint (const Link &link, Direction direction) const;

  inline std::size_t size_nodes () const { return nodes.size (); }
  inline std::size_t size_links () const { return links.size (); }
  inline Node &get_node (const NData &data) const { return *nodes.at (data); }
  inline Link &get_link (const LData &data) const { return *links.at (data); }
  inline const_iterator begin () const
  {
    return const_iterator (nodes.begin ());
  }
  inline const_iterator end () const { return const_iterator (nodes.end ()); }
  inline const_iterator cbegin () const
  {
    return const_iterator (nodes.cbegin ());
  }
  inline const_iterator cend () const { return const_iterator (nodes.cend ()); }
};
}
}
