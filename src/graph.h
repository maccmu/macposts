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

#pragma once

#include <array>
#include <memory>
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
    Node (NData data);
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
    Link (LData data);
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
      };
      inline bool operator!= (const const_iterator &other) const
      {
        return (current != other.current) || (direction != other.direction);
      };
      inline reference operator* () const
      {
        return *current->endpoints[static_cast<int> (direction)];
      };
    };

  private:
    const Node &start;
    const Direction direction;

  public:
    Nodes (Node &start, Direction direction);
    const_iterator begin () const noexcept;
    const_iterator end () const noexcept;

    inline const_iterator cbegin () const noexcept { return begin (); };
    inline const_iterator cend () const noexcept { return end (); };
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
      };
      inline bool operator!= (const const_iterator &other) const
      {
        return (current != other.current) || (direction != other.direction);
      };
      inline reference operator* () const { return *current; }
    };

  private:
    const Node &start;
    const Direction direction;

  public:
    Links (Node &start, Direction direction);
    const_iterator begin () const noexcept;
    const_iterator end () const noexcept;

    inline const_iterator cbegin () const noexcept { return begin (); };
    inline const_iterator cend () const noexcept { return end (); };
  };

protected:
  std::vector<std::unique_ptr<Node>> nodes;
  std::vector<std::unique_ptr<Link>> links;

public:
  DiGraph ();
  Node &add_node (NData data);
  Link &add_link (Node &from, Node &to, LData data);
  Nodes neighbors (Node &node, Direction direction);
  Links connections (Node &node, Direction direction);

  inline std::size_t size_nodes () const noexcept { return nodes.size (); };
  inline std::size_t size_links () const noexcept { return links.size (); };
  inline typename std::vector<std::unique_ptr<Node>>::const_iterator
  begin () const noexcept
  {
    return nodes.begin ();
  };
  inline typename std::vector<std::unique_ptr<Node>>::const_iterator
  end () const noexcept
  {
    return nodes.end ();
  };
  inline typename std::vector<std::unique_ptr<Node>>::const_iterator
  cbegin () const noexcept
  {
    return nodes.cbegin ();
  }
  inline typename std::vector<std::unique_ptr<Node>>::const_iterator
  cend () const noexcept
  {
    return nodes.cend ();
  };
};
}
}
