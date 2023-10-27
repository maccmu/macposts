#include "graph.h"

namespace macposts
{
namespace graph
{
template <typename NData, typename LData>
DiGraph<NData, LData>::Node::Node (NData data)
    : data (data), next ({ nullptr, nullptr }){};

template <typename NData, typename LData>
DiGraph<NData, LData>::Link::Link (LData data)
    : data (data), endpoints ({ nullptr, nullptr }),
      next ({ nullptr, nullptr }){};

template <typename NData, typename LData>
DiGraph<NData, LData>::DiGraph () : nodes (), links (){};

template <typename NData, typename LData>
DiGraph<NData, LData>::Nodes::Nodes (Node &start, Direction direction)
    : start (start), direction (direction){};

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Nodes::const_iterator
DiGraph<NData, LData>::Nodes::begin () const noexcept
{
  Link *next = start.next[direction];
  return const_iterator (next, direction);
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Nodes::const_iterator
DiGraph<NData, LData>::Nodes::end () const noexcept
{
  return const_iterator (nullptr, direction);
}

template <typename NData, typename LData>
DiGraph<NData, LData>::Nodes::const_iterator::const_iterator (
  Link *start, Direction direction)
    : current (start), direction (direction), seen (){};

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Nodes::const_iterator &
DiGraph<NData, LData>::Nodes::const_iterator::operator++ ()
{
  if (current)
    {
      seen.insert (current->endpoints[direction]);
      do
        {
          current = current->next[direction];
        }
      while (current && seen.count (current->endpoints[direction]));
    }
  return *this;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Nodes::const_iterator
DiGraph<NData, LData>::Nodes::const_iterator::operator++ (int)
{
  const_iterator r = *this;
  ++(*this);
  return r;
}

template <typename NData, typename LData>
DiGraph<NData, LData>::Links::Links (Node &start, Direction direction)
    : start (start), direction (direction){};

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Links::const_iterator
DiGraph<NData, LData>::Links::begin () const noexcept
{
  Link *next = start.next[direction];
  return const_iterator (next, direction);
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Links::const_iterator
DiGraph<NData, LData>::Links::end () const noexcept
{
  return const_iterator (nullptr, direction);
}

template <typename NData, typename LData>
DiGraph<NData, LData>::Links::const_iterator::const_iterator (
  Link *start, Direction direction)
    : current (start), direction (direction){};

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Links::const_iterator &
DiGraph<NData, LData>::Links::const_iterator::operator++ ()
{
  if (current)
    current = current->next[direction];
  return *this;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Links::const_iterator
DiGraph<NData, LData>::Links::const_iterator::operator++ (int)
{
  const_iterator r = *this;
  ++(*this);
  return r;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Node &
DiGraph<NData, LData>::add_node (NData data)
{
  auto node = std::unique_ptr<Node> (new Node (data));
  auto &r = *node;
  nodes.push_back (std::move (node));
  return r;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Link &
DiGraph<NData, LData>::add_link (Node &from, Node &to, LData data)
{
  auto link = std::unique_ptr<Link> (new Link (data));
  link->endpoints[Incoming] = &from;
  link->endpoints[Outgoing] = &to;
  link->next[Incoming] = to.next[Incoming];
  link->next[Outgoing] = from.next[Outgoing];
  from.next[Outgoing] = link.get ();
  to.next[Incoming] = link.get ();
  auto &r = *link;
  links.push_back (std::move (link));
  return r;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Nodes
DiGraph<NData, LData>::neighbors (Node &node, Direction direction)
{
  auto nodes = Nodes (node, direction);
  return nodes;
}

template <typename NData, typename LData>
typename DiGraph<NData, LData>::Links
DiGraph<NData, LData>::connections (Node &node, Direction direction)
{
  auto links = Links (node, direction);
  return links;
}
}
}

#ifdef LIBMACPOSTS_INTERNAL_TEST

#  define assert(c)                                                            \
    if (!(c))                                                                  \
      __builtin_trap ();

int
main (void)
{
  using macposts::graph::Incoming;
  using macposts::graph::Outgoing;
  using DiGraph = typename macposts::graph::DiGraph<int, int>;
  using Node = typename DiGraph::Node;
  using Link = typename DiGraph::Link;

  // Test case 1: normal graph
  {
    DiGraph g;
    auto &&n0 = g.add_node (0);
    auto &&n1 = g.add_node (1);
    auto &&n2 = g.add_node (2);
    auto &&l0 = g.add_link (n0, n1, 0);
    auto &&l1 = g.add_link (n1, n2, 1);

    {
      std::unordered_set<const Node *> ns;
      for (const auto &n : g)
        ns.insert (n.get ());
      assert (std::distance (g.cbegin (), g.cend ()) == 3);
      assert (ns.count (&n0));
      assert (ns.count (&n1));
      assert (ns.count (&n2));
    }

    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n0, Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n1));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n0, Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (std::distance (nodes.cbegin (), nodes.cend ()) == 0);
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n1, Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n2));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n1, Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n0));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n2, Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 0);
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n2, Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n1));
    }

    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l0));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l1));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l0));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n2, Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n2, Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l1));
    }
  }

  // Test case 2: parallel links
  {
    DiGraph g;
    auto &&n0 = g.add_node (0);
    auto &&n1 = g.add_node (1);
    auto &&l0 = g.add_link (n0, n1, 0);
    auto &&l1 = g.add_link (n0, n1, 1);
    auto &&l2 = g.add_link (n0, n1, 2);

    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 0);
    }

    {
      auto nodes = g.neighbors (n0, Incoming);
      auto it0 = nodes.cbegin ();
      auto it1 = it0++;
      assert (std::distance (it0, nodes.cend ()) == 0);
      assert (std::distance (it1, nodes.cend ()) == 0);
    }
    {
      auto nodes = g.neighbors (n0, Outgoing);
      auto it0 = nodes.cbegin ();
      auto it1 = it0++;
      assert (std::distance (it0, nodes.cend ()) == 0);
      assert (std::distance (it1, nodes.cend ()) == 1);
    }

    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 3);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.count (&l2));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 3);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.count (&l2));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 0);
    }
  }

  // Test case 3: cycles
  {
    DiGraph g;
    auto &&n0 = g.add_node (0);
    auto &&n1 = g.add_node (1);
    auto &&l0 = g.add_link (n0, n1, 0);
    auto &&l1 = g.add_link (n1, n0, 1);

    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }

    {
      std::vector<const Link *> ls;
      auto links = g.connections (n0, Incoming);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l1);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n0, Outgoing);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l0);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n1, Incoming);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l0);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n1, Outgoing);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l1);
    }
  }

  return 0;
}

#endif
