#ifdef LIBMACPOSTS_INTERNAL_TEST

// clang-format off
#include "graph.h"
#define assert(c) if (!(c)) __builtin_trap ()
// clang-format on

// TODO: Turn those into real unit test cases.

int
main (void)
{
  using macposts::graph::Direction;
  using DiGraph = typename macposts::graph::DiGraph<int, int>;
  using Node = typename DiGraph::Node;
  using Link = typename DiGraph::Link;

  // Test case 1: normal graph
  {
    DiGraph g;
    auto &&n0 = g.add_node (0);
    auto &&n1 = g.add_node (1);
    auto &&n2 = g.add_node (2);
    // Add link by node references
    auto &&l0 = g.add_link (n0, n1, 0);
    // Add link by node data (ID)
    auto &&l1 = g.add_link (1, 2, 1);

    {
      std::unordered_set<const Node *> ns;
      for (const auto &n : g)
        ns.insert (&n);
      assert (std::distance (g.cbegin (), g.cend ()) == 3);
      assert (ns.count (&n0));
      assert (ns.count (&n1));
      assert (ns.count (&n2));
    }

    {
      assert (&n0 == &g.get_node (0));
      assert (&n1 == &g.get_node (1));
      assert (&n2 == &g.get_node (2));
      assert (&l0 == &g.get_link (0));
      assert (&l1 == &g.get_link (1));
      assert (&n0 == &g.get_endpoint (l0, Direction::Incoming));
      assert (&n1 == &g.get_endpoint (l0, Direction::Outgoing));
    }

    {
      try
        {
          g.add_node (0);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
      try
        {
          g.add_link (n0, n1, 0);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
      try
        {
          g.add_link (n0, n2, 1);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
      try
        {
          g.add_link (0, 1, 0);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
    }

    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n1));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (std::distance (nodes.cbegin (), nodes.cend ()) == 0);
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n2));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n0));
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n2, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 0);
    }
    {
      std::unordered_set<const Node *> ns;
      auto nodes = g.neighbors (n2, Direction::Incoming);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.size () == 1);
      assert (ns.count (&n1));
    }

    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Direction::Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l0));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Direction::Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Direction::Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l1));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Direction::Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 1);
      assert (ls.count (&l0));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n2, Direction::Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (ls.size () == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n2, Direction::Incoming);
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
    auto &&l2 = g.add_link (0, 1, 2);

    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 0);
    }

    {
      assert (&n0 == &g.get_endpoint (l0, Direction::Incoming));
      assert (&n1 == &g.get_endpoint (l0, Direction::Outgoing));
      assert (&n0 == &g.get_endpoint (l1, Direction::Incoming));
      assert (&n1 == &g.get_endpoint (l1, Direction::Outgoing));
      assert (&n0 == &g.get_endpoint (l2, Direction::Incoming));
      assert (&n1 == &g.get_endpoint (l2, Direction::Outgoing));
    }

    {
      auto nodes = g.neighbors (n0, Direction::Incoming);
      auto it0 = nodes.cbegin ();
      auto it1 = it0++;
      assert (std::distance (it0, nodes.cend ()) == 0);
      assert (std::distance (it1, nodes.cend ()) == 0);
    }
    {
      auto nodes = g.neighbors (n0, Direction::Outgoing);
      auto it0 = nodes.cbegin ();
      auto it1 = it0++;
      assert (std::distance (it0, nodes.cend ()) == 0);
      assert (std::distance (it1, nodes.cend ()) == 1);
    }

    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Direction::Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 0);
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n0, Direction::Outgoing);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 3);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.count (&l2));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Direction::Incoming);
      for (const auto &l : links)
        ls.insert (&l);
      assert (std::distance (links.cbegin (), links.cend ()) == 3);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.count (&l2));
    }
    {
      std::unordered_set<const Link *> ls;
      auto links = g.connections (n1, Direction::Outgoing);
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
    auto &&l1 = g.add_link (1, 0, 1);

    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n0, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n1);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Incoming);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }
    {
      std::vector<const Node *> ns;
      auto nodes = g.neighbors (n1, Direction::Outgoing);
      for (const auto &n : nodes)
        ns.push_back (&n);
      assert (ns.size () == 1);
      assert (ns[0] == &n0);
    }

    {
      assert (&n0 == &g.get_endpoint (l0, Direction::Incoming));
      assert (&n1 == &g.get_endpoint (l0, Direction::Outgoing));
      assert (&n1 == &g.get_endpoint (l1, Direction::Incoming));
      assert (&n0 == &g.get_endpoint (l1, Direction::Outgoing));
    }

    {
      std::vector<const Link *> ls;
      auto links = g.connections (n0, Direction::Incoming);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l1);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n0, Direction::Outgoing);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l0);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n1, Direction::Incoming);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l0);
    }
    {
      std::vector<const Link *> ls;
      auto links = g.connections (n1, Direction::Outgoing);
      for (const auto &l : links)
        ls.push_back (&l);
      assert (ls.size () == 1);
      assert (ls[0] == &l1);
    }
  }

  // Test case 4: other types of data
  {
    // `std::string'
    {
      using DiGraph =
        typename macposts::graph::DiGraph<std::string, std::string>;

      DiGraph g;
      std::string d0 = "node 0";
      auto &&n0 = g.add_node (std::move (d0));
      auto &&n1 = g.add_node ("node 1");
      auto &&n2 = g.add_node ("node 2");
      auto &&l0 = g.add_link ("node 0", "node 1", "link 0");
      auto &&l1 = g.add_link (n1, n2, "link 1");

      assert (d0.empty ());
      assert (&n0 == &g.get_node ("node 0"));
      assert (&n1 == &g.get_node ("node 1"));
      assert (&n2 == &g.get_node ("node 2"));
      assert (&l0 == &g.get_link ("link 0"));
      assert (&l1 == &g.get_link ("link 1"));
    }

    // `std::unique_ptr' (non-copyable)
    {
      using DiGraph =
        typename macposts::graph::DiGraph<std::unique_ptr<int>, int>;

      DiGraph g;
      auto d0 = std::unique_ptr<int> (new int (0));
      g.add_node (std::move (d0));
      assert (g.size_nodes () == 1);
      assert (!d0.get ());
    }
  }

  return 0;
}

#endif
