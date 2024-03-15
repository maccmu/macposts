#ifdef LIBMACPOSTS_INTERNAL_TEST

// clang-format off
#include "graph.h"
#include <vector>

#define assert(c) if (!(c)) __builtin_trap ()
// clang-format on

// TODO: Turn those into real unit test cases.

int
main (void)
{
  using macposts::graph::Direction;
  struct Empty
  {
  };

  // Direction graph
  {
    // All the basics
    {
      using DiGraph = macposts::graph::DiGraph<Empty, Empty>;
      using Node = typename DiGraph::Node;
      using Link = typename DiGraph::Link;

      DiGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&n2 = g.add_node (2);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (1, 2, 1);

      assert (g.size_nodes () == 3);
      assert (g.size_links () == 2);
      assert (&g.get_node (0) == &n0);
      assert (&g.get_node (1) == &n1);
      assert (&g.get_node (2) == &n2);
      assert (&g.get_link (0) == &l0);
      assert (&g.get_link (1) == &l1);
      assert (g.get_id (n0) == 0);
      assert (g.get_id (n1) == 1);
      assert (g.get_id (n2) == 2);
      assert (g.get_id (l0) == 0);
      assert (g.get_id (l1) == 1);

      auto &&endpoints = g.get_endpoints (l0);
      assert (&endpoints.first == &n0);
      assert (&endpoints.second == &n1);
      assert (&g.get_endpoints (l1).first == &n1);
      assert (&g.get_endpoints (l1).second == &n2);
      assert (&endpoints.first == &g.get_endpoints (0).first);
      assert (&endpoints.second == &g.get_endpoints (0).second);

      std::unordered_set<const Node *> ns;
      auto &&nodes = g.nodes ();
      assert (std::distance (nodes.begin (), nodes.end ()) == 3);
      assert (std::distance (nodes.cbegin (), nodes.cend ()) == 3);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.count (&n0));
      assert (ns.count (&n1));
      assert (ns.count (&n2));
      assert (ns.size () == 3);

      ns.clear ();
      for (const auto &n : g.neighbors (n0, Direction::Outgoing))
        ns.insert (&n);
      assert (ns.count (&n1));
      assert (ns.size () == 1);
      ns.clear ();
      for (const auto &n : g.neighbors (n0, Direction::Incoming))
        ns.insert (&n);
      assert (ns.size () == 0);

      ns.clear ();
      for (const auto &n : g.neighbors (0, Direction::Outgoing))
        ns.insert (&n);
      assert (ns.count (&n1));
      assert (ns.size () == 1);
      ns.clear ();
      for (const auto &n : g.neighbors (0, Direction::Incoming))
        ns.insert (&n);
      assert (ns.size () == 0);

      std::unordered_set<const Link *> ls;
      auto &&links = g.links ();
      assert (std::distance (links.begin (), links.end ()) == 2);
      assert (std::distance (links.cbegin (), links.cend ()) == 2);
      for (const auto &l : g.links ())
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.size () == 2);

      ls.clear ();
      for (const auto &l : g.connections (n1, Direction::Outgoing))
        ls.insert (&l);
      assert (ls.count (&l1));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (n1, Direction::Incoming))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (n0, Direction::Outgoing))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (n0, Direction::Incoming))
        ls.insert (&l);
      assert (ls.size () == 0);

      ls.clear ();
      for (const auto &l : g.connections (1, Direction::Outgoing))
        ls.insert (&l);
      assert (ls.count (&l1));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (1, Direction::Incoming))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (0, Direction::Outgoing))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.size () == 1);
      ls.clear ();
      for (const auto &l : g.connections (0, Direction::Incoming))
        ls.insert (&l);
      assert (ls.size () == 0);

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

      try
        {
          g.add_link (0, 0, 100);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
      try
        {
          g.add_link (n0, n0, 100);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }

      {
        const auto &g_ = g;
        auto &&nodes = g_.nodes ();
        assert (std::distance (nodes.begin (), nodes.end ()) == 3);
        auto &&links = g_.links ();
        assert (std::distance (links.begin (), links.end ()) == 2);
        auto &&neighbors = g_.neighbors (n0, Direction::Outgoing);
        assert (std::distance (neighbors.begin (), neighbors.end ()) == 1);
        auto &&connections = g_.connections (n0, Direction::Outgoing);
        assert (std::distance (connections.begin (), connections.end ()) == 1);
      }
    }

    // Changing data
    {
      auto g = macposts::graph::DiGraph<std::string, std::string> ();
      auto &&n0 = g.add_node (0, "one");
      auto &&n1 = g.add_node (1, "two");
      auto &&l0 = g.add_link (0, 1, 0);

      for (auto &&n : g.nodes ())
        n.data = "node";
      assert (n0.data == "node");
      assert (n1.data == "node");
      for (auto &&n : g.neighbors (n0, Direction::Outgoing))
        n.data = "vertex";
      assert (n1.data == "vertex");
      for (auto &&n : g.neighbors (n1, Direction::Incoming))
        n.data.push_back ('0');
      assert (n0.data == "node0");

      for (auto &&l : g.links ())
        l.data = "link";
      assert (l0.data == "link");
      for (auto &&l : g.connections (n0, Direction::Outgoing))
        l.data = "edge";
      assert (l0.data == "edge");
      for (auto &&l : g.connections (n1, Direction::Incoming))
        l.data.push_back ('0');
      assert (l0.data == "edge0");
    }

    // Parallel links
    {
      using DiGraph = macposts::graph::DiGraph<Empty, Empty>;
      using Node = typename DiGraph::Node;
      using Link = typename DiGraph::Link;

      DiGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (n0, n1, 1);
      auto &&l2 = g.add_link (0, 1, 2);

      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0, Direction::Incoming);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 0);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0, Direction::Outgoing);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n1);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1, Direction::Incoming);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n0);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1, Direction::Outgoing);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 0);
      }

      {
        assert (&n0 == &g.get_endpoints (l0).first);
        assert (&n1 == &g.get_endpoints (l0).second);
        assert (&n0 == &g.get_endpoints (l1).first);
        assert (&n1 == &g.get_endpoints (l1).second);
        assert (&n0 == &g.get_endpoints (l2).first);
        assert (&n1 == &g.get_endpoints (l2).second);
      }

      {
        auto &&nodes = g.neighbors (n0, Direction::Incoming);
        auto it0 = nodes.cbegin ();
        auto it1 = it0++;
        assert (std::distance (it0, nodes.cend ()) == 0);
        assert (std::distance (it1, nodes.cend ()) == 0);
      }
      {
        auto &&nodes = g.neighbors (n0, Direction::Outgoing);
        auto it0 = nodes.cbegin ();
        auto it1 = it0++;
        assert (std::distance (it0, nodes.cend ()) == 0);
        assert (std::distance (it1, nodes.cend ()) == 1);
      }

      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n0, Direction::Incoming);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 0);
      }
      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n0, Direction::Outgoing);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 3);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
        assert (ls.count (&l2));
      }
      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n1, Direction::Incoming);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 3);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
        assert (ls.count (&l2));
      }
      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n1, Direction::Outgoing);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 0);
      }
    }

    // Cycles
    {
      using DiGraph = macposts::graph::DiGraph<Empty, Empty>;
      using Node = typename DiGraph::Node;
      using Link = typename DiGraph::Link;

      DiGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (1, 0, 1);

      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0, Direction::Incoming);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n1);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0, Direction::Outgoing);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n1);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1, Direction::Incoming);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n0);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1, Direction::Outgoing);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n0);
      }

      {
        assert (&n0 == &g.get_endpoints (l0).first);
        assert (&n1 == &g.get_endpoints (l0).second);
        assert (&n1 == &g.get_endpoints (l1).first);
        assert (&n0 == &g.get_endpoints (l1).second);
      }

      {
        std::vector<const Link *> ls;
        auto &&links = g.connections (n0, Direction::Incoming);
        for (const auto &l : links)
          ls.push_back (&l);
        assert (ls.size () == 1);
        assert (ls[0] == &l1);
      }
      {
        std::vector<const Link *> ls;
        auto &&links = g.connections (n0, Direction::Outgoing);
        for (const auto &l : links)
          ls.push_back (&l);
        assert (ls.size () == 1);
        assert (ls[0] == &l0);
      }
      {
        std::vector<const Link *> ls;
        auto &&links = g.connections (n1, Direction::Incoming);
        for (const auto &l : links)
          ls.push_back (&l);
        assert (ls.size () == 1);
        assert (ls[0] == &l0);
      }
      {
        std::vector<const Link *> ls;
        auto &&links = g.connections (n1, Direction::Outgoing);
        for (const auto &l : links)
          ls.push_back (&l);
        assert (ls.size () == 1);
        assert (ls[0] == &l1);
      }
    }

    // Other features
    {
      // No copy
      {
        using DiGraph
          = macposts::graph::DiGraph<std::unique_ptr<int>, std::unique_ptr<int>>;
        using Node = typename DiGraph::Node;
        using Link = typename DiGraph::Link;

        DiGraph g;
        auto d0 = std::unique_ptr<int> (new int (0));
        auto d1 = std::unique_ptr<int> (new int (1));
        auto d2 = std::unique_ptr<int> (new int (2));
        g.add_node (0, std::move (d0));
        g.add_node (1, std::move (d1));
        g.add_link (0, 1, 0, std::move (d2));

        assert (g.size_nodes () == 2);
        assert (g.size_links () == 1);

        auto &&n0 = g.get_node (0);
        std::vector<const Node *> ns;
        for (const auto &n : g.nodes ())
          ns.push_back (&n);
        assert (ns.size () == 2);
        for (const auto &n : g.neighbors (n0, Direction::Outgoing))
          ns.push_back (&n);
        assert (ns.size () == 3);
        std::vector<const Link *> ls;
        for (const auto &l : g.links ())
          ls.push_back (&l);
        assert (ls.size () == 1);

        for (const auto &l : g.connections (n0, Direction::Outgoing))
          ls.push_back (&l);
        assert (ls.size () == 2);
        assert (&g.get_endpoints (g.get_link (0)).first == &n0);
      }

      // Modifying data retains index
      {
        auto g = macposts::graph::DiGraph<int, int> ();
        auto &&n0 = g.add_node (0);
        n0.data = 10;
        assert (&n0 == &g.get_node (0));
      }
    }
  }

  // Undirected graph
  {
    // All the basics
    {
      using UnGraph = macposts::graph::UnGraph<Empty, Empty>;
      using Node = UnGraph::Node;
      using Link = UnGraph::Link;

      UnGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&n2 = g.add_node (2);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (1, 2, 1);

      assert (g.size_nodes () == 3);
      assert (g.size_links () == 2);
      assert (&g.get_node (0) == &n0);
      assert (&g.get_node (1) == &n1);
      assert (&g.get_node (2) == &n2);
      assert (&g.get_link (0) == &l0);
      assert (&g.get_link (1) == &l1);
      assert (g.get_id (n0) == 0);
      assert (g.get_id (n1) == 1);
      assert (g.get_id (n2) == 2);
      assert (g.get_id (l0) == 0);
      assert (g.get_id (l1) == 1);

      auto &&endpoints = g.get_endpoints (l0);
      assert (&endpoints.first == &n0);
      assert (&endpoints.second == &n1);
      assert (&g.get_endpoints (l1).first == &n1);
      assert (&g.get_endpoints (l1).second == &n2);

      std::unordered_set<const Node *> ns;
      auto &&nodes = g.nodes ();
      assert (std::distance (nodes.begin (), nodes.end ()) == 3);
      assert (std::distance (nodes.cbegin (), nodes.cend ()) == 3);
      for (const auto &n : nodes)
        ns.insert (&n);
      assert (ns.count (&n0));
      assert (ns.count (&n1));
      assert (ns.count (&n2));
      assert (ns.size () == 3);

      ns.clear ();
      for (const auto &n : g.neighbors (n0))
        ns.insert (&n);
      assert (ns.count (&n1));
      assert (ns.size () == 1);
      ns.clear ();
      for (const auto &n : g.neighbors (n1))
        ns.insert (&n);
      assert (ns.count (&n0));
      assert (ns.count (&n2));
      assert (ns.size () == 2);

      std::unordered_set<const Link *> ls;
      auto &&links = g.links ();
      assert (std::distance (links.begin (), links.end ()) == 2);
      assert (std::distance (links.cbegin (), links.cend ()) == 2);
      for (const auto &l : g.links ())
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.size () == 2);

      ls.clear ();
      for (const auto &l : g.connections (n0))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.size () == 1);

      ls.clear ();
      for (const auto &l : g.connections (n1))
        ls.insert (&l);
      assert (ls.count (&l0));
      assert (ls.count (&l1));
      assert (ls.size () == 2);

      ls.clear ();
      for (const auto &l : g.connections (n2))
        ls.insert (&l);
      assert (ls.count (&l1));
      assert (ls.size () == 1);

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

      try
        {
          g.add_link (0, 0, 100);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }
      try
        {
          g.add_link (n0, n0, 100);
          assert (false);
        }
      catch (const std::runtime_error &)
        {
        }

      {
        const auto &g_ = g;
        auto &&nodes = g_.nodes ();
        assert (std::distance (nodes.begin (), nodes.end ()) == 3);
        auto &&links = g_.links ();
        assert (std::distance (links.begin (), links.end ()) == 2);
        auto &&neighbors = g_.neighbors (n0);
        assert (std::distance (neighbors.begin (), neighbors.end ()) == 1);
        auto &&connections = g_.connections (n0);
        assert (std::distance (connections.begin (), connections.end ()) == 1);
      }
    }

    // Changing data
    {
      auto g = macposts::graph::UnGraph<std::string, std::string> ();
      auto &&n0 = g.add_node (0, "one");
      auto &&n1 = g.add_node (1, "two");
      auto &&l0 = g.add_link (0, 1, 0);

      for (auto &&n : g.nodes ())
        n.data = "node";
      assert (n0.data == "node");
      assert (n1.data == "node");
      for (auto &&n : g.neighbors (n0))
        n.data = "vertex";
      assert (n1.data == "vertex");
      for (auto &&n : g.neighbors (n1))
        n.data.push_back ('0');
      assert (n0.data == "node0");

      for (auto &&l : g.links ())
        l.data = "link";
      assert (l0.data == "link");
      for (auto &&l : g.connections (n0))
        l.data = "edge";
      assert (l0.data == "edge");
      for (auto &&l : g.connections (n1))
        l.data.push_back ('0');
      assert (l0.data == "edge0");
    }

    // Parallel links
    {
      using UnGraph = macposts::graph::UnGraph<Empty, Empty>;
      using Node = typename UnGraph::Node;
      using Link = typename UnGraph::Link;

      UnGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (n0, n1, 1);
      auto &&l2 = g.add_link (0, 1, 2);

      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n1);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n0);
      }

      {
        assert (&n0 == &g.get_endpoints (l0).first);
        assert (&n1 == &g.get_endpoints (l0).second);
        assert (&n0 == &g.get_endpoints (l1).first);
        assert (&n1 == &g.get_endpoints (l1).second);
        assert (&n0 == &g.get_endpoints (l2).first);
        assert (&n1 == &g.get_endpoints (l2).second);
      }

      {
        auto &&nodes = g.neighbors (n0);
        auto it0 = nodes.cbegin ();
        auto it1 = it0++;
        assert (std::distance (it0, nodes.cend ()) == 0);
        assert (std::distance (it1, nodes.cend ()) == 1);
      }

      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n0);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 3);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
        assert (ls.count (&l2));
      }
      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n1);
        for (const auto &l : links)
          ls.insert (&l);
        assert (std::distance (links.cbegin (), links.cend ()) == 3);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
        assert (ls.count (&l2));
      }
    }

    // Cycles
    {
      using UnGraph = macposts::graph::UnGraph<Empty, Empty>;
      using Node = typename UnGraph::Node;
      using Link = typename UnGraph::Link;

      UnGraph g;
      auto &&n0 = g.add_node (0);
      auto &&n1 = g.add_node (1);
      auto &&l0 = g.add_link (n0, n1, 0);
      auto &&l1 = g.add_link (1, 0, 1);

      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n0);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n1);
      }
      {
        std::vector<const Node *> ns;
        auto &&nodes = g.neighbors (n1);
        for (const auto &n : nodes)
          ns.push_back (&n);
        assert (ns.size () == 1);
        assert (ns[0] == &n0);
      }

      {
        assert (&n0 == &g.get_endpoints (l0).first);
        assert (&n1 == &g.get_endpoints (l0).second);
        assert (&n1 == &g.get_endpoints (l1).first);
        assert (&n0 == &g.get_endpoints (l1).second);
      }

      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n0);
        for (const auto &l : links)
          ls.insert (&l);
        assert (ls.size () == 2);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
      }
      {
        std::unordered_set<const Link *> ls;
        auto &&links = g.connections (n1);
        for (const auto &l : links)
          ls.insert (&l);
        assert (ls.size () == 2);
        assert (ls.count (&l0));
        assert (ls.count (&l1));
      }
    }

    // Other features
    {
      // No copy
      {
        using UnGraph = typename macposts::graph::UnGraph<std::unique_ptr<int>,
                                                          std::unique_ptr<int>>;
        using Node = typename UnGraph::Node;
        using Link = typename UnGraph::Link;

        UnGraph g;
        auto d0 = std::unique_ptr<int> (new int (0));
        auto d1 = std::unique_ptr<int> (new int (1));
        auto d2 = std::unique_ptr<int> (new int (2));
        g.add_node (0, std::move (d0));
        g.add_node (1, std::move (d1));
        g.add_link (0, 1, 0, std::move (d2));

        assert (g.size_nodes () == 2);
        assert (g.size_links () == 1);

        auto &&n0 = g.get_node (0);
        std::vector<const Node *> ns;
        for (const auto &n : g.nodes ())
          ns.push_back (&n);
        assert (ns.size () == 2);
        for (const auto &n : g.neighbors (n0))
          ns.push_back (&n);
        assert (ns.size () == 3);
        std::vector<const Link *> ls;
        for (const auto &l : g.links ())
          ls.push_back (&l);
        assert (ls.size () == 1);

        for (const auto &l : g.connections (n0))
          ls.push_back (&l);
        assert (ls.size () == 2);
        assert (&g.get_endpoints (g.get_link (0)).first == &n0);
      }

      // Modifying data retains index
      {
        auto g = macposts::graph::UnGraph<int, int> ();
        auto &&n0 = g.add_node (0);
        n0.data = 10;
        assert (&n0 == &g.get_node (0));
      }
    }
  }

  return 0;
}

#endif
