#include "utils.h"
#include <common.h>
#include <pybind11/pybind11.h>
#include <random>

namespace py = pybind11;

namespace macposts
{
namespace tdsp
{
void init (py::module &m);
}

namespace dta
{
void init (py::module &m);
}

namespace mcdta
{
void init (py::module &m);
}

namespace mmdta
{
void init (py::module &m);
}
}

PYBIND11_MODULE (_macposts_ext, m)
{
  // Initialize random number generator.
  std::random_device rd;
  macposts::utils::set_random_state (rd ());

  m.def ("set_random_state", &macposts::utils::set_random_state,
         R"pbdoc(Set the random state.

Note that this only affects the random number generator used during network
simulation.

)pbdoc");

  // Graph class
  //
  // NOTE: For graphs, our memory model is simply assuming that the graph always
  // exists (see the beginning of 'src/graph.h'). That does not play well with
  // the interface of pybind11. So for the Python binding, we instead use
  // indices extensively. Also, for collections of nodes/links, we return a
  // Python list of indices, instead of our C++ container/wrapper types.
  {
    using Dir = macposts::graph::Direction;
    py::enum_<Dir> (m, "Direction")
      .value ("Incoming", Dir::Incoming)
      .value ("Outgoing", Dir::Outgoing);

    using G = macposts::Graph;
    py::class_<G> Graph (m, "Graph");
    py::class_<G::Node> Node (Graph, "Node");
    py::class_<G::Link> Link (Graph, "Link");

    Graph.def (py::init<> ())
      .def (
        "add_node", [] (G &g, int id) { g.add_node (id); },
        "Add a node of *id*.", py::arg ("id"))
      .def ("get_node", static_cast<G::Node &(G::*) (int)> (&G::get_node),
            py::return_value_policy::reference_internal, "Get node of *id*.",
            py::arg ("id"))
      .def (
        "add_link",
        [] (G &g, int from, int to, int id) { g.add_link (from, to, id); },
        "Add a link of *id* that connects nodes *from* and *to*.",
        py::arg ("from"), py::arg ("to"), py::arg ("id"))
      .def ("get_link", static_cast<G::Link &(G::*) (int)> (&G::get_link),
            py::return_value_policy::reference_internal, "Get link of *id*.",
            py::arg ("id"))
      .def ("get_id",
            static_cast<int (G::*) (const G::Node &) const> (&G::get_id),
            "Get ID of *node*.", py::arg ("node"))
      .def ("get_id",
            static_cast<int (G::*) (const G::Link &) const> (&G::get_id),
            "Get ID of *link*.", py::arg ("link"))
      .def_property_readonly ("size_nodes", &G::size_nodes,
                              "Number of nodes added to graph.")
      .def_property_readonly ("size_links", &G::size_links,
                              "Number of links added to graph.")
      .def (
        "nodes",
        [] (const G &g) {
          py::list ns;
          for (const auto &n : g.nodes ())
            ns.append (g.get_id (n));
          return ns;
        },
        "Get IDs of all nodes.")
      .def (
        "links",
        [] (const G &g) {
          py::list ls;
          for (const auto &l : g.links ())
            ls.append (g.get_id (l));
          return ls;
        },
        "Get IDs of all links.")
      .def (
        "neighbors",
        [] (const G &g, int id, Dir direction) {
          py::list ns;
          for (const auto &n : g.neighbors (id, direction))
            ns.append (g.get_id (n));
          return ns;
        },
        "Get IDs of neighboring nodes of node *id* in *direction*.",
        py::arg ("id"), py::arg ("direction"))
      .def (
        "connections",
        [] (const G &g, int id, Dir direction) {
          py::list ls;
          for (const auto &l : g.connections (id, direction))
            ls.append (g.get_id (l));
          return ls;
        },
        "Get IDs of links connecting node *id* in *direction*.", py::arg ("id"),
        py::arg ("direction"))
      .def (
        "get_endpoints",
        [] (const G &g, int id) {
          auto &&endpoints = g.get_endpoints (id);
          return py::make_tuple (g.get_id (endpoints.first),
                                 g.get_id (endpoints.second));
        },
        "Get IDs of the two endpoints (nodes) of link *id*.", py::arg ("id"));
  }

  macposts::tdsp::init (m);
  macposts::dta::init (m);
  macposts::mcdta::init (m);
  macposts::mmdta::init (m);
}
