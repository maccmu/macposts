#include "utils.h"
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

// namespace mmdta
// {
// void init (py::module &m);
// }
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

  macposts::tdsp::init (m);
  macposts::dta::init (m);
  macposts::mcdta::init (m);
  // macposts::mmdta::init (m);
}
