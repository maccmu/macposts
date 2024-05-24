#pragma once

#include "graph.h"
#include <cassert>

// Compatibility
using TInt = int;
using TFlt = double;
#define Assert(c) assert (c)
#define IAssert(c) assert (c)

namespace macposts
{
struct Empty
{
};

using Graph = graph::Graph<int, int, Empty, Empty, true>;
}
