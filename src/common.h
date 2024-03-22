#pragma once

#include "graph.h"
#include <Snap.h>

namespace macposts
{
struct Empty
{
};
using Graph = graph::Graph<TInt, TInt, Empty, Empty, true>;
}
