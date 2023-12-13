#pragma once

#include <Snap.h>
#include "graph.h"

namespace macposts
{
  struct Empty {};
  using Graph = graph::Graph<TInt, TInt, Empty, Empty, true>;
}
