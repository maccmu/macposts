// Supporting utilities and common constants.
#pragma once

namespace MNM_Ults
{
void set_random_state (unsigned int s);
}

namespace macposts
{
// FIXME: Upper bound for travel time (?).
static const double TT_UPPER_BOUND = 20;

namespace utils
{
inline void
set_random_state (unsigned int s)
{
  MNM_Ults::set_random_state (s);
}
}
}
