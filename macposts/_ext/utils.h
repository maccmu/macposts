// Supporting utilities and common constants.
#pragma once

#include <algorithm>
#include <random>

namespace MNM_Ults
{
void set_random_state (unsigned int s);
}

namespace macposts
{
// The default random number generator.
static std::mt19937 rng;
// FIXME: Upper bound for travel time (?).
static const double TT_UPPER_BOUND = 20;

namespace utils
{
inline void
set_random_state (unsigned int s)
{
  MNM_Ults::set_random_state (s);
  rng.seed (s);
}

template <typename RandomIt>
inline void
random_shuffle (RandomIt first, RandomIt last)
{
  random_shuffle (first, last, rng);
}

template <typename RandomIt, typename RandomGen>
inline void
random_shuffle (RandomIt first, RandomIt last, RandomGen &&rng)
{
  std::shuffle (first, last, rng);
}
}
}
