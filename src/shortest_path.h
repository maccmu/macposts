#pragma once

#include "common.h"
#include "limits.h"
#include "path.h"
#include "ults.h"

#include <algorithm>
#include <deque>
#include <math.h>
#include <queue>
#include <unordered_map>
#include <vector>

class MNM_Link_Cost;
class MNM_Path;

namespace MNM_Shortest_Path
{
// with link cost
int all_to_one_Dijkstra (TInt dest_node_ID, const macposts::Graph &graph,
                         const std::unordered_map<TInt, TFlt> &cost_map,
                         std::unordered_map<TInt, TInt> &output_map);
int all_to_one_Dijkstra (TInt dest_node_ID, const macposts::Graph &graph,
                         std::unordered_map<TInt, TFlt> &dist_to_dest,
                         const std::unordered_map<TInt, TFlt> &cost_map,
                         std::unordered_map<TInt, TInt> &output_map);
// for last time step of TDSP
int all_to_one_Dijkstra (TInt dest_node_ID, const macposts::Graph &graph,
                         const std::unordered_map<TInt, TFlt *> &cost_map,
                         std::unordered_map<TInt, TFlt *> &dist_to_dest,
                         std::unordered_map<TInt, TInt *> &output_map,
                         TInt cost_position, TInt dist_position,
                         TInt output_position);
// with link cost
int all_to_one_FIFO (TInt dest_node_ID, const macposts::Graph &graph,
                     const std::unordered_map<TInt, TFlt> &cost_map,
                     std::unordered_map<TInt, TInt> &output_map);
// with link cost, for last time step of TDSP
int all_to_one_FIFO (TInt dest_node_ID, const macposts::Graph &graph,
                     const std::unordered_map<TInt, TFlt *> &cost_map,
                     std::unordered_map<TInt, TFlt *> &dist_to_dest,
                     std::unordered_map<TInt, TInt *> &output_map,
                     TInt cost_position, TInt dist_position,
                     TInt output_position);
// with link cost + node cost
int all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt> &link_cost_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt>> &node_cost_map,
  std::unordered_map<TInt, TInt> &output_map);
// with link cost + node cost, for last time step of TDSP
int all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt *> &link_cost_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
    &node_cost_map,
  std::unordered_map<TInt, TFlt *> &dist_to_dest,
  std::unordered_map<TInt, TInt *> &output_map, TInt cost_position,
  TInt dist_position, TInt output_position);

int all_to_one_LIFO (TInt dest_node_ID, const macposts::Graph &graph,
                     const std::unordered_map<TInt, TFlt> &cost_map,
                     std::unordered_map<TInt, TInt> &output_map);

/*------------------------------------------------------------
  TDSP
  -------------------------------------------------------------*/
bool is_FIFO (const macposts::Graph &graph,
              const std::unordered_map<TInt, TFlt *> &cost_map,
              TInt num_interval, TFlt unit_time);
};

/*------------------------------------------------------------
                  shortest path one destination tree
-------------------------------------------------------------*/
// class MNM_SP_Tree
// {
//   MNM_SP_Tree(TInt dest_node_ID, PNEGraph graph);
//   ~MNM_SP_Tree();

//   int initialize();
//   int update_tree(std::unordered_map<TInt, TFlt>& cost_map);
//   TFlt get_distance_to_destination(TInt node_ID);
//   int get_next_link_ID(TInt node_ID);

// };

/*------------------------------------------------------------
                  TDSP  one destination tree
-------------------------------------------------------------*/
class MNM_TDSP_Tree
{
public:
  MNM_TDSP_Tree (TInt dest_node_ID, macposts::Graph &graph, TInt max_interval);
  ~MNM_TDSP_Tree ();

  int initialize ();
  int update_tree (const std::unordered_map<TInt, TFlt *> &link_cost_map,
                   const std::unordered_map<TInt, TFlt *> &link_tt_map);
  int
  update_tree (const std::unordered_map<TInt, TFlt *> &link_cost_map,
               const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
                 &node_cost_map,
               const std::unordered_map<TInt, TFlt *> &link_tt_map,
               const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
                 &node_tt_map);
  TFlt get_distance_to_destination (TInt node_ID, TFlt time_stamp);
  TFlt get_distance_to_destination (TInt node_ID, int start_time_stamp,
                                    TFlt travel_time, float p = 1e-4);
  TFlt get_tdsp (TInt src_node_ID, TInt time,
                 const std::unordered_map<TInt, TFlt *> &link_tt_map,
                 MNM_Path *path);
  TFlt
  get_tdsp (TInt src_node_ID, TInt time,
            const std::unordered_map<TInt, TFlt *> &link_tt_map,
            const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
              &node_tt_map,
            MNM_Path *path);
  int round_time (TFlt time_stamp);
  int round_time (int start_time_stamp, TFlt travel_time, float p = 1e-4);
  std::unordered_map<TInt, TFlt *> m_dist;
  std::unordered_map<TInt, TInt *> m_tree;
  TInt m_dest_node_ID;
  macposts::Graph &m_graph;
  TInt m_max_interval;
};

/*------------------------------------------------------------
                          misc
-------------------------------------------------------------*/
class MNM_Cost
{
public:
  MNM_Cost () { ; };
  MNM_Cost (TInt ID, TFlt cost)
  {
    m_ID = ID;
    m_cost = cost;
  };
  TInt m_ID;
  TFlt m_cost;
  // bool operator <(const MNM_Cost& d){return m_cost >= d.m_cost;};
  // bool operator >(const MNM_Cost& d){return m_cost > d.m_cost;};
  // bool operator <=(const MNM_Cost& d){return m_cost <= d.m_cost;};
  // bool operator >=(const MNM_Cost& d){return m_cost >= d.m_cost;};
};

struct LessThanByCost
{
  bool operator() (const MNM_Cost *lhs, const MNM_Cost *rhs) const
  {
    return lhs->m_cost >= rhs->m_cost;
  }
};

bool CompareCostDecendSort (MNM_Cost *lhs, MNM_Cost *rhs);
