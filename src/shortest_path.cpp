#include "shortest_path.h"

using macposts::graph::Direction;

static_assert (std::numeric_limits<double>::is_iec559,
               "No iec559 infinity implementation for this compiler!\n");

int
MNM_Shortest_Path::all_to_one_Dijkstra (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt> &cost_map,
  std::unordered_map<TInt, TInt> &output_map)
{
  std::unordered_map<TInt, TFlt> dist_to_dest
    = std::unordered_map<TInt, TFlt> ();
  return all_to_one_Dijkstra (dest_node_ID, graph, dist_to_dest, cost_map,
                              output_map);
}

int
MNM_Shortest_Path::all_to_one_Dijkstra (
  TInt dest_node_ID, const macposts::Graph &graph,
  std::unordered_map<TInt, TFlt> &dist_to_dest,
  const std::unordered_map<TInt, TFlt> &cost_map,
  std::unordered_map<TInt, TInt> &output_map)
{
  std::priority_queue<MNM_Cost *, std::vector<MNM_Cost *>, LessThanByCost> m_Q
    = std::priority_queue<MNM_Cost *, std::vector<MNM_Cost *>,
                          LessThanByCost> ();
  MNM_Cost *dest_cost = new MNM_Cost (dest_node_ID, TFlt (0));
  m_Q.push (dest_cost);

  TInt _node_id;
  // std::unordered_map<TInt, TFlt> dist_to_dest = std::unordered_map<TInt,
  // TFlt>();
  dist_to_dest.clear ();
  for (const auto &node : graph.nodes ())
    {
      _node_id = graph.get_id (node);
      if (_node_id != dest_node_ID)
        {
          dist_to_dest.insert (
            { _node_id, TFlt (std::numeric_limits<double>::infinity ()) });
          output_map.insert (
            { _node_id, -1 }); // If the destination is not accessible the
                               // output remains -1
        }
    }
  dist_to_dest[dest_node_ID] = TFlt (0);

  // Initialization above. Dijkstra with binary min-heap (std::priority_queue)
  // below:

  // NOTE: Since C++ std::priority_queue does not have decrease_key() function,
  // we insert [pointer to new MNM_cost object] to the min-heap every time when
  // the dist_to_dest[] changes for some node. So there could be duplicated
  // elements in the min-heap for the same nodes with different distance values.
  // But the duplication doesn't affect the correctness of algorithm. (visited
  // label for eliminating the duplication is also tested, but slower than not
  // using it, kind of weird.)

  MNM_Cost *_min_cost;
  TInt _in_node_id, _in_link_id;
  TFlt _tmp_dist, _alt;
  while (!m_Q.empty ())
    {
      _min_cost = m_Q.top ();
      m_Q.pop ();
      _node_id = _min_cost->m_ID;
      const auto &node = graph.get_node (_node_id);
      _tmp_dist = dist_to_dest[_node_id];
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          _in_link_id = graph.get_id (link);
          const auto &in_node = graph.get_endpoints (link).first;
          _in_node_id = graph.get_id (in_node);
          _alt = _tmp_dist + cost_map.find (_in_link_id)->second;
          if (_alt < dist_to_dest[_in_node_id])
            {
              dist_to_dest[_in_node_id] = _alt;
              m_Q.push (new MNM_Cost (_in_node_id, _alt));
              output_map[_in_node_id] = _in_link_id;
            }
        }
      delete _min_cost;
    }

  return 0;
}

int
MNM_Shortest_Path::all_to_one_Dijkstra (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt *> &cost_map,
  std::unordered_map<TInt, TFlt *> &dist_to_dest,
  std::unordered_map<TInt, TInt *> &output_map, TInt cost_position,
  TInt dist_position, TInt output_position)
{
  std::priority_queue<MNM_Cost *, std::vector<MNM_Cost *>, LessThanByCost> m_Q
    = std::priority_queue<MNM_Cost *, std::vector<MNM_Cost *>,
                          LessThanByCost> ();
  MNM_Cost *dest_cost = new MNM_Cost (dest_node_ID, TFlt (0));
  m_Q.push (dest_cost);

  TInt _node_id;
  for (const auto &node : graph.nodes ())
    {
      _node_id = graph.get_id (node);
      if (_node_id != dest_node_ID)
        {
          dist_to_dest[_node_id][dist_position]
            = TFlt (std::numeric_limits<double>::infinity ());
          output_map[_node_id][output_position]
            = -1; // If the destination is not accessible the output remains -1
        }
    }
  dist_to_dest[dest_node_ID][dist_position] = TFlt (0);

  // Initialization above. Dijkstra with binary min-heap (std::priority_queue)
  // below:
  MNM_Cost *_min_cost;
  TInt _in_node_id, _in_link_id;
  TFlt _tmp_dist, _alt;
  while (m_Q.size () != 0)
    {
      _min_cost = m_Q.top ();
      m_Q.pop ();
      _node_id = _min_cost->m_ID;
      const auto &node = graph.get_node (_node_id);
      _tmp_dist = dist_to_dest[_node_id][dist_position];
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          const auto &in_node = graph.get_endpoints (link).first;
          _in_node_id = graph.get_id (in_node);
          _in_link_id = graph.get_id (link);
          _alt = _tmp_dist + cost_map.find (_in_link_id)->second[cost_position];
          if (_alt < dist_to_dest[_in_node_id][dist_position])
            {
              dist_to_dest[_in_node_id][dist_position] = _alt;
              m_Q.push (new MNM_Cost (_in_node_id, _alt));
              output_map[_in_node_id][output_position] = _in_link_id;
            }
        }
      delete _min_cost;
    }

  return 0;
}

int
MNM_Shortest_Path::all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt> &cost_map,
  std::unordered_map<TInt, TInt> &output_map)
{
  std::unordered_map<TInt, TFlt> _dist = std::unordered_map<TInt, TFlt> ();
  _dist.insert (std::pair<TInt, TFlt> (dest_node_ID, TFlt (0)));

  std::deque<TInt> m_Q = std::deque<TInt> ();
  std::unordered_map<TInt, bool> m_Q_support
    = std::unordered_map<TInt, bool> ();

  m_Q.push_back (dest_node_ID);
  m_Q_support.insert (std::pair<TInt, bool> (dest_node_ID, true));

  TInt _node_ID;
  for (const auto &node : graph.nodes ())
    {
      _node_ID = graph.get_id (node);
      if (_node_ID != dest_node_ID)
        {
          _dist.insert (
            std::pair<TInt,
                      TFlt> (_node_ID,
                             TFlt (std::numeric_limits<double>::infinity ())));
          if (output_map.find (_node_ID) != output_map.end ())
            {
              output_map.find (_node_ID)->second = -1;
            }
          else
            {
              output_map.insert (std::pair<TInt, TInt> (_node_ID, -1));
            }
          m_Q_support.insert (std::pair<TInt, bool> (_node_ID, false));
        }
    }
  TInt _in_node_ID, _tmp_ID, _in_link_ID;
  TFlt _alt, _tmp_dist;
  while (m_Q.size () != 0)
    {
      // printf("current m_Q size is %d\n", m_Q.size());
      _tmp_ID = m_Q.front ();
      m_Q.pop_front ();
      m_Q_support.find (_tmp_ID)->second = false;
      const auto &node = graph.get_node (_tmp_ID);
      _tmp_dist = _dist.find (_tmp_ID)->second;
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          const auto &in_node = graph.get_endpoints (link).first;
          _in_node_ID = graph.get_id (in_node);
          _in_link_ID = graph.get_id (link);
          _alt = _tmp_dist + cost_map.find (_in_link_ID)->second;
          // printf("Current alternative distance is %.4f\n", _alt());
          if (_alt < _dist.find (_in_node_ID)->second)
            {
              // m_Q.push_back(m_Q_support.find(_in_node_ID) -> second);
              _dist.find (_in_node_ID)->second = _alt;
              output_map.find (_in_node_ID)->second = _in_link_ID;
              if (!m_Q_support.find (_in_node_ID)->second)
                {
                  m_Q.push_back (_in_node_ID);
                  m_Q_support.find (_in_node_ID)->second = true;
                }
            }
        }
    }

  m_Q.clear ();
  m_Q_support.clear ();
  _dist.clear ();
  return 0;
}

int
MNM_Shortest_Path::all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt *> &cost_map,
  std::unordered_map<TInt, TFlt *> &dist_to_dest,
  std::unordered_map<TInt, TInt *> &output_map, TInt cost_position,
  TInt dist_position, TInt output_position)
{
  std::deque<TInt> m_Q = std::deque<TInt> ();
  std::unordered_map<TInt, bool> m_Q_support
    = std::unordered_map<TInt, bool> ();

  m_Q.push_back (dest_node_ID);
  m_Q_support.insert (std::pair<TInt, bool> (dest_node_ID, true));

  TInt _node_ID;
  for (const auto &node : graph.nodes ())
    {
      _node_ID = graph.get_id (node);
      if (_node_ID != dest_node_ID)
        {
          dist_to_dest[_node_ID][dist_position]
            = TFlt (std::numeric_limits<double>::infinity ());
          output_map[_node_ID][output_position]
            = -1; // If the destination is not accessible the output remains -1
          m_Q_support.insert (std::pair<TInt, bool> (_node_ID, false));
        }
    }
  dist_to_dest[dest_node_ID][dist_position] = TFlt (0);

  TInt _in_node_ID, _tmp_ID, _in_link_ID;
  TFlt _alt, _tmp_dist;
  while (m_Q.size () != 0)
    {
      _tmp_ID = m_Q.front ();
      m_Q.pop_front ();
      m_Q_support.find (_tmp_ID)->second = false;
      const auto &node = graph.get_node (_tmp_ID);
      _tmp_dist = dist_to_dest[_tmp_ID][dist_position];
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          const auto &in_node = graph.get_endpoints (link).first;
          _in_node_ID = graph.get_id (in_node);
          _in_link_ID = graph.get_id (link);
          _alt = _tmp_dist + cost_map.find (_in_link_ID)->second[cost_position];
          if (_alt < dist_to_dest[_in_node_ID][dist_position])
            {
              dist_to_dest[_in_node_ID][dist_position] = _alt;
              output_map[_in_node_ID][output_position] = _in_link_ID;
              if (!m_Q_support.find (_in_node_ID)->second)
                {
                  m_Q.push_back (_in_node_ID);
                  m_Q_support.find (_in_node_ID)->second = true;
                }
            }
        }
    }

  m_Q.clear ();
  m_Q_support.clear ();
  return 0;
}

int
MNM_Shortest_Path::all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt> &link_cost_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt>> &node_cost_map,
  std::unordered_map<TInt, TInt> &output_map)
{
  std::unordered_map<TInt, TFlt> _dist = std::unordered_map<TInt, TFlt> ();
  _dist.insert (std::pair<TInt, TFlt> (dest_node_ID, TFlt (0)));

  std::deque<TInt> m_Q = std::deque<TInt> ();
  std::unordered_map<TInt, bool> m_Q_support
    = std::unordered_map<TInt, bool> ();

  m_Q.push_back (dest_node_ID);
  m_Q_support.insert (std::pair<TInt, bool> (dest_node_ID, true));

  TInt _node_ID;
  for (const auto &node : graph.nodes ())
    {
      _node_ID = graph.get_id (node);
      if (_node_ID != dest_node_ID)
        {
          _dist.insert (
            std::pair<TInt,
                      TFlt> (_node_ID,
                             TFlt (std::numeric_limits<double>::infinity ())));
          if (output_map.find (_node_ID) != output_map.end ())
            {
              output_map.find (_node_ID)->second = -1;
            }
          else
            {
              output_map.insert (std::pair<TInt, TInt> (_node_ID, -1));
            }
          m_Q_support.insert (std::pair<TInt, bool> (_node_ID, false));
        }
    }
  TInt _in_node_ID, _tmp_ID, _in_link_ID, _out_link_ID;
  TFlt _alt, _tmp_dist;
  while (m_Q.size () != 0)
    {
      _tmp_ID = m_Q.front ();
      m_Q.pop_front ();
      m_Q_support.find (_tmp_ID)->second = false;
      const auto &node = graph.get_node (_tmp_ID);
      _tmp_dist = _dist.find (_tmp_ID)->second;
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          const auto &in_node = graph.get_endpoints (link).first;
          _in_node_ID = graph.get_id (in_node);
          _in_link_ID = graph.get_id (link);
          _alt = _tmp_dist + link_cost_map.find (_in_link_ID)->second;
          if (_tmp_ID != dest_node_ID)
            {
              _out_link_ID = output_map.find (_tmp_ID)->second;
              if (_out_link_ID != -1
                  && node_cost_map.find (_in_link_ID) != node_cost_map.end ()
                  && node_cost_map.find (_in_link_ID)
                         ->second.find (_out_link_ID)
                       != node_cost_map.find (_in_link_ID)->second.end ())
                {
                  _alt += node_cost_map.find (_in_link_ID)
                            ->second.find (_out_link_ID)
                            ->second;
                }
            }
          if (_alt < _dist.find (_in_node_ID)->second)
            {
              _dist.find (_in_node_ID)->second = _alt;
              output_map.find (_in_node_ID)->second = _in_link_ID;
              if (!m_Q_support.find (_in_node_ID)->second)
                {
                  m_Q.push_back (_in_node_ID);
                  m_Q_support.find (_in_node_ID)->second = true;
                }
            }
        }
    }

  m_Q.clear ();
  m_Q_support.clear ();
  _dist.clear ();
  return 0;
}

int
MNM_Shortest_Path::all_to_one_FIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt *> &link_cost_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
    &node_cost_map,
  std::unordered_map<TInt, TFlt *> &dist_to_dest,
  std::unordered_map<TInt, TInt *> &output_map, TInt cost_position,
  TInt dist_position, TInt output_position)
{
  std::deque<TInt> m_Q = std::deque<TInt> ();
  std::unordered_map<TInt, bool> m_Q_support
    = std::unordered_map<TInt, bool> ();

  m_Q.push_back (dest_node_ID);
  m_Q_support.insert (std::pair<TInt, bool> (dest_node_ID, true));

  TInt _node_ID;
  for (const auto &node : graph.nodes ())
    {
      _node_ID = graph.get_id (node);
      if (_node_ID != dest_node_ID)
        {
          dist_to_dest[_node_ID][dist_position]
            = TFlt (std::numeric_limits<double>::infinity ());
          output_map[_node_ID][output_position]
            = -1; // If the destination is not accessible the output remains -1
          m_Q_support.insert (std::pair<TInt, bool> (_node_ID, false));
        }
    }
  dist_to_dest[dest_node_ID][dist_position] = TFlt (0);

  TInt _in_node_ID, _tmp_ID, _in_link_ID, _out_link_ID;
  TFlt _alt, _tmp_dist;
  while (m_Q.size () != 0)
    {
      _tmp_ID = m_Q.front ();
      m_Q.pop_front ();
      m_Q_support.find (_tmp_ID)->second = false;
      const auto &node = graph.get_node (_tmp_ID);
      _tmp_dist = dist_to_dest[_tmp_ID][dist_position];
      for (const auto &link : graph.connections (node, Direction::Incoming))
        {
          _in_node_ID = graph.get_id (graph.get_endpoints (link).first);
          _in_link_ID = graph.get_id (link);
          _alt = _tmp_dist
                 + link_cost_map.find (_in_link_ID)->second[cost_position];
          if (_tmp_ID != dest_node_ID)
            {
              _out_link_ID = output_map[_tmp_ID][output_position];
              if (_out_link_ID != -1
                  && node_cost_map.find (_in_link_ID) != node_cost_map.end ()
                  && node_cost_map.find (_in_link_ID)
                         ->second.find (_out_link_ID)
                       != node_cost_map.find (_in_link_ID)->second.end ())
                {
                  _alt += node_cost_map.find (_in_link_ID)
                            ->second.find (_out_link_ID)
                            ->second[cost_position];
                }
            }
          if (_alt < dist_to_dest[_in_node_ID][dist_position])
            {
              dist_to_dest[_in_node_ID][dist_position] = _alt;
              output_map[_in_node_ID][output_position] = _in_link_ID;
              if (!m_Q_support.find (_in_node_ID)->second)
                {
                  m_Q.push_back (_in_node_ID);
                  m_Q_support.find (_in_node_ID)->second = true;
                }
            }
        }
    }

  m_Q.clear ();
  m_Q_support.clear ();
  return 0;
}

int
MNM_Shortest_Path::all_to_one_LIFO (
  TInt dest_node_ID, const macposts::Graph &graph,
  const std::unordered_map<TInt, TFlt> &cost_map,
  std::unordered_map<TInt, TInt> &output_map)
{
  std::unordered_map<TInt, TFlt> _dist = std::unordered_map<TInt, TFlt> ();
  _dist.insert (std::pair<TInt, TFlt> (dest_node_ID, TFlt (0)));

  std::deque<TInt> m_Q = std::deque<TInt> ();
  std::unordered_map<TInt, bool> m_Q_support
    = std::unordered_map<TInt, bool> ();

  m_Q.push_front (dest_node_ID);
  m_Q_support.insert (std::pair<TInt, bool> (dest_node_ID, true));

  TInt _node_ID;
  for (const auto &node : graph.nodes ())
    {
      _node_ID = graph.get_id (node);
      if (_node_ID != dest_node_ID)
        {
          _dist.insert (
            std::pair<TInt,
                      TFlt> (_node_ID,
                             TFlt (std::numeric_limits<double>::infinity ())));
          if (output_map.find (_node_ID) != output_map.end ())
            {
              output_map.find (_node_ID)->second = -1;
            }
          else
            {
              output_map.insert (std::pair<TInt, TInt> (_node_ID, -1));
            }
          m_Q_support.insert (std::pair<TInt, bool> (_node_ID, false));
        }
    }

  TInt _in_node_ID, _tmp_ID, _in_link_ID;
  TFlt _alt, _tmp_dist;
  while (m_Q.size () != 0)
    {
      _tmp_ID = m_Q.front ();
      m_Q.pop_front ();
      m_Q_support.find (_tmp_ID)->second = false;
      _tmp_dist = _dist.find (_tmp_ID)->second;
      for (const auto &link : graph.links ())
        {
          _in_node_ID = graph.get_id (graph.get_endpoints (link).first);
          _in_link_ID = graph.get_id (link);
          _alt = _tmp_dist + cost_map.find (_in_link_ID)->second;
          if (_alt < _dist.find (_in_node_ID)->second)
            {
              _dist.find (_in_node_ID)->second = _alt;
              output_map.find (_in_node_ID)->second = _in_link_ID;
              if (!m_Q_support.find (_in_node_ID)->second)
                {
                  m_Q.push_front (_in_node_ID);
                  m_Q_support.find (_in_node_ID)->second = true;
                }
            }
        }
    }

  m_Q.clear ();
  m_Q_support.clear ();
  _dist.clear ();
  return 0;
}

bool
CompareCostDecendSort (MNM_Cost *lhs, MNM_Cost *rhs)
{
  return lhs->m_cost < rhs->m_cost;
}

/*------------------------------------------------------------
                  TDSP  one destination tree
-------------------------------------------------------------*/

/*------------------------------------------------------------
                          TDSP
-------------------------------------------------------------*/
bool
MNM_Shortest_Path::is_FIFO (const macposts::Graph &graph,
                            const std::unordered_map<TInt, TFlt *> &cost_map,
                            TInt num_interval, TFlt unit_time)
{
  TInt _edge_ID;
  TFlt *_cost_list;
  for (const auto &link : graph.links ())
    {
      _edge_ID = graph.get_id (link);
      _cost_list = cost_map.find (_edge_ID)->second;
      for (int i = 0; i < num_interval - 1; ++i)
        {
          if (_cost_list[i] > unit_time + _cost_list[i + 1])
            {
              return false;
            }
        }
    }
  return true;
}

/*------------------------------------------------------------
                  TDSP  one destination tree
-------------------------------------------------------------*/
MNM_TDSP_Tree::MNM_TDSP_Tree (TInt dest_node_ID, macposts::Graph &graph,
                              TInt max_interval)
    : m_graph (graph)
{
  m_dist = std::unordered_map<TInt, TFlt *> ();
  m_tree = std::unordered_map<TInt, TInt *> ();
  m_dest_node_ID = dest_node_ID;
  m_max_interval = max_interval;
}

MNM_TDSP_Tree::~MNM_TDSP_Tree ()
{
  TInt _node_ID;
  for (const auto &n : m_graph.nodes ())
    {
      _node_ID = m_graph.get_id (n);
      if (m_dist[_node_ID] != nullptr)
        delete m_dist[_node_ID];
      if (m_tree[_node_ID] != nullptr)
        delete m_tree[_node_ID];
    }
  m_dist.clear ();
  m_tree.clear ();
}

int
MNM_TDSP_Tree::initialize ()
{
  TInt _node_ID;
  for (const auto &n : m_graph.nodes ())
    {
      _node_ID = m_graph.get_id (n);
      m_dist.insert ({ _node_ID, new TFlt[m_max_interval] });
      m_tree.insert ({ _node_ID, new TInt[m_max_interval] });
    }
  return 0;
}

int
MNM_TDSP_Tree::update_tree (
  const std::unordered_map<TInt, TFlt *> &link_cost_map,
  const std::unordered_map<TInt, TFlt *> &link_tt_map)
{
  // printf("Init in update tree\n");
  // init tree and cost

  TInt _node_ID;
  for (const auto &n : m_graph.nodes ())
    {
      for (int t = 0; t < m_max_interval; ++t)
        {
          _node_ID = m_graph.get_id (n);
          m_dist[_node_ID][t]
            = _node_ID == m_dest_node_ID
                ? TFlt (0)
                : TFlt (std::numeric_limits<double>::infinity ());
          m_tree[_node_ID][t] = -1;
        }
    }

  // printf("SP in last t\n");
  // run last time interval
  // MNM_Shortest_Path::all_to_one_Dijkstra(m_dest_node_ID, m_graph, cost_map,
  // m_dist, m_tree,
  //                                        m_max_interval - 1, m_max_interval -
  //                                        1, m_max_interval - 1);
  MNM_Shortest_Path::all_to_one_FIFO (m_dest_node_ID, m_graph, link_cost_map,
                                      m_dist, m_tree, m_max_interval - 1,
                                      m_max_interval - 1, m_max_interval - 1);
  // printf("Process M-2 to 0\n");
  // main loop for t = M-2 down to 0
  // construct m_dist and m_tree in a reverse time order
  TFlt _temp_cost, _edge_cost, _edge_tt;
  TInt _src_node, _dst_node;
  for (int t = m_max_interval - 2; t > -1; t--)
    {
      // printf("%d\n", t);

      // DOT method has some drawbacks due to the time rounding issues, using
      // rounding up in get_distance_to_destination

      // if using rounding down in get_distance_to_destination to round the
      // given time down to the nearest integer when some links (usually for OD
      // connectors) have very short travel time (less than unit time interval)
      // the rounding down can be problematic since int(TFlt(t) + _edge_cost) ==
      // t so m_dist[_src_node][t] = m_dist[_dst_node][int(TFlt(t) +
      // _edge_cost)] = inf does not change so the loop for the links should
      // keep going on until all m_dist and m_tree are filled but be careful
      // that this while loop may not be able to break if many links have short
      // travel time

      for (const auto &l : m_graph.links ())
        {
          auto &&sd = m_graph.get_endpoints (l);
          _dst_node = m_graph.get_id (sd.first);
          _src_node = m_graph.get_id (sd.second);
          _edge_cost = link_cost_map.find (m_graph.get_id (l))->second[t];
          _edge_tt = link_tt_map.find (m_graph.get_id (l))->second[t];
          if (std::isinf (_edge_cost))
            {
              continue;
            }
          _temp_cost
            = _edge_cost
              + get_distance_to_destination (_dst_node, t, _edge_tt, 1e-4);
          if (m_dist[_src_node][t] > _temp_cost)
            {
              // printf("At time %d, src %d to des %d, m_dist is %f, _temp_cost
              // is %f\n", t, _src_node(),
              //        _edge_it.GetDstNId(), (float) m_dist[_src_node][t],
              //        (float) _temp_cost);
              m_dist[_src_node][t] = _temp_cost;
              m_tree[_src_node][t] = m_graph.get_id (l);
            }
        }
    }
  // printf("Finished update tree\n");
  return 0;
}

int
MNM_TDSP_Tree::update_tree (
  const std::unordered_map<TInt, TFlt *> &link_cost_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
    &node_cost_map,
  const std::unordered_map<TInt, TFlt *> &link_tt_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &node_tt_map)
{
  // printf("Init in update tree\n");
  // init tree and cost

  TInt _node_ID, _in_edge_ID, _out_edge_ID;
  for (const auto &n : m_graph.nodes ())
    {
      for (int t = 0; t < m_max_interval; ++t)
        {
          _node_ID = m_graph.get_id (n);
          m_dist[_node_ID][t]
            = _node_ID == m_dest_node_ID
                ? TFlt (0)
                : TFlt (std::numeric_limits<double>::infinity ());
          m_tree[_node_ID][t] = -1;
        }
    }

  // printf("SP in last t\n");
  // run last time interval
  MNM_Shortest_Path::all_to_one_FIFO (m_dest_node_ID, m_graph, link_cost_map,
                                      node_cost_map, m_dist, m_tree,
                                      m_max_interval - 1, m_max_interval - 1,
                                      m_max_interval - 1);
  // printf("Process M-2 to 0\n");
  // main loop for t = M-2 down to 0
  // construct m_dist and m_tree in a reverse time order
  TFlt _temp_cost, _temp_tt, _edge_cost, _edge_tt;
  TInt _src_node, _dst_node;
  for (int t = m_max_interval - 2; t > -1; t--)
    {
      // printf("%d\n", t);

      // DOT method has some drawbacks due to the time rounding issues, using
      // rounding up in get_distance_to_destination

      // if using rounding down in get_distance_to_destination to round the
      // given time down to the nearest integer when some links (usually for OD
      // connectors) have very short travel time (less than unit time interval)
      // the rounding down can be problematic since int(TFlt(t) + _edge_cost) ==
      // t so m_dist[_src_node][t] = m_dist[_dst_node][int(TFlt(t) +
      // _edge_cost)] = inf does not change so the loop for the links should
      // keep going on until all m_dist and m_tree are filled but be careful
      // that this while loop may not be able to break if many links have short
      // travel time

      // m_dist stores the distance to the dest node after traversing this node
      for (const auto &l : m_graph.links ())
        {
          _in_edge_ID = m_graph.get_id (l);
          auto &&sd = m_graph.get_endpoints (l);
          _dst_node = m_graph.get_id (sd.first);
          _src_node = m_graph.get_id (sd.second);
          // _src_node -> _edge_cost -> _dst_node -> node_cost -> the beigining
          // of next link after _dst_node
          _edge_cost = link_cost_map.find (m_graph.get_id (l))->second[t];
          _edge_tt = link_tt_map.find (m_graph.get_id (l))->second[t];
          if (std::isinf (_edge_cost))
            {
              continue;
            }
          _temp_cost = _edge_cost;
          _temp_tt = _edge_tt;
          if (_dst_node != m_dest_node_ID)
            {
              _out_edge_ID = m_tree[_dst_node][round_time (t, _temp_tt)];
              if (_out_edge_ID != -1
                  && node_cost_map.find (_in_edge_ID) != node_cost_map.end ()
                  && node_cost_map.find (_in_edge_ID)
                         ->second.find (_out_edge_ID)
                       != node_cost_map.find (_in_edge_ID)->second.end ())
                {
                  _temp_cost
                    += node_cost_map.find (_in_edge_ID)
                         ->second.find (_out_edge_ID)
                         ->second[round_time (t, _temp_tt)]; // node cost can be
                                                             // zero
                  _temp_tt
                    += node_tt_map.find (_in_edge_ID)
                         ->second.find (_out_edge_ID)
                         ->second[round_time (t,
                                              _temp_tt)]; // node tt can be zero
                }
            }
          _temp_cost += get_distance_to_destination (_dst_node, t, _temp_tt);
          if (m_dist[_src_node][t] > _temp_cost)
            {
              // printf("At time %d, src %d to des %d, m_dist is %f, _temp_cost
              // is %f\n", t, _src_node(),
              //        _edge_it.GetDstNId(), (float) m_dist[_src_node][t],
              //        (float) _temp_cost);
              m_dist[_src_node][t] = _temp_cost;
              m_tree[_src_node][t] = m_graph.get_id (l);
            }
        }
    }
  // printf("Finished update tree\n");
  return 0;
}

TFlt
MNM_TDSP_Tree::get_tdsp (TInt src_node_ID, TInt time,
                         const std::unordered_map<TInt, TFlt *> &link_tt_map,
                         MNM_Path *path)
{
  TInt _cur_node_ID = src_node_ID;
  TInt _cur_link_ID;
  TFlt _tt = 0.;
  int _cur_time
    = int (time) < (int) m_max_interval ? int (time) : (int) m_max_interval - 1;
  while (_cur_node_ID != m_dest_node_ID)
    {
      path->m_node_vec.push_back (_cur_node_ID);
      // _cur_link_ID = m_tree[_cur_node_ID][round_time(_cur_time)];
      _cur_link_ID = m_tree[_cur_node_ID][_cur_time];
      if (_cur_link_ID == -1)
        {
          printf ("No available path between node %d and node %d\n",
                  src_node_ID, m_dest_node_ID);
          // exit(-1);
          return -1;
        }
      path->m_link_vec.push_back (_cur_link_ID);
      _tt += link_tt_map.find (_cur_link_ID)->second[_cur_time];
      _cur_time
        = round_time (_cur_time,
                      link_tt_map.find (_cur_link_ID)->second[_cur_time]);
      _cur_node_ID
        = m_graph.get_id (m_graph.get_endpoints (_cur_link_ID).second);
    }
  path->m_node_vec.push_back (m_dest_node_ID);
  return _tt;
}

TFlt
MNM_TDSP_Tree::get_tdsp (
  TInt src_node_ID, TInt time,
  const std::unordered_map<TInt, TFlt *> &link_tt_map,
  const std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &node_tt_map,
  MNM_Path *path)
{
  TInt _cur_node_ID = src_node_ID;
  TInt _cur_link_ID;
  TFlt _tt = 0.;
  int _cur_time
    = int (time) < (int) m_max_interval ? int (time) : (int) m_max_interval - 1;
  while (_cur_node_ID != m_dest_node_ID)
    {
      path->m_node_vec.push_back (_cur_node_ID);
      _cur_link_ID = m_tree[_cur_node_ID][_cur_time];
      if (_cur_link_ID == -1)
        {
          printf ("No available path between node %d and node %d\n",
                  src_node_ID, m_dest_node_ID);
          // exit(-1);
          return -1;
        }
      path->m_link_vec.push_back (_cur_link_ID);
      // first node cost, then link cost
      if (_cur_node_ID != src_node_ID && path->m_link_vec.size () >= 2
          && node_tt_map.find (path->m_link_vec[path->m_link_vec.size () - 2])
               != node_tt_map.end ()
          && node_tt_map.find (path->m_link_vec[path->m_link_vec.size () - 2])
                 ->second.find (_cur_link_ID)
               != node_tt_map
                    .find (path->m_link_vec[path->m_link_vec.size () - 2])
                    ->second.end ())
        {
          _tt += node_tt_map
                   .find (path->m_link_vec[path->m_link_vec.size () - 2])
                   ->second.find (_cur_link_ID)
                   ->second[_cur_time];
          _cur_time += int (ceil (
            node_tt_map.find (path->m_link_vec[path->m_link_vec.size () - 2])
              ->second.find (_cur_link_ID)
              ->second[_cur_time])); // node tt can be zero
          _cur_time = _cur_time < (int) m_max_interval
                        ? _cur_time
                        : (int) m_max_interval - 1;
        }
      _tt += link_tt_map.find (_cur_link_ID)->second[_cur_time];
      _cur_time = round_time (_cur_time,
                              link_tt_map.find (_cur_link_ID)
                                ->second[_cur_time]); // link tt cannot be zero
      _cur_node_ID
        = m_graph.get_id (m_graph.get_endpoints (_cur_link_ID).second);
    }
  path->m_node_vec.push_back (m_dest_node_ID);
  return _tt;
}

TFlt
MNM_TDSP_Tree::get_distance_to_destination (TInt node_ID, TFlt time_stamp)
{
  // Warning: may be incorrect when time_stamp is exactly an integer
  IAssert (m_dist.find (node_ID) != m_dist.end ());
  IAssert (m_dist[node_ID] != nullptr);
  IAssert (time_stamp >= 0);
  // printf("Current time stamp is %lf, %d\n", time_stamp, int(time_stamp)+ 1);

  // if (time_stamp >= TFlt(m_max_interval - 1)){
  //   // printf("Enter if\n");
  //   return m_dist[node_ID][m_max_interval - 1];
  // }
  // return m_dist[node_ID][int(time_stamp)];
  return m_dist[node_ID][round_time (time_stamp)];
}

TFlt
MNM_TDSP_Tree::get_distance_to_destination (TInt node_ID, int start_time_stamp,
                                            TFlt travel_time, float p)
{
  IAssert (m_dist.find (node_ID) != m_dist.end ());
  IAssert (m_dist[node_ID] != nullptr);
  IAssert (start_time_stamp >= 0);
  int _end_time_stamp = round_time (start_time_stamp, travel_time, p);
  // printf("start time stamp is %d, end time stamp is %d\n", start_time_stamp,
  // _end_time_stamp);
  return m_dist[node_ID][_end_time_stamp];
}

int
MNM_TDSP_Tree::round_time (TFlt time_stamp)
{
  // Warning: may be incorrect when time_stamp is exactly an integer
  if (time_stamp >= TFlt (m_max_interval - 1))
    {
      // printf("Enter if\n");
      return int (m_max_interval - 1);
    }
  return int (time_stamp) + 1;
}

int
MNM_TDSP_Tree::round_time (int start_time_stamp, TFlt travel_time, float p)
{
  int _end_time_stamp
    = start_time_stamp + MNM_Ults::round_up_time (travel_time, p);
  IAssert (_end_time_stamp > start_time_stamp);
  if (_end_time_stamp >= m_max_interval - 1)
    {
      // printf("Enter if\n");
      return int (m_max_interval - 1);
    }
  else
    {
      return _end_time_stamp;
    }
}
