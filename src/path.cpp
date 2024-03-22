#include "path.h"

/**************************************************************************
                              Path
**************************************************************************/
MNM_Path::MNM_Path ()
{
  m_link_vec = std::deque<TInt> ();
  m_node_vec = std::deque<TInt> ();
  m_buffer_length = 0;
  m_p = 0;
  m_buffer = nullptr;
  m_path_ID = -1;
  m_link_set = std::set<TInt> ();

  m_travel_time_vec = std::vector<TFlt> ();
  m_travel_cost_vec = std::vector<TFlt> ();
  m_travel_disutility_vec = std::vector<TFlt> ();
}

MNM_Path::~MNM_Path ()
{
  m_link_vec.clear ();
  m_node_vec.clear ();
  if (m_buffer != nullptr)
    free (m_buffer);
  m_link_set.clear ();

  m_travel_time_vec.clear ();
  m_travel_cost_vec.clear ();
  m_travel_disutility_vec.clear ();
}

bool
MNM_Path::is_link_in (TInt link_ID)
{
  if (m_link_set.empty ())
    {
      m_link_set = std::set<TInt> (m_link_vec.begin (), m_link_vec.end ());
    }
  IAssert (!m_link_set.empty ());
  if (m_link_set.find (link_ID) != m_link_set.end ())
    {
      return true;
    }
  else
    {
      return false;
    }
}

TFlt
MNM_Path::get_path_tt (MNM_Link_Factory *link_factory)
{
  // only used in DNL
  // MNM_Dlink *_link;

  TFlt _total_time = 0.0;
  for (TInt _link_ID : m_link_vec)
    {
      _total_time += link_factory->get_link (_link_ID)->get_link_tt ();
    }
  // printf("%lf\n", _total_time());
  return _total_time;
}

TFlt
MNM_Path::get_path_fftt (MNM_Link_Factory *link_factory)
{
  MNM_Dlink *_link;
  TFlt _total_time = 0.0;
  for (TInt _link_ID : m_link_vec)
    {
      _link = link_factory->get_link (_link_ID);
      _total_time += _link->m_length / _link->m_ffs; // in seconds
    }
  // printf("%lf\n", _total_time());
  return _total_time;
}

TFlt
MNM_Path::get_path_length (MNM_Link_Factory *link_factory)
{
  // MNM_Dlink *_link;

  TFlt _total_length = 0.0;
  for (TInt _link_ID : m_link_vec)
    {
      _total_length += link_factory->get_link (_link_ID)->m_length;
    }
  // printf("%lf\n", _total_length());
  return _total_length;
}

std::string
MNM_Path::node_vec_to_string ()
{
  std::string _s;
  for (TInt node_ID : m_node_vec)
    {
      _s += std::to_string (node_ID) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

std::string
MNM_Path::link_vec_to_string ()
{
  std::string _s;
  for (TInt link_ID : m_link_vec)
    {
      _s += std::to_string (link_ID) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

std::string
MNM_Path::time_vec_to_string ()
{
  std::string _s;
  for (TFlt _v : m_travel_time_vec)
    {
      _s += std::to_string (_v) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

std::string
MNM_Path::cost_vec_to_string ()
{
  std::string _s;
  for (TFlt _v : m_travel_cost_vec)
    {
      _s += std::to_string (_v) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

std::string
MNM_Path::disutility_vec_to_string ()
{
  std::string _s;
  for (TFlt _v : m_travel_disutility_vec)
    {
      _s += std::to_string (_v) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

std::string
MNM_Path::buffer_to_string ()
{
  std::string _s;
  if (m_buffer_length == 0)
    {
      return "\n";
    }
  for (int i = 0; i < m_buffer_length; ++i)
    {
      _s += std::to_string (m_buffer[i]) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

int
MNM_Path::allocate_buffer (TInt length)
{
  if ((m_buffer_length > 0) || (m_buffer != nullptr))
    {
      throw std::runtime_error (
        "Error: MNM_Path::allocate_buffer, double allocation.");
    }
  m_buffer_length = length;
  // malloc returns void*, static_cast<TFlt*> casts void* into TFlt*
  // https://embeddedartistry.com/blog/2017/03/15/c-casting-or-oh-no-they-broke-malloc/
  m_buffer = static_cast<TFlt *> (std::malloc (sizeof (TFlt) * length));
  for (int i = 0; i < length; ++i)
    {
      m_buffer[i] = 0.0;
    }
  return 0;
}

int
MNM_Path::eliminate_cycles ()
{
  bool _flg = false;
  TInt _node_ID;
  std::vector<bool> _node_reserved = std::vector<bool> ();
  std::vector<bool> _link_reserved = std::vector<bool> ();
  IAssert (m_node_vec.size () == m_link_vec.size () + 1);

  for (size_t i = 0; i < m_node_vec.size (); ++i)
    {
      _node_reserved.push_back (true);
      if (i < m_node_vec.size () - 1)
        {
          _link_reserved.push_back (true);
        }
    }

  for (size_t i = 0; i < m_node_vec.size () - 1; ++i)
    {
      if (!_node_reserved[i])
        {
          IAssert (i > 0 && !_link_reserved[i - 1]);
          continue;
        }
      _node_ID = m_node_vec[i];
      // find the position of the last occurrence,
      // https://stackoverflow.com/questions/35822606/remove-last-occurrence-of-an-element-in-a-vector-stl
      auto _foundIt
        = std::find (m_node_vec.rbegin (), m_node_vec.rend (), _node_ID);
      auto _toRemove = --(_foundIt.base ());
      int j = (int) std::distance (m_node_vec.begin (), _toRemove);
      if ((int) i + 1 <= j)
        {
          _flg = true;
          for (int k = (int) i + 1; k < j + 1; ++k)
            {
              _node_reserved[k] = false;
              _link_reserved[k - 1] = false;
            }
        }
    }
  IAssert (_node_reserved.size () == _link_reserved.size () + 1);

  if (_flg)
    {
      std::deque<TInt> _node_vec = m_node_vec;
      std::deque<TInt> _link_vec = m_link_vec;
      m_node_vec.clear ();
      m_link_vec.clear ();
      for (size_t i = 0; i < _node_vec.size (); ++i)
        {
          if (_node_reserved[i])
            {
              m_node_vec.push_back (_node_vec[i]);
            }
          if (i < _node_vec.size () - 1)
            {
              if (_link_reserved[i])
                {
                  m_link_vec.push_back (_link_vec[i]);
                }
            }
        }

      if (m_node_vec.size () < _node_vec.size ())
        {
          printf ("cycles eliminated\n");
          printf ("modified node_vec is: \n");
          std::cout << node_vec_to_string ();
        }
      _node_vec.clear ();
      _link_vec.clear ();
    }
  IAssert (m_node_vec.size () == m_link_vec.size () + 1);
  _node_reserved.clear ();
  _link_reserved.clear ();

  return 0;
}

/**************************************************************************
                            Path Set
**************************************************************************/

MNM_Pathset::MNM_Pathset () { m_path_vec = std::vector<MNM_Path *> (); }

MNM_Pathset::~MNM_Pathset ()
{
  for (MNM_Path *_path : m_path_vec)
    {
      delete _path;
    }
  m_path_vec.clear ();
}

bool
MNM_Pathset::is_in (MNM_Path *path)
{
  for (MNM_Path *tmp_path : m_path_vec)
    {
      if (*tmp_path == *path)
        return true;
    }
  return false;
}

int
MNM_Pathset::normalize_p ()
{
  TFlt _tot_p = TFlt (0);
  TFlt _min_p = TFlt (0);
  for (MNM_Path *_path : m_path_vec)
    {
      if (_path->m_p < 0)
        {
          throw std::runtime_error ("invalid probability");
        }
      if (_path->m_p < _min_p)
        {
          _min_p = _path->m_p;
        }
    }
  for (MNM_Path *_path : m_path_vec)
    {
      _tot_p += _path->m_p - _min_p;
    }
  if (_tot_p == TFlt (0))
    {
      for (MNM_Path *_path : m_path_vec)
        {
          _path->m_p = TFlt (1) / TFlt (m_path_vec.size ());
        }
    }
  else
    {
      TFlt _true_total_p = TFlt (0);
      for (MNM_Path *_path : m_path_vec)
        {
          _true_total_p += _path->m_p;
        }
      for (MNM_Path *_path : m_path_vec)
        {
          _path->m_p = (_path->m_p) / _true_total_p;
        }
    }
  return 0;
}

namespace MNM
{
MNM_Path *
extract_path (TInt origin_node_ID, TInt dest_node_ID,
              std::unordered_map<TInt, TInt> &output_map, PNEGraph &graph)
{
  // output_map[node_ID][edge_ID], tdsp tree
  // printf("Entering extract_path\n");
  TInt _current_node_ID = origin_node_ID;
  TInt _current_link_ID = -1;
  MNM_Path *_path = new MNM_Path ();
  while (_current_node_ID != dest_node_ID)
    {
      if (output_map.find (_current_node_ID) == output_map.end ())
        {
          // printf("Cannot extract path\n");
          return nullptr;
        }
      _current_link_ID = output_map[_current_node_ID];
      if (_current_link_ID == -1)
        {
          printf ("Cannot extract path from origin node %d to destination node "
                  "%d\n",
                  origin_node_ID (), dest_node_ID ());
          return nullptr;
        }
      _path->m_node_vec.push_back (_current_node_ID);
      _path->m_link_vec.push_back (_current_link_ID);
      _current_node_ID = graph->GetEI (_current_link_ID).GetDstNId ();
    }
  _path->m_node_vec.push_back (_current_node_ID);
  // printf("Exiting extract_path\n");
  return _path;
}

MNM_Path *
extract_path (TInt origin_node_ID, TInt dest_node_ID,
              std::unordered_map<TInt, TInt> &output_map,
              macposts::Graph &graph)
{
  // output_map[node_ID][edge_ID], tdsp tree
  // printf("Entering extract_path\n");
  TInt _current_node_ID = origin_node_ID;
  TInt _current_link_ID = -1;
  MNM_Path *_path = new MNM_Path ();
  while (_current_node_ID != dest_node_ID)
    {
      if (output_map.find (_current_node_ID) == output_map.end ())
        {
          // printf("Cannot extract path\n");
          return nullptr;
        }
      _current_link_ID = output_map[_current_node_ID];
      if (_current_link_ID == -1)
        {
          printf ("Cannot extract path from origin node %d to destination node "
                  "%d\n",
                  origin_node_ID (), dest_node_ID ());
          return nullptr;
        }
      _path->m_node_vec.push_back (_current_node_ID);
      _path->m_link_vec.push_back (_current_link_ID);
      _current_node_ID
        = graph.get_id (graph.get_endpoints (_current_link_ID).second);
    }
  _path->m_node_vec.push_back (_current_node_ID);
  // printf("Exiting extract_path\n");
  return _path;
}

TFlt
get_path_tt_snapshot (MNM_Path *path,
                      const std::unordered_map<TInt, TFlt> &link_cost_map)
{
  TFlt _tt = TFlt (0);
  for (auto _link_ID : path->m_link_vec)
    {
      if (link_cost_map.find (_link_ID) == link_cost_map.end ())
        {
          throw std::runtime_error ("Wrong link in get_path_tt_snapshot()");
        }
      _tt += link_cost_map.find (_link_ID)->second;
    }
  return _tt;
}

TFlt
get_path_tt (TFlt start_time, MNM_Path *path,
             const std::unordered_map<TInt, TFlt *> &link_cost_map,
             TInt max_interval)
{
  int _end_time = int (round (start_time));
  for (auto _link_ID : path->m_link_vec)
    {
      if (link_cost_map.find (_link_ID) == link_cost_map.end ())
        {
          throw std::runtime_error ("Wrong link in get_path_tt()");
        }
      _end_time
        += link_cost_map.find (_link_ID)
             ->second[_end_time < (int) max_interval ? _end_time
                                                     : (int) max_interval - 1];
    }
  return TFlt (_end_time - start_time);
}

Path_Table *
build_shortest_pathset (PNEGraph &graph, MNM_OD_Factory *od_factory,
                        MNM_Link_Factory *link_factory)
{
  // this build for each OD pair, no matter this OD pair exists in demand file
  // or not
  Path_Table *_path_table = new Path_Table ();
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      std::unordered_map<TInt, MNM_Pathset *> *_new_map
        = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                          *> (_o_it->second->m_origin_node->m_node_ID,
                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          MNM_Pathset *_pathset = new MNM_Pathset ();
          _new_map->insert (
            std::pair<TInt, MNM_Pathset *> (_d_it->second->m_dest_node
                                              ->m_node_ID,
                                            _pathset));
        }
    }
  TInt _dest_node_ID, _origin_node_ID;
  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree;
  MNM_Path *_path;
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               _link_it->second->get_link_tt ()));
    }
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph, _free_cost_map,
                                          _free_shortest_path_tree);
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _free_shortest_path_tree, graph);
          if (_path != nullptr)
            {
              // printf("Adding to path table\n");
              // std::cout << _path -> node_vec_to_string();
              // std::cout << _path -> link_vec_to_string();
              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->m_path_vec.push_back (_path);
            }
          else
            {
              throw std::runtime_error (
                "no path between origin " + std::to_string (_origin_node_ID ())
                + " and destination " + std::to_string (_dest_node_ID ()));
            }
        }
    }
  return _path_table;
}

Path_Table *
build_shortest_pathset (macposts::Graph &graph, MNM_OD_Factory *od_factory,
                        MNM_Link_Factory *link_factory)
{
  // this build for each OD pair, no matter this OD pair exists in demand file
  // or not
  Path_Table *_path_table = new Path_Table ();
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      std::unordered_map<TInt, MNM_Pathset *> *_new_map
        = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                          *> (_o_it->second->m_origin_node->m_node_ID,
                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          MNM_Pathset *_pathset = new MNM_Pathset ();
          _new_map->insert (
            std::pair<TInt, MNM_Pathset *> (_d_it->second->m_dest_node
                                              ->m_node_ID,
                                            _pathset));
        }
    }
  TInt _dest_node_ID, _origin_node_ID;
  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree;
  MNM_Path *_path;
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               _link_it->second->get_link_tt ()));
    }
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph, _free_cost_map,
                                          _free_shortest_path_tree);
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _free_shortest_path_tree, graph);
          if (_path != nullptr)
            {
              // printf("Adding to path table\n");
              // std::cout << _path -> node_vec_to_string();
              // std::cout << _path -> link_vec_to_string();
              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->m_path_vec.push_back (_path);
            }
          else
            {
              throw std::runtime_error (
                "no path between origin " + std::to_string (_origin_node_ID ())
                + " and destination " + std::to_string (_dest_node_ID ()));
            }
        }
    }
  return _path_table;
}

Path_Table *
build_pathset (PNEGraph &graph, MNM_OD_Factory *od_factory,
               MNM_Link_Factory *link_factory, TFlt min_path_length,
               size_t MaxIter, TFlt vot, TFlt Mid_Scale, TFlt Heavy_Scale,
               TInt buffer_length)
{
  // printf("11\n");
  // MaxIter: maximum iteration to find alternative shortest path, when MaxIter
  // = 0, just shortest path Mid_Scale and Heavy_Scale are different penalties
  // to the travel cost of links in existing paths
  IAssert (vot > 0);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  Path_Table *_path_table = new Path_Table ();
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      std::unordered_map<TInt, MNM_Pathset *> *_new_map
        = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<TInt, MNM_Pathset *> *> (_origin_node_ID,
                                                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          // assume build_demand is called before this function
          if (_o_it->second->m_demand.find (_d_it->second)
              != _o_it->second->m_demand.end ())
            {
              _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
              MNM_Pathset *_pathset = new MNM_Pathset ();
              _new_map->insert (
                std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
            }
        }
    }

  // printf("111\n");
  std::unordered_map<TInt, TInt> _mid_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map
    = std::unordered_map<TInt, TFlt> ();

  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  MNM_Path *_path;
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               vot * _link_it->second->get_link_tt ()
                                 + _link_it->second->m_toll));
    }
  // printf("1111\n");
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph, _free_cost_map,
                                          _free_shortest_path_tree);
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          if (_o_it->second->m_demand.find (_d_it->second)
              == _o_it->second->m_demand.end ())
            {
              continue;
            }
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _free_shortest_path_tree, graph);
          if (_path != nullptr)
            {
              if (_path->get_path_length (link_factory) > min_path_length)
                {
                  if (buffer_length > 0)
                    {
                      _path->allocate_buffer (buffer_length);
                    }
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_path);
                }
            }
          else
            {
              throw std::runtime_error (
                "no path between origin " + std::to_string (_origin_node_ID ())
                + " and destination " + std::to_string (_dest_node_ID ()));
            }
        }
    }
  // printf("22\n");
  _mid_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());
  _heavy_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());

  MNM_Dlink *_link;
  MNM_Path *_path_mid, *_path_heavy;
  size_t _CurIter = 0;
  while (_CurIter < MaxIter)
    {
      printf ("Current trial %d\n", (int) _CurIter);
      for (auto _o_it : *_path_table)
        {
          for (auto _d_it : *_o_it.second)
            {
              for (auto &_path : _d_it.second->m_path_vec)
                {
                  for (auto &_link_ID : _path->m_link_vec)
                    {
                      _link = link_factory->get_link (_link_ID);
                      _mid_cost_map.find (_link_ID)->second
                        = vot * _link->get_link_tt () * Mid_Scale
                          + _link->m_toll;
                      _heavy_cost_map.find (_link_ID)->second
                        = vot * _link->get_link_tt () * Heavy_Scale
                          + _link->m_toll;
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _mid_cost_map,
                                              _mid_shortest_path_tree);
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _heavy_cost_map,
                                              _heavy_shortest_path_tree);
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              if (_o_it->second->m_demand.find (_d_it->second)
                  == _o_it->second->m_demand.end ())
                {
                  continue;
                }
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              _path_mid = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                             _mid_shortest_path_tree, graph);
              _path_heavy
                = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _heavy_shortest_path_tree, graph);
              if (_path_mid != nullptr)
                {
                  if (!_path_table->find (_origin_node_ID)
                         ->second->find (_dest_node_ID)
                         ->second->is_in (_path_mid))
                    {
                      if (_path_mid->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_mid->allocate_buffer (buffer_length);
                            }
                          _path_table->find (_origin_node_ID)
                            ->second->find (_dest_node_ID)
                            ->second->m_path_vec.push_back (_path_mid);
                        }
                    }
                  else
                    {
                      delete _path_mid;
                    }
                }
              if (_path_heavy != nullptr)
                {
                  if (!_path_table->find (_origin_node_ID)
                         ->second->find (_dest_node_ID)
                         ->second->is_in (_path_heavy))
                    {
                      if (_path_heavy->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_heavy->allocate_buffer (buffer_length);
                            }
                          _path_table->find (_origin_node_ID)
                            ->second->find (_dest_node_ID)
                            ->second->m_path_vec.push_back (_path_heavy);
                        }
                    }
                  else
                    {
                      delete _path_heavy;
                    }
                }
            }
        }
      _CurIter += 1;
    }

  _mid_shortest_path_tree.clear ();
  _mid_cost_map.clear ();
  _heavy_shortest_path_tree.clear ();
  _heavy_cost_map.clear ();

  _free_cost_map.clear ();
  _free_shortest_path_tree.clear ();

  return _path_table;
}

Path_Table *
build_pathset (macposts::Graph &graph, MNM_OD_Factory *od_factory,
               MNM_Link_Factory *link_factory, TFlt min_path_length,
               size_t MaxIter, TFlt vot, TFlt Mid_Scale, TFlt Heavy_Scale,
               TInt buffer_length)
{
  // printf("11\n");
  // MaxIter: maximum iteration to find alternative shortest path, when MaxIter
  // = 0, just shortest path Mid_Scale and Heavy_Scale are different penalties
  // to the travel cost of links in existing paths
  IAssert (vot > 0);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  Path_Table *_path_table = new Path_Table ();
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      std::unordered_map<TInt, MNM_Pathset *> *_new_map
        = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<TInt, MNM_Pathset *> *> (_origin_node_ID,
                                                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          // assume build_demand is called before this function
          if (_o_it->second->m_demand.find (_d_it->second)
              != _o_it->second->m_demand.end ())
            {
              _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
              MNM_Pathset *_pathset = new MNM_Pathset ();
              _new_map->insert (
                std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
            }
        }
    }

  // printf("111\n");
  std::unordered_map<TInt, TInt> _mid_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map
    = std::unordered_map<TInt, TFlt> ();

  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  MNM_Path *_path;
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               vot * _link_it->second->get_link_tt ()
                                 + _link_it->second->m_toll));
    }
  // printf("1111\n");
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph, _free_cost_map,
                                          _free_shortest_path_tree);
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          if (_o_it->second->m_demand.find (_d_it->second)
              == _o_it->second->m_demand.end ())
            {
              continue;
            }
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _free_shortest_path_tree, graph);
          if (_path != nullptr)
            {
              if (_path->get_path_length (link_factory) > min_path_length)
                {
                  if (buffer_length > 0)
                    {
                      _path->allocate_buffer (buffer_length);
                    }
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_path);
                }
            }
          else
            {
              throw std::runtime_error (
                "no path between origin " + std::to_string (_origin_node_ID ())
                + " and destination " + std::to_string (_dest_node_ID ()));
            }
        }
    }
  // printf("22\n");
  _mid_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());
  _heavy_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());

  MNM_Dlink *_link;
  MNM_Path *_path_mid, *_path_heavy;
  size_t _CurIter = 0;
  while (_CurIter < MaxIter)
    {
      printf ("Current trial %d\n", (int) _CurIter);
      for (auto _o_it : *_path_table)
        {
          for (auto _d_it : *_o_it.second)
            {
              for (auto &_path : _d_it.second->m_path_vec)
                {
                  for (auto &_link_ID : _path->m_link_vec)
                    {
                      _link = link_factory->get_link (_link_ID);
                      _mid_cost_map.find (_link_ID)->second
                        = vot * _link->get_link_tt () * Mid_Scale
                          + _link->m_toll;
                      _heavy_cost_map.find (_link_ID)->second
                        = vot * _link->get_link_tt () * Heavy_Scale
                          + _link->m_toll;
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _mid_cost_map,
                                              _mid_shortest_path_tree);
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _heavy_cost_map,
                                              _heavy_shortest_path_tree);
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              if (_o_it->second->m_demand.find (_d_it->second)
                  == _o_it->second->m_demand.end ())
                {
                  continue;
                }
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              _path_mid = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                             _mid_shortest_path_tree, graph);
              _path_heavy
                = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _heavy_shortest_path_tree, graph);
              if (_path_mid != nullptr)
                {
                  if (!_path_table->find (_origin_node_ID)
                         ->second->find (_dest_node_ID)
                         ->second->is_in (_path_mid))
                    {
                      if (_path_mid->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_mid->allocate_buffer (buffer_length);
                            }
                          _path_table->find (_origin_node_ID)
                            ->second->find (_dest_node_ID)
                            ->second->m_path_vec.push_back (_path_mid);
                        }
                    }
                  else
                    {
                      delete _path_mid;
                    }
                }
              if (_path_heavy != nullptr)
                {
                  if (!_path_table->find (_origin_node_ID)
                         ->second->find (_dest_node_ID)
                         ->second->is_in (_path_heavy))
                    {
                      if (_path_heavy->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_heavy->allocate_buffer (buffer_length);
                            }
                          _path_table->find (_origin_node_ID)
                            ->second->find (_dest_node_ID)
                            ->second->m_path_vec.push_back (_path_heavy);
                        }
                    }
                  else
                    {
                      delete _path_heavy;
                    }
                }
            }
        }
      _CurIter += 1;
    }

  _mid_shortest_path_tree.clear ();
  _mid_cost_map.clear ();
  _heavy_shortest_path_tree.clear ();
  _heavy_cost_map.clear ();

  _free_cost_map.clear ();
  _free_shortest_path_tree.clear ();

  return _path_table;
}

int
save_path_table (const std::string &file_folder, Path_Table *path_table,
                 MNM_OD_Factory *od_factory, bool w_buffer, bool w_cost)
{
  std::string _path_file_name = file_folder + "/path_table";
  std::ofstream _path_buffer_file;
  if (w_buffer)
    {
      std::string _data_file_name = file_folder + "/path_table_buffer";
      _path_buffer_file.open (_data_file_name, std::ofstream::out);
      if (!_path_buffer_file.is_open ())
        {
          throw std::runtime_error ("failed to open file: " + _data_file_name);
        }
    }
  std::ofstream _path_table_file;
  _path_table_file.open (_path_file_name, std::ofstream::out);
  if (!_path_table_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + _path_file_name);
    }

  std::ofstream _path_time_file;
  // std::ofstream _path_cost_file;
  std::ofstream _path_disutility_file;
  if (w_cost)
    {
      std::string _path_time_file_name = _path_file_name + "_time";
      _path_time_file.open (_path_time_file_name, std::ofstream::out);
      if (!_path_time_file.is_open ())
        {
          throw std::runtime_error ("failed to open file: "
                                    + _path_time_file_name);
        }
      std::string _path_disutility_file_name = _path_file_name + "_disutility";
      _path_disutility_file.open (_path_disutility_file_name,
                                  std::ofstream::out);
      if (!_path_disutility_file.is_open ())
        {
          throw std::runtime_error ("failed to open file: "
                                    + _path_disutility_file_name);
        }
    }

  for (auto _o_it : *path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto &_path : _d_it.second->m_path_vec)
            {
              _path_table_file << _path->node_vec_to_string ();
              // printf("test2\n");
              if (w_buffer)
                {
                  _path_buffer_file << _path->buffer_to_string ();
                }
              if (w_cost)
                {
                  _path_time_file << _path->time_vec_to_string ();
                  // _path_cost_file << _path -> cost_vec_to_string();
                  _path_disutility_file << _path->disutility_vec_to_string ();
                }
            }
        }
    }
  _path_table_file.close ();
  if (w_buffer)
    {
      _path_buffer_file.close ();
    }
  if (w_cost)
    {
      _path_time_file.close ();
      // _path_cost_file.close();
      _path_disutility_file.close ();
    }
  return 0;
}

int
print_path_table (Path_Table *path_table, MNM_OD_Factory *od_factory,
                  bool w_buffer, bool w_cost)
{
  // TInt _dest_node_ID, _origin_node_ID;
  // // printf("ssssssma\n");
  // for (auto _d_it = od_factory->m_destination_map.begin();
  //      _d_it != od_factory->m_destination_map.end(); _d_it++) {
  //     // printf("---\n");
  //     _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
  //     for (auto _o_it = od_factory->m_origin_map.begin(); _o_it !=
  //     od_factory->m_origin_map.end(); _o_it++) {
  //         _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
  //         // printf("----\n");
  //         // printf("o node %d, d node %d\n", _origin_node_ID(),
  //         _dest_node_ID()); for (auto &_path :
  //         path_table->find(_origin_node_ID)->second->find(_dest_node_ID)->second->m_path_vec)
  //         {
  //             // printf("test\n");
  //             std::cout << "path: " << _path->node_vec_to_string();
  //             // printf("test2\n");
  //             if (w_buffer) {
  //                 std::cout << "buffer: " << _path->buffer_to_string();
  //             }
  //         }
  //     }
  // }
  for (auto _o_it : *path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto &_path : _d_it.second->m_path_vec)
            {
              std::cout << "path: " << _path->node_vec_to_string ();
              // printf("test2\n");
              if (w_buffer)
                {
                  std::cout << "buffer: " << _path->buffer_to_string ();
                }
              if (w_cost)
                {
                  std::cout << "travel time: " << _path->time_vec_to_string ();
                  // std::cout << "travel cost: " << _path ->
                  // cost_vec_to_string();
                  std::cout << "travel disutility: "
                            << _path->disutility_vec_to_string ();
                }
            }
        }
    }
  return 0;
}

int
allocate_path_table_buffer (Path_Table *path_table, TInt num)
{
  for (auto _it : *path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (MNM_Path *_path : _it_it.second->m_path_vec)
            {
              _path->allocate_buffer (num);
            }
        }
    }
  return 0;
}

int
normalize_path_table_p (Path_Table *path_table)
{
  for (auto _it : *path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          _it_it.second->normalize_p ();
        }
    }
  return 0;
}

int
copy_p_to_buffer (Path_Table *path_table, TInt col)
{
  for (auto _it : *path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (MNM_Path *_path : _it_it.second->m_path_vec)
            {
              _path->m_buffer[col] = _path->m_p;
            }
        }
    }
  return 0;
}

int
copy_buffer_to_p (Path_Table *path_table, TInt col)
{
  // printf("Entering MNM::copy_buffer_to_p\n");
  // printf("path table is %p\n", path_table);
  IAssert (col >= 0);
  for (auto _it : *path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (MNM_Path *_path : _it_it.second->m_path_vec)
            {
              IAssert (_path->m_buffer_length > col);
              _path->m_p = _path->m_buffer[col];
            }
        }
    }
  return 0;
}

int
get_ID_path_mapping (std::unordered_map<TInt, MNM_Path *> &dict,
                     Path_Table *path_table)
{
  if (path_table != nullptr && !path_table->empty ())
    {
      for (auto _it : *path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  dict[_path->m_path_ID] = _path;
                }
            }
        }
    }
  return 0;
}

MNM_Pathset *
get_pathset (Path_Table *path_table, TInt origin_node_ID, TInt dest_node_ID)
{
  auto iter = path_table->find (origin_node_ID);
  if (iter == path_table->end ())
    {
      throw std::runtime_error ("MNM get_pathset ERROR: no origin node");
    }
  auto iterer = iter->second->find (dest_node_ID);
  if (iterer == iter->second->end ())
    {
      throw std::runtime_error ("MNM get_pathset ERROR: no dest node");
    }
  return iterer->second;
}

} // end namespace MNM
