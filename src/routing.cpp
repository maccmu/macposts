#include "routing.h"

MNM_Routing::MNM_Routing (PNEGraph &graph, MNM_OD_Factory *od_factory,
                          MNM_Node_Factory *node_factory,
                          MNM_Link_Factory *link_factory)
{
  m_graph = graph;
  m_od_factory = od_factory;
  m_node_factory = node_factory;
  m_link_factory = link_factory;
}

MNM_Routing::~MNM_Routing () { ; }

/**************************************************************************
                          Random routing
**************************************************************************/
/* assign each vehicle a random link ahead of it, only used for testing */
MNM_Routing_Random::MNM_Routing_Random (PNEGraph &graph,
                                        MNM_OD_Factory *od_factory,
                                        MNM_Node_Factory *node_factory,
                                        MNM_Link_Factory *link_factory)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
}

MNM_Routing_Random::~MNM_Routing_Random () {}

int
MNM_Routing_Random::init_routing (Path_Table *path_table)
{
  return 0;
}

int
MNM_Routing_Random::update_routing (TInt timestamp)
{
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID;
  TNEGraph::TNodeI _node_I;
  TInt _out_ID;
  MNM_Dlink *_next_link;
  MNM_Dlink *_link;
  // printf("MNM_Routing: route the origin vehciles.\n");
  /* route the vehicle in Origin nodes */
  for (auto _origin_it = m_od_factory->m_origin_map.begin ();
       _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
    {
      _origin = _origin_it->second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh_it = _origin_node->m_in_veh_queue.begin ();
           _veh_it != _origin_node->m_in_veh_queue.end (); _veh_it++)
        {
          _node_I = m_graph->GetNI (_node_ID);
          _out_ID
            = _node_I.GetOutNId (MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
          _next_link = m_link_factory->get_link (
            m_graph->GetEI (_node_ID, _out_ID).GetId ());
          (*_veh_it)->set_next_link (_next_link);
          // note that it neither initializes nor updates _veh -> m_path
        }
    }
  // printf("MNM_Routing: route the link vehciles.\n");
  TInt _link_ID;
  /* route the vehicles at the end of each link */
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      _node_ID = _link->m_to_node->m_node_ID;
      for (auto _veh_it = _link->m_finished_array.begin ();
           _veh_it != _link->m_finished_array.end (); _veh_it++)
        {
          _node_I = m_graph->GetNI (_node_ID);
          if (_node_I.GetOutDeg () > 0)
            {
              _link_ID = _node_I.GetOutEId (
                MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
              _next_link = m_link_factory->get_link (_link_ID);
              (*_veh_it)->set_next_link (_next_link);
            }
          else
            {
              (*_veh_it)->set_next_link (NULL);
            }
        }
    }
  // printf("MNM_Routing: Finished.\n");
  return 0;
}

/**************************************************************************
                          Adaptive routing
**************************************************************************/
MNM_Routing_Adaptive::MNM_Routing_Adaptive (const std::string &file_folder,
                                            PNEGraph &graph,
                                            MNM_Statistics *statistics,
                                            MNM_OD_Factory *od_factory,
                                            MNM_Node_Factory *node_factory,
                                            MNM_Link_Factory *link_factory)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
  m_statistics = statistics;
  m_self_config = new MNM_ConfReader (file_folder + "/config.conf", "ADAPTIVE");
  m_routing_freq = m_self_config->get_int ("route_frq");

  // the unit of m_vot here is different from that of m_vot in DUE (money /
  // interval)
  try
    {
      m_vot = m_self_config->get_float ("vot")
              / 3600.; // money / hour -> money / second
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "vot does not exist in config.conf/ADAPTIVE, use default "
                   "value 20 usd/hour instead\n";
      m_vot = 20. / 3600.; // money / second
    }

  m_table = new Routing_Table ();
  m_link_cost = std::unordered_map<TInt, TFlt> ();
}

MNM_Routing_Adaptive::~MNM_Routing_Adaptive ()
{
  for (auto _it = m_od_factory->m_destination_map.begin ();
       _it != m_od_factory->m_destination_map.end (); _it++)
    {
      if (m_table->find (_it->second) != m_table->end ())
        {
          m_table->find (_it->second)->second->clear ();
          delete m_table->find (_it->second)->second;
        }
    }
  m_table->clear ();
  delete m_table;
  m_link_cost.clear ();
  delete m_self_config;
}

int
MNM_Routing_Adaptive::init_routing (Path_Table *path_table)
{
  if (m_statistics->m_self_config->get_int ("rec_tt") == 0)
    {
      throw std::runtime_error (
        "MNM_Routing_Adaptive::init_routing, rec_tt should be set to 1 in "
        "config.conf to use adaptive routing");
    }
  std::unordered_map<TInt, TInt> *_shortest_path_tree;
  for (auto _it = m_od_factory->m_destination_map.begin ();
       _it != m_od_factory->m_destination_map.end (); _it++)
    {
      _shortest_path_tree = new std::unordered_map<TInt, TInt> ();
      m_table->insert (
        std::pair<MNM_Destination *,
                  std::unordered_map<TInt, TInt> *> (_it->second,
                                                     _shortest_path_tree));
      // for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it !=
      // m_node_factory -> m_node_map.end(); _node_it++){
      //   _shortest_path_tree -> insert(std::pair<TInt, TInt>(_node_it ->
      //   first, -1));
      // }
    }
  for (auto _link_it : m_link_factory->m_link_map)
    {
      m_link_cost.insert (std::pair<TInt, TFlt> (_link_it.first, -1));
    }
  return 0;
}

int
MNM_Routing_Adaptive::update_link_cost ()
{
  for (auto _it : m_statistics->m_record_interval_tt)
    { // seconds
      // for multiclass, m_toll is for car, see
      // MNM_IO_Multiclass::build_link_toll_multiclass
      m_link_cost[_it.first] = _it.second * m_vot + m_link_factory->get_link (_it.first)->m_toll;
      // printf("link %d, cost %f\n", _it.first(), m_link_cost[_it.first]());
    }
  return 0;
}

int
MNM_Routing_Adaptive::update_routing (TInt timestamp)
{
  // relying on m_statistics -> m_record_interval_tt, which is obtained in
  // simulation, not after simulation link::get_link_tt(), based on density
  MNM_Destination *_dest;
  TInt _dest_node_ID;
  std::unordered_map<TInt, TInt> *_shortest_path_tree;
  // update m_table
  if ((timestamp) % m_routing_freq == 0 || timestamp == 0)
    {
      // printf("Calculating the shortest path trees!\n");
      update_link_cost ();
      for (auto _it = m_od_factory->m_destination_map.begin ();
           _it != m_od_factory->m_destination_map.end (); _it++)
        {
          // #pragma omp task firstprivate(_it)
          // {
          _dest = _it->second;
          _dest_node_ID = _dest->m_dest_node->m_node_ID;
          // printf("Destination ID: %d\n", (int) _dest_node_ID);
          _shortest_path_tree = m_table->find (_dest)->second;
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_graph,
                                              m_link_cost,
                                              *_shortest_path_tree);
          // MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_graph,
          // m_statistics -> m_record_interval_tt, *_shortest_path_tree);
          // MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph,
          // m_statistics -> m_record_interval_tt, *_shortest_path_tree);
          // }
        }
    }

  /* route the vehicle in Origin nodes */
  // printf("Routing the vehicle!\n");
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh *_veh;
  for (auto _origin_it = m_od_factory->m_origin_map.begin ();
       _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
    {
      _origin = _origin_it->second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh_it = _origin_node->m_in_veh_queue.begin ();
           _veh_it != _origin_node->m_in_veh_queue.end (); _veh_it++)
        {
          _veh = *_veh_it;
          if (_veh->m_type == MNM_TYPE_ADAPTIVE)
            {
              _next_link_ID = m_table->find (_veh->get_destination ())
                                ->second->find (_node_ID)
                                ->second;
              if (_next_link_ID < 0)
                {
                  throw std::runtime_error ("invalid state");
                }
              // printf("From origin, The next link ID will be %d\n",
              // _next_link_ID());
              _next_link = m_link_factory->get_link (_next_link_ID);
              _veh->set_next_link (_next_link);
              // printf("The next link now it's %d\n", _veh -> get_next_link()
              // -> m_link_ID()); note that it neither initializes nor updates
              // _veh -> m_path
            }
        }
    }

  MNM_Destination *_veh_dest;
  MNM_Dlink *_link;
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      _node_ID = _link->m_to_node->m_node_ID;
      for (auto _veh_it = _link->m_finished_array.begin ();
           _veh_it != _link->m_finished_array.end (); _veh_it++)
        {
          _veh = *_veh_it;
          if (_veh->m_type == MNM_TYPE_ADAPTIVE)
            {
              if (_link != _veh->get_current_link ())
                {
                  throw std::runtime_error ("wrong current link");
                }
              _veh_dest = _veh->get_destination ();
              if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  _veh->set_next_link (nullptr);
                }
              else
                {
                  _next_link_ID = m_table->find (_veh->get_destination ())
                                    ->second->find (_node_ID)
                                    ->second;
                  if (_next_link_ID == -1)
                    {
                      printf (
                        "Something wrong in routing, wrong next link 2\n");
                      printf ("The node is %d, the vehicle should head to %d\n",
                              (int) _node_ID,
                              (int) _veh_dest->m_dest_node->m_node_ID);
                      // exit(-1);
                      auto _node_I = m_graph->GetNI (_node_ID);
                      if (_node_I.GetOutDeg () > 0)
                        {
                          printf ("Assign randomly!\n");
                          _next_link_ID = _node_I.GetOutEId (
                            MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                        }
                      else
                        {
                          printf ("Can't do anything!\n");
                        }
                    }
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  if (_next_link != nullptr)
                    {
                      // printf("Checking future\n");
                      TInt _next_node_ID = _next_link->m_to_node->m_node_ID;
                      if (_next_node_ID
                          != _veh->get_destination ()->m_dest_node->m_node_ID)
                        {
                          // printf("Destination node is %d\n", _veh ->
                          // get_destination() -> m_dest_node -> m_node_ID());
                          if (m_table->find (_veh->get_destination ())
                              == m_table->end ())
                            {
                              printf ("Cannot find Destination\n");
                            }
                          if (m_table->find (_veh->get_destination ())
                                ->second->find (_next_node_ID)
                              == m_table->find (_veh->get_destination ())
                                   ->second->end ())
                            {
                              printf ("Cannot find _next_node_ID\n");
                            }
                          if (m_table->find (_veh->get_destination ())
                                ->second->find (_next_node_ID)
                                ->second
                              == -1)
                            {
                              throw std::runtime_error ("invalid state");
                            }
                        }
                    }
                  _veh->set_next_link (_next_link);
                } // end if else
            }     // end if veh->m_type
        }         // end for veh_it
    }             // end for link_it

  // printf("Finished Routing\n");
  return 0;
}

/**************************************************************************
                          fixed routing
**************************************************************************/
MNM_Routing_Fixed::MNM_Routing_Fixed (PNEGraph &graph,
                                      MNM_OD_Factory *od_factory,
                                      MNM_Node_Factory *node_factory,
                                      MNM_Link_Factory *link_factory,
                                      TInt routing_frq, TInt buffer_len)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
  m_tracker = std::unordered_map<MNM_Veh *, std::deque<TInt> *> ();
  if ((routing_frq == -1) || (buffer_len == -1))
    {
      m_buffer_as_p = false;
      m_routing_freq = -1;
      // m_cur_routing_interval = -1;
    }
  else
    {
      m_routing_freq = routing_frq;
      m_buffer_as_p = true;
      m_buffer_length = buffer_len;
      // m_cur_routing_interval = 0;
    }
  m_path_table = nullptr;
}

MNM_Routing_Fixed::~MNM_Routing_Fixed ()
{
  for (auto _map_it : m_tracker)
    {
      _map_it.second->clear ();
      delete _map_it.second;
    }
  m_tracker.clear ();

  if ((m_path_table != nullptr) && (!m_path_table->empty ()))
    {
      // printf("Address of m_path_table is %p\n", (void *)m_path_table);
      // printf("%d\n", m_path_table -> size());
      for (auto _it : *m_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_path_table->clear ();
      delete m_path_table;
    }
}

int
MNM_Routing_Fixed::init_routing (Path_Table *path_table)
{
  if (path_table == nullptr && m_path_table == nullptr)
    {
      printf (
        "Path table not set, probably it needs to be set in Fixed routing.\n");
      // exit(-1);
    }
  if (path_table != nullptr)
    {
      set_path_table (path_table);
      // printf("path_table set successfully.\n");
    }

  return 0;
}

int
MNM_Routing_Fixed::change_choice_portion (TInt routing_interval)
{
  // printf("Entering MNM_Routing_Fixed::change_choice_portion\n");
  // printf("Current routing interval %d\n", routing_interval);
  MNM::copy_buffer_to_p (m_path_table, routing_interval);
  // printf("Finish copying\n");
  MNM::normalize_path_table_p (m_path_table);
  // printf("Exiting MNM_Routing_Fixed::change_choice_portion\n");
  return 0;
}

int
MNM_Routing_Fixed::update_routing (TInt timestamp)
{
  // printf("MNM_Routing_Fixed::update_routing\n");
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh *_veh;
  TInt _cur_ass_int;

  if (m_buffer_as_p)
    {
      if ((timestamp % m_routing_freq == 0 || timestamp == 0))
        {
          _cur_ass_int = TInt (timestamp / m_routing_freq);
          if (_cur_ass_int < m_buffer_length)
            {
              change_choice_portion (_cur_ass_int);
            }
        }
    }

  for (auto _origin_it = m_od_factory->m_origin_map.begin ();
       _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
    {
      // printf("1.1\n");
      _origin = _origin_it->second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh_it = _origin_node->m_in_veh_queue.begin ();
           _veh_it != _origin_node->m_in_veh_queue.end (); _veh_it++)
        {
          _veh = *_veh_it;
          // printf("1.2\n");
          if (_veh->m_type == MNM_TYPE_STATIC)
            {
              if (m_tracker.find (_veh) == m_tracker.end ())
                { // vehicle not in tracker
                  // printf("Registering!\n");
                  register_veh (_veh, true);
                  // printf("1.3\n");
                  _next_link_ID = m_tracker.find (_veh)->second->front ();
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  _veh->set_next_link (_next_link);
                  m_tracker.find (_veh)
                    ->second->pop_front (); // adjust links left
                }
            }
          // according to Dr. Wei Ma, add a nominal path to adaptive users for
          // DAR extraction, not rigorous, but will do the DODE job
          else if (_veh->m_type == MNM_TYPE_ADAPTIVE)
            {
              if (_veh->m_path == nullptr)
                {
                  register_veh (_veh, false); // adpative user not in m_tracker
                }
              IAssert (_veh->m_path != nullptr);
            }
        }
    }

  // printf("Finished route OD veh\n");
  MNM_Destination *_veh_dest;
  MNM_Dlink *_link;
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      // printf("2.01\n");
      _link = _link_it->second;
      // printf("2.02\n");
      _node_ID = _link->m_to_node->m_node_ID;
      // printf("2.1\n");
      for (auto _veh_it = _link->m_finished_array.begin ();
           _veh_it != _link->m_finished_array.end (); _veh_it++)
        {
          _veh = *_veh_it;
          if (_veh->m_type == MNM_TYPE_STATIC)
            {
              _veh_dest = _veh->get_destination ();
              // printf("2.2\n");
              if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                { // vehicles reaching destination
                  if (m_tracker.find (_veh)->second->size () != 0)
                    { // check if any links left in the route
                      throw std::runtime_error ("invalid state");
                    }
                  _veh->set_next_link (nullptr);
                  // m_tracker.erase(m_tracker.find(_veh));
                }
              else
                { // vehicles enroute, adjust _next_link_ID, which is changed by
                  // node->evolve() in simulation dta.cpp
                  // printf("2.3\n");
                  if (m_tracker.find (_veh) == m_tracker.end ())
                    { // check if vehicle is registered in m_tracker, which
                      // should be done in releasing from origin
                      throw std::runtime_error (
                        "invalid state: vehicle unregistered for link");
                    }
                  if (_veh->get_current_link () == _veh->get_next_link ())
                    {
                      _next_link_ID = m_tracker.find (_veh)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          throw std::runtime_error ("invalid state");
                        }
                      _next_link = m_link_factory->get_link (_next_link_ID);
                      _veh->set_next_link (_next_link);
                      m_tracker.find (_veh)->second->pop_front ();
                    }
                } // end if-else
            }     // end if veh->m_type
        }         // end for veh_it
    }             // end for link_it

  return 0;
}

// register each vehicle with a route based on the portion of path flow
int
MNM_Routing_Fixed::register_veh (MNM_Veh *veh, bool track)
{
  TFlt _r = MNM_Ults::rand_flt ();
  // printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
  // printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);
  MNM_Pathset *_pathset
    = m_path_table->find (veh->get_origin ()->m_origin_node->m_node_ID)
        ->second->find (veh->get_destination ()->m_dest_node->m_node_ID)
        ->second;
  MNM_Path *_route_path = nullptr;
  // printf("1\n");
  // note m_path_vec is an ordered vector, not unordered
  for (MNM_Path *_path : _pathset->m_path_vec)
    {
      // printf("2\n");
      if (_path->m_p >= _r)
        {
          _route_path = _path;
          break;
        }
      else
        {
          _r -= _path->m_p;
        }
    }
  // printf("3\n");
  if (_route_path == nullptr)
    {
      throw std::runtime_error ("wrong probability");
    }
  if (track)
    {
      std::deque<TInt> *_link_queue = new std::deque<TInt> ();
      std::copy (
        _route_path->m_link_vec.begin (), _route_path->m_link_vec.end (),
        std::back_inserter (
          *_link_queue)); // copy links in the route to _link_queue
                          // https://www.cplusplus.com/reference/iterator/back_inserter/
      // printf("old link q is %d, New link queue is %d\n", _route_path ->
      // m_link_vec.size(), _link_queue -> size());
      m_tracker.insert (
        std::pair<MNM_Veh *, std::deque<TInt> *> (veh, _link_queue));
    }
  veh->m_path = _route_path;
  return 0;
}

int
MNM_Routing_Fixed::remove_finished (MNM_Veh *veh, bool del)
{
  IAssert (veh->m_finish_time > 0 && veh->m_finish_time > veh->m_start_time);
  if (m_tracker.find (veh) != m_tracker.end () && del)
    {
      IAssert (veh->m_type
               == MNM_TYPE_STATIC); // adaptive user not in m_tracker
      m_tracker.find (veh)->second->clear ();
      delete m_tracker.find (veh)->second;
      m_tracker.erase (veh);
    }
  return 0;
}

int
MNM_Routing_Fixed::set_path_table (Path_Table *path_table)
{
  if (m_path_table != nullptr)
    delete m_path_table;
  m_path_table = path_table;
  return 0;
}

int
MNM_Routing_Fixed::add_veh_path (MNM_Veh *veh, std::deque<TInt> *link_que)
{
  std::deque<TInt> *_new_link_que = new std::deque<TInt> ();
  std::copy (link_que->begin (), link_que->end (),
             std::back_inserter (*_new_link_que));
  m_tracker.insert (
    std::pair<MNM_Veh *, std::deque<TInt> *> (veh, _new_link_que));
  return 0;
}

/**************************************************************************
                          Hybrid (Adaptive+Fixed) routing
**************************************************************************/
MNM_Routing_Hybrid::MNM_Routing_Hybrid (
  const std::string &file_folder, PNEGraph &graph, MNM_Statistics *statistics,
  MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
  MNM_Link_Factory *link_factory, TInt route_frq_fixed, TInt buffer_len)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
  m_routing_fixed
    = new MNM_Routing_Fixed (graph, od_factory, node_factory, link_factory,
                             route_frq_fixed, buffer_len);

  m_routing_adaptive
    = new MNM_Routing_Adaptive (file_folder, graph, statistics, od_factory,
                                node_factory, link_factory);
  auto *_tmp_config = new MNM_ConfReader (file_folder + "/config.conf", "DTA");
  m_routing_adaptive->m_working = _tmp_config->get_float ("adaptive_ratio") > 0;
  delete _tmp_config;
}

MNM_Routing_Hybrid::~MNM_Routing_Hybrid ()
{
  delete m_routing_adaptive;
  delete m_routing_fixed;
}

int
MNM_Routing_Hybrid::init_routing (Path_Table *path_table)
{
  if (m_routing_adaptive->m_working)
    m_routing_adaptive->init_routing ();
  m_routing_fixed->init_routing (path_table);
  return 0;
}

int
MNM_Routing_Hybrid::update_routing (TInt timestamp)
{
  if (m_routing_adaptive->m_working)
    m_routing_adaptive->update_routing (timestamp);
  m_routing_fixed->update_routing (timestamp);
  return 0;
}

int
MNM_Routing_Hybrid::remove_finished (MNM_Veh *veh, bool del)
{
  m_routing_fixed->remove_finished (veh, del);
  return 0;
}

/**************************************************************************
                          Bi-class Hybrid routing
**************************************************************************/
MNM_Routing_Biclass_Hybrid::MNM_Routing_Biclass_Hybrid (
  const std::string &file_folder, PNEGraph &graph, MNM_Statistics *statistics,
  MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
  MNM_Link_Factory *link_factory, TInt route_frq_fixed, TInt buffer_length)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
  m_routing_fixed_car
    = new MNM_Routing_Biclass_Fixed (graph, od_factory, node_factory,
                                     link_factory, route_frq_fixed,
                                     buffer_length, TInt (0));
  m_routing_fixed_truck
    = new MNM_Routing_Biclass_Fixed (graph, od_factory, node_factory,
                                     link_factory, route_frq_fixed,
                                     buffer_length, TInt (1));

  m_routing_adaptive
    = new MNM_Routing_Adaptive (file_folder, graph, statistics, od_factory,
                                node_factory, link_factory);
  auto *_tmp_config = new MNM_ConfReader (file_folder + "/config.conf", "DTA");
  m_routing_adaptive->m_working
    = _tmp_config->get_float ("adaptive_ratio_car") > 0
      || _tmp_config->get_float ("adaptive_ratio_truck") > 0;
  delete _tmp_config;
}

MNM_Routing_Biclass_Hybrid::~MNM_Routing_Biclass_Hybrid ()
{
  delete m_routing_adaptive;
  // printf("m_routing_adaptive\n");
  delete m_routing_fixed_car;
  // printf("m_routing_fixed_car\n");
  delete m_routing_fixed_truck;
  // printf("m_routing_fixed_truck\n");
}

int
MNM_Routing_Biclass_Hybrid::init_routing (Path_Table *path_table)
{
  if (m_routing_adaptive->m_working)
    m_routing_adaptive->init_routing ();
  // printf("Finished init all ADAPTIVE vehicles routing\n");
  m_routing_fixed_car->init_routing (path_table);
  // printf("Finished init STATIC cars routing\n");
  m_routing_fixed_truck->init_routing (path_table);
  // printf("Finished init STATIC trucks routing\n");
  return 0;
}

int
MNM_Routing_Biclass_Hybrid::update_routing (TInt timestamp)
{
  if (m_routing_adaptive->m_working)
    m_routing_adaptive->update_routing (timestamp);
  // printf("Finished update all ADAPTIVE vehicles routing\n");
  m_routing_fixed_car->update_routing (timestamp);
  // printf("Finished update STATIC cars routing\n");
  m_routing_fixed_truck->update_routing (timestamp);
  // printf("Finished update STATIC trucks routing\n");
  return 0;
}

int
MNM_Routing_Biclass_Hybrid::remove_finished (MNM_Veh *veh, bool del)
{
  m_routing_fixed_car->remove_finished (veh, del);
  m_routing_fixed_truck->remove_finished (veh, del);
  return 0;
}

/**************************************************************************
                          Bi-class fixed routing
**************************************************************************/
MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed (
  PNEGraph &graph, MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
  MNM_Link_Factory *link_factory, TInt routing_frq, TInt buffer_length,
  TInt veh_class)
    : MNM_Routing_Fixed::MNM_Routing_Fixed (graph, od_factory, node_factory,
                                            link_factory, routing_frq,
                                            buffer_length)
{
  m_buffer_length = buffer_length;
  m_veh_class = veh_class;
}

MNM_Routing_Biclass_Fixed::~MNM_Routing_Biclass_Fixed () { ; }

int
MNM_Routing_Biclass_Fixed::change_choice_portion (TInt routing_interval)
{
  // m_veh_class starts from 0 (car) to 1 (truck)
  MNM::copy_buffer_to_p (m_path_table,
                         routing_interval
                           + m_veh_class * TInt (m_buffer_length / 2));
  MNM::normalize_path_table_p (m_path_table);
  return 0;
}

int
MNM_Routing_Biclass_Fixed::update_routing (TInt timestamp)
{
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh *_veh;
  TInt _cur_ass_int;

  // printf("my buffer length %d, %d\n", m_buffer_length(), m_routing_freq());
  if (m_buffer_as_p)
    {
      // if (timestamp % m_routing_freq  == 0 || timestamp == 0) {
      // printf("11\n");
      _cur_ass_int = TInt (timestamp / m_routing_freq);
      if (_cur_ass_int < TInt (m_buffer_length / 2))
        { // first half for car, last half for truck
          // printf("Changing biclass fixed\n");
          change_choice_portion (_cur_ass_int);
        }
      // }
    }

  for (auto _origin_it = m_od_factory->m_origin_map.begin ();
       _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
    {
      _origin = _origin_it->second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh_it = _origin_node->m_in_veh_queue.begin ();
           _veh_it != _origin_node->m_in_veh_queue.end (); _veh_it++)
        {
          _veh = *_veh_it;

          // Here is the difference from single-class fixed routing
          //      TInt tmp = _veh -> m_class; // m_class of base veh
          //      TInt tmp2 = _veh -> get_class(); // virtual getter of derived
          //      veh_multiclass TInt tmp3 = _veh -> m_bus_route_ID; TInt tmp4 =
          //      _veh -> get_bus_route_ID();

          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_bus_route_ID () == TInt (-1))
              && (!_veh->get_ispnr ()))
            {
              // Here is the difference from single-class fixed routing

              if (m_tracker.find (_veh) == m_tracker.end ())
                {
                  // printf("Registering!\n");
                  register_veh (_veh, true);
                  _next_link_ID = m_tracker.find (_veh)->second->front ();
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  _veh->set_next_link (_next_link);
                  m_tracker.find (_veh)->second->pop_front ();
                }
            }
          // according to Dr. Wei Ma, add a nominal path to adaptive users for
          // DAR extraction, not rigorous, but will do the DODE job
          else if ((_veh->m_type == MNM_TYPE_ADAPTIVE)
                   && (_veh->get_class () == m_veh_class)
                   && (_veh->get_bus_route_ID () == TInt (-1))
                   && (!_veh->get_ispnr ()))
            {
              if (_veh->m_path == nullptr)
                {
                  register_veh (_veh, false);
                }
              IAssert (_veh->m_path != nullptr);
            }
        }
    }
  // printf("Finished route OD veh\n");

  MNM_Destination *_veh_dest;
  MNM_Dlink *_link;
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      _node_ID = _link->m_to_node->m_node_ID;
      for (auto _veh_it = _link->m_finished_array.begin ();
           _veh_it != _link->m_finished_array.end (); _veh_it++)
        {
          _veh = *_veh_it;

          // Here is the difference from single-class fixed routing
          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_bus_route_ID () == TInt (-1))
              && (!_veh->get_ispnr ()))
            {
              // Here is the difference from single-class fixed routing

              _veh_dest = _veh->get_destination ();
              if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  if (m_tracker.find (_veh)->second->size () != 0)
                    {
                      throw std::runtime_error ("invalid state");
                    }
                  _veh->set_next_link (nullptr);
                }
              else
                {
                  if (m_tracker.find (_veh) == m_tracker.end ())
                    {
                      throw std::runtime_error (
                        "invalid state: vehicle unregistered for link");
                    }
                  if (_veh->get_current_link () == _veh->get_next_link ())
                    {
                      _next_link_ID = m_tracker.find (_veh)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          throw std::runtime_error ("invalid state");
                        }
                      _next_link = m_link_factory->get_link (_next_link_ID);
                      _veh->set_next_link (_next_link);
                      m_tracker.find (_veh)->second->pop_front ();
                    }
                } // end if-else
            }     // end if veh->m_type
        }         // end for veh_it
    }             // end for link_it

  return 0;
}

int
MNM_Routing_Biclass_Fixed::remove_finished (MNM_Veh *veh, bool del)
{
  if (veh->get_class () == m_veh_class)
    {
      MNM_Routing_Fixed::remove_finished (veh, del);
    }
  return 0;
}
