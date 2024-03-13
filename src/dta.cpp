#include "dta.h"

using macposts::graph::Direction;

MNM_Dta::MNM_Dta (const std::string &file_folder)
{
  m_file_folder = file_folder;
  m_current_loading_interval = TInt (0);
  m_emission = nullptr;
  m_routing = nullptr;
  m_statistics = nullptr;
  m_gridlock_recorder = nullptr;
  m_workzone = nullptr;
  m_veh_factory = nullptr;
  m_node_factory = nullptr;
  m_link_factory = nullptr;
  m_od_factory = nullptr;
  m_config = nullptr;
  m_queue_veh_num = std::deque<TInt> ();
  m_enroute_veh_num = std::deque<TInt> ();
  m_queue_veh_map = std::unordered_map<TInt, std::deque<TInt> *> ();

  initialize ();
}

MNM_Dta::~MNM_Dta ()
{
  if (m_emission != nullptr)
    delete m_emission;
  // printf("m_emission\n");

  if (m_routing != nullptr)
    delete m_routing;
  // printf("m_routing\n");

  if (m_veh_factory != nullptr)
    delete m_veh_factory;
  // printf("m_veh_factory\n");
  if (m_node_factory != nullptr)
    delete m_node_factory;
  // printf("m_node_factory\n");
  if (m_link_factory != nullptr)
    delete m_link_factory;
  // printf("m_link_factory\n");
  if (m_od_factory != nullptr)
    delete m_od_factory;
  // printf("m_od_factory\n");
  if (m_config != nullptr)
    delete m_config;
  // printf("m_config\n");

  if (m_statistics != nullptr)
    delete m_statistics;
  // printf("m_statistics\n");
  if (m_gridlock_recorder != nullptr)
    delete m_gridlock_recorder;
  if (m_workzone != nullptr)
    delete m_workzone;
  // printf("m_workzone\n");

  // printf("3\n");
  m_queue_veh_num.clear ();
  m_enroute_veh_num.clear ();
  for (auto _it = m_queue_veh_map.begin (); _it != m_queue_veh_map.end ();
       _it++)
    {
      _it->second->clear ();
      delete _it->second;
    }
  m_queue_veh_map.clear ();
  // printf("4\n");
}

int
MNM_Dta::initialize ()
{
  m_veh_factory = new MNM_Veh_Factory ();
  m_node_factory = new MNM_Node_Factory ();
  m_link_factory = new MNM_Link_Factory ();
  m_od_factory = new MNM_OD_Factory ();
  m_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DTA");
  m_unit_time = m_config->get_int ("unit_time");
  m_flow_scalar = m_config->get_int ("flow_scalar");
  TInt _ev_label;
  try
    {
      _ev_label = m_config->get_int ("ev_label");
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "ev_label does not exist in config.conf/DTA, use default "
                   "value -2 instead\n";
      _ev_label = -2;
    }
  m_emission = new MNM_Cumulative_Emission (TFlt (m_unit_time), 0, _ev_label);
  // note the difference in m_assign_freq and m_total_assign_inter between
  // MNM_Dta and MNM_Dta_Multiclass and MNM_Dta_Multimodal
  m_assign_freq = m_config->get_int ("assign_frq");
  m_start_assign_interval = m_config->get_int ("start_assign_interval");
  m_total_assign_inter = m_config->get_int ("max_interval");
  m_init_demand_split = m_config->get_int ("init_demand_split");
  return 0;
}

int
MNM_Dta::set_statistics ()
{
  MNM_ConfReader *_record_config
    = new MNM_ConfReader (m_file_folder + "/config.conf", "STAT");
  if (_record_config->get_string ("rec_mode") == "LRn")
    {
      m_statistics
        = new MNM_Statistics_Lrn (m_file_folder, m_config, _record_config,
                                  m_od_factory, m_node_factory, m_link_factory);
    }
  // printf("set_statistics finished\n");
  return 0;
}

int
MNM_Dta::set_gridlock_recorder ()
{
  MNM_ConfReader *_record_config
    = new MNM_ConfReader (m_file_folder + "/config.conf", "STAT");
  TInt _rec_gridlock;
  try
    {
      _rec_gridlock = _record_config->get_int ("rec_gridlock");
    }
  catch (const std::invalid_argument &ia)
    {
      _rec_gridlock = 0;
    }
  if (_rec_gridlock > 0)
    {
      m_gridlock_recorder
        = new MNM_Gridlock_Link_Recorder (m_file_folder, _record_config);
    }
  return 0;
}

int
MNM_Dta::set_routing ()
{
  if (m_config->get_string ("routing_type") == "Adaptive")
    {
      m_routing = new MNM_Routing_Adaptive (m_file_folder, m_graph,
                                            m_statistics, m_od_factory,
                                            m_node_factory, m_link_factory);
      m_routing->init_routing ();
    }

  else if (m_config->get_string ("routing_type") == "Predetermined")
    {
      // single class
      Path_Table *_path_table
        = MNM::build_pathset (m_graph, m_od_factory, m_link_factory);
      MNM_Pre_Routing *_pre_routing
        = new MNM_Pre_Routing (_path_table, m_od_factory);
      m_routing
        = new MNM_Routing_Predetermined (m_graph, m_od_factory, m_node_factory,
                                         m_link_factory, _path_table,
                                         _pre_routing, m_total_assign_inter);
    }

  else if (m_config->get_string ("routing_type") == "Fixed")
    {
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      Path_Table *_path_table;
      if (_tmp_conf->get_string ("choice_portion") == "Buffer")
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       true);
        }
      else
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       false);
        }
      TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
      IAssert (_buffer_len == m_config->get_int ("max_interval"));
      m_routing
        = new MNM_Routing_Fixed (m_graph, m_od_factory, m_node_factory,
                                 m_link_factory,
                                 _tmp_conf->get_int ("route_frq"), _buffer_len);
      m_routing->init_routing (_path_table);
      delete _tmp_conf;
    }

  else if (m_config->get_string ("routing_type") == "Due")
    {
      // Path_Table *_path_table = MNM::build_pathset(m_graph, m_od_factory,
      // m_link_factory);
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      // no existing path_table, use column generation
      m_routing = new MNM_Routing_Fixed (m_graph, m_od_factory, m_node_factory,
                                         m_link_factory,
                                         m_config->get_int ("assign_frq"),
                                         _tmp_conf->get_int ("buffer_length"));
      // m_routing = new MNM_Routing_Fixed(m_graph, m_od_factory,
      // m_node_factory, m_link_factory, m_config -> get_int("assign_frq"));
      // m_routing -> init_routing(_path_table);
      delete _tmp_conf;
    }

  // m_routing = new MNM_Routing_Random(m_graph, m_statistics, m_od_factory,
  // m_node_factory, m_link_factory);

  else if (m_config->get_string ("routing_type") == "Hybrid")
    {
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      Path_Table *_path_table;
      if (_tmp_conf->get_string ("choice_portion") == "Buffer")
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       true);
        }
      else
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       false);
        }
      TInt _route_freq_fixed = _tmp_conf->get_int ("route_frq");
      TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
      IAssert (_buffer_len == m_config->get_int ("max_interval"));
      m_routing
        = new MNM_Routing_Hybrid (m_file_folder, m_graph, m_statistics,
                                  m_od_factory, m_node_factory, m_link_factory,
                                  _route_freq_fixed, _buffer_len);
      m_routing->init_routing (_path_table);
      delete _tmp_conf;
    }

  // Hybrid routing for bi-class vehicles.
  // Note here still only one path_table and buffer, but in buffer file each row
  // contains: Row #k : [probabilities choosing route k in all intervals for
  // cars] [probabilities choosing route k in all intervals for trucks] For
  // Bi-class Fixed routing, just set both adaptive_ratio_car=0 &
  // adaptive_ratio_truck=0 in "config.conf"
  else if (m_config->get_string ("routing_type") == "Biclass_Hybrid"
           || m_config->get_string ("routing_type")
                == "Biclass_Hybrid_ColumnGeneration")
    {
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      Path_Table *_path_table;
      if (_tmp_conf->get_string ("choice_portion") == "Buffer")
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       true);
        }
      else
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       false);
        }
      TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
      // for bi-class problem
      if (_buffer_len < 2 * m_config->get_int ("max_interval"))
        {
          _buffer_len = 2 * m_config->get_int ("max_interval");
        }
      TInt _route_freq_fixed = _tmp_conf->get_int ("route_frq");
      m_routing
        = new MNM_Routing_Biclass_Hybrid (m_file_folder, m_graph, m_statistics,
                                          m_od_factory, m_node_factory,
                                          m_link_factory, _route_freq_fixed,
                                          _buffer_len);
      m_routing->init_routing (_path_table);
      delete _tmp_conf;
    }

  else
    {
      m_routing = new MNM_Routing_Random (m_graph, m_od_factory, m_node_factory,
                                          m_link_factory);
      m_routing->init_routing ();
    }
  return 0;
}

int
MNM_Dta::build_workzone ()
{
  // TODO: Disabled for the new builtin `macposts::Graph' class because it does
  // not support removing a link yet.
  //
  // m_workzone = new MNM_Workzone (m_node_factory, m_link_factory, m_graph);
  // MNM_IO::build_workzone_list (m_file_folder, m_workzone);
  return 0;
}

int
MNM_Dta::build_from_files ()
{
  MNM_IO::build_node_factory (m_file_folder, m_config, m_node_factory);
  std::cout << "# of nodes: " << m_node_factory->m_node_map.size () << "\n";
  MNM_IO::build_link_factory (m_file_folder, m_config, m_link_factory);
  std::cout << "# of links: " << m_link_factory->m_link_map.size () << "\n";
  MNM_IO::build_od_factory (m_file_folder, m_config, m_od_factory,
                            m_node_factory);
  std::cout << "# of OD pairs: " << m_od_factory->m_origin_map.size () << "\n";
  // std::cout << m_od_factory -> m_destination_map.size() << "\n";
  m_graph = MNM_IO::build_graph (m_file_folder, m_config, 0);
  MNM_IO::build_demand (m_file_folder, m_config, m_od_factory);
  MNM_IO::read_origin_vehicle_label_ratio (m_file_folder, m_config,
                                           m_od_factory);
  MNM_IO::build_link_toll (m_file_folder, m_config, m_link_factory);
  build_workzone ();
  set_statistics ();
  set_gridlock_recorder ();
  printf ("Start building routing\n");
  set_routing ();
  printf ("Finish building routing\n");
  return 0;
}

int
MNM_Dta::hook_up_node_and_link ()
{
  TInt _node_ID;
  MNM_Dnode *_node;
  MNM_Dlink *_link;

  // hook up link to node
  for (const auto &n : m_graph.nodes ())
    {
      _node_ID = m_graph.get_id (n);
      _node = m_node_factory->get_node (_node_ID);
      for (auto const &l : m_graph.connections (n, Direction::Outgoing))
        {
          _link = m_link_factory->get_link (m_graph.get_id (l));
          _node->add_out_link (_link);
        }
      for (auto const &l : m_graph.connections (n, Direction::Incoming))
        {
          _link = m_link_factory->get_link (m_graph.get_id (l));
          _node->add_in_link (_link);
        }
    }

  // hook up node to link
  for (const auto &l : m_graph.links ())
    {
      _link = m_link_factory->get_link (m_graph.get_id (l));
      const auto &sd = m_graph.get_endpoints (l);
      _link->hook_up_node (m_node_factory->get_node (m_graph.get_id (sd.first)),
                           m_node_factory->get_node (
                             m_graph.get_id (sd.second)));
    }
  return 0;
}

/*
 *  perform a series of checks to ensure we can run the DTA model
 */
bool
MNM_Dta::is_ok ()
{
  bool _flag = true;
  bool _temp_flag = true;

  // TODO: Port this to the builtin graph (if necessary).
  //
  // Checks the graph data structure for internal consistency.
  // For each node in the graph check that its neighbors are also nodes in the
  // graph.
  // printf ("\nChecking......Driving Graph consistent!\n");
  // _temp_flag = m_graph->IsOk ();
  // _flag = _flag && _temp_flag;
  // if (_temp_flag)
  //   printf ("Passed!\n");

  // check node
  printf ("Checking......Driving Node consistent!\n");
  _temp_flag
    = (m_graph.size_nodes () == m_config->get_int ("num_of_node"))
      && (m_graph.size_nodes () == TInt (m_node_factory->m_node_map.size ()));
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  // check link
  printf ("Checking......Driving Link consistent!\n");
  _temp_flag
    = (m_graph.size_links () == m_config->get_int ("num_of_link"))
      && (m_graph.size_links () == TInt (m_link_factory->m_link_map.size ()));
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  // check OD node
  printf ("Checking......OD consistent!\n");
  TInt _node_ID;
  _temp_flag = (TInt (m_od_factory->m_origin_map.size ())
                == m_config->get_int ("num_of_O"))
               && (TInt (m_od_factory->m_destination_map.size ())
                   == m_config->get_int ("num_of_D"));
  std::unordered_map<TInt, MNM_Origin *>::iterator _origin_map_it;
  for (_origin_map_it = m_od_factory->m_origin_map.begin ();
       _origin_map_it != m_od_factory->m_origin_map.end (); _origin_map_it++)
    {
      _node_ID = _origin_map_it->second->m_origin_node->m_node_ID;
      const auto &n = m_graph.get_node (_node_ID);
      _temp_flag = _temp_flag && (m_graph.get_id (n) == _node_ID);
      auto &&outs = m_graph.connections (n, Direction::Outgoing);
      _temp_flag
        = _temp_flag && (std::distance (outs.begin (), outs.end ()) >= 1);
      auto &&ins = m_graph.connections (n, Direction::Incoming);
      _temp_flag
        = _temp_flag && (std::distance (ins.begin (), ins.end ()) == 0);
    }
  std::unordered_map<TInt, MNM_Destination *>::iterator _dest_map_it;
  for (_dest_map_it = m_od_factory->m_destination_map.begin ();
       _dest_map_it != m_od_factory->m_destination_map.end (); _dest_map_it++)
    {
      _node_ID = _dest_map_it->second->m_dest_node->m_node_ID;
      const auto &n = m_graph.get_node (_node_ID);
      _temp_flag = _temp_flag && (m_graph.get_id (n) == _node_ID);
      auto &&outs = m_graph.connections (n, Direction::Outgoing);
      _temp_flag
        = _temp_flag && (std::distance (outs.begin (), outs.end ()) == 0);
      auto &&ins = m_graph.connections (n, Direction::Incoming);
      _temp_flag
        = _temp_flag && (std::distance (ins.begin (), ins.end ()) >= 1);
    }
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  printf ("Checking......OD connectivity!\n");
  _temp_flag = check_origin_destination_connectivity ();
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  return _flag;
}

int
MNM_Dta::check_origin_destination_connectivity ()
{
  MNM_Destination *_dest;
  TInt _dest_node_ID;
  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _cost_map;
  for (auto _map_it : m_link_factory->m_link_map)
    {
      _cost_map.insert (std::pair<TInt, TFlt> (_map_it.first, TFlt (1)));
    }

  for (auto _it = m_od_factory->m_destination_map.begin ();
       _it != m_od_factory->m_destination_map.end (); _it++)
    {
      _dest = _it->second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_graph, _cost_map,
                                          _shortest_path_tree);
      for (auto _map_it : m_od_factory->m_origin_map)
        {
          if (_shortest_path_tree
                .find (_map_it.second->m_origin_node->m_node_ID)
                ->second
              == -1)
            {
              return false;
            }
        }
    }
  return true;
}

int
MNM_Dta::pre_loading ()
{
  MNM_Dnode *_node;
  // printf("MNM: Prepare loading!\n");
  m_statistics->init_record ();
  if (m_gridlock_recorder != nullptr)
    m_gridlock_recorder->init_record ();
  for (auto _node_it = m_node_factory->m_node_map.begin ();
       _node_it != m_node_factory->m_node_map.end (); _node_it++)
    {
      _node = _node_it->second;
      // printf("Node ID: %d\n", _node -> m_node_ID);
      _node->prepare_loading ();
    }
  // printf("dsf\n");
  // m_workzone -> init_workzone();

  // https://stackoverflow.com/questions/7443787/using-c-ifstream-extraction-operator-to-read-formatted-data-from-a-file
  std::ifstream _emission_file (m_file_folder + "/MNM_input_emission_linkID");
  int _link_ID;
  std::unordered_map<int, int> _emission_links = {};
  while (_emission_file >> _link_ID)
    {
      _emission_links.insert ({ _link_ID, 0 });
    }
  _emission_file.close ();

  std::deque<TInt> *_rec;
  for (auto _map_it : m_link_factory->m_link_map)
    {
      _rec = new std::deque<TInt> ();
      m_queue_veh_map.insert ({ _map_it.second->m_link_ID, _rec });
      if (_emission_links.find (int (_map_it.second->m_link_ID))
          != _emission_links.end ())
        m_emission->register_link (_map_it.second);
    }

  // printf("Exiting MNM: Prepare loading!\n");
  return 0;
}

int
MNM_Dta::load_once (bool verbose, TInt load_int, TInt assign_int)
{
  MNM_Origin *_origin;
  MNM_Dnode *_node;
  MNM_Dlink *_link;
  MNM_Destination *_dest;

  // update some link attributes over time
  m_link_factory->update_link_attribute (load_int, verbose);
  // compute empty network link tt, for adaptive routing
  if (load_int == 0)
    m_statistics->update_record (-1);
  if (verbose)
    printf ("-------------------------------    Interval %d   "
            "------------------------------ \n",
            (int) load_int);
  // step 1: Origin release vehicle
  if (verbose)
    printf ("Releasing!\n");

  if (load_int % m_assign_freq == 0 || load_int == 0)
    {
      for (auto _origin_it = m_od_factory->m_origin_map.begin ();
           _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
        {
          _origin = _origin_it->second;
          if (assign_int >= m_total_assign_inter)
            {
              _origin->release_one_interval (load_int, m_veh_factory, -1,
                                             TFlt (-1));
            }
          else
            {
              if (m_config->get_string ("routing_type") == "Fixed"
                  || m_config->get_string ("routing_type") == "Due")
                {
                  // printf("Fixed Releasing.\n");
                  _origin->release_one_interval (load_int, m_veh_factory,
                                                 assign_int, TFlt (0));
                }
              else if ((m_config->get_string ("routing_type") == "Adaptive"))
                {
                  _origin->release_one_interval (load_int, m_veh_factory,
                                                 assign_int, TFlt (1));
                }
              else if ((m_config->get_string ("routing_type") == "Hybrid"))
                {
                  TFlt _ad_ratio = m_config->get_float ("adaptive_ratio");
                  if (_ad_ratio > 1)
                    _ad_ratio = 1;
                  if (_ad_ratio < 0)
                    _ad_ratio = 0;
                  _origin->release_one_interval (load_int, m_veh_factory,
                                                 assign_int, _ad_ratio);
                }
              else if ((m_config->get_string ("routing_type")
                        == "Biclass_Hybrid"))
                {
                  TFlt _ad_ratio_car
                    = m_config->get_float ("adaptive_ratio_car");
                  if (_ad_ratio_car > 1)
                    _ad_ratio_car = 1;
                  if (_ad_ratio_car < 0)
                    _ad_ratio_car = 0;

                  TFlt _ad_ratio_truck
                    = m_config->get_float ("adaptive_ratio_truck");
                  if (_ad_ratio_truck > 1)
                    _ad_ratio_truck = 1;
                  if (_ad_ratio_truck < 0)
                    _ad_ratio_truck = 0;
                  // NOTE: in this case the release function is different
                  _origin->release_one_interval_biclass (load_int,
                                                         m_veh_factory,
                                                         assign_int,
                                                         _ad_ratio_car,
                                                         _ad_ratio_truck);
                }
              else
                {
                  printf ("WARNING:No assignment!\n");
                }
            }
        }
    }

  if (verbose)
    printf ("Routing!\n");
  // step 2: route the vehicle
  m_routing->update_routing (load_int);

  if (verbose)
    printf ("Moving through node!\n");
  // step 3: move vehicles through node
  for (auto _node_it = m_node_factory->m_node_map.begin ();
       _node_it != m_node_factory->m_node_map.end (); _node_it++)
    {
      _node = _node_it->second;
      // printf("node ID is %d\n", _node -> m_node_ID());
      _node->evolve (load_int);
    }

  // record queuing vehicles after node evolve, which is num of vehicles in
  // finished array
  record_queue_vehicles ();
  if (verbose)
    printf ("Moving through link!\n");
  // step 4: move vehicles through link
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      if ((m_gridlock_recorder != nullptr)
          && ((m_config->get_int ("total_interval") <= 0
               && load_int >= 1.5 * m_total_assign_inter * m_assign_freq)
              || (m_config->get_int ("total_interval") > 0
                  && load_int >= 0.95 * m_config->get_int ("total_interval"))))
        {
          m_gridlock_recorder->save_one_link (load_int, _link);
        }
      _link->clear_incoming_array (load_int);
      _link->evolve (load_int);
    }

  if (m_emission != nullptr)
    m_emission->update (m_veh_factory);

  if (verbose)
    printf ("Receiving!\n");
  // step 5: Destination receive vehicle
  for (auto _dest_it = m_od_factory->m_destination_map.begin ();
       _dest_it != m_od_factory->m_destination_map.end (); _dest_it++)
    {
      _dest = _dest_it->second;
      // _dest -> receive(load_int);
      _dest->receive (load_int, m_routing, m_veh_factory, true);
    }

  if (verbose)
    printf ("Update record!\n");
  // step 5: update record
  m_statistics->update_record (load_int);

  record_enroute_vehicles ();
  if (verbose)
    MNM::print_vehicle_statistics (m_veh_factory);
  // test();
  // consistent with loading()
  m_current_loading_interval = load_int + 1;
  return 0;
}

// verbose: whether to print
int
MNM_Dta::loading (bool verbose)
{
  int _current_inter = 0;
  int _assign_inter = m_start_assign_interval;

  // It at least will release all vehicles no matter what value total_interval
  // is set the least length of simulation = max_interval * assign_frq
  while (!finished_loading (_current_inter)
         || _assign_inter < m_total_assign_inter)
    {
      if (verbose)
        {
          std::cout << std::endl
                    << "Current loading interval: " << _current_inter << ", "
                    << "Current assignment interval: "
                    << int (_current_inter / m_config->get_int ("assign_frq"))
                    << std::endl;
        }
      load_once (verbose, _current_inter, _assign_inter);
      // link cc will be updated with the record at the end of this interval
      // (i.e., _current_inter + 1)
      if (++_current_inter % m_assign_freq == 0)
        {
          ++_assign_inter;
        }
    }
  if (verbose)
    {
      MNM::print_vehicle_statistics (m_veh_factory);
    }
  m_statistics->post_record ();
  if (m_gridlock_recorder != nullptr)
    m_gridlock_recorder->post_record ();
  m_current_loading_interval = _current_inter;
  return _current_inter; // total number of actual loading intervals =
                         // _current_inter
}

int
MNM_Dta::record_queue_vehicles ()
{
  TInt _tot_queue_size = 0;
  for (auto _map_it : m_link_factory->m_link_map)
    {
      TInt _queue_size = _map_it.second->m_finished_array.size ();
      if (_map_it.second->m_link_type != MNM_TYPE_PQ)
        { // PQ not included
          _tot_queue_size += _queue_size;
        }
      // Original
      // if (_map_it.second -> m_ffs == 0.0) _tot_queue_size += _queue_size;
      m_queue_veh_map[_map_it.second->m_link_ID]->push_back (_queue_size);
    }
  m_queue_veh_num.push_back (_tot_queue_size);
  return 0;
}

int
MNM_Dta::record_enroute_vehicles ()
{
  // TInt _total_veh = TInt(m_veh_factory -> m_veh_map.size());
  // TInt _finished_veh = 0;
  // TInt _enroute_veh;
  // for (auto _map_it : m_veh_factory -> m_veh_map){
  //   if (_map_it.second -> m_finish_time > 0) _finished_veh += 1;
  // }
  // _enroute_veh = _total_veh - _finished_veh;
  // m_enroute_veh_num.push_back(_enroute_veh);

  m_enroute_veh_num.push_back (m_veh_factory->m_enroute);
  return 0;
}

bool
MNM_Dta::finished_loading (int cur_int)
{
  // printf("Entering MNM_Dta::finished_loading\n");
  TInt _total_int = m_config->get_int ("total_interval");
  if (_total_int > 0)
    {
      // printf("Exiting MNM_Dta::finished_loading 1\n");
      return cur_int >= _total_int;
    }
  else
    {
      // printf("Exiting MNM_Dta::finished_loading 2\n");
      return !(MNM::has_running_vehicle (m_veh_factory) || cur_int == 0);
    }
}

namespace MNM
{
int
print_vehicle_statistics (MNM_Veh_Factory *veh_factory)
{
  // TInt _total_veh = TInt(veh_factory -> m_veh_map.size());
  // TInt _finished_veh = 0;
  // TInt _enroute_veh;
  // for (auto _map_it : veh_factory -> m_veh_map){
  //   if (_map_it.second -> m_finish_time > 0) _finished_veh += 1;
  // }
  // _enroute_veh = _total_veh - _finished_veh;
  // printf("Released vehicle %d, Enroute vehicle %d, Finished vehicle %d\n",
  // _total_veh(), _enroute_veh(), _finished_veh());

  printf (
    "############################################### Vehicle Statistics ###############################################\n \
  Released Vehicle total %d, Enroute Vehicle Total %d, Finished Vehicle Total %d\n \
  Total Travel Time: %.2f intervals\n \
  ############################################### Vehicle Statistics ###############################################\n",
    veh_factory->m_num_veh (), veh_factory->m_enroute (),
    veh_factory->m_finished (), veh_factory->m_total_time ());
  return 0;
}

int
print_vehicle_info (MNM_Veh_Factory *veh_factory)
{
  MNM_Veh *_veh;
  for (auto _map_it : veh_factory->m_veh_map)
    {
      _veh = _map_it.second;
      printf ("Vehicle ID %d, from origin node %d to destination node %d, is "
              "current on link %d and heads to link %d\n",
              _veh->m_veh_ID (),
              _veh->get_origin ()->m_origin_node->m_node_ID (),
              _veh->get_destination ()->m_dest_node->m_node_ID (),
              _veh->get_current_link ()->m_link_ID (),
              _veh->get_next_link ()->m_link_ID ());
    }
  return 0;
}

bool
has_running_vehicle (MNM_Veh_Factory *veh_factory)
{
  // TInt _total_veh = TInt(veh_factory -> m_veh_map.size());
  // TInt _finished_veh = 0;
  // TInt _enroute_veh;
  // for (auto _map_it : veh_factory -> m_veh_map){
  //   if (_map_it.second -> m_finish_time > 0) _finished_veh += 1;
  // }
  // _enroute_veh = _total_veh - _finished_veh;
  // return _enroute_veh != 0;

  return veh_factory->m_num_veh != veh_factory->m_finished;
}

// int round_time(int start_time_stamp, TFlt travel_time, TInt max_interval) {
//     int _end_time_stamp = start_time_stamp +
//     MNM_Ults::round_up_time(travel_time); if (_end_time_stamp >= max_interval
//     - 1) {
//         return int(max_interval - 1);
//     }
//     else {
//         return _end_time_stamp;
//     }
// }

}
