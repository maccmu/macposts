#include "due.h"
#include <cfloat>

MNM_Due::MNM_Due (std::string file_folder)
{
  m_file_folder = file_folder;
  m_dta_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DTA");
  IAssert (m_dta_config->get_string ("routing_type") == "Due");
  // IAssert(m_dta_config->get_int("total_interval") > 0);
  // IAssert(m_dta_config->get_int("total_interval") >=
  //         m_dta_config->get_int("assign_frq") *
  //         m_dta_config->get_int("max_interval"));
  m_unit_time = m_dta_config->get_float ("unit_time");
  m_total_loading_inter = m_dta_config->get_int ("total_interval");
  m_due_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DUE");
  m_total_assign_inter = m_dta_config->get_int ("max_interval");
  if (m_total_loading_inter <= 0)
    m_total_loading_inter
      = m_total_assign_inter * m_dta_config->get_int ("assign_frq");
  m_path_table = nullptr;
  // m_od_factory = nullptr;

  // the unit of m_vot here is different from that of m_vot in adaptive routing
  // (money / second)
  try
    {
      m_vot = m_due_config->get_float ("vot") / 3600.
              * m_unit_time; // money / hour -> money / interval
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "vot does not exist in config.conf/DUE, use default value "
                   "20 usd/hour instead\n";
      m_vot = 20. / 3600. * m_unit_time; // money / interval
    }

  // money / hour -> money / interval
  m_early_penalty
    = m_due_config->get_float ("early_penalty") / 3600. * m_unit_time;
  m_late_penalty
    = m_due_config->get_float ("late_penalty") / 3600. * m_unit_time;
  // minute -> interval
  m_target_time = m_due_config->get_float ("target_time") * 60
                  / m_unit_time; // in terms of number of unit intervals

  m_step_size = m_due_config->get_float ("lambda");    // 0.01;  // MSA
  m_link_tt_map = std::unordered_map<TInt, TFlt *> (); // time-varying link tt
  m_link_cost_map
    = std::unordered_map<TInt, TFlt *> (); // time-varying link cost
}

// int MNM_Due::init_path_table()
// {
//   std::string _file_name = m_due_conf -> get_string("path_file_name");
//   std::string _whole_path = m_file_folder + "/" + _file_name;
//   TInt _num_path = m_due_conf -> get_int("num_path");
//   m_path_table = MNM_IO::load_path_table(_whole_path, m_graph, _num_path);
//   return 0;
// }

MNM_Due::~MNM_Due ()
{
  if (m_dta_config != nullptr)
    delete m_dta_config;
  if (m_due_config != nullptr)
    delete m_due_config;
  // if (m_od_factory != nullptr) delete m_od_factory;
}

// int MNM_Due::initialize()
// {
//   // m_graph = MNM_IO::build_graph(m_file_folder, m_dta_config);
//   // MNM_IO::build_od_factory(m_file_folder, m_dta_config, m_od_factory);
//   return 0;
// }

MNM_Dta *
MNM_Due::run_dta (bool verbose)
{
  MNM_Dta *_dta = new MNM_Dta (m_file_folder);
  // printf("dd\n");
  _dta->build_from_files ();
  // _dta -> m_od_factory = m_od_factory;
  // printf("ddd\n");
  update_demand_from_path_table (_dta);

  // now dynamic_cast<MNM_Routing_Fixed*>(dta -> m_routing) -> m_path_table will
  // point to m_path_table, watch this before deleting dta
  _dta->m_routing->init_routing (m_path_table);

  // printf("dddd\n");
  _dta->hook_up_node_and_link (); // in and out links for node, beginning and
                                  // ending nodes for link
  // printf("Checking......\n");
  // _dta -> is_ok();

  for (auto _link_it = _dta->m_link_factory->m_link_map.begin ();
       _link_it != _dta->m_link_factory->m_link_map.end (); _link_it++)
    {
      _link_it->second->install_cumulative_curve ();
    }

  _dta->pre_loading (); // initiate record file, junction model for node, and
                        // vehicle queue for link
  _dta->loading (verbose);
  return _dta;
}

int
MNM_Due::update_demand_from_path_table (MNM_Dta *dta)
{
  TFlt _tot_dmd;
  MNM_Origin *_org;
  MNM_Destination *_dest;

  for (auto _it : *m_path_table)
    { // origin node
      _org
        = ((MNM_DMOND *) dta->m_node_factory->get_node (_it.first))->m_origin;
      for (auto _it_it : *(_it.second))
        { // destination node
          _dest = ((MNM_DMDND *) dta->m_node_factory->get_node (_it_it.first))
                    ->m_dest;
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _tot_dmd = 0.0;
              if (_it_it.second->m_path_vec.size () > 0)
                {
                  for (MNM_Path *_path : _it_it.second->m_path_vec)
                    { // path
                      _tot_dmd += _path->m_buffer[_col];
                    }
                }
              _org->m_demand[_dest][_col] = _tot_dmd;
            }
        }
    }
  return 0;
}

TFlt
MNM_Due::compute_total_demand (MNM_Origin *orig, MNM_Destination *dest,
                               TInt total_assign_inter)
{
  TFlt _tot_dmd = 0.0;
  for (int i = 0; i < total_assign_inter; ++i)
    {
      _tot_dmd += orig->m_demand[dest][i];
    }
  return _tot_dmd;
}

TFlt
MNM_Due::compute_total_travel_time ()
{
  TFlt _tt, _depart_time; // _dis_utl;
  TFlt _total_tt = 0.0;
  for (auto _it : *m_path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _depart_time = TFlt (_col * m_dta_config->get_int ("assign_frq"));
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  _tt = get_tt (_depart_time, _path);
                  // _dis_utl = get_disutility(_depart_time, _tt);
                  // // assume build_link_cost_map(dta) and
                  // update_path_table_cost(dta) are invoked beforehand _tt =
                  // _path -> m_travel_time_vec[_col]; _dis_utl = _path ->
                  // m_travel_disutility_vec[_col];
                  _total_tt += _tt * _path->m_buffer[_col];
                }
            }
        }
    }
  return _total_tt;
}

TFlt
MNM_Due::compute_merit_function ()
{
  TFlt _tt, _depart_time, _dis_utl, _lowest_dis_utl;
  TFlt _total_gap = 0.0;
  for (auto _it : *m_path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          _lowest_dis_utl = DBL_MAX;
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _depart_time = TFlt (_col * m_dta_config->get_int ("assign_frq"));
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                { // get lowest disutility route at time interval _col and
                  // current OD pair
                  // _tt = get_tt(_depart_time, _path);
                  // _dis_utl = get_disutility(_depart_time, _tt);
                  // assume build_link_cost_map(dta) and
                  // update_path_table_cost(dta) are invoked beforehand
                  _tt = _path->m_travel_time_vec[_col];
                  _dis_utl = _path->m_travel_disutility_vec[_col];
                  if (_dis_utl < _lowest_dis_utl)
                    _lowest_dis_utl = _dis_utl;
                }
            }
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _depart_time = TFlt (_col * m_dta_config->get_int ("assign_frq"));
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  // _tt = get_tt(_depart_time, _path);
                  // _dis_utl = get_disutility(_depart_time, _tt);
                  // assume build_link_cost_map(dta) and
                  // update_path_table_cost(dta) are invoked beforehand
                  _tt = _path->m_travel_time_vec[_col];
                  _dis_utl = _path->m_travel_disutility_vec[_col];
                  _total_gap
                    += (_dis_utl - _lowest_dis_utl) * _path->m_buffer[_col];
                }
            }
        }
    }
  return _total_gap;
}

TFlt
MNM_Due::compute_merit_function_fixed_departure_time_choice ()
{
  TFlt _tt, _depart_time, _dis_utl, _lowest_dis_utl;
  TFlt _total_gap = 0.0;
  TFlt _min_flow_cost = 0.0;
  for (auto _it : *m_path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _lowest_dis_utl = DBL_MAX;
              _depart_time = TFlt (_col * m_dta_config->get_int ("assign_frq"));
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                { // get lowest disutility route at time interval _col and
                  // current OD pair
                  if (_path->m_buffer[_col] > 0)
                    {
                      // average over all loading intervals within an assignment
                      // interval? for (int i=0;
                      // i<m_dta_config->get_int("assign_frq"); i++) {
                      //     _tt = get_tt(_depart_time + TFlt(i), _path);
                      //     _dis_utl = get_disutility(_depart_time + TFlt(i),
                      //     _tt); if (_dis_utl < _lowest_dis_utl)
                      //     _lowest_dis_utl = _dis_utl;
                      // }

                      // _tt = get_tt(_depart_time, _path);
                      // _dis_utl = get_disutility(_depart_time, _tt);
                      // assume build_link_cost_map(dta) and
                      // update_path_table_cost(dta) are invoked beforehand
                      _tt = _path->m_travel_time_vec[_col];
                      _dis_utl = _path->m_travel_disutility_vec[_col];
                      if (_dis_utl < _lowest_dis_utl)
                        _lowest_dis_utl = _dis_utl;
                    }
                }
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  if (_path->m_buffer[_col] > 0)
                    {
                      // _tt = get_tt(_depart_time, _path);
                      // _dis_utl = get_disutility(_depart_time, _tt);
                      // assume build_link_cost_map(dta) and
                      // update_path_table_cost(dta) are invoked beforehand
                      _tt = _path->m_travel_time_vec[_col];
                      _dis_utl = _path->m_travel_disutility_vec[_col];
                      _total_gap
                        += (_dis_utl - _lowest_dis_utl) * _path->m_buffer[_col];
                      _min_flow_cost += _lowest_dis_utl * _path->m_buffer[_col];
                    }
                }
            }
        }
    }
  return _total_gap / _min_flow_cost;
}

TFlt
MNM_Due::get_disutility (TFlt depart_time, TFlt tt)
{
  TFlt _arrival_time = depart_time + tt;
  if (_arrival_time >= m_target_time)
    {
      return m_vot * tt + m_late_penalty * (_arrival_time - m_target_time);
    }
  else
    {
      return m_vot * tt + m_early_penalty * (m_target_time - _arrival_time);
    }
  // throw std::runtime_error ("Error in MNM_Due::get_disutility");
}

TFlt
MNM_Due::get_tt (TFlt depart_time, MNM_Path *path)
{
  // assume build_link_cost_map is invoked beforehand
  // true path tt, intervals
  return MNM_DTA_GRADIENT::get_path_travel_time (path, depart_time,
                                                 m_link_tt_map,
                                                 m_total_loading_inter);
}

int
MNM_Due::build_link_cost_map (MNM_Dta *dta)
{
  IAssert (m_total_loading_inter <= dta->m_current_loading_interval);
  MNM_Dlink *_link;
  for (auto _link_it : dta->m_link_factory->m_link_map)
    {
      std::cout << "********************** link " << _link_it.first
                << " **********************\n";
      for (int i = 0; i < m_total_loading_inter; i++)
        {
          _link = _link_it.second;
          // use i+1 as start_time in cc to compute link travel time for
          // vehicles arriving at the beginning of interval i, i+1 is the end of
          // the interval i, the beginning of interval i + 1
          m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time (_link, TFlt (i + 1), m_unit_time,
                             dta->m_current_loading_interval); // intervals
          m_link_cost_map[_link_it.first][i]
            = m_vot * m_link_tt_map[_link_it.first][i] + _link->m_toll;
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_link_cost_map[_link_it.first][i] << "\n";
        }
    }
  return 0;
}

int
MNM_Due::update_path_table_cost (MNM_Dta *dta)
{
  TInt _o_node_ID, _d_node_ID;
  // #pragma omp parallel num_threads(20)
  for (auto _o_it : *m_path_table)
    {
      _o_node_ID = _o_it.first;
      for (auto _d_it : *(_o_it.second))
        {
          _d_node_ID = _d_it.first;
          for (auto _path : _d_it.second->m_path_vec)
            {
              update_one_path_cost (_path, _o_node_ID, _d_node_ID, dta);
              printf ("update_one_path_cost\n");
            }
        }
    }
  printf ("Finish update path table cost\n");
  return 0;
}

int
MNM_Due::update_one_path_cost (MNM_Path *path, TInt o_node_ID, TInt d_node_ID,
                               MNM_Dta *dta)
{
  TInt _depart_time;
  TFlt _travel_time, _travel_disutility; // _travel_cost;

  path->m_travel_time_vec.clear ();
  // path->m_travel_cost_vec.clear();
  path->m_travel_disutility_vec.clear ();
  for (int _col = 0; _col < m_total_assign_inter; _col++)
    {
      _depart_time = _col * m_dta_config->get_int ("assign_frq");

      _travel_time = get_tt (TFlt (_depart_time), path); // intervals
      // _travel_cost = _travel_time * m_vot;
      _travel_disutility = get_disutility (TFlt (_depart_time), _travel_time);

      _travel_time = _travel_time * m_unit_time; // seconds

      path->m_travel_time_vec.push_back (_travel_time);
      // path->m_travel_cost_vec.push_back(_travel_cost);
      path->m_travel_disutility_vec.push_back (_travel_disutility);
    }
  return 0;
}

//*************************************************************************
//                                MNM_Due_Msa
//*************************************************************************

MNM_Due_Msa::MNM_Due_Msa (std::string file_folder)
    : MNM_Due::MNM_Due (file_folder)
{
  m_base_dta = nullptr;
}

MNM_Due_Msa::~MNM_Due_Msa ()
{
  for (auto _tt_it : m_link_tt_map)
    {
      delete _tt_it.second;
    }
  m_link_tt_map.clear ();
  for (auto _cost_it : m_link_cost_map)
    {
      delete _cost_it.second;
    }
  m_link_cost_map.clear ();
  delete m_base_dta;
}

int
MNM_Due_Msa::initialize ()
{
  m_base_dta = new MNM_Dta (m_file_folder);
  m_base_dta->build_from_files ();

  // build shortest path for each OD pair, no matter it is in demand or not
  // m_path_table = MNM::build_shortest_pathset(m_base_dta->m_graph,
  //                                            m_base_dta->m_od_factory,
  //                                            m_base_dta->m_link_factory);
  // MNM::allocate_path_table_buffer(m_path_table, m_total_assign_inter);  //
  // create zero buffer

  // build shortest path for each OD pair in demand
  m_path_table
    = MNM::build_pathset (m_base_dta->m_graph, m_base_dta->m_od_factory,
                          m_base_dta->m_link_factory, 0, 0, m_vot / m_unit_time,
                          1, 1, m_total_assign_inter);
  // MNM::save_path_table(m_path_table, m_base_dta -> m_od_factory, true);

  for (auto _link_it : m_base_dta->m_link_factory->m_link_map)
    {
      m_link_tt_map[_link_it.first] = new TFlt[m_total_loading_inter];
      m_link_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
    }

  // printf("%d, %d\n", m_od_factory -> m_origin_map.size(), m_od_factory ->
  // m_destination_map.size());
  printf ("finish initialization\n");
  return 0;
}

// spread the OD demand evenly over the initial paths
int
MNM_Due_Msa::init_path_flow ()
{
  TFlt _len, _dmd;
  MNM_Origin *_org;
  MNM_Destination *_dest;
  for (auto _it : *m_path_table)
    {
      _org = ((MNM_DMOND *) m_base_dta->m_node_factory->get_node (_it.first))
               ->m_origin;
      for (auto _it_it : *(_it.second))
        {
          _dest = ((MNM_DMDND *) m_base_dta->m_node_factory->get_node (
                     _it_it.first))
                    ->m_dest;
          _len = TFlt (_it_it.second->m_path_vec.size ());
          IAssert (_len >= 1);
          for (MNM_Path *_path : _it_it.second->m_path_vec)
            {
              // printf("24m\n");
              for (int _col = 0; _col < m_total_assign_inter; _col++)
                {
                  // printf("25m\n");
                  if (_org->m_demand.find (_dest) == _org->m_demand.end ())
                    { // if this OD pair not in demand file, demand = 0
                      _dmd = TFlt (0.0);
                    }
                  else
                    {
                      _dmd = _org->m_demand[_dest][_col];
                    }
                  // printf("%lf\n", _dmd());
                  _path->m_buffer[_col] = TFlt (1.0) / _len * _dmd;
                }
            }
        }
    }
  // MNM::save_path_table(m_path_table, m_od_factory, true);
  printf ("Finish init route choice\n");
  return 0;
}

// best route for all assignment intervals
std::pair<MNM_Path *, TInt>
MNM_Due_Msa::get_best_route (TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::max ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_cost;
  for (int i = 0; i < tdsp_tree->m_max_interval; ++i)
    { // tdsp_tree -> m_max_interval = total_loading_interval
      _tmp_tt
        = tdsp_tree->m_dist[o_node_ID][i < (int) tdsp_tree->m_max_interval
                                         ? i
                                         : (int) tdsp_tree->m_max_interval - 1];
      std::cout << "interval: " << i << ", tdsp_tt: " << _tmp_tt << "\n";
      _tmp_cost = get_disutility (TFlt (i), _tmp_tt);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
        }
    }
  IAssert (_cur_best_time >= 0);
  MNM_Path *_path = new MNM_Path ();
  tdsp_tree->get_tdsp (o_node_ID, _cur_best_time, m_link_tt_map, _path);
  std::pair<MNM_Path *, TInt> _best = std::make_pair (_path, _cur_best_time);
  return _best;
}

// best route for a single interval
std::pair<MNM_Path *, TInt>
MNM_Due_Msa::get_best_route_for_single_interval (TInt interval, TInt o_node_ID,
                                                 MNM_TDSP_Tree *tdsp_tree)
{
  IAssert (interval + m_dta_config->get_int ("assign_frq")
           <= tdsp_tree->m_max_interval); // tdsp_tree -> m_max_interval =
                                          // total_loading_interval
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::max ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_cost;
  for (int i = interval; i < interval + 1; ++i)
    {
      // for (int i = interval; i < interval +
      // m_dta_config->get_int("assign_frq"); ++i) {  // tdsp_tree ->
      // m_max_interval = total_loading_interval
      _tmp_tt
        = tdsp_tree->m_dist[o_node_ID][i < (int) tdsp_tree->m_max_interval
                                         ? i
                                         : (int) tdsp_tree->m_max_interval - 1];
      std::cout << "interval: " << i << ", tdsp_tt: " << _tmp_tt << "\n";
      _tmp_cost = get_disutility (TFlt (i), _tmp_tt);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
        }
    }
  IAssert (_cur_best_time >= 0);
  MNM_Path *_path = new MNM_Path ();
  tdsp_tree->get_tdsp (o_node_ID, _cur_best_time, m_link_tt_map, _path);
  std::pair<MNM_Path *, TInt> _best = std::make_pair (_path, _cur_best_time);
  return _best;
}

int
MNM_Due_Msa::update_path_table (MNM_Dta *dta, int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID;
  std::pair<MNM_Path *, TInt> _path_result;
  MNM_Path *_path;
  MNM_Pathset *_path_set;
  TFlt _tot_change, _tmp_change, _tot_oneOD_demand, _len;
  MNM_Path *_best_path;
  int _best_time_col;
  int _best_assign_col;

  // assume build_link_cost_map(dta) and update_path_table_cost(dta) are invoked
  // beforehand
  for (auto _it : dta->m_od_factory->m_destination_map)
    {
      _dest = _it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;
      MNM_TDSP_Tree *_tdsp_tree
        = new MNM_TDSP_Tree (_dest_node_ID, dta->m_graph,
                             m_total_loading_inter);
      _tdsp_tree->initialize ();
      // printf("111\n");
      _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);
      for (auto _map_it : dta->m_od_factory->m_origin_map)
        {
          _orig = _map_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (_orig->m_demand.find (_dest) == _orig->m_demand.end ())
            {
              continue;
            }

          _path_result = get_best_route (_orig_node_ID, _tdsp_tree);
          _path = _path_result.first;
          _best_time_col = int (_path_result.second);
          _best_assign_col
            = floor (_best_time_col / m_dta_config->get_int ("assign_frq"));
          if (_best_assign_col >= m_total_assign_inter)
            _best_assign_col = m_total_assign_inter - 1;
          // printf("Best time col %d\n", _best_time_col);

          _path_set
            = MNM::get_pathset (m_path_table, _orig_node_ID, _dest_node_ID);

          _tot_oneOD_demand
            = compute_total_demand (_orig, _dest, m_total_assign_inter);
          _tot_change = 0.0;
          _best_path = NULL;

          if (_path_set->is_in (_path))
            {
              printf ("Update current pathset\n");
              for (auto _tmp_path : _path_set->m_path_vec)
                {
                  for (int _col = 0; _col < m_total_assign_inter; _col++)
                    {
                      if ((!(*_tmp_path == *_path))
                          || (_col != _best_assign_col))
                        {
                          // printf("dd %lf\n", _tmp_path -> m_buffer[_col]());
                          // printf("iter %d\n", iter);
                          // Original
                          _tmp_change
                            = std::min (_tmp_path->m_buffer[_col],
                                        _tot_oneOD_demand * m_step_size
                                          / TFlt (iter + 1));
                          // Modified by Zou
                          // _tmp_change = _tmp_path->m_buffer[_col] *
                          // m_step_size / TFlt(iter + 1); printf("tmp change
                          // %lf\n", _tmp_change());
                          _tmp_path->m_buffer[_col] -= _tmp_change;
                          _tot_change += _tmp_change;
                        }
                    }
                  if (*_tmp_path == *_path)
                    {
                      _best_path = _tmp_path;
                    }
                }
              // printf("dddd, %lf\n", _tot_change());
              _best_path->m_buffer[_best_assign_col] += _tot_change;
              // printf("sssss\n");
              delete _path;
            }
          else
            {
              printf ("Adding new path\n");

              // IAssert(_path->m_travel_time_vec.empty() &&
              //         _path->m_travel_disutility_vec.empty());
              // update_one_path_cost(_path, _orig_node_ID, _dest_node_ID, dta);

              // Original
              _path->allocate_buffer (m_total_assign_inter);
              _path->m_buffer[_best_assign_col]
                += m_step_size / TFlt (iter + 1);
              _len = TFlt (_path_set->m_path_vec.size ());
              for (auto tmp_path : _path_set->m_path_vec)
                {
                  tmp_path->m_buffer[_best_assign_col]
                    -= m_step_size / TFlt (iter + 1) / _len;
                }
              _path_set->m_path_vec.push_back (_path);

              // Modified by Zou
              //                _path->allocate_buffer(m_total_assign_inter);
              //                _path_set->m_path_vec.push_back(_path);
              //                for (auto _tmp_path : _path_set->m_path_vec) {
              //                    for (int _col = 0; _col <
              //                    m_total_assign_inter; _col++) {
              //                        if ((!(*_tmp_path == *_path)) || (_col
              //                        != _best_assign_col)) {
              //                            _tmp_change =
              //                            _tmp_path->m_buffer[_col] *
              //                            m_step_size / TFlt(iter + 1);
              //                            // printf("tmp change %lf\n",
              //                            _tmp_change());
              //                            _tmp_path->m_buffer[_col] -=
              //                            _tmp_change; _tot_change +=
              //                            _tmp_change;
              //                        }
              //                    }
              //                    if (*_tmp_path == *_path) {
              //                        _best_path = _tmp_path;
              //                    }
              //                }
              //                _best_path->m_buffer[_best_assign_col] +=
              //                _tot_change;
            }
        }
      delete _tdsp_tree;
    }
  // MNM::print_path_table(m_path_table, dta->m_od_factory, true);
  // MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics ->
  // m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory,
  // true);
  return 0;
}

int
MNM_Due_Msa::update_path_table_fixed_departure_time_choice (MNM_Dta *dta,
                                                            int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID;
  std::pair<MNM_Path *, TInt> _path_result;
  MNM_Path *_path;
  MNM_Pathset *_path_set;
  TFlt _tot_change, _tmp_change;
  MNM_Path *_best_path;
  bool _exist;

  // assume build_link_cost_map(dta) and update_path_table_cost(dta) are invoked
  // beforehand
  for (auto _it : dta->m_od_factory->m_destination_map)
    {
      _dest = _it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      MNM_TDSP_Tree *_tdsp_tree
        = new MNM_TDSP_Tree (_dest_node_ID, dta->m_graph,
                             m_total_loading_inter);
      _tdsp_tree->initialize ();
      _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);

      for (auto _map_it : dta->m_od_factory->m_origin_map)
        {
          _orig = _map_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (_orig->m_demand.find (_dest) == _orig->m_demand.end ())
            {
              continue;
            }

          _path_set
            = MNM::get_pathset (m_path_table, _orig_node_ID, _dest_node_ID);

          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _tot_change = 0.0;
              _best_path = nullptr;

              _path_result
                = get_best_route_for_single_interval (_col
                                                        * m_dta_config
                                                            ->get_int (
                                                              "assign_frq"),
                                                      _orig_node_ID,
                                                      _tdsp_tree);
              _path = _path_result.first;

              if (_path_set->is_in (_path))
                {
                  printf ("Update current pathset\n");
                  _exist = true;
                }
              else
                {
                  printf ("Adding new path\n");
                  // IAssert(_path->m_travel_time_vec.empty() &&
                  //         _path->m_travel_disutility_vec.empty());
                  // update_one_path_cost(_path, _orig_node_ID, _dest_node_ID,
                  // dta);
                  _path->allocate_buffer (m_total_assign_inter);
                  _path_set->m_path_vec.push_back (_path);
                  _exist = false;
                }

              for (auto _tmp_path : _path_set->m_path_vec)
                {
                  if (!(*_tmp_path == *_path))
                    {
                      // printf("dd %lf\n", _tmp_path -> m_buffer[_col]());
                      // printf("iter %d\n", iter);
                      _tmp_change = _tmp_path->m_buffer[_col] * m_step_size
                                    / sqrt (TFlt (iter + 1));
                      // printf("tmp change %lf\n", _tmp_change());
                      _tmp_path->m_buffer[_col] -= _tmp_change;
                      _tot_change += _tmp_change;
                    }
                  else
                    {
                      _best_path = _tmp_path;
                    }
                }
              _best_path->m_buffer[_col] += _tot_change;
              if (_exist)
                delete _path;
            }
        }
      delete _tdsp_tree;
    }
  // MNM::print_path_table(m_path_table, dta->m_od_factory, true);
  // MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics ->
  // m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory,
  // true);
  return 0;
}

int
MNM_Due_Msa::update_path_table_gp_fixed_departure_time_choice (MNM_Dta *dta,
                                                               int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID, _tot_nonzero_path;
  std::pair<MNM_Path *, TInt> _path_result;
  MNM_Path *_path;
  MNM_Pathset *_path_set;
  TFlt _tot_path_cost, _tmp_change, _tau, _tmp_tt, _tmp_cost, _min_flow;
  bool _exist, _flg;

  // assume build_link_cost_map(dta) and update_path_table_cost(dta) are invoked
  // beforehand
  for (auto _it : dta->m_od_factory->m_destination_map)
    {
      _dest = _it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      MNM_TDSP_Tree *_tdsp_tree
        = new MNM_TDSP_Tree (_dest_node_ID, dta->m_graph,
                             m_total_loading_inter);
      _tdsp_tree->initialize ();
      _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);

      for (auto _map_it : dta->m_od_factory->m_origin_map)
        {
          _orig = _map_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (_orig->m_demand.find (_dest) == _orig->m_demand.end ())
            {
              continue;
            }

          _path_set
            = MNM::get_pathset (m_path_table, _orig_node_ID, _dest_node_ID);

          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _path_result
                = get_best_route_for_single_interval (_col
                                                        * m_dta_config
                                                            ->get_int (
                                                              "assign_frq"),
                                                      _orig_node_ID,
                                                      _tdsp_tree);
              _path = _path_result.first;

              _tot_path_cost = 0.0;
              _tot_nonzero_path = 0;
              _tau = TFlt (std::numeric_limits<double>::max ());
              _flg = false;
              _min_flow = TFlt (std::numeric_limits<double>::max ());
              _tmp_change = 0.0;
              if (_path_set->is_in (_path))
                {
                  printf ("Update current pathset\n");
                  _exist = true;
                }
              else
                {
                  printf ("Adding new path\n");
                  IAssert (_path->m_travel_time_vec.empty ()
                           && _path->m_travel_disutility_vec.empty ());
                  update_one_path_cost (_path, _orig_node_ID, _dest_node_ID,
                                        dta);
                  _path->allocate_buffer (m_total_assign_inter);
                  _path_set->m_path_vec.push_back (_path);
                  _exist = false;
                }

              // average path cost
              for (auto _tmp_path : _path_set->m_path_vec)
                {
                  if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0))
                    {
                      // _tmp_tt = get_tt(_col *
                      // m_dta_config->get_int("assign_frq"), _tmp_path);
                      // printf("path in pathset, tt %lf\n", (TFlt)_tmp_tt);
                      // _tmp_cost = get_disutility(TFlt(_col *
                      // m_dta_config->get_int("assign_frq")), _tmp_tt);
                      _tmp_tt = _tmp_path->m_travel_time_vec[_col];
                      printf ("path in pathset, tt %lf\n", (TFlt) _tmp_tt);
                      _tmp_cost = _tmp_path->m_travel_disutility_vec[_col];
                      printf ("path in pathset, disutility %lf\n",
                              (TFlt) _tmp_cost);
                      _tot_path_cost += _tmp_cost;
                      _tot_nonzero_path += 1;
                      if ((_tmp_path->m_buffer[_col] > 0)
                          && (_min_flow > _tmp_path->m_buffer[_col]))
                        {
                          _min_flow = _tmp_path->m_buffer[_col];
                        }
                    }
                }
              IAssert (_tot_nonzero_path > 0);
              // minimum tau
              for (auto _tmp_path : _path_set->m_path_vec)
                {
                  if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0))
                    {
                      // _tmp_tt = get_tt(_col *
                      // m_dta_config->get_int("assign_frq"), _tmp_path);
                      // _tmp_cost = get_disutility(TFlt(_col *
                      // m_dta_config->get_int("assign_frq")), _tmp_tt);
                      _tmp_cost = _tmp_path->m_travel_disutility_vec[_col];
                      _tmp_change
                        = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
                      // printf("_tmp_change: %lf\n", _tmp_change());  // it may
                      // be zero
                      if ((_tmp_change > 0)
                          && (_tau > m_step_size * _min_flow / _tmp_change))
                        {
                          _tau = m_step_size * _min_flow / _tmp_change;
                          _flg = true;
                        }
                      // if ((_tmp_change > 0) && (_tau > 1.0 / _tmp_change)) {
                      //     _tau = 1.0 / _tmp_change;
                      // }
                    }
                }
              // printf("tau: %lf\n", _tau());
              if (!_flg)
                {
                  _tau = 0.;
                  continue;
                }
              // printf("tau: %lf\n", _tau());
              // flow adjustment
              for (auto _tmp_path : _path_set->m_path_vec)
                {
                  if ((*_tmp_path == *_path) || (_tmp_path->m_buffer[_col] > 0))
                    {
                      // _tmp_tt = get_tt(_col *
                      // m_dta_config->get_int("assign_frq"), _tmp_path);
                      // _tmp_cost = get_disutility(TFlt(_col *
                      // m_dta_config->get_int("assign_frq")), _tmp_tt);
                      _tmp_cost = _tmp_path->m_travel_disutility_vec[_col];
                      _tmp_change
                        = _tmp_cost - _tot_path_cost / _tot_nonzero_path;
                      _tmp_path->m_buffer[_col]
                        -= std::min (_tmp_path->m_buffer[_col],
                                     _tau * _tmp_change);
                    }
                }
              if (_exist)
                delete _path;
            }
        }
      delete _tdsp_tree;
    }
  // MNM::print_path_table(m_path_table, dta->m_od_factory, true);
  // MNM::save_path_table(dta -> m_file_folder + "/" + dta -> m_statistics ->
  // m_self_config -> get_string("rec_folder"), m_path_table, dta->m_od_factory,
  // true);
  return 0;
}
