#pragma once

#include "common.h"
#include "dlink.h"
#include "dnode.h"
#include "emission.h"
#include "factory.h"
#include "gridlock_checker.h"
#include "io.h"
#include "od.h"
#include "pre_routing.h"
#include "routing.h"
#include "shortest_path.h"
#include "statistics.h"
#include "ults.h"
#include <string>

class MNM_Dta
{
public:
  explicit MNM_Dta (const std::string &file_folder);
  virtual ~MNM_Dta ();
  virtual int initialize ();
  virtual int build_from_files ();
  virtual bool is_ok ();
  int hook_up_node_and_link ();
  virtual int loading (bool verbose);
  virtual int load_once (bool verbose, TInt load_int, TInt assign_int);
  int test ();
  // private:
  virtual bool finished_loading (int cur_int);
  virtual int set_statistics ();
  virtual int set_gridlock_recorder ();
  virtual int set_routing ();
  int build_workzone ();
  int check_origin_destination_connectivity ();
  virtual int pre_loading ();

  virtual int record_queue_vehicles ();
  int record_enroute_vehicles ();

  TInt m_start_assign_interval;
  TInt m_total_assign_inter;
  TFlt m_unit_time;
  TFlt m_flow_scalar;
  TInt m_assign_freq;
  TInt m_init_demand_split;
  std::string m_file_folder;
  MNM_ConfReader *m_config;
  MNM_Veh_Factory *m_veh_factory;
  MNM_Node_Factory *m_node_factory;
  MNM_Link_Factory *m_link_factory;
  MNM_OD_Factory *m_od_factory;
  macposts::Graph m_graph;
  MNM_Statistics *m_statistics;
  MNM_Gridlock_Link_Recorder *m_gridlock_recorder;
  MNM_Routing *m_routing;
  MNM_Workzone *m_workzone;
  TInt m_current_loading_interval;
  MNM_Cumulative_Emission *m_emission;

  std::unordered_map<TInt, std::deque<TInt> *>
    m_queue_veh_map;                  // queuing vehicle number for each link
  std::deque<TInt> m_queue_veh_num;   // total queuing vehicle number
  std::deque<TInt> m_enroute_veh_num; // total enroute vehicle number
};

namespace MNM
{
int print_vehicle_statistics (MNM_Veh_Factory *veh_factory);
int print_vehicle_info (MNM_Veh_Factory *veh_factory);
bool has_running_vehicle (MNM_Veh_Factory *veh_factory);
// int round_time(int start_time_stamp, TFlt travel_time, TInt max_interval);
}
