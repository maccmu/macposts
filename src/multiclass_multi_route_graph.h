#pragma once

#include "multiclass.h"

// multiclass vehicles
// car and truck both have subclasses, which may use different graph for routing

/**************************************************************************
                            Vehicle
**************************************************************************/
class MNM_Veh_Multiclass_Subclass : public MNM_Veh_Multiclass
{
public:
  MNM_Veh_Multiclass_Subclass (TInt ID, TInt vehicle_class, TInt vehicle_subclass, TInt start_time);
  virtual ~MNM_Veh_Multiclass_Subclass ();

  // virtual getter
  virtual int get_subclass () {return m_subclass;};
  virtual TInt get_class () override { return m_class; }; 
  virtual TInt get_bus_route_ID () override { return m_bus_route_ID;};  
  virtual bool get_ispnr () override { return m_pnr; };
  virtual TInt get_label () override {return m_label;}; // virtual getter for derived class for emission analysis

  int m_subclass;
};

/**************************************************************************
                            Vehicle Factory
**************************************************************************/
class MNM_Veh_Factory_Multiclass_Subclass : public MNM_Veh_Factory_Multiclass
{
public:
  MNM_Veh_Factory_Multiclass_Subclass ();
  virtual ~MNM_Veh_Factory_Multiclass_Subclass () override;

  // use this one instead of make_veh in the base class
  MNM_Veh_Multiclass_Subclass *
  make_veh_multiclass_subclass (TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls, TInt vehicle_subcls);
  virtual int remove_finished_veh (MNM_Veh *veh, bool del = true) override;
  virtual std::string print_vehicle_statistics () override;
  std::unordered_map<int, int> m_num_car_subclass;
  std::unordered_map<int, int> m_num_truck_subclass;
  std::unordered_map<int, int> m_enroute_car_subclass;
  std::unordered_map<int, int> m_enroute_truck_subclass;
  std::unordered_map<int, int> m_finished_car_subclass;
  std::unordered_map<int, int> m_finished_truck_subclass;
  std::unordered_map<int, TFlt> m_total_miles_car_subclass;
  std::unordered_map<int, TFlt> m_total_miles_truck_subclass;
  std::unordered_map<int, TFlt> m_total_time_car_subclass;
  std::unordered_map<int, TFlt> m_total_time_truck_subclass;
  std::unordered_map<int, TFlt> m_total_delay_car_subclass;
  std::unordered_map<int, TFlt> m_total_delay_truck_subclass;
};

/**************************************************************************
                            Link
**************************************************************************/
class MNM_Dlink_Multiclass_Subclass : virtual public MNM_Dlink_Multiclass
{
public:
  MNM_Dlink_Multiclass_Subclass (TInt ID, TInt number_of_lane, TFlt length, TFlt ffs_car,
    TFlt ffs_truck, TInt num_car_subclass, TInt num_truck_subclass);
  virtual ~MNM_Dlink_Multiclass_Subclass () override;

  // seperate N-curves for private car and truck subclasses
  std::unordered_map<int, MNM_Cumulative_Curve *>m_N_in_car_subclass;
  std::unordered_map<int, MNM_Cumulative_Curve *>m_N_out_car_subclass;
  std::unordered_map<int, MNM_Cumulative_Curve *>m_N_in_truck_subclass;
  std::unordered_map<int, MNM_Cumulative_Curve *>m_N_out_truck_subclass;

  // seperate N-curve_trees for private car and truck subclasses
  std::unordered_map<int, MNM_Tree_Cumulative_Curve *>m_N_in_tree_car_subclass;
  std::unordered_map<int, MNM_Tree_Cumulative_Curve *>m_N_out_tree_car_subclass;
  std::unordered_map<int, MNM_Tree_Cumulative_Curve *>m_N_in_tree_truck_subclass;
  std::unordered_map<int, MNM_Tree_Cumulative_Curve *>m_N_out_tree_truck_subclass;

  std::unordered_map<int, TFlt> m_toll_car_subclass;
  std::unordered_map<int, TFlt> m_toll_truck_subclass;

  // use this one instead of the one in Dlink class
  int install_cumulative_curve_multiclass_subclass ();
  // use this one instead of the one in Dlink class
  int install_cumulative_curve_tree_multiclass_subclass ();

  virtual std::vector<TFlt> get_link_flow_emission_car_subclass(int subclass, int ev_label) 
  {
    std::vector<TFlt> _r = { TFlt (0), TFlt (0) };
    return _r;
  };
  virtual std::vector<TFlt> get_link_flow_emission_truck_subclass(int subclass, int ev_label)
  {
    std::vector<TFlt> _r = { TFlt (0), TFlt (0) };
    return _r;
  };
};

class MNM_Dlink_Ctm_Multiclass_Subclass : public MNM_Dlink_Ctm_Multiclass, public MNM_Dlink_Multiclass_Subclass
{
public:
  MNM_Dlink_Ctm_Multiclass_Subclass (TInt ID, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
    TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
    TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar, 
    TInt num_car_subclass, TInt num_truck_subclass);
  virtual ~MNM_Dlink_Ctm_Multiclass_Subclass () override;
  virtual std::vector<TFlt> get_link_flow_emission_car_subclass(int subclass, int ev_label) override;
  virtual std::vector<TFlt> get_link_flow_emission_truck_subclass(int subclass, int ev_label) override;
};

class MNM_Dlink_Pq_Multiclass_Subclass : public MNM_Dlink_Pq_Multiclass, public MNM_Dlink_Multiclass_Subclass
{
public:
  MNM_Dlink_Pq_Multiclass_Subclass (TInt ID, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
    TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
    TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar,
    TInt num_car_subclass, TInt num_truck_subclass);
  virtual ~MNM_Dlink_Pq_Multiclass_Subclass () override;
  virtual std::vector<TFlt> get_link_flow_emission_car_subclass(int subclass, int ev_label) override;
  virtual std::vector<TFlt> get_link_flow_emission_truck_subclass(int subclass, int ev_label) override;
};

class MNM_Dlink_Lq_Multiclass_Subclass : public MNM_Dlink_Lq_Multiclass, public MNM_Dlink_Multiclass_Subclass
{
public:
  MNM_Dlink_Lq_Multiclass_Subclass (TInt ID, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
    TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
    TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar,
    TInt num_car_subclass, TInt num_truck_subclass);
  virtual ~MNM_Dlink_Lq_Multiclass_Subclass () override;
  virtual std::vector<TFlt> get_link_flow_emission_car_subclass(int subclass, int ev_label) override;
  virtual std::vector<TFlt> get_link_flow_emission_truck_subclass(int subclass, int ev_label) override;
};

struct td_link_attribute_row_biclass_subclass : public td_link_attribute_row_biclass
{
  std::unordered_map<TInt, TFlt> toll_car;
  std::unordered_map<TInt, TFlt> toll_truck;
};

class MNM_Link_Factory_Multiclass_Subclass : public MNM_Link_Factory_Multiclass
{
public:
  MNM_Link_Factory_Multiclass_Subclass ();
  virtual ~MNM_Link_Factory_Multiclass_Subclass () override;

  virtual int update_link_attribute (TInt interval,
    bool verbose = false) override;

  // use this one instead of make_link in the base class
  MNM_Dlink *make_link_multiclass_subclass (
    TInt ID, DLink_type_multiclass link_type, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
    TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar, TInt num_car_subclass, TInt num_truck_subclass);

  // <interval, <link_ID, attribute>>
  std::unordered_map<int, std::unordered_map<int, td_link_attribute_row_biclass_subclass *> *>
    *m_td_link_attribute_table;
};

/**************************************************************************
                            Node
**************************************************************************/
class MNM_DMOND_Multiclass_Subclass : public MNM_DMOND_Multiclass
{
public:
  MNM_DMOND_Multiclass_Subclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_DMOND_Multiclass_Subclass () override;
  virtual int evolve (TInt timestamp) override;
};

class MNM_DMDND_Multiclass_Subclass : public MNM_DMDND_Multiclass
{
public:
  MNM_DMDND_Multiclass_Subclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_DMDND_Multiclass_Subclass () override;
  virtual int evolve (TInt timestamp) override;
};

class MNM_Dnode_Inout_Multiclass_Subclass : virtual public MNM_Dnode_Inout_Multiclass
{
public:
  MNM_Dnode_Inout_Multiclass_Subclass (TInt ID, TFlt flow_scalar,
                                      TFlt veh_convert_factor);
  virtual ~MNM_Dnode_Inout_Multiclass_Subclass () override;
  virtual int move_one_vehicle (TInt timestamp, MNM_Dlink *_in_link, MNM_Dlink *_out_link, MNM_Veh *_veh,
    size_t _in_link_i, size_t _out_link_j, size_t _offset) override;
  // virtual int evolve (TInt timestamp) override;
};

class MNM_Dnode_FWJ_Multiclass_Subclass : public MNM_Dnode_Inout_Multiclass_Subclass, public MNM_Dnode_FWJ_Multiclass
{
public:
  MNM_Dnode_FWJ_Multiclass_Subclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_Dnode_FWJ_Multiclass_Subclass () override;
};

class MNM_Node_Factory_Multiclass_Subclass : public MNM_Node_Factory_Multiclass
{
public:
  MNM_Node_Factory_Multiclass_Subclass ();
  virtual ~MNM_Node_Factory_Multiclass_Subclass () override;

  // use this one instead of make_node in the base class
  MNM_Dnode *make_node_multiclass_subclass (TInt ID, DNode_type_multiclass node_type,
                                            TFlt flow_scalar, TFlt veh_convert_factor);
};

/**************************************************************************
                            Origin
**************************************************************************/

class MNM_Origin_Multiclass_Subclass : public MNM_Origin_Multiclass
{
public:
    MNM_Origin_Multiclass_Subclass (TInt ID, TInt max_interval, TFlt flow_scalar,
                                    TInt frequency);
    virtual ~MNM_Origin_Multiclass_Subclass () override;

    virtual int release_one_interval_biclass (TInt current_interval,
                                            MNM_Veh_Factory *veh_factory,
                                            TInt assign_interval,
                                            TFlt adaptive_ratio_car,
                                            TFlt adaptive_ratio_truck) override;

    int add_dest_demand_multiclass_subclass (MNM_Destination_Multiclass *dest, int mainclass_label, int subclass_label, 
                                            TFlt *demand);
    int get_dest_demand_multiclass();

    // <dest, <subclass_label, demand>>
    std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>>  m_demand_car_subclass;
    std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>>  m_demand_truck_subclass;
};

class MNM_OD_Factory_Multiclass_Subclass : public MNM_OD_Factory_Multiclass
{
public:
  MNM_OD_Factory_Multiclass_Subclass ();
  virtual ~MNM_OD_Factory_Multiclass_Subclass () override;
  virtual MNM_Origin_Multiclass_Subclass *make_origin (TInt ID, TInt max_interval,
                                                      TFlt flow_scalar,
                                                      TInt frequency) override;
};

/**************************************************************************
                            Routing
**************************************************************************/
class MNM_Routing_Biclass_Fixed_Subclass : public MNM_Routing_Biclass_Fixed
{
public:
  MNM_Routing_Biclass_Fixed_Subclass (
    macposts::Graph &graph, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    TInt route_frq_fixed = TInt (-1), TInt buffer_length = TInt (-1), TInt buffer_start_index = 0,
    TInt veh_class = 0, TInt veh_subclass = 0);
  virtual ~MNM_Routing_Biclass_Fixed_Subclass () override;
  virtual int change_choice_portion (TInt interval) override;
  virtual int update_routing (TInt timestamp) override;
  // virtual int remove_finished (MNM_Veh *veh, bool del = true) override;

  int m_veh_subclass;
  int m_buffer_start_index;
};

class MNM_Routing_Adaptive_Subclass : public MNM_Routing_Adaptive
{
public:
  MNM_Routing_Adaptive_Subclass (const std::string &file_folder, macposts::Graph &graph,
                        MNM_Statistics *statistics, MNM_OD_Factory *od_factory,
                        MNM_Node_Factory *node_factory,
                        MNM_Link_Factory *link_factory,
                        TInt veh_class = 0, TInt veh_subclass = 0);
  virtual ~MNM_Routing_Adaptive_Subclass () override;
  virtual int update_link_cost () override;
  virtual int update_routing (TInt timestamp) override;

  int m_veh_class;
  int m_veh_subclass;
};

class MNM_Routing_Biclass_Hybrid_Subclass : public MNM_Routing
{
public:
  MNM_Routing_Biclass_Hybrid_Subclass (
    const std::string &file_folder, macposts::Graph &graph, std::vector<macposts::Graph> &graph_vec,
    MNM_Statistics *statistics, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    std::vector<Path_Table*> &path_table_vec,
    std::unordered_map<TInt, TInt> &subclass_car_graph_mapping, std::unordered_map<TInt, TInt> &subclass_car_path_table_mapping,
    std::unordered_map<TInt, TInt> &subclass_truck_graph_mapping, std::unordered_map<TInt, TInt> &subclass_truck_path_table_mapping,
    TInt route_frq_fixed = TInt (-1), TInt buffer_length = TInt (-1));
  virtual ~MNM_Routing_Biclass_Hybrid_Subclass () override;
  virtual int init_routing (Path_Table *driving_path_table = nullptr) override;
  virtual int update_routing (TInt timestamp) override;
  virtual int remove_finished (MNM_Veh *veh, bool del = true) override;

  std::unordered_map<int, MNM_Routing_Biclass_Fixed_Subclass*> m_routing_fixed_car_subclass;
  std::unordered_map<int, MNM_Routing_Adaptive_Subclass*> m_routing_adaptive_car_subclass;
  std::unordered_map<int, MNM_Routing_Biclass_Fixed_Subclass*> m_routing_fixed_truck_subclass;
  std::unordered_map<int, MNM_Routing_Adaptive_Subclass*> m_routing_adaptive_truck_subclass;
};

/**************************************************************************
                            Emission
**************************************************************************/
class MNM_Cumulative_Emission_Multiclass_Subclass : public MNM_Cumulative_Emission_Multiclass
{
public:
  // -1 is the default veh -> m_label value, use -2 ad ev label
  MNM_Cumulative_Emission_Multiclass_Subclass (TFlt unit_time, TInt freq, int num_car_subclass, int num_truck_subclass,
                                        TInt ev_label_car = -2,
                                        TInt ev_label_truck = -2);
  virtual ~MNM_Cumulative_Emission_Multiclass_Subclass () override;


  virtual int update (MNM_Veh_Factory *veh_factory) override;
  virtual std::string output () override;

  int m_num_car_subclass;
  int m_num_truck_subclass;

  std::unordered_map<int, TFlt> m_fuel_car_subclass;
  std::unordered_map<int, TFlt> m_CO2_car_subclass;
  std::unordered_map<int, TFlt> m_HC_car_subclass;
  std::unordered_map<int, TFlt> m_CO_car_subclass;
  std::unordered_map<int, TFlt> m_NOX_car_subclass;
  std::unordered_map<int, TFlt> m_VMT_car_subclass;
  std::unordered_map<int, TFlt> m_VMT_ev_car_subclass;

  std::unordered_map<int, TFlt> m_fuel_truck_subclass;
  std::unordered_map<int, TFlt> m_CO2_truck_subclass;
  std::unordered_map<int, TFlt> m_HC_truck_subclass;
  std::unordered_map<int, TFlt> m_CO_truck_subclass;
  std::unordered_map<int, TFlt> m_NOX_truck_subclass;
  std::unordered_map<int, TFlt> m_VMT_truck_subclass;
  std::unordered_map<int, TFlt> m_VMT_ev_truck_subclass;

  std::unordered_map<int, TFlt> m_VHT_car_subclass;
  std::unordered_map<int, TFlt> m_VHT_truck_subclass;

  // std::unordered_map<int, std::unordered_set<MNM_Veh*>> m_car_subclass_set;
  // std::unordered_map<int, std::unordered_set<MNM_Veh*>> m_truck_subclass_set;
};

/**************************************************************************
                            IO
**************************************************************************/
class MNM_IO_Multiclass_Subclass : public MNM_IO_Multiclass
{
public:
  static int build_demand_subclass (const std::string &file_folder,
                                      MNM_ConfReader *conf_reader,
                                      MNM_OD_Factory *od_factory,
                                      int mainclass_label,
                                      const std::string &file_name
                                      = "MNM_input_demand_car"); 
  static macposts::Graph build_graph (const std::string &graph_file_name,
                                      MNM_ConfReader *conf_reader);                                    
  static int build_graph_vec (const std::string &file_folder,
                              MNM_ConfReader *conf_reader,
                              int num_graph,
                              std::vector<macposts::Graph> &graph_vec);

  static int build_node_factory_multiclass_subclass (const std::string &file_folder,
                                                      MNM_ConfReader *conf_reader,
                                                      MNM_Node_Factory *node_factory,
                                                      const std::string &file_name
                                                      = "MNM_input_node");
  static int build_link_factory_multiclass_subclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_Link_Factory *link_factory,
                                            const std::string &file_name
                                            = "MNM_input_link");
  static int build_link_td_attribute_multiclass_subclass (const std::string &file_folder,
                                      MNM_Link_Factory *link_factory, int num_subclass_car, int num_subclass_truck,
                                      const std::string &file_name
                                      = "MNM_input_link_td_attribute");
};

namespace MNM
{
int print_vehicle_statistics (MNM_Veh_Factory_Multiclass_Subclass *veh_factory);
};

/**************************************************************************
                            DTA Gradient
**************************************************************************/
namespace MNM_DTA_GRADIENT
{
TFlt get_link_inflow_car_subclass (MNM_Dlink_Multiclass_Subclass *link, TInt start_time,
                                  TInt end_time, int veh_subclass);
TFlt get_link_outflow_car_subclass (MNM_Dlink_Multiclass_Subclass *link, TInt start_time,
                                    TInt end_time, int veh_subclass);
TFlt get_link_inflow_truck_subclass (MNM_Dlink_Multiclass_Subclass *link, TInt start_time,
                                    TInt end_time, int veh_subclass);
TFlt get_link_outflow_truck_subclass (MNM_Dlink_Multiclass_Subclass *link, TInt start_time,
                                      TInt end_time, int veh_subclass);

TFlt get_travel_time_car_subclass (MNM_Dlink_Multiclass_Subclass *link, TFlt start_time,
                                   TFlt unit_interval, TInt end_loading_timestamp, int veh_subclass);
TFlt get_travel_time_car_robust_subclass (MNM_Dlink_Multiclass_Subclass *link, TFlt start_time,
                                          TFlt end_time, TFlt unit_interval,
                                          TInt end_loading_timestamp, int veh_subclass,
                                          TInt num_trials = TInt (10));
TFlt get_travel_time_truck_subclass (MNM_Dlink_Multiclass_Subclass *link, TFlt start_time,
                                    TFlt unit_interval, TInt end_loading_timestamp, int veh_subclass);
TFlt get_travel_time_truck_robust_subclass (MNM_Dlink_Multiclass_Subclass *link, TFlt start_time,
                                            TFlt end_time, TFlt unit_interval,
                                            TInt end_loading_timestamp, int veh_subclass,
                                            TInt num_trials = TInt (10));
};

/**************************************************************************
                            DTA
**************************************************************************/
class MNM_Dta_Multiclass_Subclass : public MNM_Dta_Multiclass
{
public:
  explicit MNM_Dta_Multiclass_Subclass (const std::string &file_folder);
  virtual ~MNM_Dta_Multiclass_Subclass () override;
  virtual int initialize () override;
  virtual int build_from_files () override;
  virtual int set_routing () override;
  // virtual int set_statistics () override;
  // virtual int pre_loading () override;
  virtual int load_once (bool verbose, TInt load_int, TInt assign_int) override;
  // virtual int record_queue_vehicles () override;

  std::vector<macposts::Graph> m_graph_vec;
  std::unordered_map<int, int> m_subclass_car_graph_mapping, m_subclass_car_path_table_mapping;
  std::unordered_map<int, std::vector<int>> m_path_table_subclass_car_mapping;
  std::unordered_map<int, int> m_subclass_truck_graph_mapping, m_subclass_truck_path_table_mapping;
  std::unordered_map<int, std::vector<int>> m_path_table_subclass_truck_mapping; 
  int m_num_graph;
  int m_num_path_table;
  int m_num_subclass_car;
  int m_num_subclass_truck;
};