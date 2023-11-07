#pragma once

#include "dlink.h"
#include "dnode.h"
#include "dta.h"
#include "dta_gradient_utls.h"
#include "emission.h"
#include "limits.h"
#include "routing.h"
#include "vehicle.h"

#include <algorithm>
#include <fstream>
#include <iostream>

class MNM_Destination_Multiclass;

///
/// Link Models
///

class MNM_Dlink_Multiclass : public MNM_Dlink
{
public:
  MNM_Dlink_Multiclass (TInt ID, TInt number_of_lane, TFlt length, TFlt ffs_car,
                        TFlt ffs_truck);
  virtual ~MNM_Dlink_Multiclass () override;

  virtual int modify_property(TInt number_of_lane, TFlt length,
                              TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                              TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                              TFlt ffs_car, TFlt ffs_truck);

  virtual void print_info() override;

  // use this one instead of the one in Dlink class
  int install_cumulative_curve_multiclass ();
  // use this one instead of the one in Dlink class
  int install_cumulative_curve_tree_multiclass ();

  virtual TFlt get_link_flow_car () { return 0; };
  virtual TFlt get_link_flow_truck () { return 0; };

  virtual std::vector<TFlt> get_link_flow_emission_car (TInt ev_label)
  {
    std::vector<TFlt> _r = { TFlt (0), TFlt (0) };
    return _r;
  };
  virtual std::vector<TFlt> get_link_flow_emission_truck (TInt ev_label)
  {
    std::vector<TFlt> _r = { TFlt (0), TFlt (0) };
    return _r;
  };

  virtual TFlt get_link_tt_from_flow_car (TFlt flow) { return 0; };
  virtual TFlt get_link_tt_from_flow_truck (TFlt flow) { return 0; };
  TFlt get_link_freeflow_tt_car ();
  TFlt get_link_freeflow_tt_truck ();

  virtual TInt get_link_freeflow_tt_loading_car () { return -1; }; // intervals
  virtual TInt get_link_freeflow_tt_loading_truck ()
  {
    return -1;
  }; // intervals

  DLink_type_multiclass m_link_type;

  TFlt m_ffs_car;
  TFlt m_ffs_truck;

  TFlt m_tot_wait_time_at_intersection;       // seconds
  TFlt m_tot_wait_time_at_intersection_car;   // seconds
  TFlt m_tot_wait_time_at_intersection_truck; // seconds
  bool m_spill_back;

  TFlt m_toll_car;
  TFlt m_toll_truck;

  // Two seperate N-curves for private cars and trucks
  MNM_Cumulative_Curve *m_N_in_car;
  MNM_Cumulative_Curve *m_N_out_car;
  MNM_Cumulative_Curve *m_N_in_truck;
  MNM_Cumulative_Curve *m_N_out_truck;
  TFlt m_last_valid_time_truck = TFlt (-1);

  // Two seperate N-curve_trees for private cars and trucks
  MNM_Tree_Cumulative_Curve *m_N_in_tree_car;
  MNM_Tree_Cumulative_Curve *m_N_out_tree_car;
  MNM_Tree_Cumulative_Curve *m_N_in_tree_truck;
  MNM_Tree_Cumulative_Curve *m_N_out_tree_truck;
};

/// Multiclass CTM Functions
/// (currently only for car & truck two classes)
/// (see: Z. (Sean) Qian et al./Trans. Res. Part B 99 (2017) 183-204)

class MNM_Dlink_Ctm_Multiclass : public MNM_Dlink_Multiclass
{
public:
  MNM_Dlink_Ctm_Multiclass (TInt ID, TInt number_of_lane, TFlt length,
                            TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                            TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                            TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                            TFlt veh_convert_factor, TFlt flow_scalar);
  virtual ~MNM_Dlink_Ctm_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;

  virtual TFlt get_link_flow_car () override;
  virtual TFlt get_link_flow_truck () override;
  virtual TFlt get_link_flow () override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow_car (TFlt flow) override;
  virtual TFlt get_link_tt_from_flow_truck (TFlt flow) override;

  virtual std::vector<TFlt> get_link_flow_emission_car (TInt ev_label) override;
  virtual std::vector<TFlt>
  get_link_flow_emission_truck (TInt ev_label) override;

  virtual TInt get_link_freeflow_tt_loading_car () override;   // intervals
  virtual TInt get_link_freeflow_tt_loading_truck () override; // intervals

  virtual int move_veh_queue (std::deque<MNM_Veh *> *from_queue,
                              std::deque<MNM_Veh *> *to_queue,
                              TInt number_tomove) override;

  class Ctm_Cell_Multiclass;
  int init_cell_array (TFlt unit_time, TFlt std_cell_length,
                       TFlt last_cell_length);
  int update_out_veh ();
  int move_last_cell ();

  virtual int modify_property(TInt number_of_lane, TFlt length,
                              TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                              TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                              TFlt ffs_car, TFlt ffs_truck) override;

  TFlt m_unit_time;
  TInt m_num_cells;
  TFlt m_lane_hold_cap_car;
  TFlt m_lane_hold_cap_truck;
  TFlt m_lane_critical_density_car;
  TFlt m_lane_critical_density_truck;
  TFlt m_lane_rho_1_N;
  TFlt m_lane_flow_cap_car;
  TFlt m_lane_flow_cap_truck;
  TFlt m_veh_convert_factor;
  TFlt m_flow_scalar;
  TFlt m_wave_speed_car;
  TFlt m_wave_speed_truck;
  std::vector<Ctm_Cell_Multiclass *> m_cell_array;
};

class MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass
{
public:
  Ctm_Cell_Multiclass (TInt cell_ID, TFlt cell_length, TFlt unit_time,
                       TFlt hold_cap_car, TFlt hold_cap_truck,
                       TFlt critical_density_car, TFlt critical_density_truck,
                       TFlt rho_1_N, TFlt flow_cap_car, TFlt flow_cap_truck,
                       TFlt ffs_car, TFlt ffs_truck, TFlt wave_speed_car,
                       TFlt wave_speed_truck, TFlt flow_scalar);
  ~Ctm_Cell_Multiclass ();
  TFlt get_perceived_demand (TInt veh_type);
  TFlt get_perceived_supply (TInt veh_type);
  int update_perceived_density ();

  int modify_property(TFlt hold_cap_car, TFlt hold_cap_truck,
                       TFlt critical_density_car, TFlt critical_density_truck,
                       TFlt rho_1_N, TFlt flow_cap_car, TFlt flow_cap_truck,
                       TFlt ffs_car, TFlt ffs_truck, TFlt wave_speed_car,
                       TFlt wave_speed_truck);

  TInt m_cell_ID;
  TFlt m_cell_length;
  TFlt m_unit_time;
  TFlt m_flow_scalar;

  TFlt m_hold_cap_car;
  TFlt m_hold_cap_truck;
  TFlt m_critical_density_car;
  TFlt m_critical_density_truck;
  TFlt m_rho_1_N;
  TFlt m_flow_cap_car;
  TFlt m_flow_cap_truck;
  TFlt m_ffs_car;
  TFlt m_ffs_truck;
  TFlt m_wave_speed_car;
  TFlt m_wave_speed_truck;

  TInt m_volume_car;
  TInt m_volume_truck;
  TFlt m_space_fraction_car;
  TFlt m_space_fraction_truck;
  TFlt m_perceived_density_car;
  TFlt m_perceived_density_truck;
  TInt m_out_veh_car;
  TInt m_out_veh_truck;
  std::deque<MNM_Veh *> m_veh_queue_car;
  std::deque<MNM_Veh *> m_veh_queue_truck;
};

/// Multiclass Link-Queue Model

class MNM_Dlink_Lq_Multiclass : public MNM_Dlink_Multiclass
{
public:
  MNM_Dlink_Lq_Multiclass (TInt ID, TInt number_of_lane, TFlt length,
                           TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                           TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                           TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                           TFlt veh_convert_factor, TFlt flow_scalar);
  virtual ~MNM_Dlink_Lq_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;

  virtual TFlt get_link_flow_car () override;
  virtual TFlt get_link_flow_truck () override;
  virtual TFlt get_link_flow () override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow_car (TFlt flow) override;
  virtual TFlt get_link_tt_from_flow_truck (TFlt flow) override;

  virtual std::vector<TFlt> get_link_flow_emission_car (TInt ev_label) override;
  virtual std::vector<TFlt>
  get_link_flow_emission_truck (TInt ev_label) override;

  virtual TInt get_link_freeflow_tt_loading_car () override;   // intervals
  virtual TInt get_link_freeflow_tt_loading_truck () override; // intervals

  int update_perceived_density ();

  virtual int modify_property(TInt number_of_lane, TFlt length,
                            TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                            TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                            TFlt ffs_car, TFlt ffs_truck) override;

  std::deque<MNM_Veh *> m_veh_queue_car;
  std::deque<MNM_Veh *> m_veh_queue_truck;
  std::deque<MNM_Veh *> m_veh_out_buffer_car;
  std::deque<MNM_Veh *> m_veh_out_buffer_truck;

  TInt m_volume_car;   // vehicle number, without the flow scalar
  TInt m_volume_truck; // vehicle number, without the flow scalar
  TFlt m_flow_scalar;
  TFlt m_k_j_car;   // jam density
  TFlt m_k_j_truck; // jam density
  TFlt m_C_car;     // maximum free flow capacity
  TFlt m_C_truck;   // maximum free flow capacity
  TFlt m_k_C_car;   // maximum free flow density
  TFlt m_k_C_truck; // maximum free flow density
  TFlt m_w_car;     // backward wave speed
  TFlt m_w_truck;   // backward wave speed
  TFlt m_rho_1_N;

  TFlt m_space_fraction_car;
  TFlt m_space_fraction_truck;
  TFlt m_perceived_density_car;
  TFlt m_perceived_density_truck;

  TFlt m_unit_time;
  TFlt m_veh_convert_factor;
};

/// Multiclass Point-Queue Model

class MNM_Dlink_Pq_Multiclass : public MNM_Dlink_Multiclass
{
public:
  MNM_Dlink_Pq_Multiclass (TInt ID, TInt number_of_lane, TFlt length,
                           TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                           TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                           TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                           TFlt veh_convert_factor, TFlt flow_scalar);
  virtual ~MNM_Dlink_Pq_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;

  virtual TFlt get_link_flow_car () override;
  virtual TFlt get_link_flow_truck () override;
  virtual TFlt get_link_flow () override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow_car (TFlt flow) override;
  virtual TFlt get_link_tt_from_flow_truck (TFlt flow) override;

  virtual std::vector<TFlt> get_link_flow_emission_car (TInt ev_label) override;
  virtual std::vector<TFlt>
  get_link_flow_emission_truck (TInt ev_label) override;

  virtual TInt get_link_freeflow_tt_loading_car () override;   // intervals
  virtual TInt get_link_freeflow_tt_loading_truck () override; // intervals

  std::unordered_map<MNM_Veh *, TInt> m_veh_pool;
  TInt m_volume_car;   // vehicle number, without the flow scalar
  TInt m_volume_truck; // vehicle number, without the flow scalar
  TFlt m_lane_hold_cap;
  TFlt m_lane_flow_cap;
  TFlt m_flow_scalar;
  TFlt m_hold_cap;
  TInt m_max_stamp;
  TFlt m_unit_time;
  TFlt m_veh_convert_factor;
};

///
/// Node Models
///

/// Origin node

class MNM_DMOND_Multiclass : public MNM_DMOND
{
public:
  MNM_DMOND_Multiclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_DMOND_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  TFlt m_veh_convert_factor;
};

/// Destination node

class MNM_DMDND_Multiclass : public MNM_DMDND
{
public:
  MNM_DMDND_Multiclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_DMDND_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  TFlt m_veh_convert_factor;
};

/// In-out node

class MNM_Dnode_Inout_Multiclass : public MNM_Dnode
{
public:
  MNM_Dnode_Inout_Multiclass (TInt ID, TFlt flow_scalar,
                              TFlt veh_convert_factor);
  virtual ~MNM_Dnode_Inout_Multiclass () override;
  virtual int evolve (TInt timestamp) override;
  virtual int prepare_loading () override;
  virtual int add_out_link (MNM_Dlink *out_link) override;
  virtual int add_in_link (MNM_Dlink *in_link) override;

protected:
  int prepare_supplyANDdemand ();
  virtual int compute_flow () { return 0; };
  // int flow_to_vehicle();
  int move_vehicle (TInt timestamp);
  int record_cumulative_curve (TInt timestamp);
  TFlt *m_demand;          // 2d
  TFlt *m_supply;          // 1d
  TFlt *m_veh_flow;        // 2d
  TFlt *m_veh_moved_car;   // 2d
  TFlt *m_veh_moved_truck; // 2d
  TFlt m_veh_convert_factor;
};

/// FWJ node

class MNM_Dnode_FWJ_Multiclass : public MNM_Dnode_Inout_Multiclass
{
public:
  MNM_Dnode_FWJ_Multiclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_Dnode_FWJ_Multiclass () override;
  virtual int compute_flow () override;
};

/// General Road Junction node

class MNM_Dnode_GRJ_Multiclass : public MNM_Dnode_Inout_Multiclass
{
public:
  MNM_Dnode_GRJ_Multiclass (TInt ID, TFlt flow_scalar, TFlt veh_convert_factor);
  virtual ~MNM_Dnode_GRJ_Multiclass () override;
  virtual int compute_flow () override;
  virtual int prepare_loading () override;

private:
  std::vector<std::vector<MNM_Dlink *>> m_pow;
  TFlt get_theta ();
  int prepare_outflux ();
  TFlt *m_d_a; // 1d array
  TFlt *m_C_a; // 1d array
  template <typename T>
  std::vector<std::vector<T>> powerSet (const std::vector<T> &set);
  std::vector<int> getOnLocations (int a);
};

///
/// Multiclass OD
///

class MNM_Origin_Multiclass : public MNM_Origin
{
public:
  MNM_Origin_Multiclass (TInt ID, TInt max_interval, TFlt flow_scalar,
                         TInt frequency);
  virtual ~MNM_Origin_Multiclass () override;
  virtual TInt generate_label (TInt veh_class) override;
  virtual int release (MNM_Veh_Factory *veh_factory,
                       TInt current_interval) override;
  virtual int release_one_interval (TInt current_interval,
                                    MNM_Veh_Factory *veh_factory,
                                    TInt assign_interval,
                                    TFlt adaptive_ratio) override;

  virtual int release_one_interval_biclass (TInt current_interval,
                                            MNM_Veh_Factory *veh_factory,
                                            TInt assign_interval,
                                            TFlt adaptive_ratio_car,
                                            TFlt adaptive_ratio_truck) override;

  // use this one instead of add_dest_demand in the base class
  int add_dest_demand_multiclass (MNM_Destination_Multiclass *dest,
                                  TFlt *demand_car, TFlt *demand_truck);
  // two new unordered_map for both classes
  std::unordered_map<MNM_Destination_Multiclass *, TFlt *> m_demand_car;
  std::unordered_map<MNM_Destination_Multiclass *, TFlt *> m_demand_truck;

  std::vector<TFlt> m_car_label_ratio;
  std::vector<TFlt> m_truck_label_ratio;
};

class MNM_Destination_Multiclass : public MNM_Destination
{
public:
  explicit MNM_Destination_Multiclass (TInt ID);
  virtual ~MNM_Destination_Multiclass () override;
};

///
/// Multiclass Vehicle
///

class MNM_Veh_Multiclass : public MNM_Veh
{
public:
  MNM_Veh_Multiclass (TInt ID, TInt vehicle_class, TInt start_time);
  virtual ~MNM_Veh_Multiclass () override;

  virtual TInt get_class () override { return m_class; }; // virtual getter
  virtual TInt get_bus_route_ID () override
  {
    return m_bus_route_ID;
  };                                                    // virtual getter
  virtual bool get_ispnr () override { return m_pnr; }; // virtual getter
  virtual TInt get_label () override
  {
    return m_label;
  }; // virtual getter for derived class

  TInt m_class;
  
};

///
/// Multiclass Factory
///

class MNM_Veh_Factory_Multiclass : public MNM_Veh_Factory
{
public:
  MNM_Veh_Factory_Multiclass ();
  virtual ~MNM_Veh_Factory_Multiclass () override;

  // use this one instead of make_veh in the base class
  MNM_Veh_Multiclass *
  make_veh_multiclass (TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls);
  virtual int remove_finished_veh (MNM_Veh *veh, bool del = true) override;
  TInt m_num_car;
  TInt m_num_truck;
  TInt m_enroute_car;
  TInt m_enroute_truck;
  TInt m_finished_car;
  TInt m_finished_truck;
  TFlt m_total_time_car;   // intervals
  TFlt m_total_time_truck; // intervals
};

class MNM_Node_Factory_Multiclass : public MNM_Node_Factory
{
public:
  MNM_Node_Factory_Multiclass ();
  virtual ~MNM_Node_Factory_Multiclass () override;

  // use this one instead of make_node in the base class
  MNM_Dnode *make_node_multiclass (TInt ID, DNode_type_multiclass node_type,
                                   TFlt flow_scalar, TFlt veh_convert_factor);
};

struct td_link_attribute_row {
  std::string link_type;
  float length;
  float FFS_car;
  float Cap_car;
  float RHOJ_car;
  int Lane;
  float FFS_truck;
  float Cap_truck;
  float RHOJ_truck;
  float Convert_factor;
  float toll;
};

class MNM_Link_Factory_Multiclass : public MNM_Link_Factory
{
public:
  MNM_Link_Factory_Multiclass ();
  virtual ~MNM_Link_Factory_Multiclass () override;

  // use this one instead of make_link in the base class
  MNM_Dlink *make_link_multiclass (
    TInt ID, DLink_type_multiclass link_type, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
    TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar);

  virtual int update_link_attribute(TInt interval, bool verbose=false) override;

  std::unordered_map<int, std::unordered_map<int, td_link_attribute_row*>*>* m_td_link_attribute_table;

};

class MNM_OD_Factory_Multiclass : public MNM_OD_Factory
{
public:
  MNM_OD_Factory_Multiclass ();
  virtual ~MNM_OD_Factory_Multiclass () override;
  virtual MNM_Destination_Multiclass *make_destination (TInt ID) override;
  virtual MNM_Origin_Multiclass *make_origin (TInt ID, TInt max_interval,
                                              TFlt flow_scalar,
                                              TInt frequency) override;
  virtual std::pair<MNM_Origin *, MNM_Destination *>
  get_random_od_pair () override;
};

///
/// Multiclass IO Functions
///

class MNM_IO_Multiclass : public MNM_IO
{
public:
  static int build_node_factory_multiclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_Node_Factory *node_factory,
                                            const std::string &file_name
                                            = "MNM_input_node");
  static int build_link_factory_multiclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_Link_Factory *link_factory,
                                            const std::string &file_name
                                            = "MNM_input_link");
  static int build_demand_multiclass (const std::string &file_folder,
                                      MNM_ConfReader *conf_reader,
                                      MNM_OD_Factory *od_factory,
                                      const std::string &file_name
                                      = "MNM_input_demand");

  static int read_origin_car_label_ratio (const std::string &file_folder,
                                          MNM_ConfReader *conf_reader,
                                          MNM_OD_Factory *od_factory,
                                          const std::string &file_name
                                          = "MNM_origin_label_car");
  static int read_origin_truck_label_ratio (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            const std::string &file_name
                                            = "MNM_origin_label_truck");

  static int build_link_toll_multiclass (const std::string &file_folder,
                                         MNM_ConfReader *conf_reader,
                                         MNM_Link_Factory *link_factory,
                                         const std::string &file_name
                                         = "MNM_input_link_toll");
  static int build_link_td_attribute(const std::string &file_folder,
                                    MNM_Link_Factory *link_factory,
                                    const std::string &file_name = "MNM_input_link_td_attribute");
};

///
/// Multiclass DTA
///

class MNM_Dta_Multiclass : public MNM_Dta
{
public:
  explicit MNM_Dta_Multiclass (const std::string &file_folder);
  virtual ~MNM_Dta_Multiclass () override;
  virtual int initialize () override;
  virtual int build_from_files () override;
  virtual int pre_loading () override;
  virtual int record_queue_vehicles () override;
  int loading_vehicle_tracking(bool verbose, const std::string &folder, double sampling_rate, int frequency);

  std::unordered_map<TInt, std::deque<TInt> *> m_queue_veh_map_car;
  std::unordered_map<TInt, std::deque<TInt> *> m_queue_veh_map_truck;
};

///
/// Multiclass DTA Gradient Utils
///

namespace MNM_DTA_GRADIENT
{
TFlt get_link_inflow_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                          TFlt end_time);
TFlt get_link_inflow_car (MNM_Dlink_Multiclass *link, TInt start_time,
                          TInt end_time);
TFlt get_link_outflow_car (MNM_Dlink_Multiclass *link, TInt start_time,
                          TInt end_time);                          
TFlt get_link_inflow_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                            TFlt end_time);
TFlt get_link_inflow_truck (MNM_Dlink_Multiclass *link, TInt start_time,
                            TInt end_time);
TFlt get_link_outflow_truck (MNM_Dlink_Multiclass *link, TInt start_time,
                            TInt end_time);
TFlt get_average_waiting_time_at_intersection (MNM_Dlink_Multiclass *link);
TFlt get_average_waiting_time_at_intersection_car (MNM_Dlink_Multiclass *link);
TFlt
get_average_waiting_time_at_intersection_truck (MNM_Dlink_Multiclass *link);
TInt get_is_spillback (
  MNM_Dlink_Multiclass *link); // 0 - no spillback, 1 - spillback

TFlt get_travel_time_from_FD_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                                  TFlt unit_interval);
TFlt get_travel_time_from_FD_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                                    TFlt unit_interval);

TFlt get_travel_time_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                          TFlt unit_interval, TInt end_loading_timestamp);
TFlt get_travel_time_car_robust (MNM_Dlink_Multiclass *link, TFlt start_time,
                                 TFlt end_time, TFlt unit_interval,
                                 TInt end_loading_timestamp,
                                 TInt num_trials = TInt (10));
TFlt get_travel_time_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                            TFlt unit_interval, TInt end_loading_timestamp);
TFlt get_travel_time_truck_robust (MNM_Dlink_Multiclass *link, TFlt start_time,
                                   TFlt end_time, TFlt unit_interval,
                                   TInt end_loading_timestamp,
                                   TInt num_trials = TInt (10));

TFlt get_path_travel_time_car (MNM_Path *path, TFlt start_time,
                               MNM_Link_Factory *link_factory,
                               TFlt unit_interval, TInt end_loading_timestamp);
TFlt
get_path_travel_time_car (MNM_Path *path, TFlt start_time,
                          std::unordered_map<TInt, TFlt *> &link_tt_map_car,
                          TInt end_loading_timestamp);
TFlt get_path_travel_time_truck (MNM_Path *path, TFlt start_time,
                                 MNM_Link_Factory *link_factory,
                                 TFlt unit_interval,
                                 TInt end_loading_timestamp);
TFlt
get_path_travel_time_truck (MNM_Path *path, TFlt start_time,
                            std::unordered_map<TInt, TFlt *> &link_tt_map_truck,
                            TInt end_loading_timestamp);

int add_dar_records_car (std::vector<dar_record *> &record,
                         MNM_Dlink_Multiclass *link,
                         std::set<MNM_Path *> pathset, TFlt start_time,
                         TFlt end_time);
int add_dar_records_truck (std::vector<dar_record *> &record,
                           MNM_Dlink_Multiclass *link,
                           std::set<MNM_Path *> pathset, TFlt start_time,
                           TFlt end_time);
int add_dar_records_car (std::vector<dar_record *> &record,
                         MNM_Dlink_Multiclass *link, std::set<TInt> pathID_set,
                         TFlt start_time, TFlt end_time);
int add_dar_records_truck (std::vector<dar_record *> &record,
                           MNM_Dlink_Multiclass *link,
                           std::set<TInt> pathID_set, TFlt start_time,
                           TFlt end_time);

int add_dar_records_eigen_car (std::vector<Eigen::Triplet<double>> &record,
                               MNM_Dlink_Multiclass *link,
                               std::set<MNM_Path *> pathset, TFlt start_time,
                               TFlt end_time, int link_ind, int interval_ind,
                               int num_of_minute, int num_e_link,
                               int num_e_path, const double *f_ptr);

int add_dar_records_eigen_car (
  Eigen::SparseMatrix<double, Eigen::RowMajor> &mat, MNM_Dlink_Multiclass *link,
  std::set<MNM_Path *> pathset, TFlt start_time, TFlt end_time, int link_ind,
  int interval_ind, int num_of_minute, int num_e_link, int num_e_path,
  const double *f_ptr);

int add_dar_records_eigen_truck (std::vector<Eigen::Triplet<double>> &record,
                                 MNM_Dlink_Multiclass *link,
                                 std::set<MNM_Path *> pathset, TFlt start_time,
                                 TFlt end_time, int link_ind, int interval_ind,
                                 int num_of_minute, int num_e_link,
                                 int num_e_path, const double *f_ptr);

int add_dar_records_eigen_truck (
  Eigen::SparseMatrix<double, Eigen::RowMajor> &mat, MNM_Dlink_Multiclass *link,
  std::set<MNM_Path *> pathset, TFlt start_time, TFlt end_time, int link_ind,
  int interval_ind, int num_of_minute, int num_e_link, int num_e_path,
  const double *f_ptr);

TFlt get_departure_cc_slope_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                                 TFlt end_time);
TFlt get_departure_cc_slope_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                                   TFlt end_time);

int add_ltg_records_veh (std::vector<ltg_record *> &record,
                         MNM_Dlink_Multiclass *link, MNM_Path *path,
                         int depart_time, int start_time, TFlt gradient);

int add_ltg_records_eigen_veh (std::vector<Eigen::Triplet<double>> &record,
                               MNM_Path *path, int depart_time, int start_time,
                               int link_ind, int assign_interval,
                               int num_e_link, int num_e_path, TFlt gradient);

};

namespace MNM
{
int print_vehicle_statistics (MNM_Veh_Factory_Multiclass *veh_factory);

Path_Table *build_pathset_multiclass (
  PNEGraph &graph, MNM_OD_Factory *od_factory, MNM_Link_Factory *link_factory,
  TFlt min_path_length = 0.0, size_t MaxIter = 10, TFlt vot = 6.,
  TFlt Mid_Scale = 3, TFlt Heavy_Scale = 6, TInt buffer_length = -1);

int print_vehicle_route_results(MNM_Veh_Factory_Multiclass *veh_factory,
                                const std::string &folder,
                                int interval,
                                double sampling_rate,
                                int cong_frequency,
                                bool verbose=false);
};

///
/// Multiclass emissions
///

class MNM_Cumulative_Emission_Multiclass : public MNM_Cumulative_Emission
{
public:
  // -1 is the default veh -> m_label value, use -2 ad ev label
  MNM_Cumulative_Emission_Multiclass (TFlt unit_time, TInt freq,
                                      TInt ev_label_car = -2,
                                      TInt ev_label_truck = -2);
  virtual ~MNM_Cumulative_Emission_Multiclass () override;

  // new functions for trucks
  TFlt calculate_fuel_rate_truck (TFlt v);
  TFlt calculate_CO2_rate_truck (TFlt v);
  TFlt calculate_HC_rate_truck (TFlt v);
  TFlt calculate_CO_rate_truck (TFlt v);
  TFlt calculate_NOX_rate_truck (TFlt v);

  virtual int update (MNM_Veh_Factory *veh_factory) override;
  virtual std::string output () override;

  TFlt m_fuel_truck;
  TFlt m_CO2_truck;
  TFlt m_HC_truck;
  TFlt m_CO_truck;
  TFlt m_NOX_truck;
  TFlt m_VMT_truck;
  TFlt m_VMT_ev_truck;

  TFlt m_VHT_truck;
  TFlt m_VHT_car;

  TInt m_ev_label_truck;

  std::unordered_set<MNM_Veh *> m_car_set;
  std::unordered_set<MNM_Veh *> m_truck_set;
};
