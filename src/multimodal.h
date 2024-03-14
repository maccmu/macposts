//
// Created by qiling on 2/18/21.
//

#pragma once

#include "due.h"
#include "multiclass.h"
#include "path.h"
#include <iostream>

class MNM_Busstop_Virtual;
class MNM_Routing_Multimodal_Hybrid;
class MNM_Origin_Multimodal;
class MNM_Destination_Multimodal;
class MNM_Transit_Link;
class MNM_Bus_Link;
class MNM_Veh_Multimodal;
class MNM_Passenger;
class MNM_Passenger_Factory;
class MNM_Walking_Link;
class MNM_Parking_Lot;
class MNM_Parking_Lot_Factory;
class MNM_PnR_Path;
class MNM_MM_Due;
using namespace MNM;
using namespace MNM_DTA_GRADIENT;

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Stop Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                Base Bus Stop
**************************************************************************/
class MNM_Busstop
{
public:
  MNM_Busstop (TInt ID, TInt linkID, TFlt linkloc, TFlt flow_scalar);
  virtual ~MNM_Busstop ();

  virtual bool hold_bus (MNM_Veh *veh, MNM_Veh_Multimodal *veh_multimodal,
                         std::deque<MNM_Veh *> *from_queue,
                         std::deque<MNM_Veh *> *held_queue, int flow_scalar)
  {
    return false;
  };
  virtual int release_bus (TInt timestamp, MNM_Veh_Multimodal *veh_multimodal)
  {
    return 0;
  };
  virtual int receive_bus (TInt timestamp, MNM_Veh_Multimodal *veh_multimodal)
  {
    return 0;
  };
  virtual int update_routing_passenger (TInt timestamp) { return 0; };
  virtual int evolve (TInt timestamp) { return 0; };

  TInt m_busstop_ID;
  TInt m_link_ID;
  TFlt m_link_loc;
  TInt m_cell_ID;
  TFlt m_flow_scalar;

  MNM_Routing_Multimodal_Hybrid *m_routing;
};

/**************************************************************************
                                                Physical Bus Stop
**************************************************************************/
class MNM_Busstop_Physical : public MNM_Busstop
{
public:
  MNM_Busstop_Physical (TInt ID, TInt linkID, TFlt linkloc, TFlt flow_scalar);
  virtual ~MNM_Busstop_Physical () override;

  virtual int evolve (TInt timestamp) override;

  std::vector<TInt> m_route_IDs_vec;
  std::vector<MNM_Busstop_Virtual *> m_busstop_virtual_vec;
  std::vector<MNM_Walking_Link *> m_boarding_links_vec;
  std::vector<MNM_Walking_Link *> m_alighting_links_vec;
  std::vector<MNM_Walking_Link *> m_walking_in_links_vec;
  std::vector<MNM_Walking_Link *> m_walking_out_links_vec;
};

/**************************************************************************
                                                Virtual Bus Stop
**************************************************************************/
class MNM_Busstop_Virtual : public MNM_Busstop
{
public:
  MNM_Busstop_Virtual (TInt ID, TInt linkID, TFlt linkloc, TFlt flow_scalar);
  virtual ~MNM_Busstop_Virtual () override;
  int install_cumulative_curve_multiclass ();

  virtual bool hold_bus (MNM_Veh *veh, MNM_Veh_Multimodal *veh_multimodal,
                         std::deque<MNM_Veh *> *from_queue,
                         std::deque<MNM_Veh *> *held_queue,
                         int flow_scalar) override;
  virtual int release_bus (TInt timestamp,
                           MNM_Veh_Multimodal *veh_multimodal) override;
  virtual int receive_bus (TInt timestamp,
                           MNM_Veh_Multimodal *veh_multimodal) override;
  virtual int update_routing_passenger (TInt timestamp) override;

  TFlt get_waiting_time_snapshot (TInt timestamp); // based on network snapshot

  // use historical bus waiting time, passenger don't need to rely on the bus to
  // traverse the bus link
  int virtual_evolve (TInt timestamp);

  TInt m_route_ID;
  MNM_Busstop_Physical *m_busstop_physical;
  TInt m_passed_bus_counter;
  std::deque<MNM_Veh_Multimodal *> m_bus_queue;
  MNM_Bus_Link *m_bus_in_link;
  MNM_Bus_Link *m_bus_out_link;
  MNM_Walking_Link *m_boarding_link;
  MNM_Walking_Link *m_alighting_link;
  MNM_Cumulative_Curve *m_N_in_bus;
  MNM_Cumulative_Curve *m_N_out_bus;
  TFlt m_last_valid_time = TFlt (-1);

  TFlt m_total_bus = -1;
  MNM_Origin_Multimodal *m_origin = nullptr;
  MNM_Destination_Multimodal *m_dest = nullptr;

  TInt m_historical_waiting_time = -1; // intervals
  TInt m_max_alighting_passengers_per_unit_time;
  TInt m_max_boarding_passengers_per_unit_time;

  //    MNM_Cumulative_Curve *m_N_in_car;
  //    MNM_Cumulative_Curve *m_N_out_car;
  //    MNM_Cumulative_Curve *m_N_in_truck;
  //    MNM_Cumulative_Curve *m_N_out_truck;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Stop Factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Busstop_Factory
{
public:
  MNM_Busstop_Factory ();
  virtual ~MNM_Busstop_Factory ();
  MNM_Busstop *make_busstop (TInt ID, TInt linkID, TFlt linkloc,
                             TFlt flow_scalar, const std::string &busstop_type);
  MNM_Busstop *get_busstop (TInt ID);

  std::unordered_map<TInt, MNM_Busstop *> m_busstop_map;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Parking lot
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Parking_Lot
{
public:
  MNM_Parking_Lot (TInt ID, TInt node_ID,
                   MNM_Passenger_Factory *passenger_factory,
                   MNM_Veh_Factory *veh_factory, TFlt base_price,
                   TFlt price_surge_coeff, TFlt avg_parking_time, TFlt capacity,
                   TFlt unit_time);
  virtual ~MNM_Parking_Lot ();
  int release_one_interval_passenger (
    TInt timestamp, MNM_Routing_Multimodal_Hybrid *routing = nullptr,
    bool del = false); // invoked in MNM_Destination_Multimodal::receive()
  int evolve (TInt timestamp);
  TFlt get_cruise_time (TInt timestamp); // intervals

  TInt m_ID;
  TFlt m_base_price;
  TFlt m_price_surge_coeff;
  TFlt m_unit_time;
  // intervals
  TFlt m_avg_parking_time;
  TFlt m_max_parking_time;
  // time-dependent occupancy
  TFlt m_occupancy;
  TFlt m_capacity;
  // dest node ID
  TInt m_node_ID;
  MNM_DMDND_Multiclass *m_dest_node;
  MNM_Veh_Factory *m_veh_factory;
  MNM_Passenger_Factory *m_passenger_factory;
  std::vector<MNM_Walking_Link *> m_walking_out_links_vec;

  // <timestamp, cruising time in intervals>
  std::unordered_map<TInt, TFlt> m_cruising_time_record;
  // parked cars
  // std::deque<MNM_Veh_Multimodal *> m_parked_car_queue;
  TInt m_parked_car;
  // pnr passengers about to leave the parking lot
  std::deque<MNM_Passenger *> m_in_passenger_queue;
  // <path_ID, cumulative counter> for converting pnr vehicles to passenger, the
  // path_ID of adaptive routing is -1
  std::unordered_map<TInt, TInt> m_pnr_fixed_routing_counter;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Parking lot factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Parking_Lot_Factory
{
public:
  MNM_Parking_Lot_Factory ();
  virtual ~MNM_Parking_Lot_Factory ();
  MNM_Parking_Lot *make_parking_lot (TInt ID, TInt node_ID,
                                     MNM_Passenger_Factory *passenger_factory,
                                     MNM_Veh_Factory *veh_factory,
                                     TFlt base_price, TFlt price_surge_coeff,
                                     TFlt avg_parking_time, TFlt capacity,
                                     TFlt unit_time);
  MNM_Parking_Lot *get_parking_lot (TInt ID);
  std::unordered_map<TInt, MNM_Parking_Lot *> m_parking_lot_map;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                            Passenger
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Passenger
{
public:
  MNM_Passenger (TInt ID, TInt start_time, TInt passenger_type);
  ~MNM_Passenger ();

  int set_current_link (MNM_Transit_Link *link);
  MNM_Transit_Link *get_current_link ();
  MNM_Transit_Link *get_next_link ();
  int set_next_link (MNM_Transit_Link *link);
  bool has_next_link ();
  MNM_Destination *get_destination ();
  int set_destination (MNM_Destination *dest);
  int finish (TInt finish_time);
  MNM_Origin *get_origin ();
  int set_origin (MNM_Origin *origin);

  // private:
  TInt m_passenger_ID;
  TInt m_passenger_type;
  bool m_pnr;
  TFlt m_waiting_time; // intervals, cruising time in parking lot
  MNM_Transit_Link *m_current_link;
  TInt m_start_time;
  TInt m_finish_time;
  MNM_Transit_Link *m_next_link;
  MNM_Destination *m_dest;
  MNM_Origin *m_origin;
  MNM_Parking_Lot *m_parking_lot; // for pnr
  MNM_Path *m_driving_path;
  MNM_Path *m_transit_path;
  MNM_PnR_Path *m_pnr_path;
  TInt m_assign_interval;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                         Passenger  Factory
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Passenger_Factory
{
public:
  MNM_Passenger_Factory ();
  ~MNM_Passenger_Factory ();

  MNM_Passenger *make_passenger (TInt timestamp, TInt passenger_type);
  MNM_Passenger *get_passenger (TInt ID);
  int remove_finished_passenger (MNM_Passenger *passenger, bool del = true);

  std::unordered_map<TInt, MNM_Passenger *> m_passenger_map;

  TInt m_num_passenger;
  TInt m_enroute_passenger;
  TInt m_finished_passenger;
  TFlt m_total_time_passenger;

  TInt m_num_passenger_pnr;
  TInt m_enroute_passenger_pnr;
  TInt m_finished_passenger_pnr;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                          Multimodal Vehicle
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Multimodal : public MNM_Veh_Multiclass
{
public:
  MNM_Veh_Multimodal (TInt ID, TInt vehicle_class, TInt start_time,
                      TInt capacity = 1, TInt bus_route_ID = TInt (-1),
                      bool is_pnr = false);
  virtual ~MNM_Veh_Multimodal () override;

  int board_and_alight (TInt timestamp, MNM_Busstop *busstop);

  virtual TInt get_class () override { return m_class; }; // virtual getter
  virtual TInt get_bus_route_ID () override
  {
    return m_bus_route_ID;
  };                                                      // virtual getter
  virtual bool get_ispnr () override { return m_pnr; };   // virtual getter
  virtual TInt get_label () override { return m_label; }; // virtual getter

  TInt m_capacity;
  TInt m_bus_route_ID;
  TInt m_min_dwell_intervals;
  TInt m_stopped_intervals;
  TInt m_boarding_lost_intervals;
  TInt m_max_alighting_passengers_per_unit_time;
  TInt m_max_boarding_passengers_per_unit_time;

  bool m_pnr;
  TFlt m_waiting_time; // intervals, pickup waiting time at origins for mobility
                       // service vehicles
  MNM_PnR_Path *m_pnr_path;
  MNM_Path *m_transit_path;
  std::deque<MNM_Passenger *> m_passenger_pool;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Veh_Factory_Multimodal : public MNM_Veh_Factory_Multiclass
{
public:
  MNM_Veh_Factory_Multimodal (TInt bus_capacity, TInt min_dwell_intervals,
                              TInt boarding_lost_intervals,
                              TInt max_alighting_passengers_per_unit_time,
                              TInt max_boarding_passengers_per_unit_time);
  virtual ~MNM_Veh_Factory_Multimodal () override;

  MNM_Veh_Multimodal *
  make_veh_multimodal (TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls,
                       TInt capacity = TInt (1), TInt bus_route = TInt (-1),
                       bool is_pnr = false,
                       TInt pickup_waiting_time = TInt (0));

  virtual int remove_finished_veh (MNM_Veh *veh, bool del = true) override;

  TInt m_bus_capacity;
  TInt m_min_dwell_intervals;
  TInt m_boarding_lost_intervals;
  TInt m_max_alighting_passengers_per_unit_time;
  TInt m_max_boarding_passengers_per_unit_time;

  TInt m_num_bus;
  TInt m_enroute_bus;
  TInt m_finished_bus;

  TFlt m_total_time_bus;

  TInt m_num_car_pnr;
  TInt m_enroute_car_pnr;
  TInt m_finished_car_pnr;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal OD
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Destination_Multimodal : public MNM_Destination_Multiclass
{
public:
  explicit MNM_Destination_Multimodal (TInt ID);
  virtual ~MNM_Destination_Multimodal () override;
  int evolve (TInt timestamp);
  virtual int receive (TInt timestamp) override;
  int receive (TInt timestamp, MNM_Routing_Multimodal_Hybrid *routing,
               MNM_Veh_Factory *veh_factory,
               MNM_Passenger_Factory *passenger_factory, bool del = true);

  MNM_Parking_Lot *m_parking_lot;
  std::deque<MNM_Passenger *> m_out_passenger_queue;
  std::vector<MNM_Walking_Link *> m_walking_in_links_vec;
  std::vector<MNM_Parking_Lot *> m_connected_pnr_parkinglot_vec;
};

class MNM_Origin_Multimodal : public MNM_Origin_Multiclass
{
public:
  MNM_Origin_Multimodal (TInt ID, TInt max_interval, TFlt flow_scalar,
                         TInt frequency, TInt pickup_waiting_time = TInt (0));
  virtual ~MNM_Origin_Multimodal () override;
  int evolve (TInt timestamp);

  virtual int release_one_interval (TInt current_interval,
                                    MNM_Veh_Factory *veh_factory,
                                    TInt assign_interval,
                                    TFlt adaptive_ratio) override;

  virtual int release_one_interval_biclass (TInt current_interval,
                                            MNM_Veh_Factory *veh_factory,
                                            TInt assign_interval,
                                            TFlt adaptive_ratio_car,
                                            TFlt adaptive_ratio_truck) override;

  int release_one_interval_passenger (TInt current_interval,
                                      MNM_Passenger_Factory *passenger_factory,
                                      TInt assign_interval,
                                      TFlt adaptive_ratio);

  int add_dest_demand_bus (MNM_Destination_Multimodal *dest, TInt routeID,
                           TFlt *demand_bus);

  int add_dest_demand_pnr_car (MNM_Destination_Multimodal *dest,
                               TFlt *demand_pnr_car);

  int add_dest_demand_passenger_bus (MNM_Destination_Multimodal *dest,
                                     TFlt *demand_passenger_bus);

  // <Destination node, <routeID, time-varying demand>>
  std::unordered_map<MNM_Destination_Multimodal *,
                     std::unordered_map<TInt, TFlt *>>
    m_demand_bus;
  // <Destination node, time-varying demand>
  std::unordered_map<MNM_Destination_Multimodal *, TFlt *> m_demand_pnr_car;
  // <Destination node, time-varying demand>
  std::unordered_map<MNM_Destination_Multimodal *, TFlt *>
    m_demand_passenger_bus;

  std::deque<MNM_Passenger *> m_in_passenger_queue;
  std::vector<MNM_Walking_Link *> m_walking_out_links_vec;

  TInt m_pickup_waiting_time; // for mobility service
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                OD Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_OD_Factory_Multimodal : public MNM_OD_Factory_Multiclass
{
public:
  MNM_OD_Factory_Multimodal ();
  virtual ~MNM_OD_Factory_Multimodal () override;
  virtual MNM_Destination_Multimodal *make_destination (TInt ID) override;
  virtual MNM_Origin_Multimodal *make_origin (TInt ID, TInt max_interval,
                                              TFlt flow_scalar,
                                              TInt frequency) override;
  virtual std::pair<MNM_Origin *, MNM_Destination *>
  get_random_od_pair () override;
  std::pair<MNM_Origin *, MNM_Destination *> get_random_od_pair_bustransit ();
  std::pair<MNM_Origin *, MNM_Destination *> get_random_od_pair_pnr ();
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Node Factory
*******************************************************************************************************************
******************************************************************************************************************/

class MNM_Node_Factory_Multimodal : public MNM_Node_Factory_Multiclass
{
public:
  MNM_Node_Factory_Multimodal ();
  virtual ~MNM_Node_Factory_Multimodal () override;

  // use this one instead of make_node in the base class
  MNM_Dnode *make_node_multimodal (TInt ID, DNode_type_multimodal node_type,
                                   TFlt flow_scalar, TFlt veh_convert_factor);
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Link Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                Transit Link Model
**************************************************************************/
class MNM_Transit_Link
{
public:
  MNM_Transit_Link (TInt link_ID, DLink_type_multimodal link_type,
                    const std::string &from_node_type,
                    const std::string &to_node_type, TInt from_node_ID,
                    TInt to_node_ID, TFlt unit_time);
  virtual ~MNM_Transit_Link ();
  int install_cumulative_curve ();
  virtual int install_cumulative_curve_tree ();
  virtual int clear_incoming_array (TInt timestamp) { return 0; };
  virtual int evolve (TInt timestamp) { return 0; };
  virtual TFlt get_link_tt (bool count_runs = true)
  {
    return TFlt (0);
  }; // real-time tt in seconds

  virtual TInt get_link_freeflow_tt_loading_passenger ()
  {
    return -1;
  }; // intervals

  TInt m_link_ID;
  DLink_type_multimodal m_link_type;
  // possible node-node pair: origin -> bus_stop, parking_lot -> bus_stop,
  // bus_stop -> destination, bus_stop -> bus_stop, parking_lot -> destination
  std::string m_from_node_type;
  std::string m_to_node_type;
  TInt m_from_node_ID;
  TInt m_to_node_ID;
  TFlt m_fftt; // seconds
  TFlt m_unit_time;

  std::deque<MNM_Passenger *> m_incoming_array;
  // <passenger, time left to be moved to finished queue>
  std::unordered_map<MNM_Passenger *, TInt> m_passenger_queue;
  std::deque<MNM_Passenger *> m_finished_array;

  // recording passengers dar
  MNM_Cumulative_Curve *m_N_in;
  MNM_Cumulative_Curve *m_N_out;
  TFlt m_last_valid_time = TFlt (-1);
  MNM_Tree_Cumulative_Curve *m_N_in_tree;
  MNM_Tree_Cumulative_Curve *m_N_out_tree;
};

/**************************************************************************
                                                Bus Link Model
**************************************************************************/
class MNM_Bus_Link : public MNM_Transit_Link
{
public:
  MNM_Bus_Link (TInt link_ID, TInt from_busstop_ID, TInt to_busstop_ID,
                TFlt bus_fftt, TFlt unit_time);
  virtual ~MNM_Bus_Link () override;
  virtual int install_cumulative_curve_tree () override;
  virtual TFlt get_link_tt (bool count_runs
                            = true) override; // real-time tt in seconds
  int get_overlapped_driving_link_length_portion ();
  // use historical avarage waiting time
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual int evolve (TInt timestamp) override;

  TInt m_route_ID;
  MNM_Busstop_Virtual *m_from_busstop;
  MNM_Busstop_Virtual *m_to_busstop;
  TFlt m_last_valid_time_bus = TFlt (-1);
  std::vector<MNM_Dlink *> m_overlapped_driving_link_vec;
  std::vector<TFlt> m_overlapped_driving_link_length_portion_vec;
  TFlt m_length;

  // recording passengers dar
  MNM_Tree_Cumulative_Curve *m_N_in_tree_bus;
  MNM_Tree_Cumulative_Curve *m_N_out_tree_bus;
};

/**************************************************************************
                                                Walking Link Model
**************************************************************************/
class MNM_Walking_Link : public MNM_Transit_Link
{
public:
  MNM_Walking_Link (TInt ID, const std::string &walking_type,
                    const std::string &from_node_type,
                    const std::string &to_node_type, TInt from_node_ID,
                    TInt to_node_ID, TFlt walking_time, TFlt unit_time);
  virtual ~MNM_Walking_Link () override;
  virtual TFlt get_link_tt (bool count_runs
                            = true) override; // real-time tt in seconds
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual int evolve (TInt timestamp) override;

  TInt m_max_stamp;
  std::string m_walking_type;
  TInt m_historical_bus_waiting_time = 0;
};

/**************************************************************************
                                        Multimodal Point-Queue Model
**************************************************************************/
class MNM_Dlink_Pq_Multimodal : public MNM_Dlink_Pq_Multiclass
{
public:
  MNM_Dlink_Pq_Multimodal (TInt ID, TInt number_of_lane, TFlt length,
                           TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                           TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                           TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                           TFlt veh_convert_factor, TFlt flow_scalar);
  virtual ~MNM_Dlink_Pq_Multimodal () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_flow_car () override;
  virtual TFlt get_link_flow_truck () override;

  std::deque<std::pair<MNM_Veh *, TInt>> m_veh_queue;
  // for multimodal with busstops on the link
  DLink_type_multimodal m_link_type;
  std::vector<MNM_Busstop_Virtual *> m_busstop_vec;
  std::unordered_map<MNM_Busstop_Virtual *, TInt> m_busstop_timeloc_map;
};

/**************************************************************************
                                        Multimodal CTM Model
**************************************************************************/
class MNM_Dlink_Ctm_Multimodal : public MNM_Dlink_Ctm_Multiclass
{
public:
  MNM_Dlink_Ctm_Multimodal (TInt ID, TInt number_of_lane, TFlt length,
                            TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck,
                            TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
                            TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
                            TFlt veh_convert_factor, TFlt flow_scalar);
  virtual ~MNM_Dlink_Ctm_Multimodal () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_flow_car () override;
  virtual TFlt get_link_flow_truck () override;

  int move_veh_queue_in_cell (std::deque<MNM_Veh *> *from_queue,
                              std::deque<MNM_Veh *> *to_queue,
                              TInt number_tomove, TInt timestamp, TInt cell_ID);

  int move_veh_queue_in_last_cell (TInt timestamp);

  DLink_type_multimodal m_link_type;
  // <cell_ID, busstop_vec>
  std::unordered_map<TInt, std::vector<MNM_Busstop_Virtual *>>
    m_cell_busstop_vec;
  // for multimodal with busstops on the link
  std::vector<MNM_Busstop_Virtual *> m_busstop_vec;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                        Link Factory
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                transit link factory
**************************************************************************/
class MNM_Transit_Link_Factory
{
public:
  MNM_Transit_Link_Factory ();
  ~MNM_Transit_Link_Factory ();

  MNM_Transit_Link *make_transit_link (TInt ID, DLink_type_multimodal link_type,
                                       const std::string &walking_type,
                                       const std::string &from_node_type,
                                       const std::string &to_node_type,
                                       TInt from_node_ID, TInt to_node_ID,
                                       TFlt fftt, TFlt unit_time);
  MNM_Transit_Link *get_transit_link (TInt link_ID);

  std::unordered_map<TInt, MNM_Transit_Link *> m_transit_link_map;
};

/**************************************************************************
                                                driving link factory
**************************************************************************/
class MNM_Link_Factory_Multimodal : public MNM_Link_Factory_Multiclass
{
public:
  MNM_Link_Factory_Multimodal ();
  virtual ~MNM_Link_Factory_Multimodal () override;

  // use this one instead of make_link in the base class
  MNM_Dlink *make_link_multimodal (
    TInt ID, DLink_type_multimodal link_type, TInt number_of_lane, TFlt length,
    TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
    TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
    TFlt veh_convert_factor, TFlt flow_scalar);
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Statistics
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Statistics_Lrn_Multimodal : public MNM_Statistics_Lrn_Multiclass
{
public:
  MNM_Statistics_Lrn_Multimodal (const std::string &file_folder,
                                 MNM_ConfReader *conf_reader,
                                 MNM_ConfReader *record_config,
                                 MNM_OD_Factory *od_factory,
                                 MNM_Node_Factory *node_factory,
                                 MNM_Link_Factory *link_factory,
                                 MNM_Transit_Link_Factory *transitlink_factory);
  virtual ~MNM_Statistics_Lrn_Multimodal () override;

  virtual int record_loading_interval_condition (TInt timestamp) override;
  virtual int record_record_interval_condition (TInt timestamp) override;
  virtual int update_record (TInt timestamp) override;
  virtual int init_record () override;
  virtual int post_record () override;

  std::unordered_map<TInt, TFlt> m_load_interval_tt_bus_transit;
  std::unordered_map<TInt, TFlt> m_record_interval_tt_bus_transit;
  std::unordered_map<TInt, TFlt> m_to_be_tt_bus_transit;

  std::vector<MNM_Transit_Link *> m_transitlink_order;

private:
  MNM_Transit_Link_Factory *m_transitlink_factory;
  std::ofstream m_load_interval_tt_bus_transit_file;
  std::ofstream m_record_interval_tt_bus_transit_file;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Path
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_BusPath : public MNM_Path
{
public:
  MNM_BusPath (TInt route_ID);
  virtual ~MNM_BusPath () override;

  std::deque<TInt> m_busstop_vec;
  TInt m_route_ID;

  int get_busstop_index (MNM_Busstop_Virtual *busstop);
  TFlt get_busroute_fftt (MNM_Link_Factory *link_factory,
                          MNM_Busstop_Virtual *start_busstop,
                          MNM_Busstop_Virtual *end_busstop, TFlt unit_interval);
  TFlt get_busroute_tt (TFlt start_time, MNM_Link_Factory *link_factory,
                        MNM_Busstop_Virtual *start_busstop,
                        MNM_Busstop_Virtual *end_busstop, TFlt unit_interval,
                        TInt end_loading_timestamp);
  TFlt get_whole_busroute_tt (TFlt start_time, MNM_Link_Factory *link_factory,
                              MNM_Busstop_Factory *busstop_factory,
                              TFlt unit_interval, TInt end_loading_timestamp);
  std::string busstop_vec_to_string ();
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Path Table
*******************************************************************************************************************
******************************************************************************************************************/
// <O, <D, <routeID, Path>>>
typedef std::unordered_map<
  TInt, std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath *> *> *>
  Bus_Path_Table;

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                PnR Path
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_PnR_Path : public MNM_Path
{
public:
  MNM_PnR_Path (TInt path_ID, TInt mid_parking_lot_ID, TInt mid_dest_node_ID,
                MNM_Path *driving_path, MNM_Path *transit_path);
  virtual ~MNM_PnR_Path () override;
  bool is_equal (MNM_Path *path);
  virtual bool is_link_in (TInt link_ID) override;

  TInt m_mid_parking_lot_ID;
  TInt m_mid_dest_node_ID;
  MNM_Path *m_driving_path;
  MNM_Path *m_transit_path;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                PnR Pathset
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_PnR_Pathset : public MNM_Pathset
{
public:
  MNM_PnR_Pathset ();
  virtual ~MNM_PnR_Pathset () override;
  virtual bool is_in (MNM_Path *path) override;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                PnR Path Table
*******************************************************************************************************************
******************************************************************************************************************/
// <O, <D, Pathset>>
typedef std::unordered_map<TInt, std::unordered_map<TInt, MNM_PnR_Pathset *> *>
  PnR_Path_Table;

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Routing
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          Bus Fixed routing
**************************************************************************/

class MNM_Routing_Bus : public MNM_Routing_Biclass_Fixed
{
public:
  MNM_Routing_Bus (macposts::Graph &driving_graph, MNM_OD_Factory *od_factory,
                   MNM_Node_Factory *node_factory,
                   MNM_Link_Factory *link_factory,
                   Bus_Path_Table *bus_path_table, TInt route_frq = TInt (-1),
                   TInt buffer_length = TInt (-1), TInt veh_class = TInt (1));
  virtual ~MNM_Routing_Bus () override;
  virtual int register_veh (MNM_Veh *veh, bool track = true) override;
  virtual int init_routing (Path_Table *driving_path_table = nullptr) override;
  virtual int update_routing (TInt timestamp) override;
  virtual int remove_finished (MNM_Veh *veh, bool del) override;

  Bus_Path_Table *m_bus_path_table;
};

/**************************************************************************
                          PnR Fixed Vehicle Routing
**************************************************************************/
class MNM_Routing_PnR_Fixed : public MNM_Routing_Biclass_Fixed
{
public:
  MNM_Routing_PnR_Fixed (
    macposts::Graph &driving_graph, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    PnR_Path_Table *pnr_path_table, TInt route_frq = TInt (-1),
    TInt buffer_length = TInt (-1), TInt veh_class = TInt (0));
  virtual ~MNM_Routing_PnR_Fixed () override;
  virtual int change_choice_portion (TInt routing_interval) override;
  virtual int register_veh (MNM_Veh *veh, bool track = true) override;
  virtual int init_routing (Path_Table *driving_path_table = nullptr) override;
  virtual int update_routing (TInt timestamp) override;
  virtual int remove_finished (MNM_Veh *veh, bool del = true) override;

  PnR_Path_Table *m_pnr_path_table;
};

/**************************************************************************
                  Passenger Bus Transit Routing
**************************************************************************/
class MNM_Routing_PassengerBusTransit
{
public:
  MNM_Routing_PassengerBusTransit (
    macposts::Graph &transit_graph, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
    MNM_Parking_Lot_Factory *parkinglot_factory,
    MNM_Transit_Link_Factory *transitlink_factory);
  virtual ~MNM_Routing_PassengerBusTransit ();
  virtual int init_routing (Path_Table *path_table = nullptr) { return 0; };
  virtual int update_routing (TInt timestamp) { return 0; };
  virtual int remove_finished (MNM_Passenger *passenger, bool del = true)
  {
    return 0;
  };

  macposts::Graph &m_graph;
  MNM_OD_Factory *m_od_factory;
  MNM_Node_Factory *m_node_factory;
  MNM_Busstop_Factory *m_busstop_factory;
  MNM_Parking_Lot_Factory *m_parkinglot_factory;
  MNM_Transit_Link_Factory *m_transitlink_factory;
};

/**************************************************************************
                  Passenger Bus Transit Fixed Routing
**************************************************************************/
class MNM_Routing_PassengerBusTransit_Fixed
    : public MNM_Routing_PassengerBusTransit
{
public:
  MNM_Routing_PassengerBusTransit_Fixed (
    macposts::Graph &transit_graph, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
    MNM_Parking_Lot_Factory *parkinglot_factory,
    MNM_Transit_Link_Factory *transitlink_factory,
    Path_Table *bustransit_path_table, TInt route_frq = TInt (-1),
    TInt buffer_length = TInt (-1));
  virtual ~MNM_Routing_PassengerBusTransit_Fixed () override;
  int register_passenger (MNM_Passenger *passenger, bool track = true);
  virtual int remove_finished (MNM_Passenger *passenger,
                               bool del = true) override;
  int add_passenger_path (MNM_Passenger *passenger, std::deque<TInt> *link_que);
  virtual int init_routing (Path_Table *path_table = nullptr) override;
  int update_routing_origin (TInt timestamp);
  int update_routing_parkinglot (TInt timestamp);
  int update_routing_one_link (TInt timestamp, MNM_Transit_Link *link);
  int update_routing_link (TInt timestamp);
  int update_routing_one_busstop (TInt timestamp, MNM_Busstop *busstop);
  int update_routing_busstop (TInt timestamp);
  virtual int update_routing (TInt timestamp) override;

  int add_bustransit_path (MNM_Passenger *passenger,
                           std::deque<TInt> *link_que);
  int change_choice_portion (TInt interval);

  Path_Table *m_bustransit_path_table;
  std::unordered_map<MNM_Passenger *, std::deque<TInt> *> m_tracker;
  bool m_buffer_as_p;
  TInt m_routing_freq;
  TInt m_buffer_length;
};

/**************************************************************************
                  Passenger and Vehicle Adaptive Routing
**************************************************************************/
class MNM_Routing_Multimodal_Adaptive
{
public:
  MNM_Routing_Multimodal_Adaptive (
    const std::string &file_folder, macposts::Graph &driving_graph,
    macposts::Graph &transit_graph, MNM_Statistics *statistics,
    MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
    MNM_Busstop_Factory *busstop_factory,
    MNM_Parking_Lot_Factory *parkinglot_factory, MNM_Link_Factory *link_factory,
    MNM_Transit_Link_Factory *transitlink_factory);
  ~MNM_Routing_Multimodal_Adaptive ();

  int init_routing ();
  int update_link_cost ();
  int update_routing (TInt timestamp);
  int update_routing_passenger_origin (TInt timestamp);
  int update_routing_passenger_parkinglot (TInt timestamp);
  int update_routing_passenger_one_link (TInt timestamp,
                                         MNM_Transit_Link *link);
  int update_routing_passenger_link (TInt timestamp);
  int update_routing_passenger_one_busstop (TInt timestamp,
                                            MNM_Busstop *busstop);
  int update_routing_passenger_busstop (TInt timestamp);
  int update_routing_passenger (TInt timestamp);
  int update_routing_vehicle (TInt timestamp);
  MNM_Destination *find_mid_destination_for_pnr (TInt timestamp,
                                                 TInt origin_node_ID,
                                                 TInt final_dest_node_ID);

  MNM_Statistics_Lrn_Multimodal *m_statistics;
  TInt m_routing_freq;
  TFlt m_vot;
  MNM_ConfReader *m_self_config;

  Routing_Table *m_driving_table;
  Routing_Table *m_transit_table;
  macposts::Graph &m_driving_graph;
  macposts::Graph &m_transit_graph;
  MNM_OD_Factory *m_od_factory;
  MNM_Node_Factory *m_node_factory;
  MNM_Busstop_Factory *m_busstop_factory;
  MNM_Parking_Lot_Factory *m_parkinglot_factory;
  MNM_Link_Factory *m_link_factory;
  MNM_Transit_Link_Factory *m_transitlink_factory;

  std::unordered_map<TInt, TFlt> m_driving_link_cost;
  std::unordered_map<TInt, TFlt> m_bustransit_link_cost;

  bool m_working;
};

/**************************************************************************
                          Multimodal_Hybrid Routing
**************************************************************************/
class MNM_Routing_Multimodal_Hybrid : public MNM_Routing_Biclass_Hybrid
{
public:
  MNM_Routing_Multimodal_Hybrid (
    const std::string &file_folder, macposts::Graph &driving_graph,
    macposts::Graph &transit_graph, MNM_Statistics *statistics,
    MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
    MNM_Link_Factory *link_factory, MNM_Busstop_Factory *busstop_factory,
    MNM_Parking_Lot_Factory *parkinglot_factory,
    MNM_Transit_Link_Factory *transitlink_factory,
    Bus_Path_Table *bus_path_table, PnR_Path_Table *pnr_path_table,
    Path_Table *bustransit_path_table, TInt route_frq_fixed = TInt (-1),
    TInt buffer_length = TInt (-1));
  virtual ~MNM_Routing_Multimodal_Hybrid () override;
  virtual int init_routing (Path_Table *driving_path_table = nullptr) override;
  virtual int update_routing (TInt timestamp) override;
  virtual int remove_finished (MNM_Veh *veh, bool del = true) override;
  virtual int remove_finished_passenger (MNM_Passenger *passenger,
                                         bool del = true);

  MNM_Routing_Bus *m_routing_bus_fixed;
  MNM_Routing_PnR_Fixed *m_routing_car_pnr_fixed;
  MNM_Routing_PassengerBusTransit_Fixed *m_routing_passenger_fixed;
  MNM_Routing_Multimodal_Adaptive *m_routing_multimodal_adaptive;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal IO Functions
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_IO_Multimodal : public MNM_IO_Multiclass
{
public:
  static int build_node_factory_multimodal (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_Node_Factory *node_factory,
                                            const std::string &file_name
                                            = "driving_node");
  static int build_link_factory_multimodal (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_Link_Factory *link_factory,
                                            const std::string &file_name
                                            = "driving_link");
  static int build_od_factory_multimodal (std::string file_folder,
                                          MNM_ConfReader *conf_reader,
                                          MNM_OD_Factory *od_factory,
                                          MNM_Node_Factory *node_factory,
                                          const std::string &file_name = "od");
  static int build_busstop_factory (const std::string &file_folder,
                                    MNM_ConfReader *conf_reader,
                                    MNM_Busstop_Factory *busstop_factory,
                                    MNM_Link_Factory *link_factory,
                                    const std::string &file_name = "bus_stop");
  static int build_parkinglot_factory (
    const std::string &file_folder, MNM_ConfReader *conf_reader,
    MNM_Parking_Lot_Factory *parkinglot_factory, MNM_Node_Factory *node_factory,
    MNM_Link_Factory *link_factory, MNM_Passenger_Factory *passenger_factory,
    MNM_Veh_Factory *veh_factory, const std::string &file_name = "parking_lot");
  static int build_walkinglink_factory (
    const std::string &file_folder, MNM_ConfReader *conf_reader,
    MNM_Transit_Link_Factory *transit_link_factory,
    MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
    MNM_Parking_Lot_Factory *parkinglot_factory,
    const std::string &file_name = "walking_link");
  static int build_buslink_factory (
    const std::string &file_folder, MNM_ConfReader *conf_reader,
    MNM_Transit_Link_Factory *transit_link_factory,
    MNM_Busstop_Factory *busstop_factory, MNM_Link_Factory *link_factory,
    const std::string &file_name = "bus_link");
  static PNEGraph
  build_bus_transit_graph (MNM_ConfReader *conf_reader,
                           MNM_Transit_Link_Factory *transit_link_factory);
  static macposts::Graph
  build_bus_transit_graph (MNM_ConfReader *conf_reader,
                           MNM_Transit_Link_Factory *transit_link_factory,
                           int _phantom);
  static int build_passenger_demand (
    const std::string &file_folder, MNM_ConfReader *conf_reader,
    MNM_OD_Factory *od_factory,
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>>
      &passenger_demand,
    const std::string &file_name = "passenger_demand");
  static int build_vehicle_demand_multimodal (
    const std::string &file_folder, MNM_ConfReader *conf_reader,
    MNM_OD_Factory *od_factory,
    const std::string &file_name_driving = "driving_demand",
    const std::string &file_name_bus = "bus_demand",
    const bool use_explicit_bus = true);
  static int build_pnr_demand (const std::string &file_folder,
                               MNM_ConfReader *conf_reader,
                               MNM_OD_Factory *od_factory,
                               const std::string &file_name = "pnr_demand");
  static int build_bustransit_demand (const std::string &file_folder,
                                      MNM_ConfReader *conf_reader,
                                      MNM_OD_Factory *od_factory,
                                      const std::string &file_name
                                      = "bustransit_demand");

  static Bus_Path_Table *
  load_bus_path_table (const macposts::Graph &graph, TInt num_path,
                       MNM_Node_Factory *node_factory = nullptr,
                       MNM_Busstop_Factory *busstop_factory = nullptr,
                       TInt num_release_interval = -1,
                       const std::string &file_name = "bus_path_table",
                       const std::string &route_file_name = "bus_route");
  static PnR_Path_Table *
  load_pnr_path_table (const macposts::Graph &driving_graph,
                       const macposts::Graph &transit_graph, TInt num_path,
                       const std::string &file_name = "pnr_path_table",
                       bool w_buffer = true, bool w_ID = false);
  static Path_Table *load_bustransit_path_table (
    const macposts::Graph &transit_graph, TInt num_path,
    const std::string &file_name = "bustransit_path_table",
    bool w_buffer = true, bool w_ID = false);
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal DTA
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Dta_Multimodal : public MNM_Dta_Multiclass
{
public:
  explicit MNM_Dta_Multimodal (const std::string &file_folder);
  virtual ~MNM_Dta_Multimodal () override;
  virtual int initialize () override;
  virtual int set_statistics () override;
  virtual int set_routing () override;
  virtual int build_from_files () override;
  int find_connected_pnr_parkinglot_for_destination ();
  bool check_bus_transit_connectivity ();
  bool check_bus_path_table ();
  virtual bool is_ok () override;
  virtual int pre_loading () override;
  virtual int load_once (bool verbose, TInt load_int, TInt assign_int) override;
  virtual int loading (bool verbose) override;
  virtual bool finished_loading (int cur_int) override;
  int record_queue_passengers ();
  int record_enroute_passengers ();

  //    MNM_Veh_Factory_Multimodal *m_veh_factory;
  //    MNM_Node_Factory_Multimodal *m_node_factory;
  //    MNM_Link_Factory_Multimodal *m_link_factory;
  // MNM_OD_Factory_Multimodal *m_od_factory;
  MNM_Passenger_Factory *m_passenger_factory;
  MNM_Busstop_Factory *m_busstop_factory;
  MNM_Parking_Lot_Factory *m_parkinglot_factory;
  MNM_Transit_Link_Factory *m_transitlink_factory;

  macposts::Graph m_bus_transit_graph;

  std::deque<TInt> m_enroute_passenger_num;
  std::deque<TInt> m_queue_passenger_num;
  std::unordered_map<TInt, std::deque<TInt> *> m_queue_passenger_map;

  bool m_explicit_bus;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Multiclass DTA Gradient Utils
*******************************************************************************************************************
******************************************************************************************************************/
namespace MNM_DTA_GRADIENT
{
TFlt get_bus_waiting_time (MNM_Busstop_Virtual *busstop, TFlt start_time,
                           TFlt unit_interval, TInt end_loading_timestamp,
                           bool return_inf = false);

TFlt get_travel_time_walking (MNM_Walking_Link *link, TFlt start_time,
                              TFlt unit_interval, TInt end_loading_timestamp);

TFlt get_travel_time_walking_robust (MNM_Walking_Link *link, TFlt start_time,
                                     TFlt end_time, TFlt unit_interval,
                                     TInt end_loading_timestamp,
                                     TInt num_trials = TInt (10));

TFlt get_travel_time_bus (MNM_Bus_Link *link, TFlt start_time,
                          TFlt unit_interval, TInt end_loading_timestamp,
                          bool explicit_bus = true, bool return_inf = false,
                          bool return_bus_time = false);

TFlt get_travel_time_bus_robust (MNM_Bus_Link *link, TFlt start_time,
                                 TFlt end_time, TFlt unit_interval,
                                 TInt end_loading_timestamp,
                                 TInt num_trials = TInt (10),
                                 bool explicit_bus = true,
                                 bool return_inf = false,
                                 bool return_bus_time = false);

TFlt get_link_inflow_bus (MNM_Bus_Link *link, TFlt start_time, TFlt end_time);

TFlt get_busstop_inflow_bus (MNM_Busstop_Virtual *busstop, TFlt start_time,
                             TFlt end_time);

TFlt get_link_inflow_passenger (MNM_Transit_Link *link, TFlt start_time,
                                TFlt end_time);

int add_dar_records_bus (std::vector<dar_record *> &record, MNM_Bus_Link *link,
                         std::set<MNM_Path *> pathset, TFlt start_time,
                         TFlt end_time);

int add_dar_records_passenger (std::vector<dar_record *> &record,
                               MNM_Transit_Link *link,
                               std::set<MNM_Path *> pathset, TFlt start_time,
                               TFlt end_time);

int add_dar_records_bus (std::vector<dar_record *> &record, MNM_Bus_Link *link,
                         std::set<TInt> pathID_set, TFlt start_time,
                         TFlt end_time);

int add_dar_records_passenger (std::vector<dar_record *> &record,
                               MNM_Transit_Link *link,
                               std::set<TInt> pathID_set, TFlt start_time,
                               TFlt end_time);

TFlt get_departure_cc_slope_walking_passenger (MNM_Walking_Link *link,
                                               TFlt start_time, TFlt end_time);
TFlt get_departure_cc_slope_bus_passenger (MNM_Bus_Link *link, TFlt start_time,
                                           TFlt end_time);

int add_ltg_records_passenger (std::vector<ltg_record *> &record,
                               MNM_Transit_Link *link, MNM_Path *path,
                               int depart_time, int start_time, TFlt gradient);
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Passenger path
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                   Base
**************************************************************************/
class MNM_Passenger_Path_Base : public MNM_Path
{
public:
  MNM_Passenger_Path_Base (int mode, TFlt vot, TFlt early_penalty,
                           TFlt late_penalty, TFlt target_time);
  virtual ~MNM_Passenger_Path_Base ();

  int m_mode;
  MNM_Path *m_path;
  TFlt m_vot;           // money / interval
  TFlt m_early_penalty; // money / interval
  TFlt m_late_penalty;  // money / interval
  TFlt m_target_time;   // intervals

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta)
  {
    return TFlt (-1);
  }; // meter
  virtual TFlt get_travel_time (TFlt start_time, MNM_Dta_Multimodal *mmdta)
  {
    return TFlt (-1.0);
  }; // interval
  virtual TFlt get_travel_cost (TFlt start_time, MNM_Dta_Multimodal *mmdta)
  {
    return TFlt (-1.0);
  };
  virtual TFlt
  get_travel_time (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                   std::unordered_map<TInt, TFlt *> driving_link_tt_map,
                   std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
  {
    return TFlt (-1.0);
  }; // interval
  virtual TFlt
  get_travel_cost (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                   std::unordered_map<TInt, TFlt *> driving_link_tt_map,
                   std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
  {
    return TFlt (-1.0);
  };
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta)
  {
    return TFlt (-1.0);
  };
  TFlt get_wrongtime_penalty (TFlt arrival_time);

  virtual bool is_equal (MNM_Passenger_Path_Base *path) { return false; };
  virtual std::string info2str ()
  {
    return "Base method should not be called\n#origin_node_ID dest_node_ID "
           "mode mid_dest_node_ID parkinglot_ID <driving_node_sequence> "
           "<bus_transit_link_sequence>\n";
  };
};

/**************************************************************************
                                                   Driving
**************************************************************************/
class MNM_Passenger_Path_Driving : public MNM_Passenger_Path_Base
{
public:
  MNM_Passenger_Path_Driving (int mode, MNM_Path *path, TFlt vot,
                              TFlt early_penalty, TFlt late_penalty,
                              TFlt target_time, TInt num_people,
                              TFlt carpool_cost_multiplier,
                              TFlt walking_time_before_driving,
                              MNM_Parking_Lot *parking_lot,
                              TFlt walking_time_after_driving);
  virtual ~MNM_Passenger_Path_Driving () override;

  TInt m_num_people;
  TFlt m_carpool_cost_multiplier;
  TFlt m_walking_time_before_driving; // seconds
  TFlt m_walking_time_after_driving;  // seconds

  MNM_Parking_Lot *m_parking_lot;

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta) override; // meter
  TFlt get_travel_time_truck (TFlt start_time, MNM_Dta_Multimodal *mmdta);
  TFlt
  get_travel_time_truck (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                         std::unordered_map<TInt, TFlt *> link_tt_map_truck);
  virtual TFlt get_travel_time (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override; // interval
  virtual TFlt get_travel_cost (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  virtual TFlt
  get_travel_time (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                   std::unordered_map<TInt, TFlt *> driving_link_tt_map,
                   std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
    override; // interval
  virtual TFlt get_travel_cost (
    TFlt start_time, MNM_Dta_Multimodal *mmdta,
    std::unordered_map<TInt, TFlt *> driving_link_tt_map,
    std::unordered_map<TInt, TFlt *> bustransit_link_tt_map) override;
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta) override;
  TFlt get_carpool_cost ();
  TFlt get_amortized_parkingfee ();
  TFlt get_toll (MNM_Dta_Multimodal *mmdta);

  virtual bool is_equal (MNM_Passenger_Path_Base *path) override;
  virtual std::string info2str () override;
};

/**************************************************************************
                                                   Bus Transit
**************************************************************************/
class MNM_Passenger_Path_Bus : public MNM_Passenger_Path_Base
{
public:
  MNM_Passenger_Path_Bus (int mode, MNM_Path *path, TFlt vot,
                          TFlt early_penalty, TFlt late_penalty,
                          TFlt target_time, TFlt bus_fare,
                          TFlt bus_inconvenience);
  virtual ~MNM_Passenger_Path_Bus () override;

  TFlt m_bus_fare;
  TFlt m_bus_inconvenience;
  // TFlt m_waiting_time;

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta) override; // meter
  virtual TFlt
  get_travel_time (TFlt start_time,
                   MNM_Dta_Multimodal *mmdta) override; // intervals
  virtual TFlt get_travel_cost (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  virtual TFlt
  get_travel_time (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                   std::unordered_map<TInt, TFlt *> driving_link_tt_map,
                   std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
    override; // interval
  virtual TFlt get_travel_cost (
    TFlt start_time, MNM_Dta_Multimodal *mmdta,
    std::unordered_map<TInt, TFlt *> driving_link_tt_map,
    std::unordered_map<TInt, TFlt *> bustransit_link_tt_map) override;
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta) override;
  TFlt get_total_bus_fare (MNM_Dta_Multimodal *mmdta);

  virtual bool is_equal (MNM_Passenger_Path_Base *path) override;
  virtual std::string info2str () override;
};

/**************************************************************************
                                                   Metro Transit
**************************************************************************/
class MNM_Passenger_Path_Metro : public MNM_Passenger_Path_Base
{
public:
  MNM_Passenger_Path_Metro (int mode, MNM_Path *path, TFlt vot,
                            TFlt early_penalty, TFlt late_penalty,
                            TFlt target_time, TFlt metro_fare,
                            TFlt metro_inconvenience, TFlt metro_time,
                            TFlt waiting_time);
  virtual ~MNM_Passenger_Path_Metro () override;

  TFlt m_metro_fare;
  TFlt m_metro_inconvenience;
  TFlt m_metro_time; // intervals, from metro network (TODO)

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta) override; // meter
  virtual TFlt get_travel_time (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override; // interval
  virtual TFlt get_travel_cost (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  // virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta,
  //                              std::unordered_map<TInt, TFlt *>
  //                              driving_link_tt_map, std::unordered_map<TInt,
  //                              TFlt *> bustransit_link_tt_map) override;  //
  //                              interval
  // virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta,
  //                              std::unordered_map<TInt, TFlt *>
  //                              driving_link_tt_map, std::unordered_map<TInt,
  //                              TFlt *> bustransit_link_tt_map) override;
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta) override;
  TFlt get_total_metro_fare (MNM_Dta_Multimodal *mmdta);

  virtual bool is_equal (MNM_Passenger_Path_Base *path) override;
  virtual std::string info2str () override;
};

/**************************************************************************
                                                   Park & Ride
**************************************************************************/
class MNM_Passenger_Path_PnR : public MNM_Passenger_Path_Base
{
public:
  MNM_Passenger_Path_PnR (int mode, MNM_PnR_Path *path, TFlt vot,
                          TFlt early_penalty, TFlt late_penalty,
                          TFlt target_time, TFlt walking_time_before_driving,
                          MNM_Parking_Lot *parking_lot, TFlt bus_fare,
                          TFlt pnr_inconvenience);
  virtual ~MNM_Passenger_Path_PnR () override;

  MNM_PnR_Path *m_path;
  MNM_Passenger_Path_Driving *m_driving_part;
  MNM_Passenger_Path_Bus *m_bus_part;
  MNM_Parking_Lot *m_mid_parking_lot;
  TFlt m_pnr_inconvenience;

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta) override; // meter
  virtual TFlt get_travel_time (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  virtual TFlt get_travel_cost (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  virtual TFlt
  get_travel_time (TFlt start_time, MNM_Dta_Multimodal *mmdta,
                   std::unordered_map<TInt, TFlt *> driving_link_tt_map,
                   std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
    override; // interval
  virtual TFlt get_travel_cost (
    TFlt start_time, MNM_Dta_Multimodal *mmdta,
    std::unordered_map<TInt, TFlt *> driving_link_tt_map,
    std::unordered_map<TInt, TFlt *> bustransit_link_tt_map) override;
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta) override;
  virtual bool is_equal (MNM_Passenger_Path_Base *path) override;
  virtual std::string info2str () override;
};

/**************************************************************************
                                                   Ride & Drive
**************************************************************************/
class MNM_Passenger_Path_RnD : public MNM_Passenger_Path_Base
{
public:
  MNM_Passenger_Path_RnD (int mode, MNM_PnR_Path *path, TFlt vot,
                          TFlt early_penalty, TFlt late_penalty,
                          TFlt target_time, TFlt walking_time_before_driving,
                          MNM_Parking_Lot *parking_lot, TFlt bus_fare,
                          TFlt rnd_inconvenience);
  virtual ~MNM_Passenger_Path_RnD () override;

  MNM_PnR_Path *m_path;
  MNM_Passenger_Path_Bus *m_bus_part;
  MNM_Passenger_Path_Driving *m_driving_part;
  MNM_Parking_Lot *m_mid_parking_lot;
  TFlt m_rnd_inconvenience;

  virtual TFlt get_length (MNM_Dta_Multimodal *mmdta) override; // meter
  virtual TFlt get_travel_time (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  virtual TFlt get_travel_cost (TFlt start_time,
                                MNM_Dta_Multimodal *mmdta) override;
  // virtual TFlt get_travel_time(TFlt start_time, MNM_Dta_Multimodal *mmdta,
  //                              std::unordered_map<TInt, TFlt *>
  //                              driving_link_tt_map, std::unordered_map<TInt,
  //                              TFlt *> bustransit_link_tt_map) override;  //
  //                              interval
  // virtual TFlt get_travel_cost(TFlt start_time, MNM_Dta_Multimodal *mmdta,
  //                              std::unordered_map<TInt, TFlt *>
  //                              driving_link_tt_map, std::unordered_map<TInt,
  //                              TFlt *> bustransit_link_tt_map) override;
  virtual TFlt get_travel_cost_with_tt (TFlt start_time, TFlt travel_time,
                                        MNM_Dta_Multimodal *mmdta) override;
  virtual bool is_equal (MNM_Passenger_Path_Base *path) override;
  virtual std::string info2str () override;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Passenger Path Set
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_Passenger_Pathset
{
public:
  MNM_Passenger_Pathset (MMDue_mode mode);
  ~MNM_Passenger_Pathset ();
  MMDue_mode m_mode; // driving, transit, pnr, rh
  std::vector<MNM_Passenger_Path_Base *> m_path_vec;
  bool is_in (MNM_Passenger_Path_Base *path);
};

// <O, <D, <mode, PassengerPathset>>>
typedef std::unordered_map<
  TInt, std::unordered_map<
          TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *> *> *>
  Passenger_Path_Table;

namespace MNM
{
std::unordered_map<int, TFlt>
logit_fn (std::unordered_map<int, TFlt> &cost,
          std::unordered_map<int, TFlt> &alpha_map, TFlt beta);

int normalize_path_table_p (PnR_Path_Table *pnr_path_table);

int copy_buffer_to_p (PnR_Path_Table *pnr_path_table, TInt col);

bool has_running_passenger (MNM_Passenger_Factory *passenger_factory);

int print_passenger_statistics (MNM_Passenger_Factory *passenger_factory);

int print_vehicle_statistics (MNM_Veh_Factory_Multimodal *veh_factory);

Passenger_Path_Table *build_shortest_passenger_pathset (
  std::vector<MMDue_mode> &mode_vec,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &passenger_demand,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_MM_Due *mmdue, macposts::Graph &driving_graph,
  macposts::Graph &bustransit_graph, MNM_OD_Factory *od_factory,
  MNM_Link_Factory *link_factory, MNM_Transit_Link_Factory *transitlink_factory,
  MNM_Busstop_Factory *busstop_factory);

Passenger_Path_Table *build_existing_passenger_pathset (
  std::vector<MMDue_mode> &mode_vec,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &passenger_demand,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_MM_Due *mmdue);

Path_Table *build_shortest_driving_pathset (
  macposts::Graph &driving_graph, MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Link_Factory *link_factory, TFlt min_path_length = 0.0,
  size_t MaxIter = 10, TFlt Mid_Scale = 3, TFlt Heavy_Scale = 6,
  TInt buffer_length = -1);

Path_Table *build_shortest_bustransit_pathset (
  macposts::Graph &bustransit_graph, MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Transit_Link_Factory *bus_transitlink_factory, TFlt min_path_length = 0.0,
  size_t MaxIter = 10, TFlt Mid_Scale = 3, TFlt Heavy_Scale = 6,
  TInt buffer_length = -1);

PnR_Path_Table *build_shortest_pnr_pathset (
  macposts::Graph &driving_graph, macposts::Graph &bustransit_graph,
  MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Link_Factory *link_factory,
  MNM_Transit_Link_Factory *bus_transitlink_factory, TFlt min_path_length = 0.0,
  size_t MaxIter = 10, TFlt Mid_Scale = 3, TFlt Heavy_Scale = 6,
  TInt buffer_length = -1);

int allocate_passenger_path_table_buffer (Passenger_Path_Table *path_table,
                                          TInt num);

int generate_init_mode_demand_file (
  MNM_MM_Due *mmdue, const std::string &file_folder,
  const std::string &driving_demand_file_name = "driving_demand",
  const std::string &bustransit_demand_file_name = "bustransit_demand",
  const std::string &pnr_demand_file_name = "pnr_demand");

int save_passenger_path_table (Passenger_Path_Table *passenger_path_table,
                               const std::string &file_folder,
                               const std::string &path_file_name
                               = "passenger_path_table",
                               const std::string &buffer_file_name
                               = "passenger_path_table_buffer",
                               bool w_buffer = true, bool w_cost = false);

int save_driving_path_table (const std::string &file_folder,
                             Path_Table *path_table,
                             const std::string &path_file_name
                             = "driving_path_table",
                             const std::string &buffer_file_name
                             = "driving_path_table_buffer",
                             bool w_buffer = true);

int save_bustransit_path_table (const std::string &file_folder,
                                Path_Table *path_table,
                                const std::string &path_file_name
                                = "bustransit_path_table",
                                const std::string &buffer_file_name
                                = "bustransit_path_table_buffer",
                                bool w_buffer = true);

int save_pnr_path_table (const std::string &file_folder,
                         PnR_Path_Table *path_table,
                         const std::string &path_file_name = "pnr_path_table",
                         const std::string &buffer_file_name
                         = "pnr_path_table_buffer",
                         bool w_buffer = true);

int get_ID_path_mapping_all_mode (
  std::unordered_map<TInt, std::pair<MNM_Path *, MNM_Passenger_Path_Base *>>
    &dict,
  Path_Table *driving_path_table, Bus_Path_Table *bus_path_table,
  PnR_Path_Table *pnr_path_table, Path_Table *bustransit_path_table,
  Passenger_Path_Table *passegner_path_table, TInt num_path_driving,
  TInt num_path_bustransit, TInt num_path_pnr, TInt num_path_bus);

}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Multimodal DUE
*******************************************************************************************************************
******************************************************************************************************************/
class MNM_MM_Due
{
public:
  MNM_MM_Due (const std::string &file_folder);

  virtual ~MNM_MM_Due ();

  virtual int initialize ();

  MNM_Dta_Multimodal *run_mmdta (bool verbose);

  MNM_Dta_Multimodal *run_mmdta_adaptive (bool verbose);

  int check_od_mode_connectivity ();

  int save_od_mode_connectivity (const std::string &connectivity_file_name
                                 = "od_mode_connectivity");

  int load_od_mode_connectivity (const std::string &connectivity_file_name
                                 = "od_mode_connectivity");

  virtual int init_passenger_path_table ();

  virtual int init_passenger_path_flow ();

  virtual int
  update_origin_demand_from_passenger_path_table (MNM_Dta_Multimodal *mmdta);

  virtual int update_origin_demand_logit_model (MNM_Dta_Multimodal *mmdta,
                                                int assign_inter);

  virtual int save_od_demand_split (MNM_Dta_Multimodal *mmdta,
                                    const std::string &file_folder,
                                    const std::string &file_name
                                    = "od_demand_split.txt");

  virtual int build_link_cost_map_snapshot (MNM_Dta_Multimodal *mmdta,
                                            int start_interval,
                                            bool in_simulation = false);

  virtual int update_snapshot_route_table (MNM_Dta_Multimodal *mmdta,
                                           int start_interval);

  virtual std::unordered_map<int, TFlt>
  get_mode_split_snapshot (MNM_Dta_Multimodal *mmdta, int start_interval,
                           int o_node_ID, int d_node_ID);

  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt, bool>, int>
  get_lowest_cost_path_snapshot (int start_interval, int o_node_ID,
                                 int d_node_ID, MNM_Dta_Multimodal *mmdta);

  virtual int clear_multimodal_path_table_buffer ();

  virtual int
  passenger_path_table_to_multimodal_path_table (MNM_Dta_Multimodal *mmdta);

  int update_path_table_cost (MNM_Dta_Multimodal *mmdta);

  int update_one_path_cost (MNM_Passenger_Path_Base *p_path, TInt o_node_ID,
                            TInt d_node_ID, MNM_Dta_Multimodal *mmdta);

  virtual int update_path_table (MNM_Dta_Multimodal *mmdta, int iter);

  virtual int
  update_path_table_fixed_departure_time_choice (MNM_Dta_Multimodal *mmdta,
                                                 int iter);

  virtual int
  update_path_table_gp_fixed_departure_time_choice (MNM_Dta_Multimodal *mmdta,
                                                    int iter);

  int build_link_cost_map (MNM_Dta_Multimodal *mmdta,
                           bool with_congestion_indicator = false);

  int get_link_queue_dissipated_time (MNM_Dta_Multimodal *mmdta);

  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
  get_best_path (
    TInt o_node_ID, TInt d_node_ID,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_bus,
    MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
  get_best_driving_path (TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_driving,
                         MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
  get_best_bus_path (TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_bus,
                     MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt> get_best_pnr_path (
    TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_bus,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
    MNM_Dta_Multimodal *mmdta);

  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
  get_best_existing_path (TInt o_node_ID, TInt d_node_ID,
                          MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
  get_best_existing_driving_path (TInt o_node_ID, TInt d_node_ID,
                                  MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
  get_best_existing_bus_path (TInt o_node_ID, TInt d_node_ID,
                              MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
  get_best_existing_pnr_path (TInt o_node_ID, TInt d_node_ID,
                              MNM_Dta_Multimodal *mmdta);

  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
  get_best_path_for_single_interval (
    TInt interval, TInt o_node_ID, TInt d_node_ID,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_bus,
    MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
  get_best_driving_path_for_single_interval (TInt interval, TInt o_node_ID,
                                             MNM_TDSP_Tree *tdsp_tree_driving,
                                             MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
  get_best_bus_path_for_single_interval (TInt interval, TInt o_node_ID,
                                         MNM_TDSP_Tree *tdsp_tree_bus,
                                         MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
  get_best_pnr_path_for_single_interval (
    TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_bus,
    std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
    MNM_Dta_Multimodal *mmdta);

  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
  get_best_existing_path_for_single_interval (TInt interval, TInt o_node_ID,
                                              TInt d_node_ID,
                                              MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
  get_best_existing_driving_path_for_single_interval (
    TInt interval, TInt o_node_ID, TInt d_node_ID, MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
  get_best_existing_bus_path_for_single_interval (TInt interval, TInt o_node_ID,
                                                  TInt d_node_ID,
                                                  MNM_Dta_Multimodal *mmdta);

  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
  get_best_existing_pnr_path_for_single_interval (TInt interval, TInt o_node_ID,
                                                  TInt d_node_ID,
                                                  MNM_Dta_Multimodal *mmdta);

  TFlt compute_total_passenger_demand_for_one_mode (int mode,
                                                    TInt origin_node_ID,
                                                    TInt dest_node_ID,
                                                    TInt assign_inter);

  TFlt get_disutility (int mode, TFlt travel_cost, TFlt total_demand_one_mode);

  TFlt compute_merit_function (MNM_Dta_Multimodal *mmdta);

  TFlt compute_merit_function_fixed_departure_time_choice (
    MNM_Dta_Multimodal *mmdta);

  TFlt compute_total_passenger_demand (MNM_Origin *orig, MNM_Destination *dest,
                                       TInt total_assign_inter);

  std::string m_file_folder;
  MNM_ConfReader *m_mmdta_config;
  MNM_ConfReader *m_mmdue_config;
  // <origin_node_ID, <dest_node_ID, <mode, true or false>>>
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    m_od_mode_connectivity;
  TInt m_num_modes;
  TFlt m_unit_time;
  TInt m_total_assign_inter; // different from m_mmdta -> m_total_assign_inter,
                             // which is actually a releasing interval
  TInt m_total_loading_inter;

  // <O_node_ID, <D_node_ID, time-varying demand with length of
  // m_total_assign_inter>>
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> m_passenger_demand;
  // <O_node_ID, <D_node_ID, <mode_ID, passenger_pathset>>>
  Passenger_Path_Table *m_passenger_path_table;

  Path_Table *m_driving_path_table;
  Path_Table *m_truck_path_table;
  PnR_Path_Table *m_pnr_path_table;
  Path_Table *m_bustransit_path_table;
  Bus_Path_Table *m_bus_path_table;

  std::vector<MMDue_mode> m_mode_vec;

  TFlt m_alpha1_driving;
  TFlt m_alpha1_transit;
  TFlt m_alpha1_pnr;
  TFlt m_beta1;

  TFlt m_vot;
  TFlt m_early_penalty;
  TFlt m_late_penalty;
  TFlt m_target_time;

  TInt m_max_iter;
  TFlt m_step_size;

  TFlt m_parking_lot_to_destination_walking_time;
  TFlt m_carpool_cost_multiplier;
  TFlt m_bus_fare;
  TFlt m_metro_fare;
  TFlt m_pnr_inconvenience;
  TFlt m_bus_inconvenience;

  MNM_Dta_Multimodal *m_mmdta;

  // single_level <mode, <passenger path ID, cost>>

  // time-varying link tt
  std::unordered_map<TInt, TFlt *> m_link_tt_map;
  std::unordered_map<TInt, TFlt *> m_link_tt_map_truck;
  std::unordered_map<TInt, TFlt *> m_transitlink_tt_map;

  // time-varying link cost
  std::unordered_map<TInt, TFlt *> m_link_cost_map;
  std::unordered_map<TInt, TFlt *> m_link_cost_map_truck;
  std::unordered_map<TInt, TFlt *> m_transitlink_cost_map;

  // time-varying indicator
  std::unordered_map<TInt, bool *> m_link_congested_car;
  std::unordered_map<TInt, bool *> m_link_congested_truck;
  std::unordered_map<TInt, bool *> m_transitlink_congested_passenger;

  // time-varying queue dissipated time
  std::unordered_map<TInt, int *> m_queue_dissipated_time_car;
  std::unordered_map<TInt, int *> m_queue_dissipated_time_truck;
  std::unordered_map<TInt, int *> m_queue_dissipated_time_passenger;

  std::unordered_map<TInt, TFlt> m_driving_link_tt_map_snapshot;
  std::unordered_map<TInt, TFlt> m_bustransit_link_tt_map_snapshot;

  std::unordered_map<TInt, TFlt> m_driving_link_cost_map_snapshot;
  std::unordered_map<TInt, TFlt> m_bustransit_link_cost_map_snapshot;

  std::unordered_map<TInt, std::unordered_map<TInt, TInt>>
    m_driving_table_snapshot;
  std::unordered_map<TInt, std::unordered_map<TInt, TInt>>
    m_bustransit_table_snapshot;

  std::unordered_map<int, TFlt> m_mode_share;

  std::unordered_map<
    TInt, std::unordered_map<TInt, std::unordered_map<int, std::vector<TFlt>>>>
    m_od_demand_by_mode;
};
