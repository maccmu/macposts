#pragma once

#include "dta.h"

class MNM_Veh_Delivery : virtual public MNM_Veh
{
public:
  MNM_Veh_Delivery (TInt ID, TInt start_time);
  virtual ~MNM_Veh_Delivery ();

  int set_multi_od_seq (
    std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *multi_od_seq);
  std::pair<MNM_Origin *, MNM_Destination *> get_current_od ();
  int move_to_next_od ();

  std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *m_multi_od_seq;
  int m_current_OD_index;

  // intervals, used to consider some delays, e.g., delivery time or pickup
  // waiting time at origins for mobility service vehicles in multimodal
  TInt m_waiting_time = 0;
};

class MNM_Veh_Factory_Delivery : public MNM_Veh_Factory
{
public:
  MNM_Veh_Factory_Delivery ();
  virtual ~MNM_Veh_Factory_Delivery ();
  MNM_Veh_Delivery *make_veh_delivery (TInt timestamp, Vehicle_type veh_type);
  TInt m_veh_delivery;
};

class MNM_Dlink_Pq_Delay : public MNM_Dlink_Pq
{
public:
  MNM_Dlink_Pq_Delay (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                      TInt number_of_lane, TFlt length, TFlt ffs,
                      TFlt unit_time, TFlt flow_scalar);
  virtual ~MNM_Dlink_Pq_Delay () override;

  virtual int clear_incoming_array (TInt timestamp) override;
};

class MNM_Link_Factory_Delivery : public MNM_Link_Factory
{
public:
  MNM_Link_Factory_Delivery ();
  virtual ~MNM_Link_Factory_Delivery ();

  virtual MNM_Dlink *make_link (TInt ID, DLink_type link_type,
                                TFlt lane_hold_cap, TFlt lane_flow_cap,
                                TInt number_of_lane, TFlt length, TFlt ffs,
                                TFlt unit_time, TFlt flow_scalar) override;
};

class MNM_Origin_Delivery : public MNM_Origin
{
public:
  MNM_Origin_Delivery (TInt ID, TInt max_interval, TFlt flow_scalar,
                       TInt frequency);
  virtual ~MNM_Origin_Delivery ();

  int add_multi_OD_seq_demand (
    std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *multi_od_seq,
    TFlt *demand);
  virtual int release_one_interval (TInt current_interval,
                                    MNM_Veh_Factory *veh_factory,
                                    TInt assign_interval,
                                    TFlt adaptive_ratio) override;

  std::unordered_map<std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *,
                     TFlt *>
    m_demand_multi_OD_seq;
  // for delivery vehicles
  TInt m_pickup_waiting_time = 0;
};

class MNM_Destination_Delivery : public MNM_Destination
{
public:
  explicit MNM_Destination_Delivery (TInt ID);
  virtual ~MNM_Destination_Delivery ();

  virtual int receive (TInt current_interval) override;
  virtual int receive (TInt current_interval, MNM_Routing *routing,
                       MNM_Veh_Factory *veh_factory, bool del = true) override;
};

class MNM_OD_Factory_Delivery : public MNM_OD_Factory
{
public:
  MNM_OD_Factory_Delivery ();
  virtual ~MNM_OD_Factory_Delivery ();

  virtual MNM_Destination *make_destination (TInt ID) override;
  virtual MNM_Origin *make_origin (TInt ID, TInt max_interval, TFlt flow_scalar,
                                   TInt frequency) override;
};

class MNM_Routing_Delivery_Fixed : public MNM_Routing_Fixed
{
public:
  MNM_Routing_Delivery_Fixed (macposts::Graph &graph,
                              MNM_OD_Factory *od_factory,
                              MNM_Node_Factory *node_factory,
                              MNM_Link_Factory *link_factory,
                              TInt route_frq = TInt (-1),
                              TInt buffer_len = TInt (-1));
  virtual ~MNM_Routing_Delivery_Fixed () override;

  virtual int remove_finished (MNM_Veh *veh, bool del = true) override;
};

class MNM_Routing_Delivery_Hybrid : public MNM_Routing_Hybrid
{
public:
  MNM_Routing_Delivery_Hybrid (
    const std::string &file_folder, macposts::Graph &graph,
    MNM_Statistics *statistics, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    TInt route_frq_fixed = TInt (-1), TInt buffer_len = TInt (-1));
  virtual ~MNM_Routing_Delivery_Hybrid () override;
};

class MNM_IO_Delivery : public MNM_IO
{
public:
  static int build_od_factory_delivery (const std::string &file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_OD_Factory *od_factory,
                                        MNM_Node_Factory *node_factory,
                                        const std::string &file_name
                                        = "MNM_input_od");
  static int build_od_factory_delivery (const std::string &file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_OD_Factory *od_factory,
                                        const std::string &file_name
                                        = "MNM_input_od");
  static int build_demand_multi_OD_seq (const std::string &file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_OD_Factory *od_factory,
                                        const std::string &od_file_name
                                        = "MNM_input_multi_od_seq",
                                        const std::string &demand_file_name
                                        = "MNM_input_multi_od_seq_demand");
};

class MNM_Dta_Delivery : public MNM_Dta
{
public:
  explicit MNM_Dta_Delivery (const std::string &file_folder);
  virtual ~MNM_Dta_Delivery ();

  virtual int initialize () override;
  virtual int build_from_files () override;
  virtual int set_routing () override;
};
