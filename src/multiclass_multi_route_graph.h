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

  std::unordered_map<int, int> m_num_car_subclass;
  std::unordered_map<int, int> m_num_truck_subclass;
  std::unordered_map<int, int> m_enroute_car_subclass;
  std::unordered_map<int, int> m_enroute_truck_subclass;
  std::unordered_map<int, int> m_finished_car_subclass;
  std::unordered_map<int, int> m_finished_truck_subclass;
  std::unordered_map<int, TFlt> m_total_time_car_subclass;
  std::unordered_map<int, TFlt> m_total_time_truck_subclass;
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