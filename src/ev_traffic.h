#pragma once

#include "delivery_traffic.h"


class MNM_Charging_Station;

class MNM_Veh_Electrified : virtual public MNM_Veh
{
public:
    MNM_Veh_Electrified (TInt ID, TInt start_time, TFlt starting_range = 100.,
                        bool using_roadside_charging = false, TFlt full_range = 200.);
    virtual ~MNM_Veh_Electrified();

    virtual bool need_charging();
    int charge();
    virtual int update_miles_traveled(MNM_Dlink *link) override;

    TFlt m_full_range;  // unit: miles, the miles the vehicle can travel with the full battery
    TFlt m_current_range;  // unit: miles, the miles the vehicle can travel with the remaining battery
    TFlt m_miles_traveled_after_charging;  // miles
    TInt m_charging_timer;
    bool m_using_roadside_charging;
    MNM_Charging_Station *m_charging_station;
    
};

// https://en.wikipedia.org/wiki/Virtual_inheritance
class MNM_Veh_Electrified_Delivery : public MNM_Veh_Electrified, public MNM_Veh_Delivery
{
public:
    MNM_Veh_Electrified_Delivery(TInt ID, TInt start_time, TFlt starting_range = 100.,
                        bool using_roadside_charging = false, TFlt full_range = 200.);
    virtual ~MNM_Veh_Electrified_Delivery();   

    virtual bool need_charging() override; 
};

class MNM_Veh_Factory_EV : public MNM_Veh_Factory_Delivery
{
public:
    MNM_Veh_Factory_EV();
    virtual ~MNM_Veh_Factory_EV();

    MNM_Veh_Electrified* make_veh_electrified(TInt timestamp, Vehicle_type veh_type, bool using_roadside_charging=false);
    MNM_Veh_Electrified_Delivery* make_veh_electrified_delivery(TInt timestamp, Vehicle_type veh_type, bool using_roadside_charging=false);
    
    int set_ev_range(TFlt EV_starting_range_roadside_charging, TFlt EV_starting_range_non_roadside_charging, TFlt EV_full_range);

    TInt m_veh_electrified;
    TInt m_veh_non_roadside_charging;

    TFlt m_starting_range_roadside_charging;
    TFlt m_starting_range_non_roadside_charging;
    TFlt m_full_range;
    // https://www.power-sonic.com/blog/guide-to-level-2-ev-charging/#:~:text=Charging%20speeds%20for%20Level%202,range%20per%20hour%20of%20charging.
    // average 40 kWh battery
    
};

class MNM_Origin_EV : public MNM_Origin_Delivery
{
public:
    MNM_Origin_EV(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency, TInt ev_label, TFlt roadside_charging_ratio = 0.);
    virtual ~MNM_Origin_EV() override;

    virtual int release_one_interval(TInt current_interval, MNM_Veh_Factory* veh_factory,
                                    TInt assign_interval, TFlt adaptive_ratio) override;
    int adjust_multi_OD_seq_veh_routing_type(TFlt adaptive_ratio);

    TInt m_ev_label;
    TFlt m_roadside_charging_ratio;
};

class MNM_OD_Factory_EV : public MNM_OD_Factory_Delivery
{
public:
  MNM_OD_Factory_EV();
  virtual ~MNM_OD_Factory_EV();

  MNM_Origin* make_origin_ev(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency, TInt ev_label, TFlt roadside_charging_ratio);
};

class MNM_Charging_Station : public MNM_Dnode
{
public:
    MNM_Charging_Station(TInt ID, TFlt flow_scalar, int unit_time, int num_slots, int charging_time, int avg_waiting_time, float price);
    virtual ~MNM_Charging_Station() override;
    virtual int evolve(TInt timestamp) override;
    virtual void print_info() override;
    virtual int add_in_link(MNM_Dlink* in_link) override;
    virtual int add_out_link(MNM_Dlink* out_link) override;
    int get_current_estimated_waiting_time();  // intervals
    int check_veh_status();
    int move_out_veh(TInt timestamp, MNM_Veh* veh);
    int save_cc(const std::string& file_name);
    int save_waiting_time_record(const std::string& file_name);

    int m_unit_time;
    int m_slots;
    int m_charging_time;  // number of loading intervals
    int m_avg_waiting_time;  // number of loading intervals
    float m_price;  // us dollar
    
    std::vector<std::deque<MNM_Veh *>> m_queue_pool;
    MNM_Cumulative_Curve *m_N_out;
    std::unordered_map<int, int> m_waiting_time_record;  // time-dependent waiting time in loading intervals
};

class MNM_Node_Factory_EV : public MNM_Node_Factory
{
public:
    MNM_Node_Factory_EV();
    virtual ~MNM_Node_Factory_EV();
    MNM_Dnode *make_charging_station(TInt ID, TFlt flow_scalar, int unit_time, int num_slots, int charging_time, int avg_waiting_time, float price);
};

typedef std::unordered_map<MNM_Origin*, std::unordered_map<MNM_Destination*, std::vector<MNM_Dnode*>*>*> OD_Candidate_POI_Table;
typedef std::unordered_map<MNM_Origin*, std::unordered_map<MNM_Destination*, MNM_Dnode*>*> Best_POI_Table;
typedef std::unordered_map<MNM_Dnode*, std::unordered_map<TInt, TInt>*> Routing_Table2;  

class MNM_Routing_Adaptive_With_POIs : public MNM_Routing_Adaptive
{
public:
    MNM_Routing_Adaptive_With_POIs(const std::string& file_folder, PNEGraph &graph, MNM_Statistics* statistics,
                        MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, 
                        OD_Candidate_POI_Table *od_candidate_poi_table = nullptr);
    virtual ~MNM_Routing_Adaptive_With_POIs() override;
    virtual int init_routing(Path_Table *path_table=nullptr) override;
    virtual int update_link_cost() override;
    virtual int update_routing(TInt timestamp) override;
    virtual int get_POIs();
    virtual int update_best_POI();
    virtual int update_best_POI2();
    int set_shortest_path_tree(std::unordered_map<TInt, TInt> **shortest_path_tree, MNM_Dnode *poi_node=nullptr, MNM_Destination *veh_dest=nullptr);
    int construct_full_od_candidate_poi_table();
    bool check_od_candidate_poi_table_connectivity();

    Routing_Table2 *m_table_POIs;
    std::vector<MNM_Dnode*> m_mid_POIs;
    OD_Candidate_POI_Table *m_od_candidate_poi_table;
    Best_POI_Table* m_best_poi_table;
};

class MNM_Routing_Hybrid_EV : public MNM_Routing
{
public:
    MNM_Routing_Hybrid_EV(const std::string& file_folder, PNEGraph &graph, MNM_Statistics* statistics, MNM_OD_Factory *od_factory,
                                MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, OD_Candidate_POI_Table *od_candidate_poi_table = nullptr,
                                TInt route_frq_fixed = TInt(-1), TInt buffer_len = TInt(-1));
    virtual ~MNM_Routing_Hybrid_EV() override;
    virtual int init_routing(Path_Table *path_table=nullptr) override;
    virtual int update_routing(TInt timestamp) override;
    virtual int remove_finished(MNM_Veh* veh, bool del=true) override;

    MNM_Routing_Adaptive_With_POIs* m_routing_adaptive;
    MNM_Routing_Delivery_Fixed* m_routing_fixed;
};

class MNM_IO_EV : public MNM_IO_Delivery
{
public:
    static int build_od_factory_ev(const std::string& file_folder, MNM_ConfReader *conf_reader,
                                MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, const std::string& file_name = "MNM_input_od");
    static int add_charging_station_node(const std::string& file_folder, MNM_ConfReader *conf_reader, 
                                            MNM_Node_Factory *node_factory, const std::string& file_name = "MNM_input_charging_station");
    static OD_Candidate_POI_Table* load_candidate_poi_table(const std::string& file_folder, MNM_ConfReader *conf_reader, 
                                                                                      MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
                                                                                      const std::string& file_name = "MNM_input_candidate_charging_station");  
    static int save_candidate_poi_table(const OD_Candidate_POI_Table& candidate_poi_table,
                                        const std::string& file_name = "MNM_input_candidate_charging_station");                                           
    
    static int save_charging_station_record(const std::string& file_folder, MNM_Node_Factory *node_factory);

    static int destruct_candidate_poi_table(OD_Candidate_POI_Table *table);
};

class MNM_Dta_EV : public MNM_Dta
{
public:
    explicit MNM_Dta_EV(const std::string& file_folder);
    virtual ~MNM_Dta_EV() override;
    virtual int initialize() override;
    virtual int build_from_files() override;
    virtual int set_routing() override;
    virtual bool is_ok() override;
    virtual int load_once(bool verbose, TInt load_int, TInt assign_int) override;
};