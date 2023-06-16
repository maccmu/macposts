#include "ev_traffic.h"

//#################################################################
//                       Electrified Vehicle
//#################################################################

MNM_Veh_Electrified::MNM_Veh_Electrified(TInt ID, TInt start_time, TFlt starting_range, bool using_roadside_charging, TFlt full_range)
      : MNM_Veh::MNM_Veh(ID, start_time) 
{
    m_full_range = full_range;
    m_current_range = starting_range;
    m_miles_traveled_after_charging = 0.;
    m_charging_timer = -1;
    m_using_roadside_charging = using_roadside_charging;
    m_charging_station = nullptr;
}

MNM_Veh_Electrified::~MNM_Veh_Electrified() {
    ;
}

// assumption
bool 
MNM_Veh_Electrified::need_charging() {
    if (m_using_roadside_charging) {
        if (m_miles_traveled <= 0.) {
            // vehicle with single OD pair just being released
            return true;
        }
        if (m_current_range - m_miles_traveled_after_charging <= 0.1 * m_full_range) {
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

int 
MNM_Veh_Electrified::charge() {
    // reset range
    m_current_range = m_full_range;
    // reset timer
    m_charging_timer = 0;
    // reset mile counter
    m_miles_traveled_after_charging = 0.;
    // reset charging station
    m_charging_station = nullptr;
    return 0;
}

int 
MNM_Veh_Electrified::update_miles_traveled(MNM_Dlink *link) {
    if (dynamic_cast<MNM_Dlink_Pq*>(link) == nullptr) {
        m_miles_traveled += link -> m_length / 1600.;  // meter -> mile
        m_miles_traveled_after_charging += link -> m_length / 1600.;
    }
    return 0;
}


//#################################################################
//                 Electrified Delivery Vehicle
//#################################################################

// https://en.wikipedia.org/wiki/Virtual_inheritance
MNM_Veh_Electrified_Delivery::MNM_Veh_Electrified_Delivery(TInt ID, TInt start_time, TFlt starting_range, bool using_roadside_charging, TFlt full_range)
      : MNM_Veh_Electrified::MNM_Veh_Electrified(ID, start_time, starting_range, using_roadside_charging, full_range),
        MNM_Veh_Delivery::MNM_Veh_Delivery(ID, start_time),
        MNM_Veh::MNM_Veh(ID, start_time) 
{
    ;
}

MNM_Veh_Electrified_Delivery::~MNM_Veh_Electrified_Delivery()
{
    ;
}

bool 
MNM_Veh_Electrified_Delivery::need_charging() {
    if (m_using_roadside_charging) {
        if (m_current_range - m_miles_traveled_after_charging <= 0.1 * m_full_range) {
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}


//#################################################################
//                 Electrified Delivery Vehicle
//#################################################################

MNM_Veh_Factory_EV::MNM_Veh_Factory_EV()
    : MNM_Veh_Factory_Delivery::MNM_Veh_Factory_Delivery()
{
    m_veh_electrified = 0;
}

MNM_Veh_Factory_EV::~MNM_Veh_Factory_EV()
{
    ;
}

MNM_Veh_Electrified* 
MNM_Veh_Factory_EV::make_veh_electrified(TInt timestamp, Vehicle_type veh_type, TFlt starting_range, bool using_roadside_charging, TFlt full_range)
{
    MNM_Veh_Electrified *_veh = new MNM_Veh_Electrified(m_num_veh + 1, timestamp, starting_range, using_roadside_charging, full_range);
    _veh -> m_type = veh_type;
    m_veh_map.insert(std::pair<TInt, MNM_Veh*>(m_num_veh + 1, _veh));
    m_num_veh += 1;
    m_enroute += 1;
    m_veh_electrified += 1;
    return _veh;
}

MNM_Veh_Electrified_Delivery* 
MNM_Veh_Factory_EV::make_veh_electrified_delivery(TInt timestamp, Vehicle_type veh_type, TFlt starting_range, bool using_roadside_charging, TFlt full_range)
{
    MNM_Veh_Electrified_Delivery *_veh = new MNM_Veh_Electrified_Delivery (m_num_veh + 1, timestamp, starting_range, using_roadside_charging, full_range);
    _veh->m_type = veh_type;
    m_veh_map.insert (std::pair<TInt, MNM_Veh *> (m_num_veh + 1, _veh));
    m_num_veh += 1;
    m_enroute += 1;
    m_veh_delivery += 1;
    m_veh_electrified += 1;
    return _veh;
}


//#################################################################
//            Origin with Electrified and Delivery Traffic
//#################################################################

MNM_Origin_EV::MNM_Origin_EV(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency, TInt ev_label, TFlt roadside_charging_ratio)
    : MNM_Origin_Delivery::MNM_Origin_Delivery(ID, max_interval, flow_scalar, frequency) {
    IAssert(ev_label >= 0);
    m_ev_label = ev_label;
    m_roadside_charging_ratio = roadside_charging_ratio;
}

MNM_Origin_EV::~MNM_Origin_EV() {
    ;
}

int 
MNM_Origin_EV::adjust_multi_OD_seq_veh_routing_type(TFlt adaptive_ratio) {
    // multiOD seq vehicle from other origins
    for (auto _it : m_origin_node->m_in_veh_queue) {
        if (auto _veh_elec = dynamic_cast<MNM_Veh_Electrified_Delivery*>(_it)) {
            Assert(_veh_elec -> m_multi_od_seq != nullptr);
            // std::cout << "EV with multi OD pairs needs charging after first trip: " << _veh_elec -> need_charging() << std::endl;
            if (_veh_elec->need_charging()){
                Assert(_veh_elec->m_using_roadside_charging = true);
                _veh_elec->m_type = MNM_TYPE_ADAPTIVE;
            }
            else{
                // just assume these vehicle will change routing type based on
                // current origin settings
                // once the veh needs charging, the m_type is changed to adaptive for the remaing OD sequence travel
                // this step can mitigate this
                _it->m_type = MNM_Ults::rand_flt () <= adaptive_ratio
                                ? MNM_TYPE_ADAPTIVE
                                : MNM_TYPE_STATIC;
            }
        }
    }
    return 0;
}

int 
MNM_Origin_EV::release_one_interval(TInt current_interval, MNM_Veh_Factory* veh_factory,
                                             TInt assign_interval, TFlt adaptive_ratio) {

    if (assign_interval < 0)return 0;
    m_current_assign_interval = assign_interval;
    auto _veh_factory_ev = dynamic_cast<MNM_Veh_Factory_EV*>(veh_factory);
    IAssert(_veh_factory_ev != nullptr);
    TInt _veh_to_release, _label;
    MNM_Veh *_veh;
    TFlt _r;

    // normal single-OD demand
    for (auto _demand_it = m_demand.begin (); _demand_it != m_demand.end (); _demand_it++) {
        _veh_to_release = TInt (MNM_Ults::round ((_demand_it->second)[assign_interval] * m_flow_scalar));
        for (int i = 0; i < _veh_to_release; ++i) {

            _label = generate_label (0);
            if (m_ev_label >= 0 && _label == m_ev_label) {
                // EV
                _r = MNM_Ults::rand_flt ();
                if (_r <= m_roadside_charging_ratio) {
                    // EV using roadside charging station and it will be routed to charging station during its trip
                    _veh = _veh_factory_ev->make_veh_electrified(current_interval, MNM_TYPE_ADAPTIVE, 5., true, 200.);
                }
                else {
                    // EV charging at origin or destination
                    _veh = _veh_factory_ev->make_veh_electrified (
                        current_interval,
                        MNM_Ults::rand_flt () <= adaptive_ratio
                            ? MNM_TYPE_ADAPTIVE
                            : MNM_TYPE_STATIC,
                        200., false, 200.);
                }
            }
            else {
                // non-EV
                _veh = _veh_factory_ev->make_veh (
                    current_interval, MNM_Ults::rand_flt () <= adaptive_ratio
                                          ? MNM_TYPE_ADAPTIVE
                                          : MNM_TYPE_STATIC);
            }

            _veh->set_destination (_demand_it->first);
            _veh->set_origin (this);
            // _veh -> m_assign_interval = assign_interval;
            // in case the multiclass modeling has 1-min release interval as
            // the "assign" interval
            _veh->m_assign_interval = int (current_interval / m_frequency);
            _veh->m_label = _label;

            if (dynamic_cast<MNM_Veh_Electrified*>(_veh) != nullptr) {
                if (dynamic_cast<MNM_Veh_Electrified*>(_veh) -> m_using_roadside_charging) {
                    // std::cout << "EV with single OD pair needs charging: " << dynamic_cast<MNM_Veh_Electrified*>(_veh) -> need_charging() << std::endl;
                    Assert(dynamic_cast<MNM_Veh_Electrified*>(_veh) -> need_charging());
                }
            }
            // printf("Pushing vehil, %d\n", m_origin_node -> m_node_ID());
            m_origin_node->m_in_veh_queue.push_back (_veh);
          }
    }

    // multi-OD sequence demand, e.g., delivery vehicles
    // departing from the first origin
    for (auto _demand_it : m_demand_multi_OD_seq) {
        _veh_to_release = TInt (MNM_Ults::round((_demand_it.second)[assign_interval] * m_flow_scalar));
        for (int i = 0; i < _veh_to_release; ++i) {

            _label = generate_label (0);
            if (m_ev_label >= 0 && _label == m_ev_label) {
                _veh = _veh_factory_ev->make_veh_electrified_delivery (
                        current_interval,
                        MNM_Ults::rand_flt () <= adaptive_ratio ? MNM_TYPE_ADAPTIVE : MNM_TYPE_STATIC,
                        0.16,   // TODO: test
                        MNM_Ults::rand_flt () <= m_roadside_charging_ratio, 
                        200.);  // TODO: test
            }
            else {
                // non-EV
                _veh = _veh_factory_ev->make_veh_delivery (
                    current_interval, MNM_Ults::rand_flt () <= adaptive_ratio
                                          ? MNM_TYPE_ADAPTIVE
                                          : MNM_TYPE_STATIC);
            }
            Assert(dynamic_cast<MNM_Veh_Delivery*>(_veh) != nullptr);
            dynamic_cast<MNM_Veh_Delivery*>(_veh)->set_multi_od_seq (_demand_it.first);  // will be used in need_charging()
            // first destination
            _veh->set_destination (_demand_it.first->at (0).second);
            _veh->set_origin (this);
            // _veh -> m_assign_interval = assign_interval;
            // in case the multiclass modeling has 1-min release interval as
            // the "assign" interval
            _veh->m_assign_interval = int (current_interval / m_frequency);
            _veh->m_label = _label;

            if (dynamic_cast<MNM_Veh_Electrified_Delivery*>(_veh) != nullptr) {
                if (dynamic_cast<MNM_Veh_Electrified_Delivery*>(_veh) -> m_using_roadside_charging) {
                    // std::cout << "EV with multi-OD pairs needs charging in its first OD trip: " << dynamic_cast<MNM_Veh_Electrified*>(_veh) -> need_charging() << std::endl;
                    if (dynamic_cast<MNM_Veh_Electrified_Delivery*>(_veh) -> need_charging()) {
                        // if it needs charging in its first OD trip
                        _veh -> m_type = MNM_TYPE_ADAPTIVE;
                    }
                }
            }
            // printf("Pushing vehil, %d\n", m_origin_node -> m_node_ID());
            m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }

    std::random_shuffle(m_origin_node -> m_in_veh_queue.begin(), 
                        m_origin_node -> m_in_veh_queue.end());
    return 0;
}


//#################################################################
//        OD Factory with Electrified and Delivery Traffic
//#################################################################

MNM_OD_Factory_EV::MNM_OD_Factory_EV()
    : MNM_OD_Factory_Delivery::MNM_OD_Factory_Delivery() 
{
    ;
}         

MNM_OD_Factory_EV::~MNM_OD_Factory_EV()
{
    ;
}        

MNM_Origin*
MNM_OD_Factory_EV::make_origin_ev(TInt ID, TInt max_interval, TFlt flow_scalar, TInt frequency, TInt ev_label, TFlt roadside_charging_ratio)
{
    MNM_Origin *_origin = new MNM_Origin_EV(ID, max_interval, flow_scalar, frequency, ev_label, roadside_charging_ratio);
    m_origin_map.insert(std::pair<TInt, MNM_Origin*>(ID, _origin));
    return _origin;
}


//#################################################################
//           Charging Station for Electrified Traffic
//#################################################################

MNM_Charging_Station::MNM_Charging_Station(TInt ID, TFlt flow_scalar, int unit_time, int num_slots, int charging_time, int avg_waiting_time, float price)
    : MNM_Dnode::MNM_Dnode(ID, flow_scalar)
{
    m_unit_time = unit_time;
    m_slots = num_slots;
    m_charging_time = charging_time;
    m_avg_waiting_time = avg_waiting_time;
    m_price = price;
    for (int i=0; i < num_slots * int(flow_scalar); ++i) {
        m_queue_pool.push_back(std::deque<MNM_Veh *>());
    };
    m_N_out = new MNM_Cumulative_Curve();
    m_N_out->add_record(std::pair<TFlt, TFlt>(TFlt(0), TFlt(0)));
    m_waiting_time_record = std::unordered_map<int, int>();
    m_waiting_time_record.insert(std::pair<int, int>(0, avg_waiting_time));  // intervals
}

MNM_Charging_Station::~MNM_Charging_Station()
{
    for (auto _it : m_queue_pool) {
        _it.clear();
    }
    m_queue_pool.clear();
    delete m_N_out;
    m_waiting_time_record.clear();
}

void 
MNM_Charging_Station::print_info() 
{
    printf("Charging station %d, Time-dependent cumulative number of charged vehicles:\n", m_node_ID());
    std::cout << m_N_out -> to_string() << std::endl;
    printf("##########################################################################\n");
}

int 
MNM_Charging_Station::save_cc(const std::string& file_name)
{
    std::ofstream _file;
    _file.open(file_name, std::ofstream::out);
    if (! _file.is_open()){
        std::cout << "MNM_Charging_Station::save_cc, Error happens when open " << file_name << std::endl;
        throw std::runtime_error("Error");
    }
    _file << "loading_interval,cum_vehicle_charged\n";
    _file << m_N_out -> to_string();
    if (_file.is_open()) _file.close();
    return 0;
}

int 
MNM_Charging_Station::save_waiting_time_record(const std::string& file_name)
{
    std::ofstream _file;
    _file.open(file_name, std::ofstream::out);
    if (! _file.is_open()){
        std::cout << "MNM_Charging_Station::save_waiting_time_record, Error happens when open " << file_name << std::endl;
        throw std::runtime_error("Error");
    }
    std::vector<TFlt> _k = std::vector<TFlt>();
    for (auto _it : m_waiting_time_record) {
        _k.push_back(_it.first);
    }
    std::sort(_k.begin(), _k.end());
    std::string _s = "loading_interval,waiting_time_in_second\n";
    for (auto i : _k) {
        auto _it = *m_waiting_time_record.find(i);
        _s += std::to_string(_it.first) + "," + std::to_string(_it.second * m_unit_time) + "\n";
    }
    _s.pop_back();
    _file << _s;
    if (_file.is_open()) _file.close();
    return 0;
}

int 
MNM_Charging_Station::add_in_link(MNM_Dlink* in_link)
{   
    if (auto _in_link_pq = dynamic_cast<MNM_Dlink_Pq*>(in_link)) {
        m_in_link_array.push_back(_in_link_pq);
    }
    else {
        throw std::runtime_error("MNM_Charging_Station::add_in_link, incoming link for charging station must be PQ link");
    }
    return 0;
}

int 
MNM_Charging_Station::add_out_link(MNM_Dlink* out_link)
{
    if (auto _out_link_pq = dynamic_cast<MNM_Dlink_Pq*>(out_link)) {
        m_out_link_array.push_back(_out_link_pq);
    }
    else {
        throw std::runtime_error("MNM_Charging_Station::add_out_link, outgoing link for charging station must be PQ link");
    }
    return 0;
}

int 
MNM_Charging_Station::check_veh_status() {
    for (auto _queue : m_queue_pool) {
        for (auto _veh : _queue) {
            // if (dynamic_cast<MNM_Veh_Electrified*>(_veh) == nullptr) {
            // if (dynamic_cast<MNM_Veh_Electrified*>(_veh) -> m_charging_station == nullptr) {
            if (std::find(m_out_link_array.begin(), m_out_link_array.end(), _veh -> get_next_link()) == m_out_link_array.end()) {
            // if (_veh -> get_current_link() == nullptr || _veh -> m_finish_time > 0) {
            // if (_veh -> get_current_link() == nullptr || _veh -> get_next_link() == nullptr || _veh -> m_finish_time > 0) {
                throw std::runtime_error("Something is wrong\n");
            }
        }
    }
    return 0;
}

int MNM_Charging_Station::evolve(TInt timestamp)
{   
    MNM_Dlink *_in_link; // *_out_link;

    std::vector<size_t> _in_link_ind_array = std::vector<size_t>();
    for (size_t i=0; i<m_in_link_array.size(); ++i){
        _in_link_ind_array.push_back(i);
    }

    // shuffle the in links, reserve the FIFO
    std::random_device rng; // random sequence
    std::shuffle(_in_link_ind_array.begin(), _in_link_ind_array.end(), rng);

    // move all in_link vehicles to queue, assuming unlimited waiting space
    for (size_t i : _in_link_ind_array) {
        _in_link = m_in_link_array[i];
        auto _veh_it = _in_link->m_finished_array.begin();
        while (_veh_it != _in_link->m_finished_array.end()){
            if (std::find(m_out_link_array.begin(), m_out_link_array.end(), (*_veh_it) -> get_next_link()) == m_out_link_array.end()) {
                printf("MNM_Charging_Station::evolve, vehicle in the wrong node, no exit!\n");
                printf("MNM_Charging_Station::evolve, vehicle is on link %d, node %d, next link ID is: %d\n", _in_link -> m_link_ID(), m_node_ID(), (*_veh_it) -> get_next_link() -> m_link_ID());
                exit(-1);
            }
            auto _veh = dynamic_cast<MNM_Veh_Electrified*>(*_veh_it);
            if (_veh == nullptr || _veh -> m_type != MNM_TYPE_ADAPTIVE || _veh -> m_charging_station != this) {
                throw std::runtime_error("wrong routing for this vehicle");
                // TODO: actually non-EV or ev using other charging station should not be here
                // move_out_veh(timestamp, *_veh_it);
                // _veh_it = _in_link->m_finished_array.erase(_veh_it);
                // continue;
            }
            // if (_veh -> need_charging() && _veh -> m_charging_station != this) {
            //     throw std::runtime_error("MNM_Charging_Station::evolve, electrified vehicle in the wrong charging station\n");
            // }
            
            // entering the shortest queue
            auto _result = std::min_element(m_queue_pool.begin(), m_queue_pool.end(), 
                                            [](const std::deque<MNM_Veh*> &q1, const std::deque<MNM_Veh*> &q2){return q1.size() < q2.size();});
            m_queue_pool.at(std::distance(m_queue_pool.begin(), _result)).push_back(_veh);
            _veh_it = _in_link->m_finished_array.erase(_veh_it);
                                                                    
        }
    }

    // charge and move vehicles finishing charging to out_link
    for (size_t i = 0; i < m_queue_pool.size(); ++i) {
        auto _queue = m_queue_pool.at(i);  // queue is actually a copy, not the original element of m_queue_pool
        if (_queue.empty()) continue;
        auto _veh = dynamic_cast<MNM_Veh_Electrified*>(_queue.front());
        if (_veh -> m_charging_timer < 0) _veh -> m_charging_timer = 0;
        if (_veh -> m_charging_timer >= m_charging_time) {
            // reset range
            _veh -> charge();
            // release this vehicle
            m_queue_pool.at(i).pop_front();  // DO NOT use _queue.pop_front(), which will not modify m_queue_pool
            move_out_veh(timestamp, _veh);
            // update charging station cc
            m_N_out -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
        }
        else {
            _veh -> m_charging_timer += 1;
        }
    }

    // update estimated waiting time
    int _est_waiting_time = get_current_estimated_waiting_time();
    m_waiting_time_record.insert(std::pair<int, int>(timestamp + 1, _est_waiting_time));                                                  
    return 0;
}

int 
MNM_Charging_Station::get_current_estimated_waiting_time() {
    std::deque<MNM_Veh*> _shortest_queue = *std::min_element(m_queue_pool.begin(), m_queue_pool.end(), 
                                                             [](const std::deque<MNM_Veh*> &q1, const std::deque<MNM_Veh*> &q2){return q1.size() < q2.size();});                          
    // int _est_waiting_time = _shortest_queue.size() > 0 ? _shortest_queue.size() * m_charging_time : m_avg_waiting_time;
    // if (_est_waiting_time < m_avg_waiting_time) _est_waiting_time = m_avg_waiting_time;
    int _est_waiting_time = _shortest_queue.size() * m_charging_time;
    return _est_waiting_time;  // intervals
}

int 
MNM_Charging_Station::move_out_veh(TInt timestamp, MNM_Veh *veh)
{
    auto _in_link = veh -> get_current_link();  // veh will be popped out from in_link outside this function
    auto _out_link = veh -> get_next_link();
    _out_link ->m_incoming_array.push_back(veh);
    veh -> set_current_link(_out_link);
    
    // update cc
    if (_in_link -> m_N_out != nullptr) {
        // printf("record in link cc: link ID %d, time %d, value %f\n", _in_link -> m_link_ID(), timestamp()+1, (float) TFlt(1)/m_flow_scalar);
        _in_link -> m_N_out -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
    }
    if (_out_link -> m_N_in != nullptr) {
        // printf("record out link cc: link ID %d, time %d, value %f\n", _out_link -> m_link_ID(), timestamp()+1, (float) TFlt(1)/m_flow_scalar);
        _out_link -> m_N_in -> add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1), TFlt(1)/m_flow_scalar));
    }

    // update cc_tree
    if (_out_link -> m_N_in_tree != nullptr) {
        // printf("record out link cc tree: link ID %d, time %d, path id %d, assign interval %d\n", _out_link -> m_link_ID(), timestamp()+1, _veh -> m_path -> m_path_ID(), _veh -> m_assign_interval());
        _out_link -> m_N_in_tree -> add_flow(TFlt(timestamp + 1), TFlt(1)/m_flow_scalar, veh -> m_path, veh -> m_assign_interval);
    }
    if (_in_link -> m_N_out_tree != nullptr) {
        // printf("record in link cc tree: link ID %d, time %d, path id %d, assign interval %d\n", _in_link -> m_link_ID(), timestamp()+1, _veh -> m_path -> m_path_ID(), _veh -> m_assign_interval());
        _in_link -> m_N_out_tree -> add_flow(TFlt(timestamp + 1), TFlt(1)/m_flow_scalar, veh -> m_path, veh -> m_assign_interval);
    }
    return 0;
}


//#################################################################
//           Node Factory with Electrified and Delivery Traffic
//#################################################################

MNM_Node_Factory_EV::MNM_Node_Factory_EV()
    : MNM_Node_Factory::MNM_Node_Factory()
{
    ;
}

MNM_Node_Factory_EV::~MNM_Node_Factory_EV() 
{
    ;
}

MNM_Dnode* 
MNM_Node_Factory_EV::make_charging_station(TInt ID, TFlt flow_scalar, int unit_time, int num_slots, int charging_time, int avg_waiting_time, float price)
{
    if (m_node_map.empty()) {
        throw std::runtime_error("MNM_Node_Factory_With_EV::make_charging_station, must add node first");
    }
    if (m_node_map.find(ID) != m_node_map.end()) {
        throw std::runtime_error("MNM_Node_Factory_With_EV::make_charging_station, charging station ID must be different from any existing node ID");
    }
    MNM_Dnode *_node = new MNM_Charging_Station(ID, flow_scalar, unit_time, num_slots, charging_time, avg_waiting_time, price);
    m_node_map.insert(std::pair<TInt, MNM_Dnode*>(ID, _node));
    return _node;
}


//#################################################################
//       Adaptive Routing for Electrified and Delivery Traffic
//#################################################################

MNM_Routing_Adaptive_With_POIs::MNM_Routing_Adaptive_With_POIs(const std::string& file_folder, PNEGraph &graph, MNM_Statistics* statistics,
                       MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, OD_Candidate_POI_Table *od_candidate_poi_table)
                       : MNM_Routing_Adaptive::MNM_Routing_Adaptive(file_folder, graph, statistics, od_factory, node_factory, link_factory)
{
    // hopefully this will not route non-EV to charging station since extra delay is added at charging station
    get_POIs();
    m_od_candidate_poi_table = od_candidate_poi_table;
    if (m_od_candidate_poi_table -> empty()) {
        construct_full_od_candidate_poi_table();  // use all possible OD pairs from OD_factory
    }

    m_table_POIs = new Routing_Table2();

    // initialize
    m_best_poi_table = new Best_POI_Table();
    for (auto _it : *m_od_candidate_poi_table) {
        if (m_best_poi_table -> find(_it.first) == m_best_poi_table -> end()) {
            m_best_poi_table -> insert(std::make_pair(_it.first, new std::unordered_map<MNM_Destination*, MNM_Dnode*>()));
        }
        for (auto _it_it : *(_it.second)) {
            if (m_best_poi_table -> find(_it.first) -> second -> find(_it_it.first) == m_best_poi_table -> find(_it.first) -> second -> end()) {
                m_best_poi_table -> find(_it.first) -> second -> insert(std::make_pair(_it_it.first, nullptr));
            }
        }
    }
}

MNM_Routing_Adaptive_With_POIs::~MNM_Routing_Adaptive_With_POIs()
{
    for (auto _it : m_mid_POIs)
    {
        if (m_table_POIs->find (_it) != m_table_POIs->end ())
        {
            m_table_POIs->find (_it)->second->clear ();
            delete m_table_POIs->find (_it)->second;
        }
    }
    m_table_POIs->clear ();
    delete m_table_POIs;
    
    m_mid_POIs.clear ();

    if (m_od_candidate_poi_table != nullptr) {
        MNM_IO_EV::destruct_candidate_poi_table(m_od_candidate_poi_table);
    }

    for (auto _it : *m_best_poi_table) {
        _it.second -> clear();
        delete _it.second;
    }
    m_best_poi_table -> clear();
    delete m_best_poi_table;
}

int 
MNM_Routing_Adaptive_With_POIs::get_POIs() {
    if (!m_mid_POIs.empty()) m_mid_POIs.clear();
    for (auto _it : m_node_factory -> m_node_map) {
        if (auto _charging_station = dynamic_cast<MNM_Charging_Station*>(_it.second)) {
            m_mid_POIs.push_back(_charging_station);
        }
    }
    return 0;
}

int 
MNM_Routing_Adaptive_With_POIs::construct_full_od_candidate_poi_table()
{
    MNM_Origin *_origin;
    MNM_Destination *_dest;
    for (auto _it1 : m_od_factory -> m_origin_map) {
        _origin = _it1.second;
        for (auto _it2 : m_od_factory -> m_destination_map) {
            _dest = _it2.second;
            if (m_od_candidate_poi_table -> find(_origin) == m_od_candidate_poi_table -> end()) {
                m_od_candidate_poi_table -> insert(std::make_pair(_origin, new std::unordered_map<MNM_Destination*, std::vector<MNM_Dnode*>*>()));
            }
            if (m_od_candidate_poi_table -> find(_origin) -> second -> find(_dest) == m_od_candidate_poi_table -> find(_origin) -> second -> end()) {
                m_od_candidate_poi_table -> find(_origin) -> second -> insert(std::make_pair(_dest, new std::vector<MNM_Dnode*>()));
            }
            for (auto _charging_station : m_mid_POIs) {
                m_od_candidate_poi_table -> find(_origin) -> second -> find(_dest) -> second -> push_back(_charging_station);
            }
        }
    }
    return 0;
}

bool 
MNM_Routing_Adaptive_With_POIs::check_od_candidate_poi_table_connectivity()
{
    MNM_Origin *_origin;
    MNM_Destination *_dest;

    std::unordered_map<TInt, TInt> _shortest_path_tree = std::unordered_map<TInt, TInt>();
    std::unordered_map<TInt, TFlt> _cost_map;
    for (auto _map_it : m_link_factory -> m_link_map){
        _cost_map.insert(std::pair<TInt, TFlt>(_map_it.first, TFlt(1)));
    }

    for (auto _it : *m_od_candidate_poi_table) {
        _origin = _it.first;
        for (auto _it_it : *(_it.second)) {
            _dest = _it_it.first;

            // check and modify
            auto _node_it = _it_it.second -> begin();
            while (_node_it != _it_it.second -> end()) {
                auto _node = *_node_it;

                // _origin to _node
                if (!_shortest_path_tree.empty()) _shortest_path_tree.clear();
                MNM_Shortest_Path::all_to_one_FIFO(_node -> m_node_ID, m_graph, _cost_map, _shortest_path_tree);
                if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID)-> second == -1){
                    printf("Disconnectivity in Origin %d (Node: %d) - Destination %d (Node: %d) pair, cannot travel from Origin %d (Node: %d) to POI Node %d", 
                        _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(),
                        _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID(),
                        _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(), _node -> m_node_ID());
                    _node_it = _it_it.second -> erase(_node_it);
                    continue;
                }

                // _node to _dest
                if (!_shortest_path_tree.empty()) _shortest_path_tree.clear();
                MNM_Shortest_Path::all_to_one_FIFO(_dest -> m_dest_node -> m_node_ID, m_graph, _cost_map, _shortest_path_tree);
                if (_shortest_path_tree.find(_node -> m_node_ID)-> second == -1){
                    printf("Disconnectivity in Origin %d (Node: %d) - Destination %d (Node: %d) pair, cannot travel from POI Node %d to Destination %d (Node: %d)", 
                        _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(),
                        _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID(),
                        _node -> m_node_ID(), _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID());
                    _node_it = _it_it.second -> erase(_node_it);
                    continue;
                }

                _node_it++;
            }

            // // check
            // for (auto _node : *(_it_it.second)) {

            //     // _origin to _node
            //     if (!_shortest_path_tree.empty()) _shortest_path_tree.clear();
            //     MNM_Shortest_Path::all_to_one_FIFO(_node -> m_node_ID, m_graph, _cost_map, _shortest_path_tree);
            //     if (_shortest_path_tree.find(_origin -> m_origin_node -> m_node_ID)-> second == -1){
            //         printf("Disconnectivity in Origin %d (Node: %d) - Destination %d (Node: %d) pair, cannot travel from Origin %d (Node: %d) to POI Node %d", 
            //             _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(),
            //             _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID(),
            //             _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(), _node -> m_node_ID());
            //         return false;
            //     }

            //     // _node to _dest
            //     if (!_shortest_path_tree.empty()) _shortest_path_tree.clear();
            //     MNM_Shortest_Path::all_to_one_FIFO(_dest -> m_dest_node -> m_node_ID, m_graph, _cost_map, _shortest_path_tree);
            //     if (_shortest_path_tree.find(_node -> m_node_ID)-> second == -1){
            //         printf("Disconnectivity in Origin %d (Node: %d) - Destination %d (Node: %d) pair, cannot travel from POI Node %d to Destination %d (Node: %d)", 
            //             _origin -> m_Origin_ID(), _origin -> m_origin_node -> m_node_ID(),
            //             _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID(),
            //             _node -> m_node_ID(), _dest -> m_Dest_ID(), _dest -> m_dest_node -> m_node_ID());
            //         return false;
            //     }
                
            // }
        }
    }
    return true;
}

int 
MNM_Routing_Adaptive_With_POIs::init_routing(Path_Table *path_table)
{   
    if (m_statistics -> m_self_config -> get_int("rec_tt") == 0) {
        throw std::runtime_error("MNM_Routing_Adaptive_With_POIs::init_routing, rec_tt should be set to 1 in config.conf to use adaptive routing");
    }
    MNM_Routing_Adaptive::init_routing (path_table);
    std::unordered_map<TInt, TInt> *_shortest_path_tree;
    for (auto _it : m_mid_POIs)
    {
        _shortest_path_tree = new std::unordered_map<TInt, TInt> ();
        m_table_POIs->insert(std::pair<MNM_Dnode *, std::unordered_map<TInt, TInt> *> (_it, _shortest_path_tree));
    }
    return 0;
}

int 
MNM_Routing_Adaptive_With_POIs::update_link_cost()
{   
    MNM_Dlink *_link;
    for (auto _it : m_statistics->m_record_interval_tt)
    { // seconds
        _link = m_link_factory->get_link (_it.first);
        // for multiclass, m_toll is for car, see MNM_IO_Multiclass::build_link_toll_multiclass
        m_link_cost[_it.first] = _it.second * m_vot + _link ->m_toll;
        // if it is charging station, add waiting time + charging time
        if (dynamic_cast<MNM_Dlink_Pq*>(_link) != nullptr) {
            if (auto _charging_station = dynamic_cast<MNM_Charging_Station*>(_link -> m_to_node)) {
                m_link_cost[_it.first] += m_vot * (_charging_station -> get_current_estimated_waiting_time() + 
                                                   _charging_station -> m_charging_time) * _charging_station -> m_unit_time + 
                                          _charging_station -> m_price;
            }
        }
    }
    return 0;
}

int 
MNM_Routing_Adaptive_With_POIs::set_shortest_path_tree(std::unordered_map<TInt, TInt> **shortest_path_tree, MNM_Dnode *poi_node, MNM_Destination *veh_dest) 
{

    if (poi_node != nullptr && veh_dest == nullptr) {
        if (m_table_POIs->find(poi_node) == m_table_POIs -> end()) {
            throw std::runtime_error("MNM_Routing_Adaptive_With_POIs::set_shortest_path_tree, cannot find middle node in m_table_POIs");
        }
        *shortest_path_tree = m_table_POIs->find(poi_node)->second;
    }
    else if (poi_node == nullptr && veh_dest != nullptr) {
        if (m_table->find(veh_dest) == m_table -> end()) {
            throw std::runtime_error("MNM_Routing_Adaptive_With_POIs::set_shortest_path_tree, cannot find destination in m_table");
        }
        *shortest_path_tree = m_table->find(veh_dest)->second;
    }
    else {
        throw std::runtime_error("MNM_Routing_Adaptive_With_POIs::set_shortest_path_tree, either poi_node or veh_dest can be nullptr, cannot be both");
    }
    return 0;
}

int 
MNM_Routing_Adaptive_With_POIs::update_routing(TInt timestamp)
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
        update_link_cost();
        for (auto _it = m_od_factory->m_destination_map.begin (); _it != m_od_factory->m_destination_map.end (); _it++)
        {
            // #pragma omp task firstprivate(_it)
            // {
            _dest = _it->second;
            _dest_node_ID = _dest->m_dest_node->m_node_ID;
            // printf("Destination ID: %d\n", (int) _dest_node_ID);
            _shortest_path_tree = m_table->find(_dest)->second;
            MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_graph, m_link_cost, *_shortest_path_tree);
            // MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_graph,
            // m_statistics -> m_record_interval_tt, *_shortest_path_tree);
            // MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID, m_graph,
            // m_statistics -> m_record_interval_tt, *_shortest_path_tree);
            // }
        }
        for (auto _it : m_mid_POIs)
        {
            _shortest_path_tree = m_table_POIs->find(_it)->second;
            MNM_Shortest_Path::all_to_one_FIFO(_it->m_node_ID, m_graph, m_link_cost, *_shortest_path_tree);
        }
        update_best_POI();
        // update_best_POI2();
    }

    /* route the vehicle in Origin nodes */
    // printf("Routing the vehicle!\n");
    MNM_Origin *_origin;
    MNM_DMOND *_origin_node;
    TInt _node_ID, _next_link_ID;
    MNM_Dlink *_next_link;
    MNM_Dnode *_poi_node;
    MNM_Destination *_veh_dest;
    for (auto _origin_it = m_od_factory->m_origin_map.begin(); _origin_it != m_od_factory->m_origin_map.end(); _origin_it++)
    {
        _origin = _origin_it->second;
        _origin_node = _origin->m_origin_node;
        _node_ID = _origin_node->m_node_ID;
        for (auto _veh : _origin_node->m_in_veh_queue)
        {
            if (_veh->m_type == MNM_TYPE_ADAPTIVE)
            {
                _veh_dest = _veh->get_destination ();
                _shortest_path_tree = nullptr;
                if (auto _veh_electrified = dynamic_cast<MNM_Veh_Electrified *> (_veh))
                {
                    if (_veh_electrified -> need_charging())
                    {
                        if (m_best_poi_table -> find(_origin) == m_best_poi_table -> end() || 
                            m_best_poi_table -> find(_origin) -> second -> find(_veh_dest) == m_best_poi_table -> find(_origin) -> second -> end()) {
                            set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                        }
                        else {
                            _poi_node = m_best_poi_table -> find(_origin) -> second -> find(_veh_dest) -> second;
                            if (_poi_node != nullptr) {
                                _veh_electrified -> m_charging_station = dynamic_cast<MNM_Charging_Station*>(_poi_node);
                                set_shortest_path_tree(&_shortest_path_tree, _poi_node, nullptr);
                            }
                            else {
                                set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                            }
                        }
                    }
                    else
                    {
                        set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                    }
                }
                else
                {
                    set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                }
                IAssert(_shortest_path_tree != nullptr);
                _next_link_ID = _shortest_path_tree->find (_node_ID)->second;
                if (_next_link_ID < 0)
                {
                    // printf("%d\n", _veh -> get_destination() -> m_Dest_ID);
                    // _shortest_path_tree = m_table -> find(_veh ->
                    // get_destination()) -> second; printf("%d\n",
                    // _shortest_path_tree -> size()); for (auto it :
                    // (*_shortest_path_tree)) printf("%d, %d\n", it.first,
                    // it.second);
                    throw std::runtime_error ("MNM_Routing_Adaptive_With_POIs::update_routing, Something wrong in adaptive routing, wrong next link 1");
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

    MNM_Dlink *_link;
    for (auto _link_it = m_link_factory->m_link_map.begin (); _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
        _link = _link_it->second;
        _node_ID = _link->m_to_node->m_node_ID;
        for (auto _veh : _link->m_finished_array)
        {
            if (_veh->m_type == MNM_TYPE_ADAPTIVE)
            {
                if (_link != _veh->get_current_link ())
                {
                    throw std::runtime_error ("MNM_Routing_Adaptive_With_POIs::update_routing, wrong current link");
                }

                _veh_dest = _veh->get_destination ();
                if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                {
                    _veh->set_next_link (nullptr);
                }
                else
                {   
                    _shortest_path_tree = nullptr;
                    if (auto _veh_electrified = dynamic_cast<MNM_Veh_Electrified *> (_veh))
                    {
                        if (_veh_electrified -> need_charging())
                        {
                            if (_veh_electrified -> m_charging_station != nullptr) {
                                if (_veh_electrified -> m_charging_station -> m_node_ID != _node_ID) {
                                    set_shortest_path_tree(&_shortest_path_tree, _veh_electrified -> m_charging_station, nullptr);
                                }
                                else {
                                    set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                                }
                            }
                            else {
                                set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                            }
                        }
                        else
                        {
                            IAssert(_veh_electrified -> m_charging_station == nullptr);
                            set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                        }
                    }
                    else {
                        set_shortest_path_tree(&_shortest_path_tree, nullptr, _veh_dest);
                    }
                    IAssert(_shortest_path_tree != nullptr);
                    _next_link_ID = _shortest_path_tree -> find(_node_ID) -> second;

                    if (_next_link_ID == -1)
                    {
                        printf ("MNM_Routing_Adaptive_With_POIs::update_routing, Something wrong in routing, wrong next link 2\n");
                        printf (
                            "MNM_Routing_Adaptive_With_POIs::update_routing, The node is %d, the vehicle should head to %d\n",
                            (int)_node_ID,
                            (int)_veh_dest->m_dest_node->m_node_ID);
                        // exit(-1);
                        auto _node_I = m_graph->GetNI (_node_ID);
                        if (_node_I.GetOutDeg () > 0)
                        {
                            printf ("MNM_Routing_Adaptive_With_POIs::update_routing, Assign randomly a next link!\n");
                            _next_link_ID = _node_I.GetOutEId (MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                        }
                        else
                        {
                            throw std::runtime_error ("MNM_Routing_Adaptive_With_POIs::update_routing, cannot do anything to find a next link");
                        }
                    }
                    _next_link = m_link_factory->get_link (_next_link_ID);
                    if (_next_link != nullptr)
                    {
                        // printf("Checking future\n");
                        TInt _next_node_ID = _next_link->m_to_node->m_node_ID;

                        _shortest_path_tree = nullptr;
                        if (auto _veh_electrified = dynamic_cast<MNM_Veh_Electrified *> (_veh))
                        {
                            if (_veh_electrified -> need_charging())
                            {
                                if (_veh_electrified -> m_charging_station != nullptr) {
                                    if (_veh_electrified -> m_charging_station -> m_node_ID != _next_node_ID) {
                                        _shortest_path_tree = m_table_POIs->find(_veh_electrified -> m_charging_station)->second;
                                    }
                                }
                                else {
                                    if (_next_node_ID != _veh_dest ->m_dest_node->m_node_ID) {
                                        _shortest_path_tree = m_table->find(_veh_dest)->second;
                                    }
                                }
                            }
                            else
                            {
                                Assert(_veh_electrified -> m_charging_station == nullptr);
                                if (_next_node_ID != _veh_dest ->m_dest_node->m_node_ID) {
                                    _shortest_path_tree = m_table->find(_veh_dest)->second;
                                }
                            }
                        }
                        else {
                            if (_next_node_ID != _veh_dest ->m_dest_node->m_node_ID) {
                                _shortest_path_tree = m_table->find(_veh_dest)->second;
                            }
                        }

                        if (_shortest_path_tree != nullptr)
                        {
                            if (_shortest_path_tree->find (_next_node_ID) == _shortest_path_tree->end ())
                            {
                                throw std::runtime_error ("MNM_Routing_Adaptive_With_POIs::update_routing, Cannot find _next_node_ID");
                            }
                            if (_shortest_path_tree->find (_next_node_ID) ->second == -1)
                            {
                                throw std::runtime_error ("MNM_Routing_Adaptive_With_POIs::update_routing, Something wrong for the future node");
                            }
                            // printf("Pass checking\n");
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


int 
MNM_Routing_Adaptive_With_POIs::update_best_POI() 
{
    MNM_Origin* _origin;
    MNM_Destination* _dest;
    std::unordered_map<TInt, TInt> *_shortest_path_tree;
    MNM_Path *_path;
    TFlt _path_cost, _current_best_path_cost;
    MNM_Dnode *_current_best_poi_node;
    for (auto _it : *m_od_candidate_poi_table) {
        _origin = _it.first;
        for (auto _it_it : *(_it.second)) {
            _dest = _it_it.first;
            _current_best_poi_node = nullptr;
            if (_it_it.second -> empty()) {
                // printf("MNM_Routing_Adaptive_With_POIs::update_best_POI, empty candidate POI list\n");
                // exit(-1);
                m_best_poi_table -> find(_origin) -> second -> find(_dest) -> second = _current_best_poi_node;
                continue;
            }
            _current_best_path_cost = std::numeric_limits<double>::infinity();
            for (auto _node : *(_it_it.second)) {
                // mid_node to dest
                _shortest_path_tree = m_table -> find(_dest) -> second;
                _path = nullptr;
                _path = MNM::extract_path(_node -> m_node_ID, _dest -> m_dest_node -> m_node_ID, *_shortest_path_tree, m_graph);
                IAssert(_path != nullptr);
                _path_cost = MNM::get_path_tt_snapshot(_path, m_link_cost);
                delete _path;

                // origin to mid_node
                _shortest_path_tree = m_table_POIs -> find(_node) -> second;
                _path = nullptr;
                _path = MNM::extract_path(_origin -> m_origin_node -> m_node_ID, _node -> m_node_ID, *_shortest_path_tree, m_graph);
                IAssert(_path != nullptr);
                _path_cost += MNM::get_path_tt_snapshot(_path, m_link_cost);
                delete _path;

                if (_current_best_path_cost > _path_cost) {
                    _current_best_poi_node = _node;
                    _current_best_path_cost = _path_cost;
                }
            }
            Assert(_current_best_poi_node != nullptr);
            m_best_poi_table -> find(_origin) -> second -> find(_dest) -> second = _current_best_poi_node;
        }
    }
    return 0;
}

int 
MNM_Routing_Adaptive_With_POIs::update_best_POI2()
{
    MNM_Origin* _origin;
    MNM_Destination* _dest;
    // std::unordered_map<TInt, TInt> *_shortest_path_tree;
    MNM_Path *_path;
    TFlt _path_cost, _current_best_path_cost;
    MNM_Dnode *_current_best_poi_node;
    for (auto _it : *m_od_candidate_poi_table) {
        _origin = _it.first;
        for (auto _it_it : *(_it.second)) {
            _dest = _it_it.first;
            _current_best_poi_node = nullptr;
            if (_it_it.second -> empty()) {
                // printf("MNM_Routing_Adaptive_With_POIs::update_best_POI, empty candidate POI list\n");
                // exit(-1);
                m_best_poi_table -> find(_origin) -> second -> find(_dest) -> second = _current_best_poi_node;
                continue;
            }
            _current_best_path_cost = std::numeric_limits<double>::infinity();
            for (auto _node : *(_it_it.second)) {
                _path_cost = dynamic_cast<MNM_Charging_Station*>(_node) -> get_current_estimated_waiting_time();
                if (_current_best_path_cost > _path_cost) {
                    _current_best_poi_node = _node;
                    _current_best_path_cost = _path_cost;
                }
            }
            Assert(_current_best_poi_node != nullptr);
            m_best_poi_table -> find(_origin) -> second -> find(_dest) -> second = _current_best_poi_node;
        }
    }
    return 0;
}


//#################################################################
//       Hybrid Routing for Electrified and Delivery Traffic
//#################################################################

MNM_Routing_Hybrid_EV::MNM_Routing_Hybrid_EV(const std::string& file_folder, PNEGraph &graph, MNM_Statistics* statistics, MNM_OD_Factory *od_factory,
                                            MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory, OD_Candidate_POI_Table *od_candidate_poi_table,
                                            TInt route_frq_fixed, TInt buffer_len)
    : MNM_Routing::MNM_Routing(graph, od_factory, node_factory, link_factory)
{
    m_routing_fixed = new MNM_Routing_Delivery_Fixed(graph, od_factory, node_factory, link_factory, route_frq_fixed, buffer_len);
    m_routing_adaptive = new MNM_Routing_Adaptive_With_POIs(file_folder, graph, statistics, od_factory, node_factory, link_factory, od_candidate_poi_table);
    m_routing_adaptive -> m_working = true;
}

MNM_Routing_Hybrid_EV::~MNM_Routing_Hybrid_EV()
{
    delete m_routing_adaptive;
    delete m_routing_fixed;
}

int 
MNM_Routing_Hybrid_EV::init_routing(Path_Table *path_table)
{
    m_routing_adaptive -> init_routing(path_table);
    m_routing_fixed -> init_routing(path_table);
    return 0;
}

int 
MNM_Routing_Hybrid_EV::update_routing(TInt timestamp)
{
    m_routing_adaptive -> update_routing(timestamp);
    m_routing_fixed -> update_routing(timestamp);
    return 0;
}

int 
MNM_Routing_Hybrid_EV::remove_finished(MNM_Veh *veh, bool del)
{
    m_routing_fixed -> remove_finished(veh, del);
    return 0;
}


//#################################################################
//           IO for Electrified and Delivery Traffic
//#################################################################

int 
MNM_IO_EV::build_od_factory_ev(const std::string& file_folder, MNM_ConfReader *conf_reader,
                                MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, const std::string& file_name)
{
    auto _od_factory_ev = dynamic_cast<MNM_OD_Factory_EV*>(od_factory);
    Assert(_od_factory_ev != nullptr);

    /* find file */
    std::string _od_file_name = file_folder + "/" + file_name;
    std::ifstream _od_file;
    _od_file.open (_od_file_name, std::ios::in);

    /* read config */
    TInt _unit_time = conf_reader->get_int ("unit_time");
    TInt _num_of_O = conf_reader->get_int ("num_of_O");
    TInt _num_of_D = conf_reader->get_int ("num_of_D");
    TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
    TInt _max_interval = conf_reader->get_int ("max_interval");
    TInt _frequency = conf_reader->get_int ("assign_frq");
    TInt _ev_label = conf_reader -> get_int("ev_label");

    /* build */
    TInt _dest_ID, _origin_ID, _node_ID;
    TFlt _pickup_waiting_time, _roadside_charging_ratio;
    std::string _line;
    std::vector<std::string> _words;
    MNM_Origin *_origin;
    MNM_Destination *_dest;
    if (_od_file.is_open ())
    {
        // printf("Start build Origin-Destination factory.\n");
        std::getline (_od_file, _line); // skip the first line
        // printf("Processing Origin node.\n");
        for (int i = 0; i < _num_of_O; ++i)
        {
            std::getline (_od_file, _line);
            _words = split (_line, ' ');
            // O_ID, node_ID, (pickup_waiting_time (seconds), roadside_charging_ratio)
            if (_words.size() == 2 || _words.size () == 3 || _words.size () == 4)
            {
                // std::cout << "Processing: " << _line << "\n";
                _origin_ID = TInt (std::stoi (_words[0]));
                _node_ID = TInt (std::stoi (_words[1]));
                _pickup_waiting_time = 0.;
                _roadside_charging_ratio = 0.;
                if (_words.size() == 3)
                {
                    _pickup_waiting_time = TFlt (std::stod (_words[2])); // second
                    
                }
                else if (_words.size() == 4)
                {
                    _pickup_waiting_time = TFlt (std::stod (_words[2])); // second
                    _roadside_charging_ratio = TFlt (std::stod (_words[3]));
                }
                else if (_words.size() > 4) {
                    throw std::runtime_error("Something is wrong in origin part in od input file");
                }
                _origin = _od_factory_ev->make_origin_ev(_origin_ID, _max_interval, _flow_scalar, _frequency, _ev_label, _roadside_charging_ratio);
                dynamic_cast<MNM_Origin_EV*>(_origin)->m_pickup_waiting_time = MNM_Ults::round(_pickup_waiting_time / _unit_time); // intervals
                /* hook up */
                _origin->m_origin_node = (MNM_DMOND *)node_factory->get_node (_node_ID);
                ((MNM_DMOND *)node_factory->get_node (_node_ID)) ->hook_up_origin (_origin);
            }
        }
        std::getline (_od_file, _line); // skip another line
        // printf("Processing Destination node.\n");
        for (int i = 0; i < _num_of_D; ++i)
        {
            std::getline (_od_file, _line);
            _words = split (_line, ' ');
            if (_words.size () == 2)
            {
                // std::cout << "Processing: " << _line << "\n";
                _dest_ID = TInt (std::stoi (_words[0]));
                _node_ID = TInt (std::stoi (_words[1]));
                _dest = od_factory->make_destination (_dest_ID);
                _dest->m_flow_scalar = _flow_scalar;
                /* hook up */
                _dest->m_dest_node= (MNM_DMDND *)node_factory->get_node (_node_ID);
                ((MNM_DMDND *)node_factory->get_node (_node_ID)) ->hook_up_destination (_dest);
            }
            else {
                throw std::runtime_error("Something is wrong in destination part in od input file");
            }
        }
    }
    _od_file.close ();
    return 0;
}

int 
MNM_IO_EV::add_charging_station_node(const std::string& file_folder, MNM_ConfReader *conf_reader, 
                                    MNM_Node_Factory *node_factory, const std::string& file_name)
{
    auto _node_factory_ev = dynamic_cast<MNM_Node_Factory_EV*>(node_factory);
    IAssert(_node_factory_ev != nullptr);
    /* find file */
    std::string _file_name = file_folder + "/" + file_name;
    std::ifstream _file;
    _file.open(_file_name, std::ios::in);

    if (_file.is_open ())
    {
        TInt _num_of_charging_station = conf_reader->get_int ("num_of_charging_station");
        if (_num_of_charging_station <= 0)
        {
            _file.close ();
            printf ("No charging station.\n");
            return 0;
        }
        TInt _unit_time = conf_reader->get_int ("unit_time");
        TFlt _flow_scalar = conf_reader -> get_float("flow_scalar");
        int _num_slots, _charging_time, _avg_waiting_time;
        float _price;

        std::string _line;
        std::vector<std::string> _words;
        TInt _ID;
        printf ("Start build charging station.\n");
        std::getline (_file, _line); // #charging_station_ID num_slots avg_charging_time avg_waiting_time price
        for (int i = 0; i < _num_of_charging_station; ++i)
        {
            std::getline (_file, _line);
            // std::cout << "Processing: " << _line << "\n";
            _words = split (_line, ' ');
            if (TInt (_words.size ()) == 5)
            {
                _ID = TInt (std::stoi (trim (_words[0])));
                _num_slots = std::stoi (trim (_words[1]));
                _charging_time = int(std::stof (trim (_words[2])) / _unit_time);  // seconds -> intervals
                _avg_waiting_time = int(std::stof (trim (_words[3])) / _unit_time);  // seconds -> intervals
                _price = std::stof (trim (_words[4]));  // US dollar
                // add charging station to node factory
                _node_factory_ev->make_charging_station(_ID, _flow_scalar, _unit_time, _num_slots, _charging_time, _avg_waiting_time, _price);
            }
            else
            {
                throw std::runtime_error ("Something wrong in MNM_IO_With_EV::add_charging_station_node");
            }
        }
        _file.close ();
        printf ("Finish build charging station.\n");
    }
    else
    {
        printf ("No charging station.\n");
    }
    return 0;
}
  
OD_Candidate_POI_Table* 
MNM_IO_EV::load_candidate_poi_table(const std::string& file_folder, MNM_ConfReader *conf_reader, 
                                    MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory, const std::string& file_name)
{   
    auto _node_factory_with_ev = dynamic_cast<MNM_Node_Factory_EV*>(node_factory);
    IAssert(_node_factory_with_ev != nullptr);

    // destruct in MNM_Routing_Adaptive_With_EV
    OD_Candidate_POI_Table *table = new OD_Candidate_POI_Table();

    /* find file */
    std::string _file_name = file_folder + "/" + file_name;
    std::ifstream _file;
    _file.open(_file_name, std::ios::in);

    if (_file.is_open ()) {
        std::string _line;
        std::vector<std::string> _words;
        TInt _O_ID, _D_ID, _charging_station_ID;
        MNM_Origin *_origin;
        MNM_Destination *_dest;
        MNM_Charging_Station *_charging_station;
        
        printf ("Start build candidate charging station for OD pair.\n");
        std::getline (_file, _line); // <O_ID, D_ID>: cs1_ID, cs2_ID, ...
        while (std::getline(_file, _line)) {
            _words = split (_line, ' ');
            if (_words.size() >= 2) {
                _O_ID = TInt(std::stoi(trim(_words[0])));
                _D_ID = TInt(std::stoi(trim(_words[1])));
                _origin = od_factory -> get_origin(_O_ID);
                _dest = od_factory -> get_destination(_D_ID);
                
                if (table -> find(_origin) == table -> end()) {
                    table -> insert(std::make_pair(_origin, new std::unordered_map<MNM_Destination*, std::vector<MNM_Dnode*>*>()));
                }
                if (table -> find(_origin) -> second -> find(_dest) == table -> find(_origin) -> second -> end()) {
                    table -> find(_origin) -> second -> insert(std::make_pair(_dest, new std::vector<MNM_Dnode*>()));
                }
                if (_words.size() == 2) continue;
                auto _candidate_poi_vec = table -> find(_origin) -> second -> find(_dest) -> second;
                for (size_t i = 2; i < _words.size(); ++i) {
                    _charging_station_ID = TInt(std::stoi(trim(_words[i])));
                    _charging_station = dynamic_cast<MNM_Charging_Station*>(_node_factory_with_ev -> get_node(_charging_station_ID));
                    IAssert(_charging_station != nullptr);
                    auto _it = std::find(_candidate_poi_vec -> begin(), _candidate_poi_vec -> end(), _charging_station);
                    if (_it == _candidate_poi_vec -> end()) {
                        _candidate_poi_vec -> push_back(_charging_station);
                    }
                    // else {
                    //     printf("Repeated POI\n");
                    // }
                }
            }
            else {
                throw std::runtime_error ("Something wrong in MNM_IO_With_EV::build_candidate_charging_station");
            }
        }
        printf ("Finish build candidate charging station for OD pair.\n");
    }
    else {
        printf("No candidate charging stations for OD pair.\n");
    }
    return table;
}

int 
MNM_IO_EV::save_candidate_poi_table(const OD_Candidate_POI_Table& candidate_poi_table,
                                    const std::string& file_name)
{
    std::ofstream _file;
    _file.open(file_name, std::ofstream::out);
    if (! _file.is_open()){
        std::cout << "MNM_Charging_Station::save_candidate_poi_table, Error happens when open " << file_name << std::endl;
        throw std::runtime_error("Error");
    }
    _file << "# (O_ID, D_ID): cs1, cs2, ...\n";
    std::string _str;
    for (auto _it : candidate_poi_table) {
        for (auto _it_it : *_it.second) {
            _str = std::to_string(_it.first -> m_Origin_ID) + " " + std::to_string(_it_it.first -> m_Dest_ID) + " ";
            for (auto _it_it_it : *_it_it.second) {
                _str += std::to_string(_it_it_it-> m_node_ID) + " ";
            }
            _str.pop_back();
            _str += "\n";
            _file << _str;
        }
    }
    if (_file.is_open()) _file.close();
    return 0;
}

int 
MNM_IO_EV::save_charging_station_record(const std::string& file_folder, MNM_Node_Factory *node_factory)
{
    for (auto _node_it : node_factory -> m_node_map) {
        if (auto _charging_station = dynamic_cast<MNM_Charging_Station*>(_node_it.second)) {
            _charging_station -> save_cc(file_folder + "/charging_station_" + std::to_string(_charging_station -> m_node_ID()) + "_cc.txt");
            _charging_station -> save_waiting_time_record(file_folder + "/charging_station_" + std::to_string(_charging_station -> m_node_ID()) + "_wt.txt");
        }
    }
    return 0;
}

int 
MNM_IO_EV::destruct_candidate_poi_table(OD_Candidate_POI_Table *table)
{
    for (auto _it : *table) {
        for (auto _it_it : *_it.second) {
            _it_it.second -> clear();
            delete _it_it.second;
        }
        _it.second -> clear();
        delete _it.second;
    }
    table -> clear();
    delete table;
    return 0;
}


//#################################################################
//           DTA for Electrified and Delivery Traffic
//#################################################################

MNM_Dta_EV::MNM_Dta_EV(const std::string& file_folder)
    : MNM_Dta::MNM_Dta(file_folder)
{
    initialize();
}

MNM_Dta_EV::~MNM_Dta_EV()
{
    ;
}

int 
MNM_Dta_EV::initialize()
{
    if (m_routing != nullptr) {
        delete m_routing;
        m_routing = nullptr;
    }
    if (m_veh_factory != nullptr) {
        delete m_veh_factory;
        m_veh_factory = nullptr;
    }
    if (m_node_factory != nullptr) {
        delete m_node_factory;
        m_node_factory = nullptr;
    }
    if (m_link_factory != nullptr) {
        delete m_link_factory;
        m_link_factory = nullptr;
    }
    if (m_od_factory != nullptr) {
        delete m_od_factory;
        m_od_factory = nullptr;
    }
    m_veh_factory = new MNM_Veh_Factory_EV();
    m_node_factory = new MNM_Node_Factory_EV();
    m_link_factory = new MNM_Link_Factory_Delivery();
    m_od_factory = new MNM_OD_Factory_EV();
    return 0;
}

int 
MNM_Dta_EV::build_from_files()
{
    MNM_IO::build_node_factory(m_file_folder, m_config, m_node_factory);
    MNM_IO_EV::add_charging_station_node(m_file_folder, m_config, m_node_factory);
    std::cout << "# of nodes: " << m_node_factory -> m_node_map.size() << "\n";  // including normal nodes + charging stations
    MNM_IO::build_link_factory(m_file_folder, m_config, m_link_factory);
    std::cout << "# of links: " << m_link_factory -> m_link_map.size() << "\n";
    MNM_IO_EV::build_od_factory_ev(m_file_folder, m_config, m_od_factory, m_node_factory);
    std::cout << "# of OD pairs: " << m_od_factory -> m_origin_map.size() << "\n";
    
    m_graph = MNM_IO::build_graph(m_file_folder, m_config);

    MNM_IO::build_demand(m_file_folder, m_config, m_od_factory);
    MNM_IO_Delivery::build_demand_multi_OD_seq(m_file_folder, m_config, m_od_factory);
    MNM_IO::read_origin_vehicle_label_ratio(m_file_folder, m_config, m_od_factory);
    MNM_IO::build_link_toll(m_file_folder, m_config, m_link_factory);
    build_workzone();
    set_statistics();
    set_gridlock_recorder();
    printf("Start building routing\n");
    set_routing();
    printf("Finish building routing\n");
    return 0;
}

int 
MNM_Dta_EV::set_routing()
{
    if (m_config -> get_string("routing_type") == "Hybrid"){
        MNM_ConfReader* _tmp_conf = new MNM_ConfReader(m_file_folder + "/config.conf", "FIXED");
        Path_Table *_path_table = MNM_IO::load_path_table(m_file_folder + "/" + _tmp_conf -> get_string("path_file_name"), 
                                                          m_graph, _tmp_conf -> get_int("num_path"), 
                                                          _tmp_conf -> get_string("choice_portion") == "Buffer");

        TInt _route_freq_fixed = _tmp_conf -> get_int("route_frq");
        TInt _buffer_len = _tmp_conf -> get_int("buffer_length");
        IAssert(_buffer_len == m_config -> get_int("max_interval"));

        OD_Candidate_POI_Table *_od_candidate_poi_table = MNM_IO_EV::load_candidate_poi_table(m_file_folder, m_config, m_od_factory, m_node_factory);

        m_routing = new MNM_Routing_Hybrid_EV(m_file_folder, m_graph, m_statistics, m_od_factory, m_node_factory, 
                                                   m_link_factory, _od_candidate_poi_table, _route_freq_fixed, _buffer_len);
        m_routing -> init_routing(_path_table);
        delete _tmp_conf;
    }
    else {
        throw std::runtime_error("MNM_Dta_EV::set_routing, routing_type must be Hybrid");
    }
    return 0;
}

bool 
MNM_Dta_EV::is_ok()
{
    bool _flag = true;
    bool _temp_flag = true;
    //Checks the graph data structure for internal consistency.
    //For each node in the graph check that its neighbors are also nodes in the graph.
    printf("\nChecking......Driving Graph consistent!\n");
    _temp_flag = m_graph -> IsOk(); 
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");

    //check node
    printf("Checking......Driving Node consistent!\n");
    _temp_flag = (m_graph -> GetNodes() == m_config -> get_int("num_of_node") + m_config -> get_int("num_of_charging_station"))
                    && (m_graph -> GetNodes() == TInt(m_node_factory -> m_node_map.size()));
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");

    //check link
    printf("Checking......Driving Link consistent!\n");
    _temp_flag = (m_graph -> GetEdges() == m_config -> get_int("num_of_link"))
                    && (m_graph -> GetEdges() == TInt(m_link_factory -> m_link_map.size()));
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");  

    //check OD node
    printf("Checking......OD consistent!\n");
    TInt _node_ID;
    _temp_flag = (TInt(m_od_factory -> m_origin_map.size()) == m_config -> get_int("num_of_O"))
                    && (TInt(m_od_factory -> m_destination_map.size()) == m_config -> get_int("num_of_D"));
    std::unordered_map<TInt, MNM_Origin*>::iterator _origin_map_it;
    for (_origin_map_it = m_od_factory->m_origin_map.begin();
        _origin_map_it != m_od_factory->m_origin_map.end(); _origin_map_it++){
        _node_ID = _origin_map_it -> second -> m_origin_node -> m_node_ID;
        _temp_flag = _temp_flag && ((m_graph -> GetNI(_node_ID)).GetId() == _node_ID)
                    && (m_graph -> GetNI(_node_ID).GetOutDeg() >= 1)
                    && (m_graph -> GetNI(_node_ID).GetInDeg() == 0);
    }
    std::unordered_map<TInt, MNM_Destination*>::iterator _dest_map_it;
    for (_dest_map_it = m_od_factory->m_destination_map.begin();
        _dest_map_it != m_od_factory->m_destination_map.end(); _dest_map_it++){
        _node_ID = _dest_map_it -> second -> m_dest_node -> m_node_ID;
        _temp_flag = _temp_flag && ((m_graph -> GetNI(_node_ID)).GetId() == _node_ID)
                    && (m_graph -> GetNI(_node_ID).GetOutDeg() == 0)
                    && (m_graph -> GetNI(_node_ID).GetInDeg() >= 1);
    }  
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");  

    printf("Checking......OD connectivity!\n");
    _temp_flag = check_origin_destination_connectivity();
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");  

    // check POI
    printf("Checking......OD Candidate POIs connectivity!\n");
    if (auto _routing = dynamic_cast<MNM_Routing_Hybrid_EV*>(m_routing)) {
        IAssert(_routing -> m_routing_adaptive != nullptr);
        IAssert(_routing -> m_routing_adaptive -> m_od_candidate_poi_table != nullptr && !_routing -> m_routing_adaptive -> m_od_candidate_poi_table -> empty());
        _temp_flag = _routing -> m_routing_adaptive -> check_od_candidate_poi_table_connectivity();
    }
    else {
        printf("Must use MNM_Routing_Hybrid_With_EV for m_routing\n");
        exit(-1);
    }
    _flag = _flag && _temp_flag;
    if (_temp_flag)  printf("Passed!\n");  

    return _flag;
}

int 
MNM_Dta_EV::load_once(bool verbose, TInt load_int, TInt assign_int)
{
    MNM_Origin *_origin;
    MNM_Dnode *_node;
    MNM_Dlink *_link;
    MNM_Destination *_dest;
    if (load_int==0) m_statistics -> update_record(load_int);
    if (verbose) printf("-------------------------------    Interval %d   ------------------------------ \n", (int)load_int);
    // step 1: Origin release vehicle
    if (verbose) printf("Releasing!\n");
    TFlt _ad_ratio = m_config -> get_float("adaptive_ratio");
    if (_ad_ratio > 1.) _ad_ratio = 1.;
    if (_ad_ratio < 0.) _ad_ratio = 0.;
    bool _releasing = (load_int % m_assign_freq == 0 || load_int==0);
    for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it != m_od_factory -> m_origin_map.end(); _origin_it++){
        _origin = _origin_it -> second;
        dynamic_cast<MNM_Origin_EV*>(_origin) -> adjust_multi_OD_seq_veh_routing_type(_ad_ratio);
        if (_releasing) {
            if (assign_int >= m_total_assign_inter) {
                _origin -> release_one_interval(load_int, m_veh_factory, -1, TFlt(-1));
            }
            else{
                if((m_config -> get_string("routing_type") == "Hybrid")){
                    _origin -> release_one_interval(load_int, m_veh_factory, assign_int, _ad_ratio);
                }
                else{
                    printf("MNM_Dta_With_EV::load_once, routing type must be Hybrid\n");
                }
            }
        }
    } 
    

    if (verbose) printf("Routing!\n");
    // step 2: route the vehicle
    m_routing -> update_routing(load_int);

    // for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
    //     _node = _node_it -> second;
    //     if (auto _charging_station = dynamic_cast<MNM_Charging_Station*>(_node)) {
    //         _charging_station -> check_veh_status();
    //     }
    // }

    if (verbose) printf("Moving through node!\n");
    // step 3: move vehicles through node
    for (auto _node_it = m_node_factory -> m_node_map.begin(); _node_it != m_node_factory -> m_node_map.end(); _node_it++){
        _node = _node_it -> second;
        // printf("node ID is %d\n", _node -> m_node_ID());
        _node -> evolve(load_int);
    }

    // record queuing vehicles after node evolve, which is num of vehicles in finished array
    record_queue_vehicles();
    if (verbose) printf("Moving through link\n");
    // step 4: move vehicles through link
    for (auto _link_it = m_link_factory -> m_link_map.begin(); _link_it != m_link_factory -> m_link_map.end(); _link_it++){
        _link = _link_it -> second;
        // if (_link -> get_link_flow() > 0){
        //   printf("Current Link %d:, traffic flow %.4f, incomming %d, finished %d\n", 
        //       _link -> m_link_ID(), _link -> get_link_flow()(), (int)_link -> m_incoming_array.size(),  (int)_link -> m_finished_array.size());
        //   _link -> print_info();
        // }
        if ((m_gridlock_recorder != nullptr) && (
            (m_config -> get_int("total_interval") <= 0 && load_int >= 1.5 * m_total_assign_inter * m_assign_freq) || 
            (m_config -> get_int("total_interval") > 0 && load_int >= 0.95 * m_config ->get_int("total_interval")))) {
            m_gridlock_recorder -> save_one_link(load_int, _link);
        }
        _link -> clear_incoming_array(load_int);
        _link -> evolve(load_int);
    }

    if (verbose) printf("Receiving!\n");
    // step 5: Destination receive vehicle  
    for (auto _dest_it = m_od_factory -> m_destination_map.begin(); _dest_it != m_od_factory -> m_destination_map.end(); _dest_it++){
        _dest = _dest_it -> second;
        // _dest -> receive(load_int);
        _dest -> receive(load_int, m_routing, m_veh_factory, true);
    }

    if (verbose) printf("Update record!\n");
    // step 5: update record
    m_statistics -> update_record(load_int);

    record_enroute_vehicles();
    if (verbose) MNM::print_vehicle_statistics(m_veh_factory);
    // test();  
    // consistent with loading()
    m_current_loading_interval = load_int + 1;
    return 0;
}