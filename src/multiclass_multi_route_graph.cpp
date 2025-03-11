#include "multiclass_multi_route_graph.h"
#include <cfloat>


/**************************************************************************
                            Vehicle
**************************************************************************/
MNM_Veh_Multiclass_Subclass::MNM_Veh_Multiclass_Subclass (TInt ID, TInt vehicle_class, TInt vehicle_subclass, TInt start_time)
    : MNM_Veh_Multiclass::MNM_Veh_Multiclass (ID, vehicle_class, start_time)
{
    m_subclass = vehicle_subclass;
}

MNM_Veh_Multiclass_Subclass::~MNM_Veh_Multiclass_Subclass ()
{
    ;
}

/**************************************************************************
                            Vehicle Factory
**************************************************************************/
MNM_Veh_Factory_Multiclass_Subclass::MNM_Veh_Factory_Multiclass_Subclass ()
    : MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass ()
{
    m_num_car_subclass = std::unordered_map<int, int> ();
    m_num_truck_subclass = std::unordered_map<int, int> ();
    m_enroute_car_subclass = std::unordered_map<int, int> ();
    m_enroute_truck_subclass = std::unordered_map<int, int> ();
    m_finished_car_subclass = std::unordered_map<int, int> ();
    m_finished_truck_subclass = std::unordered_map<int, int> ();
    m_total_time_car_subclass = std::unordered_map<int, TFlt> ();
    m_total_time_truck_subclass = std::unordered_map<int, TFlt> ();
}

MNM_Veh_Factory_Multiclass_Subclass::~MNM_Veh_Factory_Multiclass_Subclass ()
{
    m_num_car_subclass.clear ();
    m_num_truck_subclass.clear ();
    m_enroute_car_subclass.clear ();
    m_enroute_truck_subclass.clear ();
    m_finished_car_subclass.clear ();
    m_finished_truck_subclass.clear ();
    m_total_time_car_subclass.clear ();
    m_total_time_truck_subclass.clear ();
}

MNM_Veh_Multiclass_Subclass * 
MNM_Veh_Factory_Multiclass_Subclass::make_veh_multiclass_subclass (TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls, TInt vehicle_subcls)
{
    // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp,
    // (int)m_num_veh + 1);
    MNM_Veh_Multiclass_Subclass *_veh = new MNM_Veh_Multiclass_Subclass (m_num_veh + 1, vehicle_cls, vehicle_subcls, timestamp);
    _veh->m_type = veh_type;
    m_veh_map.insert ({ m_num_veh + 1, _veh });

    m_num_veh += 1;
    m_enroute += 1;
    if (vehicle_cls == 0)
    {
        m_num_car += 1;
        m_enroute_car += 1;
        if (m_num_car_subclass.find(vehicle_subcls) == m_num_car_subclass.end())
        {
            m_num_car_subclass[vehicle_subcls] = 1;
        }
        else
        {
            m_num_car_subclass[vehicle_subcls] += 1;
        }
        if (m_enroute_car_subclass.find(vehicle_subcls) == m_enroute_car_subclass.end())
        {
            m_enroute_car_subclass[vehicle_subcls] = 1;
        }
        else
        {
            m_enroute_car_subclass[vehicle_subcls] += 1;
        }
    }
    else if (vehicle_cls == 1)
    {
        m_num_truck += 1;
        m_enroute_truck += 1;
        if (m_num_truck_subclass.find(vehicle_subcls) == m_num_truck_subclass.end())
        {
            m_num_truck_subclass[vehicle_subcls] = 1;
        }
        else
        {
            m_num_truck_subclass[vehicle_subcls] += 1;
        }
        if (m_enroute_truck_subclass.find(vehicle_subcls) == m_enroute_truck_subclass.end())
        {
            m_enroute_truck_subclass[vehicle_subcls] = 1;
        }
        else
        {
            m_enroute_truck_subclass[vehicle_subcls] += 1;
        }
    }
    return _veh;
}

int
MNM_Veh_Factory_Multiclass_Subclass::remove_finished_veh (MNM_Veh *veh, bool del)
{
    MNM_Veh_Multiclass_Subclass *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass_Subclass *> (veh);
    IAssert (_veh_multiclass != nullptr);
    IAssert (veh->m_finish_time > veh->m_start_time);
    if (_veh_multiclass->m_class == 0)
    {
        m_finished_car += 1;
        m_enroute_car -= 1;
        m_total_time_car += (veh->m_finish_time - veh->m_start_time);
        if (m_finished_car_subclass.find(_veh_multiclass->m_subclass) == m_finished_car_subclass.end())
        {
            m_finished_car_subclass[_veh_multiclass->m_subclass] = 1;
        }
        else
        {
            m_finished_car_subclass[_veh_multiclass->m_subclass] += 1;    
        }    
        m_enroute_car_subclass[_veh_multiclass->m_subclass] -= 1;
        if (m_total_time_car_subclass.find(_veh_multiclass->m_subclass) == m_total_time_car_subclass.end())
        {
            m_total_time_car_subclass[_veh_multiclass->m_subclass] = (veh->m_finish_time - veh->m_start_time);
        }
        else
        {
            m_total_time_car_subclass[_veh_multiclass->m_subclass] += (veh->m_finish_time - veh->m_start_time); 
        } 
    }
    else if (_veh_multiclass->m_class == 1)
    {
        m_finished_truck += 1;
        m_enroute_truck -= 1;
        m_total_time_truck += (veh->m_finish_time - veh->m_start_time);
        if (m_finished_truck_subclass.find(_veh_multiclass->m_subclass) == m_finished_truck_subclass.end())
        {
            m_finished_truck_subclass[_veh_multiclass->m_subclass] = 1;
        }
        else
        {
            m_finished_truck_subclass[_veh_multiclass->m_subclass] += 1;
        }
        m_enroute_truck_subclass[_veh_multiclass->m_subclass] -= 1;
        if (m_total_time_truck_subclass.find(_veh_multiclass->m_subclass) == m_total_time_truck_subclass.end())
        {
            m_total_time_truck_subclass[_veh_multiclass->m_subclass] = (veh->m_finish_time - veh->m_start_time);
        }
        else
        {
            m_total_time_truck_subclass[_veh_multiclass->m_subclass] += (veh->m_finish_time - veh->m_start_time); 
        } 
    }
    MNM_Veh_Factory::remove_finished_veh (veh, del);
    IAssert (m_num_car == m_finished_car + m_enroute_car);
    IAssert (m_num_truck == m_finished_truck + m_enroute_truck);
    return 0;
}

/**************************************************************************
                            Origin
**************************************************************************/
MNM_Origin_Multiclass_Subclass::MNM_Origin_Multiclass_Subclass (TInt ID, TInt max_interval,
                                              TFlt flow_scalar, TInt frequency)
    : MNM_Origin_Multiclass::MNM_Origin_Multiclass (ID, max_interval, flow_scalar, frequency)
{
    m_demand_car_subclass = std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>> ();
    m_demand_truck_subclass = std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>> ();
    m_adaptive_ratio_car = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
    m_adaptive_ratio_truck = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
    m_car_label_ratio = std::vector<TFlt> ();
    m_truck_label_ratio = std::vector<TFlt> ();
}

MNM_Origin_Multiclass_Subclass::~MNM_Origin_Multiclass_Subclass ()
{
    for (auto _it : m_demand_car_subclass)
    {
        for (auto _demand_it : _it.second)
        {
            delete[] _demand_it.second;
        }
        _it.second.clear ();
    }
    m_demand_car_subclass.clear ();

    for (auto _it : m_demand_truck_subclass)
    {
        for (auto _demand_it : _it.second)
        {
            delete[] _demand_it.second;
        }
        _it.second.clear ();
    }
    m_demand_truck_subclass.clear ();
}

int 
MNM_Origin_Multiclass_Subclass::add_dest_demand_multiclass_subclass (MNM_Destination_Multiclass *dest, int mainclass_label, int subclass_label, 
                                                                     TFlt *demand)
{
    // split (15-mins demand) to (15 * 1-minute demand)
    double *_demand = new double[m_max_assign_interval * 15]();
    for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
        _demand[i] = TFlt (demand[i]);
    }

    if (mainclass_label == 0) {
        if (m_demand_car_subclass.find (dest) == m_demand_car_subclass.end()) {
            m_demand_car_subclass.insert ({dest, std::unordered_map<int, TFlt*> ()});
        }
        if (m_demand_car_subclass.find (dest) -> second.find (subclass_label) != m_demand_car_subclass.find (dest) -> second.end()) {
            std::cerr << "Error: add_dest_demand_multiclass_subclass: destination " << dest->m_Dest_ID << " already has demand for mainclass " << mainclass_label << " and subclass " << subclass_label << std::endl;
            return -1;
        }
        m_demand_car_subclass[dest].insert (std::make_pair(subclass_label, _demand));
    } 
    else if (mainclass_label == 1) {
        if (m_demand_truck_subclass.find (dest) == m_demand_truck_subclass.end()) {
            m_demand_truck_subclass.insert ({dest, std::unordered_map<int, TFlt*> ()});
        }
        if (m_demand_truck_subclass.find (dest) -> second.find (subclass_label) != m_demand_truck_subclass.find (dest) -> second.end()) {
            std::cerr << "Error: add_dest_demand_multiclass_subclass: destination " << dest->m_Dest_ID << " already has demand for mainclass " << mainclass_label << " and subclass " << subclass_label << std::endl;
            return -1;
        }
        m_demand_truck_subclass[dest].insert (std::make_pair(subclass_label, _demand));
    } 
    else {
        std::cout << "Error: mainclass_label is not 0 or 1" << std::endl;
        throw std::runtime_error("Invalid mainclass_label");
    }
    return 0;
}

int 
MNM_Origin_Multiclass_Subclass::get_dest_demand_multiclass ()
{ 
    double *_demand;
    for (auto _it : m_demand_car_subclass)
    {   
        _demand = new double[m_max_assign_interval * 15]();
        for (auto _demand_it : _it.second)
        { 
            for (int i = 0; i < m_max_assign_interval * 15; ++i)
            {
                _demand[i] += TFlt (_demand_it.second[i]);
            }
        }
        m_demand_car.insert ({ _it.first, _demand});
    }
    for (auto _it : m_demand_truck_subclass)
    {   
        _demand = new double[m_max_assign_interval * 15]();
        for (auto _demand_it : _it.second)
        { 
            for (int i = 0; i < m_max_assign_interval * 15; ++i)
            {
                _demand[i] += TFlt (_demand_it.second[i]);
            }
        }
        m_demand_truck.insert ({ _it.first, _demand});
    }
    return 0;
}

int 
MNM_Origin_Multiclass_Subclass::release_one_interval_biclass (TInt current_interval,
                                            MNM_Veh_Factory *veh_factory,
                                            TInt assign_interval,
                                            TFlt adaptive_ratio_car,
                                            TFlt adaptive_ratio_truck) 
{
    if (assign_interval < 0) return 0;
    m_current_assign_interval = assign_interval;
    TInt _veh_to_release;
    MNM_Veh_Multiclass_Subclass *_veh;
    MNM_Veh_Factory_Multiclass_Subclass *_vfactory = dynamic_cast<MNM_Veh_Factory_Multiclass_Subclass *> (veh_factory);

    // release all car
    for (auto _it: m_demand_car_subclass) 
    {
        // dest: _it.first;
        // override adaptive ratio with time-dependent and OD-dependent one in input file
        if (m_adaptive_ratio_car.find(_it.first) != m_adaptive_ratio_car.end()) {
            adaptive_ratio_car = m_adaptive_ratio_car.find(_it.first) -> second[assign_interval];
        }
        for (auto _demand_it : _it.second)
        { 
            // subclass: _demand_it.first;
            _veh_to_release = TInt (MNM_Ults::round ((_demand_it.second)[assign_interval] * m_flow_scalar));
            for (int i = 0; i < _veh_to_release; ++i)
            {
                if (adaptive_ratio_car == TFlt (0))
                {
                    _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                    MNM_TYPE_STATIC, 
                                                                    TInt (0), _demand_it.first);
                }
                else if (adaptive_ratio_car == TFlt (1))
                {
                    _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                    MNM_TYPE_ADAPTIVE, 
                                                                    TInt (0), _demand_it.first);
                }
                else
                {
                    TFlt _r = MNM_Ults::rand_flt ();
                    if (_r <= adaptive_ratio_car)
                    {
                        _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                        MNM_TYPE_ADAPTIVE,
                                                                        TInt (0), _demand_it.first);
                    }
                    else
                    {
                        _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                        MNM_TYPE_STATIC,
                                                                        TInt (0), _demand_it.first);
                    }
                }
                _veh->set_destination (_it.first);
                _veh->set_origin (this);
                // _veh -> m_assign_interval = assign_interval;
                // in case the multiclass modeling has 1-min release interval as the
                // "assign" interval
                _veh->m_assign_interval = int (current_interval / m_frequency);
                _veh->m_label = generate_label (_veh->get_class ());
                m_origin_node->m_in_veh_queue.push_back (_veh);
            }
        }
    }

    // release all truck
    for (auto _it: m_demand_truck_subclass) 
    {
        // dest: _it.first;
        // override adaptive ratio with time-dependent and OD-dependent one in input file
        if (m_adaptive_ratio_truck.find(_it.first) != m_adaptive_ratio_truck.end()) {
            adaptive_ratio_truck = m_adaptive_ratio_truck.find(_it.first) -> second[assign_interval];
        }
        for (auto _demand_it : _it.second)
        { 
            // subclass: _demand_it.first;
            _veh_to_release = TInt (MNM_Ults::round ((_demand_it.second)[assign_interval] * m_flow_scalar));
            for (int i = 0; i < _veh_to_release; ++i)
            {
                if (adaptive_ratio_truck == TFlt (0))
                {
                    _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                    MNM_TYPE_STATIC, 
                                                                    TInt (1), _demand_it.first);
                }
                else if (adaptive_ratio_car == TFlt (1))
                {
                    _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                    MNM_TYPE_ADAPTIVE, 
                                                                    TInt (1), _demand_it.first);
                }
                else
                {
                    TFlt _r = MNM_Ults::rand_flt ();
                    if (_r <= adaptive_ratio_car)
                    {
                        _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                        MNM_TYPE_ADAPTIVE,
                                                                        TInt (1), _demand_it.first);
                    }
                    else
                    {
                        _veh = _vfactory->make_veh_multiclass_subclass (current_interval,
                                                                        MNM_TYPE_STATIC,
                                                                        TInt (1), _demand_it.first);
                    }
                }
                _veh->set_destination (_it.first);
                _veh->set_origin (this);
                // _veh -> m_assign_interval = assign_interval;
                // in case the multiclass modeling has 1-min release interval as the
                // "assign" interval
                _veh->m_assign_interval = int (current_interval / m_frequency);
                _veh->m_label = generate_label (_veh->get_class ());
                m_origin_node->m_in_veh_queue.push_back (_veh);
            }
        }
    }
    std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                        m_origin_node->m_in_veh_queue.end ());
    return 0;
}

MNM_OD_Factory_Multiclass_Subclass::MNM_OD_Factory_Multiclass_Subclass()
    : MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass()
{
    ;
}

MNM_OD_Factory_Multiclass_Subclass::~MNM_OD_Factory_Multiclass_Subclass()
{
    ;
}

MNM_Origin_Multiclass_Subclass *
MNM_OD_Factory_Multiclass_Subclass::make_origin (TInt ID, TInt max_interval,
                                        TFlt flow_scalar, TInt frequency)
{
  MNM_Origin_Multiclass_Subclass *_origin = new MNM_Origin_Multiclass_Subclass (ID, max_interval, flow_scalar, frequency);
  m_origin_map.insert ({ ID, _origin });
  return _origin;
}

/**************************************************************************
                            Routing
**************************************************************************/

/**************** Fixed Routing ********************/
MNM_Routing_Biclass_Fixed_Subclass::MNM_Routing_Biclass_Fixed_Subclass (
    macposts::Graph &graph, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    TInt route_frq_fixed, TInt buffer_length, TInt buffer_start_index,
    TInt veh_class, TInt veh_subclass)
    : MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed(graph, od_factory, node_factory, link_factory, route_frq_fixed, buffer_length, veh_class)
{
    m_veh_subclass = veh_subclass;
    m_buffer_start_index = buffer_start_index;
    // assume buffer_length = max_interval, different from multiclass
}

MNM_Routing_Biclass_Fixed_Subclass::~MNM_Routing_Biclass_Fixed_Subclass()
{
    ;
}

int
MNM_Routing_Biclass_Fixed_Subclass::change_choice_portion(TInt routing_interval)
{
    // m_veh_class starts from 0 (car) to 1 (truck)
    // m_veh_subclass starts from 0 (small) to 1 (large)
  MNM::copy_buffer_to_p (m_path_table,
                         routing_interval + m_buffer_start_index);
  MNM::normalize_path_table_p (m_path_table);
  return 0;
}

int 
MNM_Routing_Biclass_Fixed_Subclass::update_routing (TInt timestamp)
{
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh *_veh;
  TInt _cur_ass_int;

  // printf("my buffer length %d, %d\n", m_buffer_length(), m_routing_freq());
  if (m_buffer_as_p)
    {
      _cur_ass_int = TInt (timestamp / m_routing_freq);
      // assume m_buffer_length = max_interval, different from multiclass
      if (_cur_ass_int < TInt (m_buffer_length))
        { // first half for car, last half for truck
          change_choice_portion (_cur_ass_int);
        }
    }

  for (auto _origin_it = m_od_factory->m_origin_map.begin ();
       _origin_it != m_od_factory->m_origin_map.end (); _origin_it++)
    {
      _origin = _origin_it->second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh_it = _origin_node->m_in_veh_queue.begin ();
           _veh_it != _origin_node->m_in_veh_queue.end (); _veh_it++)
        {
          _veh = *_veh_it;

          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_subclass () == m_veh_subclass)
              && (_veh->get_bus_route_ID () == TInt (-1))
              && (!_veh->get_ispnr ()))
            {
              // Here is the difference from single-class fixed routing

              if (m_tracker.find (_veh) == m_tracker.end ())
                {
                  // printf("Registering!\n");
                  register_veh (_veh, true);
                  _next_link_ID = m_tracker.find (_veh)->second->front ();
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  _veh->set_next_link (_next_link);
                  m_tracker.find (_veh)->second->pop_front ();
                }
            }
          // according to Dr. Wei Ma, add a nominal path to adaptive users for
          // DAR extraction, not rigorous, but will do the DODE job
          else if ((_veh->m_type == MNM_TYPE_ADAPTIVE)
                   && (_veh->get_class () == m_veh_class)
                   && (_veh->get_subclass () == m_veh_subclass)
                   && (_veh->get_bus_route_ID () == TInt (-1))
                   && (!_veh->get_ispnr ()))
            {
              if (_veh->m_path == nullptr)
                {
                  register_veh (_veh, false);
                }
              IAssert (_veh->m_path != nullptr);
            }
        }
    }
  // printf("Finished route OD veh\n");

  MNM_Destination *_veh_dest;
  MNM_Dlink *_link;
  for (auto _link_it = m_link_factory->m_link_map.begin ();
       _link_it != m_link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      _node_ID = _link->m_to_node->m_node_ID;
      for (auto _veh_it = _link->m_finished_array.begin ();
           _veh_it != _link->m_finished_array.end (); _veh_it++)
        {
          _veh = *_veh_it;

          // Here is the difference from single-class fixed routing
          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_subclass () == m_veh_subclass)
              && (_veh->get_bus_route_ID () == TInt (-1))
              && (!_veh->get_ispnr ()))
            {
              // Here is the difference from single-class fixed routing

              _veh_dest = _veh->get_destination ();
              if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  if (m_tracker.find (_veh)->second->size () != 0)
                    {
                      throw std::runtime_error ("invalid state");
                    }
                  _veh->set_next_link (nullptr);
                }
              else
                {
                  if (m_tracker.find (_veh) == m_tracker.end ())
                    {
                      throw std::runtime_error (
                        "invalid state: vehicle unregistered for link");
                    }
                  if (_veh->get_current_link () == _veh->get_next_link ())
                    {
                      _next_link_ID = m_tracker.find (_veh)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          throw std::runtime_error ("invalid state");
                        }
                      _next_link = m_link_factory->get_link (_next_link_ID);
                      _veh->set_next_link (_next_link);
                      m_tracker.find (_veh)->second->pop_front ();
                    }
                } // end if-else
            }     // end if veh->m_type
        }         // end for veh_it
    }             // end for link_it

  return 0;
}


/**************** Hybrid Routing ********************/
MNM_Routing_Biclass_Hybrid_Subclass::MNM_Routing_Biclass_Hybrid_Subclass(
    const std::string &file_folder, macposts::Graph &graph, std::vector<macposts::Graph> &graph_vec,
    MNM_Statistics *statistics, MNM_OD_Factory *od_factory,
    MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
    std::vector<Path_Table*> &path_table_vec,
    std::unordered_map<TInt, TInt> &subclass_car_graph_mapping, std::unordered_map<TInt, TInt> &subclass_car_path_table_mapping,
    std::unordered_map<TInt, TInt> &subclass_truck_graph_mapping, std::unordered_map<TInt, TInt> &subclass_truck_path_table_mapping,
    TInt route_frq_fixed, TInt buffer_length)
    : MNM_Routing::MNM_Routing (graph, od_factory, node_factory, link_factory)
{
    // graph_vec is a vector of graphs, the first one is an overarching graph
    auto *_tmp_config = new MNM_ConfReader (file_folder + "/config.conf", "DTA");
    int _num_assign_interval = _tmp_config->get_int ("max_interval");
    int _num_subclass_car = _tmp_config->get_int ("num_subclass_car");
    int _num_subclass_truck = _tmp_config->get_int ("num_subclass_truck");

    m_routing_fixed_car_subclass = std::unordered_map<int, MNM_Routing_Biclass_Fixed_Subclass*> ();
    m_routing_adaptive_car_subclass = std::unordered_map<int, MNM_Routing_Adaptive*> ();
    m_routing_fixed_truck_subclass = std::unordered_map<int, MNM_Routing_Biclass_Fixed_Subclass*> ();
    m_routing_adaptive_truck_subclass = std::unordered_map<int, MNM_Routing_Adaptive*> ();

    // Initialize the routing algorithm for each subclass
    for (int i = 0; i < _num_subclass_car; ++i)
    {
        m_routing_fixed_car_subclass.insert(std::make_pair(i, nullptr));
        m_routing_fixed_car_subclass[i] = new MNM_Routing_Biclass_Fixed_Subclass (
            graph_vec[subclass_car_graph_mapping[i]], od_factory, node_factory,
            link_factory, route_frq_fixed,
            buffer_length, i*_num_assign_interval,
            TInt (0), i);
        // m_routing_fixed_car_subclass.at(i) -> m_graph = graph_vec[subclass_car_graph_mapping[i]];
        m_routing_fixed_car_subclass.at(i) -> m_path_table = path_table_vec[subclass_car_path_table_mapping[i]];

        m_routing_adaptive_car_subclass.insert(std::make_pair(i, nullptr));
        m_routing_adaptive_car_subclass[i] = new MNM_Routing_Adaptive (
            file_folder, graph_vec[subclass_car_graph_mapping[i]], statistics, od_factory,
            node_factory, link_factory);
        // m_routing_adaptive_car_subclass.at(i) -> m_graph = graph_vec[subclass_car_graph_mapping[i]];
        m_routing_adaptive_car_subclass.at(i) -> m_working
            = _tmp_config->get_float ("adaptive_ratio_car") > 0;
    }

    for (int i = 0; i < _num_subclass_truck; ++i)
    {
        m_routing_fixed_truck_subclass.insert(std::make_pair(
            i, nullptr
        ));
        m_routing_fixed_truck_subclass[i] = 
            new MNM_Routing_Biclass_Fixed_Subclass (graph_vec[subclass_truck_graph_mapping[i]], od_factory, node_factory,
                                        link_factory, route_frq_fixed,
                                        buffer_length, _num_subclass_car*_num_assign_interval + i*_num_assign_interval,
                                        TInt (1), i);
        m_routing_fixed_truck_subclass.at(i) -> m_path_table = path_table_vec[subclass_truck_path_table_mapping[i]];
        m_routing_adaptive_truck_subclass.insert(std::make_pair(
            i, nullptr
        ));
        m_routing_adaptive_truck_subclass[i] = 
            new MNM_Routing_Adaptive (file_folder, graph_vec[subclass_truck_graph_mapping[i]], statistics, od_factory,
                                    node_factory, link_factory);
        m_routing_adaptive_truck_subclass.at(i) -> m_working
            = _tmp_config->get_float ("adaptive_ratio_truck") > 0;
    }
    delete _tmp_config;
};

MNM_Routing_Biclass_Hybrid_Subclass::~MNM_Routing_Biclass_Hybrid_Subclass()
{
    for (auto _it : m_routing_fixed_car_subclass)
    {
        // multiple car subclasses may share the same path table
        // MNM_Routing_Fixed deconstructor uses try-catch to handle this
        delete _it.second;
    }
    for (auto _it : m_routing_adaptive_car_subclass)
    {
        delete _it.second;
    }
    for (auto _it : m_routing_fixed_truck_subclass)
    {
        // multiple truck subclasses may share the same path table
        // MNM_Routing_Fixed deconstructor uses try-catch to handle this
        delete _it.second;
    }
    for (auto _it : m_routing_adaptive_truck_subclass)
    {
        delete _it.second;
    }
}

int 
MNM_Routing_Biclass_Hybrid_Subclass::init_routing (Path_Table *driving_path_table)
{
    for (auto _it : m_routing_fixed_car_subclass)
    {
        _it.second->init_routing (nullptr);
    }
    for (auto _it : m_routing_adaptive_car_subclass)
    {
        if (_it.second->m_working)
        {
            _it.second->init_routing (nullptr);
        }
    }
    for (auto _it : m_routing_fixed_truck_subclass)
    {
        _it.second->init_routing (nullptr);
    }
    for (auto _it : m_routing_adaptive_truck_subclass)
    {
        if (_it.second->m_working)
        {
            _it.second->init_routing (nullptr);
        }
    }
    return 0;
}

int 
MNM_Routing_Biclass_Hybrid_Subclass::update_routing (TInt timestamp)
{
    for (auto _it : m_routing_fixed_car_subclass)
    {
        _it.second->update_routing (timestamp);
    }
    for (auto _it : m_routing_adaptive_car_subclass)
    {
        if (_it.second->m_working)
        {
            _it.second->update_routing (timestamp);
        }
    }
    for (auto _it : m_routing_fixed_truck_subclass)
    {
        _it.second->update_routing (timestamp);
    }
    for (auto _it : m_routing_adaptive_truck_subclass)
    {
        if (_it.second->m_working)
        {
            _it.second->update_routing (timestamp);
        }
    }
    return 0;
}

int 
MNM_Routing_Biclass_Hybrid_Subclass::remove_finished (MNM_Veh *veh, bool del)
{
    if (veh->get_class() == 0)
    {
        m_routing_fixed_car_subclass[dynamic_cast<MNM_Veh_Multiclass_Subclass*>(veh)->get_subclass()]->remove_finished (veh, del);
    }
    else if (veh->get_class() == 1)
    {
        m_routing_fixed_truck_subclass[dynamic_cast<MNM_Veh_Multiclass_Subclass*>(veh)->get_subclass()]->remove_finished (veh, del);
    }
    else
    {
        throw std::runtime_error ("wrong vehicle class"); 
    }
    return 0;
}

/**************************************************************************
                            IO
**************************************************************************/
int
MNM_IO_Multiclass_Subclass::build_demand_subclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            int mainclass_label,
                                            const std::string &file_name)
{
    /* find file */
    std::string _demand_file_name = file_folder + "/" + file_name;
    std::ifstream _demand_file;
    _demand_file.open (_demand_file_name, std::ios::in);

    /* read config */
    TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
    TInt _unit_time = conf_reader->get_int ("unit_time");
    TInt _num_of_minute = int (conf_reader->get_int ("assign_frq"))
                        / (60 / _unit_time); // the releasing strategy is
                                                // assigning vehicles per 1 minute
    TInt _max_interval = conf_reader->get_int ("max_interval");
    TInt _num_OD = conf_reader->get_int ("OD_pair");
    TInt _init_demand_split = conf_reader->get_int ("init_demand_split");

    int _num_subclass;
    if (mainclass_label == 0) {
        _num_subclass = conf_reader->get_int ("num_subclass_car");
    }
    else if (mainclass_label == 1) {
        _num_subclass = conf_reader->get_int ("num_subclass_truck");
    }

    /* build */
    TInt _O_ID, _D_ID;
    MNM_Origin_Multiclass_Subclass *_origin;
    MNM_Destination_Multiclass *_dest;
    std::string _line;
    std::vector<std::string> _words;
    if (_demand_file.is_open ())
    {
        // printf("Start build demand profile.\n");
        double *_demand_vector = new double[_max_interval * _num_of_minute]();
        TFlt _demand;

        for (int i = 0; i < _num_OD;)
        {
            std::getline (_demand_file, _line);
            _line = trim (_line);
            if (_line.empty () || _line[0] == '#')
            continue;
            ++i;
            _words = split (_line, ' ');
            if (TInt (_words.size ()) == (_max_interval * _num_subclass + 2))
            {
                _O_ID = TInt (std::stoi (_words[0]));
                _D_ID = TInt (std::stoi (_words[1]));
                _origin = dynamic_cast<MNM_Origin_Multiclass_Subclass *> (od_factory->get_origin (_O_ID));
                _dest = dynamic_cast<MNM_Destination_Multiclass *> (od_factory->get_destination (_D_ID));
                od_factory->m_destination_with_demand_set.insert (_dest);
                for (int q = 0; q < _num_subclass; ++q) 
                {
                    memset (_demand_vector, 0x0, sizeof (TFlt) * _max_interval * _num_of_minute);
                    // the releasing strategy is assigning vehicles per 1 minute, so
                    // disaggregate 15-min demand into 1-min demand
                    for (int j = 0; j < _max_interval; ++j)
                    {
                        if (_init_demand_split == 0)
                        {
                            
                            _demand = TFlt (std::stod (_words[2 + q * _max_interval + j]));
                            _demand_vector[j * _num_of_minute] = _demand;
                        }
                        else if (_init_demand_split == 1)
                        {
                            // find suitable releasing interval so that the
                            // agent-based DNL is feasible
                            for (int p = 0; p < _num_of_minute; ++p)
                            {
                                _demand = TFlt (std::stod (_words[2 + q * _max_interval + j]))
                                            / TFlt (_num_of_minute - p);
                                // if (round(_demand * _flow_scalar) >= 1){
                                if (floor (_demand * _flow_scalar) >= 1)
                                {
                                    for (int k = 0; k < _num_of_minute - p; ++k)
                                    {
                                        _demand_vector[j * _num_of_minute + k] = _demand;
                                    }
                                    break;
                                }
                            }
                        }
                        else
                        {
                            throw std::runtime_error ("wrong init_demand_split");
                        }
                    }

                    _origin->add_dest_demand_multiclass_subclass (_dest, mainclass_label, q, _demand_vector);
                }
            }
            else
            {
                delete[] _demand_vector;
                throw std::runtime_error ("failed to build subclass demand");
            }
        }
        delete[] _demand_vector;
        _demand_file.close ();
    }
    return 0;
}

macposts::Graph
MNM_IO_Multiclass_Subclass::build_graph (const std::string &graph_file_name,
                                        MNM_ConfReader *conf_reader)
{
    /* find file */
    std::ifstream _graph_file;
    _graph_file.open (graph_file_name, std::ios::in);
    // this is the maximum number of links
    TInt _num_of_link = conf_reader->get_int ("num_of_link");

    macposts::Graph _graph;

    int _link_ID, _from_ID, _to_ID;
    std::string _line;
    std::vector<std::string> _words;
    for (int i = 0; i < _num_of_link;)
    {
        if (!std::getline(_graph_file, _line))
        {
            // Handle the case where the number of lines is less than _num_of_link
            std::cerr << "Warning: The graph file has fewer lines than the maxiumum number of links.\n";
            break;
        }
        _line = MNM_IO::trim (_line);
        if (_line.empty () || _line[0] == '#')
        {
            continue;
        }
        ++i;
        _words = MNM_IO::split (_line, ' ');
        if (_words.size () == 3)
        {
            // std::cout << "Processing: " << _line << "\n";
            _link_ID = TInt (std::stoi (_words[0]));
            _from_ID = TInt (std::stoi (_words[1]));
            _to_ID = TInt (std::stoi (_words[2]));
            try
            {
                _graph.add_node (_from_ID);
            }
            // FIXME: This could be overly generic. Maybe we should use a more
            // specific exception type.
            catch (const std::runtime_error &)
            {
            }
            try
            {
                _graph.add_node (_to_ID);
            }
            // FIXME: This could be overly generic. Maybe we should use a more
            // specific exception type.
            catch (const std::runtime_error &)
            {
            }
            _graph.add_link (_from_ID, _to_ID, _link_ID);
        }
    }
    // subgraphs
    std::cout << graph_file_name << "\n";
    std::cout << "Number of maximum links: " << _num_of_link << "\n";
    std::cout << "Number of actual links: " << _graph.size_links () << "\n";
    assert ((std::ptrdiff_t) _graph.size_links () <= _num_of_link);
    return _graph;
}

int
MNM_IO_Multiclass_Subclass::build_graph_vec (const std::string &file_folder,
                                             MNM_ConfReader *conf_reader,
                                             int num_graph,
                                             std::vector<macposts::Graph> &graph_vec)
{
    // assume the first graph is an overarching graph, and each subsequent graph has a correspoinding path table
    // _num_graph does NOT inlcude the overarching graph
    // macposts::Graph cannot be copied directly due to unique pointer used for nodes and links
    // we add move constructor and assignment operator to macposts::Graph to enable this on 01/17/2025
    graph_vec.reserve(num_graph + 1);

    std::string _network_name = conf_reader->get_string ("network_name");
    std::string _graph_file_name;
    macposts::Graph _graph;

    for (int i = 0; i < num_graph + 1; ++i)
    {   

        if (i == 0) {
            // main graph: Snap_graph containing all links
            _graph_file_name = file_folder + "/" + _network_name;
        }
        else {
            // subgraphs: Snap_graph_1, Snap_graph_2, ...
            _graph_file_name = file_folder + "/" + _network_name + "_" + std::to_string (i);
        }
        _graph = build_graph(_graph_file_name, conf_reader);
        graph_vec.push_back (std::move(_graph));
        std::cout << "Graph " << i << " loaded." << std::endl;
        std::cout << "Graph " << i << " number of links: " << graph_vec.back().size_links()  << std::endl;
        std::cout << "Graph " << i << " number of nodes: " << graph_vec.back().size_nodes()  << std::endl;
    }
    return 0;
}

/**************************************************************************
                            DTA
**************************************************************************/
MNM_Dta_Multiclass_Subclass::MNM_Dta_Multiclass_Subclass (const std::string &file_folder)
    : MNM_Dta_Multiclass::MNM_Dta_Multiclass (file_folder)
{
    initialize();
    // assume the first graph is an overarching graph, and each subsequent graph has a correspoinding path table
    // m_num_graph does NOT include the overarching graph
    m_num_graph = m_config->get_int ("num_graph");
    m_num_path_table = m_config->get_int ("num_path_table");
    assert (m_num_graph == m_num_path_table);
    m_num_subclass_car = m_config->get_int ("num_subclass_car");
    m_num_subclass_truck = m_config->get_int ("num_subclass_truck");

    m_graph_vec = std::vector<macposts::Graph> ();

    m_subclass_car_graph_mapping = std::unordered_map<int, int> ();
    m_subclass_car_path_table_mapping = std::unordered_map<int, int> ();
    m_path_table_subclass_car_mapping = std::unordered_map<int, std::vector<int>> ();

    m_subclass_truck_graph_mapping = std::unordered_map<int, int> ();
    m_subclass_truck_path_table_mapping = std::unordered_map<int, int> ();
    m_path_table_subclass_truck_mapping = std::unordered_map<int, std::vector<int>> ();

    std::string _line;
    std::vector<std::string> _words;

    _line = m_config->get_string("subclass_car_graph_mapping");
    _words = MNM_IO::split (_line, ',');
    if ((int)_words.size () != m_num_subclass_car) {
        std::cout << "Error: car graph mapping size != _num_subclass_car" << std::endl;
        exit (1);
    }
    for (int i = 0; i < m_num_subclass_car; ++i) {
        m_subclass_car_graph_mapping.insert(std::make_pair(i, std::stoi (_words[i])));
    }
    _line = m_config->get_string("subclass_car_path_table_mapping");
    _words = MNM_IO::split (_line, ',');
    if ((int)_words.size () != m_num_subclass_car) {
        std::cout << "Error: car path table mapping size != _num_subclass_car" << std::endl;
        exit (1);
    }
    for (int i = 0; i < m_num_subclass_car; ++i) {
        m_subclass_car_path_table_mapping.insert(std::make_pair(i, std::stoi (_words[i])));
        if (m_path_table_subclass_car_mapping.find(std::stoi (_words[i])) == m_path_table_subclass_car_mapping.end ()) {
            m_path_table_subclass_car_mapping.insert(std::make_pair(std::stoi (_words[i]), std::vector<int> ()));
        }
        m_path_table_subclass_car_mapping[std::stoi (_words[i])].push_back (i);
    }


    _line = m_config->get_string("subclass_truck_graph_mapping");
    _words = MNM_IO::split (_line, ',');
    if ((int)_words.size () != m_num_subclass_truck) {
        std::cout << "Error: truck graph mapping size != _num_subclass_truck" << std::endl;
    }
    for (int i = 0; i < m_num_subclass_truck; ++i) {
        m_subclass_truck_graph_mapping.insert(std::make_pair(i, std::stoi (_words[i])));
    }
    _line = m_config->get_string("subclass_truck_path_table_mapping");
    _words = MNM_IO::split (_line, ',');
    if ((int)_words.size () != m_num_subclass_truck) {
        std::cout << "Error: truck path table mapping size != _num_subclass_truck" << std::endl;
        exit (1);
    }
    for (int i = 0; i < m_num_subclass_truck; ++i) {
        m_subclass_truck_path_table_mapping.insert(std::make_pair(i, std::stoi (_words[i])));
        if (m_path_table_subclass_truck_mapping.find(std::stoi (_words[i])) == m_path_table_subclass_truck_mapping.end ()) {
            m_path_table_subclass_truck_mapping.insert(std::make_pair(std::stoi (_words[i]), std::vector<int> ()));
        }
        m_path_table_subclass_truck_mapping[std::stoi (_words[i])].push_back (i);
    }
}

MNM_Dta_Multiclass_Subclass::~MNM_Dta_Multiclass_Subclass ()
{
    m_subclass_car_graph_mapping.clear ();
    m_subclass_car_path_table_mapping.clear ();
    m_subclass_truck_graph_mapping.clear ();
    m_subclass_truck_path_table_mapping.clear ();
    for (auto _it : m_path_table_subclass_car_mapping) {
        _it.second.clear ();
    }
    m_path_table_subclass_car_mapping.clear ();
    for (auto _it : m_path_table_subclass_truck_mapping) {
        _it.second.clear ();
    }
    m_path_table_subclass_truck_mapping.clear ();
    m_graph_vec.clear ();
}

int
MNM_Dta_Multiclass_Subclass::initialize ()
{   
    MNM_Dta_Multiclass::initialize ();
    if (m_veh_factory != nullptr) delete m_veh_factory;
    m_veh_factory = new MNM_Veh_Factory_Multiclass_Subclass ();
    if (m_od_factory != nullptr) delete m_od_factory;
    m_od_factory = new MNM_OD_Factory_Multiclass_Subclass ();
    return 0;
}

int 
MNM_Dta_Multiclass_Subclass::build_from_files ()
{
    MNM_IO_Multiclass::build_node_factory_multiclass (m_file_folder, m_config,
                                                    m_node_factory);
    MNM_IO_Multiclass::build_link_factory_multiclass (m_file_folder, m_config,
                                                    m_link_factory);
    MNM_IO_Multiclass::build_od_factory (m_file_folder, m_config, m_od_factory,
                                        m_node_factory);
    MNM_IO_Multiclass_Subclass::build_graph_vec (m_file_folder, m_config, m_num_graph, m_graph_vec);
    for (int i = 0; i < m_num_graph+1; i++) {
        std::cout << m_graph_vec[i].size_links () << std::endl;
        if (m_graph_vec[i].size_links () == 0) {
            std::cerr << "Graph " << i << " has no links" << std::endl;
            return -1;
        }
    }
    assert(m_num_graph + 1 == (int)m_graph_vec.size ());
    // due to unique pointer used by macposts:Graph, we need to transfer ownership
    // after this the graph vec's size will be reduced by 1
    m_graph = std::move(m_graph_vec[0]);
    if (m_graph.size_links () == 0) {
        std::cerr << "Graph 0 has no links" << std::endl;
        return -1;
    }
    m_graph_vec.erase(m_graph_vec.begin());   // Remove it from the vector
    assert(m_num_graph == (int)m_graph_vec.size ());

    MNM_IO_Multiclass_Subclass::build_demand_subclass (m_file_folder, m_config,
                                                      m_od_factory, 0, "MNM_input_demand_car");
    MNM_IO_Multiclass_Subclass::build_demand_subclass (m_file_folder, m_config,
                                                       m_od_factory, 1, "MNM_input_demand_truck");
    MNM_IO_Multiclass::read_origin_car_label_ratio (m_file_folder, m_config,
                                                    m_od_factory);
    MNM_IO_Multiclass::read_origin_truck_label_ratio (m_file_folder, m_config,
                                                    m_od_factory);
    MNM_IO_Multiclass::build_link_toll_multiclass (m_file_folder, m_config,
                                                    m_link_factory);
    MNM_IO_Multiclass::build_link_td_attribute (m_file_folder, m_link_factory);
    MNM_IO::build_node_td_cost(m_file_folder, m_link_factory);
    MNM_IO_Multiclass::build_td_adaptive_ratio(m_file_folder, m_config, m_od_factory);
    // build_workzone();
    m_workzone = nullptr;
    set_statistics ();
    set_gridlock_recorder ();
    set_routing ();
    return 0;
}

int 
MNM_Dta_Multiclass_Subclass::set_routing ()
{
    if (m_config->get_string ("routing_type") == "Biclass_Hybrid_Subclass")
    {
        auto *_tmp_conf = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
        // assume m_buffer_length = max_interval, different from multiclass
        TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
        if (_buffer_len != m_config->get_int ("max_interval"))
        {
            _buffer_len = m_config->get_int ("max_interval");
        }
        TInt _route_freq_fixed = _tmp_conf->get_int ("route_frq");

        std::string _num_driving_path_str = _tmp_conf->get_string ("num_path");
        std::vector<TInt> _num_driving_path_vec;
        std::stringstream _ss(_num_driving_path_str);
        std::string _tmp;
        while (getline(_ss, _tmp, ',')) {
            _num_driving_path_vec.push_back(std::stoi(_tmp));
        }
        assert(m_num_path_table == (int)_num_driving_path_vec.size());
        
        std::vector<Path_Table*> path_table_vec = std::vector<Path_Table*>();
        Path_Table *_driving_path_table = nullptr;

        bool _with_buffer = (_tmp_conf->get_string ("choice_portion") == "Buffer");
        for (int i = 0; i < m_num_path_table; i++) {
            // the buffer for each path table has all vehicle subclasses, use 0 for some of them as placeholder to keep the same dimension
            _driving_path_table = MNM_IO::load_path_table (
                m_file_folder + "/path_table_" + std::to_string (i),
                m_graph_vec[i],
                _num_driving_path_vec[i],
                _with_buffer
            );
            path_table_vec.push_back(_driving_path_table);
        }
        
        m_routing = new MNM_Routing_Biclass_Hybrid_Subclass(
            m_file_folder,          // const std::string &file_folder
            m_graph,                // macposts::Graph &graph
            m_graph_vec,            // std::vector<macposts::Graph*> &graph_vec
            m_statistics,           // MNM_Statistics *statistics
            m_od_factory,           // MNM_OD_Factory *od_factory
            m_node_factory,         // MNM_Node_Factory *node_factory
            m_link_factory,         // MNM_Link_Factory *link_factory
            path_table_vec,       // std::vector<Path_Table*> path_table_vec
            m_subclass_car_graph_mapping,  // std::unordered_map<int, int> subclass_car_graph_mapping
            m_subclass_car_path_table_mapping,  // std::unordered_map<int, int> subclass_car_path_table_mapping
            m_subclass_truck_graph_mapping,  // std::unordered_map<int, int> subclass_truck_graph_mapping
            m_subclass_truck_path_table_mapping,  // std::unordered_map<int, int> subclass_truck_path_table_mapping
            _route_freq_fixed,      // TInt route_frq_fixed
            _buffer_len         // TInt buffer_length
        );

        m_routing->init_routing (nullptr);

        delete _tmp_conf;
    }
    else
    {
        throw std::runtime_error (
            "Wrong routing type for MNM_Routing_Biclass_Hybrid_Subclass");
    }

    return 0;
}

int
MNM_Dta_Multiclass_Subclass::load_once (bool verbose, TInt load_int, TInt assign_int)
{
    MNM_Origin *_origin;
    MNM_Dnode *_node;
    MNM_Dlink *_link;
    MNM_Destination *_dest;

    // update some link attributes over time
    m_link_factory->update_link_attribute (load_int, verbose);
    // compute empty network link tt, for adaptive routing
    if (load_int == 0) m_statistics->update_record (load_int);
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
                _origin->release_one_interval (load_int, m_veh_factory, -1, TFlt (-1));
            }
            else
            {
                if ((m_config->get_string ("routing_type") == "Biclass_Hybrid_Subclass"))
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