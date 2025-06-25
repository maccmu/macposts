#include "factory.h"

/**************************************************************************
                          Vehicle node
**************************************************************************/
MNM_Veh_Factory::MNM_Veh_Factory ()
{
  m_veh_map = std::unordered_map<TInt, MNM_Veh *> ();
  m_num_veh = TInt (0);

  m_enroute = TInt (0);
  m_finished = TInt (0);

  m_total_time = TFlt (0);
  m_total_delay = TFlt (0);
  m_total_miles = TFlt (0);
}

MNM_Veh_Factory::~MNM_Veh_Factory ()
{
  MNM_Veh *_veh;
  for (auto _veh_it : m_veh_map)
    {
      _veh = _veh_it.second;
      delete _veh;
    }
  m_veh_map.clear ();
}

MNM_Veh *
MNM_Veh_Factory::make_veh (TInt timestamp, Vehicle_type veh_type)
{
  // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp,
  // (int)m_num_veh + 1);
  MNM_Veh *_veh = new MNM_Veh (m_num_veh + 1, timestamp);
  _veh->m_type = veh_type;
  m_veh_map.insert (std::pair<TInt, MNM_Veh *> (m_num_veh + 1, _veh));
  m_num_veh += 1;
  m_enroute += 1;
  return _veh;
}

int
MNM_Veh_Factory::remove_finished_veh (MNM_Veh *veh, bool del)
{
  if (m_veh_map.find (veh->m_veh_ID) == m_veh_map.end ()
      || m_veh_map.find (veh->m_veh_ID)->second != veh)
    {
      throw std::runtime_error ("vehicle not found");
    }

  if (del)
    {
      m_veh_map.erase (veh->m_veh_ID);
    }

  IAssert (veh->m_finish_time > veh->m_start_time);
  m_total_time += (veh->m_finish_time - veh->m_start_time);
  m_total_delay += std::max(0, (veh->m_finish_time - veh->m_start_time) - veh -> m_cumulative_freeflow_time);
  m_total_miles += veh -> m_miles_traveled;

  if (del)
    {
      delete veh;
    }

  m_finished += 1;
  m_enroute -= 1;
  IAssert (m_num_veh == m_finished + m_enroute);
  return 0;
}

int 
MNM_Veh_Factory::update_veh_stat ()
{
  // account for enroute vehicles, do it only once at the end of DNL
  // only consider the complete links those vehicle traversed
  MNM_Veh *_veh;
  for (auto _map_it : m_veh_map)
  {
    _veh = _map_it.second;
    IAssert (_veh->m_finish_time < 0);
    m_total_time += std::max(0, (_veh -> m_last_link_exiting_time - _veh->m_start_time));
    m_total_delay += std::max(0, (_veh->m_last_link_exiting_time - _veh->m_start_time) - _veh -> m_cumulative_freeflow_time);
    m_total_miles += _veh->m_miles_traveled;
  }
  return 0;
}

std::string 
MNM_Veh_Factory::print_vehicle_statistics ()
{
  // note veh_factory->update_veh_stat() is called before this function, if not, this only includes finished vehicles' stat
  std::ostringstream oss;
  oss << "############################################### Vehicle Statistics ###############################################\n"
      << "Released Vehicle total " << m_num_veh << ", Enroute Vehicle Total " << m_enroute << ", Finished Vehicle Total " << m_finished << ",\n"
      << "Total Miles Traveled: " << std::fixed << m_total_miles << " miles, Total Travel Time: " << std::fixed << m_total_time << " intervals, Total Delayed Time: " << std::fixed << m_total_delay << " intervals\n"
      << "############################################## Vehicle Statistics ###############################################\n";
  return oss.str();
  
}

/**************************************************************************
                          Node factory
**************************************************************************/

MNM_Node_Factory::MNM_Node_Factory ()
{
  m_node_map = std::unordered_map<TInt, MNM_Dnode *> ();
}

MNM_Node_Factory::~MNM_Node_Factory ()
{
  for (auto _map_it = m_node_map.begin (); _map_it != m_node_map.end ();
       _map_it++)
    {
      delete _map_it->second;
    }
  m_node_map.clear ();
}

MNM_Dnode *
MNM_Node_Factory::make_node (TInt ID, DNode_type node_type, TFlt flow_scalar)
{
  MNM_Dnode *_node;
  switch (node_type)
    {
    case MNM_TYPE_FWJ:
      _node = new MNM_Dnode_FWJ (ID, flow_scalar);
      break;
    case MNM_TYPE_GRJ:
      _node = new MNM_Dnode_GRJ (ID, flow_scalar);
      break;
    case MNM_TYPE_ORIGIN:
      _node = new MNM_DMOND (ID, flow_scalar);
      break;
    case MNM_TYPE_DEST:
      _node = new MNM_DMDND (ID, flow_scalar);
      break;
    default:
      throw std::runtime_error ("unknown node type");
    }
  m_node_map.insert (std::pair<TInt, MNM_Dnode *> (ID, _node));
  return _node;
}

MNM_Dnode *
MNM_Node_Factory::get_node (TInt ID)
{
  auto _node_it = m_node_map.find (ID);
  if (_node_it == m_node_map.end ())
    {
      printf ("No such node ID %d\n", (int) ID);
      throw std::runtime_error (
        "Error, MNM_Node_Factory::get_node, node does not exist");
    }
  return _node_it->second;
}

/**************************************************************************
                          Link factory
**************************************************************************/
MNM_Link_Factory::MNM_Link_Factory ()
{
  m_link_map = std::unordered_map<TInt, MNM_Dlink *> ();
  m_td_link_attribute_table = new std::unordered_map<
    int, std::unordered_map<int, td_link_attribute_row *> *> ();
  m_td_node_cost_table = new std::unordered_map<
    int, std::unordered_map<int, std::unordered_map<int, float> *> *>();
}

MNM_Link_Factory::~MNM_Link_Factory ()
{
  for (auto _it : *m_td_link_attribute_table)
    {
      for (auto _it_it : *(_it.second))
        {
          delete _it_it.second;
        }
      _it.second->clear ();
    }
  m_td_link_attribute_table->clear ();
  delete m_td_link_attribute_table;

  // Clear m_td_node_cost_table
  for (auto _it : *m_td_node_cost_table)
  {
    for (auto _it_it : *(_it.second))
    {
      _it_it.second -> clear();
      delete _it_it.second;
    }
    _it.second->clear();
    delete _it.second;
  }
  m_td_node_cost_table->clear();
  delete m_td_node_cost_table;
  
  for (auto _map_it = m_link_map.begin (); _map_it != m_link_map.end ();
       _map_it++)
    {
      delete _map_it->second;
    }
  m_link_map.clear ();
}

MNM_Dlink *
MNM_Link_Factory::make_link (TInt ID, DLink_type link_type, TFlt lane_hold_cap,
                             TFlt lane_flow_cap, TInt number_of_lane,
                             TFlt length, TFlt ffs, TFlt unit_time,
                             TFlt flow_scalar)
{
  MNM_Dlink *_link;
  switch (link_type)
    {
    case MNM_TYPE_CTM:
      _link
        = new MNM_Dlink_Ctm (ID, lane_hold_cap, lane_flow_cap, number_of_lane,
                             length, ffs, unit_time, flow_scalar);
      break;
    case MNM_TYPE_PQ:
      _link
        = new MNM_Dlink_Pq (ID, lane_hold_cap, lane_flow_cap, number_of_lane,
                            length, ffs, unit_time, flow_scalar);
      break;
    case MNM_TYPE_LQ:
      _link
        = new MNM_Dlink_Lq (ID, lane_hold_cap, lane_flow_cap, number_of_lane,
                            length, ffs, unit_time, flow_scalar);
      break;
    case MNM_TYPE_LTM:
      _link
        = new MNM_Dlink_Ltm (ID, lane_hold_cap, lane_flow_cap, number_of_lane,
                             length, ffs, unit_time, flow_scalar);
      break;
    default:
      throw std::runtime_error ("unknown link type");
    }
  m_link_map.insert (std::pair<TInt, MNM_Dlink *> (ID, _link));
  return _link;
}

MNM_Dlink *
MNM_Link_Factory::get_link (TInt ID)
{
  auto _link_it = m_link_map.find (ID);
  if (_link_it == m_link_map.end ())
    {
      throw std::runtime_error (
        "Error, MNM_Link_Factory::get_link, link does not exist");
    }
  return _link_it->second;
}

int
MNM_Link_Factory::delete_link (TInt ID)
{
  auto _map_it = m_link_map.find (ID);
  if (_map_it == m_link_map.end ())
    {
      throw std::runtime_error ("failed to delete link");
    }
  m_link_map.erase (_map_it);
  return 0;
}

int
MNM_Link_Factory::update_link_attribute(TInt interval, bool verbose)
{
    if (m_td_link_attribute_table->find (interval)
      != m_td_link_attribute_table->end ())
    {
      for (auto _it : *(m_td_link_attribute_table->find (interval)->second))
        {
          auto _link = m_link_map.find (_it.first)->second;
          _link->modify_property (_it.second->Lane, _it.second->length,
                                  _it.second->RHOJ,
                                  _it.second->Cap,
                                  _it.second->FFS);
          _link->m_toll = _it.second->toll;
          if (verbose)
            printf ("link %d attribute updated\n", _it.first);
        }
    }
    return 0; // or appropriate return value
}

/**************************************************************************
                          OD factory
**************************************************************************/
MNM_OD_Factory::MNM_OD_Factory ()
{
  m_origin_map = std::unordered_map<TInt, MNM_Origin *> ();
  m_destination_map = std::unordered_map<TInt, MNM_Destination *> ();
  m_destination_with_demand_set = std::set<MNM_Destination *> ();
}

MNM_OD_Factory::~MNM_OD_Factory ()
{
  for (auto _map_it = m_origin_map.begin (); _map_it != m_origin_map.end ();
       _map_it++)
    {
      delete _map_it->second;
    }
  m_origin_map.clear ();
  for (auto _map_it = m_destination_map.begin ();
       _map_it != m_destination_map.end (); _map_it++)
    {
      delete _map_it->second;
    }
  m_destination_map.clear ();
  m_destination_with_demand_set.clear ();
}

MNM_Destination *
MNM_OD_Factory::make_destination (TInt ID)
{
  MNM_Destination *_dest;
  _dest = new MNM_Destination (ID);
  m_destination_map.insert (std::pair<TInt, MNM_Destination *> (ID, _dest));
  return _dest;
}

MNM_Origin *
MNM_OD_Factory::make_origin (TInt ID, TInt max_interval, TFlt flow_scalar,
                             TInt frequency)
{
  MNM_Origin *_origin;
  _origin = new MNM_Origin (ID, max_interval, flow_scalar, frequency);
  m_origin_map.insert (std::pair<TInt, MNM_Origin *> (ID, _origin));
  return _origin;
}

MNM_Destination *
MNM_OD_Factory::get_destination (TInt ID)
{
  auto _d_it = m_destination_map.find (ID);
  if (_d_it == m_destination_map.end ())
    {
      throw std::runtime_error (
        "Error, MNM_OD_Factory::get_destination, destination does not exist");
    }
  return _d_it->second;
}

MNM_Origin *
MNM_OD_Factory::get_origin (TInt ID)
{
  auto _o_it = m_origin_map.find (ID);
  if (_o_it == m_origin_map.end ())
    {
      throw std::runtime_error (
        "Error, MNM_OD_Factory::get_origin, origin does not exist");
    }
  return _o_it->second;
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_OD_Factory::get_random_od_pair ()
{
  MNM_Origin *_origin;
  MNM_Destination *_dest;

  auto _origin_it = m_origin_map.begin ();
  int random_index = rand () % m_origin_map.size ();
  std::advance (_origin_it, random_index);

  _origin = _origin_it->second;
  while (_origin->m_demand.empty ())
    {
      _origin_it = m_origin_map.begin ();
      random_index = rand () % m_origin_map.size ();
      std::advance (_origin_it, random_index);
      _origin = _origin_it->second;
    }

  auto _dest_it = _origin->m_demand.begin ();
  random_index = rand () % _origin->m_demand.size ();
  std::advance (_dest_it, random_index);
  _dest = _dest_it->first;

  return std::pair<MNM_Origin *, MNM_Destination *> (_origin, _dest);
}
