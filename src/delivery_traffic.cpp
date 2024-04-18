#include "delivery_traffic.h"

//#################################################################
//                       Delivery Vehicle
//#################################################################

MNM_Veh_Delivery::MNM_Veh_Delivery (TInt ID, TInt start_time)
    : MNM_Veh::MNM_Veh (ID, start_time)
{
  m_multi_od_seq = nullptr;
  m_current_OD_index = -1;
}

MNM_Veh_Delivery::~MNM_Veh_Delivery () { m_multi_od_seq = nullptr; }

int
MNM_Veh_Delivery::set_multi_od_seq (
  std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *multi_od_seq)
{
  Assert (multi_od_seq->size () > 1);
  m_multi_od_seq = multi_od_seq;
  m_current_OD_index = 0;
  return 0;
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_Veh_Delivery::get_current_od ()
{
  if (m_multi_od_seq == nullptr)
    {
      return std::make_pair (get_origin (), get_destination ());
    }
  else
    {
      return m_multi_od_seq->at (m_current_OD_index);
    }
}

int
MNM_Veh_Delivery::move_to_next_od ()
{
  if (m_current_OD_index < (std::ptrdiff_t) m_multi_od_seq->size () - 1)
    {
      m_current_OD_index += 1;
      set_origin (m_multi_od_seq->at (m_current_OD_index).first);
      set_destination (m_multi_od_seq->at (m_current_OD_index).second);
    }
  return 0;
}

//#################################################################
//             Factory for Delivery and Normal Vehicle
//#################################################################

MNM_Veh_Factory_Delivery::MNM_Veh_Factory_Delivery ()
    : MNM_Veh_Factory::MNM_Veh_Factory ()
{
  m_veh_delivery = 0;
}

MNM_Veh_Factory_Delivery::~MNM_Veh_Factory_Delivery () { ; }

MNM_Veh_Delivery *
MNM_Veh_Factory_Delivery::make_veh_delivery (TInt timestamp,
                                             Vehicle_type veh_type)
{
  // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp,
  // (int)m_num_veh + 1);
  MNM_Veh_Delivery *_veh = new MNM_Veh_Delivery (m_num_veh + 1, timestamp);
  _veh->m_type = veh_type;
  m_veh_map.insert (std::pair<TInt, MNM_Veh *> (m_num_veh + 1, _veh));
  m_num_veh += 1;
  m_enroute += 1;
  m_veh_delivery += 1;
  return _veh;
}

//#################################################################
//                  PQ Link with Delay Releasing
//#################################################################

MNM_Dlink_Pq_Delay::MNM_Dlink_Pq_Delay (TInt ID, TFlt lane_hold_cap,
                                        TFlt lane_flow_cap, TInt number_of_lane,
                                        TFlt length, TFlt ffs, TFlt unit_time,
                                        TFlt flow_scalar)
    : MNM_Dlink_Pq::MNM_Dlink_Pq (ID, lane_hold_cap, lane_flow_cap,
                                  number_of_lane, length, ffs, unit_time,
                                  flow_scalar)
{
  ;
}

MNM_Dlink_Pq_Delay::~MNM_Dlink_Pq_Delay () { ; }

int
MNM_Dlink_Pq_Delay::clear_incoming_array (TInt timestamp)
{
  MNM_Veh *_veh;
  TFlt _to_be_moved = get_link_supply () * m_flow_scalar;
  auto _veh_it = m_incoming_array.begin ();
  while (_veh_it != m_incoming_array.end () && _to_be_moved > 0)
    {
      _veh = (*_veh_it);
      if (dynamic_cast<MNM_DMOND *> (m_from_node) != nullptr)
        {
          if (auto _veh_deliver = dynamic_cast<MNM_Veh_Delivery *> (_veh))
            {
              if (_veh_deliver->m_waiting_time > 0)
                {
                  _veh_deliver->m_waiting_time -= 1;
                  _veh_it++;
                  continue;
                }
              else
                {
                  _veh_deliver->m_waiting_time = TInt (0);
                }
            }
        }
      _veh_it = m_incoming_array.erase (_veh_it);
      // node -> evolve first, then link -> clear_incoming_array(), then link ->
      // evolve() this actually leads to vehicle spending m_max_stamp + 1 in
      // this link
      // so we use m_max_stamp - 1 in link -> evolve() to ensure vehicle spends
      // m_max_stamp in this link when m_max_stamp > 1 and 1 when m_max_stamp =
      // 0
      m_veh_queue.push_back (std::pair<MNM_Veh *, TInt> (_veh, TInt (0)));
      _to_be_moved -= 1;
    }

  m_volume = TInt (m_finished_array.size () + m_veh_queue.size ());
  return 0;
}

//#################################################################
//                  Link Factory with Delivery Traffic
//#################################################################

MNM_Link_Factory_Delivery::MNM_Link_Factory_Delivery ()
    : MNM_Link_Factory::MNM_Link_Factory ()
{
  ;
}

MNM_Link_Factory_Delivery::~MNM_Link_Factory_Delivery () { ; }

MNM_Dlink *
MNM_Link_Factory_Delivery::make_link (TInt ID, DLink_type link_type,
                                      TFlt lane_hold_cap, TFlt lane_flow_cap,
                                      TInt number_of_lane, TFlt length,
                                      TFlt ffs, TFlt unit_time,
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
      _link = new MNM_Dlink_Pq_Delay (ID, lane_hold_cap, lane_flow_cap,
                                      number_of_lane, length, ffs, unit_time,
                                      flow_scalar);
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

//#################################################################
//                  Origin with Delivery Traffic
//#################################################################

MNM_Origin_Delivery::MNM_Origin_Delivery (TInt ID, TInt max_interval,
                                          TFlt flow_scalar, TInt frequency)
    : MNM_Origin::MNM_Origin (ID, max_interval, flow_scalar, frequency)
{
  ;
}

MNM_Origin_Delivery::~MNM_Origin_Delivery ()
{
  for (auto _demand_it : m_demand_multi_OD_seq)
    {
      delete _demand_it.first;
      delete[] _demand_it.second;
    }
  m_demand_multi_OD_seq.clear ();
}

int
MNM_Origin_Delivery::add_multi_OD_seq_demand (
  std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *multi_od_seq,
  TFlt *demand)
{
  double *_demand = new double[m_max_assign_interval]();
  for (int i = 0; i < m_max_assign_interval; ++i)
    {
      _demand[i] = TFlt (demand[i]);
    }
  m_demand_multi_OD_seq.insert (std::make_pair (multi_od_seq, _demand));
  return 0;
}

int
MNM_Origin_Delivery::release_one_interval (TInt current_interval,
                                           MNM_Veh_Factory *veh_factory,
                                           TInt assign_interval,
                                           TFlt adaptive_ratio)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  TInt _veh_to_release, _label;
  MNM_Veh *_veh;
  MNM_Veh_Delivery *_veh_deliver;

  auto _veh_factory_deliver
    = dynamic_cast<MNM_Veh_Factory_Delivery *> (veh_factory);
  Assert (_veh_factory_deliver != nullptr);

  // multiOD vehicle from other origins
  // all properties are already set

  // normal single-OD demand
  for (auto _demand_it = m_demand.begin (); _demand_it != m_demand.end ();
       _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      for (int i = 0; i < _veh_to_release; ++i)
        {
          _label = generate_label (0);
          _veh = _veh_factory_deliver->make_veh (current_interval,
                                                 MNM_Ults::rand_flt ()
                                                     <= adaptive_ratio
                                                   ? MNM_TYPE_ADAPTIVE
                                                   : MNM_TYPE_STATIC);
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = _label;
          // printf("Pushing vehil, %d\n", m_origin_node -> m_node_ID());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }

  // multi-OD demand, e.g., delivery vehicles
  // departing from the first origin
  for (auto _demand_it : m_demand_multi_OD_seq)
    {
      _veh_to_release = TInt (
        MNM_Ults::round ((_demand_it.second)[assign_interval] * m_flow_scalar));
      for (int i = 0; i < _veh_to_release; ++i)
        {
          _label = generate_label (0);
          _veh_deliver
            = _veh_factory_deliver->make_veh_delivery (current_interval,
                                                       MNM_Ults::rand_flt ()
                                                           <= adaptive_ratio
                                                         ? MNM_TYPE_ADAPTIVE
                                                         : MNM_TYPE_STATIC);
          _veh_deliver->set_multi_od_seq (_demand_it.first);
          // first destination
          _veh_deliver->set_destination (_demand_it.first->at (0).second);
          _veh_deliver->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh_deliver->m_assign_interval
            = int (current_interval / m_frequency);
          _veh_deliver->m_label = _label;
          // printf("Pushing vehil, %d\n", m_origin_node -> m_node_ID());
          m_origin_node->m_in_veh_queue.push_back (_veh_deliver);
        }
    }

  std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                       m_origin_node->m_in_veh_queue.end ());
  return 0;
}

//#################################################################
//                  Destination with Delivery Traffic
//#################################################################

MNM_Destination_Delivery::MNM_Destination_Delivery (TInt ID)
    : MNM_Destination::MNM_Destination (ID)
{
  ;
}

MNM_Destination_Delivery::~MNM_Destination_Delivery () { ; }

int
MNM_Destination_Delivery::receive (TInt current_interval)
{
  MNM_Veh *_veh;

  size_t _num_to_receive = m_dest_node->m_out_veh_queue.size ();
  // printf("Dest node %d out vehicle: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _veh = m_dest_node->m_out_veh_queue.front ();
      if (_veh->get_destination () != this)
        {
          printf ("The veh is heading to %d, but we are %d\n",
                  (int) _veh->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error ("Error, MNM_Destination_Delivery::receive, "
                                    "vehicle reaches wrong destination!");
        }
      // printf("Receive Vehicle ID: %d, origin node is %d, destination node is
      // %d\n", _veh -> m_veh_ID(), _veh -> get_origin() -> m_origin_node ->
      // m_node_ID(), _veh -> get_destination() -> m_dest_node -> m_node_ID());
      m_dest_node->m_out_veh_queue.pop_front ();

      if (auto _veh_deliver = dynamic_cast<MNM_Veh_Delivery *> (_veh))
        {
          if (_veh_deliver->m_multi_od_seq != nullptr
              && _veh_deliver->m_multi_od_seq->back ().second
                   != _veh_deliver->get_current_od ().second)
            {
              Assert (_veh_deliver->get_current_od ().second == this);
              _veh_deliver->move_to_next_od ();
              Assert (_veh_deliver->get_current_od ().second != this);
              // delay some time
              _veh_deliver->m_waiting_time
                = dynamic_cast<MNM_Origin_Delivery *> (_veh_deliver->m_origin)
                    ->m_pickup_waiting_time;
              _veh_deliver->m_origin->m_origin_node->m_in_veh_queue.push_back (
                _veh_deliver);
              _veh_deliver->m_path = nullptr;
            }
          else
            {
              _veh_deliver->finish (current_interval);
            }
        }
      else
        {
          _veh->finish (current_interval);
        }
    }

  return 0;
}

int
MNM_Destination_Delivery::receive (TInt current_interval, MNM_Routing *routing,
                                   MNM_Veh_Factory *veh_factory, bool del)
{
  MNM_Veh *_veh;
  size_t _num_to_receive = m_dest_node->m_out_veh_queue.size ();
  // printf("Dest node %d out vehicle: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _veh = m_dest_node->m_out_veh_queue.front ();
      if (_veh->get_destination () != this)
        {
          printf ("The veh is heading to %d, but we are %d\n",
                  (int) _veh->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error ("Error, MNM_Destination_Delivery::receive, "
                                    "vehicle reaches wrong destination!");
        }
      // printf("Receive Vehicle ID: %d, origin node is %d, destination node is
      // %d\n", _veh -> m_veh_ID(), _veh -> get_origin() -> m_origin_node ->
      // m_node_ID(), _veh -> get_destination() -> m_dest_node -> m_node_ID());
      m_dest_node->m_out_veh_queue.pop_front ();

      if (auto _veh_deliver = dynamic_cast<MNM_Veh_Delivery *> (_veh))
        {
          if (_veh_deliver->m_multi_od_seq != nullptr
              && _veh_deliver->m_multi_od_seq->back ().second
                   != _veh_deliver->get_current_od ().second)
            {
              Assert (_veh_deliver->get_current_od ().second == this);
              _veh_deliver->move_to_next_od ();
              Assert (_veh_deliver->get_current_od ().second != this);
              // delay some time
              _veh_deliver->m_waiting_time
                = dynamic_cast<MNM_Origin_Delivery *> (_veh_deliver->m_origin)
                    ->m_pickup_waiting_time;
              _veh_deliver->m_origin->m_origin_node->m_in_veh_queue.push_back (
                _veh_deliver);
              routing
                ->remove_finished (_veh_deliver,
                                   del); // remove STATIC users from m_tracker
              _veh_deliver->m_path = nullptr;
            }
          else
            {
              _veh_deliver->finish (current_interval);
              routing
                ->remove_finished (_veh_deliver,
                                   del); // remove STATIC users from m_tracker
              veh_factory->remove_finished_veh (_veh_deliver, del);
            }
        }
      else
        {
          _veh->finish (current_interval);
          routing->remove_finished (_veh,
                                    del); // remove STATIC users from m_tracker
          veh_factory->remove_finished_veh (_veh, del);
        }
    }
  return 0;
}

//#################################################################
//                  OD Factory with Delivery Traffic
//#################################################################

MNM_OD_Factory_Delivery::MNM_OD_Factory_Delivery ()
    : MNM_OD_Factory::MNM_OD_Factory ()
{
  ;
}

MNM_OD_Factory_Delivery::~MNM_OD_Factory_Delivery () { ; }

MNM_Destination *
MNM_OD_Factory_Delivery::make_destination (TInt ID)
{
  MNM_Destination_Delivery *_dest = new MNM_Destination_Delivery (ID);
  m_destination_map.insert (std::pair<TInt, MNM_Destination *> (ID, _dest));
  return _dest;
}

MNM_Origin *
MNM_OD_Factory_Delivery::make_origin (TInt ID, TInt max_interval,
                                      TFlt flow_scalar, TInt frequency)
{
  MNM_Origin_Delivery *_origin
    = new MNM_Origin_Delivery (ID, max_interval, flow_scalar, frequency);
  m_origin_map.insert (std::pair<TInt, MNM_Origin *> (ID, _origin));
  return _origin;
}

//#################################################################
//                  Fixed Routing with Delivery Traffic
//#################################################################

MNM_Routing_Delivery_Fixed::MNM_Routing_Delivery_Fixed (
  macposts::Graph &graph, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
  TInt routing_frq, TInt buffer_len)
    : MNM_Routing_Fixed::MNM_Routing_Fixed (graph, od_factory, node_factory,
                                            link_factory, routing_frq,
                                            buffer_len)
{
  ;
}

MNM_Routing_Delivery_Fixed::~MNM_Routing_Delivery_Fixed () { ; }

int
MNM_Routing_Delivery_Fixed::remove_finished (MNM_Veh *veh, bool del)
{
  if (auto _veh_deliver = dynamic_cast<MNM_Veh_Delivery *> (veh))
    {
      Assert (_veh_deliver->m_multi_od_seq != nullptr);
    }
  else
    {
      IAssert (veh->m_finish_time > 0
               && veh->m_finish_time > veh->m_start_time);
    }

  if (m_tracker.find (veh) != m_tracker.end () && del)
    {
      Assert (veh->m_type == MNM_TYPE_STATIC); // adaptive user not in m_tracker
      m_tracker.find (veh)->second->clear ();
      delete m_tracker.find (veh)->second;
      m_tracker.erase (veh);
    }
  return 0;
}

//#################################################################
//                  Hybrid Routing with Delivery Traffic
//#################################################################

MNM_Routing_Delivery_Hybrid::MNM_Routing_Delivery_Hybrid (
  const std::string &file_folder, macposts::Graph &graph,
  MNM_Statistics *statistics, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
  TInt route_frq_fixed, TInt buffer_len)
    : MNM_Routing_Hybrid::MNM_Routing_Hybrid (file_folder, graph, statistics,
                                              od_factory, node_factory,
                                              link_factory, route_frq_fixed,
                                              buffer_len)
{
  delete m_routing_fixed;
  m_routing_fixed
    = new MNM_Routing_Delivery_Fixed (graph, od_factory, node_factory,
                                      link_factory, route_frq_fixed,
                                      buffer_len);
}

MNM_Routing_Delivery_Hybrid::~MNM_Routing_Delivery_Hybrid () { ; }

//#################################################################
//                  IO with Delivery Traffic
//#################################################################

int
MNM_IO_Delivery::build_od_factory_delivery (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            MNM_Node_Factory *node_factory,
                                            const std::string &file_name)
{
  auto _od_factory_delivery
    = dynamic_cast<MNM_OD_Factory_Delivery *> (od_factory);
  Assert (_od_factory_delivery != nullptr);

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

  /* build */
  TInt _dest_ID, _origin_ID, _node_ID;
  TFlt _pickup_waiting_time;
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
          // O_ID, node_ID, (pickup_waiting_time)
          if (_words.size () == 2 || _words.size () == 3)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _pickup_waiting_time = 0.;
              if (_words.size () == 3)
                {
                  _pickup_waiting_time = TFlt (std::stod (_words[2])); // second
                }
              else if (_words.size () > 3)
                {
                  throw std::runtime_error (
                    "Error, MNM_IO_Delivery::build_od_factory_delivery, "
                    "MNM_input_od contains wrong origin information");
                }
              _origin
                = _od_factory_delivery->make_origin (_origin_ID, _max_interval,
                                                     _flow_scalar, _frequency);
              dynamic_cast<MNM_Origin_Delivery *> (_origin)
                ->m_pickup_waiting_time
                = MNM_Ults::round (_pickup_waiting_time
                                   / _unit_time); // intervals
              /* hook up */
              _origin->m_origin_node
                = (MNM_DMOND *) node_factory->get_node (_node_ID);
              ((MNM_DMOND *) node_factory->get_node (_node_ID))
                ->hook_up_origin (_origin);
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
              _dest = _od_factory_delivery->make_destination (_dest_ID);
              _dest->m_flow_scalar = _flow_scalar;
              /* hook up */
              _dest->m_dest_node
                = (MNM_DMDND *) node_factory->get_node (_node_ID);
              ((MNM_DMDND *) node_factory->get_node (_node_ID))
                ->hook_up_destination (_dest);
            }
          else
            {
              throw std::runtime_error (
                "Error, MNM_IO_Delivery::build_od_factory_delivery, "
                "MNM_input_od contains wrong destination information");
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO_Delivery::build_od_factory_delivery (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            const std::string &file_name)
{
  auto _od_factory_delivery
    = dynamic_cast<MNM_OD_Factory_Delivery *> (od_factory);
  Assert (_od_factory_delivery != nullptr);

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

  /* build */
  TInt _dest_ID, _origin_ID, _node_ID;
  TFlt _pickup_waiting_time;
  std::string _line;
  std::vector<std::string> _words;
  if (_od_file.is_open ())
    {
      // printf("Start build Origin-Destination factory.\n");
      std::getline (_od_file, _line); // skip the first line
      // printf("Processing Origin node.\n");
      for (int i = 0; i < _num_of_O; ++i)
        {
          std::getline (_od_file, _line);
          _words = split (_line, ' ');
          // O_ID, node_ID, (pickup_waiting_time)
          if (_words.size () == 2 || _words.size () == 3)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _od_factory_delivery->make_origin (_origin_ID, _max_interval,
                                                 _flow_scalar, _frequency);
              if (_words.size () == 3)
                {
                  _pickup_waiting_time = TFlt (std::stod (_words[2])); // second
                  dynamic_cast<MNM_Origin_Delivery *> (
                    _od_factory_delivery->get_origin (_origin_ID))
                    ->m_pickup_waiting_time
                    = MNM_Ults::round (_pickup_waiting_time
                                       / _unit_time); // intervals
                }
              else
                {
                  dynamic_cast<MNM_Origin_Delivery *> (
                    _od_factory_delivery->get_origin (_origin_ID))
                    ->m_pickup_waiting_time
                    = 0;
                }
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
              _od_factory_delivery->make_destination (_dest_ID);
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO_Delivery::build_demand_multi_OD_seq (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            const std::string &od_file_name,
                                            const std::string &demand_file_name)
{
  auto _od_factory_delivery
    = dynamic_cast<MNM_OD_Factory_Delivery *> (od_factory);
  Assert (_od_factory_delivery != nullptr);

  /* find file */
  std::string _od_file_name = file_folder + "/" + od_file_name;
  // std::cout << _od_file_name << std::endl;
  std::ifstream _od_file;
  _od_file.open (_od_file_name, std::ios::in);

  std::string _demand_file_name = file_folder + "/" + demand_file_name;
  // std::cout << _demand_file_name << std::endl;
  std::ifstream _demand_file;
  _demand_file.open (_demand_file_name, std::ios::in);

  /* read config */
  TInt _num_multiOD;
  try
    {
      _num_multiOD = conf_reader->get_int ("multi_OD_seq");
    }
  catch (const std::invalid_argument &ia)
    {
      _num_multiOD = 0;
      printf ("Multi-OD demand file does not exist\n");
      return 0;
    }
  TInt _max_interval = conf_reader->get_int ("max_interval");

  /* build */
  TInt _O_ID, _D_ID;
  MNM_Origin_Delivery *_origin;
  MNM_Destination_Delivery *_dest;
  std::string _line1, _line2;
  std::vector<std::string> _words;
  std::vector<std::pair<MNM_Origin *, MNM_Destination *>> *_od_seq;

  if (_od_file.is_open () && _demand_file.is_open ())
    {
      // printf("Start build multiOD demand profile.\n");
      double *_demand_vector = new double[_max_interval]();

      std::getline (_od_file, _line1);     // skip the first line
      std::getline (_demand_file, _line2); // skip the first line
      for (int i = 0; i < _num_multiOD; ++i)
        {
          std::getline (_od_file, _line1);
          std::getline (_demand_file, _line2);
          // std::cout << "Processing: " << _line1 << "\n";

          // od sequence
          _words = split (_line1, ' ');
          Assert (_words.size () % 2 == 0); // multiple OD pairs
          // destruct in Origin destructor
          _od_seq
            = new std::vector<std::pair<MNM_Origin *, MNM_Destination *>> ();
          for (size_t j = 0; j < _words.size (); j += 2)
            {
              _O_ID = TInt (std::stoi (_words[j]));
              _D_ID = TInt (std::stoi (_words[j + 1]));
              _origin = dynamic_cast<MNM_Origin_Delivery *> (
                _od_factory_delivery->get_origin (_O_ID));
              _dest = dynamic_cast<MNM_Destination_Delivery *> (
                _od_factory_delivery->get_destination (_D_ID));
              _od_seq->push_back (std::make_pair (_origin, _dest));
            }
          _O_ID = TInt (std::stoi (_words[0]));
          _origin = dynamic_cast<MNM_Origin_Delivery *> (
            _od_factory_delivery->get_origin (_O_ID));

          // add demand
          _words = split (_line2, ' ');
          if (TInt (_words.size ()) == (_max_interval))
            {
              for (int j = 0; j < _max_interval; ++j)
                {
                  _demand_vector[j] = TFlt (std::stod (_words[j]));
                }
            }
          else
            {
              delete[] _demand_vector;
              throw std::runtime_error (
                "Error, MNM_IO_Delivery::build_demand_multi_OD_seq, multi-OD "
                "sequence demand's length is NOT equal to max_interval");
            }
          _origin->add_multi_OD_seq_demand (_od_seq, _demand_vector);
        }
      delete[] _demand_vector;
      _od_file.close ();
      _demand_file.close ();
    }
  else
    {
      printf ("Multi-OD seq demand file does not exist\n");
    }
  return 0;
}

//#################################################################
//                  DTA with Delivery Traffic
//#################################################################

MNM_Dta_Delivery::MNM_Dta_Delivery (const std::string &file_folder)
    : MNM_Dta::MNM_Dta (file_folder)
{
  initialize ();
}

MNM_Dta_Delivery::~MNM_Dta_Delivery () { ; }

int
MNM_Dta_Delivery::initialize ()
{
  if (m_veh_factory != nullptr)
    {
      delete m_veh_factory;
      m_veh_factory = nullptr;
    }
  if (m_link_factory != nullptr)
    {
      delete m_link_factory;
      m_link_factory = nullptr;
    }
  if (m_od_factory != nullptr)
    {
      delete m_od_factory;
      m_od_factory = nullptr;
    }
  m_veh_factory = new MNM_Veh_Factory_Delivery ();
  m_link_factory = new MNM_Link_Factory_Delivery ();
  m_od_factory = new MNM_OD_Factory_Delivery ();
  return 0;
}

int
MNM_Dta_Delivery::build_from_files ()
{
  MNM_IO::build_node_factory (m_file_folder, m_config, m_node_factory);
  std::cout << "# of nodes: " << m_node_factory->m_node_map.size ()
            << "\n"; // including normal nodes + charging stations
  MNM_IO::build_link_factory (m_file_folder, m_config, m_link_factory);
  std::cout << "# of links: " << m_link_factory->m_link_map.size () << "\n";
  MNM_IO_Delivery::build_od_factory_delivery (m_file_folder, m_config,
                                              m_od_factory, m_node_factory);
  std::cout << "# of OD pairs: " << m_od_factory->m_origin_map.size () << "\n";

  m_graph = MNM_IO::build_graph (m_file_folder, m_config);

  MNM_IO::build_demand (m_file_folder, m_config, m_od_factory);
  MNM_IO_Delivery::build_demand_multi_OD_seq (m_file_folder, m_config,
                                              m_od_factory);
  MNM_IO::read_origin_vehicle_label_ratio (m_file_folder, m_config,
                                           m_od_factory);
  MNM_IO::build_link_toll (m_file_folder, m_config, m_link_factory);
  build_workzone ();
  set_statistics ();
  set_gridlock_recorder ();
  printf ("Start building routing\n");
  set_routing ();
  printf ("Finish building routing\n");
  return 0;
}

int
MNM_Dta_Delivery::set_routing ()
{
  if (m_config->get_string ("routing_type") == "Hybrid")
    {
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      Path_Table *_path_table;
      if (_tmp_conf->get_string ("choice_portion") == "Buffer")
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       true);
        }
      else
        {
          _path_table
            = MNM_IO::load_path_table (m_file_folder + "/"
                                         + _tmp_conf->get_string (
                                           "path_file_name"),
                                       m_graph, _tmp_conf->get_int ("num_path"),
                                       false);
        }
      TInt _route_freq_fixed = _tmp_conf->get_int ("route_frq");
      TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
      IAssert (_buffer_len == m_config->get_int ("max_interval"));
      m_routing
        = new MNM_Routing_Delivery_Hybrid (m_file_folder, m_graph, m_statistics,
                                           m_od_factory, m_node_factory,
                                           m_link_factory, _route_freq_fixed,
                                           _buffer_len);
      m_routing->init_routing (_path_table);
      delete _tmp_conf;
    }
  else
    {
      throw std::runtime_error (
        "Error, MNM_Dta_Delivery::set_routing, routing_type must be Hybrid");
    }
  return 0;
}
