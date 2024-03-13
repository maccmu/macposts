//
// Created by qiling on 2/18/21.
//

#include "multimodal.h"

static_assert (std::numeric_limits<double>::is_iec559,
               "No iec559 infinity implementation for this compiler!\n");
/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Stop Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                Base Bus Stop
**************************************************************************/
MNM_Busstop::MNM_Busstop (TInt ID, TInt linkID, TFlt linkloc, TFlt flow_scalar)
{
  m_busstop_ID = ID;
  m_link_ID = linkID;
  m_link_loc = linkloc;  // unit: m
  m_cell_ID = TInt (-1); // for CTM link
  m_flow_scalar = flow_scalar;

  m_routing = nullptr;
}

MNM_Busstop::~MNM_Busstop () { ; }

/**************************************************************************
                                                Physical Bus Stop
**************************************************************************/
MNM_Busstop_Physical::MNM_Busstop_Physical (TInt ID, TInt linkID, TFlt linkloc,
                                            TFlt flow_scalar)
    : MNM_Busstop::MNM_Busstop (ID, linkID, linkloc, flow_scalar)
{
  m_route_IDs_vec = std::vector<TInt> ();
  m_busstop_virtual_vec = std::vector<MNM_Busstop_Virtual *> ();
  m_boarding_links_vec = std::vector<MNM_Walking_Link *> ();
  m_alighting_links_vec = std::vector<MNM_Walking_Link *> ();
  m_walking_in_links_vec = std::vector<MNM_Walking_Link *> ();
  m_walking_out_links_vec = std::vector<MNM_Walking_Link *> ();
}

MNM_Busstop_Physical::~MNM_Busstop_Physical ()
{
  m_route_IDs_vec.clear ();
  m_busstop_virtual_vec.clear ();
  m_boarding_links_vec.clear ();
  m_alighting_links_vec.clear ();
  m_walking_in_links_vec.clear ();
  m_walking_out_links_vec.clear ();
}

int
MNM_Busstop_Physical::evolve (TInt timestamp)
{
  // for (auto _link : m_walking_out_links_vec) {
  //     if (_link -> m_N_in != nullptr) {
  //         _link -> m_N_in -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }
  // for (auto _link : m_walking_in_links_vec) {
  //     if (_link -> m_N_out != nullptr) {
  //         _link -> m_N_out -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }
  // for (auto _link : m_boarding_links_vec) {
  //     if (_link -> m_N_in != nullptr) {
  //         _link -> m_N_in -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }
  // for (auto _link : m_alighting_links_vec) {
  //     if (_link -> m_N_out != nullptr) {
  //         _link -> m_N_out -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }

  // move passenger from walking link to walking or boarding link
  MNM_Passenger *_passenger;
  MNM_Walking_Link *_out_walking_link;
  for (auto _in_walking_link : m_walking_in_links_vec)
    {
      auto _passenger_it = _in_walking_link->m_finished_array.begin ();
      while (_passenger_it != _in_walking_link->m_finished_array.end ())
        {
          _passenger = *_passenger_it;
          if (_passenger->get_next_link ()->m_link_type
              != MNM_TYPE_WALKING_MULTIMODAL)
            {
              throw std::runtime_error ("Next link should be walking link");
            }
          _out_walking_link
            = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
          IAssert (_out_walking_link != nullptr);
          if (std::find (m_walking_out_links_vec.begin (),
                         m_walking_out_links_vec.end (), _out_walking_link)
                == m_walking_out_links_vec.end ()
              && std::find (m_boarding_links_vec.begin (),
                            m_boarding_links_vec.end (), _out_walking_link)
                   == m_boarding_links_vec.end ())
            {
              throw std::runtime_error ("Something wrong in passenger routing");
            }
          if (_out_walking_link->m_walking_type == "boarding")
            {
              _out_walking_link->m_finished_array.push_back (_passenger);
            }
          else
            {
              _out_walking_link->m_incoming_array.push_back (_passenger);
            }
          _passenger->set_current_link (_out_walking_link);
          _passenger_it
            = _in_walking_link->m_finished_array.erase (_passenger_it);
          if (_in_walking_link->m_N_out != nullptr)
            {
              // _in_walking_link -> m_N_out -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
              // passenger
              _in_walking_link->m_N_out->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / m_flow_scalar)));
            }
          if (_out_walking_link->m_N_in != nullptr)
            {
              // _out_walking_link -> m_N_in -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
              // passenger
              _out_walking_link->m_N_in->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / m_flow_scalar)));
            }
          if (_in_walking_link->m_N_out_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _in_walking_link -> m_N_out_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _in_walking_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _in_walking_link -> m_N_out_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_transit_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _in_walking_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
          if (_out_walking_link->m_N_in_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _out_walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _out_walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _out_walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_transit_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _out_walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
        }
    }
  // move passenger from alighting link to walking or boarding link
  for (auto _in_walking_link : m_alighting_links_vec)
    {
      auto _passenger_it = _in_walking_link->m_finished_array.begin ();
      while (_passenger_it != _in_walking_link->m_finished_array.end ())
        {
          _passenger = *_passenger_it;
          if (_passenger->get_next_link ()->m_link_type
              != MNM_TYPE_WALKING_MULTIMODAL)
            {
              throw std::runtime_error ("Next link should be walking link");
            }
          _out_walking_link
            = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
          IAssert (_out_walking_link != nullptr);
          if (std::find (m_walking_out_links_vec.begin (),
                         m_walking_out_links_vec.end (), _out_walking_link)
                == m_walking_out_links_vec.end ()
              && std::find (m_boarding_links_vec.begin (),
                            m_boarding_links_vec.end (), _out_walking_link)
                   == m_boarding_links_vec.end ())
            {
              throw std::runtime_error ("Something wrong in passenger routing");
            }
          if (_out_walking_link->m_walking_type == "boarding")
            {
              _out_walking_link->m_finished_array.push_back (_passenger);
            }
          else
            {
              _out_walking_link->m_incoming_array.push_back (_passenger);
            }
          _passenger->set_current_link (_out_walking_link);
          _passenger_it
            = _in_walking_link->m_finished_array.erase (_passenger_it);
          if (_in_walking_link->m_N_out != nullptr)
            {
              // _in_walking_link -> m_N_out -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
              // passenger
              _in_walking_link->m_N_out->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / m_flow_scalar)));
              if (MNM_Ults::approximate_less_than (_in_walking_link->m_N_in
                                                     ->m_recorder.back ()
                                                     .second,
                                                   _in_walking_link->m_N_out
                                                     ->m_recorder.back ()
                                                     .second))
                {
                  throw std::runtime_error ("Debug alighting link cc");
                }
            }
          if (_out_walking_link->m_N_in != nullptr)
            {
              // _out_walking_link -> m_N_in -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
              // passenger
              _out_walking_link->m_N_in->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / m_flow_scalar)));
            }
          if (_in_walking_link->m_N_out_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _in_walking_link -> m_N_out_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _in_walking_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _in_walking_link -> m_N_out_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_transit_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _in_walking_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
          if (_out_walking_link->m_N_in_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _out_walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _out_walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _out_walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                  // + 1), TFlt(1), _passenger -> m_transit_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _out_walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1), TFlt (1 / m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
        }
    }

  return 0;
}

/**************************************************************************
                                                Virtual Bus Stop
**************************************************************************/
MNM_Busstop_Virtual::MNM_Busstop_Virtual (TInt ID, TInt linkID, TFlt linkloc,
                                          TFlt flow_scalar)
    : MNM_Busstop::MNM_Busstop (ID, linkID, linkloc, flow_scalar)
{
  m_route_ID = -1;
  m_busstop_physical = nullptr;
  m_bus_in_link = nullptr;
  m_bus_out_link = nullptr;
  m_boarding_link = nullptr;
  m_alighting_link = nullptr;

  m_passed_bus_counter = TInt (0);
  m_bus_queue = std::deque<MNM_Veh_Multimodal *> ();

  m_N_in_bus = nullptr;
  m_N_out_bus = nullptr;

  install_cumulative_curve_multiclass ();
}

MNM_Busstop_Virtual::~MNM_Busstop_Virtual ()
{
  m_bus_queue.clear ();

  delete m_N_in_bus;

  delete m_N_out_bus;
}

int
MNM_Busstop_Virtual::install_cumulative_curve_multiclass ()
{
  if (m_N_in_bus != nullptr)
    delete m_N_in_bus;
  if (m_N_out_bus != nullptr)
    delete m_N_out_bus;

  m_N_in_bus = new MNM_Cumulative_Curve ();
  m_N_out_bus = new MNM_Cumulative_Curve ();
  m_N_in_bus->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  m_N_out_bus->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  return 0;
}

bool
MNM_Busstop_Virtual::hold_bus (MNM_Veh *veh, MNM_Veh_Multimodal *veh_multimodal,
                               std::deque<MNM_Veh *> *from_queue,
                               std::deque<MNM_Veh *> *held_queue,
                               int flow_scalar)
{
  IAssert (veh->m_veh_ID == veh_multimodal->m_veh_ID
           && veh_multimodal->m_class == 1
           && veh_multimodal->m_bus_route_ID != -1
           && veh_multimodal->m_bus_route_ID == m_route_ID);
  if (std::find (m_bus_queue.begin (), m_bus_queue.end (), veh_multimodal)
      == m_bus_queue.end ())
    {
      throw std::runtime_error (
        "Bus not captured in the m_bus_queue of the bus stop");
    }
  bool _held = false;
  TInt _num_boarding_passengers = 0;
  TInt _num_alighting_passengers = 0;
  TInt _min_dwell_intervals = veh_multimodal->m_min_dwell_intervals;
  MNM_Walking_Link *_walking_link;
  MNM_Bus_Link *_bus_link;

  if (veh_multimodal->m_stopped_intervals > 0)
    {
      if (m_passed_bus_counter != flow_scalar)
        {
          throw std::runtime_error (
            "Something is wrong with m_passed_bus_counter or "
            "veh_multimodal -> m_stopped_intervals\n");
        }
    }

  // only stop bus every m_flow_scalar vehicles
  if ((m_passed_bus_counter < flow_scalar - 1)
      && ((veh_multimodal->m_stopped_intervals == 0)))
    {
      // IAssert(veh_multimodal -> m_passenger_pool.size() == 0);
      m_passed_bus_counter += 1;
      return _held;
    }

  // when there is a bus in boarding and alighting, hold the incoming bus
  // afterwards (virtual bus does not board and alight)
  if ((m_passed_bus_counter == flow_scalar)
      && ((veh_multimodal->m_stopped_intervals == 0)))
    {
      // there exists a stopped bus beforehand, hold the bus, move the bus to a
      // bay if (!veh_multimodal -> m_passenger_pool.empty()){
      //     printf("virtual bus should not have passengers on board\n");
      //     exit(-1);
      // }
      from_queue->pop_front ();
      held_queue->push_back (veh);
      _held = true;
      return _held;
    }

  // any alighting passengers?
  for (auto _passenger : veh_multimodal->m_passenger_pool)
    {
      if (_passenger->get_next_link ()->m_link_type
          == MNM_TYPE_WALKING_MULTIMODAL)
        {
          _walking_link
            = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
        }
      else
        {
          if (_passenger->get_next_link ()->m_link_ID
              != m_bus_out_link->m_link_ID)
            {
              throw std::runtime_error ("MNM_Busstop_Virtual::hold_bus, Debug");
            }
          continue;
        }
      if (_walking_link != nullptr
          && _walking_link->m_walking_type == "alighting"
          && m_alighting_link == _walking_link)
        {
          _num_alighting_passengers += 1;
        }
    }
  // any remaining boarding capacity?
  // TInt _remaining_capacity = veh_multimodal -> m_capacity -
  // (int)veh_multimodal -> m_passenger_pool.size() + _num_alighting_passengers;
  // add flow_scalar to passenger
  TInt _remaining_capacity = veh_multimodal->m_capacity * flow_scalar
                             - (int) veh_multimodal->m_passenger_pool.size ()
                             + _num_alighting_passengers;
  // any boarding passengers?
  if (_remaining_capacity > 0 && m_boarding_link != nullptr)
    {
      for (auto _passenger : m_boarding_link->m_finished_array)
        {
          _bus_link
            = dynamic_cast<MNM_Bus_Link *> (_passenger->get_next_link ());
          if (_bus_link != nullptr
              && _bus_link->m_route_ID == veh_multimodal->m_bus_route_ID)
            {
              IAssert (_bus_link->m_link_ID == m_bus_out_link->m_link_ID);
              _num_boarding_passengers += 1;
            }
        }
    }
  else
    {
      // printf("Bus has no available capacity!\n");
    }

  if (_min_dwell_intervals > 0
      && _min_dwell_intervals < veh_multimodal->m_boarding_lost_intervals
      && (_num_alighting_passengers > 0
          || (_num_boarding_passengers > 0 && _remaining_capacity > 0)))
    {
      _min_dwell_intervals = veh_multimodal->m_boarding_lost_intervals;
    }

  // no remaining capacity for arriving bus
  if (_num_alighting_passengers == 0 && _remaining_capacity == 0
      && veh_multimodal->m_stopped_intervals == 0)
    {
      _min_dwell_intervals = 0;
    }

  // update bus counter
  if (veh_multimodal->m_stopped_intervals == 0)
    {
      // just arriving
      // IAssert(m_passed_bus_counter == flow_scalar - 1);
      if (m_passed_bus_counter != flow_scalar - 1)
        {
          throw std::runtime_error ("m_passed_bus_counter is wrong");
        }
      m_passed_bus_counter += 1;
    }
  else
    {
      // already stopped
      // IAssert(m_passed_bus_counter == flow_scalar);
      if (m_passed_bus_counter != flow_scalar)
        {
          throw std::runtime_error ("m_passed_bus_counter is wrong");
        }
    }

  if (veh_multimodal->m_stopped_intervals < _min_dwell_intervals
      || _num_alighting_passengers > 0
      || (_num_boarding_passengers > 0 && _remaining_capacity > 0))
    {
      // hold the bus, move the bus to a different location of queue
      from_queue->pop_front ();
      held_queue->push_back (veh);
      veh_multimodal->m_stopped_intervals += 1;
      _held = true;
      return _held;
    }
  else
    {
      IAssert (veh_multimodal->m_stopped_intervals >= _min_dwell_intervals);
      IAssert (m_passed_bus_counter == flow_scalar);
      // reset bus counter
      m_passed_bus_counter = 0;
      return _held;
    }

  // stop every bus
  // if (veh_multimodal -> m_stopped_intervals < _stopped_intervals) {
  //     // hold the bus, move the bus to a different location of queue
  //     from_queue->pop_front();
  //     _held_queue.push_back(_veh);
  //     veh_multimodal->m_stopped_intervals += 1;
  //     _held = true;
  //     return _held;
  // }
  return _held;
}

int
MNM_Busstop_Virtual::release_bus (TInt timestamp,
                                  MNM_Veh_Multimodal *veh_multimodal)
{
  IAssert (veh_multimodal->m_class == 1
           && veh_multimodal->m_bus_route_ID != TInt (-1)
           && veh_multimodal->m_bus_route_ID == m_route_ID);
  // update current link for passengers on board
  if (m_bus_out_link != nullptr)
    {
      for (auto _passenger : veh_multimodal->m_passenger_pool)
        {
          if (_passenger->get_next_link ()->m_link_ID
              != m_bus_out_link->m_link_ID)
            {
              throw std::runtime_error ("Passenger in wrong bus");
            }
          if (_passenger->get_current_link ()->m_link_ID
              != m_bus_out_link->m_link_ID)
            {
              IAssert (m_bus_in_link != nullptr
                       && _passenger->get_current_link ()->m_link_ID
                            == m_bus_in_link->m_link_ID);

              // update cc for passengers already on board before this bus stop
              if (m_bus_out_link->m_N_in != nullptr)
                {
                  // m_bus_out_link -> m_N_in -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_bus_out_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (m_bus_in_link->m_N_out != nullptr)
                {
                  // m_bus_in_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_bus_in_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }

              // update cc tree for passengers already on board before this bus
              // stop
              if (m_bus_out_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_bus_out_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_out_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_bus_out_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_out_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (m_bus_in_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }

              _passenger->set_current_link (m_bus_out_link);
            }
          if (_passenger->get_current_link ()->m_link_ID
              != m_bus_out_link->m_link_ID)
            {
              throw std::runtime_error (
                "MNM_Busstop_Virtual::release_bus, passenger in wrong bus");
            }
        }
    }
  else
    {
      if (!veh_multimodal->m_passenger_pool.empty ())
        {
          throw std::runtime_error ("Bus is NOT empty, all passengers should "
                                    "get off bus at the last bus stop");
        }
    }

  // update cc for bus
  if (m_N_out_bus != nullptr)
    {
      m_N_out_bus->add_increment (
        std::pair<TFlt, TFlt> (TFlt (timestamp + 1), TFlt (1) / m_flow_scalar));
    }
  if (m_bus_in_link != nullptr && m_bus_in_link->m_N_out_tree_bus != nullptr)
    {
      m_bus_in_link->m_N_out_tree_bus
        ->add_flow (TFlt (timestamp + 1), TFlt (1) / m_flow_scalar,
                    veh_multimodal->m_path, veh_multimodal->m_assign_interval);
    }
  if (m_bus_out_link != nullptr && m_bus_out_link->m_N_in_tree_bus != nullptr)
    {
      m_bus_out_link->m_N_in_tree_bus
        ->add_flow (TFlt (timestamp + 1), TFlt (1) / m_flow_scalar,
                    veh_multimodal->m_path, veh_multimodal->m_assign_interval);
    }
  // remove bus from queue at bus stop
  IAssert (std::find (m_bus_queue.begin (), m_bus_queue.end (), veh_multimodal)
           != m_bus_queue.end ());
  bool _flg = false;
  auto _veh_multimodal_it = m_bus_queue.begin ();
  while (_veh_multimodal_it != m_bus_queue.end ())
    {
      if (*_veh_multimodal_it == veh_multimodal)
        {
          _veh_multimodal_it = m_bus_queue.erase (_veh_multimodal_it);
          _flg = true;
          break;
        }
      else
        {
          _veh_multimodal_it++;
        }
    }
  if (!_flg)
    {
      throw std::runtime_error ("Bus not in m_bus_queue!");
    }
  return 0;
}

int
MNM_Busstop_Virtual::receive_bus (TInt timestamp,
                                  MNM_Veh_Multimodal *veh_multimodal)
{
  IAssert (veh_multimodal->m_class == 1
           && veh_multimodal->m_bus_route_ID != TInt (-1)
           && veh_multimodal->m_bus_route_ID == m_route_ID);
  // update cc for bus
  if (m_N_in_bus != nullptr)
    {
      m_N_in_bus->add_increment (
        std::pair<TFlt, TFlt> (TFlt (timestamp + 1), TFlt (1) / m_flow_scalar));
    }
  m_bus_queue.push_back (veh_multimodal);
  update_routing_passenger (timestamp);

  // // update cc for alighting link, it has some issues which cause the out
  // link cc > in link cc MNM_Walking_Link *_walking_link; TInt
  // _num_alighting_passengers = 0; if (veh_multimodal -> m_stopped_intervals ==
  // 0 && m_passed_bus_counter == (int)m_flow_scalar - 1) {
  //     // update_routing_passenger(timestamp);
  //     // just arriving
  //     // any alighting passengers?

  //     for (auto _passenger : veh_multimodal -> m_passenger_pool) {
  //         if (_passenger -> get_next_link() -> m_link_type ==
  //         MNM_TYPE_WALKING_MULTIMODAL) {
  //             _walking_link = dynamic_cast<MNM_Walking_Link*>(_passenger ->
  //             get_next_link());
  //         } else {
  //             if (_passenger -> get_next_link() -> m_link_ID !=
  //             m_bus_out_link -> m_link_ID) {
  //                 printf("MNM_Busstop_Virtual::receive_bus, Debug");
  //                 exit(-1);
  //             }
  //             continue;
  //         }
  //         if (_walking_link != nullptr && _walking_link -> m_walking_type ==
  //         "alighting" &&
  //             m_alighting_link == _walking_link) {
  //             _num_alighting_passengers += 1;

  //             if (_num_alighting_passengers == 1 && _walking_link ->
  //             m_link_ID == 25 && _walking_link -> m_link_ID == 29 &&
  //             _walking_link -> m_N_in -> m_recorder.back().second > 0 &&
  //                 MNM_Ults::approximate_equal(_walking_link -> m_N_in ->
  //                 m_recorder.back().second, _walking_link -> m_N_out ->
  //                 m_recorder.back().second)) { printf("Debug walking cc\n");
  //             }

  //             // update cc tree for passengers about to alight at this bus
  //             stop if (m_bus_in_link != nullptr && m_bus_in_link ->
  //             m_N_out_tree != nullptr) {
  //                 if (_passenger -> m_pnr) {
  //                     m_bus_in_link -> m_N_out_tree ->
  //                     add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
  //                     m_pnr_path, _passenger -> m_assign_interval);
  //                 } else {
  //                     m_bus_in_link -> m_N_out_tree ->
  //                     add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
  //                     m_transit_path, _passenger -> m_assign_interval);
  //                 }
  //             }
  //             if (m_alighting_link -> m_N_in_tree != nullptr) {
  //                 if (_passenger -> m_pnr) {
  //                     m_alighting_link -> m_N_in_tree ->
  //                     add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
  //                     m_pnr_path, _passenger -> m_assign_interval);
  //                 } else {
  //                     m_alighting_link -> m_N_in_tree ->
  //                     add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
  //                     m_transit_path, _passenger -> m_assign_interval);
  //                 }
  //             }
  //         }
  //     }
  // }
  // // update cc for passengers about to alight at this bus stop
  // if (m_bus_in_link != nullptr && m_bus_in_link -> m_N_out != nullptr &&
  // _num_alighting_passengers > 0) {
  //     m_bus_in_link -> m_N_out -> add_increment(std::pair<TFlt,
  //     TFlt>(TFlt(timestamp+1), TFlt(_num_alighting_passengers)));
  // }
  // if (m_alighting_link != nullptr && m_alighting_link -> m_N_in != nullptr &&
  // _num_alighting_passengers > 0) {
  //     m_alighting_link -> m_N_in -> add_increment(std::pair<TFlt,
  //     TFlt>(TFlt(timestamp+1), TFlt(_num_alighting_passengers)));
  // }
  return 0;
}

int
MNM_Busstop_Virtual::update_routing_passenger (TInt timestamp)
{
  IAssert (m_routing != nullptr);
  m_routing->m_routing_passenger_fixed->update_routing_one_busstop (timestamp,
                                                                    this);
  if (m_routing->m_routing_multimodal_adaptive != nullptr)
    {
      m_routing->m_routing_multimodal_adaptive
        ->update_routing_passenger_one_busstop (timestamp, this);
    }
  return 0;
}

TFlt
MNM_Busstop_Virtual::get_waiting_time_snapshot (TInt timestamp)
{
  // waiting time
  // assume cc changes only when additional vehicles pass
  TFlt _cc = m_N_in_bus->get_result (TFlt (timestamp));

  if (!MNM_Ults::approximate_less_than (_cc, m_total_bus))
    {
      // return std::numeric_limits<double>::infinity();
      return 2 * m_origin->m_max_assign_interval * m_origin->m_frequency;
    }
  IAssert (MNM_Ults::approximate_less_than (_cc, m_total_bus));

  TFlt _tt = TFlt (0);
  TFlt _driving_link_tt;

  int _cc_floor = int (_cc);
  int _cc_ceil = _cc_floor + 1;

  TFlt _threshold;
  if (MNM_Ults::approximate_equal (_cc, TFlt (_cc_ceil)))
    {
      _threshold = _cc_ceil + 1;
    }
  else
    {
      _threshold = _cc_ceil;
    }
  MNM_Busstop_Virtual *_busstop = this;
  // check if next bus has reached any upstream bus stop
  while (_busstop->m_bus_in_link != nullptr)
    {
      // bus time for upstream bus links
      for (size_t i = 0;
           i < _busstop->m_bus_in_link->m_overlapped_driving_link_vec.size ();
           ++i)
        {
          // TODO: get_link_tt() only for car, not accurate for truck
          _driving_link_tt
            = _busstop->m_bus_in_link->m_overlapped_driving_link_vec[i]
                ->get_link_tt ();
          _tt += _driving_link_tt
                 * _busstop->m_bus_in_link
                     ->m_overlapped_driving_link_length_portion_vec[i];
        }
      if (MNM_Ults::approximate_less_than (_busstop->m_bus_in_link
                                             ->m_from_busstop->m_N_in_bus
                                             ->get_result (TFlt (timestamp)),
                                           _threshold))
        {
          _busstop = _busstop->m_bus_in_link->m_from_busstop;
        }
      else
        {
          break;
        }
    }
  // bus has not been released yet
  if (_busstop->m_bus_in_link == nullptr)
    {
      TFlt *_demand = _busstop->m_origin->m_demand_bus.find (_busstop->m_dest)
                        ->second.find (m_route_ID)
                        ->second;
      // m_origin -> m_frequency is 180 5-s interval, not 15 1-min interval
      TInt _num_minute = (int) _busstop->m_origin->m_frequency / (60 / 5);
      TInt _current_release_interval = (int) timestamp / (60 / 5);
      TFlt _tot_released_bus = 0;
      for (int i = 0; i < _current_release_interval + 1; i++)
        {
          _tot_released_bus
            += floor (_demand[i] * _busstop->m_origin->m_flow_scalar)
               / _busstop->m_origin->m_flow_scalar;
        }
      if (MNM_Ults::approximate_less_than (_tot_released_bus, _threshold))
        {
          for (int i = _current_release_interval + 1;
               i < _busstop->m_origin->m_max_assign_interval * _num_minute; i++)
            {
              _tot_released_bus
                += floor (_demand[i] * _busstop->m_origin->m_flow_scalar)
                   / _busstop->m_origin->m_flow_scalar;
              if (!MNM_Ults::approximate_less_than (_tot_released_bus,
                                                    _threshold))
                {
                  // min -> s
                  _tt += (int) (i - _current_release_interval) * 60;
                  break;
                }
            }
        }
    }
  return _tt;
}

int
MNM_Busstop_Virtual::virtual_evolve (TInt timestamp)
{
  MNM_Passenger *_passenger;
  TInt _alighting_counter = 0, _boarding_counter = 0;
  if (m_bus_in_link != nullptr)
    {
      auto _passenger_it = m_bus_in_link->m_finished_array.begin ();
      while (_passenger_it != m_bus_in_link->m_finished_array.end ())
        {
          _passenger = *_passenger_it;
          if (_passenger->get_next_link ()->m_link_type
              == MNM_TYPE_BUS_MULTIMODAL)
            {
              // continue riding
              IAssert (m_bus_out_link != nullptr);
              IAssert (m_bus_out_link->m_link_ID
                       == _passenger->get_next_link ()->m_link_ID);
              auto *_out_bus_link
                = dynamic_cast<MNM_Bus_Link *> (_passenger->get_next_link ());
              _out_bus_link->m_incoming_array.push_back (_passenger);
              _passenger->set_current_link (_out_bus_link);
              _passenger_it
                = m_bus_in_link->m_finished_array.erase (_passenger_it);
              if (m_bus_in_link->m_N_out != nullptr)
                {
                  // m_bus_in_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_bus_in_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (_out_bus_link->m_N_in != nullptr)
                {
                  // _out_bus_link -> m_N_in -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  _out_bus_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (m_bus_in_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (_out_bus_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _out_bus_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                      // m_assign_interval); add flow_scalar to passenger
                      _out_bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _out_bus_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_transit_path, _passenger
                      // -> m_assign_interval); add flow_scalar to passenger
                      _out_bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
            }
          else if (_passenger->get_next_link ()->m_link_type
                   == MNM_TYPE_WALKING_MULTIMODAL)
            {
              // alighting
              IAssert (m_alighting_link != nullptr);
              IAssert (m_alighting_link->m_link_ID
                       == _passenger->get_next_link ()->m_link_ID);
              if (_alighting_counter >= m_max_alighting_passengers_per_unit_time
                                          * int (m_flow_scalar))
                {
                  break;
                }
              auto *_out_walking_link = dynamic_cast<MNM_Walking_Link *> (
                _passenger->get_next_link ());
              _out_walking_link->m_finished_array.push_back (_passenger);
              _passenger->set_current_link (_out_walking_link);
              _passenger_it
                = m_bus_in_link->m_finished_array.erase (_passenger_it);
              _alighting_counter += 1;
              if (m_bus_in_link->m_N_out != nullptr)
                {
                  // m_bus_in_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_bus_in_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (_out_walking_link->m_N_in != nullptr)
                {
                  // _out_walking_link -> m_N_in ->
                  // add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1),
                  // TFlt(1))); add flow_scalar to passenger
                  _out_walking_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (m_bus_in_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_bus_in_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_bus_in_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (_out_walking_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _out_walking_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      _out_walking_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _out_walking_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      _out_walking_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
            }
          else
            {
              throw std::runtime_error (
                "MNM_Busstop_Virtual::virtual_evolve, wrong link type!");
            }
        }
    }

  if (m_boarding_link != nullptr)
    {
      auto _passenger_it = m_boarding_link->m_finished_array.begin ();
      while (_passenger_it != m_boarding_link->m_finished_array.end ())
        {
          _passenger = *_passenger_it;
          if (_passenger->get_next_link ()->m_link_type
              == MNM_TYPE_BUS_MULTIMODAL)
            {
              // boarding
              IAssert (m_bus_out_link != nullptr);
              IAssert (m_bus_out_link->m_link_ID
                       == _passenger->get_next_link ()->m_link_ID);
              if (_boarding_counter >= m_max_boarding_passengers_per_unit_time
                                         * int (m_flow_scalar))
                {
                  break;
                }
              auto *_out_bus_link
                = dynamic_cast<MNM_Bus_Link *> (_passenger->get_next_link ());
              // waiting for bus is done in boarding link evolve()
              _out_bus_link->m_incoming_array.push_back (_passenger);
              _passenger->set_current_link (_out_bus_link);
              _passenger_it
                = m_boarding_link->m_finished_array.erase (_passenger_it);
              _boarding_counter += 1;
              if (m_boarding_link->m_N_out != nullptr)
                {
                  // m_boarding_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_boarding_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (_out_bus_link->m_N_in != nullptr)
                {
                  // _out_bus_link -> m_N_in -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  _out_bus_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (m_boarding_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_boarding_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_boarding_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_boarding_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_boarding_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (_out_bus_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _out_bus_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                      // m_assign_interval); add flow_scalar to passenger
                      _out_bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _out_bus_link -> m_N_in_tree -> add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_transit_path, _passenger
                      // -> m_assign_interval); add flow_scalar to passenger
                      _out_bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
            }
          else if (_passenger->get_next_link ()->m_link_type
                   == MNM_TYPE_WALKING_MULTIMODAL)
            {
              // to alighting link, which is unlikely in real world, but maay
              // happen in simulation due to routing algorithm
              IAssert (m_alighting_link != nullptr);
              IAssert (m_alighting_link->m_link_ID
                       == _passenger->get_next_link ()->m_link_ID);
              auto *_out_walking_link = dynamic_cast<MNM_Walking_Link *> (
                _passenger->get_next_link ());
              _out_walking_link->m_finished_array.push_back (_passenger);
              _passenger->set_current_link (_out_walking_link);
              _passenger_it
                = m_boarding_link->m_finished_array.erase (_passenger_it);
              if (m_boarding_link->m_N_out != nullptr)
                {
                  // m_boarding_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  m_boarding_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (_out_walking_link->m_N_in != nullptr)
                {
                  // _out_walking_link -> m_N_in ->
                  // add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1),
                  // TFlt(1))); add flow_scalar to passenger
                  _out_walking_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
              if (m_boarding_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // m_boarding_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_boarding_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // m_boarding_link -> m_N_out_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      m_boarding_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (_out_walking_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _out_walking_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_pnr_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      _out_walking_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _out_walking_link -> m_N_in_tree ->
                      // add_flow(TFlt(timestamp + 1), TFlt(1), _passenger ->
                      // m_transit_path, _passenger -> m_assign_interval); add
                      // flow_scalar to passenger
                      _out_walking_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
            }
          else
            {
              throw std::runtime_error (
                "MNM_Busstop_Virtual::virtual_evolve, wrong link type!");
            }
        }
    }
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Stop Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Busstop_Factory::MNM_Busstop_Factory ()
{
  m_busstop_map = std::unordered_map<TInt, MNM_Busstop *> ();
}

MNM_Busstop_Factory::~MNM_Busstop_Factory ()
{
  for (auto _map_it = m_busstop_map.begin (); _map_it != m_busstop_map.end ();
       _map_it++)
    {
      delete _map_it->second;
    }
  m_busstop_map.clear ();
}

MNM_Busstop *
MNM_Busstop_Factory::make_busstop (TInt ID, TInt linkID, TFlt linkloc,
                                   TFlt flow_scalar,
                                   const std::string &busstop_type)
{
  MNM_Busstop *_busstop;
  if (busstop_type == "physical")
    {
      _busstop = new MNM_Busstop_Physical (ID, linkID, linkloc, flow_scalar);
    }
  else if (busstop_type == "virtual")
    {
      _busstop = new MNM_Busstop_Virtual (ID, linkID, linkloc, flow_scalar);
    }
  else
    {
      throw std::runtime_error ("Wrong bus stop type");
    }
  m_busstop_map.insert (std::pair<TInt, MNM_Busstop *> (ID, _busstop));
  return _busstop;
}

MNM_Busstop *
MNM_Busstop_Factory::get_busstop (TInt ID)
{
  auto _busstop_it = m_busstop_map.find (ID);
  if (_busstop_it == m_busstop_map.end ())
    {
      throw std::runtime_error (
        "Error, MNM_Busstop_Factory::get_busstop, busstop does not exist");
    }
  return _busstop_it->second;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Parking lot
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Parking_Lot::MNM_Parking_Lot (TInt ID, TInt node_ID,
                                  MNM_Passenger_Factory *passenger_factory,
                                  MNM_Veh_Factory *veh_factory, TFlt base_price,
                                  TFlt price_surge_coeff, TFlt avg_parking_time,
                                  TFlt capacity, TFlt unit_time)
{
  m_ID = ID;
  m_base_price = base_price;
  m_price_surge_coeff = price_surge_coeff;
  m_avg_parking_time = avg_parking_time;        // intervals
  m_max_parking_time = 10 * m_avg_parking_time; // intervals
  m_unit_time = unit_time;
  m_occupancy = 0;
  m_capacity = capacity;

  m_veh_factory = veh_factory;
  m_passenger_factory = passenger_factory;

  m_node_ID = node_ID;
  m_dest_node = nullptr; // set in building_parking_lot

  m_walking_out_links_vec = std::vector<MNM_Walking_Link *> ();

  m_cruising_time_record = std::unordered_map<TInt, TFlt> ();
  m_cruising_time_record.insert (
    std::pair<TInt, TFlt> (0, avg_parking_time)); // intervals

  m_in_passenger_queue = std::deque<MNM_Passenger *> ();
  // m_parked_car_queue = std::deque<MNM_Veh_Multimodal*>();
  m_parked_car = TInt (0);
  m_pnr_fixed_routing_counter = std::unordered_map<TInt, TInt> ();
  // for adaptive users
  m_pnr_fixed_routing_counter.insert (std::pair<TInt, TInt> (-1, 0));
}

MNM_Parking_Lot::~MNM_Parking_Lot ()
{
  m_walking_out_links_vec.clear ();
  m_cruising_time_record.clear ();
  m_in_passenger_queue.clear ();
  // m_parked_car_queue.clear();
  m_pnr_fixed_routing_counter.clear ();
}

TFlt
MNM_Parking_Lot::get_cruise_time (TInt timestamp)
{
  if (m_cruising_time_record.find (timestamp) != m_cruising_time_record.end ())
    {
      return m_cruising_time_record.find (timestamp)->second;
    }
  else if ((int) m_cruising_time_record.size () - 1 < timestamp)
    {
      return m_cruising_time_record
        .find (TInt (m_cruising_time_record.size () - 1))
        ->second;
    }
  else
    {
      throw std::runtime_error ("MNM_Parking_Lot::get_cruise_time, record does "
                                "not exist in m_cruising_time_record!");
    }
}

int
MNM_Parking_Lot::release_one_interval_passenger (
  TInt timestamp, MNM_Routing_Multimodal_Hybrid *routing, bool del)
{
  MNM_Veh *_veh;
  MNM_Veh_Multimodal *_veh_multimodal;
  MNM_Passenger *_passenger;
  MNM_PnR_Path *_pnr_path;
  // int _flow_scalar = (int)m_dest_node -> m_flow_scalar;
  // add flow_scalar to passenger
  int _flow_scalar = 1;
  TFlt _cruising_time;

  IAssert (m_in_passenger_queue.empty ());

  auto _veh_it = m_dest_node->m_out_veh_queue.begin ();
  while (_veh_it != m_dest_node->m_out_veh_queue.end ())
    {
      _veh = *_veh_it;
      _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
      if (_veh_multimodal->m_class == 0)
        {
          _veh->finish (timestamp);
          // m_parked_car_queue.push_back(_veh_multimodal);
          m_parked_car += 1;
          if (_veh_multimodal->get_ispnr ())
            {
              IAssert (_veh->get_destination ()->m_dest_node->m_node_ID
                       != m_node_ID);
              if (_veh_multimodal->m_type == MNM_TYPE_STATIC)
                {
                  _pnr_path
                    = dynamic_cast<MNM_PnR_Path *> (_veh_multimodal->m_path);
                  IAssert (_pnr_path != nullptr
                           && _pnr_path->m_mid_parking_lot_ID == m_ID
                           && _pnr_path->m_path_ID != TInt (-1));

                  if (m_pnr_fixed_routing_counter.find (_pnr_path->m_path_ID)
                      == m_pnr_fixed_routing_counter.end ())
                    {
                      m_pnr_fixed_routing_counter.insert (
                        std::pair<TInt, TInt> (_pnr_path->m_path_ID, 1));
                    }
                  else
                    {
                      m_pnr_fixed_routing_counter.find (_pnr_path->m_path_ID)
                        ->second
                        += 1;
                    }
                  if ((m_pnr_fixed_routing_counter.find (_pnr_path->m_path_ID)
                         ->second
                       - 1)
                        % _flow_scalar
                      == 0)
                    {
                      _passenger
                        = m_passenger_factory->make_passenger (timestamp,
                                                               MNM_TYPE_STATIC);
                      m_passenger_factory->m_num_passenger_pnr += 1;
                      m_passenger_factory->m_enroute_passenger_pnr += 1;
                      _passenger->set_origin (_veh->get_origin ());
                      _passenger->set_destination (_veh->get_destination ());
                      _passenger->m_pnr = true;
                      _passenger->m_parking_lot = this;
                      _passenger->m_pnr_path = _pnr_path;
                      _passenger->m_driving_path = _pnr_path->m_driving_path;
                      _passenger->m_transit_path
                        = _pnr_path->m_transit_path; // put this in m_tracker in
                                                     // passenger fixed routing
                      _passenger->m_assign_interval = _veh->m_assign_interval;
                      m_in_passenger_queue.push_back (_passenger);
                    }
                }
              else
                {
                  IAssert (_veh_multimodal->m_type == MNM_TYPE_ADAPTIVE);
                  // nominal path for adaptive user, not exactly the actual path
                  _pnr_path
                    = dynamic_cast<MNM_PnR_Path *> (_veh_multimodal->m_path);
                  IAssert (_pnr_path != nullptr
                           && _pnr_path->m_path_ID != TInt (-1));

                  m_pnr_fixed_routing_counter.find (-1)->second += 1;
                  if ((m_pnr_fixed_routing_counter.find (-1)->second - 1)
                        % _flow_scalar
                      == 0)
                    {
                      _passenger
                        = m_passenger_factory
                            ->make_passenger (timestamp, MNM_TYPE_ADAPTIVE);
                      m_passenger_factory->m_num_passenger_pnr += 1;
                      m_passenger_factory->m_enroute_passenger_pnr += 1;
                      _passenger->set_origin (_veh->get_origin ());
                      _passenger->set_destination (_veh->get_destination ());
                      _passenger->m_pnr = true;
                      _passenger->m_parking_lot = this;
                      // nominal path for adaptive user, not exactly the actual
                      // path
                      _passenger->m_pnr_path = _pnr_path;
                      _passenger->m_driving_path = _pnr_path->m_driving_path;
                      _passenger->m_transit_path = _pnr_path->m_transit_path;
                      _passenger->m_assign_interval = _veh->m_assign_interval;
                      m_in_passenger_queue.push_back (_passenger);
                    }
                }
            }
          else
            {
              IAssert (_veh->get_destination ()->m_dest_node->m_node_ID
                       == m_node_ID);
            }
          _veh_it = m_dest_node->m_out_veh_queue.erase (_veh_it);

          if (routing != nullptr)
            {
              routing->remove_finished (_veh, del);
              m_veh_factory->remove_finished_veh (_veh, del);
            }
        }
      else
        {
          IAssert (_veh->get_destination ()->m_dest_node->m_node_ID
                   == m_node_ID);
          _veh_it++;
        }
    }

  // All cars reaching this destination will park here
  // IAssert(m_occupancy <= (float) (m_parked_car_queue.size())  /
  // (float)_flow_scalar); m_occupancy = (float) (m_parked_car_queue.size())  /
  // (float)_flow_scalar;
  IAssert (m_occupancy <= (float) (m_parked_car) / (float) _flow_scalar);
  m_occupancy = (float) (m_parked_car) / (float) _flow_scalar;

  if (m_occupancy >= m_capacity)
    {
      _cruising_time = m_max_parking_time;
    }
  else
    {
      _cruising_time = m_avg_parking_time / (1 - m_occupancy / m_capacity);
    }
  m_cruising_time_record.insert (
    std::pair<TInt, TFlt> (timestamp + 1, _cruising_time)); // intervals
  return 0;
}

int
MNM_Parking_Lot::evolve (TInt timestamp)
{
  // for (auto _link : m_walking_out_links_vec) {
  //     if (_link -> m_N_in != nullptr) {
  //         _link -> m_N_in -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }

  MNM_Walking_Link *_link;
  MNM_Passenger *_passenger;
  auto _passenger_it = m_in_passenger_queue.begin ();
  while (_passenger_it != m_in_passenger_queue.end ())
    {
      _passenger = *_passenger_it;
      IAssert (_passenger->m_start_time == timestamp);
      // only for pnr
      // _passenger -> m_waiting_time = MNM_Ults::max(get_cruise_time(_passenger
      // -> m_start_time) - 1, 0); for mobility service, no cruising time in
      // parking lot
      _passenger->m_waiting_time = 0;

      _link = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
      // IAssert(std::find(m_walking_out_links_vec.begin(),
      // m_walking_out_links_vec.end(), _link) !=
      //         m_walking_out_links_vec.end());
      if (std::find (m_walking_out_links_vec.begin (),
                     m_walking_out_links_vec.end (), _link)
          == m_walking_out_links_vec.end ())
        {
          throw std::runtime_error (
            "MNM_Parking_Lot::evolve, passenger cannot walk out parking lot");
        }
      _link->m_incoming_array.push_back (_passenger);
      _passenger->set_current_link (_link);
      _passenger_it = m_in_passenger_queue.erase (_passenger_it);
      if (_link->m_N_in != nullptr)
        {
          // _link -> m_N_in -> add_increment(std::pair<TFlt,
          // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to passenger
          _link->m_N_in->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (1 / m_dest_node->m_flow_scalar)));
        }
      if (_link->m_N_in_tree != nullptr)
        {
          IAssert (_passenger->m_pnr_path != nullptr);
          // _link -> m_N_in_tree -> add_flow(TFlt(timestamp + 1), TFlt(1),
          // _passenger -> m_pnr_path, _passenger -> m_assign_interval); add
          // flow_scalar to passenger
          _link->m_N_in_tree->add_flow (TFlt (timestamp + 1),
                                        TFlt (1 / m_dest_node->m_flow_scalar),
                                        _passenger->m_pnr_path,
                                        _passenger->m_assign_interval);
        }
    }
  IAssert (m_in_passenger_queue.empty ());
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Parking lot factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Parking_Lot_Factory::MNM_Parking_Lot_Factory ()
{
  m_parking_lot_map = std::unordered_map<TInt, MNM_Parking_Lot *> ();
}

MNM_Parking_Lot_Factory::~MNM_Parking_Lot_Factory ()
{
  for (auto _map_it = m_parking_lot_map.begin ();
       _map_it != m_parking_lot_map.end (); _map_it++)
    {
      delete _map_it->second;
    }
  m_parking_lot_map.clear ();
}

MNM_Parking_Lot *
MNM_Parking_Lot_Factory::make_parking_lot (
  TInt ID, TInt node_ID, MNM_Passenger_Factory *passenger_factory,
  MNM_Veh_Factory *veh_factory, TFlt base_price, TFlt price_surge_coeff,
  TFlt avg_parking_time, TFlt capacity, TFlt unit_time)
{
  MNM_Parking_Lot *_parking_lot
    = new MNM_Parking_Lot (ID, node_ID, passenger_factory, veh_factory,
                           base_price, price_surge_coeff, avg_parking_time,
                           capacity, unit_time);
  m_parking_lot_map.insert (
    std::pair<TInt, MNM_Parking_Lot *> (ID, _parking_lot));
  return _parking_lot;
}

MNM_Parking_Lot *
MNM_Parking_Lot_Factory::get_parking_lot (TInt ID)
{
  auto _parking_lot_it = m_parking_lot_map.find (ID);
  if (_parking_lot_it == m_parking_lot_map.end ())
    {
      printf ("No such parking lot ID %d\n", (int) ID);
      throw std::runtime_error (
        "Error, MNM_Parking_Lot_Factory::get_parking_lot, parking lot does not "
        "exist");
    }
  return _parking_lot_it->second;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                            Passenger
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Passenger::MNM_Passenger (TInt ID, TInt start_time, TInt passenger_type)
{
  m_passenger_ID = ID;
  m_passenger_type = passenger_type;
  m_pnr = false;
  m_current_link = nullptr;
  m_next_link = nullptr;
  m_origin = nullptr;
  m_dest = nullptr;
  m_parking_lot = nullptr;
  m_start_time = start_time;
  m_finish_time = -1; // use to calculate number of still running passenger
  m_assign_interval = -1;
  m_driving_path = nullptr;
  m_transit_path = nullptr;
  m_pnr_path = nullptr;
}

MNM_Passenger::~MNM_Passenger ()
{
  m_current_link = nullptr;
  m_next_link = nullptr;
  m_origin = nullptr;
  m_dest = nullptr;
  m_parking_lot = nullptr;
  m_driving_path = nullptr;
  m_transit_path = nullptr;
  m_pnr_path = nullptr;
}

int
MNM_Passenger::set_current_link (MNM_Transit_Link *link)
{
  m_current_link = link;
  return 0;
}

MNM_Transit_Link *
MNM_Passenger::get_current_link ()
{
  return m_current_link;
}

MNM_Transit_Link *
MNM_Passenger::get_next_link ()
{
  return m_next_link;
}

int
MNM_Passenger::set_next_link (MNM_Transit_Link *link)
{
  m_next_link = link;
  return 0;
}

bool
MNM_Passenger::has_next_link ()
{
  return (m_next_link != nullptr);
}

MNM_Destination *
MNM_Passenger::get_destination ()
{
  return m_dest;
}

int
MNM_Passenger::set_destination (MNM_Destination *dest)
{
  m_dest = dest;
  return 0;
}

int
MNM_Passenger::finish (TInt finish_time)
{
  m_finish_time = finish_time;
  return 0;
}

MNM_Origin *
MNM_Passenger::get_origin ()
{
  return m_origin;
}

int
MNM_Passenger::set_origin (MNM_Origin *origin)
{
  m_origin = origin;
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                         Passenger  Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Passenger_Factory::MNM_Passenger_Factory ()
{
  m_num_passenger = 0; // include m_num_passenger_pnr
  m_enroute_passenger = 0;
  m_finished_passenger = 0;
  m_total_time_passenger = TFlt (0);
  m_num_passenger_pnr = 0;
  m_enroute_passenger_pnr = 0;
  m_finished_passenger_pnr = 0;
  m_passenger_map = std::unordered_map<TInt, MNM_Passenger *> ();
}

MNM_Passenger_Factory::~MNM_Passenger_Factory ()
{
  for (auto _it : m_passenger_map)
    {
      delete _it.second;
    }
  m_passenger_map.clear ();
}

MNM_Passenger *
MNM_Passenger_Factory::make_passenger (TInt timestamp, TInt passenger_type)
{
  // printf("A passenger is produce at time %d, ID is %d\n", (int)timestamp,
  // (int)m_num_passenger + 1);
  MNM_Passenger *_passenger
    = new MNM_Passenger (m_num_passenger + 1, timestamp, passenger_type);
  m_passenger_map.insert ({ m_num_passenger + 1, _passenger });
  m_num_passenger += 1; // m_enroute_passenger_pnr is updated in parking_lot ->
                        // release_one_interval_passenger()
  m_enroute_passenger += 1;
  return _passenger;
}

MNM_Passenger *
MNM_Passenger_Factory::get_passenger (TInt ID)
{
  auto _passenger_it = m_passenger_map.find (ID);
  if (_passenger_it == m_passenger_map.end ())
    {
      printf ("No such passenger ID %d\n", (int) ID);
      throw std::runtime_error ("Error, MNM_Passenger_Factory::get_passenger, "
                                "passenger does not exist");
    }
  return _passenger_it->second;
}

int
MNM_Passenger_Factory::remove_finished_passenger (MNM_Passenger *passenger,
                                                  bool del)
{
  if (m_passenger_map.find (passenger->m_passenger_ID) == m_passenger_map.end ()
      || m_passenger_map.find (passenger->m_passenger_ID)->second != passenger)
    {
      throw std::runtime_error (
        "Error, MNM_Passenger_Factory::remove_finished_passenger, passenger "
        "not in factory");
    }
  if (del)
    {
      m_passenger_map.erase (passenger->m_passenger_ID);
    }

  IAssert (passenger->m_finish_time > passenger->m_start_time);
  m_total_time_passenger
    += (passenger->m_finish_time - passenger->m_start_time);
  if (del)
    {
      delete passenger;
    }

  m_finished_passenger += 1;
  m_enroute_passenger -= 1;
  if (passenger->m_pnr)
    {
      m_finished_passenger_pnr += 1;
      m_enroute_passenger_pnr -= 1;
    }
  IAssert (m_num_passenger >= m_num_passenger_pnr);
  IAssert (m_num_passenger == m_finished_passenger + m_enroute_passenger);
  IAssert (m_num_passenger_pnr
           == m_finished_passenger_pnr + m_enroute_passenger_pnr);
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                          Multimodal Vehicle
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Veh_Multimodal::MNM_Veh_Multimodal (TInt ID, TInt vehicle_class,
                                        TInt start_time, TInt capacity,
                                        TInt bus_route_ID, bool is_pnr)
    : MNM_Veh_Multiclass::MNM_Veh_Multiclass (ID, vehicle_class, start_time)
{
  m_capacity = capacity;
  m_bus_route_ID = bus_route_ID;
  m_stopped_intervals = TInt (0);
  m_min_dwell_intervals = TInt (0);
  m_boarding_lost_intervals = TInt (0);
  m_max_alighting_passengers_per_unit_time = TInt (0);
  m_max_boarding_passengers_per_unit_time = TInt (0);
  m_pnr = is_pnr;
  m_waiting_time = TInt (0);
  m_path = nullptr;
  m_transit_path = nullptr;
  m_pnr_path = nullptr;
  m_passenger_pool = std::deque<MNM_Passenger *> ();
}

MNM_Veh_Multimodal::~MNM_Veh_Multimodal ()
{
  m_transit_path = nullptr;
  m_passenger_pool.clear ();
}

int
MNM_Veh_Multimodal::board_and_alight (TInt timestamp, MNM_Busstop *busstop)
{
  IAssert (m_class == 1 && m_bus_route_ID != -1);
  auto *_busstop_virtual = dynamic_cast<MNM_Busstop_Virtual *> (busstop);
  if (_busstop_virtual == nullptr)
    {
      throw std::runtime_error (
        "Boarding and alighting only occur at virtual busstop");
    }
  MNM_Passenger *_passenger;
  MNM_Walking_Link *_walking_link;
  MNM_Bus_Link *_bus_link;
  TInt _alighting_counter = 0;
  TInt _boarding_counter = 0;
  // Simultaneous boarding and alighting
  // alighting
  auto _passenger_it = m_passenger_pool.begin ();
  while (_passenger_it != m_passenger_pool.end ())
    {
      _passenger = *_passenger_it;
      if (_passenger->get_next_link ()->m_link_type
          == MNM_TYPE_WALKING_MULTIMODAL)
        {
          _walking_link
            = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
        }
      else
        {
          _passenger_it++;
          continue;
        }
      if (_walking_link != nullptr
          && _busstop_virtual->m_alighting_link == _walking_link)
        {
          // if (_alighting_counter >= m_max_alighting_passengers_per_unit_time)
          // {
          //     break;
          // }
          // add flow_scalar to passenger
          if (_alighting_counter >= m_max_alighting_passengers_per_unit_time
                                      * int (busstop->m_flow_scalar))
            {
              break;
            }
          // // in cc of alighting link has been updated in
          // MNM_Busstop_Virtual::receive_bus()
          _walking_link->m_finished_array.push_back (_passenger);
          _passenger->m_waiting_time = 0;
          _passenger->set_current_link (_walking_link);
          _passenger_it = m_passenger_pool.erase (_passenger_it);
          _alighting_counter += 1;

          // update cc tree for passengers about to alight at this bus stop
          // update cc for passengers about to alight at this bus stop
          IAssert (_busstop_virtual->m_bus_in_link != nullptr);
          if (_busstop_virtual->m_bus_in_link->m_N_out != nullptr)
            {
              // _busstop_virtual -> m_bus_in_link -> m_N_out ->
              // add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1),
              // TFlt(1))); add flow_scalar to passenger
              _busstop_virtual->m_bus_in_link->m_N_out->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / busstop->m_flow_scalar)));
            }
          if (_busstop_virtual->m_bus_in_link->m_N_out_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _busstop_virtual -> m_bus_in_link -> m_N_out_tree ->
                  // add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
                  // m_pnr_path, _passenger -> m_assign_interval); add
                  // flow_scalar to passenger
                  _busstop_virtual->m_bus_in_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1),
                                TFlt (1 / busstop->m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _busstop_virtual -> m_bus_in_link -> m_N_out_tree ->
                  // add_flow(TFlt(timestamp+1), TFlt(1), _passenger ->
                  // m_transit_path, _passenger -> m_assign_interval); add
                  // flow_scalar to passenger
                  _busstop_virtual->m_bus_in_link->m_N_out_tree
                    ->add_flow (TFlt (timestamp + 1),
                                TFlt (1 / busstop->m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
          if (_walking_link->m_N_in != nullptr)
            {
              // _walking_link -> m_N_in -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp+1), TFlt(1))); add flow_scalar to
              // passenger
              _walking_link->m_N_in->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / busstop->m_flow_scalar)));
            }
          if (_walking_link->m_N_in_tree != nullptr)
            {
              if (_passenger->m_pnr)
                {
                  // _walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp+1),
                  // TFlt(1), _passenger -> m_pnr_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1),
                                TFlt (1 / busstop->m_flow_scalar),
                                _passenger->m_pnr_path,
                                _passenger->m_assign_interval);
                }
              else
                {
                  // _walking_link -> m_N_in_tree -> add_flow(TFlt(timestamp+1),
                  // TFlt(1), _passenger -> m_transit_path, _passenger ->
                  // m_assign_interval); add flow_scalar to passenger
                  _walking_link->m_N_in_tree
                    ->add_flow (TFlt (timestamp + 1),
                                TFlt (1 / busstop->m_flow_scalar),
                                _passenger->m_transit_path,
                                _passenger->m_assign_interval);
                }
            }
        }
      else
        {
          _passenger_it++;
        }
    }

  // boarding
  // TInt _remaining_capacity = MNM_Ults::min(m_capacity -
  // (int)m_passenger_pool.size(), m_max_boarding_passengers_per_unit_time); add
  // flow_scalar to passenger
  TInt _remaining_capacity
    = MNM_Ults::min (TInt (int (m_capacity * busstop->m_flow_scalar)
                           - (int) m_passenger_pool.size ()),
                     int (m_max_boarding_passengers_per_unit_time
                          * busstop->m_flow_scalar));
  if (_remaining_capacity > 0 && _busstop_virtual->m_boarding_link != nullptr)
    {
      _walking_link = _busstop_virtual->m_boarding_link;
      TInt _tot_boarding_passengers
        = (int) _walking_link->m_finished_array.size ();

      if (_tot_boarding_passengers >= 1)
        {
          // board passengers
          auto _passenger_it = _walking_link->m_finished_array.begin ();
          while (_passenger_it != _walking_link->m_finished_array.end ())
            {
              _passenger = *_passenger_it;
              _bus_link
                = dynamic_cast<MNM_Bus_Link *> (_passenger->get_next_link ());
              if (_bus_link == nullptr
                  || _bus_link != _busstop_virtual->m_bus_out_link
                  || _bus_link->m_route_ID != m_bus_route_ID)
                {
                  throw std::runtime_error (
                    "Something wrong in passenger routing");
                }

              if (_boarding_counter >= _remaining_capacity)
                {
                  break;
                }
              m_passenger_pool.push_back (_passenger);
              _passenger->set_current_link (_bus_link);
              if (_walking_link->m_N_out != nullptr)
                {
                  // _walking_link -> m_N_out -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  _walking_link->m_N_out->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / busstop->m_flow_scalar)));
                }
              if (_bus_link->m_N_in != nullptr)
                {
                  // _bus_link -> m_N_in -> add_increment(std::pair<TFlt,
                  // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
                  // passenger
                  _bus_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / busstop->m_flow_scalar)));
                }
              if (_walking_link->m_N_out_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _walking_link -> m_N_out_tree ->add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                      // m_assign_interval); add flow_scalar to passenger
                      _walking_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / busstop->m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _walking_link -> m_N_out_tree ->add_flow(TFlt(timestamp
                      // + 1), TFlt(1), _passenger -> m_transit_path, _passenger
                      // -> m_assign_interval); add flow_scalar to passenger
                      _walking_link->m_N_out_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / busstop->m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              if (_bus_link->m_N_in_tree != nullptr)
                {
                  if (_passenger->m_pnr)
                    {
                      // _bus_link -> m_N_in_tree ->add_flow(TFlt(timestamp +
                      // 1), TFlt(1), _passenger -> m_pnr_path, _passenger ->
                      // m_assign_interval); add flow_scalar to passenger
                      _bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / busstop->m_flow_scalar),
                                    _passenger->m_pnr_path,
                                    _passenger->m_assign_interval);
                    }
                  else
                    {
                      // _bus_link -> m_N_in_tree ->add_flow(TFlt(timestamp +
                      // 1), TFlt(1), _passenger -> m_transit_path, _passenger
                      // -> m_assign_interval); add flow_scalar to passenger
                      _bus_link->m_N_in_tree
                        ->add_flow (TFlt (timestamp + 1),
                                    TFlt (1 / busstop->m_flow_scalar),
                                    _passenger->m_transit_path,
                                    _passenger->m_assign_interval);
                    }
                }
              _passenger_it
                = _walking_link->m_finished_array.erase (_passenger_it);
              _boarding_counter += 1;
            }
        }
    }

  // https://stackoverflow.com/questions/6926433/how-to-shuffle-a-stdvector
  std::random_shuffle (m_passenger_pool.begin (), m_passenger_pool.end ());

  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Vehicle Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Veh_Factory_Multimodal::MNM_Veh_Factory_Multimodal (
  TInt bus_capacity, TInt min_dwell_intervals, TInt boarding_lost_intervals,
  TInt max_alighting_passengers_per_unit_time,
  TInt max_boarding_passengers_per_unit_time)
    : MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass ()
{
  m_bus_capacity = bus_capacity;
  m_min_dwell_intervals = min_dwell_intervals;
  m_boarding_lost_intervals = boarding_lost_intervals;
  m_max_alighting_passengers_per_unit_time
    = max_alighting_passengers_per_unit_time;
  m_max_boarding_passengers_per_unit_time
    = max_boarding_passengers_per_unit_time;

  m_num_bus = TInt (0);
  m_enroute_bus = TInt (0);
  m_finished_bus = TInt (0);
  m_total_time_bus = TFlt (0);

  m_num_car_pnr = TInt (0);
  m_enroute_car_pnr = TInt (0);
  m_finished_car_pnr = TInt (0);
}

MNM_Veh_Factory_Multimodal::~MNM_Veh_Factory_Multimodal () { ; }

MNM_Veh_Multimodal *
MNM_Veh_Factory_Multimodal::make_veh_multimodal (
  TInt timestamp, Vehicle_type veh_type, TInt vehicle_cls, TInt capacity,
  TInt bus_route_ID, bool is_pnr, TInt pickup_waiting_time)
{
  // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp,
  // (int)m_num_veh + 1);
  MNM_Veh_Multimodal *_veh
    = new MNM_Veh_Multimodal (m_num_veh + 1, vehicle_cls, timestamp, capacity,
                              bus_route_ID, is_pnr);
  _veh->m_type = veh_type;
  if (vehicle_cls == 1 && bus_route_ID != -1)
    {
      _veh->m_min_dwell_intervals = m_min_dwell_intervals;
      _veh->m_boarding_lost_intervals = m_boarding_lost_intervals;
      _veh->m_max_alighting_passengers_per_unit_time
        = m_max_alighting_passengers_per_unit_time;
      _veh->m_max_boarding_passengers_per_unit_time
        = m_max_boarding_passengers_per_unit_time;
    }
  if (_veh->get_ispnr () && vehicle_cls == 0)
    {
      _veh->m_waiting_time = pickup_waiting_time;
    }
  m_veh_map.insert ({ m_num_veh + 1, _veh });

  m_num_veh += 1;
  m_enroute += 1;
  if (vehicle_cls == 0)
    {
      if (is_pnr)
        {
          m_num_car_pnr += 1;
          m_enroute_car_pnr += 1;
        }
      else
        {
          m_num_car += 1;
          m_enroute_car += 1;
        }
    }
  else if (vehicle_cls == 1)
    {
      if (bus_route_ID != -1)
        {
          m_num_bus += 1;
          m_enroute_bus += 1;
        }
      else
        {
          m_num_truck += 1;
          m_enroute_truck += 1;
        }
    }
  return _veh;
}

int
MNM_Veh_Factory_Multimodal::remove_finished_veh (MNM_Veh *veh, bool del)
{
  IAssert (veh->m_finish_time > veh->m_start_time);
  if (veh->get_class () == 0)
    {
      if (veh->get_ispnr ())
        {
          m_finished_car_pnr += 1;
          m_enroute_car_pnr -= 1;
        }
      else
        {
          m_finished_car += 1;
          m_enroute_car -= 1;
        }
      m_total_time_car += (veh->m_finish_time - veh->m_start_time);
    }
  else if (veh->get_class () == 1)
    {
      if (veh->get_bus_route_ID () == TInt (-1))
        {
          m_finished_truck += 1;
          m_enroute_truck -= 1;
          m_total_time_truck += (veh->m_finish_time - veh->m_start_time);
        }
      else
        {
          m_finished_bus += 1;
          m_enroute_bus -= 1;
          m_total_time_bus += (veh->m_finish_time - veh->m_start_time);
        }
    }
  MNM_Veh_Factory::remove_finished_veh (veh, del);
  IAssert (m_num_bus == m_finished_bus + m_enroute_bus);
  IAssert (m_num_car == m_finished_car + m_enroute_car);
  IAssert (m_num_truck == m_finished_truck + m_enroute_truck);
  IAssert (m_num_car_pnr == m_finished_car_pnr + m_enroute_car_pnr);
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal OD
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Destination_Multimodal::MNM_Destination_Multimodal (TInt ID)
    : MNM_Destination_Multiclass::MNM_Destination_Multiclass (ID)
{
  m_parking_lot = nullptr;
  m_walking_in_links_vec = std::vector<MNM_Walking_Link *> ();
  m_out_passenger_queue = std::deque<MNM_Passenger *> ();
  m_connected_pnr_parkinglot_vec
    = std::vector<MNM_Parking_Lot *> (); // set in
                                         // check_bus_transit_connectivity()
}

MNM_Destination_Multimodal::~MNM_Destination_Multimodal ()
{
  m_walking_in_links_vec.clear ();
  m_out_passenger_queue.clear ();
  m_connected_pnr_parkinglot_vec.clear ();
}

int
MNM_Destination_Multimodal::evolve (TInt timestamp)
{
  MNM_Passenger *_passenger;
  for (auto _link : m_walking_in_links_vec)
    {
      // if (_link -> m_N_out != nullptr) {
      //     _link -> m_N_out -> add_increment(std::pair<TFlt,
      //     TFlt>(TFlt(timestamp + 1), TFlt(0)));
      // }
      auto _passenger_it = _link->m_finished_array.begin ();
      while (_passenger_it != _link->m_finished_array.end ())
        {
          _passenger = *_passenger_it;
          if (_passenger->get_next_link () != nullptr)
            {
              MNM_Walking_Link *_next_walking_link
                = dynamic_cast<MNM_Walking_Link *> (
                  _passenger->get_next_link ());
              if (_next_walking_link == nullptr || m_parking_lot == nullptr
                  || _passenger->get_destination ()->m_dest_node->m_node_ID
                       == m_dest_node->m_node_ID
                  || std::find (m_parking_lot->m_walking_out_links_vec.begin (),
                                m_parking_lot->m_walking_out_links_vec.end (),
                                _next_walking_link)
                       == m_parking_lot->m_walking_out_links_vec.end ())
                {
                  // just passing by this middle destination node
                  throw std::runtime_error (
                    "MNM_Destination_Multimodal::evolve, passenger cannot walk "
                    "out middle destination");
                }
              _next_walking_link->m_incoming_array.push_back (_passenger);
              _passenger->set_current_link (_next_walking_link);
              if (_next_walking_link->m_N_in != nullptr)
                {
                  // _next_walking_link -> m_N_in ->
                  // add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp + 1),
                  // TFlt(1))); add flow_scalar to passenger
                  _next_walking_link->m_N_in->add_increment (
                    std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                           TFlt (1 / m_flow_scalar)));
                }
            }
          else
            {
              // final destination
              IAssert (_passenger->get_destination ()->m_dest_node->m_node_ID
                       == m_dest_node->m_node_ID);
              m_out_passenger_queue.push_back (_passenger);
              _passenger->set_current_link (nullptr);
            }

          _passenger_it = _link->m_finished_array.erase (_passenger_it);
          if (_link->m_N_out != nullptr)
            {
              // _link -> m_N_out -> add_increment(std::pair<TFlt,
              // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to
              // passenger
              _link->m_N_out->add_increment (
                std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                       TFlt (1 / m_flow_scalar)));
            }
        }
      IAssert (_link->m_finished_array.empty ());
    }
  return 0;
}

int
MNM_Destination_Multimodal::receive (TInt timestamp)
{
  // vehicles
  if (m_parking_lot != nullptr)
    {
      m_parking_lot->release_one_interval_passenger (timestamp, nullptr, false);
    }

  MNM_Veh *_veh;
  size_t _num_to_receive = m_dest_node->m_out_veh_queue.size ();
  // printf("Dest node %d out vehicle: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _veh = m_dest_node->m_out_veh_queue.front ();
      IAssert (!_veh->get_ispnr ());
      if (_veh->get_destination ()->m_dest_node->m_node_ID
          != m_dest_node->m_node_ID)
        {
          printf ("The veh is heading to %d, but we are %d\n",
                  (int) _veh->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error (
            "MNM_Destination::receive, vehicle reaches wrong destination!");
        }
      _veh->finish (timestamp);
      // printf("Receive Vehicle ID: %d, origin node is %d, destination node is
      // %d\n", _veh -> m_veh_ID(), _veh -> get_origin() -> m_origin_node ->
      // m_node_ID(), _veh -> get_destination() -> m_dest_node -> m_node_ID());
      m_dest_node->m_out_veh_queue.pop_front ();
    }

  // passengers
  MNM_Passenger *_passenger;
  _num_to_receive = m_out_passenger_queue.size ();
  // printf("Dest node %d out passenger: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _passenger = m_out_passenger_queue.front ();
      if (_passenger->get_destination ()->m_dest_node->m_node_ID
          != m_dest_node->m_node_ID)
        {
          printf ("The passenger is heading to %d, but we are %d\n",
                  (int) _passenger->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error (
            "MNM_Destination::receive, passenger reaches wrong destination!");
        }
      _passenger->finish (timestamp);
      // printf("Receive Passenger ID: %d, origin node is %d, destination node
      // is %d\n", _passenger -> m_ID(), _passenger -> get_origin() ->
      // m_origin_node -> m_node_ID(), _passenger -> get_destination() ->
      // m_dest_node -> m_node_ID());
      m_out_passenger_queue.pop_front ();
    }
  return 0;
}

int
MNM_Destination_Multimodal::receive (TInt timestamp,
                                     MNM_Routing_Multimodal_Hybrid *routing,
                                     MNM_Veh_Factory *veh_factory,
                                     MNM_Passenger_Factory *passenger_factory,
                                     bool del)
{
  // vehicles
  if (m_parking_lot != nullptr)
    {
      m_parking_lot->release_one_interval_passenger (timestamp, routing, del);
    }

  MNM_Veh *_veh;
  size_t _num_to_receive = m_dest_node->m_out_veh_queue.size ();
  // printf("Dest node %d out vehicle: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _veh = m_dest_node->m_out_veh_queue.front ();
      IAssert (!_veh->get_ispnr ());
      if (_veh->get_destination ()->m_dest_node->m_node_ID
          != m_dest_node->m_node_ID)
        {
          printf ("The veh is heading to %d, but we are %d\n",
                  (int) _veh->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error (
            "MNM_Destination::receive, vehicle reaches wrong destination!");
        }
      _veh->finish (timestamp);
      // printf("Receive Vehicle ID: %d, origin node is %d, destination node is
      // %d\n", _veh -> m_veh_ID(), _veh -> get_origin() -> m_origin_node ->
      // m_node_ID(), _veh -> get_destination() -> m_dest_node -> m_node_ID());
      m_dest_node->m_out_veh_queue.pop_front ();

      routing->remove_finished (_veh, del);
      veh_factory->remove_finished_veh (_veh, del);
    }

  // passengers
  MNM_Passenger *_passenger;
  _num_to_receive = m_out_passenger_queue.size ();
  // printf("Dest node %d out passenger: %d\n", m_dest_node -> m_node_ID,
  // _num_to_receive);
  for (size_t i = 0; i < _num_to_receive; ++i)
    {
      _passenger = m_out_passenger_queue.front ();
      if (_passenger->get_destination ()->m_dest_node->m_node_ID
          != m_dest_node->m_node_ID)
        {
          printf ("The passenger is heading to %d, but we are %d\n",
                  (int) _passenger->get_destination ()->m_dest_node->m_node_ID,
                  (int) m_dest_node->m_node_ID);
          throw std::runtime_error (
            "MNM_Destination::receive, passenger reaches wrong destination!");
        }
      _passenger->finish (timestamp);
      // printf("Receive Passenger ID: %d, origin node is %d, destination node
      // is %d\n", _passenger -> m_ID(), _passenger -> get_origin() ->
      // m_origin_node -> m_node_ID(), _passenger -> get_destination() ->
      // m_dest_node -> m_node_ID());
      m_out_passenger_queue.pop_front ();

      routing->remove_finished_passenger (_passenger, del);
      passenger_factory->remove_finished_passenger (_passenger, del);
    }
  return 0;
}

MNM_Origin_Multimodal::MNM_Origin_Multimodal (TInt ID, TInt max_interval,
                                              TFlt flow_scalar, TInt frequency,
                                              TInt pickup_waiting_time)
    : MNM_Origin_Multiclass::MNM_Origin_Multiclass (ID, max_interval,
                                                    flow_scalar, frequency)
{
  m_demand_bus = std::unordered_map<MNM_Destination_Multimodal *,
                                    std::unordered_map<TInt, TFlt *>> ();
  m_demand_pnr_car
    = std::unordered_map<MNM_Destination_Multimodal *, TFlt *> ();
  m_demand_passenger_bus
    = std::unordered_map<MNM_Destination_Multimodal *, TFlt *> ();
  m_walking_out_links_vec = std::vector<MNM_Walking_Link *> ();
  m_in_passenger_queue = std::deque<MNM_Passenger *> ();
  m_pickup_waiting_time = pickup_waiting_time;
}

MNM_Origin_Multimodal::~MNM_Origin_Multimodal ()
{
  for (auto _demand_it : m_demand_car)
    {
      free (_demand_it.second);
    }
  m_demand_car.clear ();

  for (auto _demand_it : m_demand_truck)
    {
      free (_demand_it.second);
    }
  m_demand_truck.clear ();

  for (auto _demand_it : m_demand_bus)
    {
      for (auto _demand_it_it : _demand_it.second)
        {
          free (_demand_it_it.second);
        }
      _demand_it.second.clear ();
    }
  m_demand_bus.clear ();

  for (auto _demand_it : m_demand_pnr_car)
    {
      free (_demand_it.second);
    }
  m_demand_pnr_car.clear ();

  for (auto _demand_it : m_demand_passenger_bus)
    {
      free (_demand_it.second);
    }
  m_demand_passenger_bus.clear ();

  m_walking_out_links_vec.clear ();

  m_in_passenger_queue.clear ();
}

int
MNM_Origin_Multimodal::evolve (TInt timestamp)
{
  // for (auto _link : m_walking_out_links_vec) {
  //     if (_link -> m_N_in != nullptr) {
  //         _link -> m_N_in -> add_increment(std::pair<TFlt,
  //         TFlt>(TFlt(timestamp + 1), TFlt(0)));
  //     }
  // }
  MNM_Walking_Link *_link;
  MNM_Passenger *_passenger;
  auto _passenger_it = m_in_passenger_queue.begin ();
  while (_passenger_it != m_in_passenger_queue.end ())
    {
      _passenger = *_passenger_it;
      _link = dynamic_cast<MNM_Walking_Link *> (_passenger->get_next_link ());
      IAssert (std::find (m_walking_out_links_vec.begin (),
                          m_walking_out_links_vec.end (), _link)
               != m_walking_out_links_vec.end ());
      _link->m_incoming_array.push_back (_passenger);
      _passenger->set_current_link (_link);
      _passenger_it = m_in_passenger_queue.erase (_passenger_it);
      if (_link->m_N_in != nullptr)
        {
          // _link -> m_N_in -> add_increment(std::pair<TFlt,
          // TFlt>(TFlt(timestamp + 1), TFlt(1))); add flow_scalar to passenger
          _link->m_N_in->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (1 / m_flow_scalar)));
        }
      if (_link->m_N_in_tree != nullptr)
        {
          IAssert (_passenger->m_transit_path != nullptr);
          // _link -> m_N_in_tree -> add_flow(TFlt(timestamp + 1), TFlt(1),
          // _passenger -> m_transit_path, _passenger -> m_assign_interval); add
          // flow_scalar to passenger
          _link->m_N_in_tree->add_flow (TFlt (timestamp + 1),
                                        TFlt (1 / m_flow_scalar),
                                        _passenger->m_transit_path,
                                        _passenger->m_assign_interval);
        }
    }
  IAssert (m_in_passenger_queue.empty ());
  return 0;
}

int
MNM_Origin_Multimodal::add_dest_demand_bus (MNM_Destination_Multimodal *dest,
                                            TInt routeID, TFlt *demand_bus)
{
  // split (15-mins demand) to (15 * 1-minute demand)
  // bus demand
  TFlt *_demand_bus
    = (TFlt *) malloc (sizeof (TFlt) * m_max_assign_interval * 15);
  for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
      _demand_bus[i] = TFlt (demand_bus[i]);
    }
  if (m_demand_bus.find (dest) == m_demand_bus.end ())
    {
      std::unordered_map<TInt, TFlt *> _demand_route
        = std::unordered_map<TInt, TFlt *> ();
      _demand_route.insert ({ routeID, _demand_bus });
      m_demand_bus.insert ({ dest, _demand_route });
    }
  else
    {
      m_demand_bus.find (dest)->second.insert ({ routeID, _demand_bus });
    }
  return 0;
}

int
MNM_Origin_Multimodal::add_dest_demand_pnr_car (
  MNM_Destination_Multimodal *dest, TFlt *demand_pnr_car)
{
  // split (15-mins demand) to (15 * 1-minute demand)
  TFlt *_demand_pnr_car
    = (TFlt *) malloc (sizeof (TFlt) * m_max_assign_interval * 15);
  for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
      _demand_pnr_car[i] = TFlt (demand_pnr_car[i]);
    }
  m_demand_pnr_car.insert ({ dest, _demand_pnr_car });
  return 0;
}

int
MNM_Origin_Multimodal::add_dest_demand_passenger_bus (
  MNM_Destination_Multimodal *dest, TFlt *demand_passenger_bus)
{
  // split (15-mins demand) to (15 * 1-minute demand)
  TFlt *_demand_passenger_bus
    = (TFlt *) malloc (sizeof (TFlt) * m_max_assign_interval * 15);
  for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
      _demand_passenger_bus[i] = TFlt (demand_passenger_bus[i]);
    }
  m_demand_passenger_bus.insert ({ dest, _demand_passenger_bus });
  return 0;
}

int
MNM_Origin_Multimodal::release_one_interval (TInt current_interval,
                                             MNM_Veh_Factory *veh_factory,
                                             TInt assign_interval,
                                             TFlt adaptive_ratio)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  TInt _veh_to_release;
  MNM_Veh_Multimodal *_veh;
  MNM_Veh_Factory_Multimodal *_vfactory
    = dynamic_cast<MNM_Veh_Factory_Multimodal *> (veh_factory);
  // release all car
  for (auto _demand_it = m_demand_car.begin ();
       _demand_it != m_demand_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0),
                                                     TInt (1));
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_ADAPTIVE,
                                                     TInt (0), TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0), TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (0), TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // release all truck
  for (auto _demand_it = m_demand_truck.begin ();
       _demand_it != m_demand_truck.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1),
                                                     TInt (1));
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_ADAPTIVE,
                                                     TInt (1), TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (1), TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (1), TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // release all bus, which have all veh_type = MNM_TYPE_STATIC, vehicle_cls =
  // 1, and different bus_route
  for (auto _demand_it = m_demand_bus.begin ();
       _demand_it != m_demand_bus.end (); _demand_it++)
    { // destination
      for (auto _demand_it_it = _demand_it->second.begin ();
           _demand_it_it != _demand_it->second.end (); _demand_it_it++)
        { // bus route
          // _veh_to_release = TInt(MNM_Ults::round((_demand_it_it ->
          // second)[assign_interval] * m_flow_scalar));
          _veh_to_release = TInt (
            floor ((_demand_it_it->second)[assign_interval] * m_flow_scalar));

          for (int i = 0; i < _veh_to_release; ++i)
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1),
                                                     _vfactory->m_bus_capacity,
                                                     _demand_it_it->first);

              _veh->set_destination (_demand_it->first);
              _veh->set_origin (this);
              // in case the multiclass modeling has 1-min release interval as
              // the "assign" interval
              _veh->m_assign_interval = int (current_interval / m_frequency);
              m_origin_node->m_in_veh_queue.push_back (_veh);
            }
        }
    }
  // release pnr car
  for (auto _demand_it = m_demand_pnr_car.begin ();
       _demand_it != m_demand_pnr_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0),
                                                     TInt (1), TInt (-1), true,
                                                     m_pickup_waiting_time);
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multimodal (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (0),
                                                  TInt (1), TInt (-1), true,
                                                  m_pickup_waiting_time);
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0), TInt (1),
                                                         TInt (-1), true,
                                                         m_pickup_waiting_time);
                }
              else
                {
                  _veh
                    = _vfactory->make_veh_multimodal (current_interval,
                                                      MNM_TYPE_STATIC, TInt (0),
                                                      TInt (1), TInt (-1), true,
                                                      m_pickup_waiting_time);
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // https://stackoverflow.com/questions/6926433/how-to-shuffle-a-stdvector
  std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                       m_origin_node->m_in_veh_queue.end ());

  return 0;
}

int
MNM_Origin_Multimodal::release_one_interval_biclass (
  TInt current_interval, MNM_Veh_Factory *veh_factory, TInt assign_interval,
  TFlt adaptive_ratio_car, TFlt adaptive_ratio_truck)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  TInt _veh_to_release;
  MNM_Veh_Multimodal *_veh;
  MNM_Veh_Factory_Multimodal *_vfactory
    = dynamic_cast<MNM_Veh_Factory_Multimodal *> (veh_factory);
  // release all car
  for (auto _demand_it = m_demand_car.begin ();
       _demand_it != m_demand_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio_car == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0),
                                                     TInt (1));
            }
          else if (adaptive_ratio_car == TFlt (1))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_ADAPTIVE,
                                                     TInt (0), TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio_car)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0), TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (0), TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // release all truck
  for (auto _demand_it = m_demand_truck.begin ();
       _demand_it != m_demand_truck.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio_truck == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1),
                                                     TInt (1));
            }
          else if (adaptive_ratio_truck == TFlt (1))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_ADAPTIVE,
                                                     TInt (1), TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio_truck)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (1), TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (1), TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // release all bus, which have all veh_type = MNM_TYPE_STATIC, vehicle_cls =
  // 1, and different bus_route
  for (auto _demand_it = m_demand_bus.begin ();
       _demand_it != m_demand_bus.end (); _demand_it++)
    { // destination
      for (auto _demand_it_it = _demand_it->second.begin ();
           _demand_it_it != _demand_it->second.end (); _demand_it_it++)
        { // bus route
          // _veh_to_release = TInt(MNM_Ults::round((_demand_it_it ->
          // second)[assign_interval] * m_flow_scalar));
          _veh_to_release = TInt (
            floor ((_demand_it_it->second)[assign_interval] * m_flow_scalar));

          for (int i = 0; i < _veh_to_release; ++i)
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1),
                                                     _vfactory->m_bus_capacity,
                                                     _demand_it_it->first);

              _veh->set_destination (_demand_it->first);
              _veh->set_origin (this);
              // in case the multiclass modeling has 1-min release interval as
              // the "assign" interval
              _veh->m_assign_interval = int (current_interval / m_frequency);
              m_origin_node->m_in_veh_queue.push_back (_veh);
            }
        }
    }
  // release pnr car
  for (auto _demand_it = m_demand_pnr_car.begin ();
       _demand_it != m_demand_pnr_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _veh_to_release = TInt(floor((_demand_it -> second)[assign_interval] *
      // m_flow_scalar));

      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio_car == TFlt (0))
            {
              _veh = _vfactory->make_veh_multimodal (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0),
                                                     TInt (1), TInt (-1), true,
                                                     m_pickup_waiting_time);
            }
          else if (adaptive_ratio_car == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multimodal (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (0),
                                                  TInt (1), TInt (-1), true,
                                                  m_pickup_waiting_time);
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio_car)
                {
                  _veh = _vfactory->make_veh_multimodal (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0), TInt (1),
                                                         TInt (-1), true,
                                                         m_pickup_waiting_time);
                }
              else
                {
                  _veh
                    = _vfactory->make_veh_multimodal (current_interval,
                                                      MNM_TYPE_STATIC, TInt (0),
                                                      TInt (1), TInt (-1), true,
                                                      m_pickup_waiting_time);
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  // https://stackoverflow.com/questions/6926433/how-to-shuffle-a-stdvector
  std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                       m_origin_node->m_in_veh_queue.end ());
  return 0;
}

int
MNM_Origin_Multimodal::release_one_interval_passenger (
  TInt current_interval, MNM_Passenger_Factory *passenger_factory,
  TInt assign_interval, TFlt adaptive_ratio)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  // release passenger riding bus
  TInt _passenger_to_release;
  MNM_Passenger *_passenger;
  for (auto _demand_it = m_demand_passenger_bus.begin ();
       _demand_it != m_demand_passenger_bus.end (); _demand_it++)
    {
      // _passenger_to_release = TInt(MNM_Ults::round((_demand_it ->
      // second)[assign_interval]));
      // // _passenger_to_release = TInt(floor((_demand_it ->
      // second)[assign_interval]));

      // add flow_scalar to passenger
      _passenger_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      // _passenger_to_release = TInt(floor((_demand_it ->
      // second)[assign_interval] * m_flow_scalar));

      for (int i = 0; i < _passenger_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _passenger = passenger_factory->make_passenger (current_interval,
                                                              MNM_TYPE_STATIC);
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _passenger
                = passenger_factory->make_passenger (current_interval,
                                                     MNM_TYPE_ADAPTIVE);
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _passenger
                    = passenger_factory->make_passenger (current_interval,
                                                         MNM_TYPE_ADAPTIVE);
                }
              else
                {
                  _passenger
                    = passenger_factory->make_passenger (current_interval,
                                                         MNM_TYPE_STATIC);
                }
            }
          _passenger->set_destination (_demand_it->first);
          _passenger->set_origin (this);
          _passenger->m_pnr = false;
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _passenger->m_assign_interval = int (current_interval / m_frequency);
          m_in_passenger_queue.push_back (_passenger);
        }
    }
  std::random_shuffle (m_in_passenger_queue.begin (),
                       m_in_passenger_queue.end ());
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                OD Factory
*******************************************************************************************************************
******************************************************************************************************************/
MNM_OD_Factory_Multimodal::MNM_OD_Factory_Multimodal ()
    : MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass ()
{
  ;
}

MNM_OD_Factory_Multimodal::~MNM_OD_Factory_Multimodal () { ; }

MNM_Destination_Multimodal *
MNM_OD_Factory_Multimodal::make_destination (TInt ID)
{
  MNM_Destination_Multimodal *_dest;
  _dest = new MNM_Destination_Multimodal (ID);
  m_destination_map.insert ({ ID, _dest });
  return _dest;
}

MNM_Origin_Multimodal *
MNM_OD_Factory_Multimodal::make_origin (TInt ID, TInt max_interval,
                                        TFlt flow_scalar, TInt frequency)
{
  MNM_Origin_Multimodal *_origin;
  _origin
    = new MNM_Origin_Multimodal (ID, max_interval, flow_scalar, frequency);
  m_origin_map.insert ({ ID, _origin });
  return _origin;
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_OD_Factory_Multimodal::get_random_od_pair ()
{
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;

  auto _origin_it = m_origin_map.begin ();
  int random_index = rand () % m_origin_map.size ();
  std::advance (_origin_it, random_index);

  _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
  while (_origin->m_demand_car.empty ())
    {
      _origin_it = m_origin_map.begin ();
      random_index = rand () % m_origin_map.size ();
      std::advance (_origin_it, random_index);
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
    }

  auto _dest_it = _origin->m_demand_car.begin ();
  random_index = rand () % _origin->m_demand_car.size ();
  std::advance (_dest_it, random_index);
  _dest = dynamic_cast<MNM_Destination_Multimodal *> (_dest_it->first);

  return std::pair<MNM_Origin *, MNM_Destination *> (_origin, _dest);
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_OD_Factory_Multimodal::get_random_od_pair_bustransit ()
{
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;

  auto _origin_it = m_origin_map.begin ();
  int random_index = rand () % m_origin_map.size ();
  std::advance (_origin_it, random_index);

  _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
  while (_origin->m_demand_passenger_bus.empty ())
    {
      _origin_it = m_origin_map.begin ();
      random_index = rand () % m_origin_map.size ();
      std::advance (_origin_it, random_index);
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
    }

  auto _dest_it = _origin->m_demand_passenger_bus.begin ();
  random_index = rand () % _origin->m_demand_passenger_bus.size ();
  std::advance (_dest_it, random_index);
  _dest = dynamic_cast<MNM_Destination_Multimodal *> (_dest_it->first);

  return std::pair<MNM_Origin *, MNM_Destination *> (_origin, _dest);
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_OD_Factory_Multimodal::get_random_od_pair_pnr ()
{
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;

  auto _origin_it = m_origin_map.begin ();
  int random_index = rand () % m_origin_map.size ();
  std::advance (_origin_it, random_index);

  _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
  while (_origin->m_demand_pnr_car.empty ())
    {
      _origin_it = m_origin_map.begin ();
      random_index = rand () % m_origin_map.size ();
      std::advance (_origin_it, random_index);
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it->second);
    }

  auto _dest_it = _origin->m_demand_pnr_car.begin ();
  random_index = rand () % _origin->m_demand_pnr_car.size ();
  std::advance (_dest_it, random_index);
  _dest = dynamic_cast<MNM_Destination_Multimodal *> (_dest_it->first);

  return std::pair<MNM_Origin *, MNM_Destination *> (_origin, _dest);
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Node Factory
*******************************************************************************************************************
******************************************************************************************************************/

MNM_Node_Factory_Multimodal::MNM_Node_Factory_Multimodal ()
    : MNM_Node_Factory_Multiclass::MNM_Node_Factory_Multiclass ()
{
  ;
}

MNM_Node_Factory_Multimodal::~MNM_Node_Factory_Multimodal () { ; }

MNM_Dnode *
MNM_Node_Factory_Multimodal::make_node_multimodal (
  TInt ID, DNode_type_multimodal node_type, TFlt flow_scalar,
  TFlt veh_convert_factor)
{
  MNM_Dnode *_node;
  switch (node_type)
    {
    case MNM_TYPE_FWJ_MULTIMODAL:
      _node
        = new MNM_Dnode_FWJ_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    case MNM_TYPE_ORIGIN_MULTIMODAL:
      _node = new MNM_DMOND_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    case MNM_TYPE_DEST_MULTIMODAL:
      _node = new MNM_DMDND_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    default:
      throw std::runtime_error ("Wrong node type");
    }
  m_node_map.insert ({ ID, _node });
  return _node;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Link Models
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                Transit Link Model
**************************************************************************/
MNM_Transit_Link::MNM_Transit_Link (TInt link_ID,
                                    DLink_type_multimodal link_type,
                                    const std::string &from_node_type,
                                    const std::string &to_node_type,
                                    TInt from_node_ID, TInt to_node_ID,
                                    TFlt unit_time)
{
  m_link_ID = link_ID;
  m_link_type = link_type;
  m_from_node_type = from_node_type;
  m_to_node_type = to_node_type;
  m_from_node_ID = from_node_ID;
  m_to_node_ID = to_node_ID;

  m_fftt = TFlt (-1); // seconds
  m_unit_time = unit_time;

  m_passenger_queue = std::unordered_map<MNM_Passenger *, TInt> ();
  m_finished_array = std::deque<MNM_Passenger *> ();
  m_incoming_array = std::deque<MNM_Passenger *> ();

  // recording passengers
  m_N_in = nullptr;
  m_N_out = nullptr;
  m_N_in_tree = nullptr;
  m_N_out_tree = nullptr;

  install_cumulative_curve ();

  // !!! Close cc_tree if only doing loading to save a lot of memory !!!
  // install_cumulative_curve_tree();
}

MNM_Transit_Link::~MNM_Transit_Link ()
{
  m_passenger_queue.clear ();
  m_finished_array.clear ();
  m_incoming_array.clear ();

  if (m_N_out != nullptr)
    delete m_N_out;
  if (m_N_in != nullptr)
    delete m_N_in;
  if (m_N_in_tree != nullptr)
    delete m_N_in_tree;
  if (m_N_out_tree != nullptr)
    delete m_N_out_tree;
}

int
MNM_Transit_Link::install_cumulative_curve ()
{
  if (m_N_out != nullptr)
    {
      delete m_N_out;
    }
  if (m_N_in != nullptr)
    {
      delete m_N_in;
    }
  m_N_out = new MNM_Cumulative_Curve ();
  m_N_in = new MNM_Cumulative_Curve ();
  m_N_in->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  m_N_out->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  return 0;
}

int
MNM_Transit_Link::install_cumulative_curve_tree ()
{
  if (m_N_in_tree != nullptr)
    {
      delete m_N_in_tree;
    }
  if (m_N_out_tree != nullptr)
    {
      delete m_N_out_tree;
    }
  m_N_in_tree = new MNM_Tree_Cumulative_Curve ();
  // m_N_out_tree = new MNM_Tree_Cumulative_Curve();
  return 0;
}

/**************************************************************************
                                                Bus Link Model
**************************************************************************/
MNM_Bus_Link::MNM_Bus_Link (TInt link_ID, TInt from_busstop_ID,
                            TInt to_busstop_ID, TFlt bus_fftt, TFlt unit_time)
    : MNM_Transit_Link::MNM_Transit_Link (link_ID, MNM_TYPE_BUS_MULTIMODAL,
                                          "bus_stop", "bus_stop",
                                          from_busstop_ID, to_busstop_ID,
                                          unit_time)
{
  // when different routes share one same link, make each link for each route,
  // then each link only corresponds to one single route
  m_from_busstop = nullptr;
  m_to_busstop = nullptr;
  m_route_ID = -1;   // set in build_transit_link_factory()
  m_fftt = bus_fftt; // seconds
  m_length = -1.;
  m_overlapped_driving_link_vec = std::vector<MNM_Dlink *> ();
  m_overlapped_driving_link_length_portion_vec = std::vector<TFlt> ();

  // recording bus
  m_N_in_tree_bus = nullptr;
  m_N_out_tree_bus = nullptr;

  // !!! Close cc_tree if only doing loading to save a lot of memory !!!
  // install_cumulative_curve_tree();
}

MNM_Bus_Link::~MNM_Bus_Link ()
{
  m_overlapped_driving_link_vec.clear ();
  m_overlapped_driving_link_length_portion_vec.clear ();
  if (m_N_in_tree_bus != nullptr)
    delete m_N_in_tree_bus;
  if (m_N_out_tree_bus != nullptr)
    delete m_N_out_tree_bus;
}

int
MNM_Bus_Link::install_cumulative_curve_tree ()
{
  // passenger
  if (m_N_in_tree != nullptr)
    {
      delete m_N_in_tree;
    }
  if (m_N_out_tree != nullptr)
    {
      delete m_N_out_tree;
    }
  m_N_in_tree = new MNM_Tree_Cumulative_Curve ();
  // m_N_out_tree = new MNM_Tree_Cumulative_Curve();

  IAssert (m_N_in_tree != nullptr);

  // bus
  if (m_N_in_tree_bus != nullptr)
    {
      delete m_N_in_tree;
    }
  if (m_N_out_tree_bus != nullptr)
    {
      delete m_N_out_tree;
    }
  m_N_in_tree_bus = new MNM_Tree_Cumulative_Curve ();
  // m_N_out_tree_bus = new MNM_Tree_Cumulative_Curve();

  IAssert (m_N_in_tree_bus != nullptr);
  return 0;
}

int
MNM_Bus_Link::get_overlapped_driving_link_length_portion ()
{
  IAssert (!m_overlapped_driving_link_vec.empty ());
  IAssert (m_route_ID > -1 && m_from_busstop->m_route_ID == m_route_ID
           && m_to_busstop->m_route_ID == m_route_ID);
  IAssert (m_from_busstop->m_link_ID
           == m_overlapped_driving_link_vec.front ()->m_link_ID);
  IAssert (m_to_busstop->m_link_ID
           == m_overlapped_driving_link_vec.back ()->m_link_ID);

  if (!m_overlapped_driving_link_length_portion_vec.empty ())
    {
      m_overlapped_driving_link_length_portion_vec.clear ();
    }
  for (auto _driving_link : m_overlapped_driving_link_vec)
    {
      if (m_from_busstop->m_link_ID == m_to_busstop->m_link_ID
          && m_overlapped_driving_link_vec.size () == 1
          && m_from_busstop->m_link_ID == _driving_link->m_link_ID)
        {
          // m_from_busstop and m_to_busstop on the same link
          IAssert (m_to_busstop->m_link_loc > m_from_busstop->m_link_loc);
          m_overlapped_driving_link_length_portion_vec.push_back (
            (m_to_busstop->m_link_loc - m_from_busstop->m_link_loc)
            / _driving_link->m_length);
          break;
        }
      if (m_from_busstop->m_link_ID == _driving_link->m_link_ID)
        {
          m_overlapped_driving_link_length_portion_vec.push_back (
            1 - m_from_busstop->m_link_loc / _driving_link->m_length);
        }
      else if (m_to_busstop->m_link_ID == _driving_link->m_link_ID)
        {
          m_overlapped_driving_link_length_portion_vec.push_back (
            m_to_busstop->m_link_loc / _driving_link->m_length);
        }
      else
        {
          m_overlapped_driving_link_length_portion_vec.push_back (TFlt (1));
        }
    }
  IAssert (m_overlapped_driving_link_length_portion_vec.size ()
           == m_overlapped_driving_link_vec.size ());
  return 0;
}

TFlt
MNM_Bus_Link::get_link_tt (bool count_runs)
{
  IAssert (m_route_ID > -1 && m_from_busstop->m_route_ID == m_route_ID
           && m_to_busstop->m_route_ID == m_route_ID);
  IAssert (m_from_busstop->m_link_ID
           == m_overlapped_driving_link_vec.front ()->m_link_ID);
  IAssert (m_to_busstop->m_link_ID
           == m_overlapped_driving_link_vec.back ()->m_link_ID);
  IAssert (!m_overlapped_driving_link_vec.empty ());
  IAssert (m_overlapped_driving_link_length_portion_vec.size ()
           == m_overlapped_driving_link_vec.size ());

  TFlt _tt = TFlt (0);
  TFlt _driving_link_tt;

  // waiting time
  // assume cc changes only when additional vehicles pass
  // if (MNM_Ults::approximate_less_than(m_from_busstop -> m_N_in_bus ->
  // m_recorder.back().second, m_from_busstop -> m_total_bus)) {

  //     TFlt _cc = m_from_busstop -> m_N_in_bus -> m_recorder.back().second;
  //     int _cc_floor = int(_cc);
  //     int _cc_ceil = _cc_floor + 1;

  //     TFlt _threshold;
  //     if (MNM_Ults::approximate_equal(_cc, TFlt(_cc_ceil))) {
  //         _threshold = _cc_ceil + 1;
  //     }
  //     else {
  //         _threshold = _cc_ceil;
  //     }
  //     MNM_Busstop_Virtual *_busstop = m_from_busstop;
  //     // check if next bus has reached any upstream bus stop
  //     while (_busstop -> m_bus_in_link != nullptr) {
  //         // bus time for upstream bus links
  //         for (size_t i=0; i < _busstop -> m_bus_in_link ->
  //         m_overlapped_driving_link_vec.size(); ++i) {
  //             _driving_link_tt = _busstop -> m_bus_in_link ->
  //             m_overlapped_driving_link_vec[i] -> get_link_tt(); _tt +=
  //             _driving_link_tt * _busstop -> m_bus_in_link ->
  //             m_overlapped_driving_link_length_portion_vec[i];
  //         }
  //         if (MNM_Ults::approximate_less_than(_busstop -> m_bus_in_link ->
  //         m_from_busstop -> m_N_in_bus -> m_recorder.back().second,
  //         _threshold)) {
  //             _busstop = _busstop -> m_bus_in_link -> m_from_busstop;
  //         }
  //         else {
  //             break;
  //         }
  //     }
  //     // bus has not been released yet
  //     if (_busstop -> m_bus_in_link == nullptr) {
  //         TFlt* _demand = _busstop -> m_origin -> m_demand_bus.find(_busstop
  //         -> m_dest) -> second.find(m_route_ID) -> second;
  //         // m_origin -> m_frequency is 180 5-s interval, not 15 1-min
  //         interval TInt _num_minute = (int) _busstop -> m_origin ->
  //         m_frequency / (60 / 5); TFlt _tot_released_bus = 0; for (int i = 0;
  //         i < _busstop -> m_origin -> m_current_assign_interval + 1; i++) {
  //             _tot_released_bus += floor(_demand[i] * _busstop -> m_origin ->
  //             m_flow_scalar) / _busstop -> m_origin -> m_flow_scalar;
  //         }
  //         if (MNM_Ults::approximate_less_than(_tot_released_bus, _threshold))
  //         {
  //             for (int i = _busstop -> m_origin -> m_current_assign_interval
  //             + 1; i < _busstop -> m_origin -> m_max_assign_interval *
  //             _num_minute; i++) {
  //                 _tot_released_bus += floor(_demand[i] * _busstop ->
  //                 m_origin -> m_flow_scalar) / _busstop -> m_origin ->
  //                 m_flow_scalar; if
  //                 (!MNM_Ults::approximate_less_than(_tot_released_bus,
  //                 _threshold)) {
  //                     // min -> s
  //                     _tt += (int) (i - _busstop -> m_origin ->
  //                     m_current_assign_interval) * 60; break;
  //                 }
  //             }
  //         }
  //     }
  // }
  // else {
  //     return std::numeric_limits<double>::infinity();
  // }

  if (count_runs)
    {
      if (!MNM_Ults::approximate_less_than (m_from_busstop->m_N_in_bus
                                              ->m_recorder.back ()
                                              .second,
                                            m_from_busstop->m_total_bus))
        {
          // return std::numeric_limits<double>::infinity();
          return 2 * m_from_busstop->m_origin->m_max_assign_interval
                 * m_from_busstop->m_origin->m_frequency;
        }
    }

  // bus time for this bus link
  for (size_t i = 0; i < m_overlapped_driving_link_vec.size (); ++i)
    {
      // TODO: get_link_tt() only for car, not accurate for truck
      _driving_link_tt = m_overlapped_driving_link_vec[i]->get_link_tt ();
      _tt += _driving_link_tt * m_overlapped_driving_link_length_portion_vec[i];
    }

  // if (_tt < m_fftt) _tt = m_fftt;
  return _tt; // seconds
}

int
MNM_Bus_Link::clear_incoming_array (TInt timestamp)
{
  MNM_Passenger *_passenger;
  auto _passenger_it = m_incoming_array.begin ();
  while (_passenger_it != m_incoming_array.end ())
    {
      _passenger = *_passenger_it;
      IAssert (_passenger->get_current_link ()->m_link_ID == m_link_ID);
      _passenger->m_waiting_time = 0;
      m_passenger_queue.insert (
        std::pair<MNM_Passenger *, TInt> (_passenger, TInt (0)));
      _passenger_it = m_incoming_array.erase (_passenger_it);
    }
  return 0;
}

int
MNM_Bus_Link::evolve (TInt timestamp)
{
  // TODO: use real bus schedule
  // use instant as approximate bus travel time
  TInt _max_stamp = int (get_link_tt (false) / m_unit_time);
  // if (_max_stamp > 2 * m_fftt / m_unit_time) {
  //     _max_stamp = int(2 * m_fftt / m_unit_time);
  // }
  auto _passenger_it = m_passenger_queue.begin ();
  while (_passenger_it != m_passenger_queue.end ())
    {
      if (_passenger_it->second >= MNM_Ults::max (0, _max_stamp - 1))
        {
          m_finished_array.push_back (_passenger_it->first);
          _passenger_it = m_passenger_queue.erase (_passenger_it); // c++ 11
        }
      else
        {
          _passenger_it->second += 1;
          _passenger_it++;
        }
    }
  return 0;
}

/**************************************************************************
                                                Walking Link Model
**************************************************************************/
MNM_Walking_Link::MNM_Walking_Link (TInt link_ID,
                                    const std::string &walking_type,
                                    const std::string &from_node_type,
                                    const std::string &to_node_type,
                                    TInt from_node_ID, TInt to_node_ID,
                                    TFlt walking_time, TFlt unit_time)
    : MNM_Transit_Link::MNM_Transit_Link (link_ID, MNM_TYPE_WALKING_MULTIMODAL,
                                          from_node_type, to_node_type,
                                          from_node_ID, to_node_ID, unit_time)
{
  m_walking_type = walking_type;
  m_fftt = walking_time; // seconds
  // MNM_Ults::round() can randomly round up or down the input
  // m_max_stamp = MNM_Ults::round(m_fftt / m_unit_time);
  // round down time, but ensures m_max_stamp >= 1
  m_max_stamp = MNM_Ults::round_down_time (m_fftt / m_unit_time);
}

MNM_Walking_Link::~MNM_Walking_Link () { ; }

int
MNM_Walking_Link::clear_incoming_array (TInt timestamp)
{
  MNM_Passenger *_passenger;
  auto _passenger_it = m_incoming_array.begin ();
  while (_passenger_it != m_incoming_array.end ())
    {
      _passenger = *_passenger_it;
      IAssert (_passenger->get_current_link ()->m_link_ID == m_link_ID);
      if (_passenger->m_waiting_time <= 0)
        {
          _passenger->m_waiting_time = 0;
          m_passenger_queue.insert (
            std::pair<MNM_Passenger *, TInt> (_passenger, TInt (0)));
          _passenger_it = m_incoming_array.erase (_passenger_it);
        }
      else
        {
          _passenger->m_waiting_time -= 1;
          _passenger_it++;
        }
    }
  return 0;
}

int
MNM_Walking_Link::evolve (TInt timestamp)
{
  TInt _max_stamp;
  if (m_walking_type == "normal")
    {
      _max_stamp = m_max_stamp;
    }
  else if (m_walking_type == "boarding")
    {
      _max_stamp = m_historical_bus_waiting_time;
    }
  else
    {
      // for alighting links
      _max_stamp = TInt (0);
    }
  auto _passenger_it = m_passenger_queue.begin ();
  while (_passenger_it != m_passenger_queue.end ())
    {
      if (_passenger_it->second >= MNM_Ults::max (0, _max_stamp - 1))
        {
          m_finished_array.push_back (_passenger_it->first);
          _passenger_it = m_passenger_queue.erase (_passenger_it); // c++ 11
        }
      else
        {
          _passenger_it->second += 1;
          _passenger_it++;
        }
    }
  return 0;
}

TFlt
MNM_Walking_Link::get_link_tt (bool count_runs)
{
  if (m_walking_type == "boarding")
    {
      return m_fftt + m_historical_bus_waiting_time * m_unit_time; // seconds
    }
  else
    {
      return m_fftt; // seconds
    }
}

/**************************************************************************
                                        Multimodal Point-Queue Model
**************************************************************************/

MNM_Dlink_Pq_Multimodal::MNM_Dlink_Pq_Multimodal (
  TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
  TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
  TFlt ffs_car, TFlt ffs_truck, TFlt unit_time, TFlt veh_convert_factor,
  TFlt flow_scalar)
    : MNM_Dlink_Pq_Multiclass::MNM_Dlink_Pq_Multiclass (ID, number_of_lane,
                                                        length,
                                                        lane_hold_cap_car,
                                                        lane_hold_cap_truck,
                                                        lane_flow_cap_car,
                                                        lane_flow_cap_truck,
                                                        ffs_car, ffs_truck,
                                                        unit_time,
                                                        veh_convert_factor,
                                                        flow_scalar)
{
  m_link_type = MNM_TYPE_PQ_MULTIMODAL;
  // original unordered_map is replaced with deque to track the order of buses
  m_veh_queue = std::deque<std::pair<MNM_Veh *, TInt>> ();
  m_busstop_vec = std::vector<MNM_Busstop_Virtual *> ();
  m_busstop_timeloc_map = std::unordered_map<MNM_Busstop_Virtual *, TInt> ();
}

MNM_Dlink_Pq_Multimodal::~MNM_Dlink_Pq_Multimodal ()
{
  m_veh_queue.clear ();
  m_busstop_vec.clear ();
  m_busstop_timeloc_map.clear ();
}

TFlt
MNM_Dlink_Pq_Multimodal::get_link_flow_car ()
{
  TFlt _flow_car = MNM_Dlink_Pq_Multiclass::get_link_flow_car ();
  return _flow_car;
}

TFlt
MNM_Dlink_Pq_Multimodal::get_link_flow_truck ()
{
  TFlt _flow_truck = MNM_Dlink_Pq_Multiclass::get_link_flow_truck ();
  return _flow_truck;
}

int
MNM_Dlink_Pq_Multimodal::clear_incoming_array (TInt timestamp)
{
  // add zero bus count to cc to ensure the cc has the right length after DNL
  // if (!m_busstop_vec.empty()) {
  //     for (auto _busstop : m_busstop_vec) {
  //         if (_busstop -> m_N_in_bus != nullptr) {
  //             _busstop -> m_N_in_bus -> add_increment(std::pair<TFlt,
  //             TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
  //         }
  //         if (_busstop -> m_N_out_bus != nullptr) {
  //             _busstop -> m_N_out_bus -> add_increment(std::pair<TFlt,
  //             TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
  //         }
  //         if (_busstop -> m_bus_in_link != nullptr && _busstop ->
  //         m_bus_in_link -> m_N_out != nullptr) {
  //             _busstop -> m_bus_in_link -> m_N_out ->
  //             add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1),
  //             TFlt(0)));
  //         }
  //         if (_busstop -> m_bus_out_link != nullptr && _busstop ->
  //         m_bus_out_link -> m_N_in != nullptr) {
  //             _busstop -> m_bus_out_link -> m_N_in ->
  //             add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1),
  //             TFlt(0)));
  //         }
  //     }
  // }

  MNM_Veh_Multimodal *_veh;
  TFlt _to_be_moved = get_link_supply () * m_flow_scalar;

  auto _veh_it = m_incoming_array.begin ();
  while (_veh_it != m_incoming_array.end () && _to_be_moved > 0)
    {
      _veh = dynamic_cast<MNM_Veh_Multimodal *> (*_veh_it);

      // mobility service waiting for pickup at origins
      if (dynamic_cast<MNM_DMOND *> (m_from_node) != nullptr)
        {
          if (_veh->get_ispnr () && _veh->m_waiting_time > 0)
            {
              IAssert (_veh->m_class == 0);
              _veh->m_waiting_time -= 1;
              _veh_it++;
              continue;
            }
          else
            {
              _veh->m_waiting_time = TInt (0);
            }
        }

      // pnr
      // _veh -> m_waiting_time = TInt(0);

      _veh_it = m_incoming_array.erase (_veh_it);
      // node -> evolve first, then link -> clear_incoming_array(), then link ->
      // evolve() this actually leads to vehicle spending m_max_stamp + 1 in
      // this link
      // so we use m_max_stamp - 1 in link -> evolve() to ensure vehicle spends
      // m_max_stamp in this link when m_max_stamp > 1 and 1 when m_max_stamp =
      // 0
      m_veh_queue.emplace_back (_veh, TInt (0));
      // m_veh_pool.insert({_veh, TInt(0)});
      if (_veh->m_class == 0)
        {
          // printf("car\n");
          // m_volume_car += 1;
          _to_be_moved -= 1;
        }
      else
        {
          // printf("truck\n");
          // m_volume_truck += 1;
          // _to_be_moved -= m_veh_convert_factor;
          _to_be_moved -= 1;

          // update bus cc at bus stop
          if ((_veh->m_bus_route_ID != TInt (-1)) && (!m_busstop_vec.empty ()))
            {
              for (auto _busstop_it : m_busstop_timeloc_map)
                {
                  MNM_Busstop_Virtual *_busstop = _busstop_it.first;
                  if (_veh->m_bus_route_ID == _busstop->m_route_ID
                      && _busstop_it.second == TInt (0))
                    {
                      if (_veh->m_stopped_intervals != 0)
                        {
                          throw std::runtime_error (
                            "vehicle's m_stopped_intervals is not reset\n");
                        }
                      _busstop->receive_bus (timestamp, _veh);
                      break;
                    }
                }
            }
        }
    }

  //    while (!m_incoming_array.empty()) {
  //        if (_to_be_moved > 0){
  //            _veh = dynamic_cast<MNM_Veh_Multimodal
  //            *>(m_incoming_array.front()); m_incoming_array.pop_front();
  //            m_veh_queue.emplace_back(_veh, TInt(0));
  ////            m_veh_pool.insert({_veh, TInt(0)});
  //            if (_veh -> m_class == 0) {
  //                //printf("car\n");
  //                // m_volume_car += 1;
  //                _to_be_moved -= 1;
  //            }
  //            else {
  //                //printf("truck\n");
  //                // m_volume_truck += 1;
  //                // _to_be_moved -= m_veh_convert_factor;
  //                _to_be_moved -= 1;
  //
  //                // update bus cc at bus stop
  //                if ((_veh -> m_bus_route_ID != TInt(-1)) &&
  //                (!m_busstop_vec.empty())) {
  //                    for (auto _busstop_it : m_busstop_timeloc_map) {
  //                        MNM_Busstop_Virtual* _busstop = _busstop_it.first;
  //                        if (_veh -> m_bus_route_ID == _busstop -> m_route_ID
  //                        && _busstop_it.second == TInt(0)) {
  //                            _busstop -> receive_bus(timestamp, _veh);
  //                        }
  //                    }
  //                }
  //            }
  //        }
  //        else {
  //            break;
  //        }
  //    }

  m_volume_car = 0;
  m_volume_truck = 0;
  for (auto _veh_it : m_veh_pool)
    {
      auto *_veh_multimodal
        = dynamic_cast<MNM_Veh_Multimodal *> (_veh_it.first);
      if (_veh_multimodal->m_class == 0)
        m_volume_car += 1;
      if (_veh_multimodal->m_class == 1)
        m_volume_truck += 1;
    }
  for (auto _veh_it : m_finished_array)
    {
      auto *_veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh_it);
      if (_veh_multimodal->m_class == 0)
        m_volume_car += 1;
      if (_veh_multimodal->m_class == 1)
        m_volume_truck += 1;
    }
  // printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
  return 0;
}

int
MNM_Dlink_Pq_Multimodal::evolve (TInt timestamp)
{
  MNM_Veh *_veh;
  MNM_Veh_Multimodal *_veh_multimodal;
  std::deque<MNM_Veh *> _from_queue = std::deque<MNM_Veh *> ();
  std::deque<MNM_Veh *> _held_queue = std::deque<MNM_Veh *> ();
  TInt _num_car = 0, _num_truck = 0;
  bool _held;

  //    std::unordered_map<MNM_Veh*, TInt>::iterator _que_it =
  //    m_veh_pool.begin(); while (_que_it != m_veh_pool.end())
  auto _que_it = m_veh_queue.begin ();
  while (_que_it != m_veh_queue.end ())
    {
      _veh = _que_it->first;
      _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
      IAssert (_from_queue.empty ());
      _from_queue.push_back (_veh);
      _held = false;
      // determine whether to hold the bus
      if ((_veh_multimodal->m_bus_route_ID != TInt (-1))
          && (!m_busstop_vec.empty ()))
        {
          // if it is bus, update cc
          for (auto _busstop_it : m_busstop_timeloc_map)
            {
              MNM_Busstop_Virtual *_busstop = _busstop_it.first;
              if (_veh_multimodal->m_bus_route_ID == _busstop->m_route_ID
                  && _que_it->second == _busstop_it.second)
                {
                  _held
                    = _busstop->hold_bus (_veh, _veh_multimodal, &_from_queue,
                                          &_held_queue, int (m_flow_scalar));
                  if (_held)
                    {
                      // m_stopped_intervals has been updated in hold_bus(), so
                      // when stopped intervals - 1 >= time for opening and
                      // closing door
                      if (_veh_multimodal->m_stopped_intervals
                          > _veh_multimodal->m_boarding_lost_intervals)
                        {
                          _veh_multimodal->board_and_alight (timestamp,
                                                             _busstop);
                        }
                    }
                  break;
                }
            }
        }

      if (!_held)
        {
          _veh_multimodal->m_stopped_intervals = 0;
          _from_queue.pop_front ();
          // update bus cc at bus stop
          if ((_veh_multimodal->m_bus_route_ID != TInt (-1))
              && (!m_busstop_vec.empty ()))
            {
              // first release then receive
              for (auto _busstop_it : m_busstop_timeloc_map)
                {
                  MNM_Busstop_Virtual *_busstop = _busstop_it.first;
                  if (_veh_multimodal->m_bus_route_ID == _busstop->m_route_ID
                      && _que_it->second == _busstop_it.second)
                    {
                      _busstop->release_bus (timestamp, _veh_multimodal);
                      break;
                    }
                }
              for (auto _busstop_it : m_busstop_timeloc_map)
                {
                  MNM_Busstop_Virtual *_busstop = _busstop_it.first;
                  if (_veh_multimodal->m_bus_route_ID == _busstop->m_route_ID
                      && _que_it->second + 1 == _busstop_it.second)
                    {
                      _busstop->receive_bus (timestamp, _veh_multimodal);
                      break;
                    }
                }
            }

          // we use m_max_stamp - 1 in link -> evolve() to ensure vehicle spends
          // m_max_stamp in this link when m_max_stamp > 1 and 1 when
          // m_max_stamp = 0
          if (_que_it->second >= MNM_Ults::max (0, m_max_stamp - 1))
            {
              m_finished_array.push_back (_veh);
              if (_veh->get_class () == 0)
                {
                  _num_car += 1;
                }
              else
                {
                  _num_truck += 1;
                }
              _que_it = m_veh_queue.erase (_que_it); // c++ 11
              //                _que_it = m_veh_pool.erase(_que_it); //c++ 11
            }
          else
            {
              _que_it->second += 1;
              _que_it++;
            }
        }
      else
        {
          _que_it++;
        }
    }

  _from_queue.clear ();
  _held_queue.clear ();
  // printf("car: %d, truck: %d\n", _num_car, _num_truck);
  m_tot_wait_time_at_intersection
    += m_finished_array.size () / m_flow_scalar * m_unit_time;
  TInt _count_car = 0;
  TInt _count_truck = 0;
  for (auto *_v : m_finished_array)
    {
      if (_v->get_class () == 0)
        _count_car += 1;
      if (_v->get_class () == 1)
        _count_truck += 1;
    }
  m_tot_wait_time_at_intersection_car
    += TFlt (_count_car) / m_flow_scalar * m_unit_time;
  m_tot_wait_time_at_intersection_truck
    += TFlt (_count_truck) / m_flow_scalar * m_unit_time;
  return 0;
}

/**************************************************************************
                                        Multimodal CTM Model
**************************************************************************/

MNM_Dlink_Ctm_Multimodal::MNM_Dlink_Ctm_Multimodal (
  TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
  TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
  TFlt ffs_car, TFlt ffs_truck, TFlt unit_time, TFlt veh_convert_factor,
  TFlt flow_scalar)
    : MNM_Dlink_Ctm_Multiclass::MNM_Dlink_Ctm_Multiclass (ID, number_of_lane,
                                                          length,
                                                          lane_hold_cap_car,
                                                          lane_hold_cap_truck,
                                                          lane_flow_cap_car,
                                                          lane_flow_cap_truck,
                                                          ffs_car, ffs_truck,
                                                          unit_time,
                                                          veh_convert_factor,
                                                          flow_scalar)
{
  m_link_type = MNM_TYPE_CTM_MULTIMODAL;
  m_cell_busstop_vec = std::unordered_map<
    TInt,
    std::vector<MNM_Busstop_Virtual *>> (); // set in build_busstop_factory
  m_busstop_vec
    = std::vector<MNM_Busstop_Virtual *> (); // set in build_busstop_factory
}

MNM_Dlink_Ctm_Multimodal::~MNM_Dlink_Ctm_Multimodal ()
{
  for (auto _it : m_cell_busstop_vec)
    {
      _it.second.clear ();
    }
  m_cell_busstop_vec.clear ();

  m_busstop_vec.clear ();
}

TFlt
MNM_Dlink_Ctm_Multimodal::get_link_flow_car ()
{
  TFlt _flow_car = MNM_Dlink_Ctm_Multiclass::get_link_flow_car ();
  return _flow_car;
}

TFlt
MNM_Dlink_Ctm_Multimodal::get_link_flow_truck ()
{
  TFlt _flow_truck = MNM_Dlink_Ctm_Multiclass::get_link_flow_truck ();
  return _flow_truck;
}

int
MNM_Dlink_Ctm_Multimodal::clear_incoming_array (TInt timestamp)
{
  // if (get_link_supply() * m_flow_scalar < m_incoming_array.size()){
  // 	printf("Wrong incoming array size\n");
  // 	exit(-1);
  // }

  // add zero bus count to cc to ensure the cc has the right length after DNL
  // if (!m_busstop_vec.empty()) {
  //     for (auto _busstop : m_busstop_vec) {
  //         if (_busstop -> m_N_in_bus != nullptr) {
  //             _busstop -> m_N_in_bus -> add_increment(std::pair<TFlt,
  //             TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
  //         }
  //         if (_busstop -> m_N_out_bus != nullptr) {
  //             _busstop -> m_N_out_bus -> add_increment(std::pair<TFlt,
  //             TFlt>(TFlt(timestamp+1), TFlt(0)/m_flow_scalar));
  //         }
  //         if (_busstop -> m_bus_in_link != nullptr && _busstop ->
  //         m_bus_in_link -> m_N_out != nullptr) {
  //             _busstop -> m_bus_in_link -> m_N_out ->
  //             add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1),
  //             TFlt(0)));
  //         }
  //         if (_busstop -> m_bus_out_link != nullptr && _busstop ->
  //         m_bus_out_link -> m_N_in != nullptr) {
  //             _busstop -> m_bus_out_link -> m_N_in ->
  //             add_increment(std::pair<TFlt, TFlt>(TFlt(timestamp+1),
  //             TFlt(0)));
  //         }
  //     }
  // }

  MNM_Veh_Multimodal *_veh;
  size_t _cur_size = m_incoming_array.size ();
  for (size_t i = 0; i < _cur_size; ++i)
    {
      _veh = dynamic_cast<MNM_Veh_Multimodal *> (m_incoming_array.front ());
      m_incoming_array.pop_front ();
      if (_veh->m_class == TInt (0))
        {
          // printf("car\n");
          m_cell_array[0]->m_veh_queue_car.push_back (_veh);
        }
      else
        {
          // printf("truck\n");
          m_cell_array[0]->m_veh_queue_truck.push_back (_veh);

          if ((_veh->m_bus_route_ID != TInt (-1)) && (!m_busstop_vec.empty ()))
            {
              // check if the bus stop at this cell
              for (auto *_busstop : m_busstop_vec)
                {
                  if (_busstop->m_cell_ID == 0)
                    {
                      if (_veh->m_bus_route_ID == _busstop->m_route_ID)
                        {
                          if (_veh->m_stopped_intervals != 0)
                            {
                              throw std::runtime_error (
                                "vehicle's m_stopped_intervals is not reset");
                            }
                          _busstop->receive_bus (timestamp, _veh);
                          break;
                        }
                    }
                }
            }
        }
      _veh->m_visual_position_on_link
        = float (1) / float (m_num_cells)
          / float (2); // initial position at first cell
    }
  m_cell_array[0]->m_volume_car = m_cell_array[0]->m_veh_queue_car.size ();
  m_cell_array[0]->m_volume_truck = m_cell_array[0]->m_veh_queue_truck.size ();
  m_cell_array[0]->update_perceived_density ();

  return 0;
}

int
MNM_Dlink_Ctm_Multimodal::move_veh_queue_in_cell (
  std::deque<MNM_Veh *> *from_queue, std::deque<MNM_Veh *> *to_queue,
  TInt number_tomove, TInt timestamp, TInt cell_ID)
{
  MNM_Veh *_veh;
  MNM_Veh_Multimodal *_veh_multimodal;
  bool _held = false;
  TInt _veh_moved_count = 0;
  TInt _veh_held_count = 0;
  std::deque<MNM_Veh *> _held_queue = std::deque<MNM_Veh *> ();
  TInt _current_cell = cell_ID;

  // only for CTM link
  if (_current_cell < 0)
    {
      _current_cell = 0;
    }
  else if (_current_cell > m_num_cells - 2)
    {
      // last cell is special, handled in move_last_cell()
      _current_cell = m_num_cells - 2;
    }

  while (_veh_moved_count < number_tomove && !(from_queue->empty ()))
    {
      // for (int i = 0; i < number_tomove; ++i) {
      _veh = from_queue->front ();
      _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
      _held = false;
      if (_veh_multimodal->m_bus_route_ID != TInt (-1))
        {
          // check if the bus stop at this cell
          //            for (auto *_busstop :
          //            m_cell_busstop_vec.find(_current_cell) -> second) {
          //                std::vector<TInt>::iterator it;
          //                it = std::find(_busstop -> m_routeID_vec.begin(),
          //                _busstop -> m_routeID_vec.end(), _veh_multimodal ->
          //                m_bus_route_ID); if (it != _busstop ->
          //                m_routeID_vec.end()) {
          //                    // hold bus at bus stop for 2 intervals, 2 x 5s
          //                    = 10 s if (_veh_multimodal ->
          //                    m_stopped_intervals < 2) {
          //                        // hold the bus, move the bus to a different
          //                        location of queue from_queue -> pop_front();
          //                        _veh_moved_count += 1;
          //                        std::deque<MNM_Veh *>::iterator pos =
          //                        from_queue -> begin(); pos = from_queue ->
          //                        begin() + number_tomove - _veh_moved_count +
          //                        _veh_held_count; pos = from_queue ->
          //                        insert(pos, _veh); _veh_multimodal ->
          //                        m_stopped_intervals += 1; _held = true;
          //                        break;
          //                    }
          //                }
          //            }

          for (auto *_busstop : m_busstop_vec)
            {
              if (_busstop->m_cell_ID == _current_cell)
                {
                  if (_veh_multimodal->m_bus_route_ID == _busstop->m_route_ID)
                    {
                      // bus stop is at cell, originally passing a cell requires
                      // at least 1 interval now hold bus at bus stop for n
                      // intervals, n x 5s passing a cell requires at least n
                      // intervals

                      // only stop bus every m_flow_scalar vehicles
                      _held = _busstop->hold_bus (_veh, _veh_multimodal,
                                                  from_queue, &_held_queue,
                                                  int (m_flow_scalar));
                      if (_held)
                        {
                          _veh_held_count += 1;
                          // m_stopped_intervals has been updated in hold_bus(),
                          // so when stopped intervals - 1 >= time for opening
                          // and closing door
                          if (_veh_multimodal->m_stopped_intervals
                              > _veh_multimodal->m_boarding_lost_intervals)
                            {
                              _veh_multimodal->board_and_alight (timestamp,
                                                                 _busstop);
                            }
                        }
                      break;
                    }
                }
            }
        }

      if (!_held)
        {
          // bus can move to next cell
          from_queue->pop_front ();
          // update the vehicle position on current link. 0: at the beginning,
          // 1: at the end.
          _veh_multimodal->m_visual_position_on_link
            += float (1) / float (m_num_cells);
          if (_veh_multimodal->m_visual_position_on_link > 0.99)
            _veh_multimodal->m_visual_position_on_link = 0.99;
          to_queue->push_back (_veh);
          _veh_moved_count += 1;
          // reset
          _veh_multimodal->m_stopped_intervals = 0;
          _held = false;

          if ((_veh_multimodal->m_class == 1)
              && (_veh_multimodal->m_bus_route_ID != TInt (-1))
              && (!m_busstop_vec.empty ()))
            {
              // check if this cell or next cell has busstops
              // first release then receive
              for (auto *_busstop : m_busstop_vec)
                {
                  if (_busstop->m_cell_ID == _current_cell)
                    {
                      if (_veh_multimodal->m_bus_route_ID
                          == _busstop->m_route_ID)
                        {
                          _busstop->release_bus (timestamp, _veh_multimodal);
                          break;
                        }
                    }
                }
              for (auto *_busstop : m_busstop_vec)
                {
                  if (_busstop->m_cell_ID == _current_cell + 1)
                    {
                      if (_veh_multimodal->m_bus_route_ID
                          == _busstop->m_route_ID)
                        {
                          _busstop->receive_bus (timestamp, _veh_multimodal);
                          break;
                        }
                    }
                }
            }
        }
    }
  IAssert (int (_held_queue.size ()) == _veh_held_count);
  if (!_held_queue.empty ())
    {
      for (int i = 0; i < _veh_held_count; ++i)
        {
          _veh = _held_queue.back ();
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          IAssert (_veh_multimodal->m_class == 1
                   && _veh_multimodal->m_bus_route_ID != -1);
          _held_queue.pop_back ();
          from_queue->push_front (_veh);
        }
    }
  _held_queue.clear ();
  return 0;
}

int
MNM_Dlink_Ctm_Multimodal::move_veh_queue_in_last_cell (TInt timestamp)
{
  TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1]->m_out_veh_car;
  TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1]->m_out_veh_truck;
  TInt _num_veh_tomove_total = _num_veh_tomove_car + _num_veh_tomove_truck;
  TFlt _pstar = TFlt (_num_veh_tomove_car) / TFlt (_num_veh_tomove_total);
  MNM_Veh *_veh;
  MNM_Veh_Multimodal *_veh_multimodal;
  bool _held;
  TFlt _r; // randomly mix truck and car
  TInt _car_moved_count = 0;
  TInt _truck_moved_count = 0;

  TInt _veh_held_count = 0;
  std::deque<MNM_Veh *> _held_queue = std::deque<MNM_Veh *> ();

  while ((_num_veh_tomove_car > _car_moved_count)
         || ((_num_veh_tomove_truck > _truck_moved_count)
             && (!(m_cell_array[m_num_cells - 1]->m_veh_queue_truck.empty ()))))
    {
      _held = false;
      _r = MNM_Ults::rand_flt ();
      // probability = _pstar to move a car
      if (_r < _pstar)
        {
          // still has car to move
          if (_num_veh_tomove_car > _car_moved_count)
            {
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_car.front ();
              m_cell_array[m_num_cells - 1]->m_veh_queue_car.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error (
                    "Dlink_CTM_Multimodal, car cannot move out CTM link");
                }
              _car_moved_count += 1;
            }
          // no car to move, move a truck
          else
            {
              if (m_cell_array[m_num_cells - 1]->m_veh_queue_truck.empty ())
                {
                  continue;
                }
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.front ();

              _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
              if (_veh_multimodal->m_bus_route_ID != TInt (-1))
                {
                  // check if the bus stop at this cell
                  for (auto *_busstop : m_busstop_vec)
                    {
                      if (_busstop->m_cell_ID == m_num_cells - 1)
                        {
                          if (_veh_multimodal->m_bus_route_ID
                              == _busstop->m_route_ID)
                            {
                              // only stop bus every m_flow_scalar vehicles
                              _held
                                = _busstop
                                    ->hold_bus (_veh, _veh_multimodal,
                                                &(m_cell_array[m_num_cells - 1]
                                                    ->m_veh_queue_truck),
                                                &_held_queue,
                                                int (m_flow_scalar));
                              if (_held)
                                {
                                  _veh_held_count += 1;
                                  // m_stopped_intervals has been updated in
                                  // hold_bus(), so when stopped intervals - 1
                                  // >= time for opening and closing door
                                  if (_veh_multimodal->m_stopped_intervals
                                      > _veh_multimodal
                                          ->m_boarding_lost_intervals)
                                    {
                                      _veh_multimodal
                                        ->board_and_alight (timestamp,
                                                            _busstop);
                                    }
                                }
                              break;
                            }
                        }
                    }
                }

              if (!_held)
                {
                  m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front ();
                  if (_veh->has_next_link ())
                    {
                      // reset
                      _veh_multimodal->m_stopped_intervals = 0;
                      _held = false;
                      m_finished_array.push_back (_veh);

                      if ((_veh_multimodal->m_class == 1)
                          && (_veh_multimodal->m_bus_route_ID != TInt (-1)))
                        {
                          // check if this cell has busstops
                          for (auto *_busstop : m_busstop_vec)
                            {
                              if (_busstop->m_cell_ID == m_num_cells - 1)
                                {
                                  if (_veh_multimodal->m_bus_route_ID
                                      == _busstop->m_route_ID)
                                    {
                                      _busstop->release_bus (timestamp,
                                                             _veh_multimodal);
                                      break;
                                    }
                                }
                            }
                        }
                    }
                  else
                    {
                      throw std::runtime_error (
                        "Dlink_CTM_Multimodal::Something wrong, truck cannot "
                        "move out CTM link");
                    }
                  _truck_moved_count += 1;
                }
            }
        }
      // probability = 1 - _pstar to move a truck
      else
        {
          // still has truck to move
          if (_num_veh_tomove_truck > _truck_moved_count)
            {
              if (m_cell_array[m_num_cells - 1]->m_veh_queue_truck.empty ())
                {
                  continue;
                }
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.front ();

              _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
              if (_veh_multimodal->m_bus_route_ID != TInt (-1))
                {
                  // check if the bus stop at this cell
                  for (auto *_busstop : m_busstop_vec)
                    {
                      if (_busstop->m_cell_ID == m_num_cells - 1)
                        {
                          if (_veh_multimodal->m_bus_route_ID
                              == _busstop->m_route_ID)
                            {
                              // only stop bus every m_flow_scalar vehicles
                              _held
                                = _busstop
                                    ->hold_bus (_veh, _veh_multimodal,
                                                &(m_cell_array[m_num_cells - 1]
                                                    ->m_veh_queue_truck),
                                                &_held_queue,
                                                int (m_flow_scalar));
                              if (_held)
                                {
                                  _veh_held_count += 1;
                                  // m_stopped_intervals has been updated in
                                  // hold_bus(), so when stopped intervals -1 >=
                                  // time for opening and closing door
                                  if (_veh_multimodal->m_stopped_intervals
                                      > _veh_multimodal
                                          ->m_boarding_lost_intervals)
                                    {
                                      _veh_multimodal
                                        ->board_and_alight (timestamp,
                                                            _busstop);
                                    }
                                }
                              break;
                            }
                        }
                    }
                }

              if (!_held)
                {
                  m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front ();
                  if (_veh->has_next_link ())
                    {
                      // reset
                      _veh_multimodal->m_stopped_intervals = 0;
                      _held = false;
                      m_finished_array.push_back (_veh);

                      if ((_veh_multimodal->m_class == 1)
                          && (_veh_multimodal->m_bus_route_ID != TInt (-1)))
                        {
                          // check if this cell has busstops
                          for (auto *_busstop : m_busstop_vec)
                            {
                              if (_busstop->m_cell_ID == m_num_cells - 1)
                                {
                                  if (_veh_multimodal->m_bus_route_ID
                                      == _busstop->m_route_ID)
                                    {
                                      _busstop->release_bus (timestamp,
                                                             _veh_multimodal);
                                      break;
                                    }
                                }
                            }
                        }
                    }
                  else
                    {
                      throw std::runtime_error (
                        "Dlink_CTM_Multimodal::Something wrong, truck cannot "
                        "move out CTM link");
                    }
                  _truck_moved_count += 1;
                }
            }
          // no truck to move, move a car
          else
            {
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_car.front ();
              m_cell_array[m_num_cells - 1]->m_veh_queue_car.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error (
                    "Dlink_CTM_Multimodal::Something wrong, car cannot move "
                    "out CTM link");
                }
              _car_moved_count += 1;
            }
        }
    }
  IAssert (int (_held_queue.size ()) == _veh_held_count);
  if (!_held_queue.empty ())
    {
      for (int i = 0; i < _veh_held_count; ++i)
        {
          _veh = _held_queue.back ();
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          IAssert (_veh_multimodal->m_class == 1
                   && _veh_multimodal->m_bus_route_ID != -1);
          _held_queue.pop_back ();
          m_cell_array[m_num_cells - 1]->m_veh_queue_truck.push_front (_veh);
        }
    }
  _held_queue.clear ();
  return 0;
}

int
MNM_Dlink_Ctm_Multimodal::evolve (TInt timestamp)
{
  std::deque<MNM_Veh *>::iterator _veh_it;
  TInt _count_car = 0;
  TInt _count_truck = 0;
  TInt _count_tot_vehs = 0;

  /* update volume */
  update_out_veh ();

  TInt _num_veh_tomove_car, _num_veh_tomove_truck;
  /* previous cells */
  if (m_num_cells > 1)
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          // Car
          _num_veh_tomove_car = m_cell_array[i]->m_out_veh_car;
          move_veh_queue_in_cell (&(m_cell_array[i]->m_veh_queue_car),
                                  &(m_cell_array[i + 1]->m_veh_queue_car),
                                  _num_veh_tomove_car, timestamp, TInt (i));
          // Truck
          _num_veh_tomove_truck = m_cell_array[i]->m_out_veh_truck;
          move_veh_queue_in_cell (&(m_cell_array[i]->m_veh_queue_truck),
                                  &(m_cell_array[i + 1]->m_veh_queue_truck),
                                  _num_veh_tomove_truck, timestamp, TInt (i));
        }
    }

  /* last cell */
  move_veh_queue_in_last_cell (timestamp);
  m_tot_wait_time_at_intersection
    += TFlt (m_finished_array.size ()) / m_flow_scalar * m_unit_time;

  // if (m_link_ID == _output_link)
  // 	printf("Link %d volume after: ", int(m_link_ID));

  /* update volume */
  if (m_num_cells > 1)
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          m_cell_array[i]->m_volume_car
            = m_cell_array[i]->m_veh_queue_car.size ();
          m_cell_array[i]->m_volume_truck
            = m_cell_array[i]->m_veh_queue_truck.size ();
          // Update perceived density of the i-th cell
          m_cell_array[i]->update_perceived_density ();
          // if (m_link_ID == _output_link)
          // 	printf("(%d, %d) ", int(m_cell_array[i] -> m_volume_car),
          // int(m_cell_array[i] -> m_volume_truck));
        }
    }

  _count_car = 0;
  _count_truck = 0;
  // m_class: 0 - private car, 1 - truck
  for (_veh_it = m_finished_array.begin (); _veh_it != m_finished_array.end ();
       _veh_it++)
    {
      MNM_Veh_Multimodal *_veh = dynamic_cast<MNM_Veh_Multimodal *> (*_veh_it);
      if (_veh->m_class == 0)
        _count_car += 1;
      if (_veh->m_class == 1)
        _count_truck += 1;
    }
  m_cell_array[m_num_cells - 1]->m_volume_car
    = m_cell_array[m_num_cells - 1]->m_veh_queue_car.size () + _count_car;
  m_cell_array[m_num_cells - 1]->m_volume_truck
    = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.size () + _count_truck;
  m_cell_array[m_num_cells - 1]->update_perceived_density ();

  m_tot_wait_time_at_intersection_car
    += TFlt (_count_car) / m_flow_scalar * m_unit_time;
  m_tot_wait_time_at_intersection_truck
    += TFlt (_count_truck) / m_flow_scalar * m_unit_time;

  /* compute total volume of link, check if spill back */
  _count_tot_vehs = 0;
  for (int i = 0; i <= m_num_cells - 1; ++i)
    {
      _count_tot_vehs += m_cell_array[i]->m_volume_car;
      _count_tot_vehs += m_cell_array[i]->m_volume_truck * m_veh_convert_factor;
    }
  if (TFlt (_count_tot_vehs) / m_flow_scalar / m_length
      > m_lane_hold_cap_car * m_number_of_lane)
    {
      m_spill_back = true;
    }
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                        Link Factory
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                transit link factory
**************************************************************************/
MNM_Transit_Link_Factory::MNM_Transit_Link_Factory ()
{
  m_transit_link_map = std::unordered_map<TInt, MNM_Transit_Link *> ();
}

MNM_Transit_Link_Factory::~MNM_Transit_Link_Factory ()
{
  for (auto _map_it = m_transit_link_map.begin ();
       _map_it != m_transit_link_map.end (); _map_it++)
    {
      delete _map_it->second;
    }
  m_transit_link_map.clear ();
}

MNM_Transit_Link *
MNM_Transit_Link_Factory::make_transit_link (
  TInt ID, DLink_type_multimodal link_type, const std::string &walking_type,
  const std::string &from_node_type, const std::string &to_node_type,
  TInt from_node_ID, TInt to_node_ID, TFlt fftt, TFlt unit_time)
{
  MNM_Transit_Link *_link;
  switch (link_type)
    {
    case MNM_TYPE_BUS_MULTIMODAL:
      _link = new MNM_Bus_Link (ID, from_node_ID, to_node_ID, fftt, unit_time);
      break;
    case MNM_TYPE_WALKING_MULTIMODAL:
      _link
        = new MNM_Walking_Link (ID, walking_type, from_node_type, to_node_type,
                                from_node_ID, to_node_ID, fftt, unit_time);
      break;
    default:
      throw std::runtime_error ("Wrong link type");
    }
  if (m_transit_link_map.find (ID) == m_transit_link_map.end ())
    {
      m_transit_link_map.insert (
        std::pair<TInt, MNM_Transit_Link *> (ID, _link));
    }
  return _link;
}

MNM_Transit_Link *
MNM_Transit_Link_Factory::get_transit_link (TInt ID)
{
  auto _link_it = m_transit_link_map.find (ID);
  if (_link_it == m_transit_link_map.end ())
    {
      printf ("No such transit link ID %d\n", (int) ID);
      throw std::runtime_error (
        "Error, MNM_Transit_Link_Factory::get_transit_link, transit link does "
        "not exist");
    }
  return _link_it->second;
}

/**************************************************************************
                                                driving link factory
**************************************************************************/
MNM_Link_Factory_Multimodal::MNM_Link_Factory_Multimodal ()
    : MNM_Link_Factory_Multiclass::MNM_Link_Factory_Multiclass ()
{
  ;
}

MNM_Link_Factory_Multimodal::~MNM_Link_Factory_Multimodal () { ; }

MNM_Dlink *
MNM_Link_Factory_Multimodal::make_link_multimodal (
  TInt ID, DLink_type_multimodal link_type, TInt number_of_lane, TFlt length,
  TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
  TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
  TFlt veh_convert_factor, TFlt flow_scalar)
{
  MNM_Dlink *_link;
  switch (link_type)
    {
    case MNM_TYPE_CTM_MULTIMODAL:
      _link
        = new MNM_Dlink_Ctm_Multimodal (ID, number_of_lane, length,
                                        lane_hold_cap_car, lane_hold_cap_truck,
                                        lane_flow_cap_car, lane_flow_cap_truck,
                                        ffs_car, ffs_truck, unit_time,
                                        veh_convert_factor, flow_scalar);
      break;
    case MNM_TYPE_PQ_MULTIMODAL:
      _link
        = new MNM_Dlink_Pq_Multimodal (ID, number_of_lane, length,
                                       lane_hold_cap_car, lane_hold_cap_truck,
                                       lane_flow_cap_car, lane_flow_cap_truck,
                                       ffs_car, ffs_truck, unit_time,
                                       veh_convert_factor, flow_scalar);
      break;
    default:
      throw std::runtime_error ("Wrong link type");
    }
  m_link_map.insert ({ ID, _link });
  return _link;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Statistics
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Statistics_Lrn_Multimodal::MNM_Statistics_Lrn_Multimodal (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_ConfReader *record_config, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
  MNM_Transit_Link_Factory *transitlink_factory)
    : MNM_Statistics_Lrn_Multiclass::
        MNM_Statistics_Lrn_Multiclass (file_folder, conf_reader, record_config,
                                       od_factory, node_factory, link_factory)
{
  m_transitlink_factory = transitlink_factory;
  m_transitlink_order = std::vector<MNM_Transit_Link *> ();
  m_load_interval_tt_bus_transit = std::unordered_map<TInt, TFlt> ();
  m_record_interval_tt_bus_transit = std::unordered_map<TInt, TFlt> ();
  m_to_be_tt_bus_transit = std::unordered_map<TInt, TFlt> ();
}

MNM_Statistics_Lrn_Multimodal::~MNM_Statistics_Lrn_Multimodal ()
{
  m_transitlink_order.clear ();
  m_load_interval_tt_bus_transit.clear ();
  m_record_interval_tt_bus_transit.clear ();
  m_to_be_tt_bus_transit.clear ();
}

int
MNM_Statistics_Lrn_Multimodal::record_loading_interval_condition (
  TInt timestamp)
{
  std::string _str;
  TFlt _tt;
  if (m_record_tt && m_load_interval_tt_bus_transit_file.is_open ())
    {
      _str = std::to_string (timestamp) + " ";
      for (auto _link : m_transitlink_order)
        {
          _tt = m_load_interval_tt_bus_transit.find (_link->m_link_ID)->second;
          _str += std::to_string (_tt) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_load_interval_tt_bus_transit_file << _str;
    }
  return 0;
}

int
MNM_Statistics_Lrn_Multimodal::record_record_interval_condition (TInt timestamp)
{
  std::string _str;
  TFlt _tt;
  if (m_record_tt && m_record_interval_tt_bus_transit_file.is_open ())
    {
      _str = std::to_string (timestamp) + " ";
      for (auto _link : m_transitlink_order)
        {
          _tt
            = m_record_interval_tt_bus_transit.find (_link->m_link_ID)->second;
          _str += std::to_string (_tt) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_record_interval_tt_bus_transit_file << _str;
    }
  return 0;
}

int
MNM_Statistics_Lrn_Multimodal::init_record ()
{
  MNM_Statistics_Lrn_Multiclass::init_record ();

  TInt _link_ID;
  std::string _file_name;
  if (m_record_tt)
    {
      for (auto _link_it : m_transitlink_factory->m_transit_link_map)
        {
          _link_ID = _link_it.first;
          m_load_interval_tt_bus_transit.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
          m_record_interval_tt_bus_transit.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
          m_to_be_tt_bus_transit.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
        }

      if (m_self_config->get_int ("tt_load_automatic_rec") == 1
          || m_self_config->get_int ("tt_record_automatic_rec") == 1)
        {
          std::string _str = "Interval ";
          for (auto _link_it : m_transitlink_factory->m_transit_link_map)
            {
              _str += std::to_string (_link_it.first) + " ";
            }
          _str.pop_back ();
          _str += "\n";

          if (m_self_config->get_int ("tt_load_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_load_interval_tt_bus_transit";
              m_load_interval_tt_bus_transit_file.open (_file_name,
                                                        std::ofstream::out);
              if (!m_load_interval_tt_bus_transit_file.is_open ())
                {
                  throw std::runtime_error (
                    "Error happens when open "
                    "m_load_interval_tt_bus_transit_file");
                }
              m_load_interval_tt_bus_transit_file << _str;
            }

          if (m_self_config->get_int ("tt_record_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_record_interval_tt_bus_transit";
              m_record_interval_tt_bus_transit_file.open (_file_name,
                                                          std::ofstream::out);
              if (!m_record_interval_tt_bus_transit_file.is_open ())
                {
                  throw std::runtime_error (
                    "Error happens when open "
                    "m_record_interval_tt_bus_transit_file");
                }
              m_record_interval_tt_bus_transit_file << _str;
            }
        }
    }

  // store links in a fixed order in a vector, same as the order of creating the
  // head of the record file
  // https://stackoverflow.com/questions/18301302/is-forauto-i-unordered-map-guaranteed-to-have-the-same-order-every-time
  // The iteration order of unordered associative containers can only change
  // when rehashing as a result of a mutating operation (as described in
  // C++11 23.2.5/8). You are not modifying the container between iterations, so
  // the order will not change.
  for (auto _link_it : m_transitlink_factory->m_transit_link_map)
    {
      m_transitlink_order.push_back (_link_it.second);
    }
  return 0;
}

int
MNM_Statistics_Lrn_Multimodal::update_record (TInt timestamp)
{
  MNM_Statistics_Lrn_Multiclass::update_record (timestamp);

  MNM_Transit_Link *_link;
  TFlt _tt;
  if (m_record_tt)
    {
      if ((timestamp) % m_n == 0 || timestamp == 0)
        {
          for (auto _link_it : m_transitlink_factory->m_transit_link_map)
            {
              _link = _link_it.second;
              _tt = _link->get_link_tt ();
              m_load_interval_tt_bus_transit.find (_link->m_link_ID)->second
                = _tt;
              if (timestamp == 0)
                {
                  m_record_interval_tt_bus_transit.find (_link->m_link_ID)
                    ->second
                    = _tt;
                }
              else
                {
                  m_record_interval_tt_bus_transit.find (_link->m_link_ID)
                    ->second
                    = m_to_be_tt_bus_transit.find (_link->m_link_ID)->second
                      + _tt / TFlt (m_n);
                }
              // reset
              m_to_be_tt_bus_transit.find (_link->m_link_ID)->second = TFlt (0);
            }
        }
      else
        {
          for (auto _link_it : m_transitlink_factory->m_transit_link_map)
            {
              _link = _link_it.second;
              _tt = _link->get_link_tt ();
              m_load_interval_tt_bus_transit.find (_link->m_link_ID)->second
                = _tt;
              m_to_be_tt_bus_transit.find (_link->m_link_ID)->second
                += _tt / TFlt (m_n);
            }
        }
    }

  record_loading_interval_condition (timestamp);

  if ((timestamp) % m_n == 0 || timestamp == 0)
    {
      record_record_interval_condition (timestamp);
    }
  return 0;
}

int
MNM_Statistics_Lrn_Multimodal::post_record ()
{
  MNM_Statistics_Lrn_Multiclass::post_record ();

  if (m_record_tt)
    {
      if (m_load_interval_tt_bus_transit_file.is_open ())
        {
          m_load_interval_tt_bus_transit_file.close ();
        }
      if (m_record_interval_tt_bus_transit_file.is_open ())
        {
          m_record_interval_tt_bus_transit_file.close ();
        }
    }
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Bus Path
*******************************************************************************************************************
******************************************************************************************************************/
MNM_BusPath::MNM_BusPath (TInt route_ID) : MNM_Path::MNM_Path ()
{
  m_busstop_vec = std::deque<TInt> ();
  m_route_ID = route_ID;
}

MNM_BusPath::~MNM_BusPath ()
{
  m_link_vec.clear ();
  m_node_vec.clear ();
  m_busstop_vec.clear ();
  if (m_buffer != nullptr)
    free (m_buffer);
}

int
MNM_BusPath::get_busstop_index (MNM_Busstop_Virtual *busstop)
{
  if (std::find (m_busstop_vec.begin (), m_busstop_vec.end (),
                 busstop->m_busstop_ID)
      == m_busstop_vec.end ())
    {
      return -1;
    }
  else
    {
      for (size_t i = 0; i < m_busstop_vec.size (); i++)
        {
          if (m_busstop_vec[i] == busstop->m_busstop_ID)
            {
              return (int) i;
            }
        }
      return -1;
    }
}

TFlt
MNM_BusPath::get_busroute_fftt (MNM_Link_Factory *link_factory,
                                MNM_Busstop_Virtual *start_busstop,
                                MNM_Busstop_Virtual *end_busstop,
                                TFlt unit_interval)
{
  IAssert (start_busstop->m_route_ID == m_route_ID
           && end_busstop->m_route_ID == m_route_ID);
  auto *_start_link = dynamic_cast<MNM_Dlink_Multiclass *> (
    link_factory->get_link (start_busstop->m_link_ID));
  auto *_end_link = dynamic_cast<MNM_Dlink_Multiclass *> (
    link_factory->get_link (end_busstop->m_link_ID));
  if (_start_link->m_link_ID == _end_link->m_link_ID)
    { // start and end bus stops on the same link
      return (end_busstop->m_link_loc - start_busstop->m_link_loc)
             / _start_link->m_ffs_truck / unit_interval;
    }

  TFlt fftt;
  TFlt first_interval = (_start_link->m_length - start_busstop->m_link_loc)
                        / _start_link->m_ffs_truck / unit_interval;
  TFlt last_interval
    = end_busstop->m_link_loc / _end_link->m_ffs_truck / unit_interval;
  auto _start_it
    = find (m_link_vec.begin (), m_link_vec.end (), start_busstop->m_link_ID);
  auto _end_it
    = find (m_link_vec.begin (), m_link_vec.end (), end_busstop->m_link_ID);
  if (_start_it == m_link_vec.end ())
    {
      throw std::runtime_error (
        "Error, MNM_BusPath::get_busroute_fftt, start bus stop not on "
        "this route");
    }
  if (_end_it == m_link_vec.end ())
    {
      throw std::runtime_error (
        "Error, MNM_BusPath::get_busroute_fftt, end bus stop not on this "
        "route");
    }
  fftt = first_interval + last_interval;
  if (_start_it - m_link_vec.begin () + 1 == _end_it - m_link_vec.begin ())
    { // start and end bus stops on two consecutive links
      return fftt;
    }

  for (int i = _start_it - m_link_vec.begin () + 1;
       i < _end_it - m_link_vec.begin (); ++i)
    {
      auto _link = dynamic_cast<MNM_Dlink_Multiclass *> (
        link_factory->get_link (m_link_vec[i]));
      fftt += _link->m_length / _link->m_ffs_truck / unit_interval;
    }
  return fftt;
}

TFlt
MNM_BusPath::get_busroute_tt (TFlt start_time, MNM_Link_Factory *link_factory,
                              MNM_Busstop_Virtual *start_busstop,
                              MNM_Busstop_Virtual *end_busstop,
                              TFlt unit_interval, TInt end_loading_timestamp)
{
  IAssert (start_busstop->m_route_ID == m_route_ID
           && end_busstop->m_route_ID == m_route_ID);
  // TFlt fftt = get_busroute_fftt(link_factory, start_busstop, end_busstop,
  // unit_interval); // intervals

  TFlt _waiting_time
    = MNM_DTA_GRADIENT::get_bus_waiting_time (start_busstop, start_time + 1,
                                              unit_interval,
                                              end_loading_timestamp);
  if (std::isinf (_waiting_time))
    {
      // return std::numeric_limits<double>::infinity();
      return 2 * end_loading_timestamp;
    }
  TFlt _true_start_time = start_time + _waiting_time;

  TFlt _last_valid_time
    = MNM_DTA_GRADIENT::get_last_valid_time_bus (start_busstop->m_N_in_bus,
                                                 end_busstop->m_N_in_bus,
                                                 end_loading_timestamp);

  TFlt _bus_time
    = MNM_DTA_GRADIENT::get_travel_time_from_cc (_true_start_time + 1,
                                                 start_busstop->m_N_in_bus,
                                                 end_busstop->m_N_in_bus,
                                                 _last_valid_time, -1);
  if (_bus_time < 0)
    {
      // return std::numeric_limits<double>::infinity();
      return 2 * end_loading_timestamp;
    }
  return _waiting_time + _bus_time;
}

TFlt
MNM_BusPath::get_whole_busroute_tt (TFlt start_time,
                                    MNM_Link_Factory *link_factory,
                                    MNM_Busstop_Factory *busstop_factory,
                                    TFlt unit_interval,
                                    TInt end_loading_timestamp)
{
  TInt _start_busstop_ID = m_busstop_vec.front ();
  TInt _end_busstop_ID = m_busstop_vec.back ();
  MNM_Busstop_Virtual *_start_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
    busstop_factory->get_busstop (_start_busstop_ID));
  MNM_Busstop_Virtual *_end_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
    busstop_factory->get_busstop (_end_busstop_ID));

  TFlt tt
    = get_busroute_tt (start_time, link_factory, _start_busstop, _end_busstop,
                       unit_interval, end_loading_timestamp);

  return tt;
}

std::string
MNM_BusPath::busstop_vec_to_string ()
{
  std::string _s;
  for (TInt node_ID : m_busstop_vec)
    {
      _s += std::to_string (node_ID) + " ";
    }
  _s.pop_back ();
  _s += "\n";
  return _s;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                PnR Path
*******************************************************************************************************************
******************************************************************************************************************/
MNM_PnR_Path::MNM_PnR_Path (TInt path_ID, TInt mid_parking_lot_ID,
                            TInt mid_dest_node_ID, MNM_Path *driving_path,
                            MNM_Path *transit_path)
    : MNM_Path::MNM_Path ()
{
  m_path_ID = path_ID;
  m_mid_parking_lot_ID = mid_parking_lot_ID;
  m_mid_dest_node_ID = mid_dest_node_ID;
  m_driving_path = driving_path;
  m_transit_path = transit_path;
}

MNM_PnR_Path::~MNM_PnR_Path ()
{
  delete m_driving_path;
  delete m_transit_path;
}

bool
MNM_PnR_Path::is_equal (MNM_Path *path)
{
  auto *_pnr_path = dynamic_cast<MNM_PnR_Path *> (path);
  if (_pnr_path == nullptr)
    {
      return false;
    }
  if (m_mid_dest_node_ID != _pnr_path->m_mid_dest_node_ID
      || m_mid_parking_lot_ID != _pnr_path->m_mid_parking_lot_ID)
    {
      return false;
    }
  if (!(*m_driving_path == *(_pnr_path->m_driving_path))
      || !(*m_transit_path == *(_pnr_path->m_transit_path)))
    {
      return false;
    }
  return true;
}

bool
MNM_PnR_Path::is_link_in (TInt link_ID)
{
  return m_driving_path->is_link_in (link_ID)
         || m_transit_path->is_link_in (link_ID);
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                PnR Pathset
*******************************************************************************************************************
******************************************************************************************************************/
MNM_PnR_Pathset::MNM_PnR_Pathset () : MNM_Pathset::MNM_Pathset () { ; };

MNM_PnR_Pathset::~MNM_PnR_Pathset () { ; }

bool
MNM_PnR_Pathset::is_in (MNM_Path *path)
{
  auto *_pnr_path = dynamic_cast<MNM_PnR_Path *> (path);
  if (_pnr_path == nullptr)
    {
      return false;
    }
  else
    {
      for (auto _tmp_path : m_path_vec)
        {
          if (_pnr_path->is_equal (_tmp_path))
            {
              return true;
            }
        }
      return false;
    }
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Routing
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                          Bus Fixed Routing
**************************************************************************/
MNM_Routing_Bus::MNM_Routing_Bus (PNEGraph &driving_graph,
                                  MNM_OD_Factory *od_factory,
                                  MNM_Node_Factory *node_factory,
                                  MNM_Link_Factory *link_factory,
                                  Bus_Path_Table *bus_path_table,
                                  TInt route_frq, TInt buffer_length,
                                  TInt veh_class)
    : MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed (driving_graph,
                                                            od_factory,
                                                            node_factory,
                                                            link_factory,
                                                            route_frq,
                                                            buffer_length,
                                                            veh_class)
{
  m_veh_class = 1; // bus is treated as truck
  m_bus_path_table = bus_path_table;
}

MNM_Routing_Bus::~MNM_Routing_Bus ()
{
  for (auto _map_it : m_tracker)
    {
      _map_it.second->clear ();
      delete _map_it.second;
    }
  m_tracker.clear ();

  // clear bus_path_table
  if ((m_bus_path_table != nullptr) && (!m_bus_path_table->empty ()))
    {
      // printf("Address of m_bus_path_table is %p\n", (void
      // *)m_bus_path_table); printf("%d\n", m_bus_path_table -> size());
      for (auto _it : *m_bus_path_table)
        { // origin
          for (auto _it_it : *(_it.second))
            { // destination
              for (auto _it_it_it : *(_it_it.second))
                {                          // route ID
                  delete _it_it_it.second; // bus_path
                }
              _it_it.second->clear ();
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_bus_path_table->clear ();
      delete m_bus_path_table;
    }
}

// register each vehicle with a route based on the portion of path flow
int
MNM_Routing_Bus::register_veh (MNM_Veh *veh, bool track)
{
  // printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
  // printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);

  MNM_Veh_Multimodal *_veh_multimodal
    = dynamic_cast<MNM_Veh_Multimodal *> (veh);
  IAssert (_veh_multimodal != nullptr);
  MNM_BusPath *_route_path
    = m_bus_path_table
        ->find (_veh_multimodal->get_origin ()->m_origin_node->m_node_ID)
        ->second
        ->find (_veh_multimodal->get_destination ()->m_dest_node->m_node_ID)
        ->second->find (_veh_multimodal->m_bus_route_ID)
        ->second;
  // printf("1\n");
  // note m_path_vec is an ordered vector, not unordered

  // printf("3\n");
  if (_route_path == nullptr)
    {
      throw std::runtime_error ("Wrong bus route in register_veh!");
    }
  if (track)
    {
      std::deque<TInt> *_link_queue = new std::deque<TInt> ();
      // copy links in the route to _link_queue
      // https://www.cplusplus.com/reference/iterator/back_inserter/
      std::copy (_route_path->m_link_vec.begin (),
                 _route_path->m_link_vec.end (),
                 std::back_inserter (*_link_queue));
      // printf("old link q is %d, New link queue is %d\n", _route_path ->
      // m_link_vec.size(), _link_queue -> size());
      m_tracker.insert (
        std::pair<MNM_Veh *, std::deque<TInt> *> (veh, _link_queue));
    }
  veh->m_path = _route_path; // base vehicle class
  return 0;
}

int
MNM_Routing_Bus::init_routing (Path_Table *driving_path_table)
{
  return 0;
}

int
MNM_Routing_Bus::update_routing (TInt timestamp)
{
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh_Multimodal *_veh_multimodal;
  TInt _cur_ass_int;

  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      _origin = _origin_it.second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh : _origin_node->m_in_veh_queue)
        {
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          // if success, change _veh to _veh_multimodal
          IAssert (_veh_multimodal != nullptr);
          // Here is the difference from single-class fixed routing
          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_bus_route_ID () != TInt (-1))
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
        }
    }
  // printf("Finished route OD veh\n");

  MNM_Destination *_veh_dest;
  MNM_Dlink *_link;
  for (auto _link_it : m_link_factory->m_link_map)
    {
      _link = _link_it.second;
      _node_ID = _link->m_to_node->m_node_ID;
      for (auto _veh : _link->m_finished_array)
        {
          // Here is the difference from single-class fixed routing
          if ((_veh->m_type == MNM_TYPE_STATIC)
              && (_veh->get_class () == m_veh_class)
              && (_veh->get_bus_route_ID () != TInt (-1))
              && (!_veh->get_ispnr ()))
            {
              // Here is the difference from single-class fixed routing

              _veh_dest = _veh->get_destination ();
              if (_veh_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  if (!m_tracker.find (_veh)->second->empty ())
                    {
                      throw std::runtime_error (
                        "Something wrong in fixed bus routing!");
                    }
                  _veh->set_next_link (nullptr);
                }
              else
                {
                  if (m_tracker.find (_veh) == m_tracker.end ())
                    {
                      throw std::runtime_error (
                        "Vehicle not registered in link, impossible!");
                    }
                  if (_veh->get_current_link () == _veh->get_next_link ())
                    {
                      _next_link_ID = m_tracker.find (_veh)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          printf ("The node is %d, the vehicle should head to "
                                  "%d\n",
                                  (int) _node_ID,
                                  (int) _veh_dest->m_dest_node->m_node_ID);
                          throw std::runtime_error (
                            "Something wrong in routing, wrong next link 2\n");
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

int
MNM_Routing_Bus::remove_finished (MNM_Veh *veh, bool del)
{
  if (veh->get_bus_route_ID () != -1)
    {
      MNM_Routing_Biclass_Fixed::remove_finished (veh, del);
    }
  return 0;
}

/**************************************************************************
                          PnR Fixed Vehicle Routing
**************************************************************************/
MNM_Routing_PnR_Fixed::MNM_Routing_PnR_Fixed (
  PNEGraph &driving_graph, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
  PnR_Path_Table *pnr_path_table, TInt route_frq, TInt buffer_length,
  TInt veh_class)
    : MNM_Routing_Biclass_Fixed::MNM_Routing_Biclass_Fixed (driving_graph,
                                                            od_factory,
                                                            node_factory,
                                                            link_factory,
                                                            route_frq,
                                                            buffer_length,
                                                            veh_class)
{
  m_veh_class = 0; // only driving car
  m_pnr_path_table = pnr_path_table;
}

MNM_Routing_PnR_Fixed::~MNM_Routing_PnR_Fixed ()
{
  for (auto _map_it : m_tracker)
    {
      _map_it.second->clear ();
      delete _map_it.second;
    }
  m_tracker.clear ();

  // clear pnr_path_table
  if ((m_pnr_path_table != nullptr) && (!m_pnr_path_table->empty ()))
    {
      // printf("Address of m_pnr_path_table is %p\n", (void
      // *)m_pnr_path_table); printf("%d\n", m_pnr_path_table -> size());
      for (auto _it : *m_pnr_path_table)
        { // origin
          for (auto _it_it : *(_it.second))
            { // destination
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_pnr_path_table->clear ();
      delete m_pnr_path_table;
    }
}

int
MNM_Routing_PnR_Fixed::change_choice_portion (TInt routing_interval)
{
  // m_veh_class starts from 0 (car) to 1 (truck)
  MNM::copy_buffer_to_p (m_pnr_path_table, routing_interval);
  MNM::normalize_path_table_p (m_pnr_path_table);
  return 0;
}

// register each vehicle with a route based on the portion of path flow
int
MNM_Routing_PnR_Fixed::register_veh (MNM_Veh *veh, bool track)
{
  TFlt _r = MNM_Ults::rand_flt ();
  // printf("%d\n", veh -> get_origin() -> m_origin_node  -> m_node_ID);
  // printf("%d\n", veh -> get_destination() -> m_dest_node  -> m_node_ID);

  MNM_Veh_Multimodal *_veh_multimodal
    = dynamic_cast<MNM_Veh_Multimodal *> (veh);
  IAssert (_veh_multimodal != nullptr);
  IAssert (_veh_multimodal->get_ispnr ());
  MNM_PnR_Pathset *_pathset
    = m_pnr_path_table
        ->find (_veh_multimodal->get_origin ()->m_origin_node->m_node_ID)
        ->second
        ->find (_veh_multimodal->get_destination ()->m_dest_node->m_node_ID)
        ->second;
  MNM_PnR_Path *_route_path = nullptr;
  // printf("1\n");
  // note m_path_vec is an ordered vector, not unordered

  for (MNM_Path *_path : _pathset->m_path_vec)
    {
      // printf("2\n");
      if (_path->m_p >= _r)
        {
          _route_path = dynamic_cast<MNM_PnR_Path *> (_path);
          break;
        }
      else
        {
          _r -= _path->m_p;
        }
    }

  // printf("3\n");
  if (_route_path == nullptr)
    {
      throw std::runtime_error ("Wrong probability in register_veh!");
    }
  if (track)
    {
      auto *_link_queue = new std::deque<TInt> ();
      // copy links in the route to _link_queue
      // https://www.cplusplus.com/reference/iterator/back_inserter/
      std::copy (_route_path->m_driving_path->m_link_vec.begin (),
                 _route_path->m_driving_path->m_link_vec.end (),
                 std::back_inserter (*_link_queue));
      // printf("old link q is %d, New link queue is %d\n", _route_path ->
      // m_link_vec.size(), _link_queue -> size());
      m_tracker.insert (
        std::pair<MNM_Veh *, std::deque<TInt> *> (veh, _link_queue));
    }
  veh->m_path = _route_path; // base vehicle class, base MNM_Path pointer to
                             // derived MNM_PnR_Path
  _veh_multimodal->m_pnr_path = _route_path;
  _veh_multimodal->m_transit_path = _route_path->m_transit_path;
  return 0;
}

int
MNM_Routing_PnR_Fixed::init_routing (Path_Table *driving_path_table)
{
  return 0;
}

int
MNM_Routing_PnR_Fixed::update_routing (TInt timestamp)
{
  // printf("MNM_Routing_Fixed::update_routing\n");
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh_Multimodal *_veh_multimodal;
  TInt _cur_ass_int;

  if (m_buffer_as_p)
    {
      if ((timestamp % m_routing_freq == 0 || timestamp == 0))
        {
          _cur_ass_int = TInt (timestamp / m_routing_freq);
          if (_cur_ass_int < m_buffer_length)
            {
              change_choice_portion (_cur_ass_int);
            }
        }
    }

  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      // printf("1.1\n");
      _origin = _origin_it.second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      for (auto _veh : _origin_node->m_in_veh_queue)
        {
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          // printf("1.2\n");
          if (_veh_multimodal->m_type == MNM_TYPE_STATIC
              && _veh_multimodal->m_class == 0
              && _veh->get_bus_route_ID () == TInt (-1)
              && _veh_multimodal->get_ispnr ())
            {
              if (m_tracker.find (_veh) == m_tracker.end ())
                { // vehicle not in tracker
                  // printf("Registering!\n");
                  register_veh (_veh, true);
                  // printf("1.3\n");
                  _next_link_ID = m_tracker.find (_veh)->second->front ();
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  _veh->set_next_link (_next_link);
                  m_tracker.find (_veh)
                    ->second->pop_front (); // adjust remaining links
                }
            }
          // according to Dr. Wei Ma, add a nominal path to adaptive users for
          // DAR extraction, not rigorous, but will do the DODE job
          else if (_veh_multimodal->m_type == MNM_TYPE_ADAPTIVE
                   && _veh_multimodal->m_class == 0
                   && _veh->get_bus_route_ID () == TInt (-1)
                   && _veh_multimodal->get_ispnr ())
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
  MNM_Destination *_final_dest;
  MNM_Dlink *_link;
  TInt _mid_dest_node_ID;
  for (auto _link_it : m_link_factory->m_link_map)
    {
      // printf("2.01\n");
      _link = _link_it.second;
      // printf("2.02\n");
      _node_ID = _link->m_to_node->m_node_ID;
      // printf("2.1\n");
      for (auto _veh : _link->m_finished_array)
        {
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          if (_veh_multimodal->m_type == MNM_TYPE_STATIC
              && _veh_multimodal->m_class == 0
              && _veh->get_bus_route_ID () == TInt (-1)
              && _veh_multimodal->get_ispnr ())
            {
              _final_dest = _veh->get_destination ();
              _mid_dest_node_ID
                = dynamic_cast<MNM_PnR_Path *> (_veh_multimodal->m_path)
                    ->m_mid_dest_node_ID;
              IAssert (_final_dest->m_dest_node->m_node_ID
                       != _mid_dest_node_ID);
              // printf("2.2\n");
              if (_mid_dest_node_ID == _node_ID)
                { // vehicles reaching mid destination
                  if (!m_tracker.find (_veh)->second->empty ())
                    { // check if any links left in the route
                      throw std::runtime_error (
                        "Something wrong in fixed pnr routing!");
                    }
                  _veh->set_next_link (nullptr);
                  // m_tracker.erase(m_tracker.find(_veh));
                }
              else
                { // vehicles enroute, adjust _next_link_ID, which is changed by
                  // node->evolve() in simulation dta.cpp
                  // printf("2.3\n");
                  if (m_tracker.find (_veh) == m_tracker.end ())
                    { // check if vehicle is registered in m_tracker, which
                      // should be done in releasing from origin
                      throw std::runtime_error (
                        "Vehicle not registered in link, impossible!");
                    }
                  if (_veh->get_current_link () == _veh->get_next_link ())
                    {
                      _next_link_ID = m_tracker.find (_veh)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          printf ("The node is %d, the vehicle should head to "
                                  "mid destination %d\n",
                                  (int) _node_ID, (int) _mid_dest_node_ID);
                          throw std::runtime_error (
                            "Something wrong in fixed pnr routing, wrong "
                            "next link 2\n");
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

int
MNM_Routing_PnR_Fixed::remove_finished (MNM_Veh *veh, bool del)
{
  if (veh->get_ispnr ())
    {
      MNM_Routing_Biclass_Fixed::remove_finished (veh, del);
    }
  return 0;
}

/**************************************************************************
                  Passenger Bus Transit Routing
**************************************************************************/
MNM_Routing_PassengerBusTransit::MNM_Routing_PassengerBusTransit (
  PNEGraph &transit_graph, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
  MNM_Parking_Lot_Factory *parkinglot_factory,
  MNM_Transit_Link_Factory *transitlink_factory)
{
  m_graph = transit_graph;
  m_od_factory = od_factory;
  m_node_factory = node_factory;
  m_transitlink_factory = transitlink_factory;
  m_busstop_factory = busstop_factory;
  m_parkinglot_factory = parkinglot_factory;
}

MNM_Routing_PassengerBusTransit::~MNM_Routing_PassengerBusTransit () { ; }

/**************************************************************************
                  Passenger Bus Transit Fixed Routing
**************************************************************************/
MNM_Routing_PassengerBusTransit_Fixed::MNM_Routing_PassengerBusTransit_Fixed (
  PNEGraph &transit_graph, MNM_OD_Factory *od_factory,
  MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
  MNM_Parking_Lot_Factory *parkinglot_factory,
  MNM_Transit_Link_Factory *transitlink_factory,
  Path_Table *bustransit_path_table, TInt route_frq, TInt buffer_length)
    : MNM_Routing_PassengerBusTransit::
        MNM_Routing_PassengerBusTransit (transit_graph, od_factory,
                                         node_factory, busstop_factory,
                                         parkinglot_factory,
                                         transitlink_factory)
{
  m_tracker = std::unordered_map<MNM_Passenger *, std::deque<TInt> *> ();
  if ((route_frq == -1) || (buffer_length == -1))
    {
      m_buffer_as_p = false;
      m_routing_freq = -1;
      // m_cur_routing_interval = -1;
    }
  else
    {
      m_routing_freq = route_frq;
      m_buffer_as_p = true;
      m_buffer_length = buffer_length;
      // m_cur_routing_interval = 0;
    }
  m_bustransit_path_table = bustransit_path_table;
}

MNM_Routing_PassengerBusTransit_Fixed::~MNM_Routing_PassengerBusTransit_Fixed ()
{
  for (auto _map_it : m_tracker)
    {
      _map_it.second->clear ();
      delete _map_it.second;
    }
  m_tracker.clear ();

  if ((m_bustransit_path_table != nullptr)
      && (!m_bustransit_path_table->empty ()))
    {
      // printf("Address of m_bustransit_path_table is %p\n", (void
      // *)m_bustransit_path_table); printf("%d\n", m_bustransit_path_table ->
      // size());
      for (auto _it : *m_bustransit_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_bustransit_path_table->clear ();
      delete m_bustransit_path_table;
    }
}

int
MNM_Routing_PassengerBusTransit_Fixed::change_choice_portion (
  TInt routing_interval)
{
  MNM::copy_buffer_to_p (m_bustransit_path_table, routing_interval);
  MNM::normalize_path_table_p (m_bustransit_path_table);
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::add_passenger_path (
  MNM_Passenger *passenger, std::deque<TInt> *link_que)
{
  std::deque<TInt> *_new_link_que = new std::deque<TInt> ();
  std::copy (link_que->begin (), link_que->end (),
             std::back_inserter (*_new_link_que));
  m_tracker.insert (
    std::pair<MNM_Passenger *, std::deque<TInt> *> (passenger, _new_link_que));
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::register_passenger (
  MNM_Passenger *passenger, bool track)
{
  MNM_Path *_route_path = nullptr;

  if (!passenger->m_pnr)
    {
      // direct bus transit
      TFlt _r = MNM_Ults::rand_flt ();

      MNM_Pathset *_pathset
        = m_bustransit_path_table
            ->find (passenger->get_origin ()->m_origin_node->m_node_ID)
            ->second
            ->find (passenger->get_destination ()->m_dest_node->m_node_ID)
            ->second;

      // printf("1\n");
      // note m_path_vec is an ordered vector, not unordered
      for (MNM_Path *_path : _pathset->m_path_vec)
        {
          // printf("2\n");
          if (_path->m_p >= _r)
            {
              _route_path = _path;
              break;
            }
          else
            {
              _r -= _path->m_p;
            }
        }
      // printf("3\n");
      if (_route_path == nullptr)
        {
          throw std::runtime_error ("Wrong probability!");
        }
      passenger->m_transit_path = _route_path;
    }
  else
    {
      // PnR
      _route_path = passenger->m_transit_path;
      IAssert (_route_path != nullptr);
    }
  if (track)
    {
      std::deque<TInt> *_link_queue = new std::deque<TInt> ();
      std::copy (
        _route_path->m_link_vec.begin (), _route_path->m_link_vec.end (),
        std::back_inserter (
          *_link_queue)); // copy links in the route to _link_queue
                          // https://www.cplusplus.com/reference/iterator/back_inserter/
      // printf("old link q is %d, New link queue is %d\n", _route_path ->
      // m_link_vec.size(), _link_queue -> size());
      m_tracker.insert (
        std::pair<MNM_Passenger *, std::deque<TInt> *> (passenger,
                                                        _link_queue));
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::remove_finished (
  MNM_Passenger *passenger, bool del)
{
  IAssert (passenger->m_finish_time > 0
           && passenger->m_finish_time > passenger->m_start_time);
  if (m_tracker.find (passenger) != m_tracker.end () && del)
    {
      IAssert (passenger->m_passenger_type
               == MNM_TYPE_STATIC); // adaptive user not in m_tracker
      m_tracker.find (passenger)->second->clear ();
      delete m_tracker.find (passenger)->second;
      m_tracker.erase (passenger);
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::init_routing (Path_Table *path_table)
{
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_origin (TInt timestamp)
{
  MNM_Origin *_origin;
  MNM_Origin_Multimodal *_origin_multimodal;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;
  TInt _cur_ass_int;

  if (m_buffer_as_p)
    {
      if ((timestamp % m_routing_freq == 0 || timestamp == 0))
        {
          _cur_ass_int = TInt (timestamp / m_routing_freq);
          if (_cur_ass_int < m_buffer_length)
            {
              change_choice_portion (_cur_ass_int);
            }
        }
    }

  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      // printf("1.1\n");
      _origin = _origin_it.second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;
      _origin_multimodal = dynamic_cast<MNM_Origin_Multimodal *> (_origin);
      for (auto _passenger : _origin_multimodal->m_in_passenger_queue)
        {
          // printf("1.2\n");
          if (_passenger->m_passenger_type == MNM_TYPE_STATIC)
            {
              if (m_tracker.find (_passenger) == m_tracker.end ())
                { // passenger not in tracker
                  // printf("Registering!\n");
                  register_passenger (_passenger, true);
                  // printf("1.3\n");
                  _next_link_ID = m_tracker.find (_passenger)->second->front ();
                  _next_link
                    = m_transitlink_factory->get_transit_link (_next_link_ID);
                  _passenger->set_next_link (_next_link);
                  m_tracker.find (_passenger)
                    ->second->pop_front (); // adjust links left
                }
            }
          // according to Dr. Wei Ma, add a nominal path to adaptive users for
          // DAR extraction, not rigorous, but will do the DODE job
          else if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE)
            {
              if (_passenger->m_transit_path == nullptr)
                {
                  register_passenger (_passenger, false);
                }
              IAssert (_passenger->m_transit_path != nullptr);
            }
        }
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_parkinglot (
  TInt timestamp)
{
  MNM_Parking_Lot *_parking_lot;
  TInt _next_link_ID;
  MNM_Transit_Link *_next_link;

  for (auto _parkinglot_it : m_parkinglot_factory->m_parking_lot_map)
    {
      _parking_lot = _parkinglot_it.second;
      for (auto _passenger : _parking_lot->m_in_passenger_queue)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_STATIC)
            {
              if (m_tracker.find (_passenger) == m_tracker.end ())
                { // passenger not in tracker
                  // printf("Registering!\n");
                  IAssert (_passenger->m_driving_path != nullptr
                           && _passenger->m_pnr);
                  register_passenger (_passenger, true);
                  // printf("1.3\n");
                  _next_link_ID = m_tracker.find (_passenger)->second->front ();
                  _next_link
                    = m_transitlink_factory->get_transit_link (_next_link_ID);
                  _passenger->set_next_link (_next_link);
                  m_tracker.find (_passenger)
                    ->second->pop_front (); // adjust links left
                }
              else
                {
                  if (_passenger->m_next_link == nullptr)
                    {
                      throw std::runtime_error ("Passenger has no next link");
                    }
                }
              if (_passenger->m_next_link == nullptr)
                {
                  throw std::runtime_error ("Passenger has no next link");
                }
            }
        }
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_one_link (
  TInt timestamp, MNM_Transit_Link *link)
{
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;
  MNM_Destination *_dest;
  MNM_Walking_Link *_walking_link;
  MNM_Bus_Link *_bus_link;

  _node_ID = link->m_to_node_ID;

  if (link->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
    {
      _walking_link = dynamic_cast<MNM_Walking_Link *> (link);
      IAssert (_walking_link != nullptr);
      for (auto _passenger : _walking_link->m_finished_array)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_STATIC)
            {
              _dest = _passenger->get_destination ();
              // printf("2.2\n");
              if (_dest->m_dest_node->m_node_ID == _node_ID)
                { // passengers reaching destination
                  IAssert (link->m_to_node_type == "destination");
                  if (!(m_tracker.find (_passenger)->second->empty ()))
                    { // check if any links left in the route
                      throw std::runtime_error (
                        "Something wrong in passenger bus transit fixed "
                        "routing!");
                    }
                  _passenger->set_next_link (nullptr);
                  // m_tracker.erase(m_tracker.find(_passenger));
                }
              else
                { // passengers enroute, adjust _next_link_ID,
                  // printf("2.3\n");
                  if (m_tracker.find (_passenger) == m_tracker.end ())
                    { // check if passenger is registered in m_tracker, which
                      // should be done in releasing from origin
                      throw std::runtime_error (
                        "Passenger not registered in link, impossible!");
                    }
                  if (link->m_to_node_type != "bus_stop_physical"
                      && link->m_to_node_type != "bus_stop_virtual"
                      && link->m_to_node_type != "destination")
                    {
                      throw std::runtime_error (
                        "Something wrong in passenger bus transit fixed "
                        "routing!");
                    }
                  if (_passenger->get_current_link ()->m_link_ID
                      == _passenger->get_next_link ()->m_link_ID)
                    {
                      _next_link_ID
                        = m_tracker.find (_passenger)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          printf ("The node is %d, the passenger should head "
                                  "to %d\n",
                                  (int) _node_ID,
                                  (int) _dest->m_dest_node->m_node_ID);
                          throw std::runtime_error (
                            "Something wrong in routing, wrong next link 2");
                        }
                      _next_link = m_transitlink_factory->get_transit_link (
                        _next_link_ID);
                      _passenger->set_next_link (_next_link);
                      m_tracker.find (_passenger)->second->pop_front ();
                    }
                } // end if-else
            }     // end if passenger->m_type
        }         // end for passenger_it
    }             // end if _link->m_link_type
  else if (link->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
    {
      _bus_link = dynamic_cast<MNM_Bus_Link *> (link);
      IAssert (_bus_link != nullptr);
      for (auto _passenger : _bus_link->m_finished_array)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_STATIC)
            {
              _dest = _passenger->get_destination ();
              // printf("2.2\n");
              if (_dest->m_dest_node->m_node_ID == _node_ID)
                { // passengers reaching destination
                  throw std::runtime_error (
                    "Something wrong in passenger bus transit fixed "
                    "routing!");
                }
              else
                { // passengers enroute, adjust _next_link_ID,
                  // printf("2.3\n");
                  if (m_tracker.find (_passenger) == m_tracker.end ())
                    { // check if passenger is registered in m_tracker, which
                      // should be done in releasing from origin
                      throw std::runtime_error (
                        "Passenger not registered in link, impossible!");
                    }
                  if (link->m_from_node_type != "bus_stop"
                      || link->m_to_node_type != "bus_stop")
                    {
                      throw std::runtime_error (
                        "Something wrong in passenger bus transit fixed "
                        "routing!");
                    }
                  if (_passenger->get_current_link ()->m_link_ID
                      == _passenger->get_next_link ()->m_link_ID)
                    {
                      _next_link_ID
                        = m_tracker.find (_passenger)->second->front ();
                      if (_next_link_ID == -1)
                        {
                          printf ("The node is %d, the passenger should head "
                                  "to %d\n",
                                  (int) _node_ID,
                                  (int) _dest->m_dest_node->m_node_ID);
                          throw std::runtime_error (
                            "Something wrong in routing, wrong next link 2");
                        }
                      _next_link = m_transitlink_factory->get_transit_link (
                        _next_link_ID);
                      _passenger->set_next_link (_next_link);
                      m_tracker.find (_passenger)->second->pop_front ();
                    }
                } // end if-else
            }     // end if passenger->m_type
        }         // end for passenger_it
    }
  else
    {
      throw std::runtime_error (
        "MNM_Routing_PassengerBusTransit_Fixed::update_routing_one_link, "
        "wrong transit link type");
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_link (TInt timestamp)
{
  MNM_Transit_Link *_link;
  for (auto _link_it : m_transitlink_factory->m_transit_link_map)
    {
      // printf("2.01\n");
      _link = _link_it.second;
      // printf("2.02\n");
      update_routing_one_link (timestamp, _link);
    } // end for link_it
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_one_busstop (
  TInt timestamp, MNM_Busstop *busstop)
{
  TInt _next_link_ID;
  MNM_Transit_Link *_next_link;
  auto *_busstop_virtual = dynamic_cast<MNM_Busstop_Virtual *> (busstop);
  if (_busstop_virtual == nullptr)
    {
      return 0;
    }
  for (auto _bus : _busstop_virtual->m_bus_queue)
    {
      for (auto _passenger : _bus->m_passenger_pool)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_STATIC)
            {
              if (m_tracker.find (_passenger) == m_tracker.end ())
                { // check if passenger is registered in m_tracker, which should
                  // be done in releasing from origin
                  throw std::runtime_error (
                    "Passenger on bus not registered, impossible!");
                }
              // passenger already on board before this bus stop (i.e., continue
              // riding or alighting)
              if (_passenger->get_current_link ()->m_link_ID
                    == _passenger->get_next_link ()->m_link_ID
                  && _busstop_virtual->m_bus_in_link != nullptr
                  && _passenger->get_current_link ()->m_link_ID
                       == _busstop_virtual->m_bus_in_link->m_link_ID)
                {
                  _next_link_ID = m_tracker.find (_passenger)->second->front ();
                  _next_link
                    = m_transitlink_factory->get_transit_link (_next_link_ID);
                  _passenger->set_next_link (_next_link);
                  m_tracker.find (_passenger)->second->pop_front ();
                }
            }
        }
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing_busstop (TInt timestamp)
{
  MNM_Busstop *_busstop;
  for (auto _busstop_it : m_busstop_factory->m_busstop_map)
    {
      _busstop = _busstop_it.second;
      update_routing_one_busstop (timestamp, _busstop);
    }
  return 0;
}

int
MNM_Routing_PassengerBusTransit_Fixed::update_routing (TInt timestamp)
{
  // printf("MNM_Routing_Fixed::update_routing\n");
  update_routing_origin (timestamp);
  update_routing_parkinglot (timestamp);
  update_routing_link (timestamp);
  update_routing_busstop (timestamp);
  return 0;
}

/**************************************************************************
                  Passenger and Vehicle Adaptive Routing
**************************************************************************/
MNM_Routing_Multimodal_Adaptive::MNM_Routing_Multimodal_Adaptive (
  const std::string &file_folder, PNEGraph &driving_graph,
  PNEGraph &transit_graph, MNM_Statistics *statistics,
  MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
  MNM_Busstop_Factory *busstop_factory,
  MNM_Parking_Lot_Factory *parkinglot_factory, MNM_Link_Factory *link_factory,
  MNM_Transit_Link_Factory *transitlink_factory)
{
  m_statistics = dynamic_cast<MNM_Statistics_Lrn_Multimodal *> (statistics);
  IAssert (m_statistics != nullptr);
  m_self_config = new MNM_ConfReader (file_folder + "/config.conf", "ADAPTIVE");
  m_routing_freq = m_self_config->get_int ("route_frq");

  // the unit of m_vot here is different from that of m_vot in DUE (money /
  // interval)
  try
    {
      m_vot = m_self_config->get_float ("vot")
              / 3600.; // money / hour -> money / second
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "vot does not exist in config.conf/ADAPTIVE, use default "
                   "value 2. instead\n";
      m_vot = 20 / 3600.; // money / hour -> money / second
    }

  m_driving_table = new Routing_Table ();
  m_transit_table = new Routing_Table ();
  m_driving_graph = driving_graph;
  m_transit_graph = transit_graph;
  m_od_factory = od_factory;
  m_node_factory = node_factory;
  m_busstop_factory = busstop_factory;
  m_parkinglot_factory = parkinglot_factory;
  m_link_factory = link_factory;
  m_transitlink_factory = transitlink_factory;

  m_driving_link_cost = std::unordered_map<TInt, TFlt> ();
  m_bustransit_link_cost = std::unordered_map<TInt, TFlt> ();

  auto *_tmp_config = new MNM_ConfReader (file_folder + "/config.conf", "DTA");
  m_working = _tmp_config->get_float ("adaptive_ratio_passenger") > 0
              || _tmp_config->get_float ("adaptive_ratio_car") > 0
              || _tmp_config->get_float ("adaptive_ratio_truck") > 0;
  delete _tmp_config;
}

MNM_Routing_Multimodal_Adaptive::~MNM_Routing_Multimodal_Adaptive ()
{
  delete m_self_config;

  for (auto _it : m_od_factory->m_destination_map)
    {
      if (m_driving_table->find (_it.second) != m_driving_table->end ())
        {
          m_driving_table->find (_it.second)->second->clear ();
          delete m_driving_table->find (_it.second)->second;
        }
      if (m_transit_graph->IsNode (_it.second->m_dest_node->m_node_ID)
          && m_transit_table->find (_it.second) != m_transit_table->end ())
        {
          m_transit_table->find (_it.second)->second->clear ();
          delete m_transit_table->find (_it.second)->second;
        }
    }
  m_driving_table->clear ();
  m_transit_table->clear ();
  delete m_driving_table;
  delete m_transit_table;
  m_driving_link_cost.clear ();
  m_bustransit_link_cost.clear ();
}

int
MNM_Routing_Multimodal_Adaptive::init_routing ()
{
  std::unordered_map<TInt, TInt> *_shortest_path_tree;
  for (auto _it : m_od_factory->m_destination_map)
    {
      _shortest_path_tree = new std::unordered_map<TInt, TInt> ();
      if (m_driving_table->find (_it.second) != m_driving_table->end ())
        {
          m_driving_table->find (_it.second)->second->clear ();
          delete m_driving_table->find (_it.second)->second;
        }
      m_driving_table->insert (
        std::pair<MNM_Destination *,
                  std::unordered_map<TInt, TInt> *> (_it.second,
                                                     _shortest_path_tree));
      if (m_transit_graph->IsNode (_it.second->m_dest_node->m_node_ID))
        {
          _shortest_path_tree = new std::unordered_map<TInt, TInt> ();
          if (m_transit_table->find (_it.second) != m_transit_table->end ())
            {
              m_transit_table->find (_it.second)->second->clear ();
              delete m_transit_table->find (_it.second)->second;
            }
          m_transit_table->insert (
            std::pair<MNM_Destination *,
                      std::unordered_map<TInt, TInt> *> (_it.second,
                                                         _shortest_path_tree));
        }
    }
  for (auto _link_it : m_link_factory->m_link_map)
    {
      m_driving_link_cost.insert (std::pair<TInt, TFlt> (_link_it.first, -1));
    }
  for (auto _link_it : m_transitlink_factory->m_transit_link_map)
    {
      m_bustransit_link_cost.insert (
        std::pair<TInt, TFlt> (_link_it.first, -1));
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_link_cost ()
{
  for (auto _it : m_statistics->m_record_interval_tt)
    { // seconds
      m_driving_link_cost[_it.first] = _it.second * m_vot
                                       + dynamic_cast<MNM_Dlink_Multiclass *> (
                                           m_link_factory->get_link (_it.first))
                                           ->m_toll_car;
    }
  for (auto _it : m_statistics->m_record_interval_tt_bus_transit)
    { // seconds
      m_bustransit_link_cost[_it.first] = _it.second * m_vot;
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing (TInt timestamp)
{
  // relying on m_statistics -> m_record_interval_tt_bus_transit, which is
  // obtained in simulation, not after simulation link::get_link_tt(), based on
  // density
  MNM_Destination *_dest;
  TInt _dest_node_ID;
  // <node ID, out link ID>
  std::unordered_map<TInt, TInt> *_shortest_path_tree;
  // update routing_table based on the snapshot of the network
  if ((timestamp) % m_routing_freq == 0 || timestamp == 0)
    {
      // printf("Calculating the shortest path trees!\n");
      update_link_cost ();
      for (auto _it : m_od_factory->m_destination_map)
        {
          // #pragma omp task firstprivate(_it)
          // {
          _dest = _it.second;
          _dest_node_ID = _dest->m_dest_node->m_node_ID;
          // printf("Destination ID: %d\n", (int) _dest_node_ID);

          // vehicle
          _shortest_path_tree = m_driving_table->find (_dest)->second;
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_driving_graph,
                                              m_driving_link_cost,
                                              *_shortest_path_tree);
          // MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID, m_driving_graph,
          // m_statistics -> m_record_interval_tt, *_shortest_path_tree);
          // MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID,
          // m_driving_graph, m_statistics -> m_record_interval_tt,
          // *_shortest_path_tree);

          // passenger
          if (m_transit_graph->IsNode (_dest_node_ID))
            {
              _shortest_path_tree = m_transit_table->find (_dest)->second;
              MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID,
                                                  m_transit_graph,
                                                  m_bustransit_link_cost,
                                                  *_shortest_path_tree);
              // MNM_Shortest_Path::all_to_one_FIFO(_dest_node_ID,
              // m_transit_graph, m_statistics ->
              // m_record_interval_tt_bus_transit, *_shortest_path_tree);
              // MNM_Shortest_Path::all_to_one_Dijkstra(_dest_node_ID,
              // m_transit_graph, m_statistics ->
              // m_record_interval_tt_bus_transit, *_shortest_path_tree);
            }
          // }
        }
    }

  update_routing_passenger (timestamp);
  update_routing_vehicle (timestamp);
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_origin (
  TInt timestamp)
{
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;

  /* route the passenger in Origin nodes */
  // printf("Routing the passenger!\n");
  MNM_Origin_Multimodal *_origin;
  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_it.second);
      _node_ID = _origin->m_origin_node->m_node_ID;
      for (auto _passenger : _origin->m_in_passenger_queue)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE)
            {
              _next_link_ID
                = m_transit_table->find (_passenger->get_destination ())
                    ->second->find (_node_ID)
                    ->second;
              if (_next_link_ID < 0)
                {
                  // printf("%d\n", _veh -> get_destination() -> m_Dest_ID);
                  // _shortest_path_tree = m_table -> find(_veh ->
                  // get_destination()) -> second; printf("%d\n",
                  // _shortest_path_tree -> size()); for (auto it :
                  // (*_shortest_path_tree)) printf("%d, %d\n", it.first,
                  // it.second);
                  throw std::runtime_error (
                    "Something wrong in passenger adaptive routing, "
                    "wrong next link 1");
                }
              // printf("From origin, The next link ID will be %d\n",
              // _next_link_ID());
              _next_link
                = m_transitlink_factory->get_transit_link (_next_link_ID);
              _passenger->set_next_link (_next_link);
              // printf("The next link now it's %d\n", _passenger ->
              // get_next_link() -> m_link_ID());
            }
        }
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_parkinglot (
  TInt timestamp)
{
  /* route the passenger out of parking lots in PnR mode */
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;
  MNM_Parking_Lot *_parkinglot;

  for (auto _parkinglot_it : m_parkinglot_factory->m_parking_lot_map)
    {
      _parkinglot = _parkinglot_it.second;
      _node_ID = _parkinglot->m_dest_node->m_node_ID;
      for (auto _passenger : _parkinglot->m_in_passenger_queue)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE)
            {
              _next_link_ID
                = m_transit_table->find (_passenger->get_destination ())
                    ->second->find (_node_ID)
                    ->second;
              if (_next_link_ID < 0)
                {
                  throw std::runtime_error (
                    "Something wrong in passenger adaptive routing, parking "
                    "lot to final destination using transit is impossible");
                }
              // printf("From parking lot, The next link ID will be %d\n",
              // _next_link_ID());
              _next_link
                = m_transitlink_factory->get_transit_link (_next_link_ID);
              _passenger->set_next_link (_next_link);
              // printf("The next link now it's %d\n", _passenger ->
              // get_next_link() -> m_link_ID());
            }
        }
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_one_link (
  TInt timestamp, MNM_Transit_Link *link)
{
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;
  MNM_Walking_Link *_link;
  MNM_Destination *_dest;
  MNM_Bus_Link *_bus_link;

  if (link->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
    {
      _link = dynamic_cast<MNM_Walking_Link *> (link);
      _node_ID = _link->m_to_node_ID;
      for (auto _passenger : _link->m_finished_array)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE)
            {
              if (_link->m_link_ID
                  != _passenger->get_current_link ()->m_link_ID)
                {
                  throw std::runtime_error ("Wrong current link!");
                }

              _dest = _passenger->get_destination ();
              if (_dest->m_dest_node->m_node_ID == _node_ID
                  && _link->m_to_node_type == "destination")
                {
                  _passenger->set_next_link (nullptr);
                }
              else
                {
                  // the next link for boarding passengers is always the bus
                  // link
                  if (_link->m_walking_type == "boarding")
                    {
                      IAssert (_link->m_to_node_type == "bus_stop_virtual");
                      _next_link = dynamic_cast<MNM_Busstop_Virtual *> (
                                     m_busstop_factory->get_busstop (_node_ID))
                                     ->m_bus_out_link;
                    }
                  else
                    {
                      _next_link_ID = m_transit_table->find (_dest)
                                        ->second->find (_node_ID)
                                        ->second;
                      if (_next_link_ID < 0)
                        {
                          printf ("Something wrong in passenger adaptive "
                                  "routing, wrong next link 2\n");
                          printf ("The node is %d, the passenger should head "
                                  "to %d\n",
                                  (int) _node_ID,
                                  (int) _dest->m_dest_node->m_node_ID);
                          // exit(-1);
                          auto _node_I = m_transit_graph->GetNI (_node_ID);
                          if (_node_I.GetOutDeg () > 0)
                            {
                              printf ("Assign randomly!\n");
                              _next_link_ID = _node_I.GetOutEId (
                                MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                            }
                          else
                            {
                              throw std::runtime_error (
                                "passenger cannot walk to next link, can't do "
                                "anything!");
                            }
                        }
                      _next_link = m_transitlink_factory->get_transit_link (
                        _next_link_ID);
                    }

                  if (_next_link != nullptr)
                    {
                      // printf("Checking future\n");
                      TInt _next_node_ID = _next_link->m_to_node_ID;
                      if (_next_node_ID != _dest->m_dest_node->m_node_ID)
                        {
                          // printf("Destination node is %d\n", _veh ->
                          // get_destination() -> m_dest_node -> m_node_ID());
                          if (m_transit_table->find (_dest)
                              == m_transit_table->end ())
                            {
                              printf ("Cannot find Destination\n");
                            }
                          if (m_transit_table->find (_dest)->second->find (
                                _next_node_ID)
                              == m_transit_table->find (_dest)->second->end ())
                            {
                              printf ("Can't find _next_node_ID\n");
                            }
                          if (m_transit_table->find (_dest)
                                ->second->find (_next_node_ID)
                                ->second
                              == -1)
                            {
                              throw std::runtime_error (
                                "Something wrong for the future node!");
                            }
                          // printf("Pass checking\n");
                        }
                    }
                  _passenger->set_next_link (_next_link);
                } // end if else
            }     // end if passenger->m_passenger_type
        }         // end for passenger
    }
  else if (link->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
    {
      _bus_link = dynamic_cast<MNM_Bus_Link *> (link);
      _node_ID = _bus_link->m_to_node_ID;
      for (auto _passenger : _bus_link->m_finished_array)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE)
            {
              if (_bus_link->m_link_ID
                  != _passenger->get_current_link ()->m_link_ID)
                {
                  throw std::runtime_error ("Wrong current link!");
                }

              _dest = _passenger->get_destination ();
              if (_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  throw std::runtime_error (
                    "Something wrong in passenger adaptive routing, "
                    "wrong current link");
                }
              else
                {
                  _next_link_ID = m_transit_table->find (_dest)
                                    ->second->find (_node_ID)
                                    ->second;
                  if (_next_link_ID < 0)
                    {
                      printf ("Something wrong in passenger adaptive routing, "
                              "wrong next link 2\n");
                      printf ("The node is %d, the passenger should head to "
                              "%d\n",
                              (int) _node_ID,
                              (int) _dest->m_dest_node->m_node_ID);
                      // exit(-1);
                      auto _node_I = m_transit_graph->GetNI (_node_ID);
                      if (_node_I.GetOutDeg () > 0)
                        {
                          printf ("Assign randomly!\n");
                          _next_link_ID = _node_I.GetOutEId (
                            MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                        }
                      else
                        {
                          throw std::runtime_error (
                            "passenger cannot walk to next link, can't do "
                            "anything!");
                        }
                    }
                  _next_link
                    = m_transitlink_factory->get_transit_link (_next_link_ID);

                  if (_next_link != nullptr)
                    {
                      // printf("Checking future\n");
                      TInt _next_node_ID = _next_link->m_to_node_ID;
                      if (_next_node_ID != _dest->m_dest_node->m_node_ID)
                        {
                          // printf("Destination node is %d\n", _veh ->
                          // get_destination() -> m_dest_node -> m_node_ID());
                          if (m_transit_table->find (_dest)
                              == m_transit_table->end ())
                            {
                              printf ("Cannot find Destination\n");
                            }
                          if (m_transit_table->find (_dest)->second->find (
                                _next_node_ID)
                              == m_transit_table->find (_dest)->second->end ())
                            {
                              printf ("Can't find _next_node_ID\n");
                            }
                          if (m_transit_table->find (_dest)
                                ->second->find (_next_node_ID)
                                ->second
                              == -1)
                            {
                              throw std::runtime_error (
                                "Something wrong for the future node!");
                            }
                          // printf("Pass checking\n");
                        }
                    }
                  _passenger->set_next_link (_next_link);
                } // end if else
            }     // end if passenger->m_passenger_type
        }         // end for passenger
    }
  else
    {
      throw std::runtime_error (
        "MNM_Routing_Multimodal_Adaptive::update_routing_passenger_one_"
        "link, wrong transit link type");
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_link (TInt timestamp)
{
  /* route the passenger en route in walking links */

  for (auto _link_it : m_transitlink_factory->m_transit_link_map)
    {
      update_routing_passenger_one_link (timestamp, _link_it.second);
    } // end for link_it
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_one_busstop (
  TInt timestamp, MNM_Busstop *busstop)
{
  TInt _node_ID, _next_link_ID;
  MNM_Transit_Link *_next_link;
  MNM_Destination *_dest;
  auto *_busstop_virtual = dynamic_cast<MNM_Busstop_Virtual *> (busstop);
  if (_busstop_virtual == nullptr)
    {
      return 0;
    }
  _node_ID = busstop->m_busstop_ID;
  for (auto _bus : _busstop_virtual->m_bus_queue)
    {
      for (auto _passenger : _bus->m_passenger_pool)
        {
          if (_passenger->m_passenger_type == MNM_TYPE_ADAPTIVE
              && _passenger->get_next_link ()->m_link_ID
                   == _passenger->get_current_link ()->m_link_ID
              && _busstop_virtual->m_bus_in_link != nullptr
              && _passenger->get_current_link ()->m_link_ID
                   == _busstop_virtual->m_bus_in_link->m_link_ID)
            {
              _dest = _passenger->get_destination ();
              _next_link_ID = m_transit_table->find (_dest)
                                ->second->find (_node_ID)
                                ->second;
              if (_next_link_ID < 0)
                {
                  printf ("Something wrong in passenger adaptive routing, "
                          "wrong next link 2\n");
                  printf ("The node is %d, the passenger should head to %d\n",
                          (int) _node_ID, (int) _dest->m_dest_node->m_node_ID);
                  // exit(-1);
                  auto _node_I = m_transit_graph->GetNI (_node_ID);
                  if (_node_I.GetOutDeg () > 0)
                    {
                      printf ("Assign randomly!\n");
                      _next_link_ID = _node_I.GetOutEId (
                        MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                    }
                  else
                    {
                      printf ("Can't do anything!\n");
                    }
                }
              _next_link
                = m_transitlink_factory->get_transit_link (_next_link_ID);
              if (_next_link != nullptr)
                {
                  // printf("Checking future\n");
                  TInt _next_node_ID = _next_link->m_to_node_ID;
                  if (_next_node_ID != _dest->m_dest_node->m_node_ID)
                    {
                      // printf("Destination node is %d\n", _dest -> m_dest_node
                      // -> m_node_ID());
                      if (m_transit_table->find (_dest)
                          == m_transit_table->end ())
                        {
                          printf ("Cannot find Destination\n");
                        }
                      if (m_transit_table->find (_dest)->second->find (
                            _next_node_ID)
                          == m_transit_table->find (_dest)->second->end ())
                        {
                          printf ("Can't find _next_node_ID\n");
                        }
                      if (m_transit_table->find (_dest)
                            ->second->find (_next_node_ID)
                            ->second
                          == -1)
                        {
                          throw std::runtime_error (
                            "Something wrong for the future node!");
                        }
                      // printf("Pass checking\n");
                    }
                }
              _passenger->set_next_link (_next_link);
            }
        }
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger_busstop (
  TInt timestamp)
{
  /* route the passenger en route already on bus */
  MNM_Busstop *_busstop;

  for (auto _busstop_it : m_busstop_factory->m_busstop_map)
    {
      _busstop = _busstop_it.second;
      update_routing_passenger_one_busstop (timestamp, _busstop);
    }
  return 0;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_passenger (TInt timestamp)
{
  update_routing_passenger_origin (timestamp);
  update_routing_passenger_parkinglot (timestamp);
  update_routing_passenger_link (timestamp);
  update_routing_passenger_busstop (timestamp);
  // printf("Finished Adaptive Routing for Passengers\n");
  return 0;
}

MNM_Destination *
MNM_Routing_Multimodal_Adaptive::find_mid_destination_for_pnr (
  TInt timestamp, TInt origin_node_ID, TInt final_dest_node_ID)
{
  MNM_Destination *_mid_dest;
  MNM_Destination_Multimodal *_final_dest;
  TInt _mid_dest_node_ID;
  std::unordered_map<TInt, TInt> *_shortest_path_tree;
  MNM_Path *_path;
  TFlt _path_tt, _cur_best_path_tt;
  MNM_Destination *_cur_best_mid_dest = nullptr;

  if (dynamic_cast<MNM_DMDND *> (m_node_factory->get_node (origin_node_ID))
      != nullptr)
    {
      if (dynamic_cast<MNM_Destination_Multimodal *> (
            ((MNM_DMDND *) m_node_factory->get_node (origin_node_ID))->m_dest)
            ->m_parking_lot
          == nullptr)
        {
          throw std::runtime_error (
            "pnr vehicle already on OD connector but cannot find mid "
            "parking lot");
        }
      return ((MNM_DMDND *) m_node_factory->get_node (origin_node_ID))->m_dest;
    }

  _final_dest = dynamic_cast<MNM_Destination_Multimodal *> (
    ((MNM_DMDND *) m_node_factory->get_node (final_dest_node_ID))->m_dest);

  if (_final_dest->m_connected_pnr_parkinglot_vec.empty ())
    {
      printf ("PnR mode is impossible for reaching destination node %d\n",
              (int) final_dest_node_ID);
      return nullptr;
    }

  _cur_best_path_tt = std::numeric_limits<double>::infinity ();
  // just use a random middle parking lot
  std::random_shuffle (_final_dest->m_connected_pnr_parkinglot_vec.begin (),
                       _final_dest->m_connected_pnr_parkinglot_vec.end ());
  for (auto _parkinglot : _final_dest->m_connected_pnr_parkinglot_vec)
    {
      // TODO: not every parking lot is suitable, add more conditions, like
      // distance threshold
      _mid_dest = _parkinglot->m_dest_node->m_dest;
      _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
      if (_mid_dest_node_ID == final_dest_node_ID)
        {
          continue;
        }

      if (origin_node_ID == _mid_dest_node_ID)
        {
          // already on the link to mid_dest_node
          return _parkinglot->m_dest_node->m_dest;
        }

      // no cruising for mobility service for middle destination
      _path_tt = TFlt (0);

      // cruising time for pnr
      // for (int i=0; i <= timestamp; i++) {
      //     if (_parkinglot -> m_cruising_time_record.find(timestamp-i) !=
      //     _parkinglot -> m_cruising_time_record.end()) {
      //         _path_tt += _parkinglot ->
      //         m_cruising_time_record.find(timestamp-i) -> second *
      //         _parkinglot -> m_unit_time; break;
      //     }
      // }

      // driving
      _shortest_path_tree = m_driving_table->find (_mid_dest)->second;
      _path = MNM::extract_path (origin_node_ID, _mid_dest_node_ID,
                                 *_shortest_path_tree, m_driving_graph);
      IAssert (_path != nullptr);
      _path_tt
        += MNM::get_path_tt_snapshot (_path,
                                      m_statistics->m_record_interval_tt);
      delete _path;

      // bus transit
      IAssert (_mid_dest_node_ID != final_dest_node_ID);
      _shortest_path_tree = m_transit_table->find (_final_dest)->second;
      _path = MNM::extract_path (_mid_dest_node_ID, final_dest_node_ID,
                                 *_shortest_path_tree, m_transit_graph);
      IAssert (_path != nullptr);
      _path_tt
        += MNM::get_path_tt_snapshot (_path,
                                      m_statistics
                                        ->m_record_interval_tt_bus_transit);
      delete _path;

      if (_cur_best_path_tt > _path_tt)
        {
          _cur_best_mid_dest = _parkinglot->m_dest_node->m_dest;
          _cur_best_path_tt = _path_tt;
        }
      // just use a random middle parking lot
      break;
    }
  // IAssert(_cur_best_mid_dest != nullptr);
  return _cur_best_mid_dest;
}

int
MNM_Routing_Multimodal_Adaptive::update_routing_vehicle (TInt timestamp)
{
  // relying on m_statistics -> m_record_interval_tt, which is obtained in
  // simulation, not after simulation link::get_link_tt(), based on density

  /* route the vehicle in Origin nodes */
  // printf("Routing the vehicle!\n");
  MNM_Origin *_origin;
  MNM_DMOND *_origin_node;
  TInt _node_ID, _next_link_ID;
  MNM_Dlink *_next_link;
  MNM_Veh_Multimodal *_veh_multimodal;
  MNM_Destination *_final_dest;
  MNM_Destination *_mid_dest;
  std::unordered_map<MNM_Destination *, MNM_Destination *> _mid_destination_map
    = std::unordered_map<MNM_Destination *, MNM_Destination *> ();

  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      _origin = _origin_it.second;
      _origin_node = _origin->m_origin_node;
      _node_ID = _origin_node->m_node_ID;

      // for pnr cars, find suitable mid dest
      if (!_mid_destination_map.empty ())
        _mid_destination_map.clear ();

      for (auto _veh : _origin_node->m_in_veh_queue)
        {
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          if (_veh_multimodal->m_type == MNM_TYPE_ADAPTIVE
              && _veh_multimodal->m_bus_route_ID == TInt (-1))
            {
              _final_dest = _veh->get_destination ();
              if (_veh_multimodal->m_pnr)
                {
                  // find a mid destination with a parking lot
                  if (_mid_destination_map.find (_final_dest)
                      == _mid_destination_map.end ())
                    {
                      _mid_dest
                        = find_mid_destination_for_pnr (timestamp, _node_ID,
                                                        _final_dest->m_dest_node
                                                          ->m_node_ID);
                      _mid_destination_map.insert (
                        std::pair<MNM_Destination *,
                                  MNM_Destination *> (_final_dest, _mid_dest));
                    }
                  _final_dest
                    = _mid_destination_map.find (_veh->get_destination ())
                        ->second;
                  if (_final_dest == nullptr
                      || dynamic_cast<MNM_Destination_Multimodal *> (
                           _final_dest)
                             ->m_parking_lot
                           == nullptr)
                    {
                      printf ("PnR mode for origin node %d to destination node "
                              "%d is impossible!\n",
                              (int) _node_ID,
                              (int) _veh->get_destination ()
                                ->m_dest_node->m_node_ID);
                      throw std::runtime_error ("PnR routing is wrong");
                    }
                }

              _next_link_ID = m_driving_table->find (_final_dest)
                                ->second->find (_node_ID)
                                ->second;
              if (_next_link_ID < 0)
                {
                  // printf("%d\n", _veh -> get_destination() -> m_Dest_ID);
                  // _shortest_path_tree = m_table -> find(_veh ->
                  // get_destination()) -> second; printf("%d\n",
                  // _shortest_path_tree -> size()); for (auto it :
                  // (*_shortest_path_tree)) printf("%d, %d\n", it.first,
                  // it.second);
                  throw std::runtime_error (
                    "Something wrong in adaptive routing, wrong next link 1");
                }
              // printf("From origin, The next link ID will be %d\n",
              // _next_link_ID());
              _next_link = m_link_factory->get_link (_next_link_ID);
              _veh->set_next_link (_next_link);
              // printf("The next link now it's %d\n", _veh -> get_next_link()
              // -> m_link_ID());
            }
        }
    }

  MNM_Dlink *_link;
  for (auto _link_it : m_link_factory->m_link_map)
    {
      _link = _link_it.second;
      _node_ID = _link->m_to_node->m_node_ID;

      // for pnr cars, find suitable mid dest
      if (!_mid_destination_map.empty ())
        _mid_destination_map.clear ();

      for (auto _veh : _link->m_finished_array)
        {
          _veh_multimodal = dynamic_cast<MNM_Veh_Multimodal *> (_veh);
          if (_veh_multimodal->m_type == MNM_TYPE_ADAPTIVE
              && _veh_multimodal->m_bus_route_ID == TInt (-1))
            {
              if (_link != _veh->get_current_link ())
                {
                  throw std::runtime_error ("Wrong current link!");
                }

              _final_dest = _veh->get_destination ();
              if (_veh_multimodal->m_pnr)
                {
                  // find a mid destination with a parking lot
                  IAssert (_node_ID != _final_dest->m_dest_node->m_node_ID);
                  if (_mid_destination_map.find (_final_dest)
                      == _mid_destination_map.end ())
                    {
                      _mid_dest
                        = find_mid_destination_for_pnr (timestamp, _node_ID,
                                                        _final_dest->m_dest_node
                                                          ->m_node_ID);
                      _mid_destination_map.insert (
                        std::pair<MNM_Destination *,
                                  MNM_Destination *> (_final_dest, _mid_dest));
                    }
                  _final_dest
                    = _mid_destination_map.find (_veh->get_destination ())
                        ->second;
                  if (_final_dest == nullptr
                      || dynamic_cast<MNM_Destination_Multimodal *> (
                           _final_dest)
                             ->m_parking_lot
                           == nullptr)
                    {
                      printf ("PnR mode for origin node %d to destination node "
                              "%d is impossible!\n",
                              (int) _veh->get_origin ()
                                ->m_origin_node->m_node_ID,
                              (int) _veh->get_destination ()
                                ->m_dest_node->m_node_ID);
                      throw std::runtime_error ("PnR routing is wrong");
                    }
                  IAssert (_final_dest != _veh->get_destination ());
                }

              if (_final_dest->m_dest_node->m_node_ID == _node_ID)
                {
                  _veh->set_next_link (nullptr);
                }
              else
                {
                  _next_link_ID = m_driving_table->find (_final_dest)
                                    ->second->find (_node_ID)
                                    ->second;
                  if (_next_link_ID < 0)
                    {
                      printf (
                        "Something wrong in routing, wrong next link 2\n");
                      printf ("The node is %d, the vehicle should head to %d\n",
                              (int) _node_ID,
                              (int) _final_dest->m_dest_node->m_node_ID);
                      // exit(-1);
                      auto _node_I = m_driving_graph->GetNI (_node_ID);
                      if (_node_I.GetOutDeg () > 0)
                        {
                          printf ("Assign randomly!\n");
                          _next_link_ID = _node_I.GetOutEId (
                            MNM_Ults::mod (rand (), _node_I.GetOutDeg ()));
                        }
                      else
                        {
                          printf ("Can't do anything!\n");
                        }
                    }
                  _next_link = m_link_factory->get_link (_next_link_ID);
                  if (_next_link != nullptr)
                    {
                      // printf("Checking future\n");
                      TInt _next_node_ID = _next_link->m_to_node->m_node_ID;
                      if (_next_node_ID != _final_dest->m_dest_node->m_node_ID)
                        {
                          // printf("Destination node is %d\n", _veh ->
                          // get_destination() -> m_dest_node -> m_node_ID());
                          if (m_driving_table->find (_final_dest)
                              == m_driving_table->end ())
                            {
                              printf ("Cannot find Destination\n");
                            }
                          if (m_driving_table->find (_final_dest)
                                ->second->find (_next_node_ID)
                              == m_driving_table->find (_final_dest)
                                   ->second->end ())
                            {
                              printf ("Cannot find _next_node_ID\n");
                            }
                          if (m_driving_table->find (_final_dest)
                                ->second->find (_next_node_ID)
                                ->second
                              == -1)
                            {
                              throw std::runtime_error (
                                "Something wrong for the future node!");
                            }
                          // printf("Pass checking\n");
                        }
                    }
                  _veh->set_next_link (_next_link);
                } // end if else
            }     // end if veh->m_type
        }         // end for veh_it
    }             // end for link_it

  if (!_mid_destination_map.empty ())
    _mid_destination_map.clear ();

  // printf("Finished Adaptive Routing for Vehicles\n");
  return 0;
}

/**************************************************************************
                          Multimodal_Hybrid Routing
**************************************************************************/
MNM_Routing_Multimodal_Hybrid::MNM_Routing_Multimodal_Hybrid (
  const std::string &file_folder, PNEGraph &driving_graph,
  PNEGraph &transit_graph, MNM_Statistics *statistics,
  MNM_OD_Factory *od_factory, MNM_Node_Factory *node_factory,
  MNM_Link_Factory *link_factory, MNM_Busstop_Factory *busstop_factory,
  MNM_Parking_Lot_Factory *parkinglot_factory,
  MNM_Transit_Link_Factory *transitlink_factory, Bus_Path_Table *bus_path_table,
  PnR_Path_Table *pnr_path_table, Path_Table *bustransit_path_table,
  TInt route_frq_fixed, TInt buffer_length)
    : MNM_Routing_Biclass_Hybrid::MNM_Routing_Biclass_Hybrid (file_folder,
                                                              driving_graph,
                                                              statistics,
                                                              od_factory,
                                                              node_factory,
                                                              link_factory,
                                                              route_frq_fixed,
                                                              buffer_length)
{
  m_routing_bus_fixed
    = new MNM_Routing_Bus (driving_graph, od_factory, node_factory,
                           link_factory, bus_path_table, route_frq_fixed,
                           buffer_length, TInt (1));
  m_routing_car_pnr_fixed
    = new MNM_Routing_PnR_Fixed (driving_graph, od_factory, node_factory,
                                 link_factory, pnr_path_table, route_frq_fixed,
                                 TInt (buffer_length / 2));
  m_routing_passenger_fixed
    = new MNM_Routing_PassengerBusTransit_Fixed (transit_graph, od_factory,
                                                 node_factory, busstop_factory,
                                                 parkinglot_factory,
                                                 transitlink_factory,
                                                 bustransit_path_table,
                                                 route_frq_fixed,
                                                 TInt (buffer_length / 2));

  m_routing_multimodal_adaptive
    = new MNM_Routing_Multimodal_Adaptive (file_folder, driving_graph,
                                           transit_graph, statistics,
                                           od_factory, node_factory,
                                           busstop_factory, parkinglot_factory,
                                           link_factory, transitlink_factory);

  ;
}

MNM_Routing_Multimodal_Hybrid::~MNM_Routing_Multimodal_Hybrid ()
{
  delete m_routing_bus_fixed;
  delete m_routing_car_pnr_fixed;
  delete m_routing_passenger_fixed;
  delete m_routing_multimodal_adaptive;
}

int
MNM_Routing_Multimodal_Hybrid::init_routing (Path_Table *driving_path_table)
{
  m_routing_fixed_car->init_routing (driving_path_table);
  // printf("Finished init STATIC and NON-PNR cars routing\n");
  m_routing_fixed_truck->init_routing (driving_path_table);
  // printf("Finished init STATIC and NON-PNR trucks routing\n");
  m_routing_bus_fixed->init_routing (driving_path_table);
  // printf("Finished init STATIC buses routing\n");
  m_routing_car_pnr_fixed->init_routing (driving_path_table);
  // printf("Finished init STATIC and PNR cars routing\n");
  m_routing_passenger_fixed->init_routing (driving_path_table);
  // printf("Finished init STATIC passengers routing\n");
  if (m_routing_multimodal_adaptive->m_working)
    m_routing_multimodal_adaptive->init_routing ();
  // printf("Finished init all ADAPTIVE cars, trucks, and passengers
  // routing\n");
  return 0;
}

int
MNM_Routing_Multimodal_Hybrid::update_routing (TInt timestamp)
{
  m_routing_fixed_car->update_routing (timestamp);
  // printf("Finished update STATIC and NON-PNR cars routing\n");
  m_routing_fixed_truck->update_routing (timestamp);
  // printf("Finished update STATIC and NON-PNR trucks routing\n");
  m_routing_bus_fixed->update_routing (timestamp);
  // printf("Finished update STATIC buses routing\n");
  m_routing_car_pnr_fixed->update_routing (timestamp);
  // printf("Finished update STATIC and PNR cars routing\n");
  m_routing_passenger_fixed->update_routing (timestamp);
  // printf("Finished update STATIC passengers routing\n");
  if (m_routing_multimodal_adaptive->m_working)
    m_routing_multimodal_adaptive->update_routing (timestamp);
  // printf("Finished update all ADAPTIVE cars, trucks, and passengers
  // routing\n");
  return 0;
}

int
MNM_Routing_Multimodal_Hybrid::remove_finished (MNM_Veh *veh, bool del)
{
  if (veh->get_class () == TInt (0))
    {
      if (veh->get_ispnr ())
        {
          m_routing_car_pnr_fixed->remove_finished (veh, del);
        }
      else
        {
          m_routing_fixed_car->remove_finished (veh, del);
        }
    }
  else if (veh->get_class () == TInt (1))
    {
      if (veh->get_bus_route_ID () != -1)
        {
          m_routing_bus_fixed->remove_finished (veh, del);
        }
      else
        {
          m_routing_fixed_truck->remove_finished (veh, del);
        }
    }
  else
    {
      throw std::runtime_error ("Wrong vehicle class");
    }
  return 0;
}

int
MNM_Routing_Multimodal_Hybrid::remove_finished_passenger (
  MNM_Passenger *passenger, bool del)
{
  m_routing_passenger_fixed->remove_finished (passenger, del);
  return 0;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal IO Functions
*******************************************************************************************************************
******************************************************************************************************************/

int
MNM_IO_Multimodal::build_od_factory_multimodal (std::string file_folder,
                                                MNM_ConfReader *conf_reader,
                                                MNM_OD_Factory *od_factory,
                                                MNM_Node_Factory *node_factory,
                                                const std::string &file_name)
{
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
  MNM_Origin_Multimodal *_origin_multimodal;
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
          if (_words.size () == 3)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _pickup_waiting_time = TFlt (std::stod (_words[2]));

              _origin = od_factory->make_origin (_origin_ID, _max_interval,
                                                 _flow_scalar, _frequency);
              _origin_multimodal
                = dynamic_cast<MNM_Origin_Multimodal *> (_origin);
              IAssert (_origin_multimodal != nullptr);
              _origin_multimodal->m_pickup_waiting_time
                = MNM_Ults::round (_pickup_waiting_time / _unit_time);

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
              _dest = od_factory->make_destination (_dest_ID);
              _dest->m_flow_scalar = _flow_scalar;
              /* hook up */
              _dest->m_dest_node
                = (MNM_DMDND *) node_factory->get_node (_node_ID);
              ((MNM_DMDND *) node_factory->get_node (_node_ID))
                ->hook_up_destination (_dest);
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO_Multimodal::build_node_factory_multimodal (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_Node_Factory *node_factory, const std::string &file_name)
{
  /* find file */
  std::string _node_file_name = file_folder + "/" + file_name;
  std::ifstream _node_file;
  _node_file.open (_node_file_name, std::ios::in);

  /* read config */
  TInt _num_of_node = conf_reader->get_int ("num_of_node");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");

  /* read file */
  std::string _line;
  std::vector<std::string> _words;
  TInt _node_ID;
  std::string _type;
  TFlt _veh_convert_factor;

  auto *_node_factory
    = dynamic_cast<MNM_Node_Factory_Multimodal *> (node_factory);
  IAssert (_node_factory != nullptr);
  if (_node_file.is_open ())
    {
      std::getline (_node_file, _line); // skip the first line
      for (int i = 0; i < _num_of_node; ++i)
        {
          std::getline (_node_file, _line);
          // printf("%d\n", i);
          _words = split (trim (_line), ' ');
          if (_words.size () == 3)
            {
              _node_ID = TInt (std::stoi (_words[0]));
              _type = trim (_words[1]);
              _veh_convert_factor = TFlt (std::stod (_words[2]));
              if (_node_factory->m_node_map.find (_node_ID)
                  != _node_factory->m_node_map.end ())
                {
                  throw std::runtime_error ("node already exists");
                }
              if (_type == "FWJ")
                {
                  _node_factory->make_node_multimodal (_node_ID,
                                                       MNM_TYPE_FWJ_MULTIMODAL,
                                                       _flow_scalar,
                                                       _veh_convert_factor);
                  continue;
                }
              else if (_type == "DMOND")
                {
                  _node_factory
                    ->make_node_multimodal (_node_ID,
                                            MNM_TYPE_ORIGIN_MULTIMODAL,
                                            _flow_scalar, _veh_convert_factor);
                  continue;
                }
              else if (_type == "DMDND")
                {
                  _node_factory->make_node_multimodal (_node_ID,
                                                       MNM_TYPE_DEST_MULTIMODAL,
                                                       _flow_scalar,
                                                       _veh_convert_factor);
                  continue;
                }
              else
                {
                  throw std::runtime_error ("Wrong node type");
                }
            }
          else
            {
              throw std::runtime_error (
                "MNM_IO_Multimodal::build_node_factory_multimodal: Wrong "
                "length of line");
            }
        }
      _node_file.close ();
    }
  IAssert (TInt (_node_factory->m_node_map.size ()) == _num_of_node);
  // node_factory -> get_node(TInt(1));
  return 0;
}

int
MNM_IO_Multimodal::build_link_factory_multimodal (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_Link_Factory *link_factory, const std::string &file_name)
{
  /* find file */
  std::string _link_file_name = file_folder + "/" + file_name;
  std::ifstream _link_file;
  _link_file.open (_link_file_name, std::ios::in);

  /* read config */
  TInt _num_of_link = conf_reader->get_int ("num_of_link");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TFlt _unit_time = conf_reader->get_float ("unit_time");

  /* read file */
  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;
  TFlt _lane_hold_cap_car;
  TFlt _lane_flow_cap_car;
  TInt _number_of_lane;
  TFlt _length;
  TFlt _ffs_car;
  std::string _type;
  // new in multiclass vehicle case
  TFlt _lane_hold_cap_truck;
  TFlt _lane_flow_cap_truck;
  TFlt _ffs_truck;
  TFlt _veh_convert_factor;

  auto *_link_factory
    = dynamic_cast<MNM_Link_Factory_Multimodal *> (link_factory);
  IAssert (_link_factory != nullptr);
  if (_link_file.is_open ())
    {
      // printf("Start build link factory.\n");
      std::getline (_link_file, _line); // skip the first line
      for (int i = 0; i < _num_of_link; ++i)
        {
          std::getline (_link_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () == 11)
            {
              _link_ID = TInt (std::stoi (_words[0]));
              _type = trim (_words[1]);
              _length = TFlt (std::stod (_words[2]));
              _ffs_car = TFlt (std::stod (_words[3]));
              _lane_flow_cap_car = TFlt (
                std::stod (_words[4])); // flow capacity (vehicles/hour/lane)
              _lane_hold_cap_car = TFlt (
                std::stod (_words[5])); // jam density (vehicles/mile/lane)
              _number_of_lane = TInt (std::stoi (_words[6]));
              // new in multiclass vehicle case
              _ffs_truck = TFlt (std::stod (_words[7]));
              _lane_flow_cap_truck = TFlt (std::stod (_words[8]));
              _lane_hold_cap_truck = TFlt (std::stod (_words[9]));
              _veh_convert_factor = TFlt (std::stod (_words[10]));

              /* unit conversion */
              // mile -> meter, hour -> second
              _length = _length * TFlt (1600);                 // m
              _ffs_car = _ffs_car * TFlt (1600) / TFlt (3600); // m/s
              _lane_flow_cap_car
                = _lane_flow_cap_car / TFlt (3600); // vehicles/s/lane
              _lane_hold_cap_car
                = _lane_hold_cap_car / TFlt (1600); // vehicles/m/lane
              _ffs_truck = _ffs_truck * TFlt (1600) / TFlt (3600); // m/s
              _lane_flow_cap_truck
                = _lane_flow_cap_truck / TFlt (3600); // vehicles/s/lane
              _lane_hold_cap_truck
                = _lane_hold_cap_truck / TFlt (1600); // vehicles/m/lane

              if (_link_factory->m_link_map.find (_link_ID)
                  != _link_factory->m_link_map.end ())
                {
                  throw std::runtime_error ("link already exists");
                }

              /* build */
              if (_type == "PQ")
                {
                  _link_factory->make_link_multimodal (_link_ID,
                                                       MNM_TYPE_PQ_MULTIMODAL,
                                                       _number_of_lane, _length,
                                                       _lane_hold_cap_car,
                                                       _lane_hold_cap_truck,
                                                       _lane_flow_cap_car,
                                                       _lane_flow_cap_truck,
                                                       _ffs_car, _ffs_truck,
                                                       _unit_time,
                                                       _veh_convert_factor,
                                                       _flow_scalar);
                  continue;
                }
              else if (_type == "CTM")
                {
                  _link_factory->make_link_multimodal (_link_ID,
                                                       MNM_TYPE_CTM_MULTIMODAL,
                                                       _number_of_lane, _length,
                                                       _lane_hold_cap_car,
                                                       _lane_hold_cap_truck,
                                                       _lane_flow_cap_car,
                                                       _lane_flow_cap_truck,
                                                       _ffs_car, _ffs_truck,
                                                       _unit_time,
                                                       _veh_convert_factor,
                                                       _flow_scalar);
                  continue;
                }
              else
                {
                  throw std::runtime_error ("Wrong link type");
                }
            }
          else
            {
              throw std::runtime_error (
                "MNM_IO_Multimodal::build_link_factory_multimodal::Wrong "
                "length of line");
            }
        }
      _link_file.close ();
    }
  IAssert (TInt (_link_factory->m_link_map.size ()) == _num_of_link);
  return 0;
}

int
MNM_IO_Multimodal::build_busstop_factory (const std::string &file_folder,
                                          MNM_ConfReader *conf_reader,
                                          MNM_Busstop_Factory *busstop_factory,
                                          MNM_Link_Factory *link_factory,
                                          const std::string &file_name)
{
  /* read config */
  TInt _num_busstops_physical
    = conf_reader->get_int ("num_of_bus_stop_physical");
  TInt _num_busstops_virtual = conf_reader->get_int ("num_of_bus_stop_virtual");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TInt _historical_waiting_time
    = conf_reader->get_int ("historical_bus_waiting_time");

  TInt _busstop_virtual_ID, _busstop_physical_ID, _link_ID, _route_ID;
  TFlt _link_loc;
  // MNM_Dlink_Ctm_Multimodal *_link;
  MNM_Busstop *_bus_stop;
  MNM_Busstop_Physical *_bus_stop_physical;
  MNM_Busstop_Virtual *_bus_stop_virtual;
  std::string _line;
  std::vector<std::string> _words;
  bool _flg = false;

  /* find file for physical busstops*/
  std::string _busstop_file_name = file_folder + "/" + file_name + "_physical";
  std::ifstream _busstop_file;
  _busstop_file.open (_busstop_file_name, std::ios::in);

  if (_busstop_file.is_open ())
    {
      std::getline (_busstop_file, _line); // skip the first line
      for (int i = 0; i < _num_busstops_physical; ++i)
        {
          std::getline (_busstop_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () >= 4)
            {
              _busstop_physical_ID = TInt (std::stoi (_words[0]));
              _link_ID = TInt (std::stoi (_words[1]));
              _link_loc = TFlt (std::stof (_words[2]));

              /* unit conversion */
              // mile -> meter, hour -> second
              _link_loc = _link_loc * TFlt (1600);

              if (busstop_factory->m_busstop_map.find (_busstop_physical_ID)
                  != busstop_factory->m_busstop_map.end ())
                {
                  throw std::runtime_error (
                    "physical bus stop already exists.");
                }

              /* make busstop factory */
              _bus_stop
                = busstop_factory->make_busstop (_busstop_physical_ID, _link_ID,
                                                 _link_loc, _flow_scalar,
                                                 "physical");
              _bus_stop_physical
                = dynamic_cast<MNM_Busstop_Physical *> (_bus_stop);
              IAssert (_bus_stop_physical != nullptr);
              for (size_t j = 3; j < _words.size (); ++j)
                {
                  _bus_stop_physical->m_route_IDs_vec.emplace_back (
                    std::stoi (_words[j]));
                }

              /* determine cell ID if bus stop on ctm link */
              auto *_link = dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (
                link_factory->get_link (_link_ID));
              if (_link != nullptr)
                { // ctm link
                  TInt cell_ID;
                  if (_link->m_num_cells == 1)
                    {
                      cell_ID = TInt (0);
                      _bus_stop->m_cell_ID = cell_ID;
                    }
                  else
                    {
                      cell_ID
                        = TInt (ceil (_link_loc
                                      / _link->m_cell_array[0]->m_cell_length))
                          - 1;
                      if (cell_ID < 0)
                        {
                          cell_ID = TInt (0);
                        }
                      else if (cell_ID + 1 > _link->m_num_cells)
                        {
                          cell_ID = _link->m_num_cells - 1;
                        }
                      _bus_stop->m_cell_ID = cell_ID;
                    }
                }
            }
          else
            {
              printf ("Wrong line in bus_stop_physical file!\n");
            }
        }
    }
  else
    {
      throw std::runtime_error ("Something wrong in build_busstop_physical!");
    }
  _busstop_file.close ();

  IAssert (TInt (busstop_factory->m_busstop_map.size ())
           == _num_busstops_physical);

  /* find file for virtual busstops */
  _busstop_file_name = file_folder + "/" + file_name + "_virtual";
  _busstop_file.open (_busstop_file_name, std::ios::in);

  if (_busstop_file.is_open ())
    {
      std::getline (_busstop_file, _line); // skip the first line
      for (int i = 0; i < _num_busstops_virtual; ++i)
        {
          std::getline (_busstop_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () == 3)
            {
              _busstop_virtual_ID = TInt (std::stoi (_words[0]));
              _busstop_physical_ID = TInt (std::stoi (_words[1]));
              _route_ID = TInt (std::stoi (_words[2]));

              if (busstop_factory->m_busstop_map.find (_busstop_virtual_ID)
                  != busstop_factory->m_busstop_map.end ())
                {
                  throw std::runtime_error ("virtual bus stop already exists");
                }

              /* make busstop factory */
              _bus_stop_physical = dynamic_cast<MNM_Busstop_Physical *> (
                busstop_factory->get_busstop (_busstop_physical_ID));
              _bus_stop
                = busstop_factory->make_busstop (_busstop_virtual_ID,
                                                 _bus_stop_physical->m_link_ID,
                                                 _bus_stop_physical->m_link_loc,
                                                 _flow_scalar, "virtual");
              _bus_stop_virtual
                = dynamic_cast<MNM_Busstop_Virtual *> (_bus_stop);
              IAssert (_bus_stop_virtual != nullptr);
              IAssert (std::find (_bus_stop_physical->m_route_IDs_vec.begin (),
                                  _bus_stop_physical->m_route_IDs_vec.end (),
                                  _route_ID)
                       != _bus_stop_physical->m_route_IDs_vec.end ());
              _bus_stop_virtual->m_route_ID = _route_ID;
              if (_bus_stop_virtual->m_route_ID == TInt (-1))
                {
                  throw std::runtime_error ("busstop has no bus route");
                }
              _bus_stop_virtual->m_historical_waiting_time
                = _historical_waiting_time;
              _bus_stop_virtual->m_max_boarding_passengers_per_unit_time
                = TInt (round (
                  float (conf_reader->get_int ("unit_time"))
                  / conf_reader->get_int ("boarding_time_per_passenger")));
              _bus_stop_virtual->m_max_alighting_passengers_per_unit_time
                = TInt (round (
                  float (conf_reader->get_int ("unit_time"))
                  / conf_reader->get_int ("alighting_time_per_passenger")));
              _bus_stop_virtual->m_cell_ID = _bus_stop_physical->m_cell_ID;
              _bus_stop_virtual->m_busstop_physical = _bus_stop_physical;
              IAssert (
                std::find (_bus_stop_physical->m_busstop_virtual_vec.begin (),
                           _bus_stop_physical->m_busstop_virtual_vec.end (),
                           _bus_stop_virtual)
                == _bus_stop_physical->m_busstop_virtual_vec.end ());
              _bus_stop_physical->m_busstop_virtual_vec.push_back (
                _bus_stop_virtual);

              /* hook virtual busstops with driving link */
              auto *_link = dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (
                link_factory->get_link (_bus_stop_physical->m_link_ID));
              if (_link != nullptr)
                { // ctm link
                  TInt cell_ID = _bus_stop_physical->m_cell_ID;
                  IAssert (cell_ID > -1);
                  if (_link->m_cell_busstop_vec.find (cell_ID)
                      == _link->m_cell_busstop_vec.end ())
                    {
                      _link->m_cell_busstop_vec.insert (
                        std::pair<TInt, std::vector<MNM_Busstop_Virtual
                                                      *>> (cell_ID,
                                                           std::vector<
                                                             MNM_Busstop_Virtual
                                                               *> ()));
                    }
                  // check if any bus stops with the same route ID fall into the
                  // same cell
                  for (auto *_bus_stop_virtual_tmp :
                       _link->m_cell_busstop_vec.find (cell_ID)->second)
                    {
                      if (_bus_stop_virtual_tmp->m_route_ID
                            == _bus_stop_virtual->m_route_ID
                          && _bus_stop_virtual_tmp->m_cell_ID == cell_ID)
                        {
                          _flg = true; // incorrect
                          printf ("CTM link ID: %d, route_ID: %d, "
                                  "_bus_stop_virtual 1: %d, _bus_stop_physical "
                                  "1: %d, _bus_stop_virtual 2: %d, "
                                  "_bus_stop_physical 2: %d\n",
                                  _link->m_link_ID (),
                                  _bus_stop_virtual_tmp->m_route_ID (),
                                  _bus_stop_virtual_tmp->m_busstop_ID (),
                                  _bus_stop_virtual_tmp->m_busstop_physical
                                    ->m_busstop_ID (),
                                  _bus_stop_virtual->m_busstop_ID (),
                                  _bus_stop_virtual->m_busstop_physical
                                    ->m_busstop_ID ());
                        }
                    }
                  _link->m_cell_busstop_vec.find (cell_ID)->second.push_back (
                    _bus_stop_virtual);
                  _link->m_busstop_vec.push_back (_bus_stop_virtual);
                }
              else
                {
                  auto *_link = dynamic_cast<MNM_Dlink_Pq_Multimodal *> (
                    link_factory->get_link (_bus_stop_physical->m_link_ID));
                  _link->m_busstop_vec.push_back (_bus_stop_virtual);
                  // Pq link only uses ffs_car
                  // _timeloc is in intervals
                  TInt _timeloc
                    = TInt (ceil (_bus_stop_physical->m_link_loc
                                  / _link->m_ffs_car / _link->m_unit_time))
                      - 1;
                  if (_timeloc >= _link->m_max_stamp)
                    _timeloc = _link->m_max_stamp - 1;
                  if (_timeloc < 0)
                    _timeloc = TInt (0);
                  // check if any bus stops with the same route ID fall into the
                  // same time stamp
                  for (auto _it : _link->m_busstop_timeloc_map)
                    {
                      if (_it.second == _timeloc
                          && _it.first->m_route_ID
                               == _bus_stop_virtual->m_route_ID)
                        {
                          _flg = true; // incorrect
                          printf ("PQ link ID: %d, route_ID: %d, "
                                  "_bus_stop_virtual 1: %d, _bus_stop_physical "
                                  "1: %d, _bus_stop_virtual 2: %d, "
                                  "_bus_stop_physical 2: %d\n",
                                  _link->m_link_ID (), _it.first->m_route_ID (),
                                  _it.first->m_busstop_ID (),
                                  _it.first->m_busstop_physical
                                    ->m_busstop_ID (),
                                  _bus_stop_virtual->m_busstop_ID (),
                                  _bus_stop_virtual->m_busstop_physical
                                    ->m_busstop_ID ());
                        }
                    }
                  _link->m_busstop_timeloc_map.insert (
                    std::pair<MNM_Busstop_Virtual *, TInt> (_bus_stop_virtual,
                                                            _timeloc));
                }
            }
          else
            {
              printf ("Wrong line in bus_stop_virtual file!\n");
            }
        }
    }
  else
    {
      throw std::runtime_error ("Something wrong in build_busstop_virtual!");
    }
  _busstop_file.close ();
  if (_flg)
    {
      throw std::runtime_error (
        "some bus stops are too close, consider merging them");
    }
  IAssert (TInt (busstop_factory->m_busstop_map.size ())
           == _num_busstops_physical + _num_busstops_virtual);
  return 0;
}

int
MNM_IO_Multimodal::build_parkinglot_factory (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_Parking_Lot_Factory *parkinglot_factory, MNM_Node_Factory *node_factory,
  MNM_Link_Factory *link_factory, MNM_Passenger_Factory *passenger_factory,
  MNM_Veh_Factory *veh_factory, const std::string &file_name)
{
  /* find file */
  std::string _parkinglot_file_name = file_folder + "/" + file_name;
  std::ifstream _parkinglot_file;
  _parkinglot_file.open (_parkinglot_file_name, std::ios::in);

  /* read config */
  TInt _num_parkinglots = conf_reader->get_int ("num_of_parking_lot");
  TFlt _unit_time = conf_reader->get_float ("unit_time");

  TInt _parkinglot_ID, _node_ID;
  MNM_Parking_Lot *_parkinglot;
  TFlt _price, _price_surge_coeff, _avg_parking_time, _capacity;

  // MNM_Dlink_Ctm_Multimodal *_link;
  std::string _line;
  std::vector<std::string> _words;

  if (_parkinglot_file.is_open ())
    {
      std::getline (_parkinglot_file, _line); // skip the first line
      for (int i = 0; i < _num_parkinglots; ++i)
        {
          std::getline (_parkinglot_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () == 6)
            {
              _parkinglot_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _price = TFlt (std::stof (_words[2]));
              _price_surge_coeff = TFlt (std::stof (_words[3]));
              _avg_parking_time = TFlt (std::stof (_words[4]));
              _capacity = TFlt (std::stof (_words[5]));

              /* unit conversion */
              // mile -> meter, hour -> second -> intervals
              _avg_parking_time /= _unit_time;

              if (parkinglot_factory->m_parking_lot_map.find (_parkinglot_ID)
                  != parkinglot_factory->m_parking_lot_map.end ())
                {
                  throw std::runtime_error ("parking lot already exists");
                }

              /* make parkinglot factory */
              _parkinglot
                = parkinglot_factory->make_parking_lot (_parkinglot_ID,
                                                        _node_ID,
                                                        passenger_factory,
                                                        veh_factory, _price,
                                                        _price_surge_coeff,
                                                        _avg_parking_time,
                                                        _capacity, _unit_time);

              /* hook parking lots with destination node */
              _parkinglot->m_dest_node = dynamic_cast<MNM_DMDND_Multiclass *> (
                node_factory->get_node (_node_ID));
              IAssert (_parkinglot->m_dest_node != nullptr);
              dynamic_cast<MNM_Destination_Multimodal *> (
                _parkinglot->m_dest_node->m_dest)
                ->m_parking_lot
                = _parkinglot;
            }
          else
            {
              printf ("Wrong line in parking_lots file!\n");
            }
        }
    }
  else
    {
      throw std::runtime_error ("Something wrong in build_parkinglot");
    }
  _parkinglot_file.close ();
  IAssert (TInt (parkinglot_factory->m_parking_lot_map.size ())
           == _num_parkinglots);
  return 0;
}

int
MNM_IO_Multimodal::build_walkinglink_factory (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_Transit_Link_Factory *transit_link_factory,
  MNM_Node_Factory *node_factory, MNM_Busstop_Factory *busstop_factory,
  MNM_Parking_Lot_Factory *parkinglot_factory, const std::string &file_name)
{
  /* find file */
  std::string _walkinglink_file_name = file_folder + "/" + file_name;
  std::ifstream _walkinglink_file;
  _walkinglink_file.open (_walkinglink_file_name, std::ios::in);

  /* read config */
  TInt _num_walkinglinks = conf_reader->get_int ("num_of_walking_link");
  TInt _historical_bus_waiting_time
    = conf_reader->get_int ("historical_bus_waiting_time");

  TInt _walkinglink_ID, _from_node_ID, _to_node_ID;
  TFlt _walking_time;
  TFlt _unit_time = conf_reader->get_float ("unit_time");
  std::string _walking_type;
  std::string _from_node_type;
  std::string _to_node_type;

  MNM_Transit_Link *_transit_link;
  MNM_Walking_Link *_walking_link;

  std::string _line;
  std::vector<std::string> _words;

  if (_walkinglink_file.is_open ())
    {
      std::getline (_walkinglink_file, _line); // skip the first line
      for (int i = 0; i < _num_walkinglinks; ++i)
        {
          std::getline (_walkinglink_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () == 7)
            {
              _walkinglink_ID = TInt (std::stoi (_words[0]));
              _from_node_ID = TInt (std::stoi (_words[1]));
              _to_node_ID = TInt (std::stoi (_words[2]));
              _from_node_type = trim (_words[3]);
              _to_node_type = trim (_words[4]);
              _walking_type = trim (_words[5]);
              _walking_time = TFlt (std::stof (_words[6])); // seconds
              // std::cout << "start reading: " << _line << std::endl;
              /* unit conversion */
              // mile -> meter, hour -> second

              if (transit_link_factory->m_transit_link_map.find (
                    _walkinglink_ID)
                  != transit_link_factory->m_transit_link_map.end ())
                {
                  throw std::runtime_error ("walking link already exists");
                }

              /* make walkinglink factory */
              _transit_link
                = transit_link_factory
                    ->make_transit_link (_walkinglink_ID,
                                         MNM_TYPE_WALKING_MULTIMODAL,
                                         _walking_type, _from_node_type,
                                         _to_node_type, _from_node_ID,
                                         _to_node_ID, _walking_time,
                                         _unit_time);
              _walking_link = dynamic_cast<MNM_Walking_Link *> (_transit_link);
              IAssert (_walking_link != nullptr);
              // for implicit bus modeling
              if (_walking_type == "boarding")
                {
                  _walking_link->m_historical_bus_waiting_time
                    = _historical_bus_waiting_time;
                }
              else
                {
                  _walking_link->m_historical_bus_waiting_time = 0;
                }
              /* hook up walking link with bus stop, parking lot, origin, and
               * destination */
              if (_from_node_type == "origin")
                {
                  dynamic_cast<MNM_Origin_Multimodal *> (
                    ((MNM_DMOND *) node_factory->get_node (_from_node_ID))
                      ->m_origin)
                    ->m_walking_out_links_vec.push_back (_walking_link);
                }
              if (_to_node_type == "destination")
                {
                  dynamic_cast<MNM_Destination_Multimodal *> (
                    ((MNM_DMDND *) node_factory->get_node (_to_node_ID))
                      ->m_dest)
                    ->m_walking_in_links_vec.push_back (_walking_link);
                }
              if (_from_node_type == "bus_stop_physical")
                {
                  if (_to_node_type == "bus_stop_virtual")
                    {
                      dynamic_cast<MNM_Busstop_Physical *> (
                        busstop_factory->get_busstop (_from_node_ID))
                        ->m_boarding_links_vec.push_back (_walking_link);
                    }
                  else
                    {
                      dynamic_cast<MNM_Busstop_Physical *> (
                        busstop_factory->get_busstop (_from_node_ID))
                        ->m_walking_out_links_vec.push_back (_walking_link);
                    }
                }
              if (_to_node_type == "bus_stop_physical")
                {
                  if (_from_node_type == "bus_stop_virtual")
                    {
                      dynamic_cast<MNM_Busstop_Physical *> (
                        busstop_factory->get_busstop (_to_node_ID))
                        ->m_alighting_links_vec.push_back (_walking_link);
                    }
                  else
                    {
                      dynamic_cast<MNM_Busstop_Physical *> (
                        busstop_factory->get_busstop (_to_node_ID))
                        ->m_walking_in_links_vec.push_back (_walking_link);
                    }
                }
              if (_from_node_type == "bus_stop_virtual")
                {
                  dynamic_cast<MNM_Busstop_Virtual *> (
                    busstop_factory->get_busstop (_from_node_ID))
                    ->m_alighting_link
                    = _walking_link;
                }
              if (_to_node_type == "bus_stop_virtual")
                {
                  dynamic_cast<MNM_Busstop_Virtual *> (
                    busstop_factory->get_busstop (_to_node_ID))
                    ->m_boarding_link
                    = _walking_link;
                }
              if (_from_node_type == "parking_lot")
                {
                  // parking lot is always associated with a destination node
                  dynamic_cast<MNM_Destination_Multimodal *> (
                    ((MNM_DMDND *) node_factory->get_node (_from_node_ID))
                      ->m_dest)
                    ->m_parking_lot->m_walking_out_links_vec.push_back (
                      _walking_link);
                }
              // std::cout << "success!" << std::endl;
            }
          else
            {
              printf ("Wrong line in walking_link file!\n");
            }
        }
    }
  else
    {
      throw std::runtime_error ("Something wrong in build_walkinglink");
    }

  _walkinglink_file.close ();
  return 0;
}

int
MNM_IO_Multimodal::build_buslink_factory (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_Transit_Link_Factory *transit_link_factory,
  MNM_Busstop_Factory *busstop_factory, MNM_Link_Factory *link_factory,
  const std::string &file_name)
{
  /* find file */
  std::string _buslink_file_name = file_folder + "/" + file_name;
  std::ifstream _buslink_file;
  _buslink_file.open (_buslink_file_name, std::ios::in);

  /* read config */
  TInt _num_buslinks = conf_reader->get_int ("num_of_bus_link");

  TInt _buslink_ID, _from_busstop_ID, _to_busstop_ID, _route_ID;
  TFlt _bus_fftt, _len;
  TFlt _unit_time = conf_reader->get_float ("unit_time");

  MNM_Transit_Link *_transit_link;
  MNM_Bus_Link *_bus_link;
  TInt _driving_link_ID;

  std::string _line;
  std::vector<std::string> _words;

  if (_buslink_file.is_open ())
    {
      std::getline (_buslink_file, _line); // skip the first line
      for (int i = 0; i < _num_buslinks; ++i)
        {
          std::getline (_buslink_file, _line);
          _words = split (trim (_line), ' ');
          if (_words.size () >= 7)
            {
              _buslink_ID = TInt (std::stoi (_words[0]));
              _from_busstop_ID = TInt (std::stoi (_words[1]));
              _to_busstop_ID = TInt (std::stoi (_words[2]));
              _len = TFlt (std::stof (_words[3]));
              _bus_fftt = TFlt (std::stof (_words[4]));
              _route_ID = TInt (std::stoi (_words[5]));

              if (_from_busstop_ID == _to_busstop_ID)
                {
                  printf ("bus link ID: %d, _from_busstop_ID: %d, "
                          "_to_busstop_ID: %d\n",
                          (int) _buslink_ID, (int) _from_busstop_ID,
                          (int) _to_busstop_ID);
                }

              /* unit conversion */
              // mile -> meter, hour -> second
              _len = _len * 1600.;
              _bus_fftt = _bus_fftt * 3600.;

              if (transit_link_factory->m_transit_link_map.find (_buslink_ID)
                  != transit_link_factory->m_transit_link_map.end ())
                {
                  throw std::runtime_error ("bus link already exists");
                }

              /* make buslink factory */
              _transit_link
                = transit_link_factory
                    ->make_transit_link (_buslink_ID, MNM_TYPE_BUS_MULTIMODAL,
                                         " ", "bus_stop", "bus_stop",
                                         _from_busstop_ID, _to_busstop_ID,
                                         _bus_fftt, _unit_time);
              _bus_link = dynamic_cast<MNM_Bus_Link *> (_transit_link);
              IAssert (_bus_link != nullptr);
              _bus_link->m_length = _len;
              // each link corresponds to a single route
              _bus_link->m_route_ID = _route_ID;

              /* hook up bus link with bus stop */
              _bus_link->m_from_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
                busstop_factory->get_busstop (_from_busstop_ID));
              _bus_link->m_to_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
                busstop_factory->get_busstop (_to_busstop_ID));
              IAssert (_bus_link->m_from_busstop != nullptr
                       && _bus_link->m_to_busstop != nullptr);
              dynamic_cast<MNM_Busstop_Virtual *> (
                busstop_factory->get_busstop (_from_busstop_ID))
                ->m_bus_out_link
                = _bus_link;
              dynamic_cast<MNM_Busstop_Virtual *> (
                busstop_factory->get_busstop (_to_busstop_ID))
                ->m_bus_in_link
                = _bus_link;

              /* hook up overlapped driving links with bus */
              for (size_t j = 6; j < _words.size (); j++)
                {
                  _driving_link_ID = TInt (std::stoi (_words[j]));
                  // TODO:
                  // if (std::find_if(_bus_link ->
                  // m_overlapped_driving_link_vec.begin(), _bus_link ->
                  // m_overlapped_driving_link_vec.end(),
                  //     [&_driving_link_ID](const MNM_Dlink *_l){return _l ->
                  //     m_link_ID == _driving_link_ID;}) != _bus_link ->
                  //     m_overlapped_driving_link_vec.end()) { _bus_link ->
                  //     m_overlapped_driving_link_vec.push_back(link_factory ->
                  //     get_link(_driving_link_ID));
                  // }
                  // else {
                  //     printf("MNM_IO_Multimodal::build_buslink_factory,
                  //     Repeated overlapped driving links for Bus Link %d\n",
                  //     _buslink_ID());
                  // }
                  _bus_link->m_overlapped_driving_link_vec.push_back (
                    link_factory->get_link (_driving_link_ID));
                }

              _bus_link->get_overlapped_driving_link_length_portion ();
            }
          else
            {
              printf ("Wrong line in bus_link file!\n");
            }
        }
    }
  else
    {
      throw std::runtime_error ("Something wrong in build_buslink");
    }

  _buslink_file.close ();
  return 0;
}

PNEGraph
MNM_IO_Multimodal::build_bus_transit_graph (
  MNM_ConfReader *conf_reader, MNM_Transit_Link_Factory *transit_link_factory)
{
  TInt _num_buslinks = conf_reader->get_int ("num_of_bus_link");
  TInt _num_walkinglinks = conf_reader->get_int ("num_of_walking_link");
  IAssert (_num_buslinks + _num_walkinglinks
           == (int) transit_link_factory->m_transit_link_map.size ());

  // multigraph, more than one link can exist between two nodes
  PNEGraph _graph = PNEGraph::TObj::New ();

  int _link_ID, _from_ID, _to_ID;

  for (auto _it : transit_link_factory->m_transit_link_map)
    {
      _link_ID = _it.first;
      _from_ID = _it.second->m_from_node_ID;
      _to_ID = _it.second->m_to_node_ID;
      if (!_graph->IsNode (_from_ID))
        {
          _graph->AddNode (_from_ID);
        }
      if (!_graph->IsNode (_to_ID))
        {
          _graph->AddNode (_to_ID);
        }
      IAssert (_from_ID != _to_ID);
      _graph->AddEdge (_from_ID, _to_ID, _link_ID);
      // std::cout << "edge ID: " << _graph -> GetEI(_from_ID, _to_ID).GetId()
      // << "\n";
    }
  _graph->Defrag ();
  // std::cout << "node ID: " << _graph -> IsNode(2000436) << "\n";
  return _graph;
}

int
MNM_IO_Multimodal::build_passenger_demand (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_OD_Factory *od_factory,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &passenger_demand,
  const std::string &file_name)
{
  /* find file for passenger demand */
  std::string _demand_file_name = file_folder + "/" + file_name;
  std::ifstream _demand_file;
  _demand_file.open (_demand_file_name, std::ios::in);

  /* read config */
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _num_OD = conf_reader->get_int ("OD_pair_passenger");

  /* build passenger demand */
  TInt _origin_ID, _dest_ID, _origin_node_ID, _dest_node_ID;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open ())
    {
      std::getline (_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_OD; ++i)
        {
          std::getline (_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval + 2))
            {
              _origin_ID = TInt (std::stoi (_words[0]));
              _dest_ID = TInt (std::stoi (_words[1]));
              _origin_node_ID
                = od_factory->get_origin (_origin_ID)->m_origin_node->m_node_ID;
              _dest_node_ID = od_factory->get_destination (_dest_ID)
                                ->m_dest_node->m_node_ID;
              if (passenger_demand.find (_origin_node_ID)
                  == passenger_demand.end ())
                {
                  passenger_demand.insert (
                    std::pair<TInt, std::unordered_map<
                                      TInt, TFlt *>> (_origin_node_ID,
                                                      std::unordered_map<
                                                        TInt, TFlt *> ()));
                }
              if (passenger_demand.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                  == passenger_demand.find (_origin_node_ID)->second.end ())
                {
                  TFlt *_demand_vector
                    = (TFlt *) malloc (sizeof (TFlt) * _max_interval);
                  memset (_demand_vector, 0x0, sizeof (TFlt) * _max_interval);
                  passenger_demand.find (_origin_node_ID)
                    ->second.insert (
                      std::pair<TInt, TFlt *> (_dest_node_ID, _demand_vector));
                }
              for (int j = 0; j < _max_interval; ++j)
                {
                  passenger_demand.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second[j]
                    = TFlt (std::stod (_words[j + 2]));
                }
            }
          else
            {
              throw std::runtime_error (
                "Something wrong in build_demand_passenger");
            }
        }
      _demand_file.close ();
    }

  return 0;
}

int
MNM_IO_Multimodal::build_vehicle_demand_multimodal (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_OD_Factory *od_factory, const std::string &file_name_driving,
  const std::string &file_name_bus, const bool use_explicit_bus)
{
  /* find file for car and truck demand */
  std::string _demand_file_name = file_folder + "/" + file_name_driving;
  std::ifstream _demand_file;
  _demand_file.open (_demand_file_name, std::ios::in);

  /* find file for bus demand */
  std::string _bus_demand_file_name = file_folder + "/" + file_name_bus;
  std::ifstream _bus_demand_file;
  _bus_demand_file.open (_bus_demand_file_name, std::ios::in);

  /* read config */
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TInt _unit_time = conf_reader->get_int ("unit_time");
  TInt _num_of_minute = int (conf_reader->get_int ("assign_frq"))
                        / (60 / _unit_time); // the releasing strategy is
                                             // assigning vehicles per 1 minute
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _num_OD_driving = conf_reader->get_int ("OD_pair_driving");
  TInt _num_bus_routes = conf_reader->get_int ("num_bus_routes");

  TInt _init_demand_split = conf_reader->get_int ("init_demand_split");

  /* build car and truck demand for origin nodes */
  TInt _origin_ID, _dest_ID;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open () && _num_OD_driving > 0)
    {
      // printf("Start build demand profile.\n");
      TFlt *_demand_vector_car
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt *_demand_vector_truck
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt _demand_car;
      TFlt _demand_truck;

      std::getline (_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_OD_driving; ++i)
        {
          std::getline (_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval * 2 + 2))
            {
              _origin_ID = TInt (std::stoi (_words[0]));
              _dest_ID = TInt (std::stoi (_words[1]));
              memset (_demand_vector_car, 0x0,
                      sizeof (TFlt) * _max_interval * _num_of_minute);
              memset (_demand_vector_truck, 0x0,
                      sizeof (TFlt) * _max_interval * _num_of_minute);
              // the releasing strategy is assigning vehicles per 1 minute, so
              // disaggregate 15-min demand into 1-min demand
              for (int j = 0; j < _max_interval; ++j)
                {
                  // _demand_car = TFlt(std::stod(_words[j + 2])) /
                  // TFlt(_num_of_minute); _demand_truck =
                  // TFlt(std::stod(_words[j + _max_interval + 2])) /
                  // TFlt(_num_of_minute); for (int k = 0; k < _num_of_minute;
                  // ++k){
                  //     _demand_vector_car[j * _num_of_minute + k] =
                  //     _demand_car; _demand_vector_truck[j * _num_of_minute +
                  //     k] = _demand_truck;
                  // }

                  if (_init_demand_split == 0)
                    {
                      _demand_car = TFlt (std::stod (_words[j + 2]));
                      _demand_truck
                        = TFlt (std::stod (_words[j + _max_interval + 2]));
                      _demand_vector_car[j * _num_of_minute] = _demand_car;
                      _demand_vector_truck[j * _num_of_minute] = _demand_truck;
                    }
                  else if (_init_demand_split == 1)
                    {
                      // find suitable releasing interval so that the
                      // agent-based DNL is feasible
                      for (int p = 0; p < _num_of_minute; ++p)
                        {
                          _demand_car = TFlt (std::stod (_words[j + 2]))
                                        / TFlt (_num_of_minute - p);
                          // if (round(_demand_car * _flow_scalar) >= 1){
                          if (floor (_demand_car * _flow_scalar) >= 1)
                            {
                              for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                  _demand_vector_car[j * _num_of_minute + k]
                                    = _demand_car;
                                }
                              break;
                            }
                        }
                      for (int p = 0; p < _num_of_minute; ++p)
                        {
                          _demand_truck
                            = TFlt (std::stod (_words[j + _max_interval + 2]))
                              / TFlt (_num_of_minute - p);
                          // if (round(_demand_truck * _flow_scalar) >= 1){
                          if (floor (_demand_truck * _flow_scalar) >= 1)
                            {
                              for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                  _demand_vector_truck[j * _num_of_minute + k]
                                    = _demand_truck;
                                }
                              break;
                            }
                        }
                    }
                  else
                    {
                      throw std::runtime_error ("Wrong init_demand_split");
                    }
                }
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (
                od_factory->get_origin (_origin_ID));
              _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                od_factory->get_destination (_dest_ID));
              _origin->add_dest_demand_multiclass (_dest, _demand_vector_car,
                                                   _demand_vector_truck);
            }
          else
            {
              free (_demand_vector_car);
              free (_demand_vector_truck);
              throw std::runtime_error (
                "Something wrong in build_vehicle_demand_multimodal");
            }
        }
      free (_demand_vector_car);
      free (_demand_vector_truck);
      _demand_file.close ();
    }
  else
    {
      printf ("driving_demand does not exist\n");
    }

  /* build bus demand for origin nodes*/
  TInt _route_ID;
  if (_bus_demand_file.is_open () && _num_bus_routes > 0)
    {
      // printf("Start build demand profile.\n");
      TFlt *_demand_vector_bus
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt _demand_bus;

      std::getline (_bus_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_bus_routes; ++i)
        {
          std::getline (_bus_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval + 3))
            {
              _origin_ID = TInt (std::stoi (_words[0]));
              _dest_ID = TInt (std::stoi (_words[1]));
              _route_ID = TInt (std::stoi (_words[2]));
              memset (_demand_vector_bus, 0x0,
                      sizeof (TFlt) * _max_interval * _num_of_minute);
              if (use_explicit_bus)
                {
                  // the releasing strategy is assigning vehicles per 1 minute,
                  // so disaggregate 15-min demand into 1-min demand
                  for (int j = 0; j < _max_interval; ++j)
                    {
                      // _demand_bus = TFlt(std::stod(_words[j + 3])) /
                      // TFlt(_num_of_minute); for (int k = 0; k <
                      // _num_of_minute; ++k){
                      //     _demand_vector_bus[j * _num_of_minute + k] =
                      //     _demand_bus;
                      // }

                      if (_init_demand_split == 0)
                        {
                          _demand_bus = TFlt (std::stod (_words[j + 3]));
                          _demand_vector_bus[j * _num_of_minute] = _demand_bus;
                        }
                      else if (_init_demand_split == 1)
                        {
                          // find suitable releasing interval so that the
                          // agent-based DNL is feasible
                          for (int p = 0; p < _num_of_minute; ++p)
                            {
                              _demand_bus = TFlt (std::stod (_words[j + 3]))
                                            / TFlt (_num_of_minute - p);
                              // if (round(_demand_bus * _flow_scalar) >= 1){
                              if (floor (_demand_bus * _flow_scalar) >= 1)
                                {
                                  for (int k = 0; k < _num_of_minute - p; ++k)
                                    {
                                      _demand_vector_bus[j * _num_of_minute + k]
                                        = _demand_bus;
                                    }
                                  break;
                                }
                            }
                        }
                      else
                        {
                          throw std::runtime_error (
                            "Wrong init_demand_split\n");
                        }
                    }
                }
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (
                od_factory->get_origin (_origin_ID));
              _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                od_factory->get_destination (_dest_ID));
              _origin->add_dest_demand_bus (_dest, _route_ID,
                                            _demand_vector_bus);
            }
          else
            {
              free (_demand_vector_bus);
              throw std::runtime_error (
                "Something wrong in build_vehicle_demand_multimodal");
            }
        }
      free (_demand_vector_bus);
      _bus_demand_file.close ();
    }
  else
    {
      printf ("bus_demand does not exist\n");
    }
  return 0;
}

int
MNM_IO_Multimodal::build_pnr_demand (const std::string &file_folder,
                                     MNM_ConfReader *conf_reader,
                                     MNM_OD_Factory *od_factory,
                                     const std::string &file_name)
{
  /* find file for pnr demand */
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
  TInt _num_OD_pnr = conf_reader->get_int ("OD_pair_pnr");
  // for DUE
  if (_num_OD_pnr <= 0)
    {
      _demand_file.close ();
      printf ("OD pairs for PnR not set!\n");
      return 0;
    }

  TInt _init_demand_split = conf_reader->get_int ("init_demand_split");

  /* build pnr car demand for origin nodes */
  TInt _origin_ID, _dest_ID;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open ())
    {
      // printf("Start build demand profile.\n");
      TFlt *_demand_vector
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt _demand;

      std::getline (_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_OD_pnr; ++i)
        {
          std::getline (_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval + 2))
            {
              _origin_ID = TInt (std::stoi (_words[0]));
              _dest_ID = TInt (std::stoi (_words[1]));
              memset (_demand_vector, 0x0,
                      sizeof (TFlt) * _max_interval * _num_of_minute);
              // the original releasing strategy is assigning vehicles per 1
              // minute, so disaggregate 15-min demand into 1-min demand
              for (int j = 0; j < _max_interval; ++j)
                {
                  if (_init_demand_split == 0)
                    {
                      _demand = TFlt (std::stod (_words[j + 2]));
                      _demand_vector[j * _num_of_minute] = _demand;
                    }
                  else if (_init_demand_split == 1)
                    {
                      // find suitable releasing interval so that the
                      // agent-based DNL is feasible
                      for (int p = 0; p < _num_of_minute; ++p)
                        {
                          _demand = TFlt (std::stod (_words[j + 2]))
                                    / TFlt (_num_of_minute - p);
                          // if (round(_demand * _flow_scalar) >= 1){
                          if (floor (_demand * _flow_scalar) >= 1)
                            {
                              for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                  _demand_vector[j * _num_of_minute + k]
                                    = _demand;
                                }
                              break;
                            }
                        }
                    }
                  else
                    {
                      throw std::runtime_error ("Wrong init_demand_split");
                    }
                }
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (
                od_factory->get_origin (_origin_ID));
              _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                od_factory->get_destination (_dest_ID));
              _origin->add_dest_demand_pnr_car (_dest, _demand_vector);
            }
          else
            {
              free (_demand_vector);
              throw std::runtime_error ("Something wrong in build_pnr_demand");
            }
        }
      free (_demand_vector);
      _demand_file.close ();
    }
  else
    {
      printf ("pnr_demand does not exist\n");
    }
  return 0;
}

int
MNM_IO_Multimodal::build_bustransit_demand (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            const std::string &file_name)
{
  /* find file for passenger demand for bus transit */
  std::string _demand_file_name = file_folder + "/" + file_name;
  std::ifstream _demand_file;
  _demand_file.open (_demand_file_name, std::ios::in);

  /* read config */
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TInt _unit_time = conf_reader->get_int ("unit_time");
  TInt _num_of_minute
    = int (conf_reader->get_int ("assign_frq")) / (60 / _unit_time);
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _num_OD = conf_reader->get_int ("OD_pair_bustransit");
  // for DUE
  if (_num_OD <= 0)
    {
      _demand_file.close ();
      printf ("OD pairs for passenger bus transit not set!\n");
      return 0;
    }

  TInt _init_demand_split = conf_reader->get_int ("init_demand_split");

  // force the passengers to be released as a lump sum
  // _init_demand_split = 0;

  /* build passenger demand for assignment */
  TInt _origin_ID, _dest_ID;
  TFlt _demand;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open ())
    {
      TFlt *_demand_vector
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);

      // printf("Start build demand profile.\n");
      std::getline (_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_OD; ++i)
        {
          std::getline (_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval + 2))
            {
              _origin_ID = TInt (std::stoi (_words[0]));
              _dest_ID = TInt (std::stoi (_words[1]));
              memset (_demand_vector, 0x0,
                      sizeof (TFlt) * _max_interval * _num_of_minute);
              // the original releasing strategy is assigning vehicles per 1
              // minute, so disaggregate 15-min demand into 1-min demand
              for (int j = 0; j < _max_interval; ++j)
                {
                  if (_init_demand_split == 0)
                    {
                      _demand = TFlt (std::stod (_words[j + 2]));
                      _demand_vector[j * _num_of_minute] = _demand;
                    }
                  else if (_init_demand_split == 1)
                    {
                      // find suitable releasing interval so that the
                      // agent-based DNL is feasible uniform
                      for (int p = 0; p < _num_of_minute; ++p)
                        {
                          _demand = TFlt (std::stod (_words[j + 2]))
                                    / TFlt (_num_of_minute - p);
                          // important to use floor(), since passenger is not
                          // amplified by flow_scalar if (floor(_demand) >= 1){
                          // add flow_scalar to passenger
                          if (floor (_demand * _flow_scalar) >= 1)
                            {
                              for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                  _demand_vector[j * _num_of_minute + k]
                                    = _demand;
                                }
                              break;
                            }
                        }
                    }
                  else
                    {
                      throw std::runtime_error ("Wrong init_demand_split");
                    }
                }
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (
                od_factory->get_origin (_origin_ID));
              _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                od_factory->get_destination (_dest_ID));
              _origin->add_dest_demand_passenger_bus (_dest, _demand_vector);
            }
          else
            {
              throw std::runtime_error (
                "Something wrong in build_bustransit_demand");
            }
        }
      _demand_file.close ();
    }
  else
    {
      printf ("bustransit_demand does not exist\n");
    }
  return 0;
}

Bus_Path_Table *
MNM_IO_Multimodal::load_bus_path_table (const PNEGraph &graph, TInt num_path,
                                        MNM_Node_Factory *node_factory,
                                        MNM_Busstop_Factory *busstop_factory,
                                        TInt num_release_interval,
                                        const std::string &path_file_name,
                                        const std::string &route_file_name)
{
  printf ("Loading Bus Path Table!\n");
  TInt Num_Path = num_path;
  printf ("Number of bus paths %d\n", Num_Path ());

  if (Num_Path <= 0)
    {
      printf ("Finish Loading Bus Path Table, which is nullptr!\n");
      return nullptr;
    }

  MNM_Node_Factory_Multimodal *node_factory_multimodal;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  TFlt *_demand;
  TFlt _demand_tot;
  if (num_release_interval > 0)
    {
      node_factory_multimodal
        = dynamic_cast<MNM_Node_Factory_Multimodal *> (node_factory);
    }

  std::ifstream _path_table_file;
  std::ifstream _route_file;

  _path_table_file.open (path_file_name, std::ios::in);
  _route_file.open (route_file_name, std::ios::in);
  auto *_path_table = new Bus_Path_Table ();

  /* read file */
  std::string _path_line, _route_line;
  std::vector<std::string> _path_words, _route_words;
  TInt _origin_node_ID, _dest_node_ID, _node_ID, _route_ID, _busstop_ID;
  std::unordered_map<TInt, std::unordered_map<TInt, MNM_BusPath *> *>
    *_new_map_1; // <destID, <routeID, Path>>
  std::unordered_map<TInt, MNM_BusPath *> *_new_map_2; // <routeID, Path>
  MNM_BusPath *_path;
  TInt _from_ID, _to_ID, _link_ID;
  TInt _path_ID_counter = 0;
  if (_path_table_file.is_open () && _route_file.is_open ())
    {
      std::getline (_path_table_file, _path_line); // skip the first line
      std::getline (_route_file, _route_line);     // skip the first line
      for (int i = 0; i < Num_Path; ++i)
        {
          std::getline (_path_table_file, _path_line);
          std::getline (_route_file, _route_line);
          // std::cout << "Processing: " << _line << "\n";
          _path_words = split (trim (_path_line), ' '); // returns a vector
          _route_words = split (trim (_route_line), ' ');
          if ((_path_words.size () >= 3) && (_route_words.size () >= 3))
            {
              _origin_node_ID = TInt (std::stoi (_path_words[0]));
              _dest_node_ID = TInt (std::stoi (_path_words[1]));
              _route_ID = TInt (std::stoi (_path_words[2]));

              if (num_release_interval > 0)
                {
                  _origin = dynamic_cast<MNM_Origin_Multimodal *> (
                    ((MNM_DMOND *) node_factory_multimodal->get_node (
                       _origin_node_ID))
                      ->m_origin);
                  _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                    ((MNM_DMDND *) node_factory_multimodal->get_node (
                       _dest_node_ID))
                      ->m_dest);
                  IAssert (
                    _origin->m_demand_bus.find (_dest)->second.find (_route_ID)
                    != _origin->m_demand_bus.find (_dest)->second.end ());
                  _demand = _origin->m_demand_bus.find (_dest)
                              ->second.find (_route_ID)
                              ->second;
                  _demand_tot = 0;
                  for (int k = 0; k < num_release_interval; ++k)
                    {
                      _demand_tot += floor (_demand[k] * _origin->m_flow_scalar)
                                     / _origin->m_flow_scalar;
                    }
                  _demand_tot = round (_demand_tot);
                }

              if (_path_table->find (_origin_node_ID) == _path_table->end ())
                {
                  _new_map_1 = new std::unordered_map<
                    TInt, std::unordered_map<TInt, MNM_BusPath *> *> ();
                  _path_table->insert (
                    std::pair<TInt,
                              std::unordered_map<
                                TInt, std::unordered_map<TInt, MNM_BusPath *> *>
                                *> (_origin_node_ID, _new_map_1));
                }
              if (_path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                  == _path_table->find (_origin_node_ID)->second->end ())
                {
                  _new_map_2 = new std::unordered_map<TInt, MNM_BusPath *> ();
                  _path_table->find (_origin_node_ID)
                    ->second->insert (
                      std::pair<TInt, std::unordered_map<TInt, MNM_BusPath *>
                                        *> (_dest_node_ID, _new_map_2));
                }
              if (_path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (_route_ID)
                  == _path_table->find (_origin_node_ID)
                       ->second->find (_dest_node_ID)
                       ->second->end ())
                {
                  _path = new MNM_BusPath (_route_ID);
                  // _path -> m_route_ID = _route_ID;
                  _path->m_path_ID = _path_ID_counter;
                  _path_ID_counter += 1;

                  for (std::size_t j = 3; j < _path_words.size (); ++j)
                    {
                      // read node sequence
                      _node_ID = TInt (std::stoi (_path_words[j]));
                      _path->m_node_vec.push_back (_node_ID);
                    }

                  for (std::size_t j = 3; j < _route_words.size (); ++j)
                    {
                      // read busstop sequence
                      _busstop_ID = TInt (std::stoi (_route_words[j]));
                      _path->m_busstop_vec.push_back (_busstop_ID);

                      if (num_release_interval > 0)
                        {
                          dynamic_cast<MNM_Busstop_Virtual *> (
                            busstop_factory->get_busstop (_busstop_ID))
                            ->m_origin
                            = _origin;
                          dynamic_cast<MNM_Busstop_Virtual *> (
                            busstop_factory->get_busstop (_busstop_ID))
                            ->m_dest
                            = _dest;
                          dynamic_cast<MNM_Busstop_Virtual *> (
                            busstop_factory->get_busstop (_busstop_ID))
                            ->m_total_bus
                            = _demand_tot;
                        }
                    }

                  for (size_t j = 0; j < _path->m_node_vec.size () - 1; ++j)
                    {
                      _from_ID = _path->m_node_vec[j];
                      _to_ID = _path->m_node_vec[j + 1];
                      _link_ID = graph->GetEI (_from_ID, _to_ID)
                                   .GetId (); // for driving graph, at most one
                                              // edge exists between two nodes
                      _path->m_link_vec.push_back (_link_ID);
                    }

                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->insert (
                      std::pair<TInt, MNM_BusPath *> (_route_ID, _path));
                }
            }
        }
      _path_table_file.close ();
      _route_file.close ();
    }
  else
    {
      throw std::runtime_error ("Can't open bus path table file");
    }
  printf ("Finish Loading Bus Path Table!\n");
  // printf("path table %p\n", _path_table);
  // printf("path table %s\n", _path_table -> find(100283) -> second ->
  // find(150153) -> second
  //                           -> m_path_vec.front() -> node_vec_to_string());
  return _path_table;
}

PnR_Path_Table *
MNM_IO_Multimodal::load_pnr_path_table (const PNEGraph &driving_graph,
                                        const PNEGraph &transit_graph,
                                        TInt num_path,
                                        const std::string &file_name,
                                        bool w_buffer, bool w_ID)
{
  if (w_ID)
    {
      throw std::runtime_error (
        "Error, MNM_IO_Multimodal::load_pnr_path_table, with ID loading not "
        "implemented");
    }

  printf ("Loading Path Table for PnR!\n");
  TInt Num_Path = num_path;
  printf ("Number of path %d\n", Num_Path ());

  if (Num_Path <= 0)
    {
      printf ("Finish Loading Path Table for PnR, which is nullptr!\n");
      return nullptr;
    }

  std::ifstream _path_table_file, _buffer_file;
  std::string _buffer_file_name;
  if (w_buffer)
    {
      _buffer_file_name = file_name + "_buffer";
      _buffer_file.open (_buffer_file_name, std::ios::in);
    }
  _path_table_file.open (file_name, std::ios::in);
  auto *_path_table = new PnR_Path_Table ();

  /* read file */
  std::string _line, _buffer_line;
  std::vector<std::string> _words, _buffer_words;
  TInt _origin_node_ID, _dest_node_ID, _node_ID, _mid_parkinglot_ID,
    _mid_dest_node_ID;
  std::unordered_map<TInt, MNM_PnR_Pathset *> *_new_map;
  MNM_PnR_Pathset *_pathset;
  MNM_PnR_Path *_path;
  MNM_Path *_driving_path;
  MNM_Path *_transit_path;
  TInt _from_ID, _to_ID, _link_ID;
  size_t _transit_ind = -1;

  TInt _path_ID_counter = 0;
  if (_path_table_file.is_open ())
    {
      std::getline (_path_table_file, _line); // skip the first line
      for (int i = 0; i < Num_Path; ++i)
        {
          std::getline (_path_table_file, _line);
          if (w_buffer)
            {
              std::getline (_buffer_file, _buffer_line);
              _buffer_words = split (trim (_buffer_line), ' ');
            }
          // std::cout << "Processing: " << _line << "\n";
          _words = split (trim (_line), ' ');
          if (_words.size () >= 7)
            {
              _origin_node_ID = TInt (std::stoi (_words[0]));
              _dest_node_ID = TInt (std::stoi (_words[1]));
              _mid_parkinglot_ID = TInt (std::stoi (_words[2]));
              _mid_dest_node_ID = TInt (std::stoi (_words[3]));

              if (_path_table->find (_origin_node_ID) == _path_table->end ())
                {
                  _new_map = new std::unordered_map<TInt, MNM_PnR_Pathset *> ();
                  _path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_PnR_Pathset *>
                                      *> (_origin_node_ID, _new_map));
                }
              if (_path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                  == _path_table->find (_origin_node_ID)->second->end ())
                {
                  _pathset = new MNM_PnR_Pathset ();
                  _path_table->find (_origin_node_ID)
                    ->second->insert (
                      std::pair<TInt, MNM_PnR_Pathset *> (_dest_node_ID,
                                                          _pathset));
                }

              // driving part
              _driving_path = new MNM_Path ();
              for (size_t j = 4; j < _words.size (); ++j)
                {
                  _node_ID = TInt (std::stoi (_words[j]));
                  _driving_path->m_node_vec.push_back (_node_ID);
                  if (_node_ID == _mid_dest_node_ID)
                    {
                      _transit_ind = j + 1;
                      break;
                    }
                }
              IAssert (_transit_ind > 4 && _transit_ind < _words.size ());
              for (size_t j = 0; j < _driving_path->m_node_vec.size () - 1; ++j)
                {
                  _from_ID = _driving_path->m_node_vec[j];
                  _to_ID = _driving_path->m_node_vec[j + 1];
                  _link_ID
                    = driving_graph->GetEI (_from_ID, _to_ID)
                        .GetId (); // assume driving_graph is not a MultiGraph
                  _driving_path->m_link_vec.push_back (_link_ID);
                }
              IAssert (_driving_path->m_node_vec.front () == _origin_node_ID);
              IAssert (_driving_path->m_node_vec.back () == _mid_dest_node_ID);

              // transit part
              _transit_path = new MNM_Path ();
              for (size_t j = _transit_ind; j < _words.size (); ++j)
                {
                  _link_ID = TInt (
                    std::stoi (_words[j])); // read link ID first since
                                            // transit_graph is a MultiGraph
                  _transit_path->m_link_vec.push_back (_link_ID);
                }
              for (size_t j = 0; j < _transit_path->m_link_vec.size (); ++j)
                {
                  _link_ID = _transit_path->m_link_vec[j];
                  _from_ID = transit_graph->GetEI (_link_ID).GetSrcNId ();
                  _to_ID = transit_graph->GetEI (_link_ID).GetDstNId ();
                  _transit_path->m_node_vec.push_back (_from_ID);
                  if (j == _transit_path->m_link_vec.size () - 1)
                    _transit_path->m_node_vec.push_back (_to_ID);
                }
              IAssert (_transit_path->m_node_vec.size ()
                       == _transit_path->m_link_vec.size () + 1);
              IAssert (_transit_path->m_node_vec.front () == _mid_dest_node_ID);
              IAssert (_transit_path->m_node_vec.back () == _dest_node_ID);

              // PnR
              _path = new MNM_PnR_Path (_path_ID_counter, _mid_parkinglot_ID,
                                        _mid_dest_node_ID, _driving_path,
                                        _transit_path);
              _path_ID_counter += 1;

              if (w_buffer && (_buffer_words.size () > 0))
                {
                  TInt _buffer_len = TInt (_buffer_words.size ());
                  // printf("Buffer len %d\n", _buffer_len());
                  _path->allocate_buffer (_buffer_len);
                  for (int j = 0; j < _buffer_len (); ++j)
                    {
                      _path->m_buffer[j]
                        = TFlt (std::stof (trim (_buffer_words[j])));
                    }
                }

              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->m_path_vec.push_back (_path);
            }
        }
      _path_table_file.close ();
      if (w_buffer)
        {
          _buffer_file.close ();
        }
    }
  else
    {
      throw std::runtime_error ("Can't open pnr path table file");
    }
  printf ("Finish Loading Path Table for PnR!\n");
  return _path_table;
}

Path_Table *
MNM_IO_Multimodal::load_bustransit_path_table (const PNEGraph &transit_graph,
                                               TInt num_path,
                                               const std::string &file_name,
                                               bool w_buffer, bool w_ID)
{
  if (w_ID)
    {
      throw std::runtime_error (
        "Error, MNM_IO_Multimodal::load_bustransit_path_table, with ID loading "
        "not implemented");
    }

  printf ("Loading Path Table for Bus Transit!\n");
  TInt Num_Path = num_path;
  printf ("Number of path %d\n", Num_Path ());

  if (Num_Path <= 0)
    {
      printf ("Finish Loading Path Table for Bus Transit, which is nullptr!\n");
      return nullptr;
    }

  std::ifstream _path_table_file, _buffer_file;
  std::string _buffer_file_name;
  if (w_buffer)
    {
      _buffer_file_name = file_name + "_buffer";
      _buffer_file.open (_buffer_file_name, std::ios::in);
    }
  _path_table_file.open (file_name, std::ios::in);
  auto *_path_table = new Path_Table ();

  /* read file */
  std::string _line, _buffer_line;
  std::vector<std::string> _words, _buffer_words;
  TInt _origin_node_ID, _dest_node_ID, _busstop_ID;
  std::unordered_map<TInt, MNM_Pathset *> *_new_map;
  MNM_Pathset *_pathset;
  MNM_Path *_path;
  TInt _from_ID, _to_ID, _link_ID;

  TInt _path_ID_counter = 0;
  if (_path_table_file.is_open ())
    {
      std::getline (_path_table_file, _line); // skip the first line
      for (int i = 0; i < Num_Path; ++i)
        {
          std::getline (_path_table_file, _line);
          if (w_buffer)
            {
              std::getline (_buffer_file, _buffer_line);
              _buffer_words = split (trim (_buffer_line), ' ');
            }
          // std::cout << "Processing: " << _line << "\n";
          _words = split (trim (_line), ' ');
          if (_words.size () >= 3)
            {
              _origin_node_ID = TInt (std::stoi (_words[0]));
              _dest_node_ID = TInt (std::stoi (_words[1]));

              if (_path_table->find (_origin_node_ID) == _path_table->end ())
                {
                  _new_map = new std::unordered_map<TInt, MNM_Pathset *> ();
                  _path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                                      *> (_origin_node_ID, _new_map));
                }
              if (_path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                  == _path_table->find (_origin_node_ID)->second->end ())
                {
                  _pathset = new MNM_Pathset ();
                  _path_table->find (_origin_node_ID)
                    ->second->insert (
                      std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
                }

              _path = new MNM_Path ();
              for (size_t j = 2; j < _words.size (); ++j)
                {
                  _link_ID = TInt (
                    std::stoi (_words[j])); // read link ID first since
                                            // transit_graph is a MultiGraph
                  _path->m_link_vec.push_back (_link_ID);
                }
              for (size_t j = 0; j < _path->m_link_vec.size (); ++j)
                {
                  _link_ID = _path->m_link_vec[j];
                  _from_ID = transit_graph->GetEI (_link_ID).GetSrcNId ();
                  _to_ID = transit_graph->GetEI (_link_ID).GetDstNId ();
                  _path->m_node_vec.push_back (_from_ID);
                  if (j == _path->m_link_vec.size () - 1)
                    _path->m_node_vec.push_back (_to_ID);
                }
              IAssert (_path->m_node_vec.size ()
                       == _path->m_link_vec.size () + 1);
              IAssert (_path->m_node_vec.front () == _origin_node_ID);
              IAssert (_path->m_node_vec.back () == _dest_node_ID);

              _path->m_path_ID = _path_ID_counter;
              _path_ID_counter += 1;

              if (w_buffer && (_buffer_words.size () > 0))
                {
                  TInt _buffer_len = TInt (_buffer_words.size ());
                  // printf("Buffer len %d\n", _buffer_len());
                  _path->allocate_buffer (_buffer_len);
                  for (int j = 0; j < _buffer_len (); ++j)
                    {
                      _path->m_buffer[j]
                        = TFlt (std::stof (trim (_buffer_words[j])));
                    }
                }

              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->m_path_vec.push_back (_path);
            }
        }
      _path_table_file.close ();
      if (w_buffer)
        {
          _buffer_file.close ();
        }
    }
  else
    {
      throw std::runtime_error ("Can't open bus transit path table file");
    }
  printf ("Finish Loading Path Table for Bus Transit!\n");
  return _path_table;
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Multimodal DTA
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Dta_Multimodal::MNM_Dta_Multimodal (const std::string &file_folder)
    : MNM_Dta_Multiclass::MNM_Dta_Multiclass (file_folder)
{
  m_passenger_factory = nullptr;
  m_busstop_factory = nullptr;
  m_parkinglot_factory = nullptr;
  m_transitlink_factory = nullptr;

  m_enroute_passenger_num = std::deque<TInt> ();
  m_queue_passenger_num = std::deque<TInt> ();
  m_queue_passenger_map = std::unordered_map<TInt, std::deque<TInt> *> ();

  initialize ();
}

MNM_Dta_Multimodal::~MNM_Dta_Multimodal ()
{
  delete m_passenger_factory;
  delete m_busstop_factory;
  delete m_parkinglot_factory;
  delete m_transitlink_factory;
  m_bus_transit_graph->Clr ();

  m_enroute_passenger_num.clear ();
  m_queue_passenger_num.clear ();
  for (auto _it = m_queue_passenger_map.begin ();
       _it != m_queue_passenger_map.end (); _it++)
    {
      _it->second->clear ();
      delete _it->second;
    }
  m_queue_passenger_map.clear ();
}

int
MNM_Dta_Multimodal::initialize ()
{
  if (m_veh_factory != nullptr)
    delete m_veh_factory;
  if (m_node_factory != nullptr)
    delete m_node_factory;
  if (m_link_factory != nullptr)
    delete m_link_factory;
  if (m_od_factory != nullptr)
    delete m_od_factory;
  if (m_config != nullptr)
    delete m_config;

  m_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DTA");
  m_unit_time = m_config->get_int ("unit_time");
  m_flow_scalar = m_config->get_int ("flow_scalar");

  m_passenger_factory = new MNM_Passenger_Factory ();
  m_veh_factory
    = new MNM_Veh_Factory_Multimodal (TInt (m_config->get_int ("bus_capacity")),
                                      TInt (round (
                                        m_config->get_float ("fixed_dwell_time")
                                        / float (m_unit_time))),
                                      TInt (round (m_config->get_float (
                                                     "boarding_lost_time")
                                                   / float (m_unit_time))),
                                      TInt (round (
                                        float (m_unit_time)
                                        / m_config->get_int (
                                          "alighting_time_per_passenger"))),
                                      TInt (round (
                                        float (m_unit_time)
                                        / m_config->get_int (
                                          "boarding_time_per_passenger"))));
  m_node_factory = new MNM_Node_Factory_Multimodal ();
  m_link_factory = new MNM_Link_Factory_Multimodal ();
  m_od_factory = new MNM_OD_Factory_Multimodal ();
  m_busstop_factory = new MNM_Busstop_Factory ();
  m_parkinglot_factory = new MNM_Parking_Lot_Factory ();
  m_transitlink_factory = new MNM_Transit_Link_Factory ();

  // printf("5\n");
  TInt _ev_label_car, _ev_label_truck;
  try
    {
      _ev_label_car = m_config->get_int ("ev_label_car");
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "ev_label_car does not exist in config.conf/DTA, use "
                   "default value -2 instead\n";
      _ev_label_car = -2;
    }
  try
    {
      _ev_label_truck = m_config->get_int ("ev_label_truck");
    }
  catch (const std::invalid_argument &ia)
    {
      std::cout << "ev_label_truck does not exist in config.conf/DTA, use "
                   "default value -2 instead\n";
      _ev_label_truck = -2;
    }
  m_emission
    = new MNM_Cumulative_Emission_Multiclass (TFlt (m_unit_time), 0,
                                              _ev_label_car, _ev_label_truck);

  // FIXME: The following lines may introduce issues related to rounding. So we
  // need to keep 60 dividable by unit_time and m_config->get_int ("assign_frq")
  // dividable by m_assign_freq.
  //
  // the releasing strategy is assigning vehicles per 1 minute, so disaggregate
  // 15-min demand into 1-min demand change assign_freq to 12 (1 minute = 12 x 5
  // second / 60) and total_assign_interval to max_interval*_num_of_minute
  m_assign_freq
    = 60 / int (m_unit_time); // # of unit intervals in 1 min = # of assign freq
  TInt _num_of_minute
    = int (m_config->get_int ("assign_frq"))
      / m_assign_freq; // 15 min, # of minutes in original assign interval
  m_total_assign_inter = m_config->get_int ("max_interval")
                         * _num_of_minute; // how many 1-min intervals
  m_start_assign_interval = m_config->get_int ("start_assign_interval");

  m_explicit_bus = m_config->get_int ("explicit_bus");
  if (!m_explicit_bus)
    {
      IAssert (m_config->get_int ("historical_bus_waiting_time") > 0);
    }
  else
    {
      IAssert (m_config->get_int ("historical_bus_waiting_time") == 0);
    }
  return 0;
}

int
MNM_Dta_Multimodal::set_statistics ()
{
  MNM_ConfReader *_record_config
    = new MNM_ConfReader (m_file_folder + "/config.conf", "STAT");
  if (_record_config->get_string ("rec_mode") == "LRn")
    {
      m_statistics
        = new MNM_Statistics_Lrn_Multimodal (m_file_folder, m_config,
                                             _record_config, m_od_factory,
                                             m_node_factory, m_link_factory,
                                             m_transitlink_factory);
    }
  // printf("set_statistics finished\n");
  return 0;
}

int
MNM_Dta_Multimodal::set_routing ()
{
  // For Bi-class with bus routing
  // Note here still only one path_table and buffer, but in buffer file each row
  // contains: Row #k : [probabilities choosing route k in all intervals for
  // cars] [probabilities choosing route k in all intervals for trucks] For
  // Bi-class Fixed routing, just set both adaptive_ratio_car=0 &
  // adaptive_ratio_truck=0 in "config.conf"
  if (m_config->get_string ("routing_type") == "Multimodal_Hybrid"
      || m_config->get_string ("routing_type") == "Multimodal_DUE_FixedPath"
      || m_config->get_string ("routing_type")
           == "Multimodal_DUE_ColumnGeneration"
      || m_config->get_string ("routing_type")
           == "Multimodal_Hybrid_ColumnGeneration")
    {
      auto *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      TInt _buffer_len = _tmp_conf->get_int ("buffer_length");
      // for bi-class problem
      if (_buffer_len < 2 * m_config->get_int ("max_interval"))
        {
          _buffer_len = 2 * m_config->get_int ("max_interval");
        }
      TInt _route_freq_fixed = _tmp_conf->get_int ("route_frq");

      Path_Table *_driving_path_table = nullptr;
      Bus_Path_Table *_bus_path_table = nullptr;
      PnR_Path_Table *_pnr_path_table = nullptr;
      Path_Table *_bustransit_path_table = nullptr;

      bool _with_buffer
        = (_tmp_conf->get_string ("choice_portion") == "Buffer");
      _driving_path_table
        = MNM_IO::load_path_table (m_file_folder + "/"
                                     + _tmp_conf->get_string (
                                       "driving_path_file_name"),
                                   m_graph,
                                   _tmp_conf->get_int ("num_driving_path"),
                                   _with_buffer);
      _pnr_path_table
        = MNM_IO_Multimodal::load_pnr_path_table (m_graph, m_bus_transit_graph,
                                                  _tmp_conf->get_int (
                                                    "num_pnr_path"),
                                                  m_file_folder + "/"
                                                    + _tmp_conf->get_string (
                                                      "pnr_path_file_name"),
                                                  _with_buffer);
      _bustransit_path_table = MNM_IO_Multimodal::
        load_bustransit_path_table (m_bus_transit_graph,
                                    _tmp_conf->get_int ("num_bustransit_path"),
                                    m_file_folder + "/"
                                      + _tmp_conf->get_string (
                                        "bustransit_path_file_name"),
                                    _with_buffer);

      _bus_path_table
        = MNM_IO_Multimodal::load_bus_path_table (m_graph,
                                                  _tmp_conf->get_int (
                                                    "num_bus_routes"),
                                                  m_node_factory,
                                                  m_busstop_factory,
                                                  m_total_assign_inter,
                                                  m_file_folder + "/"
                                                    + _tmp_conf->get_string (
                                                      "bus_path_file_name"),
                                                  m_file_folder + "/"
                                                    + _tmp_conf->get_string (
                                                      "bus_route_file_name"));

      m_routing
        = new MNM_Routing_Multimodal_Hybrid (m_file_folder, m_graph,
                                             m_bus_transit_graph, m_statistics,
                                             m_od_factory, m_node_factory,
                                             m_link_factory, m_busstop_factory,
                                             m_parkinglot_factory,
                                             m_transitlink_factory,
                                             _bus_path_table, _pnr_path_table,
                                             _bustransit_path_table,
                                             _route_freq_fixed, _buffer_len);
      // init_routing() let car and truck share the same path table if
      // _driving_path_table != nullptr, otherwise they are nullptr
      // independently
      m_routing->init_routing (_driving_path_table);

      delete _tmp_conf;
    }
  else
    {
      throw std::runtime_error (
        "Wrong routing type for MNM_Routing_Multimodal_Hybrid");
    }

  // used in MNM_Busstop_Virtual::receive_bus()
  for (auto _busstop_it : m_busstop_factory->m_busstop_map)
    {
      _busstop_it.second->m_routing
        = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_routing);
    }
  return 0;
}

int
MNM_Dta_Multimodal::build_from_files ()
{
  MNM_IO_Multimodal::build_node_factory_multimodal (m_file_folder, m_config,
                                                    m_node_factory,
                                                    "driving_node");
  MNM_IO_Multimodal::build_link_factory_multimodal (m_file_folder, m_config,
                                                    m_link_factory,
                                                    "driving_link");
  MNM_IO_Multimodal::build_od_factory_multimodal (m_file_folder, m_config,
                                                  m_od_factory, m_node_factory,
                                                  "od");
  MNM_IO_Multimodal::build_busstop_factory (m_file_folder, m_config,
                                            m_busstop_factory, m_link_factory,
                                            "bus_stop");
  MNM_IO_Multimodal::build_parkinglot_factory (m_file_folder, m_config,
                                               m_parkinglot_factory,
                                               m_node_factory, m_link_factory,
                                               m_passenger_factory,
                                               m_veh_factory, "parking_lot");
  MNM_IO_Multimodal::build_walkinglink_factory (m_file_folder, m_config,
                                                m_transitlink_factory,
                                                m_node_factory,
                                                m_busstop_factory,
                                                m_parkinglot_factory,
                                                "walking_link");
  MNM_IO_Multimodal::build_buslink_factory (m_file_folder, m_config,
                                            m_transitlink_factory,
                                            m_busstop_factory, m_link_factory,
                                            "bus_link");

  m_graph = MNM_IO_Multimodal::build_graph (m_file_folder, m_config);
  m_bus_transit_graph
    = MNM_IO_Multimodal::build_bus_transit_graph (m_config,
                                                  m_transitlink_factory);

  MNM_IO_Multimodal::build_vehicle_demand_multimodal (m_file_folder, m_config,
                                                      m_od_factory,
                                                      "driving_demand",
                                                      "bus_demand",
                                                      m_explicit_bus);
  if (m_config->get_string ("routing_type") == "Multimodal_Hybrid")
    {
      MNM_IO_Multimodal::build_pnr_demand (m_file_folder, m_config,
                                           m_od_factory, "pnr_demand");
      MNM_IO_Multimodal::build_bustransit_demand (m_file_folder, m_config,
                                                  m_od_factory,
                                                  "bustransit_demand");
    }
  MNM_IO_Multimodal::read_origin_vehicle_label_ratio (m_file_folder, m_config,
                                                      m_od_factory,
                                                      "Origin_vehicle_label");
  MNM_IO_Multiclass::build_link_toll_multiclass (m_file_folder, m_config,
                                                 m_link_factory);
  // build_workzone();
  m_workzone = nullptr;
  // record link volume and travel time in simulation
  set_statistics ();
  set_gridlock_recorder ();
  // load path table and buffer
  set_routing ();
  return 0;
}

int
MNM_Dta_Multimodal::find_connected_pnr_parkinglot_for_destination ()
{
  MNM_Destination_Multimodal *_dest;
  MNM_Parking_Lot *_parkinglot;
  TInt _origin_node_ID, _dest_node_ID, _mid_dest_node_ID;

  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  // assume cost is 1 for every link to check connectivity
  for (auto _map_it : m_transitlink_factory->m_transit_link_map)
    {
      _cost_map.insert (std::pair<TInt, TFlt> (_map_it.first, TFlt (1)));
    }

  for (auto _it : m_od_factory->m_destination_map)
    {
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_it.second);
      _dest_node_ID = _dest->m_dest_node->m_node_ID;
      if (!m_bus_transit_graph->IsNode (_dest_node_ID))
        continue;
      _shortest_path_tree.clear ();
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_bus_transit_graph,
                                          _cost_map, _shortest_path_tree);

      // parking_lot-D
      for (auto _map_it : m_parkinglot_factory->m_parking_lot_map)
        {
          _parkinglot = _map_it.second;
          _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
          printf ("connected ");
          if (_dest_node_ID != _mid_dest_node_ID
              && m_bus_transit_graph->IsNode (_mid_dest_node_ID)
              && _shortest_path_tree.find (_mid_dest_node_ID)->second > 0)
            {
              // connected
              _dest->m_connected_pnr_parkinglot_vec.push_back (_parkinglot);
            }
        }
    }
  _shortest_path_tree.clear ();
  _cost_map.clear ();
  return 0;
}

bool
MNM_Dta_Multimodal::check_bus_transit_connectivity ()
{
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  TInt _origin_node_ID, _dest_node_ID, _mid_dest_node_ID;

  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  // assume cost is 1 for every link to check connectivity
  for (auto _map_it : m_transitlink_factory->m_transit_link_map)
    {
      _cost_map.insert (std::pair<TInt, TFlt> (_map_it.first, TFlt (1)));
    }

  for (auto _it : m_od_factory->m_destination_map)
    {
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_it.second);
      _dest_node_ID = _dest->m_dest_node->m_node_ID;
      if (!m_bus_transit_graph->IsNode (_dest_node_ID))
        continue;
      _shortest_path_tree.clear ();
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_bus_transit_graph,
                                          _cost_map, _shortest_path_tree);

      // O-D
      for (auto _map_it : m_od_factory->m_origin_map)
        {
          _origin = dynamic_cast<MNM_Origin_Multimodal *> (_map_it.second);
          _origin_node_ID = _origin->m_origin_node->m_node_ID;

          if (_origin->m_demand_passenger_bus.find (_dest)
              != _origin->m_demand_passenger_bus.end ())
            {
              if (!m_bus_transit_graph->IsNode (_origin_node_ID)
                  || !m_bus_transit_graph->IsNode (_dest_node_ID)
                  || _shortest_path_tree.find (_origin_node_ID)->second == -1)
                {
                  printf ("Direct bus transit for origin node %d and "
                          "destination node %d is not possible!\n",
                          (int) _origin_node_ID, (int) _dest_node_ID);
                  _shortest_path_tree.clear ();
                  _cost_map.clear ();
                  return false;
                }
            }
          if (_origin->m_demand_pnr_car.find (_dest)
              != _origin->m_demand_pnr_car.end ())
            {
              // the connectivity of driving part has been checked in
              // dta::is_ok()
              if (!m_bus_transit_graph->IsNode (_dest_node_ID)
                  || _dest->m_connected_pnr_parkinglot_vec.empty ())
                {
                  printf ("PnR mode for origin node %d and destination node %d "
                          "is not possible!\n",
                          (int) _origin_node_ID, (int) _dest_node_ID);
                  _shortest_path_tree.clear ();
                  _cost_map.clear ();
                  return false;
                }
            }
        }
    }
  _shortest_path_tree.clear ();
  _cost_map.clear ();
  return true;
}

bool
MNM_Dta_Multimodal::check_bus_path_table ()
{
  auto *_routing = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_routing);
  if (_routing->m_routing_bus_fixed->m_bus_path_table == nullptr)
    {
      return true;
    }
  Bus_Path_Table *_bus_path_table
    = _routing->m_routing_bus_fixed->m_bus_path_table;
  std::deque<TInt> _reconstructed_driving_link_seq = std::deque<TInt> ();
  bool _consistent = true;
  for (auto _o_it : *_bus_path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto _r_it : *_d_it.second)
            {
              MNM_BusPath *_bus_path = _r_it.second;

              if (!_reconstructed_driving_link_seq.empty ())
                _reconstructed_driving_link_seq.clear ();
              for (size_t i = 0; i < _bus_path->m_busstop_vec.size () - 1; ++i)
                {
                  TInt _from_virtual_busstop_ID = _bus_path->m_busstop_vec[i];
                  TInt _to_virtual_busstop_ID = _bus_path->m_busstop_vec[i + 1];
                  // get bus link
                  MNM_Bus_Link *_bus_link
                    = dynamic_cast<MNM_Busstop_Virtual *> (
                        m_busstop_factory->get_busstop (
                          _from_virtual_busstop_ID))
                        ->m_bus_out_link;
                  IAssert (_bus_link != nullptr);
                  IAssert (_bus_link->m_to_node_ID == _to_virtual_busstop_ID);
                  TInt _transitlink_ID = _bus_link->m_link_ID;
                  // no repeated overlapped links
                  for (MNM_Dlink *_drivinglink :
                       _bus_link->m_overlapped_driving_link_vec)
                    {
                      // for (MNM_Dlink *_drivinglink :
                      // std::set<MNM_Dlink*>(_bus_link ->
                      // m_overlapped_driving_link_vec.begin(), _bus_link ->
                      // m_overlapped_driving_link_vec.end())) {
                      if (dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (
                            _drivinglink)
                          != nullptr)
                        {
                          for (auto *_busstop :
                               dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (
                                 _drivinglink)
                                 ->m_busstop_vec)
                            {
                              if (_busstop->m_route_ID == _r_it.first)
                                {
                                  if (_busstop->m_link_ID
                                        == _bus_link->m_from_busstop->m_link_ID
                                      && _busstop->m_link_ID
                                           == _bus_link->m_to_busstop
                                                ->m_link_ID)
                                    {
                                      if (_busstop->m_cell_ID
                                            >= _bus_link->m_from_busstop
                                                 ->m_cell_ID
                                          && _busstop->m_busstop_ID
                                               != _from_virtual_busstop_ID
                                          && _busstop->m_cell_ID
                                               <= _bus_link->m_to_busstop
                                                    ->m_cell_ID
                                          && _busstop->m_busstop_ID
                                               != _to_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else if (_busstop->m_link_ID
                                             == _bus_link->m_from_busstop
                                                  ->m_link_ID
                                           && _busstop->m_link_ID
                                                != _bus_link->m_to_busstop
                                                     ->m_link_ID)
                                    {
                                      if (_busstop->m_cell_ID
                                            >= _bus_link->m_from_busstop
                                                 ->m_cell_ID
                                          && _busstop->m_busstop_ID
                                               != _from_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else if (_busstop->m_link_ID
                                             != _bus_link->m_from_busstop
                                                  ->m_link_ID
                                           && _busstop->m_link_ID
                                                == _bus_link->m_to_busstop
                                                     ->m_link_ID)
                                    {
                                      if (_busstop->m_cell_ID
                                            <= _bus_link->m_to_busstop
                                                 ->m_cell_ID
                                          && _busstop->m_busstop_ID
                                               != _to_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else
                                    {
                                      IAssert (_busstop->m_link_ID
                                                 != _bus_link->m_from_busstop
                                                      ->m_link_ID
                                               && _busstop->m_link_ID
                                                    != _bus_link->m_to_busstop
                                                         ->m_link_ID);
                                      _consistent = false;
                                      printf ("Bus Link %d, from virtual bus "
                                              "stop %d, to virtual bus stop "
                                              "%d, another virtual bus stop %d "
                                              "exists on overlapped driving "
                                              "link %d\n",
                                              _transitlink_ID (),
                                              _from_virtual_busstop_ID (),
                                              _to_virtual_busstop_ID (),
                                              _busstop->m_busstop_ID (),
                                              _drivinglink->m_link_ID ());
                                    }
                                }
                            }
                        }
                      else if (dynamic_cast<MNM_Dlink_Pq_Multimodal *> (
                                 _drivinglink)
                               != nullptr)
                        {
                          for (auto _busstop_it :
                               dynamic_cast<MNM_Dlink_Pq_Multimodal *> (
                                 _drivinglink)
                                 ->m_busstop_timeloc_map)
                            {
                              MNM_Busstop_Virtual *_busstop = _busstop_it.first;
                              if (_busstop->m_route_ID == _r_it.first)
                                {
                                  if (_busstop->m_link_ID
                                        == _bus_link->m_from_busstop->m_link_ID
                                      && _busstop->m_link_ID
                                           == _bus_link->m_to_busstop
                                                ->m_link_ID)
                                    {
                                      if (_busstop_it.second
                                            >= dynamic_cast<
                                                 MNM_Dlink_Pq_Multimodal *> (
                                                 _drivinglink)
                                                 ->m_busstop_timeloc_map
                                                 .find (
                                                   _bus_link->m_from_busstop)
                                                 ->second
                                          && _busstop->m_busstop_ID
                                               != _from_virtual_busstop_ID
                                          && _busstop_it.second
                                               <= dynamic_cast<
                                                    MNM_Dlink_Pq_Multimodal *> (
                                                    _drivinglink)
                                                    ->m_busstop_timeloc_map
                                                    .find (
                                                      _bus_link->m_to_busstop)
                                                    ->second
                                          && _busstop->m_busstop_ID
                                               != _to_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else if (_busstop->m_link_ID
                                             == _bus_link->m_from_busstop
                                                  ->m_link_ID
                                           && _busstop->m_link_ID
                                                != _bus_link->m_to_busstop
                                                     ->m_link_ID)
                                    {
                                      if (_busstop_it.second
                                            >= dynamic_cast<
                                                 MNM_Dlink_Pq_Multimodal *> (
                                                 _drivinglink)
                                                 ->m_busstop_timeloc_map
                                                 .find (
                                                   _bus_link->m_from_busstop)
                                                 ->second
                                          && _busstop->m_busstop_ID
                                               != _from_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else if (_busstop->m_link_ID
                                             != _bus_link->m_from_busstop
                                                  ->m_link_ID
                                           && _busstop->m_link_ID
                                                == _bus_link->m_to_busstop
                                                     ->m_link_ID)
                                    {
                                      if (_busstop_it.second
                                            <= dynamic_cast<
                                                 MNM_Dlink_Pq_Multimodal *> (
                                                 _drivinglink)
                                                 ->m_busstop_timeloc_map
                                                 .find (_bus_link->m_to_busstop)
                                                 ->second
                                          && _busstop->m_busstop_ID
                                               != _to_virtual_busstop_ID)
                                        {
                                          _consistent = false;
                                          printf ("Bus Link %d, from virtual "
                                                  "bus stop %d, to virtual bus "
                                                  "stop %d, another virtual "
                                                  "bus stop %d exists on "
                                                  "overlapped driving link "
                                                  "%d\n",
                                                  _transitlink_ID (),
                                                  _from_virtual_busstop_ID (),
                                                  _to_virtual_busstop_ID (),
                                                  _busstop->m_busstop_ID (),
                                                  _drivinglink->m_link_ID ());
                                        }
                                    }
                                  else
                                    {
                                      IAssert (_busstop->m_link_ID
                                                 != _bus_link->m_from_busstop
                                                      ->m_link_ID
                                               && _busstop->m_link_ID
                                                    == _bus_link->m_to_busstop
                                                         ->m_link_ID);
                                      _consistent = false;
                                      printf ("Bus Link %d, from virtual bus "
                                              "stop %d, to virtual bus stop "
                                              "%d, another virtual bus stop %d "
                                              "exists on overlapped driving "
                                              "link %d\n",
                                              _transitlink_ID (),
                                              _from_virtual_busstop_ID (),
                                              _to_virtual_busstop_ID (),
                                              _busstop->m_busstop_ID (),
                                              _drivinglink->m_link_ID ());
                                    }
                                }
                            }
                        }
                      else
                        {
                          throw std::runtime_error ("Wrong link type");
                        }

                      if (_reconstructed_driving_link_seq.empty ()
                          || _drivinglink->m_link_ID
                               != _reconstructed_driving_link_seq.back ())
                        {
                          _reconstructed_driving_link_seq.push_back (
                            _drivinglink->m_link_ID);
                        }
                    }
                }

              if (_reconstructed_driving_link_seq.size ()
                  != _bus_path->m_link_vec.size ())
                {
                  _consistent = false;
                  printf ("\nbus path NOT consistent! OriginNodeID: %d, "
                          "DestinationNodeID: %d, RouteID: %d\n",
                          _o_it.first (), _d_it.first (), _r_it.first ());
                  printf ("input link seq:\n");
                  std::cout << _bus_path->link_vec_to_string ();
                  printf ("input virtual bus stop seq:\n");
                  std::cout << _bus_path->busstop_vec_to_string ();
                  printf ("reconstructed link seq from bus stop:\n");
                  _bus_path->m_link_vec = _reconstructed_driving_link_seq;
                  std::cout << _bus_path->link_vec_to_string ();
                }
              else
                {
                  for (size_t i = 0;
                       i < _reconstructed_driving_link_seq.size (); ++i)
                    {
                      if (_reconstructed_driving_link_seq[i]
                          != _bus_path->m_link_vec[i])
                        {
                          _consistent = false;
                          printf ("\nbus path NOT consistent! OriginNodeID: "
                                  "%d, DestinationNodeID: %d, RouteID: %d\n",
                                  _o_it.first (), _d_it.first (),
                                  _r_it.first ());
                          printf ("input link seq:\n");
                          std::cout << _bus_path->link_vec_to_string ();
                          printf ("input virtual bus stop seq:\n");
                          std::cout << _bus_path->busstop_vec_to_string ();
                          printf ("reconstructed link seq from bus stop:\n");
                          _bus_path->m_link_vec
                            = _reconstructed_driving_link_seq;
                          std::cout << _bus_path->link_vec_to_string ();
                          break;
                        }
                    }
                }
            }
        }
    }
  return _consistent;
}

bool
MNM_Dta_Multimodal::is_ok ()
{
  // Driving
  bool _flag = MNM_Dta::is_ok ();

  bool _temp_flag = true;
  // Checks the graph data structure for internal consistency.
  // For each node in the graph check that its neighbors are also nodes in the
  // graph.
  printf ("Checking......Bus Transit Graph consistent!\n");
  _temp_flag = m_bus_transit_graph->IsOk ();
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  // check node
  printf ("Checking......Bus Transit Node consistent!\n");
  _temp_flag = (m_bus_transit_graph->GetNodes ()
                >= m_config->get_int ("num_of_bus_stop_physical")
                     + m_config->get_int ("num_of_bus_stop_virtual"))
               && (m_bus_transit_graph->GetNodes ()
                   >= TInt (m_busstop_factory->m_busstop_map.size ()));
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  // check link
  printf ("Checking......Bus Transit Link consistent!\n");
  _temp_flag = (m_bus_transit_graph->GetEdges ()
                == m_config->get_int ("num_of_bus_link")
                     + m_config->get_int ("num_of_walking_link"))
               && (m_bus_transit_graph->GetEdges ()
                   == TInt (m_transitlink_factory->m_transit_link_map.size ()));
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  // check OD node
  TInt _node_ID;
  _temp_flag = (TInt (m_od_factory->m_origin_map.size ())
                == m_config->get_int ("num_of_O"))
               && (TInt (m_od_factory->m_destination_map.size ())
                   == m_config->get_int ("num_of_D"));

  printf ("Checking......Origin consistent!\n");
  MNM_Origin_Multimodal *_origin;
  for (auto _origin_map_it : m_od_factory->m_origin_map)
    {
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (_origin_map_it.second);
      _node_ID = _origin->m_origin_node->m_node_ID;
      if (!(_origin->m_demand_passenger_bus.empty ()))
        {
          _temp_flag
            = _temp_flag && (m_bus_transit_graph->IsNode (_node_ID))
              && ((m_bus_transit_graph->GetNI (_node_ID)).GetId () == _node_ID)
              && (m_bus_transit_graph->GetNI (_node_ID).GetOutDeg () >= 1)
              && (m_bus_transit_graph->GetNI (_node_ID).GetInDeg () == 0)
              && !(_origin->m_walking_out_links_vec.empty ());
        }
    }
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  printf ("Checking......Destination consistent!\n");
  MNM_Destination_Multimodal *_dest;
  for (auto _dest_map_it : m_od_factory->m_destination_map)
    {
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_dest_map_it.second);
      _node_ID = _dest->m_dest_node->m_node_ID;
      if (!(_dest->m_walking_in_links_vec.empty ())
          || (_dest->m_parking_lot != nullptr
              && !(_dest->m_parking_lot->m_walking_out_links_vec.empty ())))
        {
          _temp_flag
            = _temp_flag && (m_bus_transit_graph->IsNode (_node_ID))
              && ((m_bus_transit_graph->GetNI (_node_ID)).GetId () == _node_ID)
              && (m_bus_transit_graph->GetNI (_node_ID).GetOutDeg () >= 0)
              && (m_bus_transit_graph->GetNI (_node_ID).GetInDeg () >= 0);
        }
    }
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  printf ("Checking......Bus Stop consistent!\n");
  MNM_Busstop *_busstop;
  for (auto _busstop_map_it : m_busstop_factory->m_busstop_map)
    {
      _busstop = _busstop_map_it.second;
      _node_ID = _busstop->m_busstop_ID;
      _temp_flag
        = _temp_flag
          && ((m_bus_transit_graph->GetNI (_node_ID)).GetId () == _node_ID)
          && (m_bus_transit_graph->GetNI (_node_ID).GetOutDeg () >= 0)
          && (m_bus_transit_graph->GetNI (_node_ID).GetInDeg () >= 0)
          && (_busstop->m_link_ID != TInt (-1));
      if (dynamic_cast<MNM_Busstop_Physical *> (_busstop) != nullptr)
        {
          _temp_flag = _temp_flag
                       && (!dynamic_cast<MNM_Busstop_Physical *> (_busstop)
                              ->m_busstop_virtual_vec.empty ())
                       && (!dynamic_cast<MNM_Busstop_Physical *> (_busstop)
                              ->m_alighting_links_vec.empty ())
                       && (!dynamic_cast<MNM_Busstop_Physical *> (_busstop)
                              ->m_boarding_links_vec.empty ());
        }
      else if (dynamic_cast<MNM_Busstop_Virtual *> (_busstop) != nullptr)
        {
          _temp_flag
            = _temp_flag
              && (dynamic_cast<MNM_Busstop_Virtual *> (_busstop)
                    ->m_busstop_physical
                  != nullptr)
              && (dynamic_cast<MNM_Busstop_Virtual *> (_busstop)->m_bus_in_link
                    != nullptr
                  || dynamic_cast<MNM_Busstop_Virtual *> (_busstop)
                         ->m_bus_out_link
                       != nullptr);
        }
      else
        {
          _temp_flag = false;
        }
    }
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  printf ("Checking......Bus Transit connectivity!\n");
  _temp_flag = check_bus_transit_connectivity ();
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  printf ("Checking......Bus Path Table consistent!\n");
  _temp_flag = check_bus_path_table ();
  _flag = _flag && _temp_flag;
  if (_temp_flag)
    printf ("Passed!\n");

  return _flag;
}

int
MNM_Dta_Multimodal::record_queue_passengers ()
{
  TInt _tot_queue_size = 0;
  for (auto _map_it : m_transitlink_factory->m_transit_link_map)
    {
      if (auto *_walking_link
          = dynamic_cast<MNM_Walking_Link *> (_map_it.second))
        {
          TInt _queue_size = _walking_link->m_finished_array.size ();
          _tot_queue_size += _queue_size;
          m_queue_passenger_map[_walking_link->m_link_ID]->push_back (
            _queue_size);
        }
    }
  m_queue_passenger_num.push_back (_tot_queue_size);
  return 0;
}

int
MNM_Dta_Multimodal::record_enroute_passengers ()
{
  TInt _total_passenger = TInt (m_passenger_factory->m_passenger_map.size ());
  TInt _finished_passenger = 0;
  TInt _enroute_passenger;
  for (auto _map_it : m_passenger_factory->m_passenger_map)
    {
      if (_map_it.second->m_finish_time > 0)
        _finished_passenger += 1;
    }
  _enroute_passenger = _total_passenger - _finished_passenger;
  m_enroute_passenger_num.push_back (_enroute_passenger);
  return 0;
}

int
MNM_Dta_Multimodal::pre_loading ()
{
  MNM_Dta_Multiclass::pre_loading ();
  std::deque<TInt> *_rec;
  for (auto _map_it : m_transitlink_factory->m_transit_link_map)
    {
      if (auto *_walking_link
          = dynamic_cast<MNM_Walking_Link *> (_map_it.second))
        {
          _rec = new std::deque<TInt> ();
          m_queue_passenger_map.insert ({ _walking_link->m_link_ID, _rec });
        }
    }
  return 0;
}

int
MNM_Dta_Multimodal::load_once (bool verbose, TInt load_int, TInt assign_int)
{
  MNM_Origin *_origin;
  MNM_Origin_Multimodal *_origin_multimodal;
  MNM_Dnode *_node;
  MNM_Dlink *_link;
  MNM_Destination *_dest;
  MNM_Parking_Lot *_parkinglot;
  MNM_Busstop *_busstop;
  MNM_Walking_Link *_walkinglink;
  MNM_Bus_Link *_buslink;

  auto *_routing_multimodal_hybrid
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_routing);

  if (load_int == 0)
    m_statistics->update_record (-1);
  if (verbose)
    printf ("-------------------------------    Interval %d   "
            "------------------------------ \n",
            (int) load_int);
  if (verbose)
    printf ("Releasing from origins!\n");
  // step 1: origins releasing vehicles and passengers
  // for (auto _origin_it = m_od_factory -> m_origin_map.begin(); _origin_it !=
  // m_od_factory -> m_origin_map.end(); _origin_it++){
  //   _origin = _origin_it -> second;
  //   _origin -> release(m_veh_factory, _cur_int);
  // }

  // m_assign_freq = 12 intervals x 5 s = 1 min
  // m_total_assign_inter = total number of 1 min intervals
  // assign_int = the count of 1 min intervals, the vehicles and passengers
  // record this assign_int
  if (load_int % m_assign_freq == 0 || load_int == 0)
    {
      for (auto _origin_it : m_od_factory->m_origin_map)
        {
          _origin = _origin_it.second; // base origin class pointer to
                                       // multimodal origin object
          _origin_multimodal = dynamic_cast<MNM_Origin_Multimodal *> (_origin);
          if (assign_int >= m_total_assign_inter)
            {
              _origin->release_one_interval (load_int, m_veh_factory, -1,
                                             TFlt (-1));
              _origin_multimodal
                ->release_one_interval_passenger (load_int, m_passenger_factory,
                                                  -1, TFlt (-1));
            }
          else
            {
              if ((m_config->get_string ("routing_type") == "Multimodal_Hybrid")
                  || m_config->get_string ("routing_type")
                       == "Multimodal_Hybrid_ColumnGeneration"
                  || m_config->get_string ("routing_type")
                       == "Multimodal_DUE_FixedPath"
                  || m_config->get_string ("routing_type")
                       == "Multimodal_DUE_ColumnGeneration")
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
                  // NOTE: in this case the release function is different
                  _origin->release_one_interval_biclass (load_int,
                                                         m_veh_factory,
                                                         assign_int,
                                                         _ad_ratio_car,
                                                         _ad_ratio_truck);

                  TFlt _ad_ratio_passenger
                    = m_config->get_float ("adaptive_ratio_passenger");
                  if (_ad_ratio_passenger > 1)
                    _ad_ratio_passenger = 1;
                  if (_ad_ratio_passenger < 0)
                    _ad_ratio_passenger = 0;
                  _origin_multimodal
                    ->release_one_interval_passenger (load_int,
                                                      m_passenger_factory,
                                                      assign_int,
                                                      _ad_ratio_passenger);
                }
              // else if(m_config -> get_string("routing_type") ==
              // "Multimodal_DUE_FixedPath" ||
              //         m_config -> get_string("routing_type") ==
              //         "Multimodal_DUE_ColumnGeneration"){
              //     _origin -> release_one_interval_biclass(load_int,
              //     m_veh_factory, assign_int, 0., 0.); _origin_multimodal ->
              //     release_one_interval_passenger(load_int,
              //     m_passenger_factory, assign_int, 0.);
              // }
              else
                {
                  throw std::runtime_error ("WARNING:No assignment!");
                }
            }
        }
    }

  if (verbose)
    printf ("Routing vehicles and passengers in origins!\n");
  // step 2: routing the vehicles and passengers
  m_routing->update_routing (load_int);

  if (verbose)
    printf ("Moving vehicles through nodes!\n");
  // step 3: moving vehicles through node
  for (auto _node_it : m_node_factory->m_node_map)
    {
      _node = _node_it.second;
      // printf("node ID is %d\n", _node -> m_node_ID());
      _node->evolve (load_int);
    }
  // record queuing vehicles after node evolve, which is num of vehicles in
  // finished array
  record_queue_vehicles ();

  if (verbose)
    printf ("Moving passengers out of origins!\n");
  // step 4: moving passengers out of origins
  for (auto _origin_it : m_od_factory->m_origin_map)
    {
      _origin
        = _origin_it
            .second; // base origin class pointer to multimodal origin object
      _origin_multimodal = dynamic_cast<MNM_Origin_Multimodal *> (_origin);
      // printf("orgin ID is %d\n", _origin_multimodal -> m_ID());
      _origin_multimodal->evolve (load_int);
    }

  if (verbose)
    printf ("Moving passengers into destinations!\n");
  // step 5: moving passengers into destinations
  for (auto _dest_it : m_od_factory->m_destination_map)
    {
      _dest = _dest_it.second;
      dynamic_cast<MNM_Destination_Multimodal *> (_dest)->evolve (load_int);
    }

  if (verbose)
    printf ("Moving passengers through physical bus stops!\n");
  // step 6: moving passengers through bus stops
  for (auto _busstop_it : m_busstop_factory->m_busstop_map)
    {
      _busstop = _busstop_it.second;
      // printf("bus stop ID is %d\n", _busstop -> m_ID());
      _busstop->evolve (load_int);
    }

  if (verbose)
    printf ("Moving passengers through walking links!\n");
  // step 7: moving passengers through walking links
  for (auto _link_it : m_transitlink_factory->m_transit_link_map)
    {
      if (_link_it.second->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
        {
          // printf("walking link ID is %d\n", _link-> m_link_ID());
          _walkinglink = dynamic_cast<MNM_Walking_Link *> (_link_it.second);
          _walkinglink->clear_incoming_array (load_int);
          _walkinglink->evolve (load_int);

          // routing passengers on boarding links which have very short travel
          // time and may be skipped in the following board_and_alight()
          if (_walkinglink->m_walking_type == "boarding")
            {
              _routing_multimodal_hybrid->m_routing_passenger_fixed
                ->update_routing_one_link (load_int, _link_it.second);
              if (_routing_multimodal_hybrid->m_routing_multimodal_adaptive
                  != nullptr)
                {
                  _routing_multimodal_hybrid->m_routing_multimodal_adaptive
                    ->update_routing_passenger_one_link (load_int,
                                                         _link_it.second);
                }
            }
        }
    }

  if (verbose)
    printf ("Additional steps for implicit bus modeling!\n");
  if (!m_explicit_bus)
    {
      for (auto _busstop_it : m_busstop_factory->m_busstop_map)
        {
          _busstop = _busstop_it.second;
          if (auto *_busstop_virtual
              = dynamic_cast<MNM_Busstop_Virtual *> (_busstop))
            {
              _busstop_virtual->virtual_evolve (load_int);
            }
        }

      for (auto _link_it : m_transitlink_factory->m_transit_link_map)
        {
          if (_link_it.second->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
            {
              _buslink = dynamic_cast<MNM_Bus_Link *> (_link_it.second);
              _buslink->clear_incoming_array (load_int);
              _buslink->evolve (load_int);
            }
        }
    }

  // record queuing passengers, which is num of passengers in finished array
  record_queue_passengers ();

  if (verbose)
    printf ("Moving vehicles through driving link\n");
  // step 8: moving vehicles through driving links and boarding and alighting
  // passengers
  for (auto _link_it : m_link_factory->m_link_map)
    {
      _link = _link_it.second;

      if ((m_gridlock_recorder != nullptr)
          && ((m_config->get_int ("total_interval") <= 0
               && load_int >= 1.5 * m_total_assign_inter * m_assign_freq)
              || (m_config->get_int ("total_interval") > 0
                  && load_int >= 0.95 * m_config->get_int ("total_interval"))))
        {
          m_gridlock_recorder->save_one_link (load_int, _link);
        }

      _link->clear_incoming_array (load_int);
      _link->evolve (load_int); // include board_and_alight()
    }

  // only use in multiclass vehicle cases
  if (m_emission != nullptr)
    m_emission->update (m_veh_factory);

  if (verbose)
    printf ("Destinations receiving finished vehicles and passengers!\n");
  // step 9: destinations receiving finished vehicles and passengers
  for (auto _dest_it : m_od_factory->m_destination_map)
    {
      _dest = _dest_it.second;
      // _dest -> receive(load_int);
      dynamic_cast<MNM_Destination_Multimodal *> (_dest)
        ->receive (load_int, _routing_multimodal_hybrid, m_veh_factory,
                   m_passenger_factory,
                   true); // true means delete finished vehicles and passengers
    }

  if (verbose)
    printf ("Routing passengers in PnR mode!\n");
  // step 10: routing passengers in PnR mode
  _routing_multimodal_hybrid->m_routing_passenger_fixed
    ->update_routing_parkinglot (load_int);
  if (_routing_multimodal_hybrid->m_routing_multimodal_adaptive != nullptr)
    {
      _routing_multimodal_hybrid->m_routing_multimodal_adaptive
        ->update_routing_passenger_parkinglot (load_int);
    }

  if (verbose)
    printf ("Moving passengers out of parking lots!\n");
  // step 11: moving passengers out of parking lot
  for (auto _parkinglot_it : m_parkinglot_factory->m_parking_lot_map)
    {
      _parkinglot = _parkinglot_it.second;
      // printf("parking lot ID is %d\n", _parkinglot -> m_ID());
      _parkinglot->evolve (load_int);
    }

  if (verbose)
    printf ("Update record!\n");

  // step 12: update record
  m_statistics->update_record (load_int);

  record_enroute_vehicles ();
  record_enroute_passengers ();
  // if (verbose) {
  //     MNM::print_vehicle_statistics(dynamic_cast<MNM_Veh_Factory_Multimodal*>(m_veh_factory));
  //     MNM::print_passenger_statistics(m_passenger_factory);
  // }

  // test();
  m_current_loading_interval = load_int + 1;
  return 0;
}

int
MNM_Dta_Multimodal::loading (bool verbose)
{
  int _current_inter = 0;
  int _assign_inter = m_start_assign_interval;

  while (!finished_loading (_current_inter)
         || _assign_inter < m_total_assign_inter)
    {
      if (verbose)
        {
          std::cout << std::endl
                    << "Current loading interval: " << _current_inter << ", "
                    << "Current assignment interval: "
                    << int (_current_inter / m_config->get_int ("assign_frq"))
                    << std::endl;
        }
      load_once (verbose, _current_inter, _assign_inter);
      // link cc will be updated with the record at the end of this interval
      // (i.e., _current_inter + 1)
      if (++_current_inter % m_assign_freq == 0)
        {
          ++_assign_inter;
        }
    }
  if (verbose)
    {
      MNM::print_vehicle_statistics (
        dynamic_cast<MNM_Veh_Factory_Multimodal *> (m_veh_factory));
      MNM::print_passenger_statistics (m_passenger_factory);
    }
  m_statistics->post_record ();
  if (m_gridlock_recorder != nullptr)
    m_gridlock_recorder->post_record ();
  m_current_loading_interval = _current_inter;
  return _current_inter; // total number of actual loading intervals =
                         // _current_inter)
}

bool
MNM_Dta_Multimodal::finished_loading (int cur_int)
{
  // printf("Entering MNM_Dta::finished_loading\n");
  TInt _total_int = m_config->get_int ("total_interval");
  if (_total_int > 0)
    {
      // printf("Exiting MNM_Dta::finished_loading 1\n");
      return cur_int >= _total_int;
    }
  else
    {
      // printf("Exiting MNM_Dta::finished_loading 2\n");
      // probably too strong, since not all passengers are able to board a bus
      // in the analysis horizon return
      // !(MNM::has_running_vehicle(m_veh_factory) ||
      // MNM::has_running_passenger(m_passenger_factory) || cur_int == 0);
      // weaker, only check for vehicles
      return !(MNM::has_running_vehicle (m_veh_factory) || cur_int == 0);
    }
}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Multiclass DTA Gradient Utils
*******************************************************************************************************************
******************************************************************************************************************/
namespace MNM_DTA_GRADIENT
{
TFlt
get_bus_waiting_time (MNM_Busstop_Virtual *busstop, TFlt start_time,
                      TFlt unit_interval, TInt end_loading_timestamp,
                      bool return_inf)
{
  if ((int) busstop->m_N_in_bus->m_recorder.size () == 1
      || MNM_Ults::approximate_equal (busstop->m_N_in_bus->m_recorder.back ()
                                        .second,
                                      TFlt (0)))
    {
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  if (busstop->m_last_valid_time < 0)
    {
      busstop->m_last_valid_time
        = get_last_valid_time_bus (busstop->m_N_in_bus, busstop->m_N_out_bus,
                                   end_loading_timestamp);
    }
  if (MNM_Ults::approximate_equal (busstop->m_last_valid_time, TFlt (0)))
    {
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  int _time = int (round (start_time));
  if (_time + 1 > busstop->m_last_valid_time)
    {
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  TFlt _cc_in = busstop->m_N_in_bus->get_result (TFlt (_time));
  TFlt _cc_out = busstop->m_N_out_bus->get_result (TFlt (_time));

  int _cc_floor = int (_cc_in);
  int _cc_ceil = _cc_floor + 1;

  if (_cc_floor == 0 && !MNM_Ults::approximate_equal (_cc_in, TFlt (1)))
    {
      for (int i = 1; i <= int (busstop->m_last_valid_time) - _time; ++i)
        {
          TFlt _cc_in_2 = busstop->m_N_in_bus->get_result (TFlt (_time + i));
          if (!MNM_Ults::approximate_less_than (_cc_in_2, TFlt (1))
              && MNM_Ults::approximate_less_than (_cc_in, _cc_in_2))
            {
              return TFlt (i); // interval
            }
        }
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  if (MNM_Ults::approximate_less_than (_cc_out, _cc_in))
    {
      if (MNM_Ults::approximate_equal (_cc_in, TFlt (_cc_floor)))
        {
          return TFlt (0);
        }
      else if (MNM_Ults::approximate_equal (_cc_in, TFlt (_cc_ceil)))
        {
          return TFlt (0);
        }
      else
        {
          IAssert (MNM_Ults::approximate_less_than (TFlt (_cc_floor), _cc_in));
          TFlt _tmp = busstop->m_N_in_bus->get_time (TFlt (_cc_floor));
          IAssert (MNM_Ults::approximate_less_than (_tmp, start_time));
          TFlt _flow = busstop->m_N_in_bus->get_result (TFlt (_tmp));
          if (MNM_Ults::approximate_equal (_flow, TFlt (_cc_floor)))
            {
              for (int i = 1; i <= int (busstop->m_last_valid_time) - _time;
                   ++i)
                {
                  TFlt _cc_in_2
                    = busstop->m_N_in_bus->get_result (TFlt (_time + i));
                  if (!MNM_Ults::approximate_less_than (_cc_in_2,
                                                        TFlt (_cc_ceil))
                      && MNM_Ults::approximate_less_than (_cc_in, _cc_in_2))
                    {
                      return TFlt (i); // interval
                    }
                }
              return return_inf ? std::numeric_limits<double>::infinity ()
                                : 2 * end_loading_timestamp;
            }
          else
            {
              return TFlt (0);
            }
        }
    }
  else if (MNM_Ults::approximate_equal (_cc_out, _cc_in))
    {
      TFlt _threshold = TFlt (_cc_ceil);
      if (MNM_Ults::approximate_equal (_cc_in, TFlt (_cc_ceil)))
        {
          _threshold = _cc_ceil + 1;
        }
      for (int i = 1; i <= int (busstop->m_last_valid_time) - _time; ++i)
        {
          TFlt _cc_in_2 = busstop->m_N_in_bus->get_result (TFlt (_time + i));
          if (!MNM_Ults::approximate_less_than (_cc_in_2, _threshold)
              && MNM_Ults::approximate_less_than (_cc_in, _cc_in_2))
            {
              return TFlt (i); // interval
            }
        }
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }
  else
    {
      throw std::runtime_error (
        "Error, get_bus_waiting_time out cc larger than in cc");
      return -1;
    }
}

TFlt
get_travel_time_walking (MNM_Walking_Link *link, TFlt start_time,
                         TFlt unit_interval, TInt end_loading_timestamp)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_walking link is null");
    }
  if (link->m_N_in == nullptr || link->m_N_out == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_walking link "
                                "cumulative curve is not installed");
    }

  TFlt fftt
    = link->m_fftt / unit_interval + link->m_historical_bus_waiting_time;

  if (link->m_last_valid_time < 0)
    {
      link->m_last_valid_time
        = get_last_valid_time (link->m_N_in, link->m_N_out,
                               end_loading_timestamp);
    }
  IAssert (link->m_last_valid_time >= 0);

  return get_travel_time_from_cc (start_time, link->m_N_in, link->m_N_out,
                                  link->m_last_valid_time, fftt);
}

TFlt
get_travel_time_walking_robust (MNM_Walking_Link *link, TFlt start_time,
                                TFlt end_time, TFlt unit_interval,
                                TInt end_loading_timestamp, TInt num_trials)
{
  num_trials = num_trials > TInt (end_time - start_time)
                 ? TInt (end_time - start_time)
                 : num_trials;
  TFlt _delta = (end_time - start_time) / TFlt (num_trials);
  TFlt _ave_tt = TFlt (0);
  for (int i = 0; i < num_trials (); ++i)
    {
      _ave_tt += get_travel_time_walking (link, start_time + TFlt (i) * _delta,
                                          unit_interval, end_loading_timestamp);
    }
  return _ave_tt / TFlt (num_trials);
}

TFlt
get_travel_time_bus (MNM_Bus_Link *link, TFlt start_time, TFlt unit_interval,
                     TInt end_loading_timestamp, bool explicit_bus,
                     bool return_inf, bool return_bus_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_bus link is null");
    }

  if (!explicit_bus)
    {
      TFlt _tt;
      if (return_bus_time)
        {
          _tt = start_time;
        }
      else
        {
          _tt = start_time + link->m_from_busstop->m_historical_waiting_time;
        }
      for (size_t i = 0; i < link->m_overlapped_driving_link_vec.size (); ++i)
        {
          auto *_drivinglink = dynamic_cast<MNM_Dlink_Multiclass *> (
            link->m_overlapped_driving_link_vec[i]);
          _tt += get_travel_time_truck (_drivinglink, _tt, unit_interval,
                                        end_loading_timestamp)
                 * link->m_overlapped_driving_link_length_portion_vec[i];
        }
      return _tt - start_time;
    }

  if (link->m_from_busstop->m_N_in_bus == nullptr
      || link->m_to_busstop->m_N_in_bus == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_bus bus stop "
                                "m_N_in_bus cumulative curve is not installed");
    }

  TFlt _wait_time
    = get_bus_waiting_time (link->m_from_busstop, start_time, unit_interval,
                            end_loading_timestamp, return_inf);
  if (std::isinf (_wait_time))
    {
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  if (link->m_last_valid_time_bus < 0)
    {
      link->m_last_valid_time_bus
        = get_last_valid_time_bus (link->m_from_busstop->m_N_in_bus,
                                   link->m_to_busstop->m_N_in_bus,
                                   end_loading_timestamp);
    }
  if (link->m_last_valid_time < 0)
    {
      link->m_last_valid_time = link->m_last_valid_time_bus;
    }
  IAssert (link->m_last_valid_time >= 0 && link->m_last_valid_time_bus >= 0);

  if (start_time + _wait_time > link->m_last_valid_time_bus)
    {
      return return_inf ? std::numeric_limits<double>::infinity ()
                        : 2 * end_loading_timestamp;
    }

  // TFlt fftt = link -> m_fftt / unit_interval;
  TFlt fftt = -1; // in get_travel_time_from_cc()

  if (link->m_from_busstop->m_N_in_bus->get_result (start_time + _wait_time)
      <= DBL_EPSILON)
    {
      throw std::runtime_error (
        "Error, get_travel_time_bus no bus, check get_bus_waiting_time");
    }
  TFlt _bus_time
    = get_travel_time_from_cc (start_time + _wait_time,
                               link->m_from_busstop->m_N_in_bus,
                               link->m_to_busstop->m_N_in_bus,
                               link->m_last_valid_time_bus, fftt, true);
  if (_bus_time < 0)
    {
      std::cout << link->m_from_busstop->m_N_in_bus->to_string () << std::endl;
      std::cout << link->m_to_busstop->m_N_in_bus->to_string () << std::endl;
      throw std::runtime_error (
        "Error, get_travel_time_bus no bus, check get_bus_waiting_time");
    }
  if (return_bus_time)
    {
      if (_wait_time == 0.)
        {
          return _bus_time;
        }
      else
        {
          return -1;
        }
    }
  else
    {
      return _wait_time + _bus_time;
    }
}

TFlt
get_travel_time_bus_robust (MNM_Bus_Link *link, TFlt start_time, TFlt end_time,
                            TFlt unit_interval, TInt end_loading_timestamp,
                            TInt num_trials, bool explicit_bus, bool return_inf,
                            bool return_bus_time)
{
  TFlt _delta;
  TFlt _ave_tt = TFlt (0);
  if (return_bus_time)
    {
      num_trials = TInt (end_time - start_time);
      _delta = (end_time - start_time) / TFlt (num_trials);
      TInt _bus_count = 0;
      for (int i = 0; i < num_trials (); ++i)
        {
          TFlt _bus_time
            = get_travel_time_bus (link, start_time + TFlt (i) * _delta,
                                   unit_interval, end_loading_timestamp,
                                   explicit_bus, true, true);
          if (explicit_bus)
            {
              if (!std::isinf (_bus_time) && _bus_time > 0)
                {
                  _ave_tt += _bus_time;
                  _bus_count += 1;
                }
            }
          else
            {
              _ave_tt += _bus_time;
            }
        }
      if (explicit_bus)
        {
          if (_bus_count > 0)
            {
              return _ave_tt / TFlt (_bus_count);
            }
          else
            {
              return std::numeric_limits<double>::infinity ();
            }
        }
    }
  else
    {
      num_trials = num_trials > TInt (end_time - start_time)
                     ? TInt (end_time - start_time)
                     : num_trials;
      _delta = (end_time - start_time) / TFlt (num_trials);
      for (int i = 0; i < num_trials (); ++i)
        {
          _ave_tt += get_travel_time_bus (link, start_time + TFlt (i) * _delta,
                                          unit_interval, end_loading_timestamp,
                                          explicit_bus, return_inf, false);
        }
    }
  return _ave_tt / TFlt (num_trials);
}

TFlt
get_link_inflow_bus (MNM_Bus_Link *link, TFlt start_time, TFlt end_time)
{
  // be consistent with updating link -> m_N_in_bus_tree in
  // MNM_Busstop_Virtual::release_bus() and add_dar_records_bus() only buses
  // leaving the busstop are deemed entering the next link
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_bus link is null");
    }
  if (link->m_from_busstop == nullptr
      || link->m_from_busstop->m_N_out_bus == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_bus bus stop "
                                "cumulative curve is not installed");
    }
  return link->m_from_busstop->m_N_out_bus->get_result (end_time)
         - link->m_from_busstop->m_N_out_bus->get_result (start_time);
}

TFlt
get_busstop_inflow_bus (MNM_Busstop_Virtual *busstop, TFlt start_time,
                        TFlt end_time)
{
  if (busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, get_busstop_inflow_bus busstop is null");
    }
  if (busstop->m_N_in_bus == nullptr)
    {
      throw std::runtime_error ("Error, get_busstop_inflow_bus busstop "
                                "cumulative curve is not installed");
    }
  return busstop->m_N_in_bus->get_result (end_time)
         - busstop->m_N_in_bus->get_result (start_time);
}

TFlt
get_link_inflow_passenger (MNM_Transit_Link *link, TFlt start_time,
                           TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_link_inflow_passenger link is null");
    }
  if (link->m_N_in == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_passenger link "
                                "cumulative curve is not installed");
    }
  return link->m_N_in->get_result (end_time)
         - link->m_N_in->get_result (start_time);
}

int
add_dar_records_bus (std::vector<dar_record *> &record, MNM_Bus_Link *link,
                     std::set<MNM_Path *> pathset, TFlt start_time,
                     TFlt end_time)
{
  // pathset includes fixed bus paths
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_bus link is null");
    }
  if (link->m_N_in_tree_bus == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_bus link cumulative "
                                "curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_bus->m_record)
    {
      _path = path_it.first;
      if (pathset.find (_path) != pathset.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  auto new_record = new dar_record ();
                  new_record->path_ID
                    = path_it.first
                        ->m_path_ID; // not bus route ID, the reordered path ID
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int
                  new_record->assign_int = depart_it.first;
                  new_record->link_ID = link->m_link_ID;
                  // the count of unit time interval (5s)
                  new_record->link_start_int = start_time;
                  new_record->flow = tmp_flow;
                  // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
                  // path_ID(), new_record -> assign_int(),
                  //     new_record -> link_ID(), (float)new_record ->
                  //     link_start_int(), (float) new_record -> flow());
                  record.push_back (new_record);
                }
            }
        }
    }
  return 0;
}

int
add_dar_records_passenger (std::vector<dar_record *> &record,
                           MNM_Transit_Link *link, std::set<MNM_Path *> pathset,
                           TFlt start_time, TFlt end_time)
{
  // link includes bus and walking links
  // pathset includes PnR and transit paths
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_passenger link is null");
    }
  if (link->m_N_in_tree == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_passenger link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree->m_record)
    {
      _path = path_it.first;
      if (pathset.find (_path) != pathset.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  auto new_record = new dar_record ();
                  new_record->path_ID = path_it.first->m_path_ID;
                  // the count of 1 min intervals, the passengers record this
                  // assign_int
                  new_record->assign_int = depart_it.first;
                  new_record->link_ID = link->m_link_ID;
                  // the count of unit time interval (5s)
                  new_record->link_start_int = start_time;
                  new_record->flow = tmp_flow;
                  // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
                  // path_ID(), new_record -> assign_int(),
                  //     new_record -> link_ID(), (float)new_record ->
                  //     link_start_int(), (float) new_record -> flow());
                  record.push_back (new_record);
                }
            }
        }
    }
  return 0;
}

int
add_dar_records_bus (std::vector<dar_record *> &record, MNM_Bus_Link *link,
                     std::set<TInt> pathID_set, TFlt start_time, TFlt end_time)
{
  // pathset includes fixed bus paths
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_bus link is null");
    }
  if (link->m_N_in_tree_bus == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_bus link cumulative "
                                "curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_bus->m_record)
    {
      _path = path_it.first;
      if (pathID_set.find (_path->m_path_ID) != pathID_set.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  auto new_record = new dar_record ();
                  new_record->path_ID
                    = path_it.first
                        ->m_path_ID; // not bus route ID, the reordered path ID
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int
                  new_record->assign_int = depart_it.first;
                  new_record->link_ID = link->m_link_ID;
                  // the count of unit time interval (5s)
                  new_record->link_start_int = start_time;
                  new_record->flow = tmp_flow;
                  // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
                  // path_ID(), new_record -> assign_int(),
                  //     new_record -> link_ID(), (float)new_record ->
                  //     link_start_int(), (float) new_record -> flow());
                  record.push_back (new_record);
                }
            }
        }
    }
  return 0;
}

int
add_dar_records_passenger (std::vector<dar_record *> &record,
                           MNM_Transit_Link *link, std::set<TInt> pathID_set,
                           TFlt start_time, TFlt end_time)
{
  // link includes bus and walking links
  // pathset includes PnR and transit paths
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_passenger link is null");
    }
  if (link->m_N_in_tree == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_passenger link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree->m_record)
    {
      _path = path_it.first;
      if (pathID_set.find (_path->m_path_ID) != pathID_set.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  auto new_record = new dar_record ();
                  new_record->path_ID = path_it.first->m_path_ID;
                  // the count of 1 min intervals, the passengers record this
                  // assign_int
                  new_record->assign_int = depart_it.first;
                  new_record->link_ID = link->m_link_ID;
                  // the count of unit time interval (5s)
                  new_record->link_start_int = start_time;
                  new_record->flow = tmp_flow;
                  // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
                  // path_ID(), new_record -> assign_int(),
                  //     new_record -> link_ID(), (float)new_record ->
                  //     link_start_int(), (float) new_record -> flow());
                  record.push_back (new_record);
                }
            }
        }
    }
  return 0;
}

TFlt
get_departure_cc_slope_walking_passenger (MNM_Walking_Link *link,
                                          TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope_walking_passenger link is null");
    }
  if (link->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope_walking_passenger link cumulative curve "
        "is not installed");
    }
  int _delta = int (end_time) - int (start_time);
  IAssert (_delta > 0);
  TFlt _cc1, _cc2, _slope = 0.;
  for (int i = 0; i < _delta; i++)
    {
      _cc1 = link->m_N_out->get_result (TFlt (start_time + i));
      _cc2 = link->m_N_out->get_result (TFlt (start_time + i + 1));
      _slope += (_cc2 - _cc1);
    }
  return _slope / _delta; // flow per unit interval
}

TFlt
get_departure_cc_slope_bus_passenger (MNM_Bus_Link *link, TFlt start_time,
                                      TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope_bus_passenger link is null");
    }
  if (link->m_N_out == nullptr)
    {
      throw std::runtime_error ("Error, get_departure_cc_slope_bus_passenger "
                                "link cumulative curve is not installed");
    }
  int _delta = int (end_time) - int (start_time);
  IAssert (_delta > 0);
  TFlt _cc1, _cc2, _slope = 0.;
  for (int i = 0; i < _delta; i++)
    {
      _cc1 = link->m_N_out->get_result (TFlt (start_time + i));
      _cc2 = link->m_N_out->get_result (TFlt (start_time + i + 1));
      _slope += (_cc2 - _cc1);
    }
  return _slope / _delta; // flow per unit interval
}

int
add_ltg_records_passenger (std::vector<ltg_record *> &record,
                           MNM_Transit_Link *link, MNM_Path *path,
                           int depart_time, int start_time, TFlt gradient)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_ltg_records_passenger link is null");
    }
  if (path == nullptr)
    {
      throw std::runtime_error (
        "Error, add_ltg_records_passenger path is null");
    }
  if (!path->is_link_in (link->m_link_ID))
    {
      throw std::runtime_error (
        "Error, add_ltg_records_passenger link is not in path");
    }

  auto new_record = new ltg_record ();
  new_record->path_ID = path->m_path_ID;
  // the count of 1 min intervals in terms of 5s intervals, the passengers
  // record this assign_int
  new_record->assign_int = depart_time;
  new_record->link_ID = link->m_link_ID;
  // the count of unit time interval (5s)
  new_record->link_start_int = start_time;
  new_record->gradient = gradient;
  // printf("Adding record, %d, %d, %d, %d, %f\n", new_record -> path_ID(),
  // new_record -> assign_int,
  //         new_record -> link_ID(), new_record -> link_start_int, (float)
  //         new_record -> gradient());
  record.push_back (new_record);
  return 0;
}

}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                                Passenger path
*******************************************************************************************************************
******************************************************************************************************************/

/**************************************************************************
                                                   Base
**************************************************************************/
MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (int mode, TFlt vot,
                                                  TFlt early_penalty,
                                                  TFlt late_penalty,
                                                  TFlt target_time)
    : MNM_Path::MNM_Path ()
{
  m_mode = mode;
  m_path = nullptr;
  m_vot = vot;                     // money / interval
  m_early_penalty = early_penalty; // money / interval
  m_late_penalty = late_penalty;   // money / interval
  m_target_time = target_time;     // // in terms of number of unit intervals
}

MNM_Passenger_Path_Base::~MNM_Passenger_Path_Base () { delete m_path; }

TFlt
MNM_Passenger_Path_Base::get_wrongtime_penalty (TFlt arrival_time)
{
  return MNM_Ults::max (m_late_penalty * (arrival_time - m_target_time),
                        m_early_penalty * (m_target_time - arrival_time));
}

/**************************************************************************
                                                   Driving
**************************************************************************/
MNM_Passenger_Path_Driving::MNM_Passenger_Path_Driving (
  int mode, MNM_Path *path, TFlt vot, TFlt early_penalty, TFlt late_penalty,
  TFlt target_time, TInt num_people, TFlt carpool_cost_multiplier,
  TFlt walking_time_before_driving, MNM_Parking_Lot *parking_lot,
  TFlt walking_time_after_driving)
    : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (mode, vot,
                                                        early_penalty,
                                                        late_penalty,
                                                        target_time)
{
  IAssert (parking_lot == nullptr
           || parking_lot->m_node_ID == path->m_node_vec.back ());
  IAssert (num_people >= 1);

  m_path = path;
  m_num_people = num_people;

  m_parking_lot = parking_lot;
  // used for only driving path with walking_link_after_driving == nullptr
  m_walking_time_after_driving = walking_time_after_driving; // seconds

  m_carpool_cost_multiplier = carpool_cost_multiplier;
  m_walking_time_before_driving = walking_time_before_driving; // seconds
}

MNM_Passenger_Path_Driving::~MNM_Passenger_Path_Driving () { ; }

TFlt
MNM_Passenger_Path_Driving::get_carpool_cost ()
{
  return TFlt (m_carpool_cost_multiplier * (m_num_people - 1));
}

TFlt
MNM_Passenger_Path_Driving::get_amortized_parkingfee ()
{
  if (m_parking_lot == nullptr)
    {
      // return TFlt(0);
      // for Ohio, temporarily,
      return TFlt (8);
    }
  return TFlt (m_parking_lot->m_base_price * m_parking_lot->m_price_surge_coeff
               / m_num_people);
}

TFlt
MNM_Passenger_Path_Driving::get_toll (MNM_Dta_Multimodal *mmdta)
{
  TFlt _toll = 0.;
  for (auto _link_ID : m_path->m_link_vec)
    {
      _toll += dynamic_cast<MNM_Dlink_Multiclass *> (
                 mmdta->m_link_factory->get_link (_link_ID))
                 ->m_toll_car;
    }
  return _toll;
}

TFlt
MNM_Passenger_Path_Driving::get_length (MNM_Dta_Multimodal *mmdta)
{
  return m_path->get_path_length (mmdta->m_link_factory); // meter
}

TFlt
MNM_Passenger_Path_Driving::get_travel_time_truck (TFlt start_time,
                                                   MNM_Dta_Multimodal *mmdta)
{
  MNM_Dlink *_link;
  int arrival_time = int (round (start_time));
  for (TInt _link_ID : m_path->m_link_vec)
    {
      _link = mmdta->m_link_factory->get_link (_link_ID);
      if (auto *_mclink = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          // use arrival_time + 1 as start_time in cc to compute the link travel
          // time for vehicles arriving at the beginning of the interval
          // arrival_time
          arrival_time += MNM_Ults::round_up_time (
            MNM_DTA_GRADIENT::
              get_travel_time_truck (_mclink, TFlt (arrival_time + 1),
                                     mmdta->m_unit_time,
                                     mmdta->m_current_loading_interval));
        }
      else
        {
          throw std::runtime_error ("MNM_Passenger_Path_Driving::get_travel_"
                                    "time_truck: link type is not multiclass");
        }
    }
  return TFlt (arrival_time - start_time); // intervals
}

TFlt
MNM_Passenger_Path_Driving::get_travel_time_truck (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> link_tt_map_truck)
{
  MNM_Dlink *_link;
  int arrival_time = int (round (start_time));
  for (TInt _link_ID : m_path->m_link_vec)
    {
      _link = mmdta->m_link_factory->get_link (_link_ID);
      if (dynamic_cast<MNM_Dlink_Multiclass *> (_link) != nullptr)
        {
          arrival_time += MNM_Ults::round_up_time (
            link_tt_map_truck.find (_link_ID)
              ->second[arrival_time < (int) mmdta->m_current_loading_interval
                         ? arrival_time
                         : (int) mmdta->m_current_loading_interval - 1]);
        }
      else
        {
          throw std::runtime_error ("MNM_Passenger_Path_Driving::get_travel_"
                                    "time_truck: link type is not multiclass");
        }
    }
  return TFlt (arrival_time - start_time); // intervals
}

TFlt
MNM_Passenger_Path_Driving::get_travel_time (TFlt start_time,
                                             MNM_Dta_Multimodal *mmdta)
{
  MNM_Dlink *_link;
  int arrival_time
    = int (round (start_time))
      + int (ceil (m_walking_time_before_driving / mmdta->m_unit_time));
  for (TInt _link_ID : m_path->m_link_vec)
    {
      _link = mmdta->m_link_factory->get_link (_link_ID);
      if (auto *_mclink = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          // use arrival_time + 1 as start_time in cc to compute the link travel
          // time for vehicles arriving at the beginning of the interval
          // arrival_time
          arrival_time += MNM_Ults::round_up_time (
            MNM_DTA_GRADIENT::
              get_travel_time_car (_mclink, TFlt (arrival_time + 1),
                                   mmdta->m_unit_time,
                                   mmdta->m_current_loading_interval));
        }
      else
        {
          throw std::runtime_error ("MNM_Passenger_Path_Driving::get_travel_"
                                    "time: link type is not multiclass");
        }
    }
  if (m_parking_lot != nullptr)
    {
      arrival_time += int (
        ceil (mmdta->m_parkinglot_factory->get_parking_lot (m_parking_lot->m_ID)
                ->get_cruise_time (arrival_time)));
    }
  arrival_time
    += int (ceil (m_walking_time_after_driving / mmdta->m_unit_time));
  return TFlt (arrival_time - start_time); // intervals
}

TFlt
MNM_Passenger_Path_Driving::get_travel_cost (TFlt start_time,
                                             MNM_Dta_Multimodal *mmdta)
{
  TFlt tt = get_travel_time (start_time, mmdta);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_Driving::get_travel_time (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  int arrival_time
    = int (round (start_time))
      + int (ceil (m_walking_time_before_driving / mmdta->m_unit_time));
  IAssert (mmdta->m_current_loading_interval > 0);
  for (TInt _link_ID : m_path->m_link_vec)
    {
      arrival_time += MNM_Ults::round_up_time (
        driving_link_tt_map.find (_link_ID)
          ->second[arrival_time < (int) mmdta->m_current_loading_interval
                     ? arrival_time
                     : (int) mmdta->m_current_loading_interval - 1]);
    }
  if (m_parking_lot != nullptr)
    {
      arrival_time += int (
        ceil (mmdta->m_parkinglot_factory->get_parking_lot (m_parking_lot->m_ID)
                ->get_cruise_time (arrival_time)));
    }
  arrival_time
    += int (ceil (m_walking_time_after_driving / mmdta->m_unit_time));
  return TFlt (arrival_time - start_time); // intervals

} // interval

TFlt
MNM_Passenger_Path_Driving::get_travel_cost (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  TFlt tt = get_travel_time (start_time, mmdta, driving_link_tt_map,
                             bustransit_link_tt_map);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_Driving::get_travel_cost_with_tt (TFlt start_time,
                                                     TFlt travel_time,
                                                     MNM_Dta_Multimodal *mmdta)
{
  TFlt wrongtime_penalty = get_wrongtime_penalty (start_time + travel_time);
  return m_vot * travel_time + wrongtime_penalty + get_carpool_cost ()
         + get_amortized_parkingfee () + get_toll (mmdta);
}

bool
MNM_Passenger_Path_Driving::is_equal (MNM_Passenger_Path_Base *path)
{
  auto *_path_driving = dynamic_cast<MNM_Passenger_Path_Driving *> (path);
  if (_path_driving == nullptr)
    {
      return false;
    }
  else
    {
      if (m_num_people != _path_driving->m_num_people)
        return false;
      if (m_parking_lot != _path_driving->m_parking_lot)
        return false;
      if (!(*m_path == *(_path_driving->m_path)))
        return false;
      return true;
    }
}

std::string
MNM_Passenger_Path_Driving::info2str ()
{
  std::string _s;
  // origin_node_ID
  _s = std::to_string (m_path->m_node_vec.front ()) + " ";
  // dest_node_ID
  _s += std::to_string (m_path->m_node_vec.back ()) + " ";
  // mode
  _s += std::string ("driving") + " ";
  // mid_dest_node_ID
  _s += std::string ("-1") + " ";
  // parkinglot_ID
  if (m_parking_lot != nullptr)
    {
      _s += std::to_string (m_parking_lot->m_ID) + " ";
    }
  else
    {
      _s += std::to_string (-1) + " ";
    }
  // <driving_node_sequence>, \n included
  _s += std::string ("<") + m_path->node_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">") + " ";
  // <bus_transit_link_sequence>, \n included
  _s += std::string ("<--->") + "\n";
  return _s;
};

/**************************************************************************
                                                   Bus Transit
**************************************************************************/
MNM_Passenger_Path_Bus::MNM_Passenger_Path_Bus (int mode, MNM_Path *path,
                                                TFlt vot, TFlt early_penalty,
                                                TFlt late_penalty,
                                                TFlt target_time, TFlt bus_fare,
                                                TFlt bus_inconvenience)
    : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (mode, vot,
                                                        early_penalty,
                                                        late_penalty,
                                                        target_time)
{
  m_path = path;
  m_bus_fare = bus_fare;
  m_bus_inconvenience = bus_inconvenience;
}

MNM_Passenger_Path_Bus::~MNM_Passenger_Path_Bus () { ; };

TFlt
MNM_Passenger_Path_Bus::get_length (MNM_Dta_Multimodal *mmdta)
{
  TFlt _len = 0.;
  MNM_Transit_Link *_link;
  for (auto _link_ID : m_path->m_link_vec)
    {
      _link = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
      if (_link->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
        {
          _len += dynamic_cast<MNM_Bus_Link *> (_link)->m_length;
        }
      else if (_link->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
        {
          if (dynamic_cast<MNM_Walking_Link *> (_link)->m_walking_type
              == "normal")
            {
              _len += _link->m_fftt * 1.5; // meter, walking speed 1.5 m/s
            }
        }
      else
        {
          throw std::runtime_error ("Wrong type of transit link");
        }
    }
  return _len; // meter
}

TFlt
MNM_Passenger_Path_Bus::get_travel_time (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  int arrival_time = int (round (start_time));
  MNM_Transit_Link *_link;
  TFlt _tt;
  for (size_t i = 0; i < m_path->m_link_vec.size (); ++i)
    {
      TInt _link_ID = m_path->m_link_vec[i];
      _link = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
      if (_link->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
        {
          // use arrival_time + 1 as start_time in cc to compute the link travel
          // time for vehicles arriving at the beginning of the interval
          // arrival_time
          _tt = MNM_DTA_GRADIENT::
            get_travel_time_bus (dynamic_cast<MNM_Bus_Link *> (_link),
                                 TFlt (arrival_time + 1), mmdta->m_unit_time,
                                 mmdta->m_current_loading_interval,
                                 mmdta->m_explicit_bus, false,
                                 !mmdta->m_explicit_bus);
          // if (std::isinf(_t)) {
          //     return std::numeric_limits<double>::infinity();
          // }
          if (_tt > mmdta->m_current_loading_interval ())
            {
              return 2 * mmdta->m_current_loading_interval ();
            }
          arrival_time += MNM_Ults::round_up_time (_tt);
        }
      else if (_link->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
        {
          // use arrival_time + 1 as start_time in cc to compute the link travel
          // time for vehicles arriving at the beginning of the interval
          // arrival_time
          arrival_time += MNM_Ults::round_up_time (
            MNM_DTA_GRADIENT::
              get_travel_time_walking (dynamic_cast<MNM_Walking_Link *> (_link),
                                       TFlt (arrival_time + 1),
                                       mmdta->m_unit_time,
                                       mmdta->m_current_loading_interval));
        }
      else
        {
          throw std::runtime_error ("Wrong type of transit link");
        }
    }
  return TFlt (arrival_time - start_time); // intervals
}

TFlt
MNM_Passenger_Path_Bus::get_total_bus_fare (MNM_Dta_Multimodal *mmdta)
{
  TInt _num_routes = 0;

  // count the transfers when transfers are not free
  // MNM_Transit_Link* _link;
  // TInt _route_ID = -1;
  // for (auto _link_ID : m_path -> m_link_vec) {
  //     _link = mmdta -> m_transitlink_factory -> get_transit_link(_link_ID);
  //     if (_link -> m_link_type == MNM_TYPE_BUS_MULTIMODAL) {
  //         if (_route_ID != dynamic_cast<MNM_Bus_Link*>(_link) -> m_route_ID)
  //         {
  //             _route_ID = dynamic_cast<MNM_Bus_Link*>(_link) -> m_route_ID;
  //             _num_routes += 1;
  //         }
  //     }
  // }

  // when transfer is free
  _num_routes = 1;

  IAssert (_num_routes >= 1);
  return _num_routes * m_bus_fare;
}

TFlt
MNM_Passenger_Path_Bus::get_travel_cost (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  TFlt tt = get_travel_time (start_time, mmdta);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_Bus::get_travel_time (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  int arrival_time = int (round (start_time));
  IAssert (mmdta->m_current_loading_interval > 0);
  for (auto _link_ID : m_path->m_link_vec)
    {
      TFlt _tt
        = bustransit_link_tt_map.find (_link_ID)
            ->second[arrival_time < (int) mmdta->m_current_loading_interval
                       ? arrival_time
                       : (int) mmdta->m_current_loading_interval - 1];
      // if (std::isinf(_tt)) {
      //     return std::numeric_limits<double>::infinity();
      // }
      if (_tt > mmdta->m_current_loading_interval ())
        {
          return 2 * mmdta->m_current_loading_interval ();
        }
      arrival_time += MNM_Ults::round_up_time (_tt);
    }
  return TFlt (arrival_time - start_time); // intervals

} // interval

TFlt
MNM_Passenger_Path_Bus::get_travel_cost (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  TFlt tt = get_travel_time (start_time, mmdta, driving_link_tt_map,
                             bustransit_link_tt_map);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_Bus::get_travel_cost_with_tt (TFlt start_time,
                                                 TFlt travel_time,
                                                 MNM_Dta_Multimodal *mmdta)
{
  TFlt wrongtime_penalty = get_wrongtime_penalty (start_time + travel_time);
  TFlt tot_fare = get_total_bus_fare (mmdta);
  if (travel_time > mmdta->m_current_loading_interval ())
    {
      return 1e3
             * (m_vot * travel_time + wrongtime_penalty + tot_fare
                + m_bus_inconvenience);
    }
  return m_vot * travel_time + wrongtime_penalty + tot_fare
         + m_bus_inconvenience;
}

bool
MNM_Passenger_Path_Bus::is_equal (MNM_Passenger_Path_Base *path)
{
  auto *_path_bus = dynamic_cast<MNM_Passenger_Path_Bus *> (path);
  if (_path_bus == nullptr)
    {
      return false;
    }
  else
    {
      if (!(*m_path == *(_path_bus->m_path)))
        return false;
      return true;
    }
};

std::string
MNM_Passenger_Path_Bus::info2str ()
{
  std::string _s;
  // origin_node_ID
  _s = std::to_string (m_path->m_node_vec.front ()) + " ";
  // dest_node_ID
  _s += std::to_string (m_path->m_node_vec.back ()) + " ";
  // mode
  _s += std::string ("bus") + " ";
  // mid_dest_node_ID
  _s += std::string ("-1") + " ";
  // parkinglot_ID
  _s += std::string ("-1") + " ";
  // <driving_node_sequence>, \n included
  _s += std::string ("<--->") + " ";
  // <bus_transit_link_sequence>, \n included
  _s += std::string ("<") + m_path->link_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">\n");
  return _s;
};

/**************************************************************************
                                                   Metro Transit
**************************************************************************/
MNM_Passenger_Path_Metro::MNM_Passenger_Path_Metro (
  int mode, MNM_Path *path, TFlt vot, TFlt early_penalty, TFlt late_penalty,
  TFlt target_time, TFlt metro_fare, TFlt metro_inconvenience, TFlt metro_time,
  TFlt waiting_time)
    : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (mode, vot,
                                                        early_penalty,
                                                        late_penalty,
                                                        target_time)
{
  m_path = path;

  m_metro_fare = metro_fare;
  m_metro_inconvenience = metro_inconvenience;
  m_metro_time = metro_time; // intervals, need metro network
}

MNM_Passenger_Path_Metro::~MNM_Passenger_Path_Metro () { ; }

TFlt
MNM_Passenger_Path_Metro::get_length (MNM_Dta_Multimodal *mmdta)
{
  // TODO: add metro transit network
  return 0; // meter
}

TFlt
MNM_Passenger_Path_Metro::get_travel_time (TFlt start_time,
                                           MNM_Dta_Multimodal *mmdta)
{
  // TODO: add metro transit network
  return 0; // interval
}

TFlt
MNM_Passenger_Path_Metro::get_total_metro_fare (MNM_Dta_Multimodal *mmdta)
{
  // TODO: count the transfers
  TInt _num_routes = 0;
  TInt _route_ID = -1;
  // IAssert(_num_routes >= 1);
  return _num_routes * m_metro_fare;
}

TFlt
MNM_Passenger_Path_Metro::get_travel_cost (TFlt start_time,
                                           MNM_Dta_Multimodal *mmdta)
{
  TFlt tt = get_travel_time (start_time, mmdta);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_Metro::get_travel_cost_with_tt (TFlt start_time,
                                                   TFlt travel_time,
                                                   MNM_Dta_Multimodal *mmdta)
{
  TFlt wrongtime_penalty = get_wrongtime_penalty (start_time + travel_time);
  TFlt _tot_fare = get_total_metro_fare (mmdta);
  if (travel_time > mmdta->m_current_loading_interval ())
    {
      return 1e3
             * (m_vot * travel_time + wrongtime_penalty + _tot_fare
                + m_metro_inconvenience);
    }
  return m_vot * travel_time + wrongtime_penalty + _tot_fare
         + m_metro_inconvenience;
}

bool
MNM_Passenger_Path_Metro::is_equal (MNM_Passenger_Path_Base *path)
{
  auto *_path_metro = dynamic_cast<MNM_Passenger_Path_Metro *> (path);
  if (_path_metro == nullptr)
    {
      return false;
    }
  else
    {
      if (!(*m_path == *(_path_metro->m_path)))
        return false;
      return true;
    }
};

std::string
MNM_Passenger_Path_Metro::info2str ()
{
  std::string _s;
  // origin_node_ID
  _s = std::to_string (m_path->m_node_vec.front ()) + " ";
  // dest_node_ID
  _s += std::to_string (m_path->m_node_vec.back ()) + " ";
  // mode
  _s += std::string ("metro") + " ";
  // mid_dest_node_ID
  _s += std::string ("-1") + " ";
  // parkinglot_ID
  _s += std::string ("-1") + " ";
  // <driving_node_sequence>, \n included
  _s += std::string ("<--->") + " ";
  // <metro_transit_link_sequence>, \n included
  _s += std::string ("<") + m_path->link_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">\n");
  return _s;
};

/**************************************************************************
                                                   Park & Ride
**************************************************************************/
MNM_Passenger_Path_PnR::MNM_Passenger_Path_PnR (
  int mode, MNM_PnR_Path *path, TFlt vot, TFlt early_penalty, TFlt late_penalty,
  TFlt target_time, TFlt walking_time_before_driving,
  MNM_Parking_Lot *parking_lot, TFlt bus_fare, TFlt pnr_inconvenience)
    : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (mode, vot,
                                                        early_penalty,
                                                        late_penalty,
                                                        target_time)
{
  IAssert (parking_lot != nullptr);
  IAssert (parking_lot->m_node_ID == path->m_driving_path->m_node_vec.back ());
  IAssert (path->m_driving_path->m_node_vec.back ()
           == path->m_transit_path->m_node_vec.front ());
  m_path = path;
  // the parking lot cruising time is counted in the walking link out of the
  // parking lot
  m_driving_part
    = new MNM_Passenger_Path_Driving (driving, path->m_driving_path, vot,
                                      early_penalty, late_penalty, target_time,
                                      TInt (1), TFlt (0),
                                      walking_time_before_driving, nullptr,
                                      TFlt (0));
  m_bus_part = new MNM_Passenger_Path_Bus (transit, path->m_transit_path, vot,
                                           early_penalty, late_penalty,
                                           target_time, bus_fare, TFlt (0));
  m_mid_parking_lot = parking_lot;
  m_pnr_inconvenience = pnr_inconvenience;
}

MNM_Passenger_Path_PnR::~MNM_Passenger_Path_PnR ()
{
  if (m_driving_part != nullptr)
    delete m_driving_part;
  if (m_bus_part != nullptr)
    delete m_bus_part;
}

TFlt
MNM_Passenger_Path_PnR::get_length (MNM_Dta_Multimodal *mmdta)
{
  TFlt _len
    = m_driving_part->get_length (mmdta) + m_bus_part->get_length (mmdta);
  return _len; // meter
}

TFlt
MNM_Passenger_Path_PnR::get_travel_time (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  // driving part
  TFlt arrival_time
    = start_time + m_driving_part->get_travel_time (start_time, mmdta);
  // bus transit part, parking cruising time is counted in the walking out link
  // from parking lot
  arrival_time += m_bus_part->get_travel_time (arrival_time, mmdta);
  return arrival_time - start_time; // intervals
}

TFlt
MNM_Passenger_Path_PnR::get_travel_cost (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  TFlt tt = get_travel_time (start_time, mmdta);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_PnR::get_travel_cost_with_tt (TFlt start_time,
                                                 TFlt travel_time,
                                                 MNM_Dta_Multimodal *mmdta)
{
  TFlt wrongtime_penalty = get_wrongtime_penalty (start_time + travel_time);
  TFlt tot_fare = m_bus_part->get_total_bus_fare (mmdta);
  // for pnr, there is a parking fee for middle destination
  // return m_vot * travel_time + wrongtime_penalty +
  // m_driving_part->get_amortized_parkingfee() + tot_fare + m_pnr_inconvenience
  // + m_driving_part -> get_toll(mmdta); for mobility service, no parking fee
  // for middle destination
  if (travel_time > mmdta->m_current_loading_interval ())
    {
      return 1e3
             * (m_vot * travel_time + wrongtime_penalty + tot_fare
                + m_pnr_inconvenience);
    }
  return m_vot * travel_time + wrongtime_penalty + tot_fare
         + m_pnr_inconvenience;
}

TFlt
MNM_Passenger_Path_PnR::get_travel_time (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  // driving part
  TFlt arrival_time
    = start_time
      + m_driving_part->get_travel_time (start_time, mmdta, driving_link_tt_map,
                                         bustransit_link_tt_map);
  // bus transit part, parking cruising time is counted in the walking out link
  // from parking lot
  arrival_time
    += m_bus_part->get_travel_time (arrival_time, mmdta, driving_link_tt_map,
                                    bustransit_link_tt_map);
  return arrival_time - start_time; // intervals
} // interval

TFlt
MNM_Passenger_Path_PnR::get_travel_cost (
  TFlt start_time, MNM_Dta_Multimodal *mmdta,
  std::unordered_map<TInt, TFlt *> driving_link_tt_map,
  std::unordered_map<TInt, TFlt *> bustransit_link_tt_map)
{
  TFlt tt = get_travel_time (start_time, mmdta, driving_link_tt_map,
                             bustransit_link_tt_map);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

bool
MNM_Passenger_Path_PnR::is_equal (MNM_Passenger_Path_Base *path)
{
  auto *_path_pnr = dynamic_cast<MNM_Passenger_Path_PnR *> (path);
  if (_path_pnr == nullptr)
    {
      return false;
    }
  else
    {
      if (!m_driving_part->is_equal (
            dynamic_cast<MNM_Passenger_Path_Base *> (_path_pnr->m_driving_part))
          || !m_bus_part->is_equal (
            dynamic_cast<MNM_Passenger_Path_Base *> (_path_pnr->m_bus_part)))
        {
          return false;
        }
      return true;
    }
};

std::string
MNM_Passenger_Path_PnR::info2str ()
{
  std::string _s;
  // origin_node_ID
  _s = std::to_string (m_path->m_driving_path->m_node_vec.front ()) + " ";
  // dest_node_ID
  _s += std::to_string (m_path->m_transit_path->m_node_vec.back ()) + " ";
  // mode
  _s += std::string ("pnr") + " ";
  // mid_dest_node_ID
  _s += std::to_string (m_path->m_driving_path->m_node_vec.back ()) + " ";
  // parkinglot_ID
  _s += std::to_string (m_mid_parking_lot->m_ID ()) + " ";
  // <driving_node_sequence>, \n included
  _s += std::string ("<") + m_path->m_driving_path->node_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">") + " ";
  // <bus_or_metro_transit_link_sequence>, \n included
  _s += std::string ("<") + m_path->m_transit_path->link_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">\n");
  return _s;
};

/**************************************************************************
                                                   Ride & Drive
**************************************************************************/
MNM_Passenger_Path_RnD::MNM_Passenger_Path_RnD (
  int mode, MNM_PnR_Path *path, TFlt vot, TFlt early_penalty, TFlt late_penalty,
  TFlt target_time, TFlt walking_time_before_driving,
  MNM_Parking_Lot *parking_lot, TFlt bus_fare, TFlt rnd_inconvenience)
    : MNM_Passenger_Path_Base::MNM_Passenger_Path_Base (mode, vot,
                                                        early_penalty,
                                                        late_penalty,
                                                        target_time)
{
  IAssert (parking_lot != nullptr);
  IAssert (parking_lot->m_node_ID == path->m_driving_path->m_node_vec.front ());
  IAssert (path->m_driving_path->m_node_vec.front ()
           == path->m_transit_path->m_node_vec.back ());
  m_path = path;
  // the parking lot cruising time is counted in the walking link out of the
  // parking lot
  m_bus_part = new MNM_Passenger_Path_Bus (transit, path->m_transit_path, vot,
                                           early_penalty, late_penalty,
                                           target_time, bus_fare, TFlt (0));
  m_driving_part
    = new MNM_Passenger_Path_Driving (driving, path->m_driving_path, vot,
                                      early_penalty, late_penalty, target_time,
                                      TInt (1), TFlt (0),
                                      walking_time_before_driving, nullptr,
                                      TFlt (0));

  m_mid_parking_lot = parking_lot;
  m_rnd_inconvenience = rnd_inconvenience;
}

MNM_Passenger_Path_RnD::~MNM_Passenger_Path_RnD ()
{
  if (m_driving_part != nullptr)
    delete m_driving_part;
  if (m_bus_part != nullptr)
    delete m_bus_part;
}

TFlt
MNM_Passenger_Path_RnD::get_length (MNM_Dta_Multimodal *mmdta)
{
  TFlt _len
    = m_bus_part->get_length (mmdta) + m_driving_part->get_length (mmdta);
  return _len; // meter
}

TFlt
MNM_Passenger_Path_RnD::get_travel_time (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  // bus transit part
  TFlt arrival_time
    = start_time + m_bus_part->get_travel_time (start_time, mmdta);
  // driving part, parking cruising time is counted in the walking out link from
  // parking lot
  arrival_time += m_driving_part->get_travel_time (arrival_time, mmdta);
  return arrival_time - start_time; // intervals
}

TFlt
MNM_Passenger_Path_RnD::get_travel_cost (TFlt start_time,
                                         MNM_Dta_Multimodal *mmdta)
{
  TFlt tt = get_travel_time (start_time, mmdta);
  return get_travel_cost_with_tt (start_time, tt, mmdta);
}

TFlt
MNM_Passenger_Path_RnD::get_travel_cost_with_tt (TFlt start_time,
                                                 TFlt travel_time,
                                                 MNM_Dta_Multimodal *mmdta)
{
  TFlt wrongtime_penalty = get_wrongtime_penalty (start_time + travel_time);
  TFlt tot_fare = m_bus_part->get_total_bus_fare (mmdta);
  // for rnd, there is a parking fee for middle destination
  // return m_vot * tt + wrongtime_penalty +
  // m_driving_part->get_amortized_parkingfee() + tot_fare + m_rnd_inconvenience
  // + m_driving_part -> get_toll(mmdta); for mobility service, no parking fee
  // for middle destination
  if (travel_time > mmdta->m_current_loading_interval ())
    {
      return 1e3
             * (m_vot * travel_time + wrongtime_penalty + tot_fare
                + m_rnd_inconvenience);
    }
  return m_vot * travel_time + wrongtime_penalty + tot_fare
         + m_rnd_inconvenience;
}

bool
MNM_Passenger_Path_RnD::is_equal (MNM_Passenger_Path_Base *path)
{
  auto *_path_rnd = dynamic_cast<MNM_Passenger_Path_RnD *> (path);
  if (_path_rnd == nullptr)
    {
      return false;
    }
  else
    {
      if (!m_driving_part->is_equal (
            dynamic_cast<MNM_Passenger_Path_Base *> (_path_rnd->m_driving_part))
          || !m_bus_part->is_equal (
            dynamic_cast<MNM_Passenger_Path_Base *> (_path_rnd->m_bus_part)))
        {
          return false;
        }
      return true;
    }
}

std::string
MNM_Passenger_Path_RnD::info2str ()
{
  std::string _s;
  // origin_node_ID
  _s = std::to_string (m_path->m_transit_path->m_node_vec.front ()) + " ";
  // dest_node_ID
  _s += std::to_string (m_path->m_driving_path->m_node_vec.back ()) + " ";
  // mode
  _s += std::string ("rnd") + " ";
  // mid_origin_node_ID
  _s += std::to_string (m_path->m_transit_path->m_node_vec.back ()) + " ";
  // parkinglot_ID
  _s += std::to_string (m_mid_parking_lot->m_ID ()) + " ";
  // <bus_or_metro_transit_link_sequence>, \n included
  _s += std::string ("<") + m_path->m_transit_path->link_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">") + " ";
  // <driving_node_sequence>, \n included
  _s += std::string ("<") + m_path->m_driving_path->node_vec_to_string ();
  _s.pop_back ();
  _s += std::string (">\n");
  return _s;
};

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Passenger Path Set
*******************************************************************************************************************
******************************************************************************************************************/
MNM_Passenger_Pathset::MNM_Passenger_Pathset (MMDue_mode mode)
{
  m_mode = mode; // driving, transit, pnr, rh
  m_path_vec = std::vector<MNM_Passenger_Path_Base *> ();
}

MNM_Passenger_Pathset::~MNM_Passenger_Pathset ()
{
  for (auto *_path : m_path_vec)
    {
      if (_path != nullptr)
        delete _path;
    }
  m_path_vec.clear ();
}

bool
MNM_Passenger_Pathset::is_in (MNM_Passenger_Path_Base *path)
{
  if (path->m_mode != (int) m_mode)
    return false;
  // https://stackoverflow.com/questions/92396/why-cant-variables-be-declared-in-a-switch-statement
  switch (m_mode)
    {
    case driving:
      {
        auto *_path = dynamic_cast<MNM_Passenger_Path_Driving *> (path);
        IAssert (_path != nullptr);
        for (auto *_path_it : m_path_vec)
          {
            if (_path->is_equal (_path_it))
              {
                return true;
              }
          }
        return false;
        //            break;
      }
    case transit:
      {
        // TODO: metro
        auto *_path = dynamic_cast<MNM_Passenger_Path_Bus *> (path);
        IAssert (_path != nullptr);
        for (auto *_path_it : m_path_vec)
          {
            if (_path->is_equal (_path_it))
              {
                return true;
              }
          }
        return false;
        //            break;
      }
    case pnr:
      {
        auto *_path = dynamic_cast<MNM_Passenger_Path_PnR *> (path);
        IAssert (_path != nullptr);
        for (auto *_path_it : m_path_vec)
          {
            if (_path->is_equal (_path_it))
              {
                return true;
              }
          }
        return false;
        //            break;
      }
    case rh:
      {
        throw std::runtime_error (
          "ride hailing passenger path not implemented");
      }
    default:
      {
        throw std::runtime_error ("undefined passenger path");
      }
    }
}

namespace MNM
{
std::unordered_map<int, TFlt>
logit_fn (std::unordered_map<int, TFlt> &cost,
          std::unordered_map<int, TFlt> &alpha_map, TFlt beta)
{
  std::unordered_map<int, TFlt> mode_split = std::unordered_map<int, TFlt> ();
  TFlt _sum = 0.;
  for (auto _it : cost)
    {
      if (std::isinf (_it.second))
        {
          mode_split.insert (std::pair<int, TFlt> (_it.first, 0.));
        }
      else
        {
          mode_split.insert (
            std::pair<int, TFlt> (_it.first,
                                  exp (-(alpha_map.find (_it.first)->second
                                         + beta * _it.second))));
        }
      _sum += mode_split.find (_it.first)->second;
    }
  // _sum = MNM_Ults::max(1e-4, _sum);
  IAssert (_sum > 0);
  for (auto _it : mode_split)
    {
      mode_split.find (_it.first)->second = _it.second / _sum;
    }
  return mode_split;
}

int
normalize_path_table_p (PnR_Path_Table *pnr_path_table)
{
  for (auto _it : *pnr_path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          _it_it.second->normalize_p ();
        }
    }
  return 0;
}

int
copy_buffer_to_p (PnR_Path_Table *pnr_path_table, TInt col)
{
  // printf("Entering MNM::copy_buffer_to_p\n");
  // printf("path table is %p\n", path_table);
  IAssert (col >= 0);
  for (auto _it : *pnr_path_table)
    {
      for (auto _it_it : *(_it.second))
        {
          for (MNM_Path *_path : _it_it.second->m_path_vec)
            {
              IAssert (_path->m_buffer_length > col);
              _path->m_p = _path->m_buffer[col];
            }
        }
    }
  return 0;
}

bool
has_running_passenger (MNM_Passenger_Factory *passenger_factory)
{
  // TInt _total_passenger = TInt(passenger_factory -> m_passenger_map.size());
  // TInt _finished_passenger = 0;
  // TInt _enroute_passenger;
  // for (auto _map_it : passenger_factory -> m_passenger_map){
  //     if (_map_it.second -> m_finish_time > 0) _finished_passenger += 1;
  // }
  // _enroute_passenger = _total_passenger - _finished_passenger;
  // return _enroute_passenger != 0;
  return passenger_factory->m_enroute_passenger != 0;
}

int
print_passenger_statistics (MNM_Passenger_Factory *passenger_factory)
{
  // TInt _total_passenger = TInt(passenger_factory -> m_passenger_map.size());
  // TInt _finished_passenger = 0;
  // TInt _enroute_passenger;
  // for (auto _map_it : passenger_factory -> m_passenger_map){
  //     if (_map_it.second -> m_finish_time > 0) _finished_passenger += 1;
  // }
  // _enroute_passenger = _total_passenger - _finished_passenger;
  // printf("Released passenger %d, Enroute passenger %d, Finished passenger
  // %d\n", _total_passenger(), _enroute_passenger(), _finished_passenger());

  printf (
    "############################################### Passenger Statistics ###############################################\n \
    Released Passenger Total %d, Enroute Passenger Total %d, Finished Passenger Total %d,\n \
    Total Travel Time Passenger %.2f intervals,\n \
    Released Passenger PnR %d, Enroute Passenger PnR %d, Finished Passenger PnR %d,\n \
    ############################################### Passenger Statistics ###############################################\n",
    passenger_factory->m_num_passenger (),
    passenger_factory->m_enroute_passenger (),
    passenger_factory->m_finished_passenger (),
    passenger_factory->m_total_time_passenger (),
    passenger_factory->m_num_passenger_pnr (),
    passenger_factory->m_enroute_passenger_pnr (),
    passenger_factory->m_finished_passenger_pnr ());
  return 0;
}

int
print_vehicle_statistics (MNM_Veh_Factory_Multimodal *veh_factory)
{
  printf (
    "############################################### Vehicle Statistics ###############################################\n \
    Released Vehicle total %d, Enroute Vehicle Total %d, Finished Vehicle Total %d,\n \
    Total Travel Time: %.2f intervals,\n \
    Released Car Driving %d, Enroute Car Driving %d, Finished Car Driving %d,\n \
    Released Car PnR %d, Enroute Car PnR %d, Finished Car PnR %d,\n \
    Released Truck %d, Enroute Truck %d, Finished Truck %d,\n \
    Released Bus %d, Enroute Bus %d, Finished Bus %d,\n \
    Total Travel Time Car: %.2f intervals, Total Travel Time Truck: %.2f intervals, Total Travel Time Bus: %.2f intervals\n \
    ############################################### Vehicle Statistics ###############################################\n",
    veh_factory->m_num_veh (), veh_factory->m_enroute (),
    veh_factory->m_finished (), veh_factory->m_total_time (),
    veh_factory->m_num_car (), veh_factory->m_enroute_car (),
    veh_factory->m_finished_car (), veh_factory->m_num_car_pnr (),
    veh_factory->m_enroute_car_pnr (), veh_factory->m_finished_car_pnr (),
    veh_factory->m_num_truck (), veh_factory->m_enroute_truck (),
    veh_factory->m_finished_truck (), veh_factory->m_num_bus (),
    veh_factory->m_enroute_bus (), veh_factory->m_finished_bus (),
    veh_factory->m_total_time_car (), veh_factory->m_total_time_truck (),
    veh_factory->m_total_time_bus ());
  return 0;
}

Passenger_Path_Table *
build_shortest_passenger_pathset (
  std::vector<MMDue_mode> &mode_vec,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &passenger_demand,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_MM_Due *mmdue, PNEGraph &driving_graph, PNEGraph &bustransit_graph,
  MNM_OD_Factory *od_factory, MNM_Link_Factory *link_factory,
  MNM_Transit_Link_Factory *transitlink_factory,
  MNM_Busstop_Factory *busstop_factory)
{
  // create an empty passenger path table, only for connected OD pairs with
  // demand
  auto *_path_table = new Passenger_Path_Table ();
  for (auto _o_it : passenger_demand)
    {
      auto *_new_map_1 = new std::unordered_map<
        TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *> *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<
                    TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *> *>
                    *> (_o_it.first, _new_map_1));
      for (auto _d_it : _o_it.second)
        {
          auto *_new_map_2
            = new std::unordered_map<TInt, MNM_Passenger_Pathset *> ();
          _new_map_1->insert (
            std::pair<TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *>
                              *> (_d_it.first, _new_map_2));
          for (auto _mode : mode_vec)
            {
              if (od_mode_connectivity.find (_o_it.first)
                    ->second.find (_d_it.first)
                    ->second.find (_mode)
                    ->second)
                {
                  auto _pathset = new MNM_Passenger_Pathset (_mode);
                  _new_map_2->insert (
                    std::pair<TInt, MNM_Passenger_Pathset *> (_mode, _pathset));
                }
            }
        }
    }

  // driving
  MNM_Passenger_Path_Driving *_p_path_driving;
  // bus
  MNM_Passenger_Path_Bus *_p_path_bus;
  // pnr
  MNM_Passenger_Path_PnR *_p_path_pnr;

  MNM_Parking_Lot *_best_mid_parkinglot;
  MNM_Destination_Multimodal *_dest;
  TInt _dest_node_ID, _origin_node_ID, _mid_dest_node_ID;

  TFlt _cur_best_path_tt, _path_tt;

  // <d_node_ID, <node_ID, out_link_ID>>
  std::unordered_map<TInt, std::unordered_map<TInt, TInt>> _driving_table
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();
  std::unordered_map<TInt, std::unordered_map<TInt, TInt>> _bustransit_table
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree_driving;
  std::unordered_map<TInt, TInt> _free_shortest_path_tree_bustransit;
  std::unordered_map<TInt, TInt> _free_shortest_path_tree_pnr;
  std::unordered_map<TInt, TFlt> _free_cost_map_driving
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TFlt> _free_cost_map_bustransit
    = std::unordered_map<TInt, TFlt> ();

  MNM_Path *_path;
  MNM_Path *_driving_path;
  MNM_Path *_transit_path;
  MNM_PnR_Path *_pnr_path;

  // TODO: use link cost instead of link travel time
  for (auto _link_it : link_factory->m_link_map)
    {
      _free_cost_map_driving.insert (
        std::pair<TInt, TFlt> (_link_it.first, _link_it.second->get_link_tt ()
                                                 / mmdue->m_unit_time));
    }
  for (auto _link_it : transitlink_factory->m_transit_link_map)
    {
      _free_cost_map_bustransit.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               _link_it.second->m_fftt / mmdue->m_unit_time));
    }

  for (auto _d_it : od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;

      _free_shortest_path_tree_driving = std::unordered_map<TInt, TInt> ();
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, driving_graph,
                                          _free_cost_map_driving,
                                          _free_shortest_path_tree_driving);
      _driving_table.insert (
        std::pair<TInt, std::unordered_map<
                          TInt, TInt>> (_dest_node_ID,
                                        _free_shortest_path_tree_driving));

      if (bustransit_graph->IsNode (_dest_node_ID))
        {
          _free_shortest_path_tree_bustransit
            = std::unordered_map<TInt, TInt> ();
          MNM_Shortest_Path::
            all_to_one_FIFO (_dest_node_ID, bustransit_graph,
                             _free_cost_map_bustransit,
                             _free_shortest_path_tree_bustransit);
          _bustransit_table.insert (
            std::pair<TInt,
                      std::unordered_map<
                        TInt, TInt>> (_dest_node_ID,
                                      _free_shortest_path_tree_bustransit));
        }
    }

  for (auto _d_it : od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it.second);

      for (auto _o_it : od_factory->m_origin_map)
        {
          _origin_node_ID = _o_it.second->m_origin_node->m_node_ID;

          // only process OD pairs with demand
          if (passenger_demand.find (_origin_node_ID) == passenger_demand.end ()
              || passenger_demand.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   == passenger_demand.find (_origin_node_ID)->second.end ())
            {
              continue;
            }

          _free_shortest_path_tree_driving
            = _driving_table.find (_dest_node_ID)->second;
          if (bustransit_graph->IsNode (_dest_node_ID))
            {
              _free_shortest_path_tree_bustransit
                = _bustransit_table.find (_dest_node_ID)->second;
            }

          // driving
          if (od_mode_connectivity.find (_origin_node_ID)
                  ->second.find (_dest_node_ID)
                  ->second.find (driving)
                != od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree_driving,
                                         driving_graph);
              IAssert (_path != nullptr);
              _p_path_driving = new MNM_Passenger_Path_Driving (
                driving, _path, mmdue->m_vot, mmdue->m_early_penalty,
                mmdue->m_late_penalty, mmdue->m_target_time, 1,
                mmdue->m_carpool_cost_multiplier, 0.0, _dest->m_parking_lot,
                mmdue->m_parking_lot_to_destination_walking_time);
              printf ("Adding driving path to path table\n");
              _path = nullptr;
              IAssert (_p_path_driving->m_path != nullptr);
              std::cout << _p_path_driving->info2str ();
              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->find (driving)
                ->second->m_path_vec.push_back (_p_path_driving);
              _p_path_driving->m_path->m_path_ID
                = (int) _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (driving)
                    ->second->m_path_vec.size ()
                  - 1;
            }

          // bus transit
          if (od_mode_connectivity.find (_origin_node_ID)
                  ->second.find (_dest_node_ID)
                  ->second.find (transit)
                != od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree_bustransit,
                                         bustransit_graph);
              IAssert (_path != nullptr);
              _p_path_bus
                = new MNM_Passenger_Path_Bus (transit, _path, mmdue->m_vot,
                                              mmdue->m_early_penalty,
                                              mmdue->m_late_penalty,
                                              mmdue->m_target_time,
                                              mmdue->m_bus_fare,
                                              mmdue->m_bus_inconvenience);
              printf ("Adding bus transit path to path table\n");
              _path = nullptr;
              IAssert (_p_path_bus->m_path != nullptr);
              std::cout << _p_path_bus->info2str ();
              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->find (transit)
                ->second->m_path_vec.push_back (_p_path_bus);
              _p_path_bus->m_path->m_path_ID
                = (int) _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (transit)
                    ->second->m_path_vec.size ()
                  - 1;
            }

          // pnr
          if (od_mode_connectivity.find (_origin_node_ID)
                  ->second.find (_dest_node_ID)
                  ->second.find (pnr)
                != od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              _cur_best_path_tt = DBL_MAX;
              _best_mid_parkinglot = nullptr;
              _pnr_path = nullptr;
              for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
                {
                  _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
                  _path_tt = 0.;
                  _free_shortest_path_tree_pnr
                    = _driving_table.find (_mid_dest_node_ID)->second;
                  _driving_path
                    = MNM::extract_path (_origin_node_ID, _mid_dest_node_ID,
                                         _free_shortest_path_tree_pnr,
                                         driving_graph);
                  IAssert (_driving_path != nullptr);
                  _path_tt
                    += MNM::get_path_tt_snapshot (_driving_path,
                                                  _free_cost_map_driving);

                  _transit_path
                    = MNM::extract_path (_mid_dest_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree_bustransit,
                                         bustransit_graph);
                  IAssert (_transit_path != nullptr);
                  _path_tt
                    += MNM::get_path_tt_snapshot (_transit_path,
                                                  _free_cost_map_bustransit);

                  if (_cur_best_path_tt > _path_tt)
                    {
                      _cur_best_path_tt = _path_tt;
                      _best_mid_parkinglot = _parkinglot;
                      delete _pnr_path;
                      _pnr_path
                        = new MNM_PnR_Path (0, _best_mid_parkinglot->m_ID,
                                            _mid_dest_node_ID, _driving_path,
                                            _transit_path);
                      _driving_path = nullptr;
                      _transit_path = nullptr;
                    }
                  else
                    {
                      delete _driving_path;
                      delete _transit_path;
                    }
                }
              IAssert (_pnr_path != nullptr && _best_mid_parkinglot != nullptr);
              _p_path_pnr
                = new MNM_Passenger_Path_PnR (pnr, _pnr_path, mmdue->m_vot,
                                              mmdue->m_early_penalty,
                                              mmdue->m_late_penalty,
                                              mmdue->m_target_time, 0.0,
                                              _best_mid_parkinglot,
                                              mmdue->m_bus_fare,
                                              mmdue->m_pnr_inconvenience);
              printf ("Adding pnr path to path table\n");
              _pnr_path = nullptr;
              IAssert (_p_path_pnr->m_path != nullptr);
              std::cout << _p_path_pnr->info2str ();
              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->find (pnr)
                ->second->m_path_vec.push_back (_p_path_pnr);
              _p_path_pnr->m_path->m_path_ID
                = (int) _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (pnr)
                    ->second->m_path_vec.size ()
                  - 1;
            }
        }
    }

  for (auto _d_it : _driving_table)
    {
      _d_it.second.clear ();
    }
  _driving_table.clear ();
  for (auto _d_it : _bustransit_table)
    {
      _d_it.second.clear ();
    }
  _bustransit_table.clear ();

  _free_shortest_path_tree_driving.clear ();
  _free_shortest_path_tree_bustransit.clear ();
  _free_shortest_path_tree_pnr.clear ();
  _free_cost_map_driving.clear ();
  _free_cost_map_bustransit.clear ();
  return _path_table;
}

Passenger_Path_Table *
build_existing_passenger_pathset (
  std::vector<MMDue_mode> &mode_vec,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &passenger_demand,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_MM_Due *mmdue)
{
  IAssert (mmdue->m_mmdta != nullptr);
  MNM_Routing_Multimodal_Hybrid *_routing
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdue->m_mmdta->m_routing);
  IAssert (_routing != nullptr);
  Path_Table *driving_path_table = _routing->m_routing_fixed_car->m_path_table;
  PnR_Path_Table *pnr_path_table
    = _routing->m_routing_car_pnr_fixed->m_pnr_path_table;
  Path_Table *bustransit_path_table
    = _routing->m_routing_passenger_fixed->m_bustransit_path_table;

  TInt _origin_node_ID, _dest_node_ID;
  MNM_Destination_Multimodal *_dest;

  // driving
  MNM_Passenger_Path_Driving *_p_path_driving;
  // bus
  MNM_Passenger_Path_Bus *_p_path_bus;
  // pnr
  MNM_Passenger_Path_PnR *_p_path_pnr;

  // create an empty passenger path table, only for connected OD pairs with
  // demand
  auto *_path_table = new Passenger_Path_Table ();
  for (auto _o_it : passenger_demand)
    {
      auto *_new_map_1 = new std::unordered_map<
        TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *> *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<
                    TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *> *>
                    *> (_o_it.first, _new_map_1));
      for (auto _d_it : _o_it.second)
        {
          auto *_new_map_2
            = new std::unordered_map<TInt, MNM_Passenger_Pathset *> ();
          _new_map_1->insert (
            std::pair<TInt, std::unordered_map<TInt, MNM_Passenger_Pathset *>
                              *> (_d_it.first, _new_map_2));
          for (auto _mode : mode_vec)
            {
              if (od_mode_connectivity.find (_o_it.first)
                    ->second.find (_d_it.first)
                    ->second.find (_mode)
                    ->second)
                {
                  auto _pathset = new MNM_Passenger_Pathset (_mode);
                  _new_map_2->insert (
                    std::pair<TInt, MNM_Passenger_Pathset *> (_mode, _pathset));
                }
            }
        }
    }
  // !!! Note path in path_table from m_mmdta is directly stored in
  // passenger_path_table, avoid double deleting driving paths
  if (driving_path_table != nullptr && !driving_path_table->empty ()
      && std::find (mode_vec.begin (), mode_vec.end (), driving)
           != mode_vec.end ())
    {
      for (auto _it : *driving_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  _origin_node_ID = _path->m_node_vec.front ();
                  _dest_node_ID = _path->m_node_vec.back ();
                  if (!(od_mode_connectivity.find (_origin_node_ID)
                          ->second.find (_dest_node_ID)
                          ->second.find (driving)
                          ->second))
                    {
                      continue;
                    }
                  if (_path_table->find (_origin_node_ID) == _path_table->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                           == _path_table->find (_origin_node_ID)
                                ->second->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->find (driving)
                           == _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->end ())
                    {
                      throw std::runtime_error ("Wrong driving path");
                    }
                  _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                    ((MNM_DMDND *) mmdue->m_mmdta->m_node_factory->get_node (
                       _dest_node_ID))
                      ->m_dest);

                  // MNM_Path *_tmp_path = new MNM_Path();
                  // _tmp_path -> m_path_type = _path -> m_path_type;
                  // _tmp_path -> m_node_vec = _path -> m_node_vec;
                  // _tmp_path -> m_link_vec = _path -> m_link_vec;
                  _p_path_driving = new MNM_Passenger_Path_Driving (
                    driving, _path, mmdue->m_vot, mmdue->m_early_penalty,
                    mmdue->m_late_penalty, mmdue->m_target_time, 1,
                    mmdue->m_carpool_cost_multiplier, 0.0, _dest->m_parking_lot,
                    mmdue->m_parking_lot_to_destination_walking_time);
                  printf ("Adding driving path to path table\n");
                  IAssert (_p_path_driving->m_path != nullptr);
                  std::cout << _p_path_driving->info2str ();
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (driving)
                    ->second->m_path_vec.push_back (_p_path_driving);
                  // _p_path_driving -> m_path -> m_path_ID = (int)_path_table
                  // -> find(_origin_node_ID) -> second -> find(_dest_node_ID)
                  // -> second ->
                  //         find(driving) -> second -> m_path_vec.size() - 1;
                  _p_path_driving->m_path->m_path_ID = _path->m_path_ID;
                }
            }
        }
    }
  // bus transit paths
  if (bustransit_path_table != nullptr && !bustransit_path_table->empty ()
      && std::find (mode_vec.begin (), mode_vec.end (), transit)
           != mode_vec.end ())
    {
      for (auto _it : *bustransit_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  _origin_node_ID = _path->m_node_vec.front ();
                  _dest_node_ID = _path->m_node_vec.back ();
                  if (!(od_mode_connectivity.find (_origin_node_ID)
                          ->second.find (_dest_node_ID)
                          ->second.find (transit)
                          ->second))
                    {
                      continue;
                    }
                  if (_path_table->find (_origin_node_ID) == _path_table->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                           == _path_table->find (_origin_node_ID)
                                ->second->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->find (transit)
                           == _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->end ())
                    {
                      throw std::runtime_error ("Wrong bustransit path");
                    }

                  // MNM_Path *_tmp_path = new MNM_Path();
                  // _tmp_path -> m_path_type = _path -> m_path_type;
                  // _tmp_path -> m_node_vec = _path -> m_node_vec;
                  // _tmp_path -> m_link_vec = _path -> m_link_vec;
                  _p_path_bus
                    = new MNM_Passenger_Path_Bus (transit, _path, mmdue->m_vot,
                                                  mmdue->m_early_penalty,
                                                  mmdue->m_late_penalty,
                                                  mmdue->m_target_time,
                                                  mmdue->m_bus_fare,
                                                  mmdue->m_bus_inconvenience);
                  printf ("Adding bus transit path to path table\n");
                  IAssert (_p_path_bus->m_path != nullptr);
                  std::cout << _p_path_bus->info2str ();
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (transit)
                    ->second->m_path_vec.push_back (_p_path_bus);
                  // _p_path_bus -> m_path -> m_path_ID = (int)_path_table ->
                  // find(_origin_node_ID) -> second -> find(_dest_node_ID) ->
                  // second ->
                  //         find(transit) -> second -> m_path_vec.size() - 1;
                  _p_path_bus->m_path->m_path_ID = _path->m_path_ID;
                }
            }
        }
    }
  // pnr paths
  if (pnr_path_table != nullptr && !pnr_path_table->empty ()
      && std::find (mode_vec.begin (), mode_vec.end (), pnr) != mode_vec.end ())
    {
      for (auto _it : *pnr_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              for (MNM_Path *_path : _it_it.second->m_path_vec)
                {
                  auto *_pnr_path = dynamic_cast<MNM_PnR_Path *> (_path);
                  IAssert (_pnr_path != nullptr);
                  _origin_node_ID
                    = _pnr_path->m_driving_path->m_node_vec.front ();
                  _dest_node_ID = _pnr_path->m_transit_path->m_node_vec.back ();
                  if (!(od_mode_connectivity.find (_origin_node_ID)
                          ->second.find (_dest_node_ID)
                          ->second.find (pnr)
                          ->second))
                    {
                      continue;
                    }
                  if (_path_table->find (_origin_node_ID) == _path_table->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                           == _path_table->find (_origin_node_ID)
                                ->second->end ()
                      || _path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->find (pnr)
                           == _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->end ())
                    {
                      throw std::runtime_error ("Wrong pnr path");
                    }

                  // MNM_Path *_tmp_driving_path = new MNM_Path();
                  // _tmp_driving_path -> m_node_vec = _pnr_path ->
                  // m_driving_path -> m_node_vec; _tmp_driving_path ->
                  // m_link_vec = _pnr_path -> m_driving_path -> m_link_vec;
                  // MNM_Path *_tmp_bustransit_path = new MNM_Path();
                  // _tmp_bustransit_path -> m_node_vec = _pnr_path ->
                  // m_transit_path -> m_node_vec; _tmp_bustransit_path ->
                  // m_link_vec = _pnr_path -> m_transit_path -> m_link_vec;
                  // MNM_PnR_Path *_tmp_pnr_path = new MNM_PnR_Path(-1,
                  // _pnr_path -> m_mid_parking_lot_ID,
                  //                                                _pnr_path ->
                  //                                                m_mid_dest_node_ID,
                  //                                                _tmp_driving_path,
                  //                                                _tmp_bustransit_path);
                  // _tmp_pnr_path -> m_path_type = _pnr_path -> m_path_type;
                  _p_path_pnr
                    = new MNM_Passenger_Path_PnR (pnr, _pnr_path, mmdue->m_vot,
                                                  mmdue->m_early_penalty,
                                                  mmdue->m_late_penalty,
                                                  mmdue->m_target_time, 0.0,
                                                  mmdue->m_mmdta
                                                    ->m_parkinglot_factory
                                                    ->get_parking_lot (
                                                      _pnr_path
                                                        ->m_mid_parking_lot_ID),
                                                  mmdue->m_bus_fare,
                                                  mmdue->m_pnr_inconvenience);
                  printf ("Adding pnr path to path table\n");
                  IAssert (_p_path_pnr->m_path != nullptr);
                  std::cout << _p_path_pnr->info2str ();
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->find (pnr)
                    ->second->m_path_vec.push_back (_p_path_pnr);
                  // _p_path_pnr -> m_path -> m_path_ID = (int)_path_table ->
                  // find(_origin_node_ID) -> second -> find(_dest_node_ID) ->
                  // second ->
                  //         find(pnr) -> second -> m_path_vec.size() - 1;
                  _p_path_pnr->m_path->m_path_ID = _pnr_path->m_path_ID;
                }
            }
        }
    }

  return _path_table;
}

Path_Table *
build_shortest_driving_pathset (
  PNEGraph &graph, MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Link_Factory *link_factory, TFlt min_path_length, size_t MaxIter,
  TFlt Mid_Scale, TFlt Heavy_Scale, TInt buffer_length)
{
  // MaxIter: maximum iteration to find alternative shortest path, when MaxIter
  // = 0, just shortest path Mid_Scale and Heavy_Scale are different penalties
  // to the travel cost of links in existing paths
  printf ("build_shortest_driving_pathset, start finding driving_pathset with "
          "number of paths up to %d\n",
          (int) MaxIter + 1);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  Path_Table *_path_table = new Path_Table ();
  std::unordered_map<TInt, MNM_Pathset *> *_new_map = nullptr;
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      _new_map = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<TInt, MNM_Pathset *> *> (_origin_node_ID,
                                                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          if (od_mode_connectivity.find (_origin_node_ID)
                != od_mode_connectivity.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   != od_mode_connectivity.find (_origin_node_ID)->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.find (driving)
                   != od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              MNM_Pathset *_pathset = new MNM_Pathset ();
              _new_map->insert (
                std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
            }
        }
    }

  // printf("111\n");
  std::unordered_map<TInt, TInt> _mid_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map
    = std::unordered_map<TInt, TFlt> ();

  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  MNM_Path *_path;
  // TODO: use link cost instead of link travel time
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               _link_it->second->get_link_tt ()));
    }
  // printf("1111\n");
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph, _free_cost_map,
                                          _free_shortest_path_tree);
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          if (od_mode_connectivity.find (_origin_node_ID)
                != od_mode_connectivity.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   != od_mode_connectivity.find (_origin_node_ID)->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.find (driving)
                   != od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree, graph);
              if (_path != nullptr)
                {
                  _path->allocate_buffer (buffer_length);
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_path);
                }
              else
                {
                  printf ("No driving path found to connect origin node ID %d "
                          "and destination node ID %d\n",
                          _origin_node_ID (), _dest_node_ID ());
                  throw std::runtime_error ("No path connecting OD");
                }
            }
        }
    }
  // printf("22\n");
  _mid_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());
  _heavy_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());

  MNM_Dlink *_link;
  MNM_Path *_path_mid, *_path_heavy;
  TFlt _path_tt;
  size_t _CurIter = 0;
  while (_CurIter < MaxIter)
    {
      printf ("Current trial %d\n", (int) _CurIter);
      for (auto _o_it : *_path_table)
        {
          for (auto _d_it : *_o_it.second)
            {
              for (auto &_path : _d_it.second->m_path_vec)
                {
                  for (auto &_link_ID : _path->m_link_vec)
                    {
                      _link = link_factory->get_link (_link_ID);
                      // TODO: use link cost instead of link travel time
                      _mid_cost_map.find (_link_ID)->second
                        = _link->get_link_tt () * Mid_Scale;
                      _heavy_cost_map.find (_link_ID)->second
                        = _link->get_link_tt () * Heavy_Scale;
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _mid_cost_map,
                                              _mid_shortest_path_tree);
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _heavy_cost_map,
                                              _heavy_shortest_path_tree);
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              if (od_mode_connectivity.find (_origin_node_ID)
                    != od_mode_connectivity.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                         ->second.find (driving)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                       ->second.find (_dest_node_ID)
                       ->second.find (driving)
                       ->second)
                {
                  _path_mid
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _mid_shortest_path_tree, graph);
                  _path_heavy
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _heavy_shortest_path_tree, graph);
                  if (_path_mid != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_path_mid))
                        {
                          _path_tt = MNM::get_path_tt_snapshot (_path_mid,
                                                                _free_cost_map);
                          if (_path_tt > min_path_length)
                            {
                              _path_mid->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (_path_mid);
                            }
                        }
                      else
                        {
                          delete _path_mid;
                        }
                    }
                  if (_path_heavy != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_path_heavy))
                        {
                          _path_tt = MNM::get_path_tt_snapshot (_path_heavy,
                                                                _free_cost_map);
                          if (_path_tt > min_path_length)
                            {
                              _path_heavy->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (_path_heavy);
                            }
                        }
                      else
                        {
                          delete _path_heavy;
                        }
                    }
                }
            }
        }
      _CurIter += 1;
    }

  _mid_shortest_path_tree.clear ();
  _mid_cost_map.clear ();
  _heavy_shortest_path_tree.clear ();
  _heavy_cost_map.clear ();

  _free_cost_map.clear ();
  _free_shortest_path_tree.clear ();

  printf ("build_shortest_driving_pathset, finish finding driving_pathset with "
          "number of paths up to %d\n",
          (int) MaxIter + 1);
  return _path_table;
}

Path_Table *
build_shortest_bustransit_pathset (
  PNEGraph &graph, MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Transit_Link_Factory *link_factory, TFlt min_path_length, size_t MaxIter,
  TFlt Mid_Scale, TFlt Heavy_Scale, TInt buffer_length)
{
  // MaxIter: maximum iteration to find alternative shortest path, when MaxIter
  // = 0, just shortest path Mid_Scale and Heavy_Scale are different penalties
  // to the travel cost of links in existing paths
  printf ("build_shortest_bustransit_pathset, start finding bustransit_pathset "
          "with number of paths up to %d\n",
          (int) MaxIter + 1);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  Path_Table *_path_table = new Path_Table ();
  std::unordered_map<TInt, MNM_Pathset *> *_new_map = nullptr;
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      if (graph->IsNode (_origin_node_ID)
          && graph->GetNI (_origin_node_ID).GetOutDeg () > 0)
        {
          _new_map = new std::unordered_map<TInt, MNM_Pathset *> ();
          _path_table->insert (
            std::pair<
              TInt, std::unordered_map<TInt, MNM_Pathset *> *> (_origin_node_ID,
                                                                _new_map));
        }
      else
        {
          _new_map = nullptr;
          continue;
        }
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          if (graph->IsNode (_dest_node_ID)
              && graph->GetNI (_dest_node_ID).GetInDeg () > 0)
            {
              if (od_mode_connectivity.find (_origin_node_ID)
                    != od_mode_connectivity.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                         ->second.find (transit)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                       ->second.find (_dest_node_ID)
                       ->second.find (transit)
                       ->second)
                {
                  MNM_Pathset *_pathset = new MNM_Pathset ();
                  IAssert (_new_map != nullptr);
                  _new_map->insert (
                    std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
                }
            }
        }
    }

  // printf("111\n");
  std::unordered_map<TInt, TInt> _mid_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map
    = std::unordered_map<TInt, TFlt> ();

  std::unordered_map<TInt, TFlt> _free_cost_map
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  MNM_Path *_path;
  // TODO: use link cost instead of link travel time
  for (auto _link_it : link_factory->m_transit_link_map)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it.first, _link_it.second->m_fftt));
    }
  // printf("1111\n");
  for (auto _d_it = od_factory->m_destination_map.begin ();
       _d_it != od_factory->m_destination_map.end (); _d_it++)
    {
      _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
      if (graph->IsNode (_dest_node_ID)
          && graph->GetNI (_dest_node_ID).GetInDeg () > 0)
        {
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                              _free_cost_map,
                                              _free_shortest_path_tree);
        }
      else
        {
          continue;
        }
      for (auto _o_it = od_factory->m_origin_map.begin ();
           _o_it != od_factory->m_origin_map.end (); _o_it++)
        {
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          if (od_mode_connectivity.find (_origin_node_ID)
                != od_mode_connectivity.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   != od_mode_connectivity.find (_origin_node_ID)->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.find (transit)
                   != od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree, graph);
              if (_path != nullptr)
                {
                  _path->allocate_buffer (buffer_length);
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_path);
                }
              else
                {
                  printf ("No bus transit path found to connect origin node ID "
                          "%d and destination node ID %d\n",
                          _origin_node_ID (), _dest_node_ID ());
                  throw std::runtime_error (
                    "No bus transit path connecting OD");
                }
            }
        }
    }
  // printf("22\n");
  _mid_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());
  _heavy_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());

  MNM_Transit_Link *_link;
  MNM_Path *_path_mid, *_path_heavy;
  TFlt _path_tt;
  size_t _CurIter = 0;
  while (_CurIter < MaxIter)
    {
      printf ("Current trial %d\n", (int) _CurIter);
      for (auto _o_it : *_path_table)
        {
          for (auto _d_it : *_o_it.second)
            {
              for (auto &_path : _d_it.second->m_path_vec)
                {
                  for (auto &_link_ID : _path->m_link_vec)
                    {
                      _link = link_factory->get_transit_link (_link_ID);
                      // TODO: use link cost instead of link travel time
                      _mid_cost_map.find (_link_ID)->second
                        = _link->m_fftt * Mid_Scale;
                      _heavy_cost_map.find (_link_ID)->second
                        = _link->m_fftt * Heavy_Scale;
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          if (graph->IsNode (_dest_node_ID)
              && graph->GetNI (_dest_node_ID).GetInDeg () > 0)
            {
              MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                                  _mid_cost_map,
                                                  _mid_shortest_path_tree);
              MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                                  _heavy_cost_map,
                                                  _heavy_shortest_path_tree);
            }
          else
            {
              continue;
            }
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              if (od_mode_connectivity.find (_origin_node_ID)
                    != od_mode_connectivity.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                         ->second.find (transit)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                       ->second.find (_dest_node_ID)
                       ->second.find (transit)
                       ->second)
                {
                  _path_mid
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _mid_shortest_path_tree, graph);
                  _path_heavy
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _heavy_shortest_path_tree, graph);
                  if (_path_mid != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_path_mid))
                        {
                          _path_tt = MNM::get_path_tt_snapshot (_path_mid,
                                                                _free_cost_map);
                          if (_path_tt > min_path_length)
                            {
                              _path_mid->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (_path_mid);
                            }
                        }
                      else
                        {
                          delete _path_mid;
                        }
                    }
                  if (_path_heavy != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_path_heavy))
                        {
                          _path_tt = MNM::get_path_tt_snapshot (_path_heavy,
                                                                _free_cost_map);
                          if (_path_tt > min_path_length)
                            {
                              _path_heavy->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (_path_heavy);
                            }
                        }
                      else
                        {
                          delete _path_heavy;
                        }
                    }
                }
            }
        }
      _CurIter += 1;
    }
  printf ("build_shortest_bustransit_pathset, finish finding "
          "bustransit_pathset with number of paths up to %d\n",
          (int) MaxIter + 1);
  return _path_table;
}

PnR_Path_Table *
build_shortest_pnr_pathset (
  PNEGraph &driving_graph, PNEGraph &bustransit_graph,
  MNM_OD_Factory *od_factory,
  std::unordered_map<TInt,
                     std::unordered_map<TInt, std::unordered_map<int, bool>>>
    &od_mode_connectivity,
  MNM_Link_Factory *link_factory,
  MNM_Transit_Link_Factory *bus_transitlink_factory, TFlt min_path_length,
  size_t MaxIter, TFlt Mid_Scale, TFlt Heavy_Scale, TInt buffer_length)
{
  printf ("build_shortest_pnr_pathset, start finding pnr_pathset with number "
          "of paths up to %d\n",
          (int) MaxIter + 1);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  MNM_Destination_Multimodal *_dest;
  PnR_Path_Table *_path_table = new PnR_Path_Table ();
  std::unordered_map<TInt, MNM_PnR_Pathset *> *_new_map = nullptr;
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      _new_map = new std::unordered_map<TInt, MNM_PnR_Pathset *> ();
      _path_table->insert (
        std::pair<
          TInt, std::unordered_map<TInt, MNM_PnR_Pathset *> *> (_origin_node_ID,
                                                                _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it->second);
          if (bustransit_graph->IsNode (_dest_node_ID)
              && bustransit_graph->GetNI (_dest_node_ID).GetInDeg () > 0
              && !_dest->m_connected_pnr_parkinglot_vec.empty ())
            {
              if (od_mode_connectivity.find (_origin_node_ID)
                    != od_mode_connectivity.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                         ->second.find (pnr)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                       ->second.find (_dest_node_ID)
                       ->second.find (pnr)
                       ->second)
                {
                  MNM_PnR_Pathset *_pathset = new MNM_PnR_Pathset ();
                  IAssert (_new_map != nullptr);
                  _new_map->insert (
                    std::pair<TInt, MNM_PnR_Pathset *> (_dest_node_ID,
                                                        _pathset));
                }
            }
        }
    }

  // for (auto _o_it : od_mode_connectivity) {
  //     _origin_node_ID = _o_it.first;
  //     _new_map = new std::unordered_map<TInt, MNM_PnR_Pathset *>();
  //     _path_table->insert(std::pair<TInt, std::unordered_map<TInt,
  //     MNM_PnR_Pathset *> *>(_origin_node_ID, _new_map)); for (auto _d_it :
  //     _o_it.second) {
  //         _dest_node_ID = _d_it.first;
  //         if (_d_it.second.find(pnr) != _d_it.second.end() &&
  //         _d_it.second.find(pnr) -> second) {
  //             MNM_PnR_Pathset *_pathset = new MNM_PnR_Pathset();
  //             _new_map->insert(std::pair<TInt, MNM_PnR_Pathset
  //             *>(_dest_node_ID, _pathset));
  //         }
  //     }
  // }

  MNM_Parking_Lot *_best_mid_parkinglot, *_best_mid_parkinglot_mid,
    *_best_mid_parkinglot_heavy;
  TInt _mid_dest_node_ID;

  TFlt _cur_best_path_tt, _path_tt, _cur_best_path_tt_mid,
    _cur_best_path_tt_heavy;

  // <d_node_ID, <node_ID, out_link_ID>>
  std::unordered_map<TInt, std::unordered_map<TInt, TInt>> _driving_table
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();
  std::unordered_map<TInt, std::unordered_map<TInt, TInt>> _bustransit_table
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();
  std::unordered_map<TInt, TInt> _free_shortest_path_tree_driving;
  std::unordered_map<TInt, TInt> _free_shortest_path_tree_bustransit;
  std::unordered_map<TInt, TFlt> _free_cost_map_driving
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TFlt> _free_cost_map_bustransit
    = std::unordered_map<TInt, TFlt> ();

  MNM_Path *_driving_path;
  MNM_Path *_transit_path;
  MNM_PnR_Path *_pnr_path;
  // TODO: use link cost instead of link travel time
  for (auto _link_it : link_factory->m_link_map)
    {
      _free_cost_map_driving.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               _link_it.second->get_link_tt ()));
    }
  for (auto _link_it : bus_transitlink_factory->m_transit_link_map)
    {
      _free_cost_map_bustransit.insert (
        std::pair<TInt, TFlt> (_link_it.first, _link_it.second->m_fftt));
    }

  for (auto _d_it : od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it.second);
      if (bustransit_graph->IsNode (_dest_node_ID))
        {
          _free_shortest_path_tree_driving = std::unordered_map<TInt, TInt> ();
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, driving_graph,
                                              _free_cost_map_driving,
                                              _free_shortest_path_tree_driving);
          _driving_table.insert (
            std::pair<TInt, std::unordered_map<
                              TInt, TInt>> (_dest_node_ID,
                                            _free_shortest_path_tree_driving));

          if (bustransit_graph->GetNI (_dest_node_ID).GetInDeg () > 0
              && !_dest->m_connected_pnr_parkinglot_vec.empty ())
            {
              _free_shortest_path_tree_bustransit
                = std::unordered_map<TInt, TInt> ();
              MNM_Shortest_Path::
                all_to_one_FIFO (_dest_node_ID, bustransit_graph,
                                 _free_cost_map_bustransit,
                                 _free_shortest_path_tree_bustransit);
              _bustransit_table.insert (
                std::pair<TInt,
                          std::unordered_map<
                            TInt, TInt>> (_dest_node_ID,
                                          _free_shortest_path_tree_bustransit));
            }
        }
    }

  std::unordered_map<TInt, TInt> _mid_shortest_path_tree_driving
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map_driving
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree_driving
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map_driving
    = std::unordered_map<TInt, TFlt> ();

  std::unordered_map<TInt, TInt> _mid_shortest_path_tree_bustransit
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _mid_cost_map_bustransit
    = std::unordered_map<TInt, TFlt> ();
  std::unordered_map<TInt, TInt> _heavy_shortest_path_tree_bustransit
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _heavy_cost_map_bustransit
    = std::unordered_map<TInt, TFlt> ();

  for (auto _d_it : od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;
      _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it.second);
      if (!bustransit_graph->IsNode (_dest_node_ID)
          || bustransit_graph->GetNI (_dest_node_ID).GetInDeg () <= 0
          || _dest->m_connected_pnr_parkinglot_vec.empty ())
        {
          continue;
        }
      _free_shortest_path_tree_bustransit
        = _bustransit_table.find (_dest_node_ID)->second;
      for (auto _o_it : od_factory->m_origin_map)
        {
          _origin_node_ID = _o_it.second->m_origin_node->m_node_ID;

          // pnr
          if (od_mode_connectivity.find (_origin_node_ID)
                != od_mode_connectivity.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   != od_mode_connectivity.find (_origin_node_ID)->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                     ->second.find (pnr)
                   != od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.end ()
              && od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              _cur_best_path_tt = DBL_MAX;
              _best_mid_parkinglot = nullptr;
              _pnr_path = nullptr;
              for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
                {
                  _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
                  _path_tt = 0.;
                  _free_shortest_path_tree_driving
                    = _driving_table.find (_mid_dest_node_ID)->second;
                  _driving_path
                    = MNM::extract_path (_origin_node_ID, _mid_dest_node_ID,
                                         _free_shortest_path_tree_driving,
                                         driving_graph);
                  IAssert (_driving_path != nullptr);
                  _path_tt
                    += MNM::get_path_tt_snapshot (_driving_path,
                                                  _free_cost_map_driving);

                  _transit_path
                    = MNM::extract_path (_mid_dest_node_ID, _dest_node_ID,
                                         _free_shortest_path_tree_bustransit,
                                         bustransit_graph);
                  IAssert (_transit_path != nullptr);
                  _path_tt
                    += MNM::get_path_tt_snapshot (_transit_path,
                                                  _free_cost_map_bustransit);

                  if (_cur_best_path_tt > _path_tt)
                    {
                      _cur_best_path_tt = _path_tt;
                      _best_mid_parkinglot = _parkinglot;
                      delete _pnr_path;
                      _pnr_path
                        = new MNM_PnR_Path (0, _best_mid_parkinglot->m_ID,
                                            _mid_dest_node_ID, _driving_path,
                                            _transit_path);
                      _driving_path = nullptr;
                      _transit_path = nullptr;
                    }
                  else
                    {
                      delete _driving_path;
                      delete _transit_path;
                    }
                }
              if (_pnr_path != nullptr && _best_mid_parkinglot != nullptr)
                {
                  _pnr_path->allocate_buffer (buffer_length);
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_pnr_path);
                }
              else
                {
                  printf ("No PnR path found to connect origin node ID %d and "
                          "destination node ID %d\n",
                          _origin_node_ID (), _dest_node_ID ());
                  throw std::runtime_error ("No PnR path connecting OD");
                }
            }
        }
    }

  _mid_cost_map_driving.insert (_free_cost_map_driving.begin (),
                                _free_cost_map_driving.end ());
  _heavy_cost_map_driving.insert (_free_cost_map_driving.begin (),
                                  _free_cost_map_driving.end ());
  _mid_cost_map_bustransit.insert (_free_cost_map_bustransit.begin (),
                                   _free_cost_map_bustransit.end ());
  _heavy_cost_map_bustransit.insert (_free_cost_map_bustransit.begin (),
                                     _free_cost_map_bustransit.end ());

  MNM_Dlink *_link;
  MNM_Transit_Link *_transit_link;
  MNM_Path *_driving_path_mid, *_driving_path_heavy;
  MNM_Path *_bustransit_path_mid, *_bustransit_path_heavy;
  MNM_PnR_Path *_pnr_path_mid, *_pnr_path_heavy;
  size_t _CurIter = 0;
  while (_CurIter < MaxIter)
    {
      printf ("Current trial %d\n", (int) _CurIter);

      for (auto _o_it : *_path_table)
        {
          for (auto _d_it : *_o_it.second)
            {
              for (auto &_path : _d_it.second->m_path_vec)
                {
                  for (auto &_link_ID : dynamic_cast<MNM_PnR_Path *> (_path)
                                          ->m_driving_path->m_link_vec)
                    {
                      _link = link_factory->get_link (_link_ID);
                      // TODO: use link cost instead of link travel time
                      _mid_cost_map_driving.find (_link_ID)->second
                        = _link->get_link_tt () * Mid_Scale;
                      _heavy_cost_map_driving.find (_link_ID)->second
                        = _link->get_link_tt () * Heavy_Scale;
                    }
                  for (auto &_link_ID : dynamic_cast<MNM_PnR_Path *> (_path)
                                          ->m_transit_path->m_link_vec)
                    {
                      _transit_link
                        = bus_transitlink_factory->get_transit_link (_link_ID);
                      // TODO: use link cost instead of link travel time
                      _mid_cost_map_bustransit.find (_link_ID)->second
                        = _transit_link->m_fftt * Mid_Scale;
                      _heavy_cost_map_bustransit.find (_link_ID)->second
                        = _transit_link->m_fftt * Heavy_Scale;
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it->second);
          if (!bustransit_graph->IsNode (_dest_node_ID)
              || bustransit_graph->GetNI (_dest_node_ID).GetInDeg () <= 0
              || _dest->m_connected_pnr_parkinglot_vec.empty ())
            {
              continue;
            }
          MNM_Shortest_Path::
            all_to_one_FIFO (_dest_node_ID, bustransit_graph,
                             _mid_cost_map_bustransit,
                             _mid_shortest_path_tree_bustransit);
          MNM_Shortest_Path::
            all_to_one_FIFO (_dest_node_ID, bustransit_graph,
                             _heavy_cost_map_bustransit,
                             _heavy_shortest_path_tree_bustransit);
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              if (od_mode_connectivity.find (_origin_node_ID)
                    != od_mode_connectivity.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                         ->second.find (_dest_node_ID)
                         ->second.find (transit)
                       != od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.end ()
                  && od_mode_connectivity.find (_origin_node_ID)
                       ->second.find (_dest_node_ID)
                       ->second.find (transit)
                       ->second)
                {
                  _cur_best_path_tt_mid = DBL_MAX;
                  _best_mid_parkinglot_mid = nullptr;
                  _pnr_path_mid = nullptr;

                  _cur_best_path_tt_heavy = DBL_MAX;
                  _best_mid_parkinglot_heavy = nullptr;
                  _pnr_path_heavy = nullptr;

                  for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
                    {
                      _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;

                      MNM_Shortest_Path::
                        all_to_one_FIFO (_mid_dest_node_ID, driving_graph,
                                         _mid_cost_map_driving,
                                         _mid_shortest_path_tree_driving);
                      _driving_path_mid
                        = MNM::extract_path (_origin_node_ID, _mid_dest_node_ID,
                                             _mid_shortest_path_tree_driving,
                                             driving_graph);
                      IAssert (_driving_path_mid != nullptr);
                      _bustransit_path_mid
                        = MNM::extract_path (_mid_dest_node_ID, _dest_node_ID,
                                             _mid_shortest_path_tree_bustransit,
                                             bustransit_graph);
                      IAssert (_bustransit_path_mid != nullptr);

                      MNM_Shortest_Path::
                        all_to_one_FIFO (_mid_dest_node_ID, driving_graph,
                                         _heavy_cost_map_driving,
                                         _heavy_shortest_path_tree_driving);
                      _driving_path_heavy
                        = MNM::extract_path (_origin_node_ID, _mid_dest_node_ID,
                                             _heavy_shortest_path_tree_driving,
                                             driving_graph);
                      IAssert (_driving_path_heavy != nullptr);
                      _bustransit_path_heavy = MNM::
                        extract_path (_mid_dest_node_ID, _dest_node_ID,
                                      _heavy_shortest_path_tree_bustransit,
                                      bustransit_graph);
                      IAssert (_bustransit_path_heavy != nullptr);

                      _path_tt = 0.;
                      _path_tt
                        += MNM::get_path_tt_snapshot (_driving_path_mid,
                                                      _free_cost_map_driving);
                      _path_tt += MNM::
                        get_path_tt_snapshot (_bustransit_path_mid,
                                              _free_cost_map_bustransit);

                      if (_cur_best_path_tt_mid > _path_tt)
                        {
                          _cur_best_path_tt_mid = _path_tt;
                          _best_mid_parkinglot_mid = _parkinglot;
                          delete _pnr_path_mid;
                          _pnr_path_mid
                            = new MNM_PnR_Path (0,
                                                _best_mid_parkinglot_mid->m_ID,
                                                _mid_dest_node_ID,
                                                _driving_path_mid,
                                                _bustransit_path_mid);
                          _driving_path_mid = nullptr;
                          _bustransit_path_mid = nullptr;
                        }
                      else
                        {
                          delete _driving_path_mid;
                          delete _bustransit_path_mid;
                        }

                      _path_tt = 0.;
                      _path_tt
                        += MNM::get_path_tt_snapshot (_driving_path_heavy,
                                                      _free_cost_map_driving);
                      _path_tt += MNM::
                        get_path_tt_snapshot (_bustransit_path_heavy,
                                              _free_cost_map_bustransit);

                      if (_cur_best_path_tt_heavy > _path_tt)
                        {
                          _cur_best_path_tt_heavy = _path_tt;
                          _best_mid_parkinglot_heavy = _parkinglot;
                          delete _pnr_path_heavy;
                          _pnr_path_heavy
                            = new MNM_PnR_Path (0,
                                                _best_mid_parkinglot_heavy
                                                  ->m_ID,
                                                _mid_dest_node_ID,
                                                _driving_path_heavy,
                                                _bustransit_path_heavy);
                          _driving_path_heavy = nullptr;
                          _bustransit_path_heavy = nullptr;
                        }
                      else
                        {
                          delete _driving_path_heavy;
                          delete _bustransit_path_heavy;
                        }
                    }

                  if (_pnr_path_mid != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_pnr_path_mid))
                        {
                          TFlt _tot_tt
                            = MNM::get_path_tt_snapshot (_pnr_path_mid
                                                           ->m_driving_path,
                                                         _free_cost_map_driving)
                              + MNM::get_path_tt_snapshot (
                                _pnr_path_mid->m_transit_path,
                                _free_cost_map_bustransit);
                          if (_tot_tt > min_path_length)
                            {
                              _pnr_path_mid->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (_pnr_path_mid);
                            }
                        }
                      else
                        {
                          delete _pnr_path_mid;
                        }
                    }
                  if (_pnr_path_heavy != nullptr)
                    {
                      if (!_path_table->find (_origin_node_ID)
                             ->second->find (_dest_node_ID)
                             ->second->is_in (_pnr_path_heavy))
                        {
                          TFlt _tot_tt
                            = MNM::get_path_tt_snapshot (_pnr_path_heavy
                                                           ->m_driving_path,
                                                         _free_cost_map_driving)
                              + MNM::get_path_tt_snapshot (
                                _pnr_path_heavy->m_transit_path,
                                _free_cost_map_bustransit);
                          if (_tot_tt > min_path_length)
                            {
                              _pnr_path_heavy->allocate_buffer (buffer_length);
                              _path_table->find (_origin_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->m_path_vec.push_back (
                                  _pnr_path_heavy);
                            }
                        }
                      else
                        {
                          delete _pnr_path_heavy;
                        }
                    }
                }
            }
        }
      _CurIter += 1;
    }

  for (auto _d_it : _driving_table)
    {
      _d_it.second.clear ();
    }
  _driving_table.clear ();
  for (auto _d_it : _bustransit_table)
    {
      _d_it.second.clear ();
    }
  _bustransit_table.clear ();

  _free_shortest_path_tree_driving.clear ();
  _free_shortest_path_tree_bustransit.clear ();
  _free_cost_map_driving.clear ();
  _free_cost_map_bustransit.clear ();

  _mid_shortest_path_tree_driving.clear ();
  _mid_cost_map_driving.clear ();
  _heavy_shortest_path_tree_driving.clear ();
  _heavy_cost_map_driving.clear ();

  _mid_shortest_path_tree_bustransit.clear ();
  _mid_cost_map_bustransit.clear ();
  _heavy_shortest_path_tree_bustransit.clear ();
  _heavy_cost_map_bustransit.clear ();

  printf ("build_shortest_pnr_pathset, finish finding pnr_pathset with number "
          "of paths up to %d\n",
          (int) MaxIter + 1);
  return _path_table;
}

int
allocate_passenger_path_table_buffer (Passenger_Path_Table *path_table,
                                      TInt num)
{
  // num is the length of buffer, m_total_assign_inter
  for (auto _o_it : *path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto _m_it : *_d_it.second)
            {
              for (auto _path : _m_it.second->m_path_vec)
                {
                  _path->allocate_buffer (num);
                }
            }
        }
    }
  return 0;
}

int
generate_init_mode_demand_file (MNM_MM_Due *mmdue,
                                const std::string &file_folder,
                                const std::string &driving_demand_file_name,
                                const std::string &bustransit_demand_file_name,
                                const std::string &pnr_demand_file_name)
{
  std::ofstream _driving_demand_file, _bustransit_demand_file, _pnr_demand_file;
  std::string _str;

  if (mmdue->m_mmdue_config->get_int ("driving") == 1)
    {
      std::string _driving_demand_file_name
        = file_folder + "/" + driving_demand_file_name;

      _driving_demand_file.open (_driving_demand_file_name, std::ofstream::out);
      if (!_driving_demand_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _driving_demand_file");
        }

      _str = "#Origin_ID Destination_ID <car demand by interval> <truck demand "
             "by interval>\n";
      _driving_demand_file << _str;
    }

  if (mmdue->m_mmdue_config->get_int ("transit") == 1)
    {
      std::string _bustransit_demand_file_name
        = file_folder + "/" + bustransit_demand_file_name;

      _bustransit_demand_file.open (_bustransit_demand_file_name,
                                    std::ofstream::out);
      if (!_bustransit_demand_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _bustransit_demand_file");
        }

      _str = "#Origin_ID Destination_ID <passenger demand by interval>\n";
      _bustransit_demand_file << _str;
    }

  if (mmdue->m_mmdue_config->get_int ("pnr") == 1)
    {
      std::string _pnr_demand_file_name
        = file_folder + "/" + pnr_demand_file_name;

      _pnr_demand_file.open (_pnr_demand_file_name, std::ofstream::out);
      if (!_pnr_demand_file.is_open ())
        {
          throw std::runtime_error ("Error happens when open _pnr_demand_file");
        }

      _str = "#OriginID DestID <passenger demand by interval>\n";
      _pnr_demand_file << _str;
    }

  TInt _o_node, _d_node, _o_ID, _d_ID;
  int _total_assign_inter = (int) mmdue->m_total_assign_inter;
  for (auto _o_it : mmdue->m_od_mode_connectivity)
    {
      _o_node = _o_it.first;
      _o_ID = ((MNM_DMOND *) mmdue->m_mmdta->m_node_factory->get_node (_o_node))
                ->m_origin->m_Origin_ID;
      for (auto _d_it : _o_it.second)
        {
          _d_node = _d_it.first;
          _d_ID
            = ((MNM_DMDND *) mmdue->m_mmdta->m_node_factory->get_node (_d_node))
                ->m_dest->m_Dest_ID;
          for (auto _m_it : _d_it.second)
            {
              if (_m_it.first == driving)
                {
                  if (_m_it.second == true)
                    {
                      _str = std::to_string (_o_ID) + " "
                             + std::to_string (_d_ID) + " ";
                      for (int i = 0; i < _total_assign_inter * 2; ++i)
                        {
                          _str += "1 ";
                        }
                      _str.pop_back ();
                      _str += "\n";
                      _driving_demand_file << _str;
                    }
                }
              else if (_m_it.first == transit)
                {
                  if (_m_it.second == true)
                    {
                      _str = std::to_string (_o_ID) + " "
                             + std::to_string (_d_ID) + " ";
                      for (int i = 0; i < _total_assign_inter; ++i)
                        {
                          _str += "1 ";
                        }
                      _str.pop_back ();
                      _str += "\n";
                      _bustransit_demand_file << _str;
                    }
                }
              else if (_m_it.first == pnr)
                {
                  if (_m_it.second == true)
                    {
                      _str = std::to_string (_o_ID) + " "
                             + std::to_string (_d_ID) + " ";
                      for (int i = 0; i < _total_assign_inter; ++i)
                        {
                          _str += "1 ";
                        }
                      _str.pop_back ();
                      _str += "\n";
                      _pnr_demand_file << _str;
                    }
                }
              else
                {
                  throw std::runtime_error (
                    "Error, MNM::generate_init_mode_demand_file(), mode "
                    "not implemented");
                }
            }
        }
    }

  if (mmdue->m_mmdue_config->get_int ("driving") == 1)
    {
      _driving_demand_file.close ();
    }

  if (mmdue->m_mmdue_config->get_int ("transit") == 1)
    {
      _bustransit_demand_file.close ();
    }

  if (mmdue->m_mmdue_config->get_int ("pnr") == 1)
    {
      _pnr_demand_file.close ();
    }

  return 0;
}

int
save_passenger_path_table (Passenger_Path_Table *passenger_path_table,
                           const std::string &file_folder,
                           const std::string &path_file_name,
                           const std::string &buffer_file_name, bool w_buffer,
                           bool w_cost)
{
  std::string _file_name = file_folder + "/" + path_file_name;

  std::ofstream _path_buffer_file;
  if (w_buffer)
    {
      std::string _path_buffer_file_name = file_folder + "/" + buffer_file_name;
      _path_buffer_file.open (_path_buffer_file_name, std::ofstream::out);
      if (!_path_buffer_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _path_buffer_file");
        }
    }

  std::ofstream _path_time_file;
  std::ofstream _path_cost_file;
  std::ofstream _path_disutility_file;
  if (w_cost)
    {
      std::string _path_time_file_name
        = file_folder + "/" + path_file_name + "_time";
      _path_time_file.open (_path_time_file_name, std::ofstream::out);
      if (!_path_time_file.is_open ())
        {
          throw std::runtime_error ("Error happens when open _path_time_file");
        }
      std::string _path_cost_file_name
        = file_folder + "/" + path_file_name + "_cost";
      _path_cost_file.open (_path_cost_file_name, std::ofstream::out);
      if (!_path_cost_file.is_open ())
        {
          throw std::runtime_error ("Error happens when open _path_cost_file");
        }
      std::string _path_disutility_file_name
        = file_folder + "/" + path_file_name + "_disutility";
      _path_disutility_file.open (_path_disutility_file_name,
                                  std::ofstream::out);
      if (!_path_disutility_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _path_disutility_file");
        }
    }

  std::ofstream _path_table_file;
  _path_table_file.open (_file_name, std::ofstream::out);
  if (!_path_table_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _path_table_file");
    }

  // path
  std::string _str;
  TInt _dest_node_ID, _origin_node_ID, _mode, _route_ID, _start_bussstop_ID,
    _end_busstop_ID, _start_walkinglink_ID, _end_walkinglink_ID, _parkinglot_ID;
  _str = std::string (
    "#origin_node_ID dest_node_ID mode mid_dest_node_ID parkinglot_ID "
    "<driving_node_sequence> <bus_transit_link_sequence>\n");
  _path_table_file << _str;
  for (auto _o_it : *passenger_path_table)
    {
      _origin_node_ID = _o_it.first;
      for (auto _d_it : *_o_it.second)
        {
          _dest_node_ID = _d_it.first;
          for (auto _m_it : *_d_it.second)
            {
              _mode = _m_it.first;
              for (auto _path : _m_it.second->m_path_vec)
                {
                  _path_table_file << _path->info2str ();
                  if (w_buffer)
                    {
                      _path_buffer_file << _path->buffer_to_string ();
                    }
                  if (w_cost)
                    {
                      _path_time_file << _path->time_vec_to_string ();
                      _path_cost_file << _path->cost_vec_to_string ();
                      _path_disutility_file
                        << _path->disutility_vec_to_string ();
                    }
                }
            }
        }
    }
  _path_table_file.close ();
  if (w_buffer)
    {
      _path_buffer_file.close ();
    }
  if (w_cost)
    {
      _path_time_file.close ();
      _path_cost_file.close ();
      _path_disutility_file.close ();
    }
  return 0;
}

int
save_driving_path_table (const std::string &file_folder, Path_Table *path_table,
                         const std::string &path_file_name,
                         const std::string &buffer_file_name, bool w_buffer)
{
  std::string _path_table_file_name = file_folder + "/" + path_file_name;

  std::ofstream _path_buffer_file;
  if (w_buffer)
    {
      std::string _path_buffer_file_name = file_folder + "/" + buffer_file_name;
      _path_buffer_file.open (_path_buffer_file_name, std::ofstream::out);
      if (!_path_buffer_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _path_buffer_file");
        }
    }

  std::ofstream _path_table_file;
  _path_table_file.open (_path_table_file_name, std::ofstream::out);
  if (!_path_table_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _path_table_file");
    }

  // path
  for (auto _o_it : *path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto _path : _d_it.second->m_path_vec)
            {
              _path_table_file << _path->node_vec_to_string ();
              // printf("test2\n");
              if (w_buffer)
                {
                  _path_buffer_file << _path->buffer_to_string ();
                }
            }
        }
    }
  _path_table_file.close ();
  if (w_buffer)
    {
      _path_buffer_file.close ();
    }
  return 0;
}

int
save_bustransit_path_table (const std::string &file_folder,
                            Path_Table *path_table,
                            const std::string &path_file_name,
                            const std::string &buffer_file_name, bool w_buffer)
{
  std::string _path_table_file_name = file_folder + "/" + path_file_name;

  std::ofstream _path_buffer_file;
  if (w_buffer)
    {
      std::string _path_buffer_file_name = file_folder + "/" + buffer_file_name;
      _path_buffer_file.open (_path_buffer_file_name, std::ofstream::out);
      if (!_path_buffer_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _path_buffer_file");
        }
    }

  std::ofstream _path_table_file;
  _path_table_file.open (_path_table_file_name, std::ofstream::out);
  if (!_path_table_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _path_table_file");
    }

  // path
  std::string _str;
  TInt _dest_node_ID, _origin_node_ID;
  _str = std::string (
    "#OriginNodeID DestNodeID <transit link IDs in MultiGraph>\n");
  _path_table_file << _str;
  for (auto _o_it : *path_table)
    {
      _origin_node_ID = _o_it.first;
      for (auto _d_it : *_o_it.second)
        {
          _dest_node_ID = _d_it.first;
          for (auto _path : _d_it.second->m_path_vec)
            {
              _str = std::to_string (_origin_node_ID) + " "
                     + std::to_string (_dest_node_ID) + " "
                     + _path->link_vec_to_string ();
              _path_table_file << _str;
              if (w_buffer)
                {
                  _path_buffer_file << _path->buffer_to_string ();
                }
            }
        }
    }
  _path_table_file.close ();
  if (w_buffer)
    {
      _path_buffer_file.close ();
    }
  return 0;
}

int
save_pnr_path_table (const std::string &file_folder, PnR_Path_Table *path_table,
                     const std::string &path_file_name,
                     const std::string &buffer_file_name, bool w_buffer)
{
  std::string _path_table_file_name = file_folder + "/" + path_file_name;

  std::ofstream _path_buffer_file;
  if (w_buffer)
    {
      std::string _path_buffer_file_name = file_folder + "/" + buffer_file_name;
      _path_buffer_file.open (_path_buffer_file_name, std::ofstream::out);
      if (!_path_buffer_file.is_open ())
        {
          throw std::runtime_error (
            "Error happens when open _path_buffer_file");
        }
    }

  std::ofstream _path_table_file;
  _path_table_file.open (_path_table_file_name, std::ofstream::out);
  if (!_path_table_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _path_table_file");
    }

  // path
  std::string _str;
  TInt _dest_node_ID, _origin_node_ID, _mid_parkinglot_ID, _mid_dest_node_ID;
  _str = std::string ("#OriginNodeID DestNodeID MidParkinglotID MidDestNodeID "
                      "<driving node IDs> <transit link IDs in MultiGraph>\n");
  _path_table_file << _str;
  for (auto _o_it : *path_table)
    {
      _origin_node_ID = _o_it.first;
      for (auto _d_it : *_o_it.second)
        {
          _dest_node_ID = _d_it.first;
          for (auto _path : _d_it.second->m_path_vec)
            {
              auto *_pnr_path = dynamic_cast<MNM_PnR_Path *> (_path);
              IAssert (_pnr_path != nullptr);
              _mid_parkinglot_ID = _pnr_path->m_mid_parking_lot_ID;
              _mid_dest_node_ID = _pnr_path->m_mid_dest_node_ID;
              _str = std::to_string (_origin_node_ID) + " "
                     + std::to_string (_dest_node_ID) + " "
                     + std::to_string (_mid_parkinglot_ID) + " "
                     + std::to_string (_mid_dest_node_ID) + " ";
              _str += _pnr_path->m_driving_path->node_vec_to_string ();
              _str.pop_back ();
              _str += " " + _pnr_path->m_transit_path->link_vec_to_string ();
              _path_table_file << _str;
              if (w_buffer)
                {
                  _path_buffer_file << _path->buffer_to_string ();
                }
            }
        }
    }
  _path_table_file.close ();
  if (w_buffer)
    {
      _path_buffer_file.close ();
    }
  return 0;
}

int
get_ID_path_mapping_all_mode (
  std::unordered_map<TInt, std::pair<MNM_Path *, MNM_Passenger_Path_Base *>>
    &dict,
  Path_Table *driving_path_table, Bus_Path_Table *bus_path_table,
  PnR_Path_Table *pnr_path_table, Path_Table *bustransit_path_table,
  Passenger_Path_Table *passenger_path_table, TInt num_path_driving,
  TInt num_path_bustransit, TInt num_path_pnr, TInt num_path_bus)
{
  // NOTE: be careful with the construction of m_mmdue -> m_passenger_path_table
  // in MNM::build_existing_passenger_pathset() the path in passenger_path_table
  // and that in path_table from m_mmdta SHOULD BE in the same memory address
  // because the registered link's cc_tree stored the path in path_table in
  // m_mmdta This is to ensure the correctness of DAR extraction

  // the original path ID is based on the order of paths in the path_file input
  // reorder all path IDs starting from 0
  TInt _path_count = 0;
  MNM_Passenger_Pathset *_p_pathset;
  // MNM_PnR_Path* _pnr_path;

  if (passenger_path_table != nullptr && !passenger_path_table->empty ())
    {
      for (auto _it : *passenger_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              // driving paths
              if (driving_path_table != nullptr && !driving_path_table->empty ()
                  && driving_path_table->find (_it.first)
                       != driving_path_table->end ()
                  && driving_path_table->find (_it.first)->second->find (
                       _it_it.first)
                       != driving_path_table->find (_it.first)->second->end ())
                {
                  _p_pathset = _it_it.second->find (driving)->second;
                  IAssert (_p_pathset->m_path_vec.size ()
                           == driving_path_table->find (_it.first)
                                ->second->find (_it_it.first)
                                ->second->m_path_vec.size ());
                  for (auto *_p_path : _p_pathset->m_path_vec)
                    {
                      _p_path->m_path->m_path_type = driving;
                      IAssert (dict.find (_p_path->m_path->m_path_ID)
                               == dict.end ());
                      dict[_p_path->m_path->m_path_ID]
                        = std::make_pair (_p_path->m_path, _p_path);
                      _path_count += 1;
                    }
                  // for (auto *_path : driving_path_table -> find(_it.first) ->
                  // second -> find(_it_it.first) -> second -> m_path_vec) {
                  //     IAssert(_path -> m_path_type == driving);
                  // }
                }
              // bus transit paths
              if (bustransit_path_table != nullptr
                  && !bustransit_path_table->empty ()
                  && bustransit_path_table->find (_it.first)
                       != bustransit_path_table->end ()
                  && bustransit_path_table->find (_it.first)->second->find (
                       _it_it.first)
                       != bustransit_path_table->find (_it.first)
                            ->second->end ())
                {
                  _p_pathset = _it_it.second->find (transit)->second;
                  IAssert (_p_pathset->m_path_vec.size ()
                           == bustransit_path_table->find (_it.first)
                                ->second->find (_it_it.first)
                                ->second->m_path_vec.size ());
                  for (auto *_p_path : _p_pathset->m_path_vec)
                    {
                      _p_path->m_path->m_path_ID += num_path_driving;
                      _p_path->m_path->m_path_type = transit;
                      IAssert (dict.find (_p_path->m_path->m_path_ID)
                               == dict.end ());
                      dict[_p_path->m_path->m_path_ID]
                        = std::make_pair (_p_path->m_path, _p_path);
                      _path_count += 1;
                    }
                  // for (auto *_path : bustransit_path_table -> find(_it.first)
                  // -> second -> find(_it_it.first) -> second -> m_path_vec) {
                  //     IAssert(_path -> m_path_type == transit);
                  // }
                }
              // pnr paths
              if (pnr_path_table != nullptr && !pnr_path_table->empty ()
                  && pnr_path_table->find (_it.first) != pnr_path_table->end ()
                  && pnr_path_table->find (_it.first)->second->find (
                       _it_it.first)
                       != pnr_path_table->find (_it.first)->second->end ())
                {
                  _p_pathset = _it_it.second->find (pnr)->second;
                  IAssert (_p_pathset->m_path_vec.size ()
                           == pnr_path_table->find (_it.first)
                                ->second->find (_it_it.first)
                                ->second->m_path_vec.size ());
                  for (auto *_p_path : _p_pathset->m_path_vec)
                    {
                      IAssert (dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)
                               != nullptr);
                      // _pnr_path =
                      // dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) ->
                      // m_path; std::cout <<
                      // std::to_string(dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path)
                      // -> m_path -> m_path_ID) << "\n";
                      dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)
                        ->m_path->m_path_ID
                        += num_path_driving + num_path_bustransit;
                      dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)
                        ->m_path->m_path_type
                        = pnr;
                      // std::cout <<
                      // std::to_string(dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path)
                      // -> m_path -> m_path_ID) << "\n";
                      IAssert (
                        dict.find (
                          dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)
                            ->m_path->m_path_ID)
                        == dict.end ());
                      dict[dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)
                             ->m_path->m_path_ID]
                        = std::make_pair (dynamic_cast<
                                            MNM_Passenger_Path_PnR *> (_p_path)
                                            ->m_path,
                                          _p_path);
                      _path_count += 1;
                    }
                  // for (auto *_path : pnr_path_table -> find(_it.first) ->
                  // second -> find(_it_it.first) -> second -> m_path_vec) {
                  //     IAssert(_path -> m_path_type == pnr);
                  // }
                }
            }
        }
    }

  // bus paths
  if (bus_path_table != nullptr && !bus_path_table->empty ())
    {
      for (auto _it : *bus_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              for (auto _it_it_it : *(_it_it.second))
                {
                  MNM_Path *_path = _it_it_it.second;
                  _path->m_path_ID
                    += num_path_driving + num_path_bustransit + num_path_pnr;
                  _path->m_path_type = bus_route;
                  IAssert (dict.find (_path->m_path_ID) == dict.end ());
                  dict[_path->m_path_ID] = std::make_pair (_path, nullptr);
                  _path_count += 1;
                }
            }
        }
    }

  if (_path_count
      != num_path_driving + num_path_bustransit + num_path_pnr + num_path_bus)
    {
      throw std::runtime_error (
        "path count is wrong in MNM::get_ID_path_mapping_all_mode()");
    }
  return _path_count;
}

}

/******************************************************************************************************************
*******************************************************************************************************************
                                                                                Multimodal DUE
*******************************************************************************************************************
******************************************************************************************************************/
MNM_MM_Due::MNM_MM_Due (const std::string &file_folder)
{
  m_file_folder = file_folder;

  m_mmdta_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DTA");
  IAssert (m_mmdta_config->get_string ("routing_type")
             == "Multimodal_DUE_FixedPath"
           || m_mmdta_config->get_string ("routing_type")
                == "Multimodal_DUE_ColumnGeneration"
           || m_mmdta_config->get_string ("routing_type") == "Multimodal_Hybrid"
           || m_mmdta_config->get_string ("routing_type")
                == "Multimodal_Hybrid_ColumnGeneration");   // Multimodal_Hybrid
                                                            // only for DODE
  IAssert (m_mmdta_config->get_int ("total_interval") > 0); // sufficiently long

  m_od_mode_connectivity = std::unordered_map<
    TInt, std::unordered_map<TInt, std::unordered_map<int, bool>>> ();
  m_unit_time = m_mmdta_config->get_float ("unit_time"); // 5s
  m_total_loading_inter = m_mmdta_config->get_int ("total_interval");
  m_total_assign_inter = m_mmdta_config->get_int ("max_interval");
  if (m_total_loading_inter <= 0)
    m_total_loading_inter
      = m_total_assign_inter * m_mmdta_config->get_int ("assign_frq");

  m_mmdue_config = new MNM_ConfReader (m_file_folder + "/config.conf", "MMDUE");

  m_mode_share = std::unordered_map<int, TFlt> ();
  m_mode_vec = std::vector<MMDue_mode> ();
  if (m_mmdue_config->get_int ("driving") == 1)
    {
      m_mode_vec.push_back (driving);
      m_mode_share.insert (std::pair<int, TFlt> (driving, 0.));
    }
  if (m_mmdue_config->get_int ("transit") == 1)
    {
      m_mode_vec.push_back (transit);
      m_mode_share.insert (std::pair<int, TFlt> (transit, 0.));
    }
  if (m_mmdue_config->get_int ("pnr") == 1)
    {
      m_mode_vec.push_back (pnr);
      m_mode_share.insert (std::pair<int, TFlt> (pnr, 0.));
    }
  m_num_modes = (int) m_mode_vec.size ();

  // single level MMDUE
  m_alpha1_driving = m_mmdue_config->get_float ("alpha1_driving");
  m_alpha1_transit = m_mmdue_config->get_float ("alpha1_transit");
  m_alpha1_pnr = m_mmdue_config->get_float ("alpha1_pnr");
  m_beta1 = m_mmdue_config->get_float ("beta1");

  // money / hour -> money / interval
  // the unit of m_vot here is different from that of m_vot in adaptive routing
  // (money / second)
  m_vot = m_mmdue_config->get_float ("vot") / 3600. * m_unit_time;
  m_early_penalty
    = m_mmdue_config->get_float ("early_penalty") / 3600. * m_unit_time;
  m_late_penalty
    = m_mmdue_config->get_float ("late_penalty") / 3600. * m_unit_time;
  // minute -> interval
  m_target_time = m_mmdue_config->get_float ("target_time") * 60 / m_unit_time;

  m_parking_lot_to_destination_walking_time = m_mmdue_config->get_float (
    "parking_lot_to_destination_walking_time"); // seconds
  m_carpool_cost_multiplier
    = m_mmdue_config->get_float ("carpool_cost_multiplier");
  m_bus_fare = m_mmdue_config->get_float ("bus_fare");
  m_metro_fare = m_mmdue_config->get_float ("metro_fare");
  m_pnr_inconvenience = m_mmdue_config->get_float ("pnr_inconvenience");
  m_bus_inconvenience = m_mmdue_config->get_float ("bus_inconvenience");

  m_max_iter = m_mmdue_config->get_int ("max_iter");
  m_step_size = m_mmdue_config->get_float ("step_size"); // 0.01;  // MSA

  // <O_node_ID, <D_node_ID, time-varying demand with length of
  // m_total_assign_inter>>
  m_passenger_demand
    = std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> ();
  m_passenger_path_table = nullptr; // every passenger path has m_buffer
                                    // indicating the time-varying path flow

  // for DNL, from passenger_path_table, to update external mmdta
  m_driving_path_table = nullptr;
  // for background truck
  m_truck_path_table = nullptr;
  // for PnR
  m_pnr_path_table = nullptr;
  // for bus riding
  m_bustransit_path_table = nullptr;
  // for bus
  m_bus_path_table = nullptr;

  // time-varying link tt
  m_link_tt_map = std::unordered_map<TInt, TFlt *> ();
  m_link_tt_map_truck = std::unordered_map<TInt, TFlt *> ();
  m_transitlink_tt_map = std::unordered_map<TInt, TFlt *> ();

  // time-varying link cost
  m_link_cost_map = std::unordered_map<TInt, TFlt *> ();
  m_link_cost_map_truck = std::unordered_map<TInt, TFlt *> ();
  m_transitlink_cost_map = std::unordered_map<TInt, TFlt *> ();

  m_link_congested_car = std::unordered_map<TInt, bool *> ();
  m_link_congested_truck = std::unordered_map<TInt, bool *> ();
  m_transitlink_congested_passenger = std::unordered_map<TInt, bool *> ();

  m_queue_dissipated_time_car = std::unordered_map<TInt, int *> ();
  m_queue_dissipated_time_truck = std::unordered_map<TInt, int *> ();
  m_queue_dissipated_time_passenger = std::unordered_map<TInt, int *> ();

  m_driving_link_tt_map_snapshot = std::unordered_map<TInt, TFlt> ();
  m_bustransit_link_tt_map_snapshot = std::unordered_map<TInt, TFlt> ();

  m_driving_link_cost_map_snapshot = std::unordered_map<TInt, TFlt> ();
  m_bustransit_link_cost_map_snapshot = std::unordered_map<TInt, TFlt> ();

  m_driving_table_snapshot
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();
  m_bustransit_table_snapshot
    = std::unordered_map<TInt, std::unordered_map<TInt, TInt>> ();

  m_mmdta = nullptr;
}

MNM_MM_Due::~MNM_MM_Due ()
{
  m_mode_vec.clear ();
  m_mode_share.clear ();

  for (auto _o_it : m_od_mode_connectivity)
    {
      for (auto _d_it : _o_it.second)
        {
          _d_it.second.clear ();
        }
      _o_it.second.clear ();
    }
  m_od_mode_connectivity.clear ();

  for (auto _tt_it : m_link_tt_map)
    {
      delete _tt_it.second;
    }
  m_link_tt_map.clear ();

  for (auto _tt_it : m_link_tt_map_truck)
    {
      delete _tt_it.second;
    }
  m_link_tt_map_truck.clear ();

  for (auto _tt_it : m_transitlink_tt_map)
    {
      delete _tt_it.second;
    }
  m_transitlink_tt_map.clear ();

  for (auto _cost_it : m_link_cost_map)
    {
      delete _cost_it.second;
    }
  m_link_cost_map.clear ();

  for (auto _cost_it : m_link_cost_map_truck)
    {
      delete _cost_it.second;
    }
  m_link_cost_map_truck.clear ();

  for (auto _cost_it : m_transitlink_cost_map)
    {
      delete _cost_it.second;
    }
  m_transitlink_cost_map.clear ();

  for (auto _it : m_link_congested_car)
    {
      delete _it.second;
    }
  m_link_congested_car.clear ();

  for (auto _it : m_link_congested_truck)
    {
      delete _it.second;
    }
  m_link_congested_truck.clear ();

  for (auto _it : m_transitlink_congested_passenger)
    {
      delete _it.second;
    }
  m_transitlink_congested_passenger.clear ();

  for (auto _it : m_queue_dissipated_time_car)
    {
      delete _it.second;
    }
  m_queue_dissipated_time_car.clear ();

  for (auto _it : m_queue_dissipated_time_truck)
    {
      delete _it.second;
    }
  m_queue_dissipated_time_truck.clear ();

  for (auto _it : m_queue_dissipated_time_passenger)
    {
      delete _it.second;
    }
  m_queue_dissipated_time_passenger.clear ();

  m_driving_link_tt_map_snapshot.clear ();
  m_bustransit_link_tt_map_snapshot.clear ();

  m_driving_link_cost_map_snapshot.clear ();
  m_bustransit_link_cost_map_snapshot.clear ();

  for (auto _d_it : m_driving_table_snapshot)
    {
      _d_it.second.clear ();
    }
  m_driving_table_snapshot.clear ();
  for (auto _d_it : m_bustransit_table_snapshot)
    {
      _d_it.second.clear ();
    }
  m_bustransit_table_snapshot.clear ();

  for (auto _it : m_passenger_demand)
    {
      for (auto _it_it : _it.second)
        {
          free (_it_it.second);
        }
      _it.second.clear ();
    }
  m_passenger_demand.clear ();

  // those paths in these path tables are also stored in m_passenger_path_table
  // avoid double deleting them
  MNM_Routing_Multimodal_Hybrid *_routing
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing);
  IAssert (_routing != nullptr);
  Path_Table *driving_path_table = _routing->m_routing_fixed_car->m_path_table;
  PnR_Path_Table *pnr_path_table
    = _routing->m_routing_car_pnr_fixed->m_pnr_path_table;
  Path_Table *bustransit_path_table
    = _routing->m_routing_passenger_fixed->m_bustransit_path_table;

  if (driving_path_table != nullptr && !driving_path_table->empty ())
    {
      for (auto _it : *driving_path_table)
        {
          for (auto _it_it : *(_it.second))
            {
              _it_it.second->m_path_vec.clear ();
            }
        }
    }
  if (pnr_path_table != nullptr && !pnr_path_table->empty ())
    {
      for (auto _it : *pnr_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              _it_it.second->m_path_vec.clear ();
            }
        }
    }
  if (bustransit_path_table != nullptr && !bustransit_path_table->empty ())
    {
      for (auto _it : *bustransit_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              _it_it.second->m_path_vec.clear ();
            }
        }
    }

  for (auto _o_it : *m_passenger_path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          for (auto _m_it : *_d_it.second)
            {
              if (_m_it.second != nullptr)
                {
                  delete _m_it.second;
                }
            }
          _d_it.second->clear ();
          delete _d_it.second;
        }
      _o_it.second->clear ();
      delete _o_it.second;
    }
  m_passenger_path_table->clear ();
  delete m_passenger_path_table;

  delete m_mmdta;

  if (m_mmdta_config != nullptr)
    delete m_mmdta_config;
  if (m_mmdue_config != nullptr)
    delete m_mmdue_config;
}

int
MNM_MM_Due::check_od_mode_connectivity ()
{
  load_od_mode_connectivity ();
  if (!m_od_mode_connectivity.empty ())
    {
      return 0;
    }

  TInt _origin_node_ID, _dest_node_ID, _mid_dest_node_ID;
  MNM_Destination_Multimodal *_dest;
  bool _connected, _origin_node_in_transit_graph, _dest_node_in_transit_graph;

  // driving
  std::unordered_map<TInt, TInt> _shortest_path_tree_driving
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _cost_map_driving
    = std::unordered_map<TInt, TFlt> ();
  for (auto _map_it : m_mmdta->m_link_factory->m_link_map)
    {
      _cost_map_driving.insert (
        std::pair<TInt, TFlt> (_map_it.first, TFlt (1)));
    }
  // bus transit
  std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TFlt> _cost_map_bustransit
    = std::unordered_map<TInt, TFlt> ();
  for (auto _map_it : m_mmdta->m_transitlink_factory->m_transit_link_map)
    {
      _cost_map_bustransit.insert (
        std::pair<TInt, TFlt> (_map_it.first, TFlt (1)));
    }
  // pnr
  std::unordered_map<TInt, TInt> _shortest_path_tree_pnr
    = std::unordered_map<TInt, TInt> ();

  for (auto _d_it : m_mmdta->m_od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;
      _dest_node_in_transit_graph
        = m_mmdta->m_bus_transit_graph->IsNode (_dest_node_ID);
      _shortest_path_tree_driving.clear ();
      _shortest_path_tree_bustransit.clear ();
      _shortest_path_tree_pnr.clear ();
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, m_mmdta->m_graph,
                                          _cost_map_driving,
                                          _shortest_path_tree_driving);
      if (_dest_node_in_transit_graph)
        {
          MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID,
                                              m_mmdta->m_bus_transit_graph,
                                              _cost_map_bustransit,
                                              _shortest_path_tree_bustransit);
        }
      for (auto _o_it : m_mmdta->m_od_factory->m_origin_map)
        {
          _origin_node_ID = _o_it.second->m_origin_node->m_node_ID;
          _origin_node_in_transit_graph
            = m_mmdta->m_bus_transit_graph->IsNode (_origin_node_ID);

          if (m_passenger_demand.find (_origin_node_ID)
                != m_passenger_demand.end ()
              && m_passenger_demand.find (_origin_node_ID)
                     ->second.find (_dest_node_ID)
                   != m_passenger_demand.find (_origin_node_ID)->second.end ())
            {
              if (m_od_mode_connectivity.find (_origin_node_ID)
                  == m_od_mode_connectivity.end ())
                {
                  m_od_mode_connectivity.insert (
                    std::pair<
                      TInt, std::unordered_map<
                              TInt, std::unordered_map<
                                      int, bool>>> (_origin_node_ID,
                                                    std::unordered_map<
                                                      TInt, std::unordered_map<
                                                              int, bool>> ()));
                }
              if (m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                  == m_od_mode_connectivity.find (_origin_node_ID)
                       ->second.end ())
                {
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.insert (
                      std::pair<
                        TInt, std::unordered_map<
                                int, bool>> (_dest_node_ID,
                                             std::unordered_map<int, bool> ()));
                }
              for (auto _mode : m_mode_vec)
                {
                  if (m_od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.find (_mode)
                      == m_od_mode_connectivity.find (_origin_node_ID)
                           ->second.find (_dest_node_ID)
                           ->second.end ())
                    {
                      m_od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.insert (std::pair<int, bool> (_mode, false));
                    }
                }

              _connected = false;
              // driving
              if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                    != m_mode_vec.end ()
                  && _shortest_path_tree_driving.find (_origin_node_ID)->second
                       > 0)
                {
                  printf ("Driving from origin node %d to destination node %d "
                          "is possible!\n",
                          (int) _origin_node_ID, (int) _dest_node_ID);
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second.find (driving)
                    ->second
                    = true;
                  _connected = true;
                }
              // bus transit
              if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                    != m_mode_vec.end ()
                  && _dest_node_in_transit_graph
                  && _origin_node_in_transit_graph
                  && _shortest_path_tree_bustransit.find (_origin_node_ID)
                         ->second
                       > 0)
                {
                  printf ("Direct bus transit from origin node %d to "
                          "destination node %d is possible!\n",
                          (int) _origin_node_ID, (int) _dest_node_ID);
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second.find (transit)
                    ->second
                    = true;
                  _connected = true;
                }
              // pnr
              _dest = dynamic_cast<MNM_Destination_Multimodal *> (_d_it.second);
              if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                    != m_mode_vec.end ()
                  && _dest_node_in_transit_graph
                  && !_dest->m_connected_pnr_parkinglot_vec.empty ())
                {
                  for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
                    {
                      _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
                      MNM_Shortest_Path::
                        all_to_one_FIFO (_mid_dest_node_ID, m_mmdta->m_graph,
                                         _cost_map_driving,
                                         _shortest_path_tree_pnr);
                      if (_shortest_path_tree_pnr.find (_origin_node_ID)->second
                          > 0)
                        {
                          printf ("PnR from origin node %d to destination node "
                                  "%d is possible!\n",
                                  (int) _origin_node_ID, (int) _dest_node_ID);
                          m_od_mode_connectivity.find (_origin_node_ID)
                            ->second.find (_dest_node_ID)
                            ->second.find (pnr)
                            ->second
                            = true;
                          _connected = true;
                          break;
                        }
                    }
                }
              // not a single mode is available
              if (!_connected)
                {
                  printf ("No mode is available from origin node %d to "
                          "destination node %d!\n",
                          (int) _origin_node_ID, (int) _dest_node_ID);
                  throw std::runtime_error ("No mode available for OD");
                }
            }
        }
    }
  _shortest_path_tree_driving.clear ();
  _shortest_path_tree_bustransit.clear ();
  _shortest_path_tree_pnr.clear ();
  _cost_map_driving.clear ();
  _cost_map_bustransit.clear ();

  save_od_mode_connectivity ();
  return 0;
}

int
MNM_MM_Due::save_od_mode_connectivity (
  const std::string &connectivity_file_name)
{
  std::string _connectivity_file_name
    = m_file_folder + "/" + connectivity_file_name;

  std::ofstream _connectivity_file;
  std::string _str;

  _connectivity_file.open (_connectivity_file_name, std::ofstream::out);
  if (!_connectivity_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _connectivity_file");
    }

  _str = "#OriginNodeID DestNodeID Driving BusTransit PnR\n";
  _connectivity_file << _str;

  // O_node, D_node, driving, bustransit, pnr
  for (const auto &_o_it : m_od_mode_connectivity)
    {
      for (const auto &_d_it : _o_it.second)
        {
          _str = std::to_string (_o_it.first) + " ";
          _str += std::to_string (_d_it.first) + " ";
          _str
            += std::to_string ((int) _d_it.second.find (driving)->second) + " ";
          _str
            += std::to_string ((int) _d_it.second.find (transit)->second) + " ";
          _str += std::to_string ((int) _d_it.second.find (pnr)->second) + "\n";
          _connectivity_file << _str;
        }
    }
  _connectivity_file.close ();
  return 0;
}

int
MNM_MM_Due::load_od_mode_connectivity (
  const std::string &connectivity_file_name)
{
  std::string _connectivity_file_name
    = m_file_folder + "/" + connectivity_file_name;

  std::ifstream _connectivity_file;
  std::string _line;
  std::vector<std::string> _words;
  TInt _origin_node_ID, _dest_node_ID;
  int _num_OD = m_mmdta_config->get_int ("OD_pair_passenger");

  _connectivity_file.open (_connectivity_file_name, std::ios::in);
  if (_connectivity_file.is_open ())
    {
      std::getline (_connectivity_file, _line); // skip the first line
      for (int i = 0; i < _num_OD; ++i)
        {
          std::getline (_connectivity_file, _line);
          _words = MNM_IO::split (MNM_IO::trim (_line), ' ');
          if ((int) _words.size () == 5)
            {
              _origin_node_ID = TInt (std::stoi (_words[0]));
              _dest_node_ID = TInt (std::stoi (_words[1]));

              if (m_od_mode_connectivity.find (_origin_node_ID)
                  == m_od_mode_connectivity.end ())
                {
                  m_od_mode_connectivity.insert (
                    std::pair<
                      TInt, std::unordered_map<
                              TInt, std::unordered_map<
                                      int, bool>>> (_origin_node_ID,
                                                    std::unordered_map<
                                                      TInt, std::unordered_map<
                                                              int, bool>> ()));
                }
              if (m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                  == m_od_mode_connectivity.find (_origin_node_ID)
                       ->second.end ())
                {
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.insert (
                      std::pair<
                        TInt, std::unordered_map<
                                int, bool>> (_dest_node_ID,
                                             std::unordered_map<int, bool> ()));
                }
              for (auto _mode : m_mode_vec)
                {
                  if (m_od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.find (_mode)
                      == m_od_mode_connectivity.find (_origin_node_ID)
                           ->second.find (_dest_node_ID)
                           ->second.end ())
                    {
                      m_od_mode_connectivity.find (_origin_node_ID)
                        ->second.find (_dest_node_ID)
                        ->second.insert (std::pair<int, bool> (_mode, false));
                    }
                }

              if (TInt (std::stoi (_words[2])) == 1)
                {
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second.find (driving)
                    ->second
                    = true;
                }
              if (TInt (std::stoi (_words[3])) == 1)
                {
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second.find (transit)
                    ->second
                    = true;
                }
              if (TInt (std::stoi (_words[4])) == 1)
                {
                  m_od_mode_connectivity.find (_origin_node_ID)
                    ->second.find (_dest_node_ID)
                    ->second.find (pnr)
                    ->second
                    = true;
                }
            }
          else
            {
              throw std::runtime_error ("number of modes is not correct");
            }
        }
    }
  else
    {
      printf ("cannot open _connectivity_file\n");
      return -1;
    }

  _connectivity_file.close ();
  return 0;
}

int
MNM_MM_Due::initialize ()
{
  // path_table and path_table_buffer files exist for background truck traffic,
  // car is zero m_mmdta is used mainly for reading files, an external mmdta
  // will used for actual DNL
  m_mmdta = new MNM_Dta_Multimodal (m_file_folder);
  m_mmdta->build_from_files ();
  m_mmdta->hook_up_node_and_link ();
  m_mmdta->find_connected_pnr_parkinglot_for_destination ();
  // m_mmdta -> is_ok();  // do this check once if it is time-consuming

  for (auto _link_it : m_mmdta->m_link_factory->m_link_map)
    {
      m_link_tt_map[_link_it.first] = new TFlt[m_total_loading_inter];
      m_link_tt_map_truck[_link_it.first] = new TFlt[m_total_loading_inter];
      m_link_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
      m_link_cost_map_truck[_link_it.first] = new TFlt[m_total_loading_inter];
      // m_link_congested_car[_link_it.first] = new bool[m_total_loading_inter];
      // m_link_congested_truck[_link_it.first] = new
      // bool[m_total_loading_inter];
      // m_queue_dissipated_time_car[_link_it.first] = new
      // int[m_total_loading_inter];
      // m_queue_dissipated_time_truck[_link_it.first] = new
      // int[m_total_loading_inter];
    }

  for (auto _link_it : m_mmdta->m_transitlink_factory->m_transit_link_map)
    {
      m_transitlink_tt_map[_link_it.first] = new TFlt[m_total_loading_inter];
      m_transitlink_cost_map[_link_it.first] = new TFlt[m_total_loading_inter];
      // m_transitlink_congested_passenger[_link_it.first] = new
      // bool[m_total_loading_inter];
      // m_queue_dissipated_time_passenger[_link_it.first] = new
      // int[m_total_loading_inter];
    }
  // for DODE, initialize as 0 in the input file
  MNM_IO_Multimodal::build_passenger_demand (m_file_folder, m_mmdta_config,
                                             m_mmdta->m_od_factory,
                                             m_passenger_demand,
                                             "passenger_demand");
  // construct m_od_mode_connectivity
  check_od_mode_connectivity ();

  // m_truck_path_table and m_bus_path_table will be fixed after initialization
  m_driving_path_table = nullptr; // new Path_Table();
  // m_truck_path_table records the background truck traffic, the car part is
  // zeros, first half part of m_buffer
  m_truck_path_table
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
        ->m_routing_fixed_truck->m_path_table;
  m_pnr_path_table = nullptr;        // new PnR_Path_Table();
  m_bustransit_path_table = nullptr; // new Path_Table();
  m_bus_path_table
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
        ->m_routing_bus_fixed->m_bus_path_table;

  printf ("finish initialization\n");
  return 0;
}

int
MNM_MM_Due::init_passenger_path_table ()
{
  // init shortest path
  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_Hybrid_ColumnGeneration")
    {
      // only one path for one OD for each mode
      m_passenger_path_table
        = MNM::build_shortest_passenger_pathset (m_mode_vec, m_passenger_demand,
                                                 m_od_mode_connectivity, this,
                                                 m_mmdta->m_graph,
                                                 m_mmdta->m_bus_transit_graph,
                                                 m_mmdta->m_od_factory,
                                                 m_mmdta->m_link_factory,
                                                 m_mmdta->m_transitlink_factory,
                                                 m_mmdta->m_busstop_factory);
    }
  else if (m_mmdta_config->get_string ("routing_type")
           == "Multimodal_DUE_ColumnGeneration")
    {
      auto *_tmp_conf
        = new MNM_ConfReader (m_file_folder + "/config.conf", "FIXED");
      if (_tmp_conf->get_int ("num_driving_path") > 0
          && _tmp_conf->get_int ("num_pnr_path") >= 0
          && _tmp_conf->get_int ("num_bustransit_path") >= 0)
        {
          m_passenger_path_table
            = MNM::build_existing_passenger_pathset (m_mode_vec,
                                                     m_passenger_demand,
                                                     m_od_mode_connectivity,
                                                     this);
        }
      else
        {
          m_passenger_path_table = MNM::
            build_shortest_passenger_pathset (m_mode_vec, m_passenger_demand,
                                              m_od_mode_connectivity, this,
                                              m_mmdta->m_graph,
                                              m_mmdta->m_bus_transit_graph,
                                              m_mmdta->m_od_factory,
                                              m_mmdta->m_link_factory,
                                              m_mmdta->m_transitlink_factory,
                                              m_mmdta->m_busstop_factory);
        }
      delete _tmp_conf;
    }
  else
    {
      m_passenger_path_table
        = MNM::build_existing_passenger_pathset (m_mode_vec, m_passenger_demand,
                                                 m_od_mode_connectivity, this);
    }

  MNM::allocate_passenger_path_table_buffer (m_passenger_path_table,
                                             m_total_assign_inter);

  printf ("finish init_passenger_path_table\n");
  return 0;
}

// spread the OD demand evenly over the initial paths for each mode
int
MNM_MM_Due::init_passenger_path_flow ()
{
  TFlt _num_mode, _len, _dmd;
  TInt _o_node_ID, _d_node_ID, _mode;
  for (auto _o_it : *m_passenger_path_table)
    {
      _o_node_ID = _o_it.first;
      for (auto _d_it : *(_o_it.second))
        {
          _d_node_ID = _d_it.first;
          // only account for driving since bus may not be available during some
          // time
          _num_mode = TFlt (_d_it.second->size ());
          // _num_mode = 1;
          if (_num_mode == TFlt (0))
            {
              printf ("No available mode for origin node %d to destination "
                      "node %d\n",
                      (int) _o_node_ID, (int) _d_node_ID);
              throw std::runtime_error ("No mode available for OD");
            }
          for (auto _m_it : *(_d_it.second))
            {
              _mode = _m_it.first;
              // if (_mode != driving) {
              //     continue;
              // }
              _len = TFlt (_m_it.second->m_path_vec.size ());
              if (_len == TFlt (0))
                {
                  throw std::runtime_error ("No available paths");
                }

              // uniform
              // for (MNM_Passenger_Path_Base *_path : _m_it.second->m_path_vec)
              // {
              //     for (int _col = 0; _col < m_total_assign_inter; _col++) {
              //         _dmd =
              //         m_passenger_demand[_o_node_ID][_d_node_ID][_col];
              //         // printf("%lf\n", _dmd());
              //         _path->m_buffer[_col] = TFlt(1.0) / _num_mode / _len *
              //         _dmd;
              //     }
              // }

              // one path for each od mode
              int p = 0;
              for (MNM_Passenger_Path_Base *_path : _m_it.second->m_path_vec)
                {
                  if (p == 0)
                    {
                      for (int _col = 0; _col < m_total_assign_inter; _col++)
                        {
                          _dmd
                            = m_passenger_demand[_o_node_ID][_d_node_ID][_col];
                          // printf("%lf\n", _dmd());
                          _path->m_buffer[_col] = TFlt (1.0) / _num_mode * _dmd;
                          // printf("%lf\n", (float)_path->m_buffer[_col]);
                        }
                    }
                  else
                    {
                      for (int _col = 0; _col < m_total_assign_inter; _col++)
                        {
                          _path->m_buffer[_col] = TFlt (0);
                          // printf("%lf\n", (float)_path->m_buffer[_col]);
                        }
                    }
                  p++;
                }
            }
          // verification
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              TFlt _tot_dmd_before
                = m_passenger_demand[_o_node_ID][_d_node_ID][_col];
              TFlt _tot_dmd_after = 0.;
              for (auto _m_it : *(_d_it.second))
                {
                  _mode = _m_it.first;
                  _len = TFlt (_m_it.second->m_path_vec.size ());
                  if (_len == TFlt (0))
                    {
                      throw std::runtime_error ("No available paths");
                    }
                  for (MNM_Passenger_Path_Base *_path :
                       _m_it.second->m_path_vec)
                    {
                      _tot_dmd_after += _path->m_buffer[_col];
                    }
                }
              IAssert (std::abs (_tot_dmd_after - _tot_dmd_before)
                       <= 1e-6
                            * std::max (std::abs (_tot_dmd_after),
                                        std::abs (_tot_dmd_before)));
            }
        }
    }

  printf ("Finish init route choice\n");
  return 0;
}

int
MNM_MM_Due::update_origin_demand_from_passenger_path_table (
  MNM_Dta_Multimodal *mmdta)
{
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  MNM_Passenger_Pathset *_pathset;
  MNM_Passenger_Path_Driving *_path_driving;
  MNM_Passenger_Path_PnR *_path_pnr;
  MNM_Passenger_Path_Bus *_path_bus;
  TFlt *_demand_vector;
  TFlt _demand;
  int _max_num_of_minute;
  int _num_of_minute = (int) mmdta->m_total_assign_inter / m_total_assign_inter;
  IAssert (_num_of_minute == 15);
  TFlt _flow_scalar = mmdta->m_flow_scalar;

  for (auto _o_it : m_passenger_demand)
    {
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (
        ((MNM_DMOND *) mmdta->m_node_factory->get_node (_o_it.first))
          ->m_origin);
      for (auto _d_it : _o_it.second)
        {
          _dest = dynamic_cast<MNM_Destination_Multimodal *> (
            ((MNM_DMDND *) mmdta->m_node_factory->get_node (_d_it.first))
              ->m_dest);

          // the driving mode impacts the car demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (driving)
                   ->second)
            {
              if (_origin->m_demand_car.find (_dest)
                  == _origin->m_demand_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  _origin->m_demand_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_car.find (_dest)->second;
              memset (_demand_vector, 0x0,
                      sizeof (TFlt) * mmdta->m_total_assign_inter);

              _pathset = m_passenger_path_table->find (_o_it.first)
                           ->second->find (_d_it.first)
                           ->second->find (driving)
                           ->second;
              if (!_pathset->m_path_vec.empty ())
                {
                  for (int _col = 0; _col < m_total_assign_inter; _col++)
                    {
                      _demand = 0.;
                      for (auto _path : _pathset->m_path_vec)
                        {
                          _path_driving
                            = dynamic_cast<MNM_Passenger_Path_Driving *> (
                              _path);
                          _demand += _path_driving->m_buffer[_col]
                                     / _path_driving->m_num_people;
                        }

                      if (mmdta->m_init_demand_split == 0)
                        {
                          _demand_vector[_col * _num_of_minute] = _demand;
                        }
                      else if (mmdta->m_init_demand_split == 1)
                        {
                          _max_num_of_minute = _num_of_minute;
                          for (int k = 0; k < _num_of_minute; ++k)
                            {
                              // if (round(_demand / TFlt(_num_of_minute - k) *
                              // _flow_scalar) >= 1){
                              if (floor (_demand / TFlt (_num_of_minute - k)
                                         * _flow_scalar)
                                  >= 1)
                                {
                                  _max_num_of_minute = _num_of_minute - k;
                                  break;
                                }
                            }
                          IAssert (_max_num_of_minute > 0
                                   && _max_num_of_minute <= _num_of_minute);
                          _demand = _demand / TFlt (_max_num_of_minute);
                          for (int k = 0; k < _max_num_of_minute; ++k)
                            {
                              // printf("original demand value is %f\n",
                              // (float)_demand_vector[_col * _num_of_minute +
                              // k]);
                              _demand_vector[_col * _num_of_minute + k]
                                = _demand;
                            }
                        }
                      else
                        {
                          throw std::runtime_error ("Wrong init_demand_split");
                        }
                    }
                }
            }

          // the pnr mode impacts the car demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (pnr)
                   ->second)
            {
              if (_origin->m_demand_pnr_car.find (_dest)
                  == _origin->m_demand_pnr_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  _origin->m_demand_pnr_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_pnr_car.find (_dest)->second;
              memset (_demand_vector, 0x0,
                      sizeof (TFlt) * mmdta->m_total_assign_inter);

              _pathset = m_passenger_path_table->find (_o_it.first)
                           ->second->find (_d_it.first)
                           ->second->find (pnr)
                           ->second;
              if (!_pathset->m_path_vec.empty ())
                {
                  for (int _col = 0; _col < m_total_assign_inter; _col++)
                    {
                      _demand = 0.;
                      for (auto _path : _pathset->m_path_vec)
                        {
                          _path_pnr
                            = dynamic_cast<MNM_Passenger_Path_PnR *> (_path);
                          _demand += _path_pnr->m_buffer[_col]
                                     / _path_pnr->m_driving_part->m_num_people;
                        }

                      if (mmdta->m_init_demand_split == 0)
                        {
                          _demand_vector[_col * _num_of_minute] = _demand;
                        }
                      else if (mmdta->m_init_demand_split == 1)
                        {
                          _max_num_of_minute = _num_of_minute;
                          for (int k = 0; k < _num_of_minute; ++k)
                            {
                              // if (round(_demand / TFlt(_num_of_minute - k) *
                              // _flow_scalar) >= 1){
                              if (floor (_demand / TFlt (_num_of_minute - k)
                                         * _flow_scalar)
                                  >= 1)
                                {
                                  _max_num_of_minute = _num_of_minute - k;
                                  break;
                                }
                            }
                          IAssert (_max_num_of_minute > 0
                                   && _max_num_of_minute <= _num_of_minute);
                          _demand = _demand / TFlt (_max_num_of_minute);
                          for (int k = 0; k < _max_num_of_minute; ++k)
                            {
                              // printf("original demand value is %f\n",
                              // (float)_demand_vector[_col * _num_of_minute +
                              // k]);
                              _demand_vector[_col * _num_of_minute + k]
                                = _demand;
                            }
                        }
                      else
                        {
                          throw std::runtime_error ("Wrong init_demand_split");
                        }
                    }
                }
            }

          // the transit mode impacts the passenger demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (transit)
                   ->second)
            {
              if (_origin->m_demand_passenger_bus.find (_dest)
                  == _origin->m_demand_passenger_bus.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  _origin->m_demand_passenger_bus.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector
                = _origin->m_demand_passenger_bus.find (_dest)->second;
              memset (_demand_vector, 0x0,
                      sizeof (TFlt) * mmdta->m_total_assign_inter);

              _pathset = m_passenger_path_table->find (_o_it.first)
                           ->second->find (_d_it.first)
                           ->second->find (transit)
                           ->second;
              if (!_pathset->m_path_vec.empty ())
                {
                  for (int _col = 0; _col < m_total_assign_inter; _col++)
                    {
                      _demand = 0.;
                      for (auto _path : _pathset->m_path_vec)
                        {
                          _path_bus
                            = dynamic_cast<MNM_Passenger_Path_Bus *> (_path);
                          _demand += _path_bus->m_buffer[_col];
                        }

                      // force the passengers to be released as a lump sum
                      // _demand_vector[_col * _num_of_minute] = _demand;

                      if (mmdta->m_init_demand_split == 0)
                        {
                          _demand_vector[_col * _num_of_minute] = _demand;
                        }
                      else if (mmdta->m_init_demand_split == 1)
                        {
                          // uniform
                          _max_num_of_minute = _num_of_minute;
                          for (int k = 0; k < _num_of_minute; ++k)
                            {
                              // if (floor(_demand / TFlt(_num_of_minute - k))
                              // >= 1){ add flow_scalar to passenger
                              if (floor (_demand / TFlt (_num_of_minute - k)
                                         * _flow_scalar)
                                  >= 1)
                                {
                                  _max_num_of_minute = _num_of_minute - k;
                                  break;
                                }
                            }
                          IAssert (_max_num_of_minute > 0
                                   && _max_num_of_minute <= _num_of_minute);
                          _demand = _demand / TFlt (_max_num_of_minute);
                          for (int k = 0; k < _max_num_of_minute; ++k)
                            {
                              // printf("original demand value is %f\n",
                              // (float)_demand_vector[_col * _num_of_minute +
                              // k]);
                              _demand_vector[_col * _num_of_minute + k]
                                = _demand;
                            }
                        }
                      else
                        {
                          throw std::runtime_error ("Wrong init_demand_split");
                        }
                    }
                }
            }
        }
    }

  return 0;
}

int
MNM_MM_Due::update_origin_demand_logit_model (MNM_Dta_Multimodal *mmdta,
                                              int assign_inter)
{
  IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
             ->m_routing_fixed_car->m_path_table
           != nullptr);
  IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
             ->m_routing_car_pnr_fixed->m_pnr_path_table
           != nullptr);
  IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
             ->m_routing_passenger_fixed->m_bustransit_path_table
           != nullptr);

  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;

  TFlt *_demand_vector;
  TFlt _demand, _tot_passenger_demand;
  int _max_num_of_minute;
  int _num_of_minute = (int) mmdta->m_total_assign_inter / m_total_assign_inter;
  IAssert (_num_of_minute == 15);
  TFlt _flow_scalar = mmdta->m_flow_scalar;

  std::unordered_map<int, TFlt> _mode_split;

  for (auto _o_it : m_passenger_demand)
    {
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (
        ((MNM_DMOND *) mmdta->m_node_factory->get_node (_o_it.first))
          ->m_origin);
      for (auto _d_it : _o_it.second)
        {
          _dest = dynamic_cast<MNM_Destination_Multimodal *> (
            ((MNM_DMDND *) mmdta->m_node_factory->get_node (_d_it.first))
              ->m_dest);

          _tot_passenger_demand = _d_it.second[assign_inter];

          _mode_split = get_mode_split_snapshot (mmdta,
                                                 assign_inter
                                                   * m_mmdta_config->get_int (
                                                     "assign_frq"),
                                                 _o_it.first, _d_it.first);

          // the driving mode impacts the car demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (driving)
                   ->second
              && _mode_split.find (driving) != _mode_split.end ())
            {
              if (_origin->m_demand_car.find (_dest)
                  == _origin->m_demand_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_car.find (_dest)->second;

              _demand
                = _mode_split.find (driving)->second * _tot_passenger_demand;
              m_mode_share.find (driving)->second += _demand;

              if (mmdta->m_init_demand_split == 0)
                {
                  _demand_vector[assign_inter * _num_of_minute] = _demand;
                  for (int k = 1; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      _demand_vector[assign_inter * _num_of_minute + k] = 0;
                    }
                }
              else if (mmdta->m_init_demand_split == 1)
                {
                  _max_num_of_minute = _num_of_minute;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // if (round(_demand / TFlt(_num_of_minute - k) *
                      // _flow_scalar) >= 1){
                      if (floor (_demand / TFlt (_num_of_minute - k)
                                 * _flow_scalar)
                          >= 1)
                        {
                          _max_num_of_minute = _num_of_minute - k;
                          break;
                        }
                    }
                  IAssert (_max_num_of_minute > 0
                           && _max_num_of_minute <= _num_of_minute);
                  _demand = _demand / TFlt (_max_num_of_minute);
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      if (k < _max_num_of_minute)
                        {
                          _demand_vector[assign_inter * _num_of_minute + k]
                            = _demand;
                        }
                      else
                        {
                          _demand_vector[assign_inter * _num_of_minute + k] = 0;
                        }
                    }
                }
              else
                {
                  throw std::runtime_error ("Wrong init_demand_split");
                }
            }
          else
            {
              if (_origin->m_demand_car.find (_dest)
                  != _origin->m_demand_car.end ())
                {
                  _demand_vector = _origin->m_demand_car.find (_dest)->second;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      _demand_vector[assign_inter * _num_of_minute + k] = 0;
                    }
                }
            }

          // the pnr mode impacts the car demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (pnr)
                   ->second
              && _mode_split.find (pnr) != _mode_split.end ())
            {
              if (_origin->m_demand_pnr_car.find (_dest)
                  == _origin->m_demand_pnr_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_pnr_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_pnr_car.find (_dest)->second;

              _demand = _mode_split.find (pnr)->second * _tot_passenger_demand;
              m_mode_share.find (pnr)->second += _demand;

              if (mmdta->m_init_demand_split == 0)
                {
                  _demand_vector[assign_inter * _num_of_minute] = _demand;
                  for (int k = 1; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      _demand_vector[assign_inter * _num_of_minute + k] = 0;
                    }
                }
              else if (mmdta->m_init_demand_split == 1)
                {
                  _max_num_of_minute = _num_of_minute;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // if (round(_demand / TFlt(_num_of_minute - k) *
                      // _flow_scalar) >= 1){
                      if (floor (_demand / TFlt (_num_of_minute - k)
                                 * _flow_scalar)
                          >= 1)
                        {
                          _max_num_of_minute = _num_of_minute - k;
                          break;
                        }
                    }
                  IAssert (_max_num_of_minute > 0
                           && _max_num_of_minute <= _num_of_minute);
                  _demand = _demand / TFlt (_max_num_of_minute);
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      if (k < _max_num_of_minute)
                        {
                          _demand_vector[assign_inter * _num_of_minute + k]
                            = _demand;
                        }
                      else
                        {
                          _demand_vector[assign_inter * _num_of_minute + k] = 0;
                        }
                    }
                }
              else
                {
                  throw std::runtime_error ("Wrong init_demand_split");
                }
            }
          else
            {
              if (_origin->m_demand_pnr_car.find (_dest)
                  != _origin->m_demand_pnr_car.end ())
                {
                  _demand_vector
                    = _origin->m_demand_pnr_car.find (_dest)->second;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      _demand_vector[assign_inter * _num_of_minute + k] = 0;
                    }
                }
            }

          // the transit mode impacts the passenger demand
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (transit)
                   ->second
              && _mode_split.find (transit) != _mode_split.end ())
            {
              if (_origin->m_demand_passenger_bus.find (_dest)
                  == _origin->m_demand_passenger_bus.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_passenger_bus.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector
                = _origin->m_demand_passenger_bus.find (_dest)->second;

              _demand
                = _mode_split.find (transit)->second * _tot_passenger_demand;
              m_mode_share.find (transit)->second += _demand;

              // force the passengers to be released as a lump sum
              // _demand_vector[assign_inter * _num_of_minute] = _demand;
              // for (int k = 1; k < _num_of_minute; ++k){
              //     // printf("original demand value is %f\n",
              //     (float)_demand_vector[_col * _num_of_minute + k]);
              //     _demand_vector[assign_inter * _num_of_minute + k] = 0;
              // }

              if (mmdta->m_init_demand_split == 0)
                {
                  _demand_vector[assign_inter * _num_of_minute] = _demand;
                  for (int k = 1; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      _demand_vector[assign_inter * _num_of_minute + k] = 0;
                    }
                }
              else if (mmdta->m_init_demand_split == 1)
                {
                  // uniform
                  _max_num_of_minute = _num_of_minute;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // if (floor(_demand / TFlt(_num_of_minute - k)) >= 1){
                      // add flow_scalar to passenger
                      if (floor (_demand / TFlt (_num_of_minute - k)
                                 * _flow_scalar)
                          >= 1)
                        {
                          _max_num_of_minute = _num_of_minute - k;
                          break;
                        }
                    }
                  IAssert (_max_num_of_minute > 0
                           && _max_num_of_minute <= _num_of_minute);
                  _demand = _demand / TFlt (_max_num_of_minute);
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      // printf("original demand value is %f\n",
                      // (float)_demand_vector[_col * _num_of_minute + k]);
                      if (k < _max_num_of_minute)
                        {
                          _demand_vector[assign_inter * _num_of_minute + k]
                            = _demand;
                        }
                      else
                        {
                          _demand_vector[assign_inter * _num_of_minute + k] = 0;
                        }
                    }
                }
              else
                {
                  throw std::runtime_error ("Wrong init_demand_split");
                }
            }
          else
            {
              if (_origin->m_demand_passenger_bus.find (_dest)
                  != _origin->m_demand_passenger_bus.end ())
                {
                  _demand_vector
                    = _origin->m_demand_passenger_bus.find (_dest)->second;
                  for (int k = 0; k < _num_of_minute; ++k)
                    {
                      _demand_vector[assign_inter * _num_of_minute + k]
                        = _demand;
                    }
                }
            }
        }
    }

  return 0;
}

int
MNM_MM_Due::save_od_demand_split (MNM_Dta_Multimodal *mmdta,
                                  const std::string &file_folder,
                                  const std::string &file_name)
{
  std::string _file_name = file_folder + "/" + file_name;

  std::ofstream _file;
  _file.open (_file_name, std::ofstream::out);
  if (!_file.is_open ())
    {
      throw std::runtime_error ("Error happens when open _file");
    }

  std::string _str;
  TInt _dest_node_ID, _origin_node_ID, _mode;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  int _num_of_minute = (int) mmdta->m_total_assign_inter / m_total_assign_inter;
  TFlt *_demand_vector;
  TFlt _demand;

  _str = std::string ("#origin_node_ID dest_node_ID mode <demand_sequence>\n");
  _file << _str;
  for (auto _o_it : *m_passenger_path_table)
    {
      _origin_node_ID = _o_it.first;
      _origin = dynamic_cast<MNM_Origin_Multimodal *> (
        ((MNM_DMOND *) mmdta->m_node_factory->get_node (_origin_node_ID))
          ->m_origin);
      for (auto _d_it : *_o_it.second)
        {
          _dest_node_ID = _d_it.first;
          _dest = dynamic_cast<MNM_Destination_Multimodal *> (
            ((MNM_DMDND *) mmdta->m_node_factory->get_node (_dest_node_ID))
              ->m_dest);

          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              if (_origin->m_demand_car.find (_dest)
                  == _origin->m_demand_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_car.find (_dest)->second;

              _str = std::to_string (_origin_node_ID ()) + " "
                     + std::to_string (_dest_node_ID ()) + " " + "driving"
                     + " ";
              for (int i = 0; i < m_total_assign_inter; ++i)
                {
                  _demand = 0;
                  for (int j = 0; j < _num_of_minute; ++j)
                    {
                      _demand += _demand_vector[i * _num_of_minute + j];
                    }
                  _str += std::to_string (_demand ()) + " ";
                }
              _str.pop_back ();
              _str += "\n";
              _file << _str;
            }

          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              if (_origin->m_demand_pnr_car.find (_dest)
                  == _origin->m_demand_pnr_car.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_pnr_car.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector = _origin->m_demand_pnr_car.find (_dest)->second;

              _str = std::to_string (_origin_node_ID ()) + " "
                     + std::to_string (_dest_node_ID ()) + " " + "pnr" + " ";
              for (int i = 0; i < m_total_assign_inter; ++i)
                {
                  _demand = 0;
                  for (int j = 0; j < _num_of_minute; ++j)
                    {
                      _demand += _demand_vector[i * _num_of_minute + j];
                    }
                  _str += std::to_string (_demand ()) + " ";
                }
              _str.pop_back ();
              _str += "\n";
              _file << _str;
            }

          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_origin_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              if (_origin->m_demand_passenger_bus.find (_dest)
                  == _origin->m_demand_passenger_bus.end ())
                {
                  TFlt *_demand_vector_tmp = (TFlt *) malloc (
                    sizeof (TFlt) * m_total_assign_inter * _num_of_minute);
                  memset (_demand_vector_tmp, 0x0,
                          sizeof (TFlt) * m_total_assign_inter
                            * _num_of_minute);
                  _origin->m_demand_passenger_bus.insert (
                    std::pair<MNM_Destination_Multimodal *,
                              TFlt *> (_dest, _demand_vector_tmp));
                }
              _demand_vector
                = _origin->m_demand_passenger_bus.find (_dest)->second;

              _str = std::to_string (_origin_node_ID ()) + " "
                     + std::to_string (_dest_node_ID ()) + " " + "bus_transit"
                     + " ";
              for (int i = 0; i < m_total_assign_inter; ++i)
                {
                  _demand = 0;
                  for (int j = 0; j < _num_of_minute; ++j)
                    {
                      _demand += _demand_vector[i * _num_of_minute + j];
                    }
                  _str += std::to_string (_demand ()) + " ";
                }
              _str.pop_back ();
              _str += "\n";
              _file << _str;
            }
        }
    }
  _file.close ();
  return 0;
}

std::unordered_map<int, TFlt>
MNM_MM_Due::get_mode_split_snapshot (MNM_Dta_Multimodal *mmdta,
                                     int start_interval, int o_node_ID,
                                     int d_node_ID)
{
  IAssert (m_passenger_demand.find (o_node_ID) != m_passenger_demand.end ()
           && m_passenger_demand.find (o_node_ID)->second.find (d_node_ID)
                != m_passenger_demand.find (o_node_ID)->second.end ());

  int _assign_inter
    = (int) start_interval / m_mmdta_config->get_int ("assign_frq");
  if (_assign_inter >= m_total_assign_inter)
    _assign_inter = m_total_assign_inter - 1;

  std::unordered_map<int, TFlt> _cost = std::unordered_map<int, TFlt> ();
  std::unordered_map<int, TFlt> _alpha_map = std::unordered_map<int, TFlt> ();

  MNM_Parking_Lot *_best_mid_parkinglot;
  MNM_Destination_Multimodal *_dest;
  TInt _mid_dest_node_ID;
  TFlt _cur_best_path_tt, _path_tt, _tot_dmd_one_mode, _tmp_cost;

  std::unordered_map<TInt, TInt> _shortest_path_tree_driving;
  std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit;
  std::unordered_map<TInt, TInt> _shortest_path_tree_pnr;

  MNM_Path *_path;
  MNM_Path *_driving_path;
  MNM_Path *_transit_path;
  MNM_PnR_Path *_pnr_path;

  // driving
  MNM_Passenger_Path_Driving *_p_path_driving;
  // bus
  MNM_Passenger_Path_Bus *_p_path_bus;
  // pnr
  MNM_Passenger_Path_PnR *_p_path_pnr;

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _cost.insert (std::pair<int, TFlt> (driving, 0.));
      _alpha_map.insert (std::pair<int, TFlt> (driving, m_alpha1_driving));
    }
  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _cost.insert (std::pair<int, TFlt> (transit, 0.));
      _alpha_map.insert (std::pair<int, TFlt> (transit, m_alpha1_transit));
    }
  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _cost.insert (std::pair<int, TFlt> (pnr, 0.));
      _alpha_map.insert (std::pair<int, TFlt> (pnr, m_alpha1_pnr));
    }

  _dest = dynamic_cast<MNM_Destination_Multimodal *> (
    ((MNM_DMDND *) mmdta->m_node_factory->get_node (d_node_ID))->m_dest);

  _shortest_path_tree_driving
    = m_driving_table_snapshot.find (d_node_ID)->second;
  if (mmdta->m_bus_transit_graph->IsNode (d_node_ID))
    {
      _shortest_path_tree_bustransit
        = m_bustransit_table_snapshot.find (d_node_ID)->second;
    }

  // driving
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (driving)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path = MNM::extract_path (o_node_ID, d_node_ID,
                                 _shortest_path_tree_driving, mmdta->m_graph);
      IAssert (_path != nullptr);
      _p_path_driving = new MNM_Passenger_Path_Driving (
        driving, _path, m_vot, m_early_penalty, m_late_penalty, m_target_time,
        1, m_carpool_cost_multiplier, 0.0, _dest->m_parking_lot,
        m_parking_lot_to_destination_walking_time);

      // tdsp cost
      // _tmp_cost = _p_path_driving -> get_travel_cost(TFlt(start_interval),
      // mmdta); _tot_dmd_one_mode =
      // compute_total_passenger_demand_for_one_mode(driving, o_node_ID,
      // d_node_ID, _assign_inter); _tmp_cost =  get_disutility(driving,
      // _tmp_cost, _tot_dmd_one_mode);

      // snapshot cost
      _tmp_cost
        = MNM::get_path_tt_snapshot (_path, m_driving_link_tt_map_snapshot);
      if (_p_path_driving->m_parking_lot != nullptr)
        {
          for (int i = 0; i <= start_interval; i++)
            {
              if (_p_path_driving->m_parking_lot->m_cruising_time_record.find (
                    start_interval - i)
                  != _p_path_driving->m_parking_lot->m_cruising_time_record
                       .end ())
                {
                  _tmp_cost
                    += _p_path_driving->m_parking_lot->m_cruising_time_record
                         .find (start_interval - i)
                         ->second;
                  break;
                }
            }
        }
      _tmp_cost = _p_path_driving->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

      _cost.find (driving)->second = _tmp_cost;
      delete _p_path_driving;
    }

  // bus transit
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (transit)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path = MNM::extract_path (o_node_ID, d_node_ID,
                                 _shortest_path_tree_bustransit,
                                 mmdta->m_bus_transit_graph);
      if (_path != nullptr)
        {
          _p_path_bus
            = new MNM_Passenger_Path_Bus (transit, _path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, m_bus_fare,
                                          m_bus_inconvenience);

          // tdsp cost
          // _tmp_cost = _p_path_bus -> get_travel_cost(TFlt(start_interval),
          // mmdta); _tot_dmd_one_mode =
          // compute_total_passenger_demand_for_one_mode(transit, o_node_ID,
          // d_node_ID, _assign_inter); _tmp_cost = get_disutility(transit,
          // _tmp_cost, _tot_dmd_one_mode);

          // snapshot cost
          _tmp_cost
            = MNM::get_path_tt_snapshot (_path,
                                         m_bustransit_link_tt_map_snapshot);
          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _tmp_cost
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_tmp_cost));

          _tmp_cost = _p_path_bus->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

          _cost.find (transit)->second = _tmp_cost;
          delete _p_path_bus;
        }
      else
        {
          _cost.find (transit)->second
            = TFlt (std::numeric_limits<double>::infinity ());
        }
    }

  // pnr
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (pnr)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _cur_best_path_tt = TFlt (std::numeric_limits<double>::infinity ());
      _best_mid_parkinglot = nullptr;
      _pnr_path = nullptr;
      for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
        {
          _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
          // pnr
          _path_tt = 0.;

          // mobility service waiting time at origins
          _path_tt
            += dynamic_cast<MNM_Origin_Multimodal *> (
                 ((MNM_DMOND *) mmdta->m_node_factory->get_node (o_node_ID))
                   ->m_origin)
                 ->m_pickup_waiting_time;

          _shortest_path_tree_pnr
            = m_driving_table_snapshot.find (_mid_dest_node_ID)->second;
          _driving_path
            = MNM::extract_path (o_node_ID, _mid_dest_node_ID,
                                 _shortest_path_tree_pnr, mmdta->m_graph);
          IAssert (_driving_path != nullptr);
          _path_tt
            += MNM::get_path_tt_snapshot (_driving_path,
                                          m_driving_link_tt_map_snapshot);

          _transit_path = MNM::extract_path (_mid_dest_node_ID, d_node_ID,
                                             _shortest_path_tree_bustransit,
                                             mmdta->m_bus_transit_graph);
          if (_transit_path == nullptr)
            {
              delete _driving_path;
              continue;
            }
          IAssert (_transit_path != nullptr);
          _path_tt
            += MNM::get_path_tt_snapshot (_transit_path,
                                          m_bustransit_link_tt_map_snapshot);

          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _transit_path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _path_tt
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_path_tt));

          if (_cur_best_path_tt > _path_tt)
            {
              _cur_best_path_tt = _path_tt;
              _best_mid_parkinglot = _parkinglot;
              delete _pnr_path;
              _pnr_path = new MNM_PnR_Path (0, _best_mid_parkinglot->m_ID,
                                            _mid_dest_node_ID, _driving_path,
                                            _transit_path);
              _driving_path = nullptr;
              _transit_path = nullptr;
            }
          else
            {
              delete _driving_path;
              delete _transit_path;
            }
        }
      if (_pnr_path != nullptr && _best_mid_parkinglot != nullptr)
        {
          _p_path_pnr
            = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, 0.0,
                                          _best_mid_parkinglot, m_bus_fare,
                                          m_pnr_inconvenience);

          // tdsp cost
          // _tmp_cost = _p_path_pnr -> get_travel_cost(TFlt(start_interval),
          // mmdta); _tot_dmd_one_mode =
          // compute_total_passenger_demand_for_one_mode(pnr, o_node_ID,
          // d_node_ID, _assign_inter); _tmp_cost = get_disutility(pnr,
          // _tmp_cost, _tot_dmd_one_mode);

          // snapshot cost
          _tmp_cost
            = MNM::get_path_tt_snapshot (_pnr_path->m_driving_path,
                                         m_driving_link_tt_map_snapshot)
              + MNM::get_path_tt_snapshot (_pnr_path->m_transit_path,
                                           m_bustransit_link_tt_map_snapshot);

          // cruising time for pnr, no cruising for mobility service for middle
          // destination for (int i=0; i <= start_interval; i++) {
          //     if (_p_path_pnr -> m_mid_parking_lot ->
          //     m_cruising_time_record.find(start_interval-i) != _p_path_pnr ->
          //     m_mid_parking_lot -> m_cruising_time_record.end()) {
          //         _tmp_cost += _p_path_pnr -> m_mid_parking_lot ->
          //         m_cruising_time_record.find(start_interval-i) -> second;
          //         break;
          //     }
          // }

          // mobility service waiting time at origins
          _tmp_cost
            += dynamic_cast<MNM_Origin_Multimodal *> (
                 ((MNM_DMOND *) mmdta->m_node_factory->get_node (o_node_ID))
                   ->m_origin)
                 ->m_pickup_waiting_time;

          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _pnr_path->m_transit_path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _tmp_cost
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_tmp_cost));

          _tmp_cost = _p_path_pnr->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

          _cost.find (pnr)->second = _tmp_cost;
          delete _p_path_pnr;
        }
      else
        {
          _cost.find (pnr)->second
            = TFlt (std::numeric_limits<double>::infinity ());
        }
    }

  return MNM::logit_fn (_cost, _alpha_map, m_beta1);
}

int
MNM_MM_Due::build_link_cost_map_snapshot (MNM_Dta_Multimodal *mmdta,
                                          int start_interval,
                                          bool in_simulation)
{
  // two scenarios: in simulation and after simulation
  // in simulation, if we invoke MNM_DTA_GRADIENT::get_travel_time_car(), this
  // will change m_last_valid_time

  m_driving_link_tt_map_snapshot.clear ();
  m_bustransit_link_tt_map_snapshot.clear ();
  m_driving_link_cost_map_snapshot.clear ();
  m_bustransit_link_cost_map_snapshot.clear ();

  MNM_Dlink_Multiclass *_link;
  MNM_Transit_Link *_transitlink;
  TFlt _tt;

  std::cout << "********************** build_link_cost_map_snapshot interval "
            << start_interval << " **********************\n";
  for (auto _link_it : mmdta->m_link_factory->m_link_map)
    {
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
      if (in_simulation)
        {
          _tt = dynamic_cast<MNM_Statistics_Lrn_Multimodal *> (
                  mmdta->m_statistics)
                  ->m_load_interval_tt.find (_link_it.first)
                  ->second
                / m_unit_time; // intervals
        }
      else
        {
          _tt = MNM_DTA_GRADIENT::
            get_travel_time_car (_link, TFlt (start_interval + 1), m_unit_time,
                                 mmdta
                                   ->m_current_loading_interval); // intervals
        }
      m_driving_link_tt_map_snapshot.insert (
        std::pair<TInt, TFlt> (_link_it.first, _tt));
      m_driving_link_cost_map_snapshot.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               m_vot * _tt + _link->m_toll_car));
      // std::cout << "interval: " << i << ", link: " << _link_it.first << ",
      // tt: " << m_link_tt_map[_link_it.first] << "\n";
    }
  for (auto _link_it : mmdta->m_transitlink_factory->m_transit_link_map)
    {
      _transitlink = _link_it.second;
      if (_transitlink->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
        {
          if (in_simulation)
            {
              _tt = dynamic_cast<MNM_Statistics_Lrn_Multimodal *> (
                      mmdta->m_statistics)
                      ->m_load_interval_tt_bus_transit.find (_link_it.first)
                      ->second
                    / m_unit_time; // intervals
            }
          else
            {
              _tt = MNM_DTA_GRADIENT::get_travel_time_walking (
                dynamic_cast<MNM_Walking_Link *> (_transitlink),
                TFlt (start_interval + 1), m_unit_time,
                mmdta->m_current_loading_interval); // intervals
            }
        }
      else if (_transitlink->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
        {
          if (in_simulation)
            {
              _tt = dynamic_cast<MNM_Statistics_Lrn_Multimodal *> (
                      mmdta->m_statistics)
                      ->m_load_interval_tt_bus_transit.find (_link_it.first)
                      ->second
                    / m_unit_time; // intervals
            }
          else
            {
              _tt = MNM_DTA_GRADIENT::
                get_travel_time_bus (dynamic_cast<MNM_Bus_Link *> (
                                       _transitlink),
                                     TFlt (start_interval + 1), m_unit_time,
                                     mmdta->m_current_loading_interval,
                                     mmdta->m_explicit_bus, false,
                                     !mmdta->m_explicit_bus); // intervals
            }
        }
      else
        {
          throw std::runtime_error ("Wrong transit link type");
        }
      m_bustransit_link_tt_map_snapshot.insert (
        std::pair<TInt, TFlt> (_link_it.first, _tt));
      m_bustransit_link_cost_map_snapshot.insert (
        std::pair<TInt, TFlt> (_link_it.first, m_vot * _tt));
      // std::cout << "interval: " << i << ", link: " << _link_it.first << ",
      // tt: " << m_transitlink_cost_map[_link_it.first] << "\n";
    }

  return 0;
}

int
MNM_MM_Due::update_snapshot_route_table (MNM_Dta_Multimodal *mmdta,
                                         int start_interval)
{
  // build_link_cost_map_snapshot should be called first
  for (auto _d_it : m_driving_table_snapshot)
    {
      _d_it.second.clear ();
    }
  m_driving_table_snapshot.clear ();
  for (auto _d_it : m_bustransit_table_snapshot)
    {
      _d_it.second.clear ();
    }
  m_bustransit_table_snapshot.clear ();

  TInt _dest_node_ID;

  // <d_node_ID, <node_ID, out_link_ID>>
  std::unordered_map<TInt, TInt> _shortest_path_tree_driving
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit
    = std::unordered_map<TInt, TInt> ();

  for (auto _d_it : mmdta->m_od_factory->m_destination_map)
    {
      _dest_node_ID = _d_it.second->m_dest_node->m_node_ID;

      if (!_shortest_path_tree_driving.empty ())
        {
          _shortest_path_tree_driving.clear ();
        }
      MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, mmdta->m_graph,
                                          m_driving_link_cost_map_snapshot,
                                          _shortest_path_tree_driving);
      m_driving_table_snapshot.insert (
        std::pair<
          TInt, std::unordered_map<TInt, TInt>> (_dest_node_ID,
                                                 _shortest_path_tree_driving));

      if (mmdta->m_bus_transit_graph->IsNode (_dest_node_ID))
        {
          if (!_shortest_path_tree_bustransit.empty ())
            {
              _shortest_path_tree_bustransit.clear ();
            }
          MNM_Shortest_Path::
            all_to_one_FIFO (_dest_node_ID, mmdta->m_bus_transit_graph,
                             m_bustransit_link_cost_map_snapshot,
                             _shortest_path_tree_bustransit);
          m_bustransit_table_snapshot.insert (
            std::pair<TInt, std::unordered_map<
                              TInt, TInt>> (_dest_node_ID,
                                            _shortest_path_tree_bustransit));
        }
    }
  return 0;
}

std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt, bool>, int>
MNM_MM_Due::get_lowest_cost_path_snapshot (int start_interval, int o_node_ID,
                                           int d_node_ID,
                                           MNM_Dta_Multimodal *mmdta)
{
  // get lowest cost path departing at start_interval snapshot

  IAssert (m_passenger_demand.find (o_node_ID) != m_passenger_demand.end ()
           && m_passenger_demand.find (o_node_ID)->second.find (d_node_ID)
                != m_passenger_demand.find (o_node_ID)->second.end ());

  int _assign_inter
    = (int) start_interval / m_mmdta_config->get_int ("assign_frq");
  if (_assign_inter >= m_total_assign_inter)
    _assign_inter = m_total_assign_inter - 1;

  MNM_Parking_Lot *_best_mid_parkinglot;
  MNM_Destination_Multimodal *_dest;
  TInt _mid_dest_node_ID;
  TFlt _cur_best_path_tt, _path_tt, _tot_dmd_one_mode, _tmp_cost;

  std::unordered_map<TInt, TInt> _shortest_path_tree_driving;
  std::unordered_map<TInt, TInt> _shortest_path_tree_bustransit;
  std::unordered_map<TInt, TInt> _shortest_path_tree_pnr;

  MNM_Path *_path;
  MNM_Path *_driving_path;
  MNM_Path *_transit_path;
  MNM_PnR_Path *_pnr_path;

  // driving
  MNM_Passenger_Path_Driving *_p_path_driving;
  // bus
  MNM_Passenger_Path_Bus *_p_path_bus;
  // pnr
  MNM_Passenger_Path_PnR *_p_path_pnr;

  MNM_Passenger_Path_Base *_p_path = nullptr;

  TInt _mode;
  TFlt _cost = TFlt (std::numeric_limits<double>::infinity ());
  int _best_time_col, _best_assign_col;
  bool _exist;
  MNM_Passenger_Pathset *_path_set_driving;
  MNM_Passenger_Pathset *_path_set_bus;
  MNM_Passenger_Pathset *_path_set_pnr;

  _path_set_driving = nullptr;
  _path_set_bus = nullptr;
  _path_set_pnr = nullptr;
  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path_set_driving = m_passenger_path_table->find (o_node_ID)
                            ->second->find (d_node_ID)
                            ->second->find (driving)
                            ->second;
    }
  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path_set_bus = m_passenger_path_table->find (o_node_ID)
                        ->second->find (d_node_ID)
                        ->second->find (transit)
                        ->second;
    }
  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _path_set_pnr = m_passenger_path_table->find (o_node_ID)
                        ->second->find (d_node_ID)
                        ->second->find (pnr)
                        ->second;
    }

  _dest = dynamic_cast<MNM_Destination_Multimodal *> (
    ((MNM_DMDND *) mmdta->m_node_factory->get_node (d_node_ID))->m_dest);

  _shortest_path_tree_driving
    = m_driving_table_snapshot.find (d_node_ID)->second;
  if (mmdta->m_bus_transit_graph->IsNode (d_node_ID))
    {
      _shortest_path_tree_bustransit
        = m_bustransit_table_snapshot.find (d_node_ID)->second;
    }

  // driving
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (driving)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path = MNM::extract_path (o_node_ID, d_node_ID,
                                 _shortest_path_tree_driving, mmdta->m_graph);
      IAssert (_path != nullptr);
      _p_path_driving = new MNM_Passenger_Path_Driving (
        driving, _path, m_vot, m_early_penalty, m_late_penalty, m_target_time,
        1, m_carpool_cost_multiplier, 0.0, _dest->m_parking_lot,
        m_parking_lot_to_destination_walking_time);

      // tdsp cost
      // _tmp_cost = _p_path_driving -> get_travel_cost(TFlt(start_interval),
      // mmdta); _tot_dmd_one_mode =
      // compute_total_passenger_demand_for_one_mode(driving, o_node_ID,
      // d_node_ID, _assign_inter); _tmp_cost =  get_disutility(driving,
      // _tmp_cost, _tot_dmd_one_mode);

      // snapshot cost
      _tmp_cost
        = MNM::get_path_tt_snapshot (_path, m_driving_link_tt_map_snapshot);
      if (_p_path_driving->m_parking_lot != nullptr)
        {
          for (int i = 0; i <= start_interval; i++)
            {
              if (_p_path_driving->m_parking_lot->m_cruising_time_record.find (
                    start_interval - i)
                  != _p_path_driving->m_parking_lot->m_cruising_time_record
                       .end ())
                {
                  _tmp_cost
                    += _p_path_driving->m_parking_lot->m_cruising_time_record
                         .find (start_interval - i)
                         ->second;
                  break;
                }
            }
        }
      _tmp_cost = _p_path_driving->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

      if (_cost > _tmp_cost)
        {
          _mode = driving;
          _cost = _tmp_cost;
          delete _p_path;
          _p_path = _p_path_driving;
        }
      _path = nullptr;
      IAssert (_p_path_driving->m_path != nullptr);
    }

  // bus transit
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (transit)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path = MNM::extract_path (o_node_ID, d_node_ID,
                                 _shortest_path_tree_bustransit,
                                 mmdta->m_bus_transit_graph);
      if (_path != nullptr)
        {
          _p_path_bus
            = new MNM_Passenger_Path_Bus (transit, _path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, m_bus_fare,
                                          m_bus_inconvenience);

          // tdsp cost
          // _tmp_cost = _p_path_bus -> get_travel_cost(TFlt(start_interval),
          // mmdta); _tot_dmd_one_mode =
          // compute_total_passenger_demand_for_one_mode(transit, o_node_ID,
          // d_node_ID, _assign_inter); _tmp_cost = get_disutility(transit,
          // _tmp_cost, _tot_dmd_one_mode);

          // snapshot cost
          // moving time
          _tmp_cost
            = MNM::get_path_tt_snapshot (_path,
                                         m_bustransit_link_tt_map_snapshot);

          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _tmp_cost
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_tmp_cost));

          _tmp_cost = _p_path_bus->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

          if (_cost > _tmp_cost)
            {
              _mode = transit;
              _cost = _tmp_cost;
              delete _p_path;
              _p_path = _p_path_bus;
            }

          _path = nullptr;
          IAssert (_p_path_bus->m_path != nullptr);
        }
    }

  // pnr
  if (m_od_mode_connectivity.find (o_node_ID)
          ->second.find (d_node_ID)
          ->second.find (pnr)
        != m_od_mode_connectivity.find (o_node_ID)
             ->second.find (d_node_ID)
             ->second.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _cur_best_path_tt = TFlt (std::numeric_limits<double>::infinity ());
      _best_mid_parkinglot = nullptr;
      _pnr_path = nullptr;
      // just use a random middle parking lot
      std::random_shuffle (_dest->m_connected_pnr_parkinglot_vec.begin (),
                           _dest->m_connected_pnr_parkinglot_vec.end ());
      for (auto _parkinglot : _dest->m_connected_pnr_parkinglot_vec)
        {
          _mid_dest_node_ID = _parkinglot->m_dest_node->m_node_ID;
          // pnr
          _path_tt = 0.;

          // mobility service waiting time at origins
          _path_tt
            += dynamic_cast<MNM_Origin_Multimodal *> (
                 ((MNM_DMOND *) mmdta->m_node_factory->get_node (o_node_ID))
                   ->m_origin)
                 ->m_pickup_waiting_time;

          _shortest_path_tree_pnr
            = m_driving_table_snapshot.find (_mid_dest_node_ID)->second;
          _driving_path
            = MNM::extract_path (o_node_ID, _mid_dest_node_ID,
                                 _shortest_path_tree_pnr, mmdta->m_graph);
          IAssert (_driving_path != nullptr);
          _path_tt
            += MNM::get_path_tt_snapshot (_driving_path,
                                          m_driving_link_tt_map_snapshot);

          _transit_path = MNM::extract_path (_mid_dest_node_ID, d_node_ID,
                                             _shortest_path_tree_bustransit,
                                             mmdta->m_bus_transit_graph);
          if (_transit_path == nullptr)
            {
              delete _driving_path;
              continue;
            }
          IAssert (_transit_path != nullptr);
          _path_tt
            += MNM::get_path_tt_snapshot (_transit_path,
                                          m_bustransit_link_tt_map_snapshot);

          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _transit_path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _path_tt
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_path_tt));

          if (_cur_best_path_tt > _path_tt)
            {
              _cur_best_path_tt = _path_tt;
              _best_mid_parkinglot = _parkinglot;
              delete _pnr_path;
              _pnr_path = new MNM_PnR_Path (0, _best_mid_parkinglot->m_ID,
                                            _mid_dest_node_ID, _driving_path,
                                            _transit_path);
              _driving_path = nullptr;
              _transit_path = nullptr;
            }
          else
            {
              delete _driving_path;
              delete _transit_path;
            }
          // just use a random middle parking lot
          break;
        }
      if (_pnr_path != nullptr && _best_mid_parkinglot != nullptr)
        {
          _p_path_pnr
            = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, 0.0,
                                          _best_mid_parkinglot, m_bus_fare,
                                          m_pnr_inconvenience);

          // tdsp cost
          // _tmp_cost = _p_path_pnr -> get_travel_cost(TFlt(start_interval),
          // mmdta); _tot_dmd_one_mode =
          // compute_total_passenger_demand_for_one_mode(pnr, o_node_ID,
          // d_node_ID, _assign_inter); _tmp_cost = get_disutility(pnr,
          // _tmp_cost, _tot_dmd_one_mode);

          // snapshot cost
          _tmp_cost
            = MNM::get_path_tt_snapshot (_pnr_path->m_driving_path,
                                         m_driving_link_tt_map_snapshot)
              + MNM::get_path_tt_snapshot (_pnr_path->m_transit_path,
                                           m_bustransit_link_tt_map_snapshot);

          // cruising time for pnr, no cruising for mobility service for middle
          // destination for (int i=0; i <= start_interval; i++) {
          //     if (_p_path_pnr -> m_mid_parking_lot ->
          //     m_cruising_time_record.find(start_interval-i) != _p_path_pnr ->
          //     m_mid_parking_lot -> m_cruising_time_record.end()) {
          //         _tmp_cost += _p_path_pnr -> m_mid_parking_lot ->
          //         m_cruising_time_record.find(start_interval-i) -> second;
          //         break;
          //     }
          // }

          // mobility service waiting time at origins
          _tmp_cost
            += dynamic_cast<MNM_Origin_Multimodal *> (
                 ((MNM_DMOND *) mmdta->m_node_factory->get_node (o_node_ID))
                   ->m_origin)
                 ->m_pickup_waiting_time;

          // TODO:
          // waiting time: only account for the waiting time at the first bus
          // stop in the path
          for (int _link_ID : _pnr_path->m_transit_path->m_link_vec)
            {
              auto *_link
                = mmdta->m_transitlink_factory->get_transit_link (_link_ID);
              if (auto *_link_bus = dynamic_cast<MNM_Bus_Link *> (_link))
                {
                  _tmp_cost
                    += _link_bus->m_from_busstop->get_waiting_time_snapshot (
                         start_interval)
                       / m_unit_time;
                  break;
                }
            }
          IAssert (!std::isinf (_tmp_cost));

          _tmp_cost = _p_path_pnr->get_travel_cost_with_tt (start_interval,
                                                            _tmp_cost, mmdta);

          if (_cost > _tmp_cost)
            {
              _mode = pnr;
              _cost = _tmp_cost;
              delete _p_path;
              _p_path = _p_path_pnr;
            }

          _pnr_path = nullptr;
          IAssert (_p_path_pnr->m_path != nullptr);
        }
    }

  IAssert (_p_path != nullptr);

  _best_time_col = start_interval;
  _best_assign_col
    = (int) _best_time_col / m_mmdta_config->get_int ("assign_frq");
  if (_best_assign_col >= m_total_assign_inter)
    _best_assign_col = m_total_assign_inter - 1;

  _exist = false;
  _path = nullptr;
  if (_mode == driving && _path_set_driving != nullptr)
    {
      _exist = _path_set_driving->is_in (_p_path);
      // _path = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) -> m_path;
      // _num_col = (int) _path -> m_node_vec.size();
    }
  else if (_mode == transit && _path_set_bus != nullptr)
    {
      _exist = _path_set_bus->is_in (_p_path);
      // _path = dynamic_cast<MNM_Passenger_Path_Bus*>(_p_path) -> m_path;
      // _num_col = (int) _path -> m_link_vec.size();
    }
  else if (_mode == pnr && _path_set_pnr != nullptr)
    {
      _exist = _path_set_pnr->is_in (_p_path);
      // _path = dynamic_cast<MNM_Passenger_Path_PnR*>(_p_path) -> m_path;
      // _num_col = std::max(int(dynamic_cast<MNM_PnR_Path*>(_path) ->
      // m_driving_path -> m_node_vec.size()),
      //                     int(dynamic_cast<MNM_PnR_Path*>(_path) ->
      //                     m_transit_path -> m_link_vec.size()));
    }
  else
    {
      throw std::runtime_error ("Mode not implemented");
    }
  // IAssert(_path != nullptr);

  return std::make_pair (std::make_tuple (_p_path, TInt (_best_time_col), _cost,
                                          _exist),
                         (int) _mode);
}

int
MNM_MM_Due::clear_multimodal_path_table_buffer ()
{
  //    for (auto _it: *m_driving_path_table) {
  //        for (auto _it_it : *_it.second) {
  //            for (auto _path : _it_it.second -> m_path_vec) {
  //                for (int i = 0; i < _path -> m_buffer_length; ++i) {
  //                    _path -> m_buffer[i] = 0.;
  //                }
  //            }
  //        }
  //    }
  //
  //    for (auto _it: *m_pnr_path_table) {
  //        for (auto _it_it : *_it.second) {
  //            for (auto _path : _it_it.second -> m_path_vec) {
  //                for (int i = 0; i < _path -> m_buffer_length; ++i) {
  //                    _path -> m_buffer[i] = 0.;
  //                }
  //            }
  //        }
  //    }
  //
  //    for (auto _it: *m_bustransit_path_table) {
  //        for (auto _it_it : *_it.second) {
  //            for (auto _path : _it_it.second -> m_path_vec) {
  //                for (int i = 0; i < _path -> m_buffer_length; ++i) {
  //                    _path -> m_buffer[i] = 0.;
  //                }
  //            }
  //        }
  //    }

  if (!m_driving_path_table->empty ())
    {
      for (auto _it : *m_driving_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_driving_path_table->clear ();
    }

  if (!m_pnr_path_table->empty ())
    {
      for (auto _it : *m_pnr_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_pnr_path_table->clear ();
    }

  if (!m_bustransit_path_table->empty ())
    {
      for (auto _it : *m_bustransit_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      m_bustransit_path_table->clear ();
    }
  return 0;
}

int
MNM_MM_Due::passenger_path_table_to_multimodal_path_table (
  MNM_Dta_Multimodal *mmdta)
{
  TInt _num_people;
  MNM_Passenger_Pathset *_passenger_pathset;
  MNM_Pathset *_driving_pathset;
  MNM_Pathset *_truck_pathset;
  MNM_PnR_Pathset *_pnr_pathset;
  MNM_Pathset *_bustransit_pathset;
  MNM_Passenger_Path_Driving *_passenger_path_driving;
  MNM_Passenger_Path_PnR *_passenger_path_pnr;
  MNM_Passenger_Path_Bus *_passenger_path_bus;
  MNM_Path *_driving_path;
  MNM_PnR_Path *_pnr_path;
  MNM_Path *_bustransit_path;
  bool _is_in;

  // build m_driving_path_table, m_pnr_path_table, and m_bustransit_path_table
  // from scratch check set_routing() for "MNM_Routing_Multimodal_Hybrid";
  // "Multimodal_Hybrid" is for loading with fixed pathset
  // "Multimodal_Hybrid_ColumnGeneration" is only for DODE's INITIALIZATION,
  // where no path files exist as input for both car and truck,
  // MNM_mmnb.py/MNM_network_builder.load_from_folder() ColumnGeneration for
  // DODE's subsequent iterations is an option in python function in
  // mmDODE.py/MMDODE.estimate_path_flow_pytorch() "Multimodal_DUE_FixedPath" is
  // for loading with fixed pathset "Multimodal_DUE_ColumnGeneration" assumes
  // some fixed background truck demand MAY exist, which means
  // m_driving_path_table is not nullptr "Multimodal_DUE_ColumnGeneration" is
  // used for one-time loading and will not be used in DODE setting
  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_Hybrid_ColumnGeneration")
    { // no existing paths
      // m_mmdta == mmdta
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_fixed_car->m_path_table
               == nullptr);
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_fixed_truck->m_path_table
               == nullptr);
      IAssert (m_truck_path_table == nullptr);
      auto *_path_table = new Path_Table ();
      dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_fixed_car->m_path_table
        = _path_table;
      dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_fixed_truck->m_path_table
        = _path_table;
      m_truck_path_table = _path_table;
    }
  else if (m_mmdta_config->get_string ("routing_type")
           == "Multimodal_DUE_ColumnGeneration")
    {
      // m_mmdta != mmdta
      if (m_truck_path_table != nullptr)
        { // existing paths
          IAssert (
            dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
              ->m_routing_fixed_car->m_path_table
            != nullptr);
          IAssert (
            dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
              ->m_routing_fixed_truck->m_path_table
            != nullptr);
        }
      else
        { // no existing paths
          IAssert (
            dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
              ->m_routing_fixed_car->m_path_table
            == nullptr);
          IAssert (
            dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
              ->m_routing_fixed_truck->m_path_table
            == nullptr);
          auto *_path_table = new Path_Table ();
          dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_fixed_car->m_path_table
            = _path_table;
          dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_fixed_truck->m_path_table
            = _path_table;
        }
    }
  else
    { // existing paths
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_fixed_car->m_path_table
               != nullptr);
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_fixed_truck->m_path_table
               != nullptr);
      IAssert (m_truck_path_table != nullptr);
    }
  m_driving_path_table
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_fixed_car->m_path_table;
  if (m_mmdta_config->get_string ("routing_type")
        == "Multimodal_Hybrid_ColumnGeneration"
      || m_mmdta_config->get_string ("routing_type") == "Multimodal_Hybrid")
    {
      IAssert (m_mmdta == mmdta && m_truck_path_table == m_driving_path_table);
    }
  else
    {
      IAssert (m_mmdta != mmdta && m_truck_path_table != m_driving_path_table);
    }

  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_Hybrid_ColumnGeneration")
    { // no existing paths
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_car_pnr_fixed->m_pnr_path_table
               == nullptr);
      dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_car_pnr_fixed->m_pnr_path_table
        = new PnR_Path_Table ();

      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_passenger_fixed->m_bustransit_path_table
               == nullptr);
      dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_passenger_fixed->m_bustransit_path_table
        = new Path_Table ();
    }
  else if (m_mmdta_config->get_string ("routing_type")
           == "Multimodal_DUE_ColumnGeneration")
    {
      // may have exisiting paths
      if (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_car_pnr_fixed->m_pnr_path_table
          == nullptr)
        { // no existing paths
          dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_car_pnr_fixed->m_pnr_path_table
            = new PnR_Path_Table ();
        }
      if (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_passenger_fixed->m_bustransit_path_table
          == nullptr)
        { // no existing paths
          dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
            ->m_routing_passenger_fixed->m_bustransit_path_table
            = new Path_Table ();
        }
    }
  else
    {
      // existing paths
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_car_pnr_fixed->m_pnr_path_table
               != nullptr);
      IAssert (dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
                 ->m_routing_passenger_fixed->m_bustransit_path_table
               != nullptr);
    }

  m_pnr_path_table
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_car_pnr_fixed->m_pnr_path_table;
  m_bustransit_path_table
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing)
        ->m_routing_passenger_fixed->m_bustransit_path_table;

  clear_multimodal_path_table_buffer ();
  IAssert (m_driving_path_table != nullptr && m_driving_path_table->empty ()
           && m_pnr_path_table != nullptr && m_pnr_path_table->empty ()
           && m_bustransit_path_table != nullptr
           && m_bustransit_path_table->empty ());

  for (auto _o_it : *m_passenger_path_table)
    {
      for (auto _d_it : *_o_it.second)
        {
          // driving
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (driving)
                   ->second)
            {
              if (m_driving_path_table->find (_o_it.first)
                  == m_driving_path_table->end ())
                {
                  auto *_new_map
                    = new std::unordered_map<TInt, MNM_Pathset *> ();
                  m_driving_path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                                      *> (_o_it.first, _new_map));
                }
              if (m_driving_path_table->find (_o_it.first)
                    ->second->find (_d_it.first)
                  == m_driving_path_table->find (_o_it.first)->second->end ())
                {
                  auto *_new_pathset = new MNM_Pathset ();
                  m_driving_path_table->find (_o_it.first)
                    ->second->insert (
                      std::pair<TInt, MNM_Pathset *> (_d_it.first,
                                                      _new_pathset));
                }
              _driving_pathset = m_driving_path_table->find (_o_it.first)
                                   ->second->find (_d_it.first)
                                   ->second;

              // add fixed truck paths
              if (m_truck_path_table != nullptr)
                {
                  if (m_truck_path_table->find (_o_it.first)
                        != m_truck_path_table->end ()
                      && m_truck_path_table->find (_o_it.first)
                             ->second->find (_d_it.first)
                           != m_truck_path_table->find (_o_it.first)
                                ->second->end ())
                    {
                      _truck_pathset = m_truck_path_table->find (_o_it.first)
                                         ->second->find (_d_it.first)
                                         ->second;

                      for (auto _truck_path : _truck_pathset->m_path_vec)
                        {
                          _is_in = false;
                          for (auto _tmp_path : _driving_pathset->m_path_vec)
                            {
                              if (*_tmp_path == *_truck_path)
                                {
                                  IAssert (_tmp_path->m_buffer_length
                                           == 2 * m_total_assign_inter);
                                  for (int i = 0; i < m_total_assign_inter; ++i)
                                    { // _tmp_path -> m_buffer has a length of
                                      // 2*m_total_assign_inter
                                      _tmp_path
                                        ->m_buffer[m_total_assign_inter + i]
                                        += _truck_path
                                             ->m_buffer[m_total_assign_inter
                                                        + i];
                                    }
                                  _is_in = true;
                                  break;
                                }
                            }
                          if (!_is_in)
                            {
                              MNM_Path *_path = new MNM_Path ();
                              _path->m_path_ID
                                = (int) _driving_pathset->m_path_vec.size ();
                              _path->m_node_vec = _truck_path->m_node_vec;
                              _path->m_link_vec = _truck_path->m_link_vec;
                              _path->allocate_buffer (2 * m_total_assign_inter);
                              for (int i = 0; i < _path->m_buffer_length; ++i)
                                { // _tmp_path -> m_buffer has a length of
                                  // 2*m_total_assign_inter
                                  _path->m_buffer[i] = _truck_path->m_buffer[i];
                                }
                              _driving_pathset->m_path_vec.push_back (_path);
                            }
                        }
                    }
                }

              // driving
              if (_d_it.second->find (driving) != _d_it.second->end ())
                {
                  _passenger_pathset = _d_it.second->find (driving)->second;
                  for (auto _passenger_path : _passenger_pathset->m_path_vec)
                    {
                      _passenger_path_driving
                        = dynamic_cast<MNM_Passenger_Path_Driving *> (
                          _passenger_path);
                      _num_people = _passenger_path_driving->m_num_people;
                      _driving_path = _passenger_path_driving->m_path;
                      _is_in = false;
                      for (auto _tmp_path : _driving_pathset->m_path_vec)
                        {
                          if (*_tmp_path == *_driving_path)
                            {
                              for (int i = 0; i < m_total_assign_inter; ++i)
                                { // _tmp_path -> m_buffer has a length of
                                  // 2*m_total_assign_inter
                                  _tmp_path->m_buffer[i]
                                    += _passenger_path_driving->m_buffer[i]
                                       / _num_people;
                                }
                              _is_in = true;
                              break;
                            }
                        }
                      if (!_is_in)
                        {
                          //                            if (_driving_path ->
                          //                            m_buffer != nullptr)
                          //                            free(_driving_path ->
                          //                            m_buffer); _driving_path
                          //                            -> allocate_buffer(2 *
                          //                            m_total_assign_inter);
                          //                            for (int i=0; i <
                          //                            m_total_assign_inter;
                          //                            ++i) { // _tmp_path ->
                          //                            m_buffer has a length of
                          //                            2*m_total_assign_inter
                          //                                _driving_path ->
                          //                                m_buffer[i] =
                          //                                _passenger_path_driving
                          //                                -> m_buffer[i] /
                          //                                _num_people;
                          //                            }
                          //                            _driving_pathset ->
                          //                            m_path_vec.push_back(_driving_path);

                          MNM_Path *_path = new MNM_Path ();
                          _path->m_path_ID
                            = (int) _driving_pathset->m_path_vec.size ();
                          _path->m_node_vec = _driving_path->m_node_vec;
                          _path->m_link_vec = _driving_path->m_link_vec;
                          _path->allocate_buffer (2 * m_total_assign_inter);
                          for (int i = 0; i < m_total_assign_inter; ++i)
                            { // _tmp_path -> m_buffer has a length of
                              // 2*m_total_assign_inter
                              _path->m_buffer[i]
                                = _passenger_path_driving->m_buffer[i]
                                  / _num_people;
                            }
                          _driving_pathset->m_path_vec.push_back (_path);
                        }
                    }
                }
            }

          // pnr
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (pnr)
                   ->second)
            {
              if (m_pnr_path_table->find (_o_it.first)
                  == m_pnr_path_table->end ())
                {
                  auto *_new_map
                    = new std::unordered_map<TInt, MNM_PnR_Pathset *> ();
                  m_pnr_path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_PnR_Pathset *>
                                      *> (_o_it.first, _new_map));
                }
              if (m_pnr_path_table->find (_o_it.first)
                    ->second->find (_d_it.first)
                  == m_pnr_path_table->find (_o_it.first)->second->end ())
                {
                  auto *_new_pathset = new MNM_PnR_Pathset ();
                  m_pnr_path_table->find (_o_it.first)
                    ->second->insert (
                      std::pair<TInt, MNM_PnR_Pathset *> (_d_it.first,
                                                          _new_pathset));
                }
              _pnr_pathset = m_pnr_path_table->find (_o_it.first)
                               ->second->find (_d_it.first)
                               ->second;

              if (_d_it.second->find (pnr) != _d_it.second->end ())
                {
                  _passenger_pathset = _d_it.second->find (pnr)->second;
                  for (auto _passenger_path : _passenger_pathset->m_path_vec)
                    {
                      _passenger_path_pnr
                        = dynamic_cast<MNM_Passenger_Path_PnR *> (
                          _passenger_path);
                      _num_people
                        = _passenger_path_pnr->m_driving_part->m_num_people;
                      _pnr_path = _passenger_path_pnr->m_path;
                      _is_in = false;
                      for (auto _tmp_path : _pnr_pathset->m_path_vec)
                        {
                          if (_pnr_path->is_equal (_tmp_path))
                            {
                              for (int i = 0; i < m_total_assign_inter; ++i)
                                { // _tmp_path -> m_buffer has a length of
                                  // m_total_assign_inter
                                  _tmp_path->m_buffer[i]
                                    += _passenger_path_pnr->m_buffer[i]
                                       / _num_people;
                                }
                              _is_in = true;
                              break;
                            }
                        }
                      if (!_is_in)
                        {
                          //                            if (_pnr_path ->
                          //                            m_buffer != nullptr)
                          //                            free(_pnr_path ->
                          //                            m_buffer); _pnr_path ->
                          //                            allocate_buffer(m_total_assign_inter);
                          //                            for (int i=0; i <
                          //                            m_total_assign_inter;
                          //                            ++i) { // _tmp_path ->
                          //                            m_buffer has a length of
                          //                            m_total_assign_inter
                          //                                _pnr_path ->
                          //                                m_buffer[i] =
                          //                                _passenger_path_pnr
                          //                                -> m_buffer[i] /
                          //                                _num_people;
                          //                            }
                          //                            _pnr_pathset ->
                          //                            m_path_vec.push_back(_pnr_path);

                          MNM_Path *_driving_path_tmp = new MNM_Path ();
                          _driving_path_tmp->m_node_vec
                            = _pnr_path->m_driving_path->m_node_vec;
                          _driving_path_tmp->m_link_vec
                            = _pnr_path->m_driving_path->m_link_vec;

                          MNM_Path *_transit_path_tmp = new MNM_Path ();
                          _transit_path_tmp->m_node_vec
                            = _pnr_path->m_transit_path->m_node_vec;
                          _transit_path_tmp->m_link_vec
                            = _pnr_path->m_transit_path->m_link_vec;

                          MNM_PnR_Path *_path
                            = new MNM_PnR_Path ((int) _pnr_pathset->m_path_vec
                                                  .size (),
                                                _pnr_path->m_mid_parking_lot_ID,
                                                _pnr_path->m_mid_dest_node_ID,
                                                _driving_path_tmp,
                                                _transit_path_tmp);

                          _path->allocate_buffer (m_total_assign_inter);
                          for (int i = 0; i < m_total_assign_inter; ++i)
                            { // _tmp_path -> m_buffer has a length of
                              // m_total_assign_inter
                              _path->m_buffer[i]
                                = _passenger_path_pnr->m_buffer[i]
                                  / _num_people;
                            }
                          _pnr_pathset->m_path_vec.push_back (_path);
                        }
                    }
                }
            }

          // bus transit
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_o_it.first)
                   ->second.find (_d_it.first)
                   ->second.find (transit)
                   ->second)
            {
              if (m_bustransit_path_table->find (_o_it.first)
                  == m_bustransit_path_table->end ())
                {
                  auto *_new_map
                    = new std::unordered_map<TInt, MNM_Pathset *> ();
                  m_bustransit_path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                                      *> (_o_it.first, _new_map));
                }
              if (m_bustransit_path_table->find (_o_it.first)
                    ->second->find (_d_it.first)
                  == m_bustransit_path_table->find (_o_it.first)
                       ->second->end ())
                {
                  auto *_new_pathset = new MNM_Pathset ();
                  m_bustransit_path_table->find (_o_it.first)
                    ->second->insert (
                      std::pair<TInt, MNM_Pathset *> (_d_it.first,
                                                      _new_pathset));
                }
              _bustransit_pathset = m_bustransit_path_table->find (_o_it.first)
                                      ->second->find (_d_it.first)
                                      ->second;

              if (_d_it.second->find (transit) != _d_it.second->end ())
                {
                  _passenger_pathset = _d_it.second->find (transit)->second;
                  for (auto _passenger_path : _passenger_pathset->m_path_vec)
                    {
                      _passenger_path_bus
                        = dynamic_cast<MNM_Passenger_Path_Bus *> (
                          _passenger_path);
                      _bustransit_path = _passenger_path_bus->m_path;
                      _is_in = false;
                      for (auto _tmp_path : _bustransit_pathset->m_path_vec)
                        {
                          if (*_bustransit_path == *_tmp_path)
                            {
                              for (int i = 0; i < m_total_assign_inter; ++i)
                                { // _tmp_path -> m_buffer has a length of
                                  // m_total_assign_inter
                                  _tmp_path->m_buffer[i]
                                    += _passenger_path_bus->m_buffer[i];
                                }
                              _is_in = true;
                              break;
                            }
                        }
                      if (!_is_in)
                        {
                          //                            if (_bustransit_path ->
                          //                            m_buffer != nullptr)
                          //                            free(_bustransit_path ->
                          //                            m_buffer);
                          //                            _bustransit_path ->
                          //                            allocate_buffer(m_total_assign_inter);
                          //                            for (int i=0; i <
                          //                            m_total_assign_inter;
                          //                            ++i) { // _tmp_path ->
                          //                            m_buffer has a length of
                          //                            m_total_assign_inter
                          //                                _bustransit_path ->
                          //                                m_buffer[i] =
                          //                                _passenger_path_bus
                          //                                -> m_buffer[i];
                          //                            }
                          //                            _bustransit_pathset ->
                          //                            m_path_vec.push_back(_bustransit_path);

                          MNM_Path *_path = new MNM_Path ();
                          _path->m_path_ID
                            = (int) _bustransit_pathset->m_path_vec.size ();
                          _path->m_node_vec = _bustransit_path->m_node_vec;
                          _path->m_link_vec = _bustransit_path->m_link_vec;
                          _path->allocate_buffer (m_total_assign_inter);
                          for (int i = 0; i < m_total_assign_inter; ++i)
                            { // _tmp_path -> m_buffer has a length of
                              // m_total_assign_inter
                              _path->m_buffer[i]
                                = _passenger_path_bus->m_buffer[i];
                            }
                          _bustransit_pathset->m_path_vec.push_back (_path);
                        }
                    }
                }
            }
        }
    }

  return 0;
}

int
MNM_MM_Due::build_link_cost_map (MNM_Dta_Multimodal *mmdta,
                                 bool with_congestion_indicator)
{
  MNM_Dlink_Multiclass *_link;
  MNM_Transit_Link *_transitlink;
  std::cout << "\n********************** Begin build_link_cost_map  "
               "**********************\n";

  for (auto _link_it : mmdta->m_link_factory->m_link_map)
    {
      // #pragma omp task
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
      std::cout << "********************** build_link_cost_map driving link "
                << _link_it.first << " **********************\n";
      for (int i = 0; i < m_total_loading_inter; i++)
        {
          m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time_car (_link, TFlt (i + 1), m_unit_time,
                                 m_total_loading_inter); // intervals
          m_link_tt_map_truck[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time_truck (_link, TFlt (i + 1), m_unit_time,
                                   m_total_loading_inter); // intervals
          m_link_cost_map[_link_it.first][i]
            = m_vot * m_link_tt_map[_link_it.first][i] + _link->m_toll_car;
          m_link_cost_map_truck[_link_it.first][i]
            = m_vot * m_link_tt_map_truck[_link_it.first][i]
              + _link->m_toll_truck;
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_link_tt_map[_link_it.first][i] << "\n"; std::cout <<
          // "car in" << "\n"; std::cout << _link -> m_N_in_car -> to_string()
          // << "\n"; std::cout << "car out" << "\n"; std::cout << _link ->
          // m_N_out_car -> to_string() << "\n"; std::cout << "truck in" <<
          // "\n"; std::cout << _link -> m_N_in_truck -> to_string() << "\n";
          // std::cout << "truck out" << "\n";
          // std::cout << _link -> m_N_out_truck -> to_string() << "\n";
          if (with_congestion_indicator)
            {
              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " << m_link_tt_map[_link_it.first][i]
              // << ", fftt: " << _link -> get_link_freeflow_tt_car() /
              // m_unit_time << "\n"; std::cout << "truck, interval: " << i <<
              // ", link: " << _link_it.first << ", tt: " <<
              // m_link_tt_map_truck[_link_it.first][i] << ", fftt: " << _link
              // -> get_link_freeflow_tt_truck() / m_unit_time << "\n";

              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " << m_link_tt_map[_link_it.first][i]
              // << ", fftt: " << _link -> get_link_freeflow_tt_loading_car() <<
              // "\n"; std::cout << "truck, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " <<
              // m_link_tt_map_truck[_link_it.first][i] << ", fftt: " << _link
              // -> get_link_freeflow_tt_loading_truck() << "\n";
              if (m_link_congested_car.find (_link_it.first)
                  == m_link_congested_car.end ())
                {
                  m_link_congested_car[_link_it.first]
                    = new bool[m_total_loading_inter];
                }
              m_link_congested_car[_link_it.first][i]
                = m_link_tt_map[_link_it.first][i]
                  > _link->get_link_freeflow_tt_loading_car ();
              if (m_link_congested_truck.find (_link_it.first)
                  == m_link_congested_truck.end ())
                {
                  m_link_congested_truck[_link_it.first]
                    = new bool[m_total_loading_inter];
                }
              m_link_congested_truck[_link_it.first][i]
                = m_link_tt_map_truck[_link_it.first][i]
                  > _link->get_link_freeflow_tt_loading_truck ();

              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", congested?: " <<
              // m_link_congested_car[_link_it.first][i] << "\n"; std::cout <<
              // "truck, interval: " << i << ", link: " << _link_it.first << ",
              // congested?: " << m_link_congested_car[_link_it.first][i] <<
              // "\n";
            }
        }
    }
  for (auto _link_it : mmdta->m_transitlink_factory->m_transit_link_map)
    {
      // #pragma omp task
      _transitlink = _link_it.second;
      std::cout
        << "********************** build_link_cost_map bus transit link "
        << _link_it.first << " **********************\n";
      for (int i = 0; i < m_total_loading_inter; i++)
        {
          if (_transitlink->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
            {
              m_transitlink_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
                get_travel_time_walking (dynamic_cast<MNM_Walking_Link *> (
                                           _transitlink),
                                         TFlt (i + 1), m_unit_time,
                                         m_total_loading_inter); // intervals
            }
          else if (_transitlink->m_link_type == MNM_TYPE_BUS_MULTIMODAL)
            {
              m_transitlink_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
                get_travel_time_bus (dynamic_cast<MNM_Bus_Link *> (
                                       _transitlink),
                                     TFlt (i + 1), m_unit_time,
                                     m_total_loading_inter,
                                     mmdta->m_explicit_bus, false,
                                     !mmdta->m_explicit_bus); // intervals
            }
          else
            {
              throw std::runtime_error ("Wrong transit link type");
            }
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_transitlink_tt_map[_link_it.first][i] << "\n";

          m_transitlink_cost_map[_link_it.first][i]
            = m_vot * m_transitlink_tt_map[_link_it.first][i];

          if (with_congestion_indicator)
            {
              if (m_transitlink_congested_passenger.find (_link_it.first)
                  == m_transitlink_congested_passenger.end ())
                {
                  m_transitlink_congested_passenger[_link_it.first]
                    = new bool[m_total_loading_inter];
                }
              // TODO: walking link will not be congested, bus link may has
              // infinity values
              m_transitlink_congested_passenger[_link_it.first][i] = false;
              // m_transitlink_congested_passenger[_link_it.first][i] =
              // m_transitlink_cost_map[_link_it.first][i] > _transitlink ->
              // m_fftt / m_unit_time; std::cout << "passenger, interval: " << i
              // << ", link: " << _link_it.first << ", congested?: " <<
              // m_transitlink_congested_passenger[_link_it.first][i] << "\n";
            }
        }
    }

  std::cout << "********************** End build_link_cost_map  "
               "**********************\n";
  return 0;
}

int
MNM_MM_Due::get_link_queue_dissipated_time (MNM_Dta_Multimodal *mmdta)
{
  // suppose m_link_congested_car, m_link_congested_truck, and
  // m_transitlink_congested_passenger are constructed already in
  // build_link_cost_map()
  MNM_Dlink_Multiclass *_link;
  // MNM_Transit_Link *_transitlink;
  bool _flg;
  std::cout << "\n********************** Begin get_link_queue_dissipated_time "
               "**********************\n";
  // TODO: switch t and link order
  for (int i = 0; i < m_total_loading_inter; i++)
    {
      // std::cout << "********************** get_link_queue_dissipated_time
      // interval " << i << " **********************\n";
      for (auto _link_it : mmdta->m_link_factory->m_link_map)
        {
          // ************************** car **************************
          if (m_queue_dissipated_time_car.find (_link_it.first)
              == m_queue_dissipated_time_car.end ())
            {
              m_queue_dissipated_time_car[_link_it.first]
                = new int[m_total_loading_inter];
            }
          if (m_link_congested_car[_link_it.first][i])
            {
              if (i == m_total_loading_inter - 1)
                {
                  m_queue_dissipated_time_car[_link_it.first][i]
                    = m_total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < m_total_loading_inter; k++)
                    {
                      if (m_link_congested_car[_link_it.first][k - 1]
                          && !m_link_congested_car[_link_it.first][k])
                        {
                          m_queue_dissipated_time_car[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time_car[_link_it.first][i]
                        = m_total_loading_inter;
                    }
                }
            }
          else
            {
              _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
              if (MNM_Ults::
                    approximate_equal (m_link_tt_map[_link_it.first][i],
                                       (float) _link
                                         ->get_link_freeflow_tt_loading_car ()))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
                      != nullptr)
                    {
                      // TODO: use spline to interpolate the N_out and extract
                      // the deriviative (out flow rate) and compare it with the
                      // capacity
                      // https://kluge.in-chemnitz.de/opensource/spline/spline.h
                      // tk::spline s;
                      // s.set_boundary(tk::spline::second_deriv, 0.0,
                      //                tk::spline::second_deriv, 0.0);
                      // s.set_points(X,Y,tk::spline::cspline);
                      // s.make_monotonic();
                      // s.deriv(1, X[i])
                      TFlt _outflow_rate
                        = MNM_DTA_GRADIENT::get_departure_cc_slope_car (
                          _link,
                          TFlt (
                            i
                            + (int) _link->get_link_freeflow_tt_loading_car ()),
                          TFlt (
                            i + (int) _link->get_link_freeflow_tt_loading_car ()
                            + 1)); // veh / 5s
                      TFlt _cap
                        = dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
                            ->m_cell_array.back ()
                            ->m_flow_cap_car
                          * m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * m_mmdta
                                                             ->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * m_mmdta
                                                             ->m_flow_scalar)))
                        {
                          if (i == m_total_loading_inter - 1)
                            {
                              m_queue_dissipated_time_car[_link_it.first][i]
                                = m_total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < m_total_loading_inter;
                                   k++)
                                {
                                  if (m_link_congested_car[_link_it.first]
                                                          [k - 1]
                                      && !m_link_congested_car[_link_it.first]
                                                              [k])
                                    {
                                      m_queue_dissipated_time_car[_link_it
                                                                    .first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time_car[_link_it.first][i]
                                    = m_total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time_car[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq_Multimodal *> (_link)
                           != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time_car[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "MNM_MM_Due::get_link_queue_dissipated_time, Link type "
                        "not implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time_car[_link_it.first][i] = i;
                  throw std::runtime_error (
                    "MNM_MM_Due::get_link_queue_dissipated_time, Link travel "
                    "time less than fftt");
                }
              // m_queue_dissipated_time_car[_link_it.first][i] = i;
            }

          // ************************** truck **************************
          if (m_queue_dissipated_time_truck.find (_link_it.first)
              == m_queue_dissipated_time_truck.end ())
            {
              m_queue_dissipated_time_truck[_link_it.first]
                = new int[m_total_loading_inter];
            }
          if (m_link_congested_truck[_link_it.first][i])
            {
              if (i == m_total_loading_inter - 1)
                {
                  m_queue_dissipated_time_truck[_link_it.first][i]
                    = m_total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < m_total_loading_inter; k++)
                    {
                      if (m_link_congested_truck[_link_it.first][k - 1]
                          && !m_link_congested_truck[_link_it.first][k])
                        {
                          m_queue_dissipated_time_truck[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time_truck[_link_it.first][i]
                        = m_total_loading_inter;
                    }
                }
            }
          else
            {
              _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
              if (
                MNM_Ults::
                  approximate_equal (m_link_tt_map_truck[_link_it.first][i],
                                     (float) _link
                                       ->get_link_freeflow_tt_loading_truck ()))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
                      != nullptr)
                    {
                      // TODO: use spline to interpolate the N_out and extract
                      // the deriviative (out flow rate) and compare it with the
                      // capacity
                      // https://kluge.in-chemnitz.de/opensource/spline/spline.h
                      // tk::spline s;
                      // s.set_boundary(tk::spline::second_deriv, 0.0,
                      //                tk::spline::second_deriv, 0.0);
                      // s.set_points(X,Y,tk::spline::cspline);
                      // s.make_monotonic();
                      // s.deriv(1, X[i])
                      TFlt _outflow_rate
                        = MNM_DTA_GRADIENT::get_departure_cc_slope_truck (
                          _link,
                          TFlt (i
                                + (int) _link
                                    ->get_link_freeflow_tt_loading_truck ()),
                          TFlt (
                            i
                            + (int) _link->get_link_freeflow_tt_loading_truck ()
                            + 1)); // veh / 5s
                      TFlt _cap
                        = dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
                            ->m_cell_array.back ()
                            ->m_flow_cap_truck
                          * m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * m_mmdta
                                                             ->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * m_mmdta
                                                             ->m_flow_scalar)))
                        {
                          if (i == m_total_loading_inter - 1)
                            {
                              m_queue_dissipated_time_truck[_link_it.first][i]
                                = m_total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < m_total_loading_inter;
                                   k++)
                                {
                                  if (m_link_congested_truck[_link_it.first]
                                                            [k - 1]
                                      && !m_link_congested_truck[_link_it.first]
                                                                [k])
                                    {
                                      m_queue_dissipated_time_truck[_link_it
                                                                      .first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time_truck[_link_it.first]
                                                               [i]
                                    = m_total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time_truck[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq_Multimodal *> (_link)
                           != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time_truck[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "MNM_MM_Due::get_link_queue_dissipated_time, Link type "
                        "not implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time_truck[_link_it.first][i] = i;
                  throw std::runtime_error (
                    "MNM_MM_Due::get_link_queue_dissipated_time, Link travel "
                    "time less than fftt");
                }
              // m_queue_dissipated_time_truck[_link_it.first][i] = i;
            }
        }

      // ************************** passenger **************************
      // TODO: bus, waiting, infinity values
      for (auto _link_it : mmdta->m_transitlink_factory->m_transit_link_map)
        {
          if (m_queue_dissipated_time_passenger.find (_link_it.first)
              == m_queue_dissipated_time_passenger.end ())
            {
              m_queue_dissipated_time_passenger[_link_it.first]
                = new int[m_total_loading_inter];
            }
          if (m_transitlink_congested_passenger[_link_it.first][i])
            {
              if (i == m_total_loading_inter - 1)
                {
                  m_queue_dissipated_time_passenger[_link_it.first][i]
                    = m_total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < m_total_loading_inter; k++)
                    {
                      if (m_transitlink_congested_passenger[_link_it.first]
                                                           [k - 1]
                          && !m_transitlink_congested_passenger[_link_it.first]
                                                               [k])
                        {
                          m_queue_dissipated_time_passenger[_link_it.first][i]
                            = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time_passenger[_link_it.first][i]
                        = m_total_loading_inter;
                    }
                }
            }
          else
            {
              m_queue_dissipated_time_passenger[_link_it.first][i] = i;
            }
        }
    }
  std::cout << "********************** End get_link_queue_dissipated_time "
               "**********************\n";
  return 0;
}

TFlt
MNM_MM_Due::compute_total_passenger_demand (MNM_Origin *orig,
                                            MNM_Destination *dest,
                                            TInt total_assign_inter)
{
  TFlt _tot_dmd = 0.0;
  for (int i = 0; i < total_assign_inter (); ++i)
    {
      _tot_dmd += m_passenger_demand[orig->m_origin_node->m_node_ID]
                                    [dest->m_dest_node->m_node_ID][i];
    }
  return _tot_dmd;
}

TFlt
MNM_MM_Due::compute_total_passenger_demand_for_one_mode (int mode,
                                                         TInt origin_node_ID,
                                                         TInt dest_node_ID,
                                                         TInt assign_inter)
{
  if (m_passenger_path_table->find (origin_node_ID)
        == m_passenger_path_table->end ()
      || m_passenger_path_table->find (origin_node_ID)
             ->second->find (dest_node_ID)
           == m_passenger_path_table->find (origin_node_ID)->second->end ()
      || m_passenger_path_table->find (origin_node_ID)
             ->second->find (dest_node_ID)
             ->second->find (mode)
           == m_passenger_path_table->find (origin_node_ID)
                ->second->find (dest_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "No such combination of origin, destination, and mode");
    }
  TFlt _tot_dmd = 0;
  MNM_Passenger_Pathset *_pathset
    = m_passenger_path_table->find (origin_node_ID)
        ->second->find (dest_node_ID)
        ->second->find (mode)
        ->second;
  IAssert (assign_inter < m_total_assign_inter);
  for (auto _path : _pathset->m_path_vec)
    {
      _tot_dmd += _path->m_buffer[assign_inter];
    }

  return _tot_dmd;
}

TFlt
MNM_MM_Due::get_disutility (int mode, TFlt travel_cost,
                            TFlt total_demand_one_mode)
{
  TFlt alpha;
  if (mode == driving)
    {
      alpha = m_alpha1_driving;
    }
  else if (mode == transit)
    {
      alpha = m_alpha1_transit;
    }
  else if (mode == pnr)
    {
      alpha = m_alpha1_pnr;
    }
  else
    {
      throw std::runtime_error ("Mode not implemented\n");
    }
  return travel_cost + (alpha + log (total_demand_one_mode + 1e-9)) / m_beta1;
}

TFlt
MNM_MM_Due::compute_merit_function (MNM_Dta_Multimodal *mmdta)
{
  TFlt _dis_utl, _lowest_dis_utl;
  TFlt _total_gap = 0.0;
  TFlt _min_flow_cost = 0.0;
  for (auto _o_it : *m_passenger_path_table)
    {
      for (auto _d_it : *(_o_it.second))
        {
          _lowest_dis_utl = TFlt (std::numeric_limits<double>::infinity ());
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              for (auto _m_it : *(_d_it.second))
                {
                  for (auto _path : _m_it.second->m_path_vec)
                    { // get lowest disutility route at time interval _col and
                      // current OD pair
                      if (_path->m_buffer[_col] > 0)
                        {
                          _dis_utl = _path->m_travel_disutility_vec[_col];
                          if (_dis_utl < _lowest_dis_utl)
                            _lowest_dis_utl = _dis_utl;
                        }
                    }
                }
            }
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              for (auto _m_it : *(_d_it.second))
                {
                  for (auto _path : _m_it.second->m_path_vec)
                    {
                      if (_path->m_buffer[_col] > 0)
                        {
                          _dis_utl = _path->m_travel_disutility_vec[_col];
                          _total_gap += std::abs (_dis_utl - _lowest_dis_utl)
                                        * _path->m_buffer[_col];
                          // _total_gap += (_dis_utl - _lowest_dis_utl) *
                          // _path->m_buffer[_col];
                          _min_flow_cost += std::abs (_lowest_dis_utl)
                                            * _path->m_buffer[_col];
                        }
                    }
                }
            }
        }
    }
  return _total_gap / MNM_Ults::max (_min_flow_cost, 1e-6);
}

TFlt
MNM_MM_Due::compute_merit_function_fixed_departure_time_choice (
  MNM_Dta_Multimodal *mmdta)
{
  TFlt _dis_utl, _lowest_dis_utl;
  TFlt _total_gap = 0.0;
  TFlt _min_flow_cost = 0.0;
  for (auto _o_it : *m_passenger_path_table)
    {
      for (auto _d_it : *(_o_it.second))
        {
          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _lowest_dis_utl = TFlt (std::numeric_limits<double>::infinity ());
              for (auto _m_it : *(_d_it.second))
                {
                  for (auto _path : _m_it.second->m_path_vec)
                    {
                      if (_path->m_buffer[_col] > 0)
                        {
                          _dis_utl = _path->m_travel_disutility_vec[_col];
                          if (_dis_utl < _lowest_dis_utl)
                            _lowest_dis_utl = _dis_utl;
                        }
                    }
                }
              for (auto _m_it : *(_d_it.second))
                {
                  for (auto _path : _m_it.second->m_path_vec)
                    {
                      if (_path->m_buffer[_col] > 0)
                        {
                          _dis_utl = _path->m_travel_disutility_vec[_col];
                          _total_gap += std::abs (_dis_utl - _lowest_dis_utl)
                                        * _path->m_buffer[_col];
                          _min_flow_cost += std::abs (_lowest_dis_utl)
                                            * _path->m_buffer[_col];
                          // _total_gap += (_dis_utl - _lowest_dis_utl) *
                          // _path->m_buffer[_col]; _min_flow_cost +=
                          // _lowest_dis_utl * _path->m_buffer[_col];
                        }
                    }
                }
            }
        }
    }
  return _total_gap / MNM_Ults::max (_min_flow_cost, 1e-6);
}

std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
MNM_MM_Due::get_best_driving_path (TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree,
                                   MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_tt_parking, _tmp_cost, _tot_dmd_one_mode;
  TInt _assign_inter;
  MNM_Path *_path;
  MNM_Passenger_Path_Driving *_p_path_driving = nullptr;
  MNM_Passenger_Path_Driving *_cur_best_p_path_driving = nullptr;
  MNM_Parking_Lot *_parking_lot
    = dynamic_cast<MNM_Destination_Multimodal *> (
        ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (
           tdsp_tree->m_dest_node_ID))
          ->m_dest)
        ->m_parking_lot;

  for (int i = 0; i < tdsp_tree->m_max_interval; ++i)
    { // tdsp_tree -> m_max_interval = total_loading_interval
      _path = new MNM_Path ();
      _tmp_tt = tdsp_tree->get_tdsp (o_node_ID, i, m_link_tt_map, _path);
      IAssert (_tmp_tt > 0);

      if (_parking_lot != nullptr)
        {
          _tmp_tt_parking
            = mmdta->m_parkinglot_factory->get_parking_lot (_parking_lot->m_ID)
                ->get_cruise_time (TInt (i + _tmp_tt));
        }
      else
        {
          _tmp_tt_parking = 0.;
        }
      std::cout << "interval: " << i
                << ", tdsp_tt: " << _tmp_tt + _tmp_tt_parking << "\n";

      _path->eliminate_cycles ();
      _p_path_driving = new MNM_Passenger_Path_Driving (
        driving, _path, m_vot, m_early_penalty, m_late_penalty, m_target_time,
        1, m_carpool_cost_multiplier, 0.0, _parking_lot,
        m_parking_lot_to_destination_walking_time);
      _path = nullptr;
      IAssert (_p_path_driving->m_path != nullptr);

      // _tmp_cost = _p_path_driving -> get_travel_cost(TFlt(i), mmdta);
      _tmp_cost
        = _p_path_driving->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (driving, o_node_ID,
                                                       tdsp_tree
                                                         ->m_dest_node_ID,
                                                       _assign_inter);
      _tmp_cost = get_disutility (driving, _tmp_cost, _tot_dmd_one_mode);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
          if (_cur_best_p_path_driving != nullptr)
            delete _cur_best_p_path_driving;
          _cur_best_p_path_driving = _p_path_driving;
        }
      else
        {
          delete _p_path_driving;
        }
    }
  IAssert (_cur_best_time >= 0);
  IAssert (_cur_best_p_path_driving != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_driving, _cur_best_time,
                          _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
MNM_MM_Due::get_best_bus_path (TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree,
                               MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_cost, _tot_dmd_one_mode;
  TInt _assign_inter;
  MNM_Path *_path;
  MNM_Passenger_Path_Bus *_p_path_bus = nullptr;
  MNM_Passenger_Path_Bus *_cur_best_p_path_bus = nullptr;

  // find a min cost route
  for (int i = 0; i < m_total_loading_inter; ++i)
    {
      _tmp_cost
        = tdsp_tree->m_dist[o_node_ID][i < (int) tdsp_tree->m_max_interval
                                         ? i
                                         : (int) tdsp_tree->m_max_interval - 1];
      if (std::isinf (_tmp_cost))
        {
          continue;
        }

      _path = new MNM_Path ();
      _tmp_tt = tdsp_tree->get_tdsp (o_node_ID, i, m_transitlink_tt_map, _path);
      IAssert (_tmp_tt > 0);
      std::cout << "interval: " << i << ", tdsp_tt: " << _tmp_tt << "\n";

      _path->eliminate_cycles ();
      _p_path_bus
        = new MNM_Passenger_Path_Bus (transit, _path, m_vot, m_early_penalty,
                                      m_late_penalty, m_target_time, m_bus_fare,
                                      m_bus_inconvenience);
      _path = nullptr;
      IAssert (_p_path_bus->m_path != nullptr);

      // _tmp_cost = _p_path_bus ->get_travel_cost(TFlt(i), mmdta);
      _tmp_cost = _p_path_bus->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                                m_transitlink_tt_map);
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       tdsp_tree
                                                         ->m_dest_node_ID,
                                                       _assign_inter);
      _tmp_cost = get_disutility (transit, _tmp_cost, _tot_dmd_one_mode);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
          if (_cur_best_p_path_bus != nullptr)
            delete _cur_best_p_path_bus;
          _cur_best_p_path_bus = _p_path_bus;
        }
      else
        {
          delete _p_path_bus;
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_bus != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_bus, _cur_best_time, _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
MNM_MM_Due::get_best_pnr_path (
  TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_bus,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
  MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt_driving, _tmp_tt_parking, _tmp_tt_bustransit, _tmp_cost,
    _tot_dmd_one_mode;
  TInt _assign_inter;
  TInt _mid_dest_node_ID;
  MNM_Path *_driving_path;
  MNM_Path *_bustransit_path;
  MNM_PnR_Path *_pnr_path;
  MNM_Passenger_Path_PnR *_p_path_pnr = nullptr;
  MNM_Passenger_Path_PnR *_cur_best_p_path_pnr = nullptr;
  MNM_TDSP_Tree *_tdsp_tree_driving;
  std::vector<MNM_Parking_Lot *> _connected_pnr_parkinglot_vec
    = dynamic_cast<MNM_Destination_Multimodal *> (
        ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (
           tdsp_tree_bus->m_dest_node_ID))
          ->m_dest)
        ->m_connected_pnr_parkinglot_vec;

  // find a min cost route
  for (int i = 0; i < m_total_loading_inter; ++i)
    {
      for (auto _parking_lot : _connected_pnr_parkinglot_vec)
        {
          _mid_dest_node_ID = _parking_lot->m_dest_node->m_node_ID;

          _tdsp_tree_driving
            = tdsp_tree_map_driving.find (_mid_dest_node_ID)->second;
          IAssert (_tdsp_tree_driving != nullptr);
          // _tmp_cost = _tdsp_tree_driving-> m_dist[o_node_ID][i <
          // (int)_tdsp_tree_driving -> m_max_interval ? i :
          // (int)_tdsp_tree_driving -> m_max_interval - 1];
          _driving_path = new MNM_Path ();
          _tmp_tt_driving
            = _tdsp_tree_driving->get_tdsp (o_node_ID, i, m_link_tt_map,
                                            _driving_path);
          IAssert (_tmp_tt_driving > 0);
          _driving_path->eliminate_cycles ();

          _tmp_tt_parking
            = mmdta->m_parkinglot_factory->get_parking_lot (_parking_lot->m_ID)
                ->get_cruise_time (TInt (i + _tmp_tt_driving));

          _tmp_cost = tdsp_tree_bus
                        ->m_dist[_mid_dest_node_ID]
                                [i + int (ceil (_tmp_tt_driving))
                                       + int (ceil (_tmp_tt_parking))
                                     < (int) tdsp_tree_bus->m_max_interval
                                   ? i + int (ceil (_tmp_tt_driving))
                                       + int (ceil (_tmp_tt_parking))
                                   : (int) tdsp_tree_bus->m_max_interval - 1];

          if (std::isinf (_tmp_cost))
            {
              delete _driving_path;
              continue;
            }

          _bustransit_path = new MNM_Path ();
          _tmp_tt_bustransit
            = tdsp_tree_bus->get_tdsp (_mid_dest_node_ID,
                                       TInt (i + int (ceil (_tmp_tt_driving))
                                             + int (ceil (_tmp_tt_parking))),
                                       m_transitlink_tt_map, _bustransit_path);
          IAssert (_tmp_tt_bustransit > 0);

          std::cout << "interval: " << i << ", tdsp_tt: "
                    << _tmp_tt_driving + _tmp_tt_parking + _tmp_tt_bustransit
                    << "\n";

          _bustransit_path->eliminate_cycles ();
          // path_ID = -1 is arbitrary
          _pnr_path
            = new MNM_PnR_Path (-1, _parking_lot->m_ID, _mid_dest_node_ID,
                                _driving_path, _bustransit_path);
          _p_path_pnr
            = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, 0.0, _parking_lot,
                                          m_bus_fare, m_pnr_inconvenience);
          _driving_path = nullptr;
          _bustransit_path = nullptr;
          _pnr_path = nullptr;
          IAssert (_p_path_pnr->m_path != nullptr);

          // _tmp_cost = _p_path_pnr ->get_travel_cost(TFlt(i), mmdta);
          _tmp_cost
            = _p_path_pnr->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
          _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
          if (_assign_inter >= m_total_assign_inter)
            _assign_inter = m_total_assign_inter - 1;
          _tot_dmd_one_mode
            = compute_total_passenger_demand_for_one_mode (pnr, o_node_ID,
                                                           tdsp_tree_bus
                                                             ->m_dest_node_ID,
                                                           _assign_inter);
          _tmp_cost = get_disutility (pnr, _tmp_cost, _tot_dmd_one_mode);
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_pnr != nullptr)
                delete _cur_best_p_path_pnr;
              _cur_best_p_path_pnr = _p_path_pnr;
            }
          else
            {
              delete _p_path_pnr;
            }
        }
    }

  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_pnr != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_pnr, _cur_best_time, _cur_best_cost);
}

std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
MNM_MM_Due::get_best_path (
  TInt o_node_ID, TInt d_node_ID,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_bus,
  MNM_Dta_Multimodal *mmdta)
{
  MNM_TDSP_Tree *_tdsp_tree_driving;
  MNM_TDSP_Tree *_tdsp_tree_bus;
  // <path, loading_inter, cost>
  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt> _path_driving_result;
  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt> _path_bus_result;
  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt> _path_pnr_result;
  // <<path, loading_inter, cost>, mode>
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int> _best
    = std::make_pair (std::make_tuple (nullptr, -1, 0.), -1);

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _tdsp_tree_driving = tdsp_tree_map_driving.find (d_node_ID)->second;
      _path_driving_result
        = get_best_driving_path (o_node_ID, _tdsp_tree_driving, mmdta);
      _best = std::make_pair (_path_driving_result, driving);
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _tdsp_tree_bus = tdsp_tree_map_bus.find (d_node_ID)->second;
      _path_bus_result = get_best_bus_path (o_node_ID, _tdsp_tree_bus, mmdta);
      if (std::get<0> (_path_bus_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_bus_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_bus_result, transit);
                }
              else
                {
                  delete std::get<0> (_path_bus_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_bus_result, transit);
            }
        }
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _tdsp_tree_bus = tdsp_tree_map_bus.find (d_node_ID)->second;
      _path_pnr_result = get_best_pnr_path (o_node_ID, _tdsp_tree_bus,
                                            tdsp_tree_map_driving, mmdta);
      if (std::get<0> (_path_pnr_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_pnr_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_pnr_result, pnr);
                }
              else
                {
                  delete std::get<0> (_path_pnr_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_pnr_result, pnr);
            }
        }
    }
  IAssert (std::get<0> (_best.first) != nullptr);
  return _best;
}

std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
MNM_MM_Due::get_best_existing_driving_path (TInt o_node_ID, TInt d_node_ID,
                                            MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (driving)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_driving_path() no driving path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (driving)
                                      ->second;
  MNM_Passenger_Path_Driving *_p_path_driving = nullptr;
  MNM_Passenger_Path_Driving *_cur_best_p_path_driving = nullptr;
  MNM_Path *_path = nullptr;
  for (int i = 0; i < m_total_loading_inter; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (driving, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
          _tmp_cost = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                                m_transitlink_tt_map);
          _tmp_cost
            = get_disutility (_p_path->m_mode, _tmp_cost, _tot_dmd_one_mode);
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_driving == nullptr
                  || !_cur_best_p_path_driving->is_equal (_p_path))
                {
                  if (_cur_best_p_path_driving != nullptr)
                    delete _cur_best_p_path_driving;
                  _p_path_driving
                    = dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path);
                  _path = new MNM_Path ();
                  _path->m_node_vec = _p_path_driving->m_path->m_node_vec;
                  _path->m_link_vec = _p_path_driving->m_path->m_link_vec;

                  _cur_best_p_path_driving = new MNM_Passenger_Path_Driving (
                    driving, _path, m_vot, m_early_penalty, m_late_penalty,
                    m_target_time, 1, m_carpool_cost_multiplier, 0.0,
                    _p_path_driving->m_parking_lot,
                    m_parking_lot_to_destination_walking_time);
                  _path = nullptr;
                  IAssert (_cur_best_p_path_driving->m_path != nullptr);
                }
            }
        }
    }
  IAssert (_cur_best_time >= 0);
  IAssert (_cur_best_p_path_driving != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_driving, _cur_best_time,
                          _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
MNM_MM_Due::get_best_existing_bus_path (TInt o_node_ID, TInt d_node_ID,
                                        MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (transit)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_bus_path() no bustransit path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (transit)
                                      ->second;
  MNM_Passenger_Path_Bus *_p_path_bus = nullptr;
  MNM_Passenger_Path_Bus *_cur_best_p_path_bus = nullptr;
  MNM_Path *_path = nullptr;
  for (int i = 0; i < m_total_loading_inter; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
          _tmp_cost = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                                m_transitlink_tt_map);
          if (std::isinf (_tmp_cost))
            {
              continue;
            }
          _tmp_cost
            = get_disutility (_p_path->m_mode, _tmp_cost, _tot_dmd_one_mode);
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_bus == nullptr
                  || !_cur_best_p_path_bus->is_equal (_p_path))
                {
                  if (_cur_best_p_path_bus != nullptr)
                    delete _cur_best_p_path_bus;
                  _p_path_bus
                    = dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path);
                  _path = new MNM_Path ();
                  _path->m_node_vec = _p_path_bus->m_path->m_node_vec;
                  _path->m_link_vec = _p_path_bus->m_path->m_link_vec;

                  _cur_best_p_path_bus
                    = new MNM_Passenger_Path_Bus (transit, _path, m_vot,
                                                  m_early_penalty,
                                                  m_late_penalty, m_target_time,
                                                  m_bus_fare,
                                                  m_bus_inconvenience);
                  _path = nullptr;
                  IAssert (_cur_best_p_path_bus->m_path != nullptr);
                }
            }
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_bus != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_bus, _cur_best_time, _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
MNM_MM_Due::get_best_existing_pnr_path (TInt o_node_ID, TInt d_node_ID,
                                        MNM_Dta_Multimodal *mmdta)
{
  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (pnr)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_pnr_path() no pnr path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (pnr)
                                      ->second;
  MNM_Passenger_Path_PnR *_p_path_pnr = nullptr;
  MNM_Passenger_Path_PnR *_cur_best_p_path_pnr = nullptr;
  MNM_Path *_driving_path = nullptr;
  MNM_Path *_bustransit_path = nullptr;
  MNM_PnR_Path *_pnr_path = nullptr;
  for (int i = 0; i < m_total_loading_inter; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
          _tmp_cost = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                                m_transitlink_tt_map);
          if (std::isinf (_tmp_cost))
            {
              continue;
            }
          _tmp_cost
            = get_disutility (_p_path->m_mode, _tmp_cost, _tot_dmd_one_mode);
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_pnr == nullptr
                  || !_cur_best_p_path_pnr->is_equal (_p_path))
                {
                  if (_cur_best_p_path_pnr != nullptr)
                    delete _cur_best_p_path_pnr;
                  _p_path_pnr
                    = dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path);
                  _driving_path = new MNM_Path ();
                  _driving_path->m_node_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_driving_path->m_node_vec;
                  _driving_path->m_link_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_driving_path->m_link_vec;
                  _bustransit_path = new MNM_Path ();
                  _bustransit_path->m_node_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_transit_path->m_node_vec;
                  _bustransit_path->m_link_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_transit_path->m_link_vec;
                  _pnr_path
                    = new MNM_PnR_Path (-1,
                                        _p_path_pnr->m_mid_parking_lot->m_ID,
                                        _p_path_pnr->m_mid_parking_lot
                                          ->m_dest_node->m_node_ID,
                                        _driving_path, _bustransit_path);

                  _cur_best_p_path_pnr
                    = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                                  m_early_penalty,
                                                  m_late_penalty, m_target_time,
                                                  0.0,
                                                  _p_path_pnr
                                                    ->m_mid_parking_lot,
                                                  m_bus_fare,
                                                  m_pnr_inconvenience);
                  _driving_path = nullptr;
                  _bustransit_path = nullptr;
                  _pnr_path = nullptr;
                  IAssert (_cur_best_p_path_pnr->m_path != nullptr);
                }
            }
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_pnr != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_pnr, _cur_best_time, _cur_best_cost);
}

std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
MNM_MM_Due::get_best_existing_path (TInt o_node_ID, TInt d_node_ID,
                                    MNM_Dta_Multimodal *mmdta)
{
  // <path, loading_inter, cost>
  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt> _path_driving_result;
  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt> _path_bus_result;
  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt> _path_pnr_result;
  // <<path, loading_inter, cost>, mode>
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int> _best
    = std::make_pair (std::make_tuple (nullptr, -1, 0.), -1);

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path_driving_result
        = get_best_existing_driving_path (o_node_ID, d_node_ID, mmdta);
      _best = std::make_pair (_path_driving_result, driving);
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path_bus_result
        = get_best_existing_bus_path (o_node_ID, d_node_ID, mmdta);
      if (std::get<0> (_path_bus_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_bus_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_bus_result, transit);
                }
              else
                {
                  delete std::get<0> (_path_bus_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_bus_result, transit);
            }
        }
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _path_pnr_result
        = get_best_existing_pnr_path (o_node_ID, d_node_ID, mmdta);
      if (std::get<0> (_path_pnr_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_pnr_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_pnr_result, pnr);
                }
              else
                {
                  delete std::get<0> (_path_pnr_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_pnr_result, pnr);
            }
        }
    }
  IAssert (std::get<0> (_best.first) != nullptr);
  return _best;
}

std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
MNM_MM_Due::get_best_driving_path_for_single_interval (
  TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree,
  MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_tt_parking, _tmp_cost, _tot_dmd_one_mode;
  TInt _assign_inter;
  MNM_Path *_path;
  MNM_Passenger_Path_Driving *_p_path_driving = nullptr;
  MNM_Passenger_Path_Driving *_cur_best_p_path_driving = nullptr;
  MNM_Parking_Lot *_parking_lot
    = dynamic_cast<MNM_Destination_Multimodal *> (
        ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (
           tdsp_tree->m_dest_node_ID))
          ->m_dest)
        ->m_parking_lot;

  for (int i = interval; i < interval + 1; ++i)
    { // tdsp_tree -> m_max_interval = total_loading_interval

      _path = new MNM_Path ();
      _tmp_tt = tdsp_tree->get_tdsp (o_node_ID, i, m_link_tt_map, _path);
      IAssert (_tmp_tt > 0);

      if (_parking_lot != nullptr)
        {
          _tmp_tt_parking
            = mmdta->m_parkinglot_factory->get_parking_lot (_parking_lot->m_ID)
                ->get_cruise_time (TInt (i + _tmp_tt));
        }
      else
        {
          _tmp_tt_parking = 0.;
        }
      std::cout << "interval: " << i
                << ", tdsp_tt: " << _tmp_tt + _tmp_tt_parking << "\n";

      _path->eliminate_cycles ();
      _p_path_driving = new MNM_Passenger_Path_Driving (
        driving, _path, m_vot, m_early_penalty, m_late_penalty, m_target_time,
        1, m_carpool_cost_multiplier, 0.0, _parking_lot,
        m_parking_lot_to_destination_walking_time);
      _path = nullptr;
      IAssert (_p_path_driving->m_path != nullptr);

      // _tmp_cost = _p_path_driving -> get_travel_cost(TFlt(i), mmdta);
      _tmp_cost
        = _p_path_driving->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (driving, o_node_ID,
                                                       tdsp_tree
                                                         ->m_dest_node_ID,
                                                       _assign_inter);
      _tmp_cost = get_disutility (driving, _tmp_cost, _tot_dmd_one_mode);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
          if (_cur_best_p_path_driving != nullptr)
            delete _cur_best_p_path_driving;
          _cur_best_p_path_driving = _p_path_driving;
        }
      else
        {
          delete _p_path_driving;
        }
    }
  IAssert (_cur_best_time >= 0);
  IAssert (_cur_best_p_path_driving != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_driving, _cur_best_time,
                          _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
MNM_MM_Due::get_best_bus_path_for_single_interval (TInt interval,
                                                   TInt o_node_ID,
                                                   MNM_TDSP_Tree *tdsp_tree,
                                                   MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt, _tmp_cost, _tot_dmd_one_mode;
  TInt _assign_inter;
  MNM_Path *_path;
  MNM_Passenger_Path_Bus *_p_path_bus = nullptr;
  MNM_Passenger_Path_Bus *_cur_best_p_path_bus = nullptr;

  // find a min cost route
  for (int i = interval; i < interval + 1; ++i)
    {
      _tmp_cost
        = tdsp_tree->m_dist[o_node_ID][i < (int) tdsp_tree->m_max_interval
                                         ? i
                                         : (int) tdsp_tree->m_max_interval - 1];

      if (std::isinf (_tmp_cost))
        {
          continue;
        }

      _path = new MNM_Path ();
      _tmp_tt = tdsp_tree->get_tdsp (o_node_ID, i, m_transitlink_tt_map, _path);
      IAssert (_tmp_tt > 0);
      std::cout << "interval: " << i << ", tdsp_tt: " << _tmp_tt << "\n";

      _path->eliminate_cycles ();
      _p_path_bus
        = new MNM_Passenger_Path_Bus (transit, _path, m_vot, m_early_penalty,
                                      m_late_penalty, m_target_time, m_bus_fare,
                                      m_bus_inconvenience);
      _path = nullptr;
      IAssert (_p_path_bus->m_path != nullptr);

      // _tmp_cost = _p_path_bus ->get_travel_cost(TFlt(i), mmdta);
      _tmp_cost = _p_path_bus->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                                m_transitlink_tt_map);
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       tdsp_tree
                                                         ->m_dest_node_ID,
                                                       _assign_inter);
      _tmp_cost = get_disutility (transit, _tmp_cost, _tot_dmd_one_mode);
      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
          if (_cur_best_p_path_bus != nullptr)
            delete _cur_best_p_path_bus;
          _cur_best_p_path_bus = _p_path_bus;
        }
      else
        {
          delete _p_path_bus;
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_bus != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_bus, _cur_best_time, _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
MNM_MM_Due::get_best_pnr_path_for_single_interval (
  TInt interval, TInt o_node_ID, MNM_TDSP_Tree *tdsp_tree_bus,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
  MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TFlt _tmp_tt_driving, _tmp_tt_parking, _tmp_tt_bustransit, _tmp_cost,
    _tot_dmd_one_mode;
  TInt _assign_inter;
  TInt _mid_dest_node_ID;
  MNM_Path *_driving_path;
  MNM_Path *_bustransit_path;
  MNM_PnR_Path *_pnr_path;
  MNM_Passenger_Path_PnR *_p_path_pnr = nullptr;
  MNM_Passenger_Path_PnR *_cur_best_p_path_pnr = nullptr;
  MNM_TDSP_Tree *_tdsp_tree_driving;
  std::vector<MNM_Parking_Lot *> _connected_pnr_parkinglot_vec
    = dynamic_cast<MNM_Destination_Multimodal *> (
        ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (
           tdsp_tree_bus->m_dest_node_ID))
          ->m_dest)
        ->m_connected_pnr_parkinglot_vec;

  // find a min cost route
  for (int i = interval; i < interval + 1; ++i)
    {
      for (auto _parking_lot : _connected_pnr_parkinglot_vec)
        {
          _mid_dest_node_ID = _parking_lot->m_dest_node->m_node_ID;

          _tdsp_tree_driving
            = tdsp_tree_map_driving.find (_mid_dest_node_ID)->second;
          IAssert (_tdsp_tree_driving != nullptr);
          // _tmp_cost = _tdsp_tree_driving-> m_dist[o_node_ID][i <
          // (int)_tdsp_tree_driving -> m_max_interval ? i :
          // (int)_tdsp_tree_driving -> m_max_interval - 1];
          _driving_path = new MNM_Path ();
          _tmp_tt_driving
            = _tdsp_tree_driving->get_tdsp (o_node_ID, i, m_link_tt_map,
                                            _driving_path);
          IAssert (_tmp_tt_driving > 0);
          _driving_path->eliminate_cycles ();

          _tmp_tt_parking
            = mmdta->m_parkinglot_factory->get_parking_lot (_parking_lot->m_ID)
                ->get_cruise_time (TInt (i + _tmp_tt_driving));

          _tmp_cost = tdsp_tree_bus
                        ->m_dist[_mid_dest_node_ID]
                                [i + int (ceil (_tmp_tt_driving))
                                       + int (ceil (_tmp_tt_parking))
                                     < (int) tdsp_tree_bus->m_max_interval
                                   ? i + int (ceil (_tmp_tt_driving))
                                       + int (ceil (_tmp_tt_parking))
                                   : (int) tdsp_tree_bus->m_max_interval - 1];

          if (std::isinf (_tmp_cost))
            {
              delete _driving_path;
              continue;
            }

          // std::cout << _mid_dest_node_ID << "\n";
          // std::cout << "max interval: " << tdsp_tree_bus -> m_max_interval <<
          // " cur time: " << TInt(i + int(ceil(_tmp_tt_driving)) +
          // int(ceil(_tmp_tt_parking))) << "\n";
          _bustransit_path = new MNM_Path ();
          _tmp_tt_bustransit
            = tdsp_tree_bus->get_tdsp (_mid_dest_node_ID,
                                       TInt (i + int (ceil (_tmp_tt_driving))
                                             + int (ceil (_tmp_tt_parking))),
                                       m_transitlink_tt_map, _bustransit_path);
          IAssert (_tmp_tt_bustransit > 0);

          std::cout << "interval: " << i << ", tdsp_tt: "
                    << _tmp_tt_driving + _tmp_tt_parking + _tmp_tt_bustransit
                    << "\n";

          _bustransit_path->eliminate_cycles ();
          // path_ID = -1 is arbitrary
          _pnr_path
            = new MNM_PnR_Path (-1, _parking_lot->m_ID, _mid_dest_node_ID,
                                _driving_path, _bustransit_path);
          _p_path_pnr
            = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                          m_early_penalty, m_late_penalty,
                                          m_target_time, 0.0, _parking_lot,
                                          m_bus_fare, m_pnr_inconvenience);
          _driving_path = nullptr;
          _bustransit_path = nullptr;
          _pnr_path = nullptr;
          IAssert (_p_path_pnr->m_path != nullptr);

          // _tmp_cost = _p_path_pnr ->get_travel_cost(TFlt(i), mmdta);
          _tmp_cost
            = _p_path_pnr->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
          _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
          if (_assign_inter >= m_total_assign_inter)
            _assign_inter = m_total_assign_inter - 1;
          _tot_dmd_one_mode
            = compute_total_passenger_demand_for_one_mode (pnr, o_node_ID,
                                                           tdsp_tree_bus
                                                             ->m_dest_node_ID,
                                                           _assign_inter);
          _tmp_cost = get_disutility (pnr, _tmp_cost, _tot_dmd_one_mode);
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_pnr != nullptr)
                delete _cur_best_p_path_pnr;
              _cur_best_p_path_pnr = _p_path_pnr;
            }
          else
            {
              delete _p_path_pnr;
            }
        }
    }

  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_pnr != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_pnr, _cur_best_time, _cur_best_cost);
}

std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
MNM_MM_Due::get_best_path_for_single_interval (
  TInt interval, TInt o_node_ID, TInt d_node_ID,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_driving,
  std::unordered_map<TInt, MNM_TDSP_Tree *> &tdsp_tree_map_bus,
  MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  MNM_TDSP_Tree *_tdsp_tree_driving;
  MNM_TDSP_Tree *_tdsp_tree_bus;
  // <path, loading_inter, cost>
  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt> _path_driving_result;
  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt> _path_bus_result;
  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt> _path_pnr_result;
  // <<path, loading_inter, cost>, mode>
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int> _best
    = std::make_pair (std::make_tuple (nullptr, -1, 0.), -1);

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _tdsp_tree_driving = tdsp_tree_map_driving.find (d_node_ID)->second;
      _path_driving_result
        = get_best_driving_path_for_single_interval (interval, o_node_ID,
                                                     _tdsp_tree_driving, mmdta);
      printf ("new driving path cost: %f\n",
              (float) std::get<2> (_path_driving_result));
      _best = std::make_pair (_path_driving_result, driving);
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _tdsp_tree_bus = tdsp_tree_map_bus.find (d_node_ID)->second;
      _path_bus_result
        = get_best_bus_path_for_single_interval (interval, o_node_ID,
                                                 _tdsp_tree_bus, mmdta);
      printf ("new bus transit path cost: %f\n",
              (float) std::get<2> (_path_bus_result));
      if (std::get<0> (_path_bus_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_bus_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_bus_result, transit);
                }
              else
                {
                  delete std::get<0> (_path_bus_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_bus_result, transit);
            }
        }
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _tdsp_tree_bus = tdsp_tree_map_bus.find (d_node_ID)->second;
      _path_pnr_result
        = get_best_pnr_path_for_single_interval (interval, o_node_ID,
                                                 _tdsp_tree_bus,
                                                 tdsp_tree_map_driving, mmdta);
      printf ("new pnr path cost: %f\n",
              (float) std::get<2> (_path_pnr_result));
      if (std::get<0> (_path_pnr_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_pnr_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_pnr_result, pnr);
                }
              else
                {
                  delete std::get<0> (_path_pnr_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_pnr_result, pnr);
            }
        }
    }
  IAssert (std::get<0> (_best.first) != nullptr);
  return _best;
}

std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt>
MNM_MM_Due::get_best_existing_driving_path_for_single_interval (
  TInt interval, TInt o_node_ID, TInt d_node_ID, MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (driving)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_driving_path_for_single_interval()"
        " no driving path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (driving)
                                      ->second;
  MNM_Passenger_Path_Driving *_p_path_driving = nullptr;
  MNM_Passenger_Path_Driving *_cur_best_p_path_driving = nullptr;
  MNM_Path *_path = nullptr;
  for (int i = interval; i < interval + 1; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (driving, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          if (_p_path->m_travel_disutility_vec.empty ())
            {
              // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
              _tmp_cost
                = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
              _tmp_cost = get_disutility (_p_path->m_mode, _tmp_cost,
                                          _tot_dmd_one_mode);
            }
          else
            {
              IAssert ((int) _assign_inter
                       < (int) _p_path->m_travel_disutility_vec.size ());
              _tmp_cost = _p_path->m_travel_disutility_vec[_assign_inter];
            }

          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_driving == nullptr
                  || !_cur_best_p_path_driving->is_equal (_p_path))
                {
                  if (_cur_best_p_path_driving != nullptr)
                    delete _cur_best_p_path_driving;
                  _p_path_driving
                    = dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path);
                  _path = new MNM_Path ();
                  _path->m_node_vec = _p_path_driving->m_path->m_node_vec;
                  _path->m_link_vec = _p_path_driving->m_path->m_link_vec;

                  _cur_best_p_path_driving = new MNM_Passenger_Path_Driving (
                    driving, _path, m_vot, m_early_penalty, m_late_penalty,
                    m_target_time, 1, m_carpool_cost_multiplier, 0.0,
                    _p_path_driving->m_parking_lot,
                    m_parking_lot_to_destination_walking_time);
                  _path = nullptr;
                  IAssert (_cur_best_p_path_driving->m_path != nullptr);
                }
            }
        }
    }
  IAssert (_cur_best_time >= 0);
  IAssert (_cur_best_p_path_driving != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_driving, _cur_best_time,
                          _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt>
MNM_MM_Due::get_best_existing_bus_path_for_single_interval (
  TInt interval, TInt o_node_ID, TInt d_node_ID, MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (transit)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_bus_path_for_single_interval() no "
        "bustransit path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (transit)
                                      ->second;
  MNM_Passenger_Path_Bus *_p_path_bus = nullptr;
  MNM_Passenger_Path_Bus *_cur_best_p_path_bus = nullptr;
  MNM_Path *_path = nullptr;
  for (int i = interval; i < interval + 1; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          if (_p_path->m_travel_disutility_vec.empty ())
            {
              // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
              _tmp_cost
                = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
              _tmp_cost = get_disutility (_p_path->m_mode, _tmp_cost,
                                          _tot_dmd_one_mode);
            }
          else
            {
              IAssert ((int) _assign_inter
                       < (int) _p_path->m_travel_disutility_vec.size ());
              _tmp_cost = _p_path->m_travel_disutility_vec[_assign_inter];
            }
          if (std::isinf (_tmp_cost))
            {
              continue;
            }
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_bus == nullptr
                  || !_cur_best_p_path_bus->is_equal (_p_path))
                {
                  if (_cur_best_p_path_bus != nullptr)
                    delete _cur_best_p_path_bus;
                  _p_path_bus
                    = dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path);
                  _path = new MNM_Path ();
                  _path->m_node_vec = _p_path_bus->m_path->m_node_vec;
                  _path->m_link_vec = _p_path_bus->m_path->m_link_vec;

                  _cur_best_p_path_bus
                    = new MNM_Passenger_Path_Bus (transit, _path, m_vot,
                                                  m_early_penalty,
                                                  m_late_penalty, m_target_time,
                                                  m_bus_fare,
                                                  m_bus_inconvenience);
                  _path = nullptr;
                  IAssert (_cur_best_p_path_bus->m_path != nullptr);
                }
            }
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_bus != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_bus, _cur_best_time, _cur_best_cost);
}

std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt>
MNM_MM_Due::get_best_existing_pnr_path_for_single_interval (
  TInt interval, TInt o_node_ID, TInt d_node_ID, MNM_Dta_Multimodal *mmdta)
{
  // input interval is the loading interval
  IAssert (interval + m_mmdta_config->get_int ("assign_frq")
           <= m_total_loading_inter); // tdsp_tree -> m_max_interval =
                                      // total_loading_interval

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _assign_inter;
  TFlt _tmp_cost, _tot_dmd_one_mode;

  if (m_passenger_path_table->find (o_node_ID) == m_passenger_path_table->end ()
      || m_passenger_path_table->find (o_node_ID)->second->find (d_node_ID)
           == m_passenger_path_table->find (o_node_ID)->second->end ()
      || m_passenger_path_table->find (o_node_ID)
             ->second->find (d_node_ID)
             ->second->find (pnr)
           == m_passenger_path_table->find (o_node_ID)
                ->second->find (d_node_ID)
                ->second->end ())
    {
      throw std::runtime_error (
        "MNM_MM_Due::get_best_existing_pnr_path_for_single_interval() no "
        "pnr path");
    }
  MNM_Passenger_Pathset *_pathset = m_passenger_path_table->find (o_node_ID)
                                      ->second->find (d_node_ID)
                                      ->second->find (pnr)
                                      ->second;
  MNM_Passenger_Path_PnR *_p_path_pnr = nullptr;
  MNM_Passenger_Path_PnR *_cur_best_p_path_pnr = nullptr;
  MNM_Path *_driving_path = nullptr;
  MNM_Path *_bustransit_path = nullptr;
  MNM_PnR_Path *_pnr_path = nullptr;
  for (int i = interval; i < interval + 1; i++)
    {
      _assign_inter = (int) i / m_mmdta_config->get_int ("assign_frq");
      if (_assign_inter >= m_total_assign_inter)
        _assign_inter = m_total_assign_inter - 1;
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (transit, o_node_ID,
                                                       d_node_ID,
                                                       _assign_inter);
      for (auto _p_path : _pathset->m_path_vec)
        { // get lowest disutility route at time interval _col and current OD
          // pair
          if (_p_path->m_travel_disutility_vec.empty ())
            {
              // _tmp_cost = _p_path -> get_travel_cost(TFlt(i), mmdta);
              _tmp_cost
                = _p_path->get_travel_cost (TFlt (i), mmdta, m_link_tt_map,
                                            m_transitlink_tt_map);
              _tmp_cost = get_disutility (_p_path->m_mode, _tmp_cost,
                                          _tot_dmd_one_mode);
            }
          else
            {
              IAssert ((int) _assign_inter
                       < (int) _p_path->m_travel_disutility_vec.size ());
              _tmp_cost = _p_path->m_travel_disutility_vec[_assign_inter];
            }
          if (std::isinf (_tmp_cost))
            {
              continue;
            }
          if (_tmp_cost < _cur_best_cost)
            {
              _cur_best_cost = _tmp_cost;
              _cur_best_time = i;
              if (_cur_best_p_path_pnr == nullptr
                  || !_cur_best_p_path_pnr->is_equal (_p_path))
                {
                  if (_cur_best_p_path_pnr != nullptr)
                    delete _cur_best_p_path_pnr;
                  _p_path_pnr
                    = dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path);
                  _driving_path = new MNM_Path ();
                  _driving_path->m_node_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_driving_path->m_node_vec;
                  _driving_path->m_link_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_driving_path->m_link_vec;
                  _bustransit_path = new MNM_Path ();
                  _bustransit_path->m_node_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_transit_path->m_node_vec;
                  _bustransit_path->m_link_vec
                    = dynamic_cast<MNM_PnR_Path *> (_p_path_pnr->m_path)
                        ->m_transit_path->m_link_vec;
                  _pnr_path
                    = new MNM_PnR_Path (-1,
                                        _p_path_pnr->m_mid_parking_lot->m_ID,
                                        _p_path_pnr->m_mid_parking_lot
                                          ->m_dest_node->m_node_ID,
                                        _driving_path, _bustransit_path);

                  _cur_best_p_path_pnr
                    = new MNM_Passenger_Path_PnR (pnr, _pnr_path, m_vot,
                                                  m_early_penalty,
                                                  m_late_penalty, m_target_time,
                                                  0.0,
                                                  _p_path_pnr
                                                    ->m_mid_parking_lot,
                                                  m_bus_fare,
                                                  m_pnr_inconvenience);
                  _driving_path = nullptr;
                  _bustransit_path = nullptr;
                  _pnr_path = nullptr;
                  IAssert (_cur_best_p_path_pnr->m_path != nullptr);
                }
            }
        }
    }
  // IAssert(_cur_best_time >= 0);
  // IAssert(_cur_best_p_path_pnr != nullptr);
  // <path, loading_inter, cost>
  return std::make_tuple (_cur_best_p_path_pnr, _cur_best_time, _cur_best_cost);
}

std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
MNM_MM_Due::get_best_existing_path_for_single_interval (
  TInt interval, TInt o_node_ID, TInt d_node_ID, MNM_Dta_Multimodal *mmdta)
{
  // <path, loading_inter, cost>
  std::tuple<MNM_Passenger_Path_Driving *, TInt, TFlt> _path_driving_result;
  std::tuple<MNM_Passenger_Path_Bus *, TInt, TFlt> _path_bus_result;
  std::tuple<MNM_Passenger_Path_PnR *, TInt, TFlt> _path_pnr_result;
  // <<path, loading_inter, cost>, mode>
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int> _best
    = std::make_pair (std::make_tuple (nullptr, -1, 0.), -1);

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path_driving_result
        = get_best_existing_driving_path_for_single_interval (interval,
                                                              o_node_ID,
                                                              d_node_ID, mmdta);
      printf ("new driving path cost: %f\n",
              (float) std::get<2> (_path_driving_result));
      _best = std::make_pair (_path_driving_result, driving);
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path_bus_result
        = get_best_existing_bus_path_for_single_interval (interval, o_node_ID,
                                                          d_node_ID, mmdta);
      printf ("new bustransit path cost: %f\n",
              (float) std::get<2> (_path_bus_result));
      if (std::get<0> (_path_bus_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_bus_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_bus_result, transit);
                }
              else
                {
                  delete std::get<0> (_path_bus_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_bus_result, transit);
            }
        }
    }

  if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
        != m_mode_vec.end ()
      && m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _path_pnr_result
        = get_best_existing_pnr_path_for_single_interval (interval, o_node_ID,
                                                          d_node_ID, mmdta);
      printf ("new pnr path cost: %f\n",
              (float) std::get<2> (_path_pnr_result));
      if (std::get<0> (_path_pnr_result) != nullptr)
        {
          if (std::get<0> (_best.first) != nullptr)
            {
              if (std::get<2> (_path_pnr_result) < std::get<2> (_best.first))
                {
                  delete std::get<0> (_best.first);
                  _best = std::make_pair (_path_pnr_result, pnr);
                }
              else
                {
                  delete std::get<0> (_path_pnr_result);
                }
            }
          else
            {
              _best = std::make_pair (_path_pnr_result, pnr);
            }
        }
    }
  IAssert (std::get<0> (_best.first) != nullptr);
  return _best;
}

int
MNM_MM_Due::update_one_path_cost (MNM_Passenger_Path_Base *p_path,
                                  TInt o_node_ID, TInt d_node_ID,
                                  MNM_Dta_Multimodal *mmdta)
{
  TInt _depart_time;
  TFlt _tot_dmd_one_mode, _travel_time, _travel_cost, _travel_disutility;

  p_path->m_travel_time_vec.clear ();
  p_path->m_travel_cost_vec.clear ();
  p_path->m_travel_disutility_vec.clear ();
  for (int _col = 0; _col < m_total_assign_inter; _col++)
    {
      _depart_time = _col * m_mmdta_config->get_int ("assign_frq");
      // _travel_time = p_path->get_travel_time(TFlt(_depart_time), mmdta) *
      // m_unit_time;  // seconds _travel_cost =
      // p_path->get_travel_cost(TFlt(_depart_time), mmdta);
      _travel_time
        = p_path->get_travel_time (TFlt (_depart_time), mmdta, m_link_tt_map,
                                   m_transitlink_tt_map); // intervals
      _travel_cost = p_path->get_travel_cost_with_tt (TFlt (_depart_time),
                                                      _travel_time, mmdta);
      _travel_time = _travel_time * m_unit_time; // seconds
      _tot_dmd_one_mode
        = compute_total_passenger_demand_for_one_mode (p_path->m_mode,
                                                       o_node_ID, d_node_ID,
                                                       _col);
      _travel_disutility
        = get_disutility (p_path->m_mode, _travel_cost, _tot_dmd_one_mode);
      p_path->m_travel_time_vec.push_back (_travel_time);
      p_path->m_travel_cost_vec.push_back (_travel_cost);
      p_path->m_travel_disutility_vec.push_back (_travel_disutility);
    }
  return 0;
}

int
MNM_MM_Due::update_path_table_cost (MNM_Dta_Multimodal *mmdta)
{
  TInt _o_node_ID, _d_node_ID;
  // #pragma omp parallel num_threads(20)
  for (auto _o_it : *m_passenger_path_table)
    {
      _o_node_ID = _o_it.first;
      for (auto _d_it : *(_o_it.second))
        {
          _d_node_ID = _d_it.first;
          for (auto _m_it : *(_d_it.second))
            {
              for (MNM_Passenger_Path_Base *_path : _m_it.second->m_path_vec)
                {
                  // #pragma omp task
                  update_one_path_cost (_path, _o_node_ID, _d_node_ID, mmdta);
                  printf ("update_one_path_cost\n");
                }
            }
        }
    }
  printf ("Finish update path table cost\n");
  MNM::save_passenger_path_table (m_passenger_path_table,
                                  m_file_folder + "/"
                                    + mmdta->m_statistics->m_self_config
                                        ->get_string ("rec_folder"),
                                  std::string ("passenger_path_table"),
                                  std::string ("passenger_path_table_buffer"),
                                  true, true);
  return 0;
}

int
MNM_MM_Due::update_path_table (MNM_Dta_Multimodal *mmdta, int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID, _mode;
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, TInt>
    _path_result;
  MNM_Passenger_Path_Base *_path;
  MNM_Passenger_Pathset *_path_set_driving;
  MNM_Passenger_Pathset *_path_set_bus;
  MNM_Passenger_Pathset *_path_set_pnr;
  std::vector<MNM_Passenger_Pathset *> _path_set_vec;
  TFlt _tot_change, _tmp_change, _tot_oneOD_demand, _len, _cost;
  MNM_Passenger_Path_Base *_best_path;
  int _best_time_col;
  int _best_assign_col;
  bool _exist;

  MNM_TDSP_Tree *_tdsp_tree;
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_driving
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_bus
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  // build_link_cost_map(mmdta) is called before this function
  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_DUE_ColumnGeneration")
    {
      for (auto _d_it : mmdta->m_od_factory->m_destination_map)
        {
          _dest = _d_it.second;
          _dest_node_ID = _dest->m_dest_node->m_node_ID;

          // for driving
          _tdsp_tree = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_graph,
                                          m_total_loading_inter);
          _tdsp_tree->initialize ();
          _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);
          _tdsp_tree_map_driving.insert (
            std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
          _tdsp_tree = nullptr;
          IAssert (_tdsp_tree_map_driving.find (_dest_node_ID)->second
                   != nullptr);

          // for bus transit
          if (mmdta->m_bus_transit_graph->IsNode (_dest_node_ID))
            {
              _tdsp_tree
                = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_bus_transit_graph,
                                     m_total_loading_inter);
              _tdsp_tree->initialize ();
              _tdsp_tree->update_tree (m_transitlink_cost_map,
                                       m_transitlink_tt_map);
              _tdsp_tree_map_bus.insert (
                std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
              _tdsp_tree = nullptr;
              IAssert (_tdsp_tree_map_bus.find (_dest_node_ID)->second
                       != nullptr);
            }
        }
    }

  for (auto _d_it : mmdta->m_od_factory->m_destination_map)
    {
      _dest = _d_it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      for (auto _o_it : mmdta->m_od_factory->m_origin_map)
        {
          _orig = _o_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (m_passenger_demand.find (_orig_node_ID)
                == m_passenger_demand.end ()
              || m_passenger_demand.find (_orig_node_ID)
                     ->second.find (_dest_node_ID)
                   == m_passenger_demand.find (_orig_node_ID)->second.end ())
            {
              continue;
            }

          _tot_oneOD_demand
            = compute_total_passenger_demand (_orig, _dest,
                                              m_total_assign_inter);
          _tot_change = 0.0;
          _best_path = nullptr;
          if (m_mmdta_config->get_string ("routing_type")
              == "Multimodal_DUE_ColumnGeneration")
            {
              _path_result = get_best_path (_orig_node_ID, _dest_node_ID,
                                            _tdsp_tree_map_driving,
                                            _tdsp_tree_map_bus, mmdta);
            }
          else
            {
              _path_result
                = get_best_existing_path (_orig_node_ID, _dest_node_ID, mmdta);
            }

          _path = std::get<0> (_path_result.first);
          _cost = std::get<2> (_path_result.first);
          _mode = _path_result.second;
          _best_time_col = std::get<1> (_path_result.first);
          _best_assign_col
            = (int) _best_time_col / m_mmdta_config->get_int ("assign_frq");
          if (_best_assign_col >= m_total_assign_inter)
            _best_assign_col = m_total_assign_inter - 1;
          // printf("Best time col %d\n", _best_time_col);

          if (_mode == driving)
            {
              printf ("best path is in driving mode\n");
            }
          else if (_mode == transit)
            {
              printf ("best path is in bustransit mode\n");
            }
          else if (_mode == pnr)
            {
              printf ("best path is in pnr mode\n");
            }
          else
            {
              throw std::runtime_error ("Mode not implemented");
            }

          // TODO: other modes
          _path_set_driving = nullptr;
          _path_set_bus = nullptr;
          _path_set_pnr = nullptr;
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              _path_set_driving = m_passenger_path_table->find (_orig_node_ID)
                                    ->second->find (_dest_node_ID)
                                    ->second->find (driving)
                                    ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              _path_set_bus = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (transit)
                                ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              _path_set_pnr = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (pnr)
                                ->second;
            }
          _path_set_vec = { _path_set_driving, _path_set_bus, _path_set_pnr };

          if (m_mmdta_config->get_string ("routing_type")
              == "Multimodal_DUE_ColumnGeneration")
            {
              _exist = false;
              if (_mode == driving && _path_set_driving != nullptr)
                {
                  _exist = _path_set_driving->is_in (_path);
                }
              else if (_mode == transit && _path_set_bus != nullptr)
                {
                  _exist = _path_set_bus->is_in (_path);
                }
              else if (_mode == pnr && _path_set_pnr != nullptr)
                {
                  _exist = _path_set_pnr->is_in (_path);
                }
              else
                {
                  throw std::runtime_error ("Mode not implemented");
                }
            }
          else
            {
              _exist = true;
            }

          if (_exist)
            {
              printf ("Update current pathset\n\n");
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          for (int _col = 0; _col < m_total_assign_inter;
                               _col++)
                            {
                              if (!(_tmp_path->is_equal (_path))
                                  || _col != _best_assign_col)
                                {
                                  _tmp_change
                                    = MNM_Ults::min (_tmp_path->m_buffer[_col],
                                                     _tot_oneOD_demand
                                                       * m_step_size
                                                       / TFlt (iter + 1));
                                  _tmp_path->m_buffer[_col] -= _tmp_change;
                                  _tot_change += _tmp_change;
                                }
                            }
                          if (_tmp_path->is_equal (_path))
                            {
                              IAssert (_best_path == nullptr);
                              _best_path = _tmp_path;
                            }
                        }
                    }
                }

              _best_path->m_buffer[_best_assign_col] += _tot_change;
              delete _path;
            }
          else
            {
              printf ("Adding new path\n\n");

              // _path -> allocate_buffer(m_total_assign_inter);
              // _path -> m_buffer[_best_assign_col] += m_step_size / TFlt(iter
              // + 1); _len = 0; for (auto _path_set : _path_set_vec) {
              //     if (_path_set != nullptr) {
              //         _len += TFlt(_path_set -> m_path_vec.size());
              //     }
              // }
              // for (auto _path_set : _path_set_vec) {
              //     if (_path_set != nullptr) {
              //         for (auto _tmp_path : _path_set -> m_path_vec){
              //             _tmp_path -> m_buffer[_best_assign_col] -=
              //             m_step_size / TFlt(iter + 1) / _len;
              //         }
              //     }
              // }
              // if (_mode == driving) {
              //     _path -> m_path -> m_path_ID = (int)_path_set_driving ->
              //     m_path_vec.size(); _path_set_driving ->
              //     m_path_vec.push_back(_path);
              // }
              // else if (_mode == transit) {
              //     _path -> m_path -> m_path_ID = (int)_path_set_bus ->
              //     m_path_vec.size(); _path_set_bus ->
              //     m_path_vec.push_back(_path);
              // }
              // else if (_mode == pnr) {
              //     dynamic_cast<MNM_Passenger_Path_PnR*>(_path) -> m_path ->
              //     m_path_ID = (int)_path_set_pnr -> m_path_vec.size();
              //     _path_set_pnr -> m_path_vec.push_back(_path);
              // }
              // else {
              //     printf("Mode not implemented!\n");
              //     exit(-1);
              // }

              // another way
              _path->allocate_buffer (m_total_assign_inter);
              if (_mode == driving)
                {
                  _path->m_path->m_path_ID
                    = (int) _path_set_driving->m_path_vec.size ();
                  _path_set_driving->m_path_vec.push_back (_path);
                }
              else if (_mode == transit)
                {
                  _path->m_path->m_path_ID
                    = (int) _path_set_bus->m_path_vec.size ();
                  _path_set_bus->m_path_vec.push_back (_path);
                }
              else if (_mode == pnr)
                {
                  dynamic_cast<MNM_Passenger_Path_PnR *> (_path)
                    ->m_path->m_path_ID
                    = (int) _path_set_pnr->m_path_vec.size ();
                  _path_set_pnr->m_path_vec.push_back (_path);
                }
              else
                {
                  throw std::runtime_error ("Mode not implemented");
                }
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          if (!(_tmp_path->is_equal (_path)))
                            {
                              _tmp_change
                                = _tmp_path->m_buffer[_best_assign_col]
                                  * m_step_size / TFlt (iter + 1);
                              // printf("tmp change %lf\n", _tmp_change());
                              _tmp_path->m_buffer[_best_assign_col]
                                -= _tmp_change;
                              _tot_change += _tmp_change;
                            }
                          else
                            {
                              IAssert (_best_path == nullptr);
                              _best_path = _tmp_path;
                            }
                        }
                    }
                }
              _best_path->m_buffer[_best_assign_col] += _tot_change;
            }
        }
    }

  for (auto _it : _tdsp_tree_map_driving)
    {
      delete _it.second;
    }
  _tdsp_tree_map_driving.clear ();
  for (auto _it : _tdsp_tree_map_bus)
    {
      delete _it.second;
    }
  _tdsp_tree_map_bus.clear ();
  return 0;
}

int
MNM_MM_Due::update_path_table_fixed_departure_time_choice (
  MNM_Dta_Multimodal *mmdta, int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID, _mode;
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, TInt>
    _path_result;
  MNM_Passenger_Path_Base *_path;
  MNM_Passenger_Pathset *_path_set_driving;
  MNM_Passenger_Pathset *_path_set_bus;
  MNM_Passenger_Pathset *_path_set_pnr;
  std::vector<MNM_Passenger_Pathset *> _path_set_vec;
  TFlt _tot_change, _tmp_change, _len, _cost;
  MNM_Passenger_Path_Base *_best_path;
  int _best_time_col;
  int _best_assign_col;
  bool _exist;

  MNM_TDSP_Tree *_tdsp_tree;
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_driving
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_bus
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  // build_link_cost_map(mmdta) is called before this function
  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_DUE_ColumnGeneration")
    {
      for (auto _d_it : mmdta->m_od_factory->m_destination_map)
        {
          _dest = _d_it.second;
          _dest_node_ID = _dest->m_dest_node->m_node_ID;

          // for driving
          _tdsp_tree = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_graph,
                                          m_total_loading_inter);
          _tdsp_tree->initialize ();
          _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);
          _tdsp_tree_map_driving.insert (
            std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
          _tdsp_tree = nullptr;
          IAssert (_tdsp_tree_map_driving.find (_dest_node_ID)->second
                   != nullptr);

          // for bus transit
          if (mmdta->m_bus_transit_graph->IsNode (_dest_node_ID))
            {
              _tdsp_tree
                = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_bus_transit_graph,
                                     m_total_loading_inter);
              _tdsp_tree->initialize ();
              _tdsp_tree->update_tree (m_transitlink_cost_map,
                                       m_transitlink_tt_map);
              _tdsp_tree_map_bus.insert (
                std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
              _tdsp_tree = nullptr;
              IAssert (_tdsp_tree_map_bus.find (_dest_node_ID)->second
                       != nullptr);
            }
        }
    }

  for (auto _d_it : mmdta->m_od_factory->m_destination_map)
    {
      _dest = _d_it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      for (auto _o_it : mmdta->m_od_factory->m_origin_map)
        {
          _orig = _o_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (m_passenger_demand.find (_orig_node_ID)
                == m_passenger_demand.end ()
              || m_passenger_demand.find (_orig_node_ID)
                     ->second.find (_dest_node_ID)
                   == m_passenger_demand.find (_orig_node_ID)->second.end ())
            {
              continue;
            }

          // TODO: other modes
          _path_set_driving = nullptr;
          _path_set_bus = nullptr;
          _path_set_pnr = nullptr;
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              _path_set_driving = m_passenger_path_table->find (_orig_node_ID)
                                    ->second->find (_dest_node_ID)
                                    ->second->find (driving)
                                    ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              _path_set_bus = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (transit)
                                ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              _path_set_pnr = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (pnr)
                                ->second;
            }
          _path_set_vec = { _path_set_driving, _path_set_bus, _path_set_pnr };

          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _tot_change = 0.0;
              _best_path = nullptr;
              if (m_mmdta_config->get_string ("routing_type")
                  == "Multimodal_DUE_ColumnGeneration")
                {
                  _path_result
                    = get_best_path_for_single_interval (_col
                                                           * m_mmdta_config
                                                               ->get_int (
                                                                 "assign_frq"),
                                                         _orig_node_ID,
                                                         _dest_node_ID,
                                                         _tdsp_tree_map_driving,
                                                         _tdsp_tree_map_bus,
                                                         mmdta);
                }
              else
                {
                  _path_result = get_best_existing_path_for_single_interval (
                    _col * m_mmdta_config->get_int ("assign_frq"),
                    _orig_node_ID, _dest_node_ID, mmdta);
                }

              _path = std::get<0> (_path_result.first);
              _cost = std::get<2> (_path_result.first);
              _mode = _path_result.second;
              _best_time_col = std::get<1> (_path_result.first);
              _best_assign_col
                = (int) _best_time_col / m_mmdta_config->get_int ("assign_frq");
              if (_best_assign_col >= m_total_assign_inter)
                _best_assign_col = m_total_assign_inter - 1;
              // printf("Best time col %d\n", _best_time_col);
              IAssert (_col == _best_assign_col);

              if (_mode == driving)
                {
                  printf ("best path is in driving mode\n");
                }
              else if (_mode == transit)
                {
                  printf ("best path is in bustransit mode\n");
                }
              else if (_mode == pnr)
                {
                  printf ("best path is in pnr mode\n");
                }
              else
                {
                  throw std::runtime_error ("Mode not implemented");
                }

              if (m_mmdta_config->get_string ("routing_type")
                  == "Multimodal_DUE_ColumnGeneration")
                {
                  _exist = false;
                  if (_mode == driving && _path_set_driving != nullptr)
                    {
                      _exist = _path_set_driving->is_in (_path);
                    }
                  else if (_mode == transit && _path_set_bus != nullptr)
                    {
                      _exist = _path_set_bus->is_in (_path);
                    }
                  else if (_mode == pnr && _path_set_pnr != nullptr)
                    {
                      _exist = _path_set_pnr->is_in (_path);
                    }
                  else
                    {
                      throw std::runtime_error ("Mode not implemented");
                    }
                }
              else
                {
                  _exist = true;
                }

              if (_exist)
                {
                  printf ("Update current pathset\n\n");
                }
              else
                {
                  printf ("Adding new path\n\n");
                  _path->allocate_buffer (m_total_assign_inter);

                  // update_one_path_cost(_path, _orig_node_ID, _dest_node_ID,
                  // mmdta);

                  if (_mode == driving)
                    {
                      _path->m_path->m_path_ID
                        = (int) _path_set_driving->m_path_vec.size ();
                      _path_set_driving->m_path_vec.push_back (_path);
                    }
                  else if (_mode == transit)
                    {
                      _path->m_path->m_path_ID
                        = (int) _path_set_bus->m_path_vec.size ();
                      _path_set_bus->m_path_vec.push_back (_path);
                    }
                  else if (_mode == pnr)
                    {
                      dynamic_cast<MNM_Passenger_Path_PnR *> (_path)
                        ->m_path->m_path_ID
                        = (int) _path_set_pnr->m_path_vec.size ();
                      _path_set_pnr->m_path_vec.push_back (_path);
                    }
                  else
                    {
                      throw std::runtime_error ("Mode not implemented");
                    }
                }

              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          if (!(_tmp_path->is_equal (_path)))
                            {
                              _tmp_change = _tmp_path->m_buffer[_col]
                                            * m_step_size
                                            / sqrt (TFlt (iter + 1));
                              // if (std::isinf(_tmp_path ->
                              // m_travel_disutility_vec[_col])) {
                              //     _tmp_change = _tmp_path->m_buffer[_col];
                              // }
                              _tmp_path->m_buffer[_col] -= _tmp_change;
                              _tot_change += _tmp_change;
                            }
                          else
                            {
                              IAssert (_best_path == nullptr);
                              _best_path = _tmp_path;
                            }
                        }
                    }
                }

              _best_path->m_buffer[_col] += _tot_change;
              if (_exist)
                delete _path;

              // verification
              TFlt _tot_dmd_before
                = m_passenger_demand[_orig_node_ID][_dest_node_ID][_col];
              TFlt _tot_dmd_after = 0.;
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      _tot_dmd_after
                        += compute_total_passenger_demand_for_one_mode (
                          _path_set->m_mode, _orig_node_ID, _dest_node_ID,
                          _col);
                    }
                }
              IAssert (std::abs (_tot_dmd_after - _tot_dmd_before)
                       <= 1e-6
                            * std::max (std::abs (_tot_dmd_after),
                                        std::abs (_tot_dmd_before)));
            }
        }
    }

  for (auto _it : _tdsp_tree_map_driving)
    {
      delete _it.second;
    }
  _tdsp_tree_map_driving.clear ();
  for (auto _it : _tdsp_tree_map_bus)
    {
      delete _it.second;
    }
  _tdsp_tree_map_bus.clear ();
  return 0;
}

int
MNM_MM_Due::update_path_table_gp_fixed_departure_time_choice (
  MNM_Dta_Multimodal *mmdta, int iter)
{
  MNM_Origin *_orig;
  MNM_Destination *_dest;
  TInt _orig_node_ID, _dest_node_ID, _mode, _tot_nonzero_path;
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, TInt>
    _path_result;
  MNM_Passenger_Path_Base *_path;
  MNM_Passenger_Pathset *_path_set_driving;
  MNM_Passenger_Pathset *_path_set_bus;
  MNM_Passenger_Pathset *_path_set_pnr;
  std::vector<MNM_Passenger_Pathset *> _path_set_vec;
  TFlt _tot_change, _tot_path_cost, _tot_dmd_one_mode, _tmp_change, _len, _cost,
    _tmp_cost, _tau, _min_flow;
  int _best_time_col;
  int _best_assign_col;
  bool _exist, _flg;

  MNM_TDSP_Tree *_tdsp_tree;
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_driving
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  std::unordered_map<TInt, MNM_TDSP_Tree *> _tdsp_tree_map_bus
    = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  // build_link_cost_map(mmdta);
  if (m_mmdta_config->get_string ("routing_type")
      == "Multimodal_DUE_ColumnGeneration")
    {
      // build_link_cost_map(mmdta);
      for (auto _d_it : mmdta->m_od_factory->m_destination_map)
        {
          _dest = _d_it.second;
          _dest_node_ID = _dest->m_dest_node->m_node_ID;

          // for driving
          _tdsp_tree = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_graph,
                                          m_total_loading_inter);
          _tdsp_tree->initialize ();
          _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);
          _tdsp_tree_map_driving.insert (
            std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
          _tdsp_tree = nullptr;
          IAssert (_tdsp_tree_map_driving.find (_dest_node_ID)->second
                   != nullptr);

          // for bus transit
          if (mmdta->m_bus_transit_graph->IsNode (_dest_node_ID))
            {
              _tdsp_tree
                = new MNM_TDSP_Tree (_dest_node_ID, mmdta->m_bus_transit_graph,
                                     m_total_loading_inter);
              _tdsp_tree->initialize ();
              _tdsp_tree->update_tree (m_transitlink_cost_map,
                                       m_transitlink_tt_map);
              _tdsp_tree_map_bus.insert (
                std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
              _tdsp_tree = nullptr;
              IAssert (_tdsp_tree_map_bus.find (_dest_node_ID)->second
                       != nullptr);
            }
        }
    }

  for (auto _d_it : mmdta->m_od_factory->m_destination_map)
    {
      _dest = _d_it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      for (auto _o_it : mmdta->m_od_factory->m_origin_map)
        {
          _orig = _o_it.second;
          _orig_node_ID = _orig->m_origin_node->m_node_ID;

          // if no demand for this OD pair
          if (m_passenger_demand.find (_orig_node_ID)
                == m_passenger_demand.end ()
              || m_passenger_demand.find (_orig_node_ID)
                     ->second.find (_dest_node_ID)
                   == m_passenger_demand.find (_orig_node_ID)->second.end ())
            {
              continue;
            }

          // TODO: other modes
          _path_set_driving = nullptr;
          _path_set_bus = nullptr;
          _path_set_pnr = nullptr;
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), driving)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (driving)
                   ->second)
            {
              _path_set_driving = m_passenger_path_table->find (_orig_node_ID)
                                    ->second->find (_dest_node_ID)
                                    ->second->find (driving)
                                    ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), transit)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (transit)
                   ->second)
            {
              _path_set_bus = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (transit)
                                ->second;
            }
          if (std::find (m_mode_vec.begin (), m_mode_vec.end (), pnr)
                != m_mode_vec.end ()
              && m_od_mode_connectivity.find (_orig_node_ID)
                   ->second.find (_dest_node_ID)
                   ->second.find (pnr)
                   ->second)
            {
              _path_set_pnr = m_passenger_path_table->find (_orig_node_ID)
                                ->second->find (_dest_node_ID)
                                ->second->find (pnr)
                                ->second;
            }
          _path_set_vec = { _path_set_driving, _path_set_bus, _path_set_pnr };

          for (int _col = 0; _col < m_total_assign_inter; _col++)
            {
              _tot_change = 0.0;
              _tot_path_cost = 0.0;
              _tot_nonzero_path = 0;
              _tau = TFlt (std::numeric_limits<double>::infinity ());
              _min_flow = TFlt (std::numeric_limits<double>::infinity ());
              _flg = false;
              if (m_mmdta_config->get_string ("routing_type")
                  == "Multimodal_DUE_ColumnGeneration")
                {
                  _path_result
                    = get_best_path_for_single_interval (_col
                                                           * m_mmdta_config
                                                               ->get_int (
                                                                 "assign_frq"),
                                                         _orig_node_ID,
                                                         _dest_node_ID,
                                                         _tdsp_tree_map_driving,
                                                         _tdsp_tree_map_bus,
                                                         mmdta);
                }
              else
                {
                  _path_result = get_best_existing_path_for_single_interval (
                    _col * m_mmdta_config->get_int ("assign_frq"),
                    _orig_node_ID, _dest_node_ID, mmdta);
                }

              _path = std::get<0> (_path_result.first);
              _cost = std::get<2> (_path_result.first);
              _mode = _path_result.second;
              _best_time_col = std::get<1> (_path_result.first);
              _best_assign_col
                = (int) _best_time_col / m_mmdta_config->get_int ("assign_frq");
              if (_best_assign_col >= m_total_assign_inter)
                _best_assign_col = m_total_assign_inter - 1;
              // printf("Best time col %d\n", _best_time_col);
              IAssert (_col == _best_assign_col);

              if (_mode == driving)
                {
                  printf ("best path is in driving mode\n");
                }
              else if (_mode == transit)
                {
                  printf ("best path is in bustransit mode\n");
                }
              else if (_mode == pnr)
                {
                  printf ("best path is in pnr mode\n");
                }
              else
                {
                  throw std::runtime_error ("Mode not implemented");
                }

              if (m_mmdta_config->get_string ("routing_type")
                  == "Multimodal_DUE_ColumnGeneration")
                {
                  _exist = false;
                  if (_mode == driving && _path_set_driving != nullptr)
                    {
                      _exist = _path_set_driving->is_in (_path);
                    }
                  else if (_mode == transit && _path_set_bus != nullptr)
                    {
                      _exist = _path_set_bus->is_in (_path);
                    }
                  else if (_mode == pnr && _path_set_pnr != nullptr)
                    {
                      _exist = _path_set_pnr->is_in (_path);
                    }
                  else
                    {
                      throw std::runtime_error ("Mode not implemented");
                    }
                }
              else
                {
                  _exist = true;
                }

              if (_exist)
                {
                  printf ("Update current pathset\n\n");
                }
              else
                {
                  printf ("Adding new path\n\n");
                  _path->allocate_buffer (m_total_assign_inter);

                  IAssert (_path->m_travel_time_vec.empty ()
                           && _path->m_travel_cost_vec.empty ()
                           && _path->m_travel_disutility_vec.empty ());
                  update_one_path_cost (_path, _orig_node_ID, _dest_node_ID,
                                        mmdta);

                  if (_mode == driving)
                    {
                      _path->m_path->m_path_ID
                        = (int) _path_set_driving->m_path_vec.size ();
                      _path_set_driving->m_path_vec.push_back (_path);
                    }
                  else if (_mode == transit)
                    {
                      _path->m_path->m_path_ID
                        = (int) _path_set_bus->m_path_vec.size ();
                      _path_set_bus->m_path_vec.push_back (_path);
                    }
                  else if (_mode == pnr)
                    {
                      dynamic_cast<MNM_Passenger_Path_PnR *> (_path)
                        ->m_path->m_path_ID
                        = (int) _path_set_pnr->m_path_vec.size ();
                      _path_set_pnr->m_path_vec.push_back (_path);
                    }
                  else
                    {
                      throw std::runtime_error ("Mode not implemented");
                    }
                }

              // average path cost
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          if ((_tmp_path->is_equal (_path))
                              || (_tmp_path->m_buffer[_col] > 0))
                            {
                              _tmp_cost
                                = _tmp_path->m_travel_disutility_vec[_col];
                              // if (std::isinf(_tmp_cost)) {
                              //     continue;
                              // }
                              _tot_path_cost += _tmp_cost;
                              _tot_nonzero_path += 1;
                              if ((_tmp_path->m_buffer[_col] > 0)
                                  && (_min_flow > _tmp_path->m_buffer[_col]))
                                {
                                  _min_flow = _tmp_path->m_buffer[_col];
                                }
                            }
                        }
                    }
                }

              // minimum tau
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          if ((_tmp_path->is_equal (_path))
                              || (_tmp_path->m_buffer[_col] > 0))
                            {
                              _tmp_cost
                                = _tmp_path->m_travel_disutility_vec[_col];
                              // if (std::isinf(_tmp_cost)) {
                              //     continue;
                              // }
                              _tmp_change
                                = _tmp_cost
                                  - _tot_path_cost / _tot_nonzero_path;
                              printf ("original path flow: %.2f, _tmp_cost: "
                                      "%.2f, _tot_path_cost: %.2f, "
                                      "_tot_nonzero_path: %d, _tmp_change: "
                                      "%.2f, m_step_size * "
                                      "_tmp_path->m_buffer[_col] / "
                                      "_tmp_change: %.2f\n",
                                      _tmp_path->m_buffer[_col](), _tmp_cost (),
                                      _tot_path_cost (), _tot_nonzero_path (),
                                      _tmp_change (),
                                      m_step_size ()
                                        * _tmp_path->m_buffer[_col]()
                                        / _tmp_change ());
                              if ((_tmp_change > 0)
                                  && (_tau > m_step_size
                                               * _tmp_path->m_buffer[_col]
                                               / _tmp_change))
                                {
                                  _tau = m_step_size * _tmp_path->m_buffer[_col]
                                         / _tmp_change;
                                  _flg = true;
                                }
                              //                                if ((_tmp_change
                              //                                > 0) && (_tau >
                              //                                m_step_size *
                              //                                _min_flow /
                              //                                _tmp_change)) {
                              //                                    _tau =
                              //                                    m_step_size
                              //                                    * _min_flow
                              //                                    /
                              //                                    _tmp_change;
                              //                                }
                              //                                if ((_tmp_change
                              //                                > 0) && (_tau
                              //                                > 1.0 /
                              //                                _tmp_change)) {
                              //                                    _tau = 1.0 /
                              //                                    _tmp_change;
                              //                                }
                            }
                        }
                    }
                }
              printf ("tau: %.2f\n", _tau ());
              if (!_flg)
                {
                  _tau = 0.;
                  continue;
                }
              // flow adjustment
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      for (auto _tmp_path : _path_set->m_path_vec)
                        {
                          if ((_tmp_path->is_equal (_path))
                              || (_tmp_path->m_buffer[_col] > 0))
                            {
                              _tmp_cost
                                = _tmp_path->m_travel_disutility_vec[_col];
                              _tmp_change
                                = _tmp_cost
                                  - _tot_path_cost / _tot_nonzero_path;
                              // printf("original path flow: %.2f, tau: %.2f,
                              // _tmp_change: %.2f, tau*_tmp_change: %.2f\n",
                              // _tmp_path->m_buffer[_col](), _tau(),
                              // _tmp_change(), _tau() * _tmp_change());
                              _tmp_path->m_buffer[_col] -= _tau * _tmp_change;
                              if (_tmp_path->m_buffer[_col] < 0.)
                                {
                                  throw std::runtime_error (
                                    "Something is wrong in GP method");
                                }
                            }
                        }
                    }
                }

              if (_exist)
                delete _path;

              // verification
              TFlt _tot_dmd_before
                = m_passenger_demand[_orig_node_ID][_dest_node_ID][_col];
              TFlt _tot_dmd_after = 0.;
              for (auto _path_set : _path_set_vec)
                {
                  if (_path_set != nullptr)
                    {
                      _tot_dmd_after
                        += compute_total_passenger_demand_for_one_mode (
                          _path_set->m_mode, _orig_node_ID, _dest_node_ID,
                          _col);
                    }
                }
              printf ("_tot_dmd_after: %.2f, _tot_dmd_before: %.2f\n",
                      _tot_dmd_after (), _tot_dmd_before ());
              IAssert (
                MNM_Ults::approximate_equal (_tot_dmd_after, _tot_dmd_before));
              // IAssert(std::abs(_tot_dmd_after - _tot_dmd_before) <= 1e-6 *
              // std::max(std::abs(_tot_dmd_after), std::abs(_tot_dmd_before)));
            }
        }
    }

  for (auto _it : _tdsp_tree_map_driving)
    {
      delete _it.second;
    }
  _tdsp_tree_map_driving.clear ();
  for (auto _it : _tdsp_tree_map_bus)
    {
      delete _it.second;
    }
  _tdsp_tree_map_bus.clear ();
  return 0;
}

MNM_Dta_Multimodal *
MNM_MM_Due::run_mmdta (bool verbose)
{
  auto *mmdta = new MNM_Dta_Multimodal (m_file_folder);
  mmdta->build_from_files (); // set_routing() is done
  mmdta->hook_up_node_and_link ();
  mmdta->find_connected_pnr_parkinglot_for_destination ();
  // mmdta -> is_ok();  // do this check once if it is time-consuming

  update_origin_demand_from_passenger_path_table (mmdta);

  passenger_path_table_to_multimodal_path_table (mmdta);

  auto *_routing
    = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (mmdta->m_routing);
  _routing->m_routing_car_pnr_fixed->m_pnr_path_table = m_pnr_path_table;
  _routing->m_routing_passenger_fixed->m_bustransit_path_table
    = m_bustransit_path_table;
  _routing->m_routing_fixed_car->init_routing (m_driving_path_table);
  _routing->m_routing_fixed_truck->init_routing (m_driving_path_table);

  //    for (auto _link_it : mmdta->m_link_factory->m_link_map) {
  //        _link_it.second->install_cumulative_curve();
  //    }

  mmdta->pre_loading ();
  mmdta->loading (verbose);
  return mmdta;
}

MNM_Dta_Multimodal *
MNM_MM_Due::run_mmdta_adaptive (bool verbose)
{
  // MNM_Dta_Multimodal* mmdta = new MNM_Dta_Multimodal(m_file_folder);
  // mmdta -> build_from_files();
  // mmdta -> hook_up_node_and_link();
  // mmdta -> find_connected_pnr_parkinglot_for_destination();
  // mmdta -> is_ok();  // do this check once if it is time-consuming

  // for (auto _link_it : mmdta->m_link_factory->m_link_map) {
  //     _link_it.second->install_cumulative_curve();
  // }

  MNM_Dta_Multimodal *mmdta = m_mmdta;
  if (!mmdta->m_statistics->m_self_config->get_int ("rec_tt"))
    {
      throw std::runtime_error (
        "rec_tt should set to 1 in input file config.conf");
    }

  mmdta->pre_loading ();

  int _current_inter = 0;
  int _dta_assign_inter = mmdta->m_start_assign_interval;
  int _due_assign_inter = mmdta->m_start_assign_interval;

  while (!mmdta->finished_loading (_current_inter)
         || _dta_assign_inter < mmdta->m_total_assign_inter)
    {
      if ((_current_inter % m_mmdta_config->get_int ("assign_frq") == 0)
          && (_due_assign_inter < m_total_assign_inter))
        {
          if (_current_inter == 0)
            mmdta->m_statistics->update_record (
              _current_inter); // for other timestamps, this is called in
                               // load_once()
          build_link_cost_map_snapshot (mmdta, _current_inter,
                                        true); // use m_statistics instead of CC
                                               // to compute link travel time
                                               // since this is in simulation
          update_snapshot_route_table (mmdta, _current_inter);
          update_origin_demand_logit_model (mmdta, _due_assign_inter);
          _due_assign_inter += 1;
        }

      if (verbose)
        {
          std::cout << std::endl
                    << "Current loading interval: " << _current_inter << ", "
                    << "Current assignment interval: "
                    << int (_current_inter
                            / m_mmdta_config->get_int ("assign_frq"))
                    << std::endl;
        }

      mmdta->load_once (verbose, _current_inter, _dta_assign_inter);
      // link cc will be updated with the record at the end of this interval
      // (i.e., _current_inter + 1)
      if (++_current_inter % mmdta->m_assign_freq == 0)
        {
          ++_dta_assign_inter;
        }
    }
  if (verbose)
    {
      MNM::print_vehicle_statistics (
        dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory));
      MNM::print_passenger_statistics (mmdta->m_passenger_factory);
    }
  mmdta->m_statistics->post_record ();
  mmdta->m_current_loading_interval = _current_inter;
  // return _current_inter;  // total number of actual loading intervals =
  // _current_inter

  return mmdta;
}
