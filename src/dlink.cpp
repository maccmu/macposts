#include "dlink.h"
#include <cfloat>

MNM_Dlink::MNM_Dlink (TInt ID, TInt number_of_lane, TFlt length, TFlt ffs)
{
  m_link_ID = ID;
  if (ffs < 0)
    {
      throw std::runtime_error ("negative link speed for link "
                                + std::to_string (m_link_ID));
    }
  m_ffs = ffs;

  if (number_of_lane < 0)
    {
      throw std::runtime_error ("negative number of lanes for link "
                                + std::to_string (m_link_ID));
    }
  m_number_of_lane = number_of_lane;

  if (length < 0)
    {
      throw std::runtime_error ("negative length for link "
                                + std::to_string (m_link_ID));
    }
  m_length = length;

  m_finished_array = std::deque<MNM_Veh *> ();
  m_incoming_array = std::deque<MNM_Veh *> ();
  m_N_in = nullptr;
  m_N_out = nullptr;
  m_N_in_tree = nullptr;
  m_N_out_tree = nullptr;
}

MNM_Dlink::~MNM_Dlink ()
{
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

TFlt
MNM_Dlink::get_link_freeflow_tt ()
{
  return m_length / m_ffs; // seconds
}

int
MNM_Dlink::hook_up_node (MNM_Dnode *from, MNM_Dnode *to)
{
  m_from_node = from;
  m_to_node = to;
  return 0;
}

int
MNM_Dlink::install_cumulative_curve ()
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
MNM_Dlink::install_cumulative_curve_tree ()
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
  m_N_out_tree = new MNM_Tree_Cumulative_Curve ();
  return 0;
}

int
MNM_Dlink::move_veh_queue (std::deque<MNM_Veh *> *from_queue,
                           std::deque<MNM_Veh *> *to_queue, TInt number_tomove)
{
  MNM_Veh *_veh;
  for (int i = 0; i < number_tomove; ++i)
    {
      _veh = from_queue->front ();
      from_queue->pop_front ();
      // TODO: update _veh->m_visual_position_on_link for different link types
      // as in multiclass
      to_queue->push_back (_veh);
    }
  return 0;
}

MNM_Dlink_Ctm::MNM_Dlink_Ctm (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                              TInt number_of_lane, TFlt length, TFlt ffs,
                              TFlt unit_time, TFlt flow_scalar)
    : MNM_Dlink::MNM_Dlink (ID, number_of_lane, length, ffs)
{
  m_link_type = MNM_TYPE_CTM;
  if (lane_hold_cap < 0)
    {
      throw std::runtime_error ("negative lane_hold_cap for link "
                                + std::to_string (m_link_ID));
    }
  if (lane_hold_cap > TFlt (300) / TFlt (1600))
    {
      // printf("lane_hold_cap is too large, we will replace it to 300 veh/mile,
      // current link ID is %d\n", m_link_ID());
      lane_hold_cap = TFlt (300) / TFlt (1600);
    }
  m_lane_hold_cap = lane_hold_cap;

  if (lane_flow_cap < 0)
    {
      throw std::runtime_error ("negative lane_flow_cap for link "
                                + std::to_string (m_link_ID));
    }
  if (lane_flow_cap > TFlt (3500) / TFlt (3600))
    {
      // printf("lane_flow_cap is too large, we will replace it to 3500
      // veh/hour, current link ID is %d\n", m_link_ID());
      lane_flow_cap = TFlt (3500) / TFlt (3600);
    }
  m_lane_flow_cap = lane_flow_cap;
  m_flow_scalar = flow_scalar;
  m_cell_array = std::vector<Ctm_Cell *> ();
  TFlt _std_cell_length = m_ffs * unit_time;
  m_num_cells = TInt (floor (m_length / _std_cell_length));
  if (m_num_cells == 0)
    {
      m_num_cells = 1;
    }
  TFlt _lane_hold_cap_last_cell
    = std::max (((m_length - TFlt (m_num_cells - 1) * _std_cell_length)
                 / _std_cell_length)
                  * m_lane_hold_cap,
                m_lane_hold_cap);
  TFlt _wave_speed
    = m_lane_flow_cap
      / (m_lane_hold_cap
         - m_lane_flow_cap / m_ffs); // laneFlwCap/ffs is the critical density.
  m_wave_ratio
    = _wave_speed
      / m_ffs; // note that h >= 2c/v, where h is holding capacity, c is
               // capcity, v is free flow speed. i.e., wvRatio should < 1.
  if (m_wave_ratio < 0)
    {
      throw std::runtime_error ("negative wave ratio for link "
                                + std::to_string (m_link_ID));
    }
  m_last_wave_ratio
    = (m_lane_flow_cap / (_lane_hold_cap_last_cell - m_lane_flow_cap / m_ffs))
      / m_ffs;
  if (m_last_wave_ratio < 0)
    {
      throw std::runtime_error ("negative last cell wave ratio for link "
                                + std::to_string (m_link_ID));
    }
  init_cell_array (unit_time, _std_cell_length, _lane_hold_cap_last_cell);
}

MNM_Dlink_Ctm::~MNM_Dlink_Ctm ()
{
  for (Ctm_Cell *_cell : m_cell_array)
    {
      delete _cell;
    }
  m_cell_array.clear ();
}

int
MNM_Dlink_Ctm::init_cell_array (TFlt unit_time, TFlt std_cell_length,
                                TFlt lane_hold_cap_last_cell)
{
  Ctm_Cell *aCell = nullptr;
  for (int i = 0; i < m_num_cells - 1; ++i)
    {
      aCell
        = new Ctm_Cell (TFlt (m_number_of_lane) * std_cell_length
                          * m_lane_hold_cap,
                        TFlt (m_number_of_lane) * m_lane_flow_cap * unit_time,
                        m_flow_scalar, m_wave_ratio);
      if (aCell == nullptr)
        {
          throw std::runtime_error ("failed to initialize cell");
        };
      m_cell_array.push_back (aCell);
      aCell = nullptr;
    }

  // since the last cell is a non-standard cell
  if (m_length > 0.0)
    {
      aCell
        = new Ctm_Cell (TFlt (m_number_of_lane) * std_cell_length
                          * lane_hold_cap_last_cell,
                        TFlt (m_number_of_lane) * m_lane_flow_cap * unit_time,
                        m_flow_scalar, m_last_wave_ratio);
      if (aCell == NULL)
        {
          throw std::runtime_error ("failed to initialize cell");
        }
      m_cell_array.push_back (aCell);
    }

  // compress the cellArray to reduce space
  m_cell_array.shrink_to_fit ();

  return 0;
}

void
MNM_Dlink_Ctm::print_info ()
{
  printf ("Total number of cell: \t%d\t Standard wave ratio: \t%.4f\nLast cell "
          "wave ratio: \t%.4f\n",
          int (m_num_cells), double (m_wave_ratio), double (m_last_wave_ratio));
  printf ("Volume for each cell is:\n");
  for (int i = 0; i < m_num_cells; ++i)
    {
      printf ("%d, ", int (m_cell_array[i]->m_volume));
    }
  printf ("\n");
}

int
MNM_Dlink_Ctm::update_out_veh ()
{
  TFlt _temp_out_flux, _supply, _demand;
  if (m_num_cells > 1) // if only one cell, no update is needed
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          _demand = m_cell_array[i]->get_demand ();
          _supply = m_cell_array[i + 1]->get_supply ();
          _temp_out_flux = std::min (_demand, _supply) * m_flow_scalar;
          m_cell_array[i]->m_out_veh
            = std::min ((int) m_cell_array[i]->m_veh_queue.size (),
                        MNM_Ults::round (_temp_out_flux));
        }
    }
  m_cell_array[m_num_cells - 1]->m_out_veh
    = m_cell_array[m_num_cells - 1]->m_veh_queue.size ();
  return 0;
}

int
MNM_Dlink_Ctm::evolve (TInt timestamp)
{
  // printf("update_out_veh\n");
  update_out_veh ();
  TInt _num_veh_tomove;
  // printf("move previous cells\n");
  /* previous cells */
  if (m_num_cells > 1)
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          _num_veh_tomove = m_cell_array[i]->m_out_veh;
          move_veh_queue (&(m_cell_array[i]->m_veh_queue),
                          &(m_cell_array[i + 1]->m_veh_queue), _num_veh_tomove);
        }
    }
  /* last cell */
  // printf("move last cell \n");
  move_last_cell ();

  /* update volume */
  // printf("update volume\n");
  if (m_num_cells > 1)
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          m_cell_array[i]->m_volume = m_cell_array[i]->m_veh_queue.size ();
        }
    }
  m_cell_array[m_num_cells - 1]->m_volume
    = m_cell_array[m_num_cells - 1]->m_veh_queue.size ()
      + m_finished_array.size ();
  return 0;
}

TFlt
MNM_Dlink_Ctm::get_link_supply ()
{
  return m_cell_array[0]->get_supply ();
}

int
MNM_Dlink_Ctm::clear_incoming_array (TInt timestamp)
{
  // printf("link ID: %d, in comming: %d, supply : %d\n", (int
  // )m_link_ID,(int)m_incoming_array.size(), (int) (get_link_supply() *
  // m_flow_scalar) );
  if (get_link_supply () * m_flow_scalar < m_incoming_array.size ())
    {
      throw std::runtime_error ("wrong incoming array size");
    }
  move_veh_queue (&m_incoming_array, &(m_cell_array[0]->m_veh_queue),
                  m_incoming_array.size ());

  m_cell_array[0]->m_volume = m_cell_array[0]->m_veh_queue.size ();
  return 0;
}

int
MNM_Dlink_Ctm::move_last_cell ()
{
  TInt _num_veh_tomove;
  _num_veh_tomove = m_cell_array[m_num_cells - 1]->m_out_veh;
  MNM_Veh *_veh;
  std::map<TInt, std::deque<MNM_Veh *> *>::iterator _que_it;
  for (int i = 0; i < _num_veh_tomove; ++i)
    {
      _veh = m_cell_array[m_num_cells - 1]->m_veh_queue.front ();
      m_cell_array[m_num_cells - 1]->m_veh_queue.pop_front ();
      if (_veh->has_next_link ())
        {
          m_finished_array.push_back (_veh);
        }
      else
        {
          throw std::runtime_error ("invalid vehicle state");
        }
    }
  return 0;
}

MNM_Dlink_Ctm::Ctm_Cell::Ctm_Cell (TFlt hold_cap, TFlt flow_cap,
                                   TFlt flow_scalar, TFlt wave_ratio)
{
  m_hold_cap = hold_cap;
  m_flow_cap = flow_cap;
  m_flow_scalar = flow_scalar;
  if (m_flow_cap * flow_scalar < 1.)
    {
      m_flow_cap
        = 1.
          / flow_scalar; // the design capacity allows at least one car to pass
    }
  m_wave_ratio = wave_ratio;
  m_volume = TInt (0);
  m_veh_queue = std::deque<MNM_Veh *> ();
}

MNM_Dlink_Ctm::Ctm_Cell::~Ctm_Cell () { m_veh_queue.clear (); }

TFlt
MNM_Dlink_Ctm::Ctm_Cell::get_demand ()
{
  TFlt _real_volume = TFlt (m_volume) / m_flow_scalar;
  return std::min (_real_volume, m_flow_cap);
}

TFlt
MNM_Dlink_Ctm::Ctm_Cell::get_supply ()
{
  TFlt _real_volume = TFlt (m_volume) / m_flow_scalar;
  if (_real_volume >= m_hold_cap)
    {
      // _real_volume = m_hold_cap;
      return TFlt (0.0);
    }
  if (m_wave_ratio <= 1.0) // this one is quite tricky, why not just
                           // _min(flwCap, hldCap - curDensity)*wvRatio?
    return m_flow_cap > _real_volume
             ? m_flow_cap
             : TFlt ((m_hold_cap - _real_volume)
                     * m_wave_ratio); // flowCap equals to critical density
  else
    return std::min (m_flow_cap, TFlt (m_hold_cap - _real_volume));
}

TFlt
MNM_Dlink_Ctm::get_link_flow ()
{
  TInt _total_volume = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      _total_volume += m_cell_array[i]->m_volume;
    }
  _total_volume += m_finished_array.size ();
  return TFlt (_total_volume) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Ctm::get_link_flow_emission (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      for (auto veh : m_cell_array[i]->m_veh_queue)
        {
          if (veh->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  for (auto veh : m_finished_array)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Ctm::get_link_tt ()
{ // in seconds, not in unit time interval, can only be called in simulation,
  // not after simulation
  TFlt _cost, _spd;
  TFlt _rho = get_link_flow () / m_number_of_lane
              / m_length;               // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;         // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs; // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (0.001 * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                            / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Ctm::get_link_tt_from_flow (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;                   // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs;           // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (0.001 * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                            / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TInt
MNM_Dlink_Ctm::get_link_freeflow_tt_loading ()
{
  return m_num_cells;
}

/**************************************************************************
                          Point Queue
**************************************************************************/
MNM_Dlink_Pq::MNM_Dlink_Pq (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                            TInt number_of_lane, TFlt length, TFlt ffs,
                            TFlt unit_time, TFlt flow_scalar)
    : MNM_Dlink::MNM_Dlink (ID, number_of_lane, length, ffs)
{
  m_link_type = MNM_TYPE_PQ;
  m_lane_hold_cap = lane_hold_cap;
  m_lane_flow_cap = lane_flow_cap;
  m_flow_scalar = flow_scalar;
  m_hold_cap = m_lane_hold_cap * TFlt (number_of_lane) * m_length;
  // MNM_Ults::round() can randomly round up or down the input
  // m_max_stamp = MNM_Ults::round(m_length / (m_ffs * unit_time));
  // round down time, but ensures m_max_stamp >= 1
  m_max_stamp = MNM_Ults::round_down_time (m_length / (m_ffs * unit_time));
  m_veh_queue = std::deque<std::pair<MNM_Veh *, TInt>> ();
  m_volume = TInt (0);
  m_unit_time = unit_time;
}

MNM_Dlink_Pq::~MNM_Dlink_Pq () { m_veh_queue.clear (); }

TFlt
MNM_Dlink_Pq::get_link_supply ()
{
  return m_lane_flow_cap * TFlt (m_number_of_lane) * m_unit_time;
}

int
MNM_Dlink_Pq::clear_incoming_array (TInt timestamp)
{
  TInt _num_veh_tomove = std::min (TInt (m_incoming_array.size ()),
                                   TInt (get_link_supply () * m_flow_scalar));
  MNM_Veh *_veh;
  for (int i = 0; i < _num_veh_tomove; ++i)
    {
      _veh = m_incoming_array.front ();
      m_incoming_array.pop_front ();
      // node -> evolve first, then link -> clear_incoming_array(), then link ->
      // evolve() this actually leads to vehicle spending m_max_stamp + 1 in
      // this link
      // so we use m_max_stamp - 1 in link -> evolve() to ensure vehicle spends
      // m_max_stamp in this link when m_max_stamp > 1 and 1 when m_max_stamp =
      // 0
      m_veh_queue.push_back (std::pair<MNM_Veh *, TInt> (_veh, TInt (0)));
    }
  m_volume = TInt (m_finished_array.size () + m_veh_queue.size ());
  // move_veh_queue(&m_incoming_array, , m_incoming_array.size());

  // m_cell_array[0] -> m_volume = m_cell_array[0] ->m_veh_queue.size();
  return 0;
}

void
MNM_Dlink_Pq::print_info ()
{
  printf ("Link Dynamic model: Point Queue\n");
  printf ("Real volume in the link: %.4f\n", (TFlt) (m_volume / m_flow_scalar));
  printf ("Finished real volume in the link: %.2f\n",
          (TFlt) (TFlt (m_finished_array.size ()) / m_flow_scalar));
}

int
MNM_Dlink_Pq::evolve (TInt timestamp)
{
  auto _que_it = m_veh_queue.begin ();
  while (_que_it != m_veh_queue.end ())
    {
      if (_que_it->second >= std::max (0, m_max_stamp - 1))
        {
          m_finished_array.push_back (_que_it->first);
          _que_it = m_veh_queue.erase (_que_it); // c++ 11
        }
      else
        {
          _que_it->second += 1;
          _que_it++;
        }
    }

  /* volume */
  // m_volume = TInt(m_finished_array.size() + m_veh_queue.size());

  return 0;
}

TFlt
MNM_Dlink_Pq::get_link_flow ()
{
  return TFlt (m_volume) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Pq::get_link_flow_emission (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh_it : m_veh_queue)
    {
      if (veh_it.first->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }

  for (auto veh : m_finished_array)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Pq::get_link_tt ()
{
  TFlt _cost, _spd;
  TFlt _rho = get_link_flow () / m_number_of_lane
              / m_length;               // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;         // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs; // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Pq::get_link_tt_from_flow (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;                   // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs;           // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TInt
MNM_Dlink_Pq::get_link_freeflow_tt_loading ()
{
  return m_max_stamp; // ensure this >= 1 in constructor
}

/**************************************************************************
                          Link Queue
**************************************************************************/
MNM_Dlink_Lq::MNM_Dlink_Lq (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                            TInt number_of_lane, TFlt length, TFlt ffs,
                            TFlt unit_time, TFlt flow_scalar)
    : MNM_Dlink::MNM_Dlink (ID, number_of_lane, length, ffs)
{
  m_link_type = MNM_TYPE_LQ;
  m_lane_hold_cap = lane_hold_cap;
  m_lane_flow_cap = lane_flow_cap;
  m_flow_scalar = flow_scalar;
  m_hold_cap = m_lane_hold_cap * TFlt (number_of_lane) * m_length;
  m_veh_queue = std::deque<MNM_Veh *> ();
  m_volume = TInt (0);
  m_C = m_lane_flow_cap * TFlt (m_number_of_lane);
  if (m_C * unit_time * flow_scalar < 1.)
    {
      m_lane_flow_cap
        = 1. / (TFlt (m_number_of_lane) * unit_time * flow_scalar);
      m_C = 1. / (unit_time * flow_scalar); // the design capacity allows at
                                            // least one car to pass
    }
  m_k_c = m_lane_flow_cap / m_ffs * TFlt (m_number_of_lane);
  m_unit_time = unit_time;
}

MNM_Dlink_Lq::~MNM_Dlink_Lq () { m_veh_queue.clear (); }

TFlt
MNM_Dlink_Lq::get_link_supply ()
{
  TFlt _cur_density = get_link_flow () / m_length;
  return _cur_density > m_k_c
           ? TFlt (get_flow_from_density (_cur_density) * m_unit_time)
           : TFlt (m_C * m_unit_time);
}

int
MNM_Dlink_Lq::clear_incoming_array (TInt timestamp)
{
  if (TInt (m_incoming_array.size ())
      > TInt (get_link_supply () * m_flow_scalar))
    {
      throw std::runtime_error ("invalid incoming array size");
    }
  move_veh_queue (&m_incoming_array, &m_veh_queue, m_incoming_array.size ());

  m_volume = TInt (m_finished_array.size () + m_veh_queue.size ());
  return 0;
}

void
MNM_Dlink_Lq::print_info ()
{
  printf ("Link Dynamic model: Link Queue\n");
  printf ("Real volume in the link: %.4f\n", (TFlt) (m_volume / m_flow_scalar));
  printf ("Finished real volume in the link: %.2f\n",
          (TFlt) (TFlt (m_finished_array.size ()) / m_flow_scalar));
}

int
MNM_Dlink_Lq::evolve (TInt timestamp)
{
  // printf("1\n");
  TFlt _demand = get_demand ();
  MNM_Veh *_veh;
  if (_demand < TFlt (m_finished_array.size ()) / m_flow_scalar)
    {
      // printf("2\n");
      TInt _veh_to_reduce
        = TInt (m_finished_array.size ()) - TInt (_demand * m_flow_scalar);
      _veh_to_reduce
        = std::min (_veh_to_reduce, TInt (m_finished_array.size ()));
      // printf("_veh_to_reduce %d, finished size is %d\n", _veh_to_reduce(),
      // (int)m_finished_array.size());
      for (int i = 0; i < _veh_to_reduce; ++i)
        {
          _veh = m_finished_array.back ();
          m_veh_queue.push_front (_veh);
          m_finished_array.pop_back ();
        }
    }
  else
    {
      // printf("3\n");
      TInt _veh_to_move = MNM_Ults::round (_demand * m_flow_scalar)
                          - TInt (m_finished_array.size ());
      // printf("demand %f, Veh queue size %d, finished size %d, to move %d \n",
      // (TFlt) _demand(), (int) m_veh_queue.size(),
      // (int)m_finished_array.size(), _veh_to_move());

      _veh_to_move = std::min (_veh_to_move, TInt (m_veh_queue.size ()));

      for (int i = 0; i < _veh_to_move; ++i)
        {
          _veh = m_veh_queue.front ();
          m_finished_array.push_back (_veh);
          m_veh_queue.pop_front ();
        }
    }
  return 0;
}

TFlt
MNM_Dlink_Lq::get_link_flow ()
{
  return TFlt (m_volume) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Lq::get_link_flow_emission (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh : m_veh_queue)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }

  for (auto veh : m_finished_array)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Lq::get_link_tt ()
{
  TFlt _cost, _spd;
  TFlt _rho = get_link_flow () / m_number_of_lane
              / m_length;               // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;         // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs; // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Lq::get_link_tt_from_flow (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;                   // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs;           // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Lq::get_flow_from_density (TFlt density)
{
  TFlt _flow;
  if (density < m_k_c)
    {
      _flow = m_ffs * density;
    }
  else
    {
      TFlt _w = m_lane_flow_cap / (m_lane_hold_cap - m_lane_flow_cap / m_ffs);
      _flow = (m_lane_hold_cap * TFlt (m_number_of_lane) - density) * _w;
      // Only for short links, change the FD shape around rhoj:
      // _flow = MNM_Ults::max(_flow,
      // 0.25*(m_lane_hold_cap*TFlt(m_number_of_lane)-m_k_c) * _w);
    }
  return _flow;
}

TFlt
MNM_Dlink_Lq::get_demand ()
{
  TFlt _density = get_link_flow () / m_length;
  TFlt _demand;
  if (_density < m_k_c)
    {
      _demand = get_flow_from_density (get_link_flow () / m_length);
    }
  else
    {
      _demand = m_C;
    }
  return _demand * m_unit_time;
}

TInt
MNM_Dlink_Lq::get_link_freeflow_tt_loading ()
{
  // throw std::runtime_error("Error, MNM_Dlink_Lq::get_link_freeflow_tt_loading
  // NOT implemented");
  return MNM_Ults::round_up_time (m_length / m_ffs);
}

/**************************************************************************
                          Cumulative curve
**************************************************************************/

MNM_Cumulative_Curve::MNM_Cumulative_Curve ()
{
  // the implementation of CC here actually means at the beginning of interval
  // m_recorder[i].first, a total of m_recorder[i].second vehicles have passed
  // and counted the vehicles arriving at the beginning of interval
  // m_recorder[i].first will be counted and reflected at the end of interval
  // m_recorder[i].first, or equivalently, at the beginning of interval
  // m_recorder[i].first + 1 so when using CC to compute the link travel time
  // for vehicles arriving at the beginning of interval m_recorder[i].first, we
  // should use m_recorder[i].first + 1 as the start_time this also means the
  // maximum allowable m_recorder[i].first == m_total_loading_interval, not
  // m_total_loading_interval - 1
  m_recorder = std::deque<std::pair<TFlt, TFlt>> ();
}

MNM_Cumulative_Curve::~MNM_Cumulative_Curve () { m_recorder.clear (); }

bool
pair_compare (std::pair<TFlt, TFlt> i, std::pair<TFlt, TFlt> j)
{
  return (i.first < j.first);
}

// bool pair_compare(std::pair<TInt, TFlt> i, std::pair<TInt, TFlt> j) {
//     return (i.first < j.first);
// }

bool
pair_compare2 (std::pair<TFlt, TFlt> i, std::pair<TFlt, TFlt> j)
{
  return (i.second < j.second);
}

// bool pair_compare2(std::pair<TInt, TFlt> i, std::pair<TInt, TFlt> j) {
//     return (i.second < j.second);
// }

int
MNM_Cumulative_Curve::arrange ()
{
  std::sort (m_recorder.begin (), m_recorder.end (), pair_compare);
  return 0;
}

int
MNM_Cumulative_Curve::arrange2 ()
{
  std::sort (m_recorder.begin (), m_recorder.end (), pair_compare2);
  return 0;
}

int
MNM_Cumulative_Curve::add_record (std::pair<TFlt, TFlt> r)
{
  m_recorder.push_back (r);
  return 0;
}

int
MNM_Cumulative_Curve::shrink (TInt number)
{
  if (TInt (m_recorder.size ()) > number)
    {
      arrange ();
      while (TInt (m_recorder.size ()) > number)
        {
          m_recorder.pop_back ();
        }
    }
  return 0;
}

int
MNM_Cumulative_Curve::add_increment (std::pair<TFlt, TFlt> r)
{
  // printf("Add increment time %d with flow %f\n", r.first, r.second);
  if (m_recorder.empty ())
    {
      add_record (r);
      return 0;
    }
  // std::pair <TFlt, TFlt> _best = *std::max_element(m_recorder.begin(),
  // m_recorder.end(), pair_compare);
  std::pair<TFlt, TFlt> _best = m_recorder[m_recorder.size () - 1];
  if (MNM_Ults::approximate_less_than (r.first, _best.first))
    {
      throw std::runtime_error (
        "Error, MNM_Cumulative_Curve::add_increment, early time index");
    }
  if (MNM_Ults::approximate_equal (r.first, _best.first))
    {
      // printf("Incrementing\n");
      m_recorder[m_recorder.size () - 1].second += r.second;
    }
  else
    {
      // printf("Pushing\n");
      r.second += _best.second;
      // printf("New r is <%f, %lf>\n", r.first(), r.second());
      m_recorder.push_back (r);
    }

  return 0;
}

TFlt
MNM_Cumulative_Curve::get_approximated_result (TFlt time)
{
  // arrange();
  if (m_recorder.empty ())
    {
      return TFlt (0);
    }
  if (m_recorder.size () == 1)
    {
      return m_recorder[0].second;
    }
  if (m_recorder[0].first >= time)
    {
      return m_recorder[0].second;
    }
  for (size_t i = 1; i < m_recorder.size (); ++i)
    {
      if (m_recorder[i].first >= time)
        {
          return m_recorder[i - 1].second
                 + (m_recorder[i].second - m_recorder[i - 1].second)
                     / (m_recorder[i].first - m_recorder[i - 1].first)
                     * (time - m_recorder[i - 1].first);
        }
    }
  return m_recorder.back ().second;
}

TFlt
MNM_Cumulative_Curve::get_result (TFlt time)
{
  // arrange();
  if (m_recorder.empty ())
    {
      return TFlt (0);
    }
  if (m_recorder.size () == 1)
    {
      return m_recorder[0].second;
    }
  if (m_recorder[0].first >= time)
    {
      return m_recorder[0].second;
    }

  // considering non-decreasing cc record
  // Search for first element x such that result ≤ x
  int j = -1;
  auto _lower_j
    = std::lower_bound (m_recorder.begin (), m_recorder.end (), time,
                        [] (const std::pair<TFlt, TFlt> &r, TFlt v) {
                          return MNM_Ults::approximate_less_than (r.first, v);
                        });
  j = std::distance (m_recorder.begin (), _lower_j);
  IAssert (j >= 1);
  if (_lower_j != m_recorder.end ())
    {
      if (MNM_Ults::approximate_equal (m_recorder[j].first, time))
        {
          // printf("index j-1 %d, time %f, flow %f\n", j-1,
          // m_recorder[j-1].first(), m_recorder[j-1].second()); printf("index j
          // %d, time %f, flow %f\n", j, m_recorder[j].first(),
          // m_recorder[j].second()); if (j+1 < (int)m_recorder.size()) {
          //     printf("index j+1 %d, time %f, flow %f\n", j+1,
          //     m_recorder[j+1].first(), m_recorder[j+1].second());
          // }
          return m_recorder[j].second;
        }
      else if (m_recorder[j].first > time)
        {
          // if (j-2 >= 0) {
          //     printf("index j-2 %d, time %f, flow %f\n", j-2,
          //     m_recorder[j-2].first(), m_recorder[j-2].second());
          // }
          // printf("index j-1 %d, time %f, flow %f\n", j-1,
          // m_recorder[j-1].first(), m_recorder[j-1].second()); printf("index j
          // %d, time %f, flow %f\n", j, m_recorder[j].first(),
          // m_recorder[j].second());

          return m_recorder[j - 1].second; // rounding down
        }
    }

  // // original, loop over
  // for (size_t i = 1; i < m_recorder.size(); ++i) {
  //     if (MNM_Ults::approximate_equal(m_recorder[j].first, time)) { //
  //         return m_recorder[i].second;
  //     } else if (m_recorder[i].first > time) {
  //         return m_recorder[i - 1].second;  // rounding down
  //     }

  //     // if (m_recorder[i].first >= time) {
  //     //     return m_recorder[i].second;  // rounding up
  //     // }
  // }
  return m_recorder.back ().second;
}

// TFlt MNM_Cumulative_Curve::get_result(TInt time) {
//     // arrange();
//     if (m_recorder.empty()) {
//         return TFlt(0);
//     }
//     if (m_recorder.size() == 1) {
//         return m_recorder[0].second;
//     }
//     if (0 >= time) {
//         return m_recorder[0].second;
//     }
//     if (time >= m_recorder.size()) {
//         return m_recorder.back().second;
//     }
//     // find first occurrence >= time
//     auto _ind = std::lower_bound(m_recorder.begin(), m_recorder.end(), time,
//                                  [](const std::pair<TInt, TFlt> &r, TInt
//                                  v){return
//                                  MNM_Ults::approximate_less_than(r.first,
//                                  v);});
//     if (_ind != m_recorder.end()) {
//         int j = std::distance(m_recorder.begin(), _ind);
//         IAssert(j >= 1);
//         if (std::abs(m_recorder[j].first == time)) { //
//             return m_recorder[j].second;
//         }
//         else {
//             return m_recorder[j - 1].second;
//         }
//     }
// }

// TFlt MNM_Cumulative_Curve::get_time(TFlt result)
// {
//   // arrange2();
//   if (m_recorder.size() == 0){
//     return TFlt(-1);
//   }
//   if (m_recorder[0].second >= result){
//     return TFlt(-1);
//   }
//   if (m_recorder.size() == 1){
//     return TFlt(-1);
//   }
//   for (size_t i= m_recorder.size() - 1; i >= 0; --i){
//     if (m_recorder[i].second <= result){
//       return m_recorder[i].first
//           + (m_recorder[i+1].first -
//           m_recorder[i].first)/(m_recorder[i+1].second -
//           m_recorder[i].second)
//             * (result - m_recorder[i].second);
//     }
//   }
//   return TFlt(-1);
// }

TFlt
MNM_Cumulative_Curve::get_time (TFlt result, bool rounding_up)
{
  // arrange2();
  if (m_recorder.empty ())
    {
      return TFlt (-1);
    }
  if (m_recorder[0].second >= result)
    {
      return TFlt (-1);
    }
  if (m_recorder.size () == 1)
    {
      return TFlt (-1);
    }

  // considering non-decreasing cc record
  // Search for first element x such that result ≤ x
  int i = -1;
  int j = -1;
  auto _lower_j
    = std::lower_bound (m_recorder.begin (), m_recorder.end (), result,
                        [] (const std::pair<TFlt, TFlt> &r, TFlt v) {
                          return MNM_Ults::approximate_less_than (r.second, v);
                        });
  j = std::distance (m_recorder.begin (), _lower_j);
  IAssert (j >= 1);
  if (rounding_up)
    {
      return m_recorder[j].first;
    }
  if (_lower_j != m_recorder.end ())
    {
      if (MNM_Ults::approximate_equal (m_recorder[j].second, result))
        {
          // printf("index j-1 %d, time %f, flow %f\n", j-1,
          // m_recorder[j-1].first(), m_recorder[j-1].second()); printf("index j
          // %d, time %f, flow %f\n", j, m_recorder[j].first(),
          // m_recorder[j].second()); if (j+1 < (int)m_recorder.size()) {
          //     printf("index j+1 %d, time %f, flow %f\n", j+1,
          //     m_recorder[j+1].first(), m_recorder[j+1].second());
          // }
          return m_recorder[j].first;
        }
      else if (m_recorder[j].second > result)
        {
          auto _lower_i = std::
            lower_bound (m_recorder.begin (), _lower_j,
                         m_recorder[j - 1].second,
                         [] (const std::pair<TFlt, TFlt> &r, TFlt v) {
                           return MNM_Ults::approximate_less_than (r.second, v);
                         });
          i = std::distance (m_recorder.begin (), _lower_i);
          IAssert (i < j);
          // if (i-1 >= 0) {
          //     printf("index i-1 %d, time %f, flow %f\n", i-1,
          //     m_recorder[i-1].first(), m_recorder[i-1].second());
          // }
          // printf("index i %d, time %f, flow %f\n", i, m_recorder[i].first(),
          // m_recorder[i].second()); printf("index i+1 %d, time %f, flow %f\n",
          // i+1, m_recorder[i+1].first(), m_recorder[i+1].second());
          // printf("index j-1 %d, time %f, flow %f\n", j-1,
          // m_recorder[j-1].first(), m_recorder[j-1].second()); printf("index j
          // %d, time %f, flow %f\n", j, m_recorder[j].first(),
          // m_recorder[j].second()); if (j+1 < (int)m_recorder.size()) {
          //     printf("index j+1 %d, time %f, flow %f\n", j+1,
          //     m_recorder[j+1].first(), m_recorder[j+1].second());
          // }
          return m_recorder[i].first; // rounding down
        }
    }

  // // original, loop over
  // for (size_t i = 1; i < m_recorder.size(); ++i) {
  //     if (MNM_Ults::approximate_equal(m_recorder[j].second, result)) { //
  //         return m_recorder[i].first;
  //     } else if (m_recorder[i].second > result) {
  //         return m_recorder[i - 1].first;  // rounding down
  //     }

  //     // if (m_recorder[i].second >= result) {
  //     //     return m_recorder[i].first;  // rounding up
  //     // }
  // }

  return TFlt (-1);
}

std::string
MNM_Cumulative_Curve::to_string ()
{
  std::string _output = "";
  arrange ();
  for (size_t i = 0; i < m_recorder.size (); ++i)
    {
      _output += std::to_string (int (m_recorder[i].first)) + ","
                 + std::to_string (m_recorder[i].second) + "\n";
    }
  _output.pop_back (); // remove last "\n"
  return _output;
}

/**************************************************************************
                          Link Transmission model
**************************************************************************/
MNM_Dlink_Ltm::MNM_Dlink_Ltm (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                              TInt number_of_lane, TFlt length, TFlt ffs,
                              TFlt unit_time, TFlt flow_scalar)
    : MNM_Dlink::MNM_Dlink (ID, number_of_lane, length, ffs)
{
  m_lane_hold_cap = lane_hold_cap;
  if (m_lane_flow_cap * TFlt (number_of_lane) * unit_time * flow_scalar < 1.)
    {
      m_lane_flow_cap = 1. / (TFlt (number_of_lane) * unit_time * flow_scalar);
    }
  m_lane_flow_cap = lane_flow_cap;
  m_flow_scalar = flow_scalar;
  m_hold_cap = m_lane_hold_cap * TFlt (number_of_lane) * m_length;
  m_veh_queue = std::deque<MNM_Veh *> ();
  m_volume = TInt (0);
  m_current_timestamp = TInt (0);
  m_unit_time = unit_time;
  m_w = m_lane_flow_cap / (m_lane_hold_cap - m_lane_flow_cap / m_ffs);
  m_N_in2 = MNM_Cumulative_Curve ();
  m_N_out2 = MNM_Cumulative_Curve ();
  m_previous_finished_flow = TFlt (0);
  m_record_size
    = TInt (std::max (m_length / m_w, m_length / m_ffs) / m_unit_time) + 1;
}

MNM_Dlink_Ltm::~MNM_Dlink_Ltm () { m_veh_queue.clear (); }

TFlt
MNM_Dlink_Ltm::get_link_flow ()
{
  return TFlt (m_volume) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Ltm::get_link_flow_emission (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh : m_veh_queue)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }

  for (auto veh : m_finished_array)
    {
      if (veh->m_label == ev_label)
        {
          _total_volume_ev += 1;
        }
      else
        {
          _total_volume_nonev += 1;
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Ltm::get_link_tt ()
{
  TFlt _cost, _spd;
  TFlt _rho = get_link_flow () / m_number_of_lane
              / m_length;               // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;         // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs; // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Ltm::get_link_tt_from_flow (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_lane_hold_cap;                   // get the jam density
  TFlt _rhok = m_lane_flow_cap / m_ffs;           // get the critical density
  //  if (abs(rho - rhok) <= 0.0001) cost = POS_INF_INT;
  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost (); // sean: i think we should use rhoj,
                                          // not rhok
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs;
        }
      else
        {
          _spd = std::max (DBL_EPSILON * m_ffs, m_lane_flow_cap * (_rhoj - _rho)
                                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TInt
MNM_Dlink_Ltm::get_link_freeflow_tt_loading ()
{
  throw std::runtime_error (
    "Error, MNM_Dlink_Ltm::get_link_freeflow_tt_loading NOT implemented");
  return 0;
}

void
MNM_Dlink_Ltm::print_info ()
{
  printf ("Link Dynamic model: Link Transmission Model\n");
  printf ("Real volume in the link: %.4f\n", (TFlt) (m_volume / m_flow_scalar));
  printf ("Finished real volume in the link: %.2f\n",
          (TFlt) (TFlt (m_finished_array.size ()) / m_flow_scalar));
}

int
MNM_Dlink_Ltm::clear_incoming_array (TInt timestamp)
{
  // printf("MNM_Dlink_Ltm::clear_incoming_array\n");
  if (TInt (m_incoming_array.size ())
      > TInt (get_link_supply () * m_flow_scalar))
    {
      throw std::runtime_error ("invalid incoming array size");
    }
  m_N_in2.add_increment (
    std::pair<TFlt, TFlt> (TFlt (m_current_timestamp * m_unit_time),
                           TFlt (m_incoming_array.size ()) / m_flow_scalar));
  move_veh_queue (&m_incoming_array, &m_veh_queue, m_incoming_array.size ());

  m_volume = TInt (m_finished_array.size () + m_veh_queue.size ());
  // printf("Finished clear\n");
  return 0;
}

TFlt
MNM_Dlink_Ltm::get_link_supply ()
{
  // printf("MNM_Dlink_Ltm::get_link_supply\n");
  TFlt _recv
    = m_N_out2.get_result (
        TFlt (m_current_timestamp * m_unit_time + m_unit_time) - m_length / m_w)
      + m_hold_cap - m_N_in2.get_result (TFlt (m_current_timestamp));
  TFlt _res
    = std::min (_recv, m_lane_flow_cap * TFlt (m_number_of_lane) * m_unit_time);
  return std::max (_res, TFlt (0));
}

int
MNM_Dlink_Ltm::evolve (TInt timestamp)
{
  // printf("MNM_Dlink_Ltm::evolve\n");
  m_N_in2.shrink (m_record_size);
  m_N_out2.shrink (m_record_size);
  // printf("target is %d, n in size %d, n out size %d\n",m_record_size(),
  // m_N_in2.m_recorder.size(), m_N_out2.m_recorder.size());
  TFlt _demand = get_demand ();
  MNM_Veh *_veh;
  if (_demand < TFlt (m_finished_array.size ()) / m_flow_scalar)
    {
      TInt _veh_to_reduce
        = TInt (m_finished_array.size ()) - TInt (_demand * m_flow_scalar);
      _veh_to_reduce
        = std::min (_veh_to_reduce, TInt (m_finished_array.size ()));
      for (int i = 0; i < _veh_to_reduce; ++i)
        {
          _veh = m_finished_array.back ();
          if (_veh == NULL)
            {
              throw std::runtime_error ("no enough finished vehicles");
            }
          m_veh_queue.push_front (_veh);
          m_finished_array.pop_back ();
        }
    }
  else
    {
      TInt _veh_to_move = MNM_Ults::round (_demand * m_flow_scalar)
                          - TInt (m_finished_array.size ());
      // printf("demand %f, Veh queue size %d, finished size %d, to move %d \n",
      // (TFlt) _demand(), (int) m_veh_queue.size(),
      // (int)m_finished_array.size(), _veh_to_move());

      _veh_to_move = std::min (_veh_to_move, TInt (m_veh_queue.size ()));

      for (int i = 0; i < _veh_to_move; ++i)
        {
          _veh = m_veh_queue.front ();
          if (_veh == NULL)
            {
              throw std::runtime_error ("no enough vehicles in vehicle queue");
            }
          m_finished_array.push_back (_veh);
          m_veh_queue.pop_front ();
        }
    }

  m_current_timestamp += 1;
  m_previous_finished_flow = TFlt (m_finished_array.size ()) / m_flow_scalar;
  return 0;
}

TFlt
MNM_Dlink_Ltm::get_demand ()
{
  // printf("MNM_Dlink_Ltm::get_demand\n");
  TFlt _real_finished_flow = m_previous_finished_flow
                             - TFlt (m_finished_array.size ()) / m_flow_scalar;
  m_N_out2.add_increment (
    std::pair<TFlt, TFlt> (TFlt (m_current_timestamp * m_unit_time),
                           _real_finished_flow));
  TFlt _send
    = m_N_in2.get_result (TFlt (m_current_timestamp * m_unit_time + m_unit_time)
                          - m_length / m_ffs)
      - m_N_out2.get_result (TFlt (m_current_timestamp * m_unit_time));
  return std::min (_send,
                   m_lane_flow_cap * TFlt (m_number_of_lane) * m_unit_time);
}
