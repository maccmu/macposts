#include "multiclass.h"

///
/// Link Models
///

MNM_Dlink_Multiclass::MNM_Dlink_Multiclass (
  TInt ID, TInt number_of_lane,
  TFlt length,  // meters
  TFlt ffs_car, // Free-flow speed (m/s)
  TFlt ffs_truck)
    : MNM_Dlink::MNM_Dlink (ID, number_of_lane, length,
                            ffs_car) // Note: although m_ffs is not used in
                                     // child class, let it be ffs_car
{
  m_ffs_car = ffs_car;
  m_ffs_truck = ffs_truck;

  m_N_in_car = nullptr;
  m_N_out_car = nullptr;
  m_N_in_truck = nullptr;
  m_N_out_truck = nullptr;

  m_N_in_tree_car = nullptr;
  m_N_out_tree_car = nullptr;
  m_N_in_tree_truck = nullptr;
  m_N_out_tree_truck = nullptr;

  // average waiting time per vehicle = tot_wait_time/(tot_num_car +
  // tot_num_truck)
  m_tot_wait_time_at_intersection = 0; // seconds
  // average waiting time per car = tot_car_wait_time/tot_num_car
  m_tot_wait_time_at_intersection_car = 0; // seconds
  // average waiting time per truck = tot_truck_wait_time/tot_num_truck
  m_tot_wait_time_at_intersection_truck = 0; // seconds

  // flag of spill back on this link
  m_spill_back
    = false; // if spill back happens during simulation, then set to true

  m_toll_car = 0;
  m_toll_truck = 0;
}

MNM_Dlink_Multiclass::~MNM_Dlink_Multiclass ()
{
  if (m_N_out_car != nullptr)
    delete m_N_out_car;
  if (m_N_in_car != nullptr)
    delete m_N_in_car;
  if (m_N_out_truck != nullptr)
    delete m_N_out_truck;
  if (m_N_in_truck != nullptr)
    delete m_N_in_truck;
  if (m_N_out_tree_car != nullptr)
    delete m_N_out_tree_car;
  if (m_N_in_tree_car != nullptr)
    delete m_N_in_tree_car;
  if (m_N_out_tree_truck != nullptr)
    delete m_N_out_tree_truck;
  if (m_N_in_tree_truck != nullptr)
    delete m_N_in_tree_truck;
}

int
MNM_Dlink_Multiclass::install_cumulative_curve_multiclass ()
{
  if (m_N_out_car != nullptr)
    delete m_N_out_car;
  if (m_N_in_car != nullptr)
    delete m_N_in_car;
  if (m_N_out_truck != nullptr)
    delete m_N_out_truck;
  if (m_N_in_truck != nullptr)
    delete m_N_in_truck;
  m_N_in_car = new MNM_Cumulative_Curve ();
  m_N_out_car = new MNM_Cumulative_Curve ();
  m_N_in_truck = new MNM_Cumulative_Curve ();
  m_N_out_truck = new MNM_Cumulative_Curve ();
  m_N_in_car->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  m_N_out_car->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  m_N_in_truck->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  m_N_out_truck->add_record (std::pair<TFlt, TFlt> (TFlt (0), TFlt (0)));
  return 0;
}

int
MNM_Dlink_Multiclass::install_cumulative_curve_tree_multiclass ()
{
  if (m_N_out_tree_car != nullptr)
    delete m_N_out_tree_car;
  if (m_N_in_tree_car != nullptr)
    delete m_N_in_tree_car;
  if (m_N_out_tree_truck != nullptr)
    delete m_N_out_tree_truck;
  if (m_N_in_tree_truck != nullptr)
    delete m_N_in_tree_truck;

  // !!! Close all cc_tree if only doing loading to save a lot of memory !!!
  m_N_in_tree_car = new MNM_Tree_Cumulative_Curve ();
  // m_N_out_tree_car = new MNM_Tree_Cumulative_Curve();
  m_N_in_tree_truck = new MNM_Tree_Cumulative_Curve ();
  // m_N_out_tree_truck = new MNM_Tree_Cumulative_Curve();

  return 0;
}

TFlt
MNM_Dlink_Multiclass::get_link_freeflow_tt_car ()
{
  return m_length / m_ffs_car; // seconds, absolute tt
}

TFlt
MNM_Dlink_Multiclass::get_link_freeflow_tt_truck ()
{
  return m_length / m_ffs_truck; // seconds, absolute tt
}

/// Multiclass CTM Functions
/// (currently only for car & truck two classes)
/// (see: Z. (Sean) Qian et al./Trans. Res. Part B 99 (2017) 183-204)

MNM_Dlink_Ctm_Multiclass::MNM_Dlink_Ctm_Multiclass (
  TInt ID, TInt number_of_lane,
  TFlt length,            // (m)
  TFlt lane_hold_cap_car, // Jam density (veh/m/lane)
  TFlt lane_hold_cap_truck,
  TFlt lane_flow_cap_car, // Max flux (veh/s/lane)
  TFlt lane_flow_cap_truck,
  TFlt ffs_car, // Free-flow speed (m/s)
  TFlt ffs_truck,
  TFlt unit_time,          // (s)
  TFlt veh_convert_factor, // 1 * truck = c * private cars
                           // when compute node demand
  TFlt flow_scalar)        // flow_scalar can be 2.0, 5.0, 10.0, etc.
    : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass (ID, number_of_lane, length,
                                                  ffs_car, ffs_truck)
{
  m_link_type = MNM_TYPE_CTM_MULTICLASS;
  // Jam density for private cars and trucks cannot be negative
  if ((lane_hold_cap_car < 0) || (lane_hold_cap_truck < 0))
    {
      throw std::runtime_error ("negative lane_hold_cap for link "
                                + std::to_string (m_link_ID ()));
    }
  // Jam density for private cars cannot be too large
  if (lane_hold_cap_car > TFlt (400) / TFlt (1600))
    {
      // "lane_hold_cap is too large, set to 300 veh/mile
      lane_hold_cap_car = TFlt (400) / TFlt (1600);
    }
  // Jam density for trucks cannot be too large
  if (lane_hold_cap_truck > TFlt (400) / TFlt (1600))
    {
      // "lane_hold_cap is too large, set to 300 veh/mile
      lane_hold_cap_truck = TFlt (400) / TFlt (1600);
    }

  // Maximum flux for private cars and trucks cannot be negative
  if ((lane_flow_cap_car < 0) || (lane_flow_cap_truck < 0))
    {
      throw std::runtime_error ("negative lane_flow_cap for link "
                                + std::to_string (m_link_ID ()));
    }
  // Maximum flux for private cars cannot be too large
  if (lane_flow_cap_car > TFlt (3500) / TFlt (3600))
    {
      // lane_flow_cap is too large, set to 3500 veh/hour
      lane_flow_cap_car = TFlt (3500) / TFlt (3600);
    }
  // Maximum flux for trucks cannot be too large
  if (lane_flow_cap_truck > TFlt (3500) / TFlt (3600))
    {
      // lane_flow_cap is too large, set to 3500 veh/hour
      lane_flow_cap_truck = TFlt (3500) / TFlt (3600);
    }

  if ((ffs_car < 0) || (ffs_truck < 0))
    {
      throw std::runtime_error ("negative ffs for link "
                                + std::to_string (m_link_ID ()));
    }

  if (veh_convert_factor < 1)
    {
      throw std::runtime_error ("invalid veh_convert_factor for link "
                                + std::to_string (m_link_ID ()));
    }

  if (flow_scalar < 1)
    {
      throw std::runtime_error ("invalid flow_scalar for link "
                                + std::to_string (m_link_ID ()));
    }

  if (unit_time <= 0)
    {
      throw std::runtime_error ("negative unit_time for link "
                                + std::to_string (m_link_ID ()));
    }
  m_unit_time = unit_time;
  m_lane_flow_cap_car = lane_flow_cap_car;
  m_lane_flow_cap_truck = lane_flow_cap_truck;
  m_lane_hold_cap_car = lane_hold_cap_car;
  m_lane_hold_cap_truck = lane_hold_cap_truck;
  m_veh_convert_factor = veh_convert_factor;
  m_flow_scalar = flow_scalar;

  m_cell_array = std::vector<Ctm_Cell_Multiclass *> ();

  // Note m_ffs_car > m_ffs_truck, use ffs_car to define the standard cell
  // length
  TFlt _std_cell_length = m_ffs_car * unit_time;
  m_num_cells = TInt (floor (m_length / _std_cell_length));
  // this means the least link travel time is at least one cell!
  if (m_num_cells == 0)
    {
      m_num_cells = 1;
      m_length = _std_cell_length;
    }
  // this means the link travel time is rounded down!
  // the last cell can have larger length than the previous ones
  TFlt _last_cell_length = m_length - TFlt (m_num_cells - 1) * _std_cell_length;

  m_lane_critical_density_car = m_lane_flow_cap_car / m_ffs_car;
  m_lane_critical_density_truck = m_lane_flow_cap_truck / m_ffs_truck;

  if (m_lane_hold_cap_car <= m_lane_critical_density_car)
    {
      throw std::runtime_error ("invalid car parameters for link "
                                + std::to_string (m_link_ID ()));
    }
  m_wave_speed_car
    = m_lane_flow_cap_car / (m_lane_hold_cap_car - m_lane_critical_density_car);

  if (m_lane_hold_cap_truck <= m_lane_critical_density_truck)
    {
      throw std::runtime_error ("invalid truck parameters for link "
                                + std::to_string (m_link_ID ()));
    }
  m_wave_speed_truck
    = m_lane_flow_cap_truck
      / (m_lane_hold_cap_truck - m_lane_critical_density_truck);

  // see the reference paper for definition, Fig. 6
  // m_lane_rho_1_N > m_lane_critical_density_car and
  // m_lane_critical_density_truck
  m_lane_rho_1_N = m_lane_hold_cap_car
                   * (m_wave_speed_car / (m_ffs_truck + m_wave_speed_car));

  init_cell_array (unit_time, _std_cell_length, _last_cell_length);
}

MNM_Dlink_Ctm_Multiclass::~MNM_Dlink_Ctm_Multiclass ()
{
  for (Ctm_Cell_Multiclass *_cell : m_cell_array)
    {
      delete _cell;
    }
  m_cell_array.clear ();
}

int
MNM_Dlink_Ctm_Multiclass::move_veh_queue (std::deque<MNM_Veh *> *from_queue,
                                          std::deque<MNM_Veh *> *to_queue,
                                          TInt number_tomove)
{
  MNM_Veh *_veh;
  MNM_Veh_Multiclass *_veh_multiclass;
  for (int i = 0; i < number_tomove; ++i)
    {
      _veh = from_queue->front ();
      from_queue->pop_front ();
      _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (_veh);
      // update the vehicle position on current link. 0: at the beginning, 1: at
      // the end.
      _veh_multiclass->m_visual_position_on_link
        += float (1) / float (m_num_cells);
      if (_veh_multiclass->m_visual_position_on_link > 0.99)
        _veh_multiclass->m_visual_position_on_link = 0.99;
      to_queue->push_back (_veh);
    }
  return 0;
}

int
MNM_Dlink_Ctm_Multiclass::init_cell_array (TFlt unit_time, TFlt std_cell_length,
                                           TFlt last_cell_length)
{
  // All previous cells
  Ctm_Cell_Multiclass *cell = NULL;
  for (int i = 0; i < m_num_cells - 1; ++i)
    {
      cell = new Ctm_Cell_Multiclass (TInt (i), std_cell_length, unit_time,
                                      // Convert lane parameters to cell (link)
                                      // parameters by multiplying # of lanes
                                      TFlt (m_number_of_lane)
                                        * m_lane_hold_cap_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_hold_cap_truck,
                                      TFlt (m_number_of_lane)
                                        * m_lane_critical_density_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_critical_density_truck,
                                      TFlt (m_number_of_lane) * m_lane_rho_1_N,
                                      TFlt (m_number_of_lane)
                                        * m_lane_flow_cap_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_flow_cap_truck,
                                      m_ffs_car, m_ffs_truck, m_wave_speed_car,
                                      m_wave_speed_truck, m_flow_scalar);
      if (cell == NULL)
        {
          throw std::runtime_error ("failed to create cell");
        }
      m_cell_array.push_back (cell);
    }

  // The last cell
  // last cell must exist as long as link length > 0, see definition above
  if (m_length > 0.0)
    {
      cell = new Ctm_Cell_Multiclass (m_num_cells - 1,
                                      last_cell_length, // Note last cell length
                                                        // is longer but < 2 *
                                                        // standard length of
                                                        // previous length
                                      unit_time,
                                      TFlt (m_number_of_lane)
                                        * m_lane_hold_cap_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_hold_cap_truck,
                                      TFlt (m_number_of_lane)
                                        * m_lane_critical_density_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_critical_density_truck,
                                      TFlt (m_number_of_lane) * m_lane_rho_1_N,
                                      TFlt (m_number_of_lane)
                                        * m_lane_flow_cap_car,
                                      TFlt (m_number_of_lane)
                                        * m_lane_flow_cap_truck,
                                      m_ffs_car, m_ffs_truck, m_wave_speed_car,
                                      m_wave_speed_truck, m_flow_scalar);
      if (cell == NULL)
        {
          throw std::runtime_error ("failed to create cell");
        }
      m_cell_array.push_back (cell);
    }

  // compress the cell_array to reduce space
  m_cell_array.shrink_to_fit ();

  return 0;
}

void
MNM_Dlink_Ctm_Multiclass::print_info ()
{
  printf ("Total number of cell: \t%d\n Flow scalar: \t%.4f\n",
          int (m_num_cells), double (m_flow_scalar));

  printf ("Car volume for each cell is:\n");
  for (int i = 0; i < m_num_cells - 1; ++i)
    {
      printf ("%d, ", int (m_cell_array[i]->m_volume_car));
    }
  printf ("%d\n", int (m_cell_array[m_num_cells - 1]->m_volume_car));

  printf ("Truck volume for each cell is:\n");
  for (int i = 0; i < m_num_cells - 1; ++i)
    {
      printf ("%d, ", int (m_cell_array[i]->m_volume_truck));
    }
  printf ("%d\n", int (m_cell_array[m_num_cells - 1]->m_volume_truck));
}

int
MNM_Dlink_Ctm_Multiclass::update_out_veh ()
{
  TFlt _temp_out_flux_car, _supply_car, _demand_car;
  TFlt _temp_out_flux_truck, _supply_truck, _demand_truck;

  // no update is needed if only one cell
  if (m_num_cells > 1)
    {
      for (int i = 0; i < m_num_cells - 1; ++i)
        {
          // car, veh_type = TInt(0)
          _demand_car = m_cell_array[i]->get_perceived_demand (TInt (0));
          _supply_car = m_cell_array[i + 1]->get_perceived_supply (TInt (0));
          _temp_out_flux_car = m_cell_array[i]->m_space_fraction_car
                               * MNM_Ults::min (_demand_car, _supply_car);
          m_cell_array[i]->m_out_veh_car
            = MNM_Ults::round (_temp_out_flux_car * m_flow_scalar);

          // truck, veh_type = TInt(1)
          _demand_truck = m_cell_array[i]->get_perceived_demand (TInt (1));
          _supply_truck = m_cell_array[i + 1]->get_perceived_supply (TInt (1));
          _temp_out_flux_truck = m_cell_array[i]->m_space_fraction_truck
                                 * MNM_Ults::min (_demand_truck, _supply_truck);
          // MNM_Ults::round() has random effects, averagely, in the free flow
          // condition, this makes a truck travel to next cell with a
          // probability of ffs_truck / ffs_car
          m_cell_array[i]->m_out_veh_truck
            = MNM_Ults::round (_temp_out_flux_truck * m_flow_scalar);
        }
    }
  m_cell_array[m_num_cells - 1]->m_out_veh_car
    = m_cell_array[m_num_cells - 1]->m_veh_queue_car.size ();
  m_cell_array[m_num_cells - 1]->m_out_veh_truck
    = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.size ();
  return 0;
}

int
MNM_Dlink_Ctm_Multiclass::evolve (TInt timestamp)
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
          move_veh_queue (&(m_cell_array[i]->m_veh_queue_car),
                          &(m_cell_array[i + 1]->m_veh_queue_car),
                          _num_veh_tomove_car);
          // Truck
          _num_veh_tomove_truck = m_cell_array[i]->m_out_veh_truck;
          move_veh_queue (&(m_cell_array[i]->m_veh_queue_truck),
                          &(m_cell_array[i + 1]->m_veh_queue_truck),
                          _num_veh_tomove_truck);
        }
    }

  /* last cell */
  move_last_cell ();
  m_tot_wait_time_at_intersection
    += TFlt (m_finished_array.size ()) / m_flow_scalar * m_unit_time;

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
        }
    }

  _count_car = 0;
  _count_truck = 0;
  // m_class: 0 - private car, 1 - truck
  for (_veh_it = m_finished_array.begin (); _veh_it != m_finished_array.end ();
       _veh_it++)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
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

int
MNM_Dlink_Ctm_Multiclass::move_last_cell ()
{
  TInt _num_veh_tomove_car = m_cell_array[m_num_cells - 1]->m_out_veh_car;
  TInt _num_veh_tomove_truck = m_cell_array[m_num_cells - 1]->m_out_veh_truck;
  TFlt _pstar = TFlt (_num_veh_tomove_car)
                / TFlt (_num_veh_tomove_car + _num_veh_tomove_truck);
  MNM_Veh *_veh;
  TFlt _r;
  while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0))
    {
      _r = MNM_Ults::rand_flt ();
      // probability = _pstar to move a car
      if (_r < _pstar)
        {
          // still has car to move
          if (_num_veh_tomove_car > 0)
            {
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_car.front ();
              m_cell_array[m_num_cells - 1]->m_veh_queue_car.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_car--;
            }
          // no car to move, move a truck
          else
            {
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.front ();
              m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_truck--;
            }
        }
      // probability = 1 - _pstar to move a truck
      else
        {
          // still has truck to move
          if (_num_veh_tomove_truck > 0)
            {
              _veh = m_cell_array[m_num_cells - 1]->m_veh_queue_truck.front ();
              m_cell_array[m_num_cells - 1]->m_veh_queue_truck.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_truck--;
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
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_car--;
            }
        }
    }
  return 0;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_supply ()
{
  TFlt _real_volume_both
    = (TFlt (m_cell_array[0]->m_volume_truck) * m_veh_convert_factor
       + TFlt (m_cell_array[0]->m_volume_car))
      / m_flow_scalar;

  // m_cell_length can't be 0 according to implementation above
  TFlt _density = _real_volume_both / (m_cell_array[0]->m_cell_length);
  double _tmp
    = std::min (double (m_cell_array[0]->m_flow_cap_car),
                m_wave_speed_car *(m_cell_array[0]->m_hold_cap_car - _density));

  // only use when network is too large and complex and no other ways solving
  // gridlock.
  _tmp = std::max (_tmp, m_wave_speed_car * 0.25
                           * (m_cell_array[0]->m_hold_cap_car - _density));

  return std::max (0.0, _tmp) * (m_cell_array[0]->m_unit_time);
}

int
MNM_Dlink_Ctm_Multiclass::clear_incoming_array (TInt timestamp)
{
  MNM_Veh_Multiclass *_veh;
  size_t _cur_size = m_incoming_array.size ();
  for (size_t i = 0; i < _cur_size; ++i)
    {
      _veh = dynamic_cast<MNM_Veh_Multiclass *> (m_incoming_array.front ());
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

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_flow_car ()
{
  TInt _total_volume_car = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      _total_volume_car += m_cell_array[i]->m_volume_car;
    }
  std::deque<MNM_Veh *>::iterator _veh_it;
  for (_veh_it = m_finished_array.begin (); _veh_it != m_finished_array.end ();
       _veh_it++)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
      if (_veh->m_class == 0)
        _total_volume_car += 1;
    }
  return TFlt (_total_volume_car) / m_flow_scalar;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_flow_truck ()
{
  TInt _total_volume_truck = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      _total_volume_truck += m_cell_array[i]->m_volume_truck;
    }
  std::deque<MNM_Veh *>::iterator _veh_it;
  for (_veh_it = m_finished_array.begin (); _veh_it != m_finished_array.end ();
       _veh_it++)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
      if (_veh->m_class == 1)
        _total_volume_truck += 1;
    }
  return TFlt (_total_volume_truck) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Ctm_Multiclass::get_link_flow_emission_car (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      for (auto veh : m_cell_array[i]->m_veh_queue_car)
        {
          auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
          if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 0)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

std::vector<TFlt>
MNM_Dlink_Ctm_Multiclass::get_link_flow_emission_truck (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      for (auto veh : m_cell_array[i]->m_veh_queue_truck)
        {
          auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
          if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 1)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_flow ()
{
  // For get_link_tt in adaptive routing
  TInt _total_volume_car = 0;
  TInt _total_volume_truck = 0;
  for (int i = 0; i < m_num_cells; ++i)
    {
      _total_volume_car += m_cell_array[i]->m_volume_car;
      _total_volume_truck += m_cell_array[i]->m_volume_truck;
    }
  return TFlt (_total_volume_car + _total_volume_truck
               + m_finished_array.size ())
         / m_flow_scalar;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_tt ()
{
  // For adaptive routing and emissions, need modification for multiclass cases
  TFlt _cost, _spd;
  // get the density in veh/mile/lane
  TFlt _rho = get_link_flow () / m_number_of_lane / m_length;
  // get the jam density
  TFlt _rhoj = m_lane_hold_cap_car;
  // get the critical density
  TFlt _rhok = m_lane_flow_cap_car / m_ffs_car;

  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost ();
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs_car;
        }
      else
        {
          _spd = MNM_Ults::max (0.001 * m_ffs_car, m_lane_flow_cap_car
                                                     * (_rhoj - _rho)
                                                     / (_rhoj - _rhok) / _rho);
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_car (TFlt flow)
{
  TFlt _cost, _spd;
  // get the density in veh/mile/lane
  TFlt _rho = flow / m_number_of_lane / m_length;
  // get the jam density
  TFlt _rhoj = m_lane_hold_cap_car;
  // get the critical density
  TFlt _rhok = m_lane_flow_cap_car / m_ffs_car;

  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost ();
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs_car;
        }
      else
        {
          _spd = MNM_Ults::max (0.001 * m_ffs_car, m_lane_flow_cap_car
                                                     * (_rhoj - _rho)
                                                     / (_rhoj - _rhok) / _rho);
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Ctm_Multiclass::get_link_tt_from_flow_truck (TFlt flow)
{
  TFlt _cost, _spd;
  // get the density in veh/mile/lane
  TFlt _rho = flow / m_number_of_lane / m_length;
  // get the jam density
  TFlt _rhoj = m_lane_hold_cap_truck;
  // get the critical density
  TFlt _rhok = m_lane_flow_cap_truck / m_ffs_truck;

  if (_rho >= _rhoj)
    {
      _cost = MNM_Ults::max_link_cost ();
    }
  else
    {
      if (_rho <= _rhok)
        {
          _spd = m_ffs_truck;
        }
      else
        {
          _spd = MNM_Ults::max (0.001 * m_ffs_truck,
                                m_lane_flow_cap_truck * (_rhoj - _rho)
                                  / (_rhoj - _rhok) / _rho);
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TInt
MNM_Dlink_Ctm_Multiclass::get_link_freeflow_tt_loading_car ()
{
  return m_num_cells;
}

TInt
MNM_Dlink_Ctm_Multiclass::get_link_freeflow_tt_loading_truck ()
{
  // due the random rounding, this is a random number
  return m_num_cells * m_ffs_car / m_ffs_truck;
}

/// Multiclass CTM cell

MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::Ctm_Cell_Multiclass (
  TInt cell_ID, TFlt cell_length, TFlt unit_time, TFlt hold_cap_car,
  TFlt hold_cap_truck, TFlt critical_density_car, TFlt critical_density_truck,
  TFlt rho_1_N, TFlt flow_cap_car, TFlt flow_cap_truck, TFlt ffs_car,
  TFlt ffs_truck, TFlt wave_speed_car, TFlt wave_speed_truck, TFlt flow_scalar)
{
  m_cell_ID = cell_ID;
  m_cell_length = cell_length;
  m_unit_time = unit_time;
  m_flow_scalar = flow_scalar;

  m_hold_cap_car = hold_cap_car;                     // Veh/m
  m_hold_cap_truck = hold_cap_truck;                 // Veh/m
  m_critical_density_car = critical_density_car;     // Veh/m
  m_critical_density_truck = critical_density_truck; // Veh/m
  m_rho_1_N = rho_1_N;                               // Veh/m
  m_flow_cap_car = flow_cap_car;                     // Veh/s
  m_flow_cap_truck = flow_cap_truck;                 // Veh/s
  m_ffs_car = ffs_car;
  m_ffs_truck = ffs_truck;
  m_wave_speed_car = wave_speed_car;
  m_wave_speed_truck = wave_speed_truck;

  // initialized as car=1, truck=0
  m_space_fraction_car = TFlt (1);
  m_space_fraction_truck = TFlt (0);

  m_volume_car = TInt (0);
  m_volume_truck = TInt (0);
  m_out_veh_car = TInt (0);
  m_out_veh_truck = TInt (0);
  m_veh_queue_car = std::deque<MNM_Veh *> ();
  m_veh_queue_truck = std::deque<MNM_Veh *> ();
}

MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::~Ctm_Cell_Multiclass ()
{
  m_veh_queue_car.clear ();
  m_veh_queue_truck.clear ();
}

int
MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::update_perceived_density ()
{
  TFlt _real_volume_car = TFlt (m_volume_car) / m_flow_scalar;
  TFlt _real_volume_truck = TFlt (m_volume_truck) / m_flow_scalar;

  TFlt _density_car = _real_volume_car / m_cell_length;
  TFlt _density_truck = _real_volume_truck / m_cell_length;

  TFlt _space_fraction_car, _space_fraction_truck;

  // Free-flow traffic (free-flow for both car and truck classes)
  if (_density_car / m_critical_density_car
        + _density_truck / m_critical_density_truck
      <= 1)
    {
      _space_fraction_car = _density_car / m_critical_density_car;
      _space_fraction_truck = _density_truck / m_critical_density_truck;
      m_perceived_density_car
        = _density_car + m_critical_density_car * _space_fraction_truck;
      m_perceived_density_truck
        = _density_truck + m_critical_density_truck * _space_fraction_car;
      if (_space_fraction_car + _space_fraction_truck == 0)
        {
          // same to initial values car=1, truck=0
          m_space_fraction_car = 1;
          m_space_fraction_truck = 0;
        }
      else
        {
          m_space_fraction_car
            = _space_fraction_car
              / (_space_fraction_car + _space_fraction_truck);
          m_space_fraction_truck
            = _space_fraction_truck
              / (_space_fraction_car + _space_fraction_truck);
        }
    }
  // Semi-congested traffic (truck free-flow but car not)
  else if ((_density_truck / m_critical_density_truck < 1)
           && (_density_car / (1 - _density_truck / m_critical_density_truck)
               <= m_rho_1_N))
    {
      _space_fraction_truck = _density_truck / m_critical_density_truck;
      _space_fraction_car = 1 - _space_fraction_truck;
      m_perceived_density_car = _density_car / _space_fraction_car;
      m_perceived_density_truck = m_critical_density_truck;
      m_space_fraction_car = _space_fraction_car;
      m_space_fraction_truck = _space_fraction_truck;
    }
  // Fully congested traffic (both car and truck not free-flow) this case
  // should satisfy: 1. m_perceived_density_car > m_rho_1_N 2.
  // m_perceived_density_truck > m_critical_density_truck
  else
    {
      // _density_truck (m_volume_truck) could still be 0
      if (m_volume_truck == 0)
        {
          m_perceived_density_car = _density_car;
          _space_fraction_car = 1;
          _space_fraction_truck = 0;
          // this case same speed (u) for both private cars and trucks
          TFlt _u
            = (m_hold_cap_car - _density_car) * m_wave_speed_car / _density_car;
          m_perceived_density_truck = (m_hold_cap_truck * m_wave_speed_truck)
                                      / (_u + m_wave_speed_truck);
        }
      // _density_car (m_volume_car) could still be 0 in some extreme case
      else if (m_volume_car == 0)
        {
          m_perceived_density_truck = _density_truck;
          _space_fraction_car = 0;
          _space_fraction_truck = 1;
          // this case same speed (u) for both private cars and trucks
          TFlt _u = (m_hold_cap_truck - _density_truck) * m_wave_speed_truck
                    / _density_truck;
          m_perceived_density_car
            = (m_hold_cap_car * m_wave_speed_car) / (_u + m_wave_speed_car);
        }
      else
        {
          TFlt _tmp_1 = m_hold_cap_car * m_wave_speed_car * _density_truck;
          TFlt _tmp_2 = m_hold_cap_truck * m_wave_speed_truck * _density_car;
          _space_fraction_car = (_density_car * _density_truck
                                   * (m_wave_speed_car - m_wave_speed_truck)
                                 + _tmp_2)
                                / (_tmp_2 + _tmp_1);
          _space_fraction_truck = (_density_car * _density_truck
                                     * (m_wave_speed_truck - m_wave_speed_car)
                                   + _tmp_1)
                                  / (_tmp_2 + _tmp_1);
          m_perceived_density_car = _density_car / _space_fraction_car;
          m_perceived_density_truck = _density_truck / _space_fraction_truck;
        }
      m_space_fraction_car = _space_fraction_car;
      m_space_fraction_truck = _space_fraction_truck;
    }
  return 0;
}

TFlt
MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_demand (
  TInt veh_type)
{
  // car
  if (veh_type == TInt (0))
    {
      return std::min (m_flow_cap_car,
                       TFlt (m_ffs_car * m_perceived_density_car))
             * m_unit_time;
    }
  // truck
  else
    {
      return std::min (m_flow_cap_truck,
                       TFlt (m_ffs_truck * m_perceived_density_truck))
             * m_unit_time;
    }
}

TFlt
MNM_Dlink_Ctm_Multiclass::Ctm_Cell_Multiclass::get_perceived_supply (
  TInt veh_type)
{
  TFlt _tmp;
  // car
  if (veh_type == TInt (0))
    {
      _tmp = std::min (m_flow_cap_car,
                       TFlt (m_wave_speed_car
                             * (m_hold_cap_car - m_perceived_density_car)));
    }
  // truck
  else
    {
      _tmp = std::min (m_flow_cap_truck,
                       TFlt (m_wave_speed_truck
                             * (m_hold_cap_truck - m_perceived_density_truck)));
    }
  return std::max (TFlt (0.0), _tmp) * m_unit_time;
}

/// Multiclass Link-Queue Model

MNM_Dlink_Lq_Multiclass::MNM_Dlink_Lq_Multiclass (
  TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
  TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
  TFlt ffs_car, TFlt ffs_truck, TFlt unit_time, TFlt veh_convert_factor,
  TFlt flow_scalar)
    : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass (ID, number_of_lane, length,
                                                  ffs_car, ffs_truck)
{
  m_link_type = MNM_TYPE_LQ_MULTICLASS;
  m_k_j_car = lane_hold_cap_car * number_of_lane;
  m_k_j_truck = lane_hold_cap_truck * number_of_lane;
  m_C_car = lane_flow_cap_car * number_of_lane;
  m_C_truck = lane_flow_cap_truck * number_of_lane;
  m_k_C_car = m_C_car / ffs_car;
  m_k_C_truck = m_C_truck / ffs_truck;
  m_w_car = m_C_car / (m_k_j_car - m_k_C_car);
  m_w_truck = m_C_truck / (m_k_j_truck - m_k_C_truck);
  m_rho_1_N = m_k_j_car * (m_w_car / (m_ffs_truck + m_w_car));

  m_veh_queue_car = std::deque<MNM_Veh *> ();
  m_veh_queue_truck = std::deque<MNM_Veh *> ();
  m_veh_out_buffer_car = std::deque<MNM_Veh *> ();
  m_veh_out_buffer_truck = std::deque<MNM_Veh *> ();
  m_volume_car = TInt (0);
  m_volume_truck = TInt (0);

  // initialized as car=1, truck=0
  m_space_fraction_car = TFlt (1);
  m_space_fraction_truck = TFlt (0);

  m_flow_scalar = flow_scalar;
  m_unit_time = unit_time;
  m_veh_convert_factor = veh_convert_factor;
}

MNM_Dlink_Lq_Multiclass::~MNM_Dlink_Lq_Multiclass ()
{
  m_veh_queue_car.clear ();
  m_veh_queue_truck.clear ();
  m_veh_out_buffer_car.clear ();
  m_veh_out_buffer_truck.clear ();
}

int
MNM_Dlink_Lq_Multiclass::update_perceived_density ()
{
  TFlt _real_volume_car = TFlt (m_volume_car) / m_flow_scalar;
  TFlt _real_volume_truck = TFlt (m_volume_truck) / m_flow_scalar;

  TFlt _density_car = _real_volume_car / m_length;
  TFlt _density_truck = _real_volume_truck / m_length;

  TFlt _space_fraction_car, _space_fraction_truck;
  // Free-flow traffic (free-flow for both car and truck classes)
  if (_density_car / m_k_C_car + _density_truck / m_k_C_truck <= 1)
    {
      _space_fraction_car = _density_car / m_k_C_car;
      _space_fraction_truck = _density_truck / m_k_C_truck;
      m_perceived_density_car
        = _density_car + m_k_C_car * _space_fraction_truck;
      m_perceived_density_truck
        = _density_truck + m_k_C_truck * _space_fraction_car;
      if (_space_fraction_car + _space_fraction_truck == 0)
        {
          // same to initial values: car=1, truck=0
          m_space_fraction_car = 1;
          m_space_fraction_truck = 0;
        }
      else
        {
          m_space_fraction_car
            = _space_fraction_car
              / (_space_fraction_car + _space_fraction_truck);
          m_space_fraction_truck
            = _space_fraction_truck
              / (_space_fraction_car + _space_fraction_truck);
        }
      // m_space_fraction_truck);
    }
  // Semi-congested traffic (truck free-flow but car not)
  else if ((_density_truck / m_k_C_truck < 1)
           && (_density_car / (1 - _density_truck / m_k_C_truck) <= m_rho_1_N))
    {
      _space_fraction_truck = _density_truck / m_k_C_truck;
      _space_fraction_car = 1 - _space_fraction_truck;
      m_perceived_density_car = _density_car / _space_fraction_car;
      m_perceived_density_truck = m_k_C_truck;
      m_space_fraction_car = _space_fraction_car;
      m_space_fraction_truck = _space_fraction_truck;
    }
  // Fully congested traffic (both car and truck not free-flow) this case
  // should satisfy: 1. m_perceived_density_car > m_rho_1_N 2.
  // m_perceived_density_truck > m_k_C_truck
  else
    {
      // _density_truck (m_volume_truck) could still be 0
      if (m_volume_truck == 0)
        {
          m_perceived_density_car = _density_car;
          _space_fraction_car = 1;
          _space_fraction_truck = 0;
          // this case same speed (u) for both private cars and trucks
          TFlt _u = (m_k_j_car - _density_car) * m_w_car / _density_car;
          m_perceived_density_truck
            = (m_k_j_truck * m_w_truck) / (_u + m_w_truck);
        }
      // _density_car (m_volume_car) could still be 0 in some extreme case
      else if (m_volume_car == 0)
        {
          m_perceived_density_truck = _density_truck;
          _space_fraction_car = 0;
          _space_fraction_truck = 1;
          // this case same speed (u) for both private cars and trucks
          TFlt _u = (m_k_j_truck - _density_truck) * m_w_truck / _density_truck;
          m_perceived_density_car = (m_k_j_car * m_w_car) / (_u + m_w_car);
        }
      else
        {
          TFlt _tmp_1 = m_k_j_car * m_w_car * _density_truck;
          TFlt _tmp_2 = m_k_j_truck * m_w_truck * _density_car;
          _space_fraction_car
            = (_density_car * _density_truck * (m_w_car - m_w_truck) + _tmp_2)
              / (_tmp_2 + _tmp_1);
          _space_fraction_truck
            = (_density_car * _density_truck * (m_w_truck - m_w_car) + _tmp_1)
              / (_tmp_2 + _tmp_1);
          m_perceived_density_car = _density_car / _space_fraction_car;
          m_perceived_density_truck = _density_truck / _space_fraction_truck;
        }
      m_space_fraction_car = _space_fraction_car;
      m_space_fraction_truck = _space_fraction_truck;
    }
  return 0;
}

int
MNM_Dlink_Lq_Multiclass::evolve (TInt timestamp)
{
  // Update volume, perceived density, space fraction, and demand/supply
  std::deque<MNM_Veh *>::iterator _veh_it;
  TInt _count_car = 0;
  TInt _count_truck = 0;
  TInt _count_tot_vehs = 0;
  for (_veh_it = m_finished_array.begin (); _veh_it != m_finished_array.end ();
       _veh_it++)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
      if (_veh->m_class == 0)
        _count_car += 1;
      if (_veh->m_class == 1)
        _count_truck += 1;
    }
  m_volume_car = m_veh_queue_car.size () + _count_car;
  m_volume_truck = m_veh_queue_truck.size () + _count_truck;

  /* compute total volume of link, check if spill back */
  _count_tot_vehs = m_volume_car + m_volume_truck * m_veh_convert_factor;
  if (TFlt (_count_tot_vehs) / m_flow_scalar / m_length > m_k_j_car)
    {
      m_spill_back = true;
    }

  update_perceived_density ();

  TFlt _demand_car
    = m_space_fraction_car
      * std::min (m_C_car, TFlt (m_ffs_car * m_perceived_density_car))
      * m_unit_time;
  TFlt _demand_truck
    = m_space_fraction_truck
      * std::min (m_C_truck, TFlt (m_ffs_truck * m_perceived_density_truck))
      * m_unit_time;
  TFlt _demand = _demand_car + m_veh_convert_factor * _demand_truck;
  TFlt _veh_to_move = _demand * m_flow_scalar - TInt (m_finished_array.size ());

  // Move vehicle from queue to buffer
  MNM_Veh *_v;
  TInt _veh_to_move_car
    = MNM_Ults::round (_veh_to_move * (_demand_car / _demand));
  _veh_to_move_car
    = std::min (_veh_to_move_car, TInt (m_veh_queue_car.size ()));

  TInt _veh_to_move_truck = MNM_Ults::round (
    _veh_to_move * (m_veh_convert_factor * _demand_truck / _demand)
    / m_veh_convert_factor);

  _veh_to_move_truck
    = std::min (_veh_to_move_truck, TInt (m_veh_queue_truck.size ()));
  for (int i = 0; i < _veh_to_move_car; ++i)
    {
      _v = m_veh_queue_car.front ();
      m_veh_out_buffer_car.push_back (_v);
      m_veh_queue_car.pop_front ();
    }
  for (int i = 0; i < _veh_to_move_truck; ++i)
    {
      _v = m_veh_queue_truck.front ();
      m_veh_out_buffer_truck.push_back (_v);
      m_veh_queue_truck.pop_front ();
    }

  // Empty buffers, nothing to move to finished array
  if ((m_veh_out_buffer_car.size () == 0)
      && (m_veh_out_buffer_truck.size () == 0))
    {
      m_tot_wait_time_at_intersection
        += m_finished_array.size () / m_flow_scalar * m_unit_time;
      m_tot_wait_time_at_intersection_car
        += TFlt (_count_car) / m_flow_scalar * m_unit_time;
      m_tot_wait_time_at_intersection_truck
        += TFlt (_count_truck) / m_flow_scalar * m_unit_time;
      return 0;
    }

  // Move vehicles from buffer to finished array
  TInt _num_veh_tomove_car = m_veh_out_buffer_car.size ();
  TInt _num_veh_tomove_truck = m_veh_out_buffer_truck.size ();
  TFlt _pstar = TFlt (_num_veh_tomove_car)
                / TFlt (_num_veh_tomove_car + _num_veh_tomove_truck);
  MNM_Veh *_veh;
  TFlt _r;
  while ((_num_veh_tomove_car > 0) || (_num_veh_tomove_truck > 0))
    {
      _r = MNM_Ults::rand_flt ();
      // probability = _pstar to move a car
      if (_r < _pstar)
        {
          // still has car to move
          if (_num_veh_tomove_car > 0)
            {
              _veh = m_veh_out_buffer_car.front ();
              m_veh_out_buffer_car.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_car--;
            }
          // no car to move, move a truck
          else
            {
              _veh = m_veh_out_buffer_truck.front ();
              m_veh_out_buffer_truck.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_truck--;
            }
        }
      // probability = 1 - _pstar to move a truck
      else
        {
          // still has truck to move
          if (_num_veh_tomove_truck > 0)
            {
              _veh = m_veh_out_buffer_truck.front ();
              m_veh_out_buffer_truck.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_truck--;
            }
          // no truck to move, move a car
          else
            {
              _veh = m_veh_out_buffer_car.front ();
              m_veh_out_buffer_car.pop_front ();
              if (_veh->has_next_link ())
                {
                  m_finished_array.push_back (_veh);
                }
              else
                {
                  throw std::runtime_error ("invalid state");
                }
              _num_veh_tomove_car--;
            }
        }
    }

  if ((m_veh_out_buffer_car.size () != 0)
      || (m_veh_out_buffer_car.size () != 0))
    {
      throw std::runtime_error ("non-empty buffer");
    }
  m_tot_wait_time_at_intersection
    += m_finished_array.size () / m_flow_scalar * m_unit_time;
  _count_car = 0;
  _count_truck = 0;
  for (auto *_v : m_finished_array)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (_v);
      if (_veh->m_class == 0)
        _count_car += 1;
      if (_veh->m_class == 1)
        _count_truck += 1;
    }
  m_tot_wait_time_at_intersection_car
    += TFlt (_count_car) / m_flow_scalar * m_unit_time;
  m_tot_wait_time_at_intersection_truck
    += TFlt (_count_truck) / m_flow_scalar * m_unit_time;
  return 0;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_supply ()
{
  TFlt _supply_car
    = m_space_fraction_car
      * std::min (m_C_car,
                  TFlt (m_w_car * (m_k_j_car - m_perceived_density_car)))
      * m_unit_time;
  TFlt _supply_truck
    = m_space_fraction_truck
      * std::min (m_C_truck,
                  TFlt (m_w_truck * (m_k_j_truck - m_perceived_density_truck)))
      * m_unit_time;

  // Only for short links, change the FD shape around rhoj:
  _supply_car
    = std::max (_supply_car, TFlt (m_space_fraction_car * m_w_car * 0.30
                                   * (m_k_j_car - m_k_C_car) * m_unit_time));
  _supply_truck = std::max (_supply_truck,
                            TFlt (m_space_fraction_truck * m_w_truck * 0.30
                                  * (m_k_j_truck - m_k_C_truck) * m_unit_time));

  TFlt _supply = std::max (TFlt (0.0), _supply_car)
                 + m_veh_convert_factor * std::max (TFlt (0.0), _supply_truck);
  return _supply;
}

int
MNM_Dlink_Lq_Multiclass::clear_incoming_array (TInt timestamp)
{
  MNM_Veh_Multiclass *_veh;
  size_t _cur_size = m_incoming_array.size ();
  for (size_t i = 0; i < _cur_size; ++i)
    {
      _veh = dynamic_cast<MNM_Veh_Multiclass *> (m_incoming_array.front ());
      m_incoming_array.pop_front ();
      if (_veh->m_class == TInt (0))
        {
          m_veh_queue_car.push_back (_veh);
        }
      else
        {
          m_veh_queue_truck.push_back (_veh);
        }
      _veh->m_visual_position_on_link = 0.5;
    }
  return 0;
}

void
MNM_Dlink_Lq_Multiclass::print_info ()
{
  printf ("Link Dynamic model: Multiclass Link Queue\n");
  printf ("Total car volume in the link: %.4f\n",
          (float) (m_volume_car / m_flow_scalar));
  printf ("Total truck volume in the link: %.4f\n",
          (float) (m_volume_truck / m_flow_scalar));
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_flow_car ()
{
  return TFlt (m_volume_car) / m_flow_scalar;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_flow_truck ()
{
  return TFlt (m_volume_truck) / m_flow_scalar;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_flow ()
{
  // For get_link_tt in adaptive routing
  return TFlt (m_volume_car + m_volume_truck) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Lq_Multiclass::get_link_flow_emission_car (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh : m_veh_queue_car)
    {
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 0)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

std::vector<TFlt>
MNM_Dlink_Lq_Multiclass::get_link_flow_emission_truck (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh : m_veh_queue_truck)
    {
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 1)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_tt ()
{
  // For adaptive routing and emissions, need modification for multiclass cases
  TFlt _cost, _spd;
  TFlt _rho = get_link_flow () / m_number_of_lane
              / m_length; // get the density in veh/mile
  TFlt _rhoj = m_k_j_car; // get the jam density
  TFlt _rhok = m_k_C_car; // get the critical density
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
          _spd = m_ffs_car;
        }
      else
        {
          _spd = MNM_Ults::max (DBL_EPSILON * m_ffs_car,
                                m_C_car * (_rhoj - _rho)
                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_car (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_k_j_car;                         // get the jam density
  TFlt _rhok = m_k_C_car;                         // get the critical density
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
          _spd = m_ffs_car;
        }
      else
        {
          _spd = MNM_Ults::max (DBL_EPSILON * m_ffs_car,
                                m_C_car * (_rhoj - _rho)
                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TFlt
MNM_Dlink_Lq_Multiclass::get_link_tt_from_flow_truck (TFlt flow)
{
  TFlt _cost, _spd;
  TFlt _rho = flow / m_number_of_lane / m_length; // get the density in veh/mile
  TFlt _rhoj = m_k_j_truck;                       // get the jam density
  TFlt _rhok = m_k_C_truck;                       // get the critical density
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
          _spd = m_ffs_truck;
        }
      else
        {
          _spd = MNM_Ults::max (DBL_EPSILON * m_ffs_truck,
                                m_C_truck * (_rhoj - _rho)
                                  / ((_rhoj - _rhok) * _rho));
        }
      _cost = m_length / _spd;
    }
  return _cost;
}

TInt
MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_car ()
{
  // throw std::runtime_error("Error,
  // MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_car NOT
  // implemented");
  return MNM_Ults::round_up_time (m_length / m_ffs_car);
}

TInt
MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_truck ()
{
  // throw std::runtime_error("Error,
  // MNM_Dlink_Lq_Multiclass::get_link_freeflow_tt_loading_truck NOT
  // implemented");
  return MNM_Ults::round_up_time (m_length / m_ffs_truck);
}

/// Multiclass point queue model

MNM_Dlink_Pq_Multiclass::MNM_Dlink_Pq_Multiclass (
  TInt ID, TInt number_of_lane, TFlt length, TFlt lane_hold_cap_car,
  TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car, TFlt lane_flow_cap_truck,
  TFlt ffs_car, TFlt ffs_truck, TFlt unit_time, TFlt veh_convert_factor,
  TFlt flow_scalar)
    : MNM_Dlink_Multiclass::MNM_Dlink_Multiclass (ID, number_of_lane, length,
                                                  ffs_car, ffs_truck)
{
  m_link_type = MNM_TYPE_PQ_MULTICLASS;
  // PQ only used for OD connectors, cap/rhoj are all 99999
  // so no need to use truck parameters
  m_lane_hold_cap = lane_hold_cap_car;
  m_lane_flow_cap = lane_flow_cap_car;
  m_flow_scalar = flow_scalar;
  m_hold_cap = m_lane_hold_cap * TFlt (number_of_lane) * m_length;
  // MNM_Ults::round() can randomly round up or down the input
  // m_max_stamp = MNM_Ults::round(m_length/(ffs_car * unit_time));
  // round down time, but ensures m_max_stamp >= 1
  m_max_stamp = MNM_Ults::round_down_time (m_length / (ffs_car * unit_time));
  // printf("m_max_stamp = %d\n", m_max_stamp);
  m_veh_pool = std::unordered_map<MNM_Veh *, TInt> ();
  m_volume_car = TInt (0);
  m_volume_truck = TInt (0);
  m_unit_time = unit_time;
  m_veh_convert_factor = veh_convert_factor;
}

MNM_Dlink_Pq_Multiclass::~MNM_Dlink_Pq_Multiclass () { m_veh_pool.clear (); }

TFlt
MNM_Dlink_Pq_Multiclass::get_link_supply ()
{
  return m_lane_flow_cap * TFlt (m_number_of_lane) * m_unit_time;
  // return 9999999;
}

int
MNM_Dlink_Pq_Multiclass::clear_incoming_array (TInt timestamp)
{
  MNM_Veh_Multiclass *_veh;
  TFlt _to_be_moved = get_link_supply () * m_flow_scalar;
  while (!m_incoming_array.empty ())
    {
      if (_to_be_moved > 0)
        {
          _veh = dynamic_cast<MNM_Veh_Multiclass *> (m_incoming_array.front ());
          m_incoming_array.pop_front ();
          // node -> evolve first, then link -> clear_incoming_array(), then
          // link -> evolve() this actually leads to vehicle spending
          // m_max_stamp + 1 in this link
          // so we use m_max_stamp - 1 in link -> evolve() to ensure vehicle
          // spends m_max_stamp in this link when m_max_stamp > 1 and 1 when
          // m_max_stamp = 0
          m_veh_pool.insert ({ _veh, TInt (0) });
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
            }
        }
      else
        {
          break;
        }
    }

  m_volume_car = 0;
  m_volume_truck = 0;
  for (auto _veh_it : m_veh_pool)
    {
      auto *_veh_multiclass
        = dynamic_cast<MNM_Veh_Multiclass *> (_veh_it.first);
      if (_veh_multiclass->m_class == 0)
        m_volume_car += 1;
      if (_veh_multiclass->m_class == 1)
        m_volume_truck += 1;
    }
  for (auto _veh_it : m_finished_array)
    {
      auto *_veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (_veh_it);
      if (_veh_multiclass->m_class == 0)
        m_volume_car += 1;
      if (_veh_multiclass->m_class == 1)
        m_volume_truck += 1;
    }
  // printf("car: %d, truck: %d\n", m_volume_car, m_volume_truck);
  return 0;
}

void
MNM_Dlink_Pq_Multiclass::print_info ()
{
  printf ("Link Dynamic model: Multiclass Point Queue\n");
  printf ("Total car volume in the link: %.4f\n",
          (float) (m_volume_car / m_flow_scalar));
  printf ("Total truck volume in the link: %.4f\n",
          (float) (m_volume_truck / m_flow_scalar));
}

int
MNM_Dlink_Pq_Multiclass::evolve (TInt timestamp)
{
  std::unordered_map<MNM_Veh *, TInt>::iterator _que_it = m_veh_pool.begin ();
  MNM_Veh_Multiclass *_veh;
  TInt _num_car = 0, _num_truck = 0;
  while (_que_it != m_veh_pool.end ())
    {
      // we use m_max_stamp - 1 in link -> evolve() to ensure vehicle spends
      // m_max_stamp in this link when m_max_stamp > 1 and 1 when m_max_stamp =
      // 0
      if (_que_it->second >= MNM_Ults::max (0, m_max_stamp - 1))
        {
          m_finished_array.push_back (_que_it->first);
          _veh = dynamic_cast<MNM_Veh_Multiclass *> (m_finished_array.back ());
          if (_veh->m_class == 0)
            {
              _num_car += 1;
            }
          else
            {
              _num_truck += 1;
            }
          _que_it = m_veh_pool.erase (_que_it); // c++ 11
        }
      else
        {
          _que_it->second += 1;
          _que_it++;
        }
    }
  // printf("car: %d, truck: %d\n", _num_car, _num_truck);
  m_tot_wait_time_at_intersection
    += m_finished_array.size () / m_flow_scalar * m_unit_time;
  TInt _count_car = 0;
  TInt _count_truck = 0;
  for (auto *_v : m_finished_array)
    {
      MNM_Veh_Multiclass *_veh = dynamic_cast<MNM_Veh_Multiclass *> (_v);
      if (_veh->m_class == 0)
        _count_car += 1;
      if (_veh->m_class == 1)
        _count_truck += 1;
    }
  m_tot_wait_time_at_intersection_car
    += TFlt (_count_car) / m_flow_scalar * m_unit_time;
  m_tot_wait_time_at_intersection_truck
    += TFlt (_count_truck) / m_flow_scalar * m_unit_time;
  return 0;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_flow_car ()
{
  return TFlt (m_volume_car) / m_flow_scalar;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_flow_truck ()
{
  return TFlt (m_volume_truck) / m_flow_scalar;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_flow ()
{
  // For adaptive routing, need modification for multiclass case
  return TFlt (m_volume_car + m_volume_truck) / m_flow_scalar;
}

std::vector<TFlt>
MNM_Dlink_Pq_Multiclass::get_link_flow_emission_car (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh_it : m_veh_pool)
    {
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh_it.first);
      if (_veh_multiclass->m_class == 0)
        {
          if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 0)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

std::vector<TFlt>
MNM_Dlink_Pq_Multiclass::get_link_flow_emission_truck (TInt ev_label)
{
  TInt _total_volume_ev = 0, _total_volume_nonev = 0;

  for (auto veh_it : m_veh_pool)
    {
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh_it.first);
      if (_veh_multiclass->m_class == 1)
        {
          if (_veh_multiclass->m_label == ev_label)
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
      auto _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (veh);
      if (_veh_multiclass->m_class == 1)
        {
          if (_veh_multiclass->m_label == ev_label)
            {
              _total_volume_ev += 1;
            }
          else
            {
              _total_volume_nonev += 1;
            }
        }
    }
  std::vector<TFlt> _r = { TFlt (_total_volume_nonev) / m_flow_scalar,
                           TFlt (_total_volume_ev) / m_flow_scalar };
  return _r;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_tt ()
{
  // FOR DEBUG ONLY RETURN FREE-FLOW TT
  return m_length / m_ffs_car;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_car (TFlt flow)
{
  return m_length / m_ffs_car;
}

TFlt
MNM_Dlink_Pq_Multiclass::get_link_tt_from_flow_truck (TFlt flow)
{
  return m_length / m_ffs_car;
}

TInt
MNM_Dlink_Pq_Multiclass::get_link_freeflow_tt_loading_car ()
{
  return m_max_stamp; // ensure this >= 1 in constructor
}

TInt
MNM_Dlink_Pq_Multiclass::get_link_freeflow_tt_loading_truck ()
{
  return m_max_stamp; // ensure this >= 1 in constructor
}

///
/// Node models
///

/// Origin node

MNM_DMOND_Multiclass::MNM_DMOND_Multiclass (TInt ID, TFlt flow_scalar,
                                            TFlt veh_convert_factor)
    : MNM_DMOND::MNM_DMOND (ID, flow_scalar)
{
  m_veh_convert_factor = veh_convert_factor;
}

MNM_DMOND_Multiclass::~MNM_DMOND_Multiclass () { ; }

int
MNM_DMOND_Multiclass::evolve (TInt timestamp)
{
  MNM_Dlink *_link;
  MNM_Veh_Multiclass *_veh;
  MNM_Dlink_Pq_Multiclass *_next_link;

  for (unsigned i = 0; i < m_out_link_array.size (); ++i)
    {
      _link = m_out_link_array[i];
      m_out_volume[_link] = 0;
    }

  /* compute out flow */
  std::deque<MNM_Veh *>::iterator _que_it = m_in_veh_queue.begin ();
  while (_que_it != m_in_veh_queue.end ())
    {
      _veh = dynamic_cast<MNM_Veh_Multiclass *> (*_que_it);
      _link = _veh->get_next_link ();
      if (_veh->m_class == 0)
        {
          m_out_volume[_link] += 1;
        }
      else
        {
          _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link);
          // m_out_volume[_link] += _next_link -> m_veh_convert_factor;
          m_out_volume[_link] += 1;
        }
      _que_it++;
    }
  for (unsigned i = 0; i < m_out_link_array.size (); ++i)
    {
      _link = m_out_link_array[i];
      if ((_link->get_link_supply () * m_flow_scalar)
          < TFlt (m_out_volume[_link]))
        {
          m_out_volume[_link] = TInt (
            MNM_Ults::round (_link->get_link_supply () * m_flow_scalar));
        }
    }

  /* move vehicle */
  TInt _moved_car, _moved_truck;
  for (unsigned i = 0; i < m_out_link_array.size (); ++i)
    {
      _link = m_out_link_array[i];
      _moved_car = 0;
      _moved_truck = 0;
      _que_it = m_in_veh_queue.begin ();
      while (_que_it != m_in_veh_queue.end ())
        {
          if (m_out_volume[_link] > 0)
            {
              _veh = dynamic_cast<MNM_Veh_Multiclass *> (*_que_it);
              if (_veh->get_next_link () == _link)
                {
                  _link->m_incoming_array.push_back (_veh);
                  _veh->set_current_link (_link);
                  if (_veh->m_class == 0)
                    {
                      m_out_volume[_link] -= 1;
                      _moved_car += 1;
                    }
                  else
                    {
                      _next_link
                        = dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link);
                      // m_out_volume[_link] -= _next_link ->
                      // m_veh_convert_factor;
                      m_out_volume[_link] -= 1;
                      // only for non-bus truck
                      if (_veh->get_bus_route_ID () == -1)
                        _moved_truck += 1;
                    }
                  _que_it = m_in_veh_queue.erase (_que_it); // c++ 11
                }
              else
                {
                  _que_it++;
                }
            }
          else
            {
              break; // break while loop
            }
        }
      // record cc for both classes
      _next_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link);
      if (_next_link->m_N_in_car != nullptr && _moved_car > 0)
        {
          _next_link->m_N_in_car->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_moved_car) / m_flow_scalar));
        }
      if (_next_link->m_N_in_truck != nullptr && _moved_truck > 0)
        {
          _next_link->m_N_in_truck->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_moved_truck) / m_flow_scalar));
        }
      // printf("car: %d, truck: %d\n", _moved_car, _moved_truck);
    }
  return 0;
}

/// Destination node

MNM_DMDND_Multiclass::MNM_DMDND_Multiclass (TInt ID, TFlt flow_scalar,
                                            TFlt veh_convert_factor)
    : MNM_DMDND::MNM_DMDND (ID, flow_scalar)
{
  m_veh_convert_factor = veh_convert_factor;
}

MNM_DMDND_Multiclass::~MNM_DMDND_Multiclass () { ; }

int
MNM_DMDND_Multiclass::evolve (TInt timestamp)
{
  MNM_Dlink *_link;
  MNM_Dlink_Pq_Multiclass *_from_link;
  MNM_Veh_Multiclass *_veh;
  size_t _size;
  TInt _moved_car, _moved_truck;
  for (size_t i = 0; i < m_in_link_array.size (); ++i)
    {
      _moved_car = 0;
      _moved_truck = 0;
      _link = m_in_link_array[i];
      _size = _link->m_finished_array.size ();
      for (size_t j = 0; j < _size; ++j)
        {
          _veh = dynamic_cast<MNM_Veh_Multiclass *> (
            _link->m_finished_array.front ());
          if (_veh->get_next_link () != nullptr)
            {
              printf ("Something wrong in DMDND evolve\n");
              exit (-1);
            }
          m_out_veh_queue.push_back (_veh);
          _veh->set_current_link (nullptr);
          if (_veh->m_class == 0)
            {
              _moved_car += 1;
            }
          else
            {
              // only for non-bus truck
              if (_veh->get_bus_route_ID () == -1)
                _moved_truck += 1;
            }
          _link->m_finished_array.pop_front ();
        }
      // record cc for both classes
      _from_link = dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link);
      if (_from_link->m_N_out_car != nullptr && _moved_car > 0)
        {
          _from_link->m_N_out_car->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_moved_car) / m_flow_scalar));
        }
      if (_from_link->m_N_out_truck != nullptr && _moved_truck > 0)
        {
          _from_link->m_N_out_truck->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_moved_truck) / m_flow_scalar));
        }
    }
  return 0;
}

/// In-out node

MNM_Dnode_Inout_Multiclass::MNM_Dnode_Inout_Multiclass (TInt ID,
                                                        TFlt flow_scalar,
                                                        TFlt veh_convert_factor)
    : MNM_Dnode::MNM_Dnode (ID, flow_scalar)
{
  m_demand = NULL;
  m_supply = NULL;
  m_veh_flow = NULL;
  m_veh_moved_car = NULL;
  m_veh_moved_truck = NULL;
  m_veh_convert_factor = veh_convert_factor;
}

MNM_Dnode_Inout_Multiclass::~MNM_Dnode_Inout_Multiclass ()
{
  if (m_demand != NULL)
    free (m_demand);
  if (m_supply != NULL)
    free (m_supply);
  if (m_veh_flow != NULL)
    free (m_veh_flow);
  if (m_veh_moved_car != NULL)
    free (m_veh_moved_car);
  if (m_veh_moved_truck != NULL)
    free (m_veh_moved_truck);
}

int
MNM_Dnode_Inout_Multiclass::prepare_loading ()
{
  TInt _num_in = m_in_link_array.size ();
  TInt _num_out = m_out_link_array.size ();
  m_demand = (TFlt *) malloc (sizeof (TFlt) * _num_in
                              * _num_out); // real-world vehicles
  memset (m_demand, 0x0, sizeof (TFlt) * _num_in * _num_out);
  m_supply = (TFlt *) malloc (sizeof (TFlt) * _num_out); // real-world vehicles
  memset (m_supply, 0x0, sizeof (TFlt) * _num_out);
  m_veh_flow = (TFlt *) malloc (sizeof (TFlt) * _num_in
                                * _num_out); // real-world vehicles
  memset (m_veh_flow, 0x0, sizeof (TFlt) * _num_in * _num_out);
  m_veh_moved_car = (TFlt *) malloc (
    sizeof (TFlt) * _num_in
    * _num_out); // simulation vehicles = real-world vehicles * flow scalar
  memset (m_veh_moved_car, 0x0, sizeof (TFlt) * _num_in * _num_out);
  m_veh_moved_truck = (TFlt *) malloc (
    sizeof (TFlt) * _num_in
    * _num_out); // simulation vehicles = real-world vehicles * flow scalar
  memset (m_veh_moved_truck, 0x0, sizeof (TFlt) * _num_in * _num_out);
  return 0;
}

int
MNM_Dnode_Inout_Multiclass::prepare_supplyANDdemand ()
{
  size_t _num_in = m_in_link_array.size ();
  size_t _num_out = m_out_link_array.size ();
  size_t _offset = m_out_link_array.size ();
  TFlt _equiv_count;
  std::deque<MNM_Veh *>::iterator _veh_it;
  MNM_Dlink *_in_link, *_out_link;

  /* zerolize num of vehicle moved */
  memset (m_veh_moved_car, 0x0, sizeof (TFlt) * _num_in * _num_out);
  memset (m_veh_moved_truck, 0x0, sizeof (TFlt) * _num_in * _num_out);

  /* calculate demand */
  for (size_t i = 0; i < _num_in; ++i)
    {
      _in_link = m_in_link_array[i];
      for (_veh_it = _in_link->m_finished_array.begin ();
           _veh_it != _in_link->m_finished_array.end (); _veh_it++)
        {
          if (std::find (m_out_link_array.begin (), m_out_link_array.end (),
                         (*_veh_it)->get_next_link ())
              == m_out_link_array.end ())
            {
              throw std::runtime_error ("vehicle in wrong node");
            }
        }
      for (size_t j = 0; j < _num_out; ++j)
        {
          _out_link = m_out_link_array[j];
          _equiv_count = 0;
          for (_veh_it = _in_link->m_finished_array.begin ();
               _veh_it != _in_link->m_finished_array.end (); _veh_it++)
            {
              MNM_Veh_Multiclass *_veh
                = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
              if (_veh->get_next_link () == _out_link)
                {
                  if (_veh->m_class == 0)
                    {
                      // private car
                      _equiv_count += 1;
                    }
                  else
                    {
                      // truck
                      _equiv_count += m_veh_convert_factor;
                      // _equiv_count += 1;
                    }
                }
            }
          m_demand[_offset * i + j] = _equiv_count / m_flow_scalar;
        }
    }

  /* calculated supply */
  for (size_t j = 0; j < _num_out; ++j)
    {
      // printf("%d, %d\n", j, m_out_link_array[j] -> m_link_ID);
      m_supply[j] = m_out_link_array[j]->get_link_supply ();
    }

  return 0;
}

int
MNM_Dnode_Inout_Multiclass::move_vehicle (TInt timestamp)
{
  MNM_Dlink *_in_link, *_out_link;
  MNM_Dlink_Multiclass *_ilink, *_olink;
  size_t _offset = m_out_link_array.size ();
  TFlt _to_move;
  TFlt _equiv_num;
  TFlt _r;

  std::vector<size_t> _in_link_ind_array = std::vector<size_t> ();
  for (size_t i = 0; i < m_in_link_array.size (); ++i)
    {
      _in_link_ind_array.push_back (i);
    }

  for (size_t j = 0; j < m_out_link_array.size (); ++j)
    {
      _out_link = m_out_link_array[j];

      // shuffle the in links, reserve the FIFO
      std::random_device rng; // random sequence
      std::shuffle (_in_link_ind_array.begin (), _in_link_ind_array.end (),
                    rng);
      for (size_t i : _in_link_ind_array)
        {
          _in_link = m_in_link_array[i];
          _to_move = m_veh_flow[i * _offset + j] * m_flow_scalar;
          auto _veh_it = _in_link->m_finished_array.begin ();

          while (_veh_it != _in_link->m_finished_array.end ())
            {
              if (_to_move > 0)
                {
                  MNM_Veh_Multiclass *_veh
                    = dynamic_cast<MNM_Veh_Multiclass *> (*_veh_it);
                  if (_veh->get_next_link () == _out_link)
                    {
                      // printf("%d ", _veh -> m_class);
                      if (_veh->m_class == 0)
                        {
                          // private car
                          _equiv_num = 1;
                        }
                      else
                        {
                          // truck
                          _equiv_num = m_veh_convert_factor;
                          // _equiv_num = 1;
                        }
                      if (_to_move < _equiv_num)
                        {
                          // Randomly decide to move or not in this case base on
                          // the probability = _to_move/_equiv_num < 1 Will
                          // result in WRONG INCOMING ARRAY SIZE if the
                          // beginning check in function
                          // MNM_Dlink_Ctm_Multiclass::clear_incoming_array()
                          // was not commented out (@_@)!

                          // Always move 1 more vehicle
                          _r = 0;
                          if (_r <= _to_move / _equiv_num)
                            {
                              _out_link->m_incoming_array.push_back (_veh);
                              _veh->set_current_link (_out_link);
                              if (_veh->m_class == 0)
                                {
                                  m_veh_moved_car[i * _offset + j] += 1;
                                  _olink
                                    = dynamic_cast<MNM_Dlink_Multiclass *> (
                                      _out_link);
                                  _ilink
                                    = dynamic_cast<MNM_Dlink_Multiclass *> (
                                      _in_link);
                                  if (_olink->m_N_in_tree_car != nullptr)
                                    {
                                      _olink->m_N_in_tree_car
                                        ->add_flow (TFlt (timestamp + 1),
                                                    1 / m_flow_scalar,
                                                    _veh->m_path,
                                                    _veh->m_assign_interval);
                                    }
                                }
                              else
                                {
                                  IAssert (_veh->m_class == 1);
                                  // only for non-bus truck
                                  if (_veh->get_bus_route_ID () == -1)
                                    m_veh_moved_truck[i * _offset + j] += 1;
                                  _olink
                                    = dynamic_cast<MNM_Dlink_Multiclass *> (
                                      _out_link);
                                  _ilink
                                    = dynamic_cast<MNM_Dlink_Multiclass *> (
                                      _in_link);
                                  if (_olink->m_N_in_tree_truck != nullptr)
                                    {
                                      _olink->m_N_in_tree_truck
                                        ->add_flow (TFlt (timestamp + 1),
                                                    1 / m_flow_scalar,
                                                    _veh->m_path,
                                                    _veh->m_assign_interval);
                                    }
                                }
                              _veh_it
                                = _in_link->m_finished_array.erase (_veh_it);
                            }
                        }
                      else
                        {
                          _out_link->m_incoming_array.push_back (_veh);
                          _veh->set_current_link (_out_link);
                          if (_veh->m_class == 0)
                            {
                              m_veh_moved_car[i * _offset + j] += 1;
                              _olink = dynamic_cast<MNM_Dlink_Multiclass *> (
                                _out_link);
                              _ilink = dynamic_cast<MNM_Dlink_Multiclass *> (
                                _in_link);
                              if (_olink->m_N_in_tree_car != nullptr)
                                {
                                  _olink->m_N_in_tree_car
                                    ->add_flow (TFlt (timestamp + 1),
                                                1 / m_flow_scalar, _veh->m_path,
                                                _veh->m_assign_interval);
                                }
                            }
                          else
                            {
                              IAssert (_veh->m_class == 1);
                              // only for non-bus truck
                              if (_veh->get_bus_route_ID () == -1)
                                m_veh_moved_truck[i * _offset + j] += 1;
                              _olink = dynamic_cast<MNM_Dlink_Multiclass *> (
                                _out_link);
                              _ilink = dynamic_cast<MNM_Dlink_Multiclass *> (
                                _in_link);
                              if (_olink->m_N_in_tree_truck != nullptr)
                                {
                                  _olink->m_N_in_tree_truck
                                    ->add_flow (TFlt (timestamp + 1),
                                                1 / m_flow_scalar, _veh->m_path,
                                                _veh->m_assign_interval);
                                }
                            }
                          _veh_it = _in_link->m_finished_array.erase (_veh_it);
                        }
                      _to_move -= _equiv_num;
                    }
                  else
                    {
                      _veh_it++;
                    }
                }
              else
                {
                  break;
                }
            }
          if (_to_move > 0.001)
            {
              throw std::runtime_error ("invalid state");
            }
        }
    }
  _in_link_ind_array.clear ();
  return 0;
}

int
MNM_Dnode_Inout_Multiclass::record_cumulative_curve (TInt timestamp)
{
  TInt _temp_sum_car, _temp_sum_truck;
  MNM_Dlink_Multiclass *_in_link, *_out_link;
  size_t _offset = m_out_link_array.size ();

  for (size_t j = 0; j < m_out_link_array.size (); ++j)
    {
      _temp_sum_car = 0;
      _temp_sum_truck = 0;
      _out_link = dynamic_cast<MNM_Dlink_Multiclass *> (m_out_link_array[j]);
      for (size_t i = 0; i < m_in_link_array.size (); ++i)
        {
          // _in_link = dynamic_cast<MNM_Dlink_Multiclass
          // *>(m_in_link_array[i]);
          _temp_sum_car += m_veh_moved_car[i * _offset + j];
          _temp_sum_truck += m_veh_moved_truck[i * _offset + j];
        }
      if (_out_link->m_N_in_car != nullptr && _temp_sum_car > 0)
        {
          _out_link->m_N_in_car->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_temp_sum_car) / m_flow_scalar));
        }
      if (_out_link->m_N_in_truck != nullptr && _temp_sum_truck > 0)
        {
          _out_link->m_N_in_truck->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_temp_sum_truck) / m_flow_scalar));
        }
    }

  for (size_t i = 0; i < m_in_link_array.size (); ++i)
    {
      _temp_sum_car = 0;
      _temp_sum_truck = 0;
      _in_link = dynamic_cast<MNM_Dlink_Multiclass *> (m_in_link_array[i]);
      for (size_t j = 0; j < m_out_link_array.size (); ++j)
        {
          // _out_link = dynamic_cast<MNM_Dlink_Multiclass
          // *>(m_out_link_array[j]);
          _temp_sum_car += m_veh_moved_car[i * _offset + j];
          _temp_sum_truck += m_veh_moved_truck[i * _offset + j];
        }
      if (_in_link->m_N_out_car != nullptr && _temp_sum_car > 0)
        {
          _in_link->m_N_out_car->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_temp_sum_car) / m_flow_scalar));
        }
      if (_in_link->m_N_out_truck != nullptr && _temp_sum_truck > 0)
        {
          _in_link->m_N_out_truck->add_increment (
            std::pair<TFlt, TFlt> (TFlt (timestamp + 1),
                                   TFlt (_temp_sum_truck) / m_flow_scalar));
        }
    }

  return 0;
}

int
MNM_Dnode_Inout_Multiclass::add_out_link (MNM_Dlink *out_link)
{
  m_out_link_array.push_back (out_link);
  return 0;
}

int
MNM_Dnode_Inout_Multiclass::add_in_link (MNM_Dlink *in_link)
{
  m_in_link_array.push_back (in_link);
  return 0;
}

int
MNM_Dnode_Inout_Multiclass::evolve (TInt timestamp)
{
  prepare_supplyANDdemand ();
  compute_flow ();
  move_vehicle (timestamp);
  record_cumulative_curve (timestamp);
  return 0;
}

/// FWJ node

MNM_Dnode_FWJ_Multiclass::MNM_Dnode_FWJ_Multiclass (TInt ID, TFlt flow_scalar,
                                                    TFlt veh_convert_factor)
    : MNM_Dnode_Inout_Multiclass::
        MNM_Dnode_Inout_Multiclass (ID, flow_scalar, veh_convert_factor)
{
}

MNM_Dnode_FWJ_Multiclass::~MNM_Dnode_FWJ_Multiclass () {}

int
MNM_Dnode_FWJ_Multiclass::compute_flow ()
{
  size_t _offset = m_out_link_array.size ();
  TFlt _sum_in_flow, _portion;
  for (size_t j = 0; j < m_out_link_array.size (); ++j)
    {
      _sum_in_flow = TFlt (0);
      for (size_t i = 0; i < m_in_link_array.size (); ++i)
        {
          _sum_in_flow += m_demand[i * _offset + j];
        }
      for (size_t i = 0; i < m_in_link_array.size (); ++i)
        {
          _portion = MNM_Ults::divide (m_demand[i * _offset + j], _sum_in_flow);
          m_veh_flow[i * _offset + j]
            = MNM_Ults::min (m_demand[i * _offset + j], _portion * m_supply[j]);
        }
    }

  return 0;
}

/// General Road Junction node

MNM_Dnode_GRJ_Multiclass::MNM_Dnode_GRJ_Multiclass (TInt ID, TFlt flow_scalar,
                                                    TFlt veh_convert_factor)
    : MNM_Dnode_Inout_Multiclass::
        MNM_Dnode_Inout_Multiclass (ID, flow_scalar, veh_convert_factor)
{
  m_d_a = nullptr;
  m_C_a = nullptr;
}

MNM_Dnode_GRJ_Multiclass::~MNM_Dnode_GRJ_Multiclass ()
{
  if (m_d_a != nullptr)
    free (m_d_a);
  if (m_C_a != nullptr)
    free (m_C_a);
}

int
MNM_Dnode_GRJ_Multiclass::prepare_loading ()
{
  MNM_Dnode_Inout_Multiclass::prepare_loading ();
  TInt _num_in = m_in_link_array.size ();
  m_d_a = (TFlt *) malloc (sizeof (TFlt) * _num_in);
  memset (m_d_a, 0x0, sizeof (TFlt) * _num_in);
  m_C_a = (TFlt *) malloc (sizeof (TFlt) * _num_in);
  memset (m_C_a, 0x0, sizeof (TFlt) * _num_in);
  return 0;
}

int
MNM_Dnode_GRJ_Multiclass::compute_flow ()
{
  throw std::runtime_error ("not implemented");
}

///
/// Multiclass OD
///

/// Origin

MNM_Origin_Multiclass::MNM_Origin_Multiclass (TInt ID, TInt max_interval,
                                              TFlt flow_scalar, TInt frequency)
    : MNM_Origin::MNM_Origin (ID, max_interval, flow_scalar, frequency)
{
  m_demand_car = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
  m_demand_truck = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
  m_car_label_ratio = std::vector<TFlt> ();
  m_truck_label_ratio = std::vector<TFlt> ();
}

MNM_Origin_Multiclass::~MNM_Origin_Multiclass ()
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
  m_car_label_ratio.clear ();
  m_truck_label_ratio.clear ();
}

TInt
MNM_Origin_Multiclass::generate_label (TInt veh_class)
{
  if (veh_class == 0)
    {
      if (m_car_label_ratio.empty ()
          || *std::max_element (m_car_label_ratio.begin (),
                                m_car_label_ratio.end ())
               <= 0)
        {
          return TInt (-1); // no label information
        }
      else
        {
          TFlt _r = MNM_Ults::rand_flt ();
          TInt _label = 0;
          for (TFlt _p : m_car_label_ratio)
            {
              if (_p >= _r)
                {
                  return _label;
                }
              else
                {
                  _r -= _p;
                  _label += 1;
                }
            }
        }
    }
  else if (veh_class == 1)
    {
      if (m_truck_label_ratio.empty ()
          || *std::max_element (m_truck_label_ratio.begin (),
                                m_truck_label_ratio.end ())
               <= 0)
        {
          return TInt (-1); // no label information
        }
      else
        {
          TFlt _r = MNM_Ults::rand_flt ();
          TInt _label = 0;
          for (TFlt _p : m_truck_label_ratio)
            {
              if (_p >= _r)
                {
                  return _label;
                }
              else
                {
                  _r -= _p;
                  _label += 1;
                }
            }
        }
    }
  else
    {
      printf ("MNM_Origin_Multiclass::generate_label, Wrong vehicle class!\n");
      exit (-1);
    }
  return -1;
}

int
MNM_Origin_Multiclass::add_dest_demand_multiclass (
  MNM_Destination_Multiclass *dest, TFlt *demand_car, TFlt *demand_truck)
{
  // split (15-mins demand) to (15 * 1-minute demand)
  TFlt *_demand_car
    = (TFlt *) malloc (sizeof (TFlt) * m_max_assign_interval * 15);
  for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
      _demand_car[i] = TFlt (demand_car[i]);
    }
  m_demand_car.insert ({ dest, _demand_car });

  TFlt *_demand_truck
    = (TFlt *) malloc (sizeof (TFlt) * m_max_assign_interval * 15);
  for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
      _demand_truck[i] = TFlt (demand_truck[i]);
    }
  m_demand_truck.insert ({ dest, _demand_truck });

  return 0;
}

int
MNM_Origin_Multiclass::release (MNM_Veh_Factory *veh_factory,
                                TInt current_interval)
{
  // if ((m_current_assign_interval < m_max_assign_interval) &&
  // (current_interval % m_frequency == 0)){
  //  	TInt _veh_to_release;
  //  	MNM_Veh_Multiclass *_veh;
  //  	MNM_Veh_Factory_Multiclass *_vfactory =
  //  dynamic_cast<MNM_Veh_Factory_Multiclass *>(veh_factory);
  //  	// release all car
  //   for (auto _demand_it = m_demand_car.begin(); _demand_it !=
  //   m_demand_car.end(); _demand_it++) { 	_veh_to_release =
  //   TInt(MNM_Ults::round((_demand_it -> second)[m_current_assign_interval] *
  //   m_flow_scalar));
  //     	for (int i = 0; i < _veh_to_release; ++i) {
  //        _veh = _vfactory -> make_veh_multiclass(current_interval,
  //        MNM_TYPE_ADAPTIVE, TInt(0)); _veh -> set_destination(_demand_it ->
  //        first); _veh -> set_origin(this); m_origin_node ->
  //        m_in_veh_queue.push_back(_veh);
  //     	}
  //   }
  //   // release all truck
  //   for (auto _demand_it = m_demand_truck.begin(); _demand_it !=
  //   m_demand_truck.end(); _demand_it++) { 	_veh_to_release =
  //   TInt(MNM_Ults::round((_demand_it -> second)[m_current_assign_interval] *
  //   m_flow_scalar));
  //     	for (int i = 0; i < _veh_to_release; ++i) {
  //        _veh = _vfactory -> make_veh_multiclass(current_interval,
  //        MNM_TYPE_ADAPTIVE, TInt(1)); _veh -> set_destination(_demand_it ->
  //        first); _veh -> set_origin(this); m_origin_node ->
  //        m_in_veh_queue.push_back(_veh);
  //     	}
  //   }
  //   m_current_assign_interval++;
  // }
  // random_shuffle(m_origin_node -> m_in_veh_queue.begin(), m_origin_node ->
  // m_in_veh_queue.end());
  return 0;
}

int
MNM_Origin_Multiclass::release_one_interval (TInt current_interval,
                                             MNM_Veh_Factory *veh_factory,
                                             TInt assign_interval,
                                             TFlt adaptive_ratio)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  TInt _veh_to_release;
  MNM_Veh_Multiclass *_veh;
  MNM_Veh_Factory_Multiclass *_vfactory
    = dynamic_cast<MNM_Veh_Factory_Multiclass *> (veh_factory);
  // release all car
  for (auto _demand_it = m_demand_car.begin ();
       _demand_it != m_demand_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _veh = _vfactory->make_veh_multiclass (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0));
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multiclass (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (0));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0));
                }
              else
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (0));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
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
      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio == TFlt (0))
            {
              _veh = _vfactory->make_veh_multiclass (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1));
            }
          else if (adaptive_ratio == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multiclass (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio)
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                       m_origin_node->m_in_veh_queue.end ());
  return 0;
}

int
MNM_Origin_Multiclass::release_one_interval_biclass (
  TInt current_interval, MNM_Veh_Factory *veh_factory, TInt assign_interval,
  TFlt adaptive_ratio_car, TFlt adaptive_ratio_truck)
{
  if (assign_interval < 0)
    return 0;
  m_current_assign_interval = assign_interval;
  TInt _veh_to_release;
  MNM_Veh_Multiclass *_veh;
  MNM_Veh_Factory_Multiclass *_vfactory
    = dynamic_cast<MNM_Veh_Factory_Multiclass *> (veh_factory);
  // release all car
  for (auto _demand_it = m_demand_car.begin ();
       _demand_it != m_demand_car.end (); _demand_it++)
    {
      _veh_to_release = TInt (MNM_Ults::round (
        (_demand_it->second)[assign_interval] * m_flow_scalar));
      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio_car == TFlt (0))
            {
              _veh = _vfactory->make_veh_multiclass (current_interval,
                                                     MNM_TYPE_STATIC, TInt (0));
            }
          else if (adaptive_ratio_car == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multiclass (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (0));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio_car)
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (0));
                }
              else
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (0));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
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
      for (int i = 0; i < _veh_to_release; ++i)
        {
          if (adaptive_ratio_truck == TFlt (0))
            {
              _veh = _vfactory->make_veh_multiclass (current_interval,
                                                     MNM_TYPE_STATIC, TInt (1));
            }
          else if (adaptive_ratio_truck == TFlt (1))
            {
              _veh
                = _vfactory->make_veh_multiclass (current_interval,
                                                  MNM_TYPE_ADAPTIVE, TInt (1));
            }
          else
            {
              TFlt _r = MNM_Ults::rand_flt ();
              if (_r <= adaptive_ratio_truck)
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_ADAPTIVE,
                                                         TInt (1));
                }
              else
                {
                  _veh = _vfactory->make_veh_multiclass (current_interval,
                                                         MNM_TYPE_STATIC,
                                                         TInt (1));
                }
            }
          _veh->set_destination (_demand_it->first);
          _veh->set_origin (this);
          // _veh -> m_assign_interval = assign_interval;
          // in case the multiclass modeling has 1-min release interval as the
          // "assign" interval
          _veh->m_assign_interval = int (current_interval / m_frequency);
          _veh->m_label = generate_label (_veh->get_class ());
          m_origin_node->m_in_veh_queue.push_back (_veh);
        }
    }
  std::random_shuffle (m_origin_node->m_in_veh_queue.begin (),
                       m_origin_node->m_in_veh_queue.end ());
  return 0;
}

/// Destination

MNM_Destination_Multiclass::MNM_Destination_Multiclass (TInt ID)
    : MNM_Destination::MNM_Destination (ID)
{
  ;
}

MNM_Destination_Multiclass::~MNM_Destination_Multiclass () { ; }

///
/// Multiclass Vehicle
///

MNM_Veh_Multiclass::MNM_Veh_Multiclass (TInt ID, TInt vehicle_class,
                                        TInt start_time)
    : MNM_Veh::MNM_Veh (ID, start_time)
{
  m_class = vehicle_class; // 0: car, 1: truck
  m_visual_position_on_link
    = 0.5; // default: visualize veh as at the middle point of link
}

MNM_Veh_Multiclass::~MNM_Veh_Multiclass () { ; }

///
/// Multiclass Factory
///

/// Vehicle Factory

MNM_Veh_Factory_Multiclass::MNM_Veh_Factory_Multiclass ()
    : MNM_Veh_Factory::MNM_Veh_Factory ()
{
  m_num_car = TInt (0);
  m_num_truck = TInt (0);
  m_enroute_car = TInt (0);
  m_enroute_truck = TInt (0);
  m_finished_car = TInt (0);
  m_finished_truck = TInt (0);
  m_total_time_car = TFlt (0);
  m_total_time_truck = TFlt (0);
}

MNM_Veh_Factory_Multiclass::~MNM_Veh_Factory_Multiclass () { ; }

MNM_Veh_Multiclass *
MNM_Veh_Factory_Multiclass::make_veh_multiclass (TInt timestamp,
                                                 Vehicle_type veh_type,
                                                 TInt vehicle_cls)
{
  // printf("A vehicle is produce at time %d, ID is %d\n", (int)timestamp,
  // (int)m_num_veh + 1);
  MNM_Veh_Multiclass *_veh
    = new MNM_Veh_Multiclass (m_num_veh + 1, vehicle_cls, timestamp);
  _veh->m_type = veh_type;
  m_veh_map.insert ({ m_num_veh + 1, _veh });

  m_num_veh += 1;
  m_enroute += 1;
  if (vehicle_cls == 0)
    {
      m_num_car += 1;
      m_enroute_car += 1;
    }
  else if (vehicle_cls == 1)
    {
      m_num_truck += 1;
      m_enroute_truck += 1;
    }
  return _veh;
}

int
MNM_Veh_Factory_Multiclass::remove_finished_veh (MNM_Veh *veh, bool del)
{
  MNM_Veh_Multiclass *_veh_multiclass
    = dynamic_cast<MNM_Veh_Multiclass *> (veh);
  IAssert (_veh_multiclass != nullptr);
  IAssert (veh->m_finish_time > veh->m_start_time);
  if (_veh_multiclass->m_class == 0)
    {
      m_finished_car += 1;
      m_enroute_car -= 1;
      m_total_time_car += (veh->m_finish_time - veh->m_start_time);
    }
  else if (_veh_multiclass->m_class == 1)
    {
      m_finished_truck += 1;
      m_enroute_truck -= 1;
      m_total_time_truck += (veh->m_finish_time - veh->m_start_time);
    }
  MNM_Veh_Factory::remove_finished_veh (veh, del);
  IAssert (m_num_car == m_finished_car + m_enroute_car);
  IAssert (m_num_truck == m_finished_truck + m_enroute_truck);
  return 0;
}

/// Node factory

MNM_Node_Factory_Multiclass::MNM_Node_Factory_Multiclass ()
    : MNM_Node_Factory::MNM_Node_Factory ()
{
  ;
}

MNM_Node_Factory_Multiclass::~MNM_Node_Factory_Multiclass () { ; }

MNM_Dnode *
MNM_Node_Factory_Multiclass::make_node_multiclass (
  TInt ID, DNode_type_multiclass node_type, TFlt flow_scalar,
  TFlt veh_convert_factor)
{
  MNM_Dnode *_node;
  switch (node_type)
    {
    case MNM_TYPE_FWJ_MULTICLASS:
      _node
        = new MNM_Dnode_FWJ_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    case MNM_TYPE_ORIGIN_MULTICLASS:
      _node = new MNM_DMOND_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    case MNM_TYPE_DEST_MULTICLASS:
      _node = new MNM_DMDND_Multiclass (ID, flow_scalar, veh_convert_factor);
      break;
    default:
      throw std::runtime_error ("unknown node type");
    }
  m_node_map.insert ({ ID, _node });
  return _node;
}

/// Link factory

MNM_Link_Factory_Multiclass::MNM_Link_Factory_Multiclass ()
    : MNM_Link_Factory::MNM_Link_Factory ()
{
  ;
}

MNM_Link_Factory_Multiclass::~MNM_Link_Factory_Multiclass () { ; }

MNM_Dlink *
MNM_Link_Factory_Multiclass::make_link_multiclass (
  TInt ID, DLink_type_multiclass link_type, TInt number_of_lane, TFlt length,
  TFlt lane_hold_cap_car, TFlt lane_hold_cap_truck, TFlt lane_flow_cap_car,
  TFlt lane_flow_cap_truck, TFlt ffs_car, TFlt ffs_truck, TFlt unit_time,
  TFlt veh_convert_factor, TFlt flow_scalar)
{
  MNM_Dlink *_link;
  switch (link_type)
    {
    case MNM_TYPE_CTM_MULTICLASS:
      _link
        = new MNM_Dlink_Ctm_Multiclass (ID, number_of_lane, length,
                                        lane_hold_cap_car, lane_hold_cap_truck,
                                        lane_flow_cap_car, lane_flow_cap_truck,
                                        ffs_car, ffs_truck, unit_time,
                                        veh_convert_factor, flow_scalar);
      break;
    case MNM_TYPE_LQ_MULTICLASS:
      _link
        = new MNM_Dlink_Lq_Multiclass (ID, number_of_lane, length,
                                       lane_hold_cap_car, lane_hold_cap_truck,
                                       lane_flow_cap_car, lane_flow_cap_truck,
                                       ffs_car, ffs_truck, unit_time,
                                       veh_convert_factor, flow_scalar);
      break;
    case MNM_TYPE_PQ_MULTICLASS:
      _link
        = new MNM_Dlink_Pq_Multiclass (ID, number_of_lane, length,
                                       lane_hold_cap_car, lane_hold_cap_truck,
                                       lane_flow_cap_car, lane_flow_cap_truck,
                                       ffs_car, ffs_truck, unit_time,
                                       veh_convert_factor, flow_scalar);
      break;
    default:
      throw std::runtime_error ("unknown link type");
    }
  m_link_map.insert ({ ID, _link });
  return _link;
}

/// OD factory

MNM_OD_Factory_Multiclass::MNM_OD_Factory_Multiclass ()
    : MNM_OD_Factory::MNM_OD_Factory ()
{
  ;
}

MNM_OD_Factory_Multiclass::~MNM_OD_Factory_Multiclass () { ; }

MNM_Destination_Multiclass *
MNM_OD_Factory_Multiclass::make_destination (TInt ID)
{
  MNM_Destination_Multiclass *_dest;
  _dest = new MNM_Destination_Multiclass (ID);
  m_destination_map.insert ({ ID, _dest });
  return _dest;
}

MNM_Origin_Multiclass *
MNM_OD_Factory_Multiclass::make_origin (TInt ID, TInt max_interval,
                                        TFlt flow_scalar, TInt frequency)
{
  MNM_Origin_Multiclass *_origin;
  _origin
    = new MNM_Origin_Multiclass (ID, max_interval, flow_scalar, frequency);
  m_origin_map.insert ({ ID, _origin });
  return _origin;
}

std::pair<MNM_Origin *, MNM_Destination *>
MNM_OD_Factory_Multiclass::get_random_od_pair ()
{
  MNM_Origin_Multiclass *_origin;
  MNM_Destination_Multiclass *_dest;

  auto _origin_it = m_origin_map.begin ();
  int random_index = rand () % m_origin_map.size ();
  std::advance (_origin_it, random_index);

  _origin = dynamic_cast<MNM_Origin_Multiclass *> (_origin_it->second);
  while (_origin->m_demand_car.empty ())
    {
      _origin_it = m_origin_map.begin ();
      random_index = rand () % m_origin_map.size ();
      std::advance (_origin_it, random_index);
      _origin = dynamic_cast<MNM_Origin_Multiclass *> (_origin_it->second);
    }

  auto _dest_it = _origin->m_demand_car.begin ();
  random_index = rand () % _origin->m_demand_car.size ();
  std::advance (_dest_it, random_index);
  _dest = _dest_it->first;

  return std::pair<MNM_Origin *, MNM_Destination *> (_origin, _dest);
}

///
/// Multiclass IO Functions
///

int
MNM_IO_Multiclass::build_node_factory_multiclass (
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

  MNM_Node_Factory_Multiclass *_node_factory
    = dynamic_cast<MNM_Node_Factory_Multiclass *> (node_factory);
  if (_node_file.is_open ())
    {
      for (int i = 0; i < _num_of_node;)
        {
          std::getline (_node_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 3)
            {
              _node_ID = TInt (std::stoi (_words[0]));
              _type = trim (_words[1]);
              _veh_convert_factor = TFlt (std::stod (_words[2]));
              if (_type == "FWJ")
                {
                  _node_factory->make_node_multiclass (_node_ID,
                                                       MNM_TYPE_FWJ_MULTICLASS,
                                                       _flow_scalar,
                                                       _veh_convert_factor);
                  continue;
                }
              if (_type == "DMOND")
                {
                  _node_factory
                    ->make_node_multiclass (_node_ID,
                                            MNM_TYPE_ORIGIN_MULTICLASS,
                                            _flow_scalar, _veh_convert_factor);
                  continue;
                }
              if (_type == "DMDND")
                {
                  _node_factory->make_node_multiclass (_node_ID,
                                                       MNM_TYPE_DEST_MULTICLASS,
                                                       _flow_scalar,
                                                       _veh_convert_factor);
                  continue;
                }
              throw std::runtime_error ("unknown node type: " + _type);
            }
          else
            {
              throw std::runtime_error ("failed to parse line: " + _line);
            }
        }
      _node_file.close ();
    }
  return 0;
}

int
MNM_IO_Multiclass::build_link_factory_multiclass (
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

  MNM_Link_Factory_Multiclass *_link_factory
    = dynamic_cast<MNM_Link_Factory_Multiclass *> (link_factory);

  if (_link_file.is_open ())
    {
      for (int i = 0; i < _num_of_link;)
        {
          std::getline (_link_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
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

              /* build */
              if (_type == "PQ")
                {
                  _link_factory->make_link_multiclass (_link_ID,
                                                       MNM_TYPE_PQ_MULTICLASS,
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
              if (_type == "LQ")
                {
                  _link_factory->make_link_multiclass (_link_ID,
                                                       MNM_TYPE_LQ_MULTICLASS,
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
              if (_type == "CTM")
                {
                  _link_factory->make_link_multiclass (_link_ID,
                                                       MNM_TYPE_CTM_MULTICLASS,
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
              throw std::runtime_error ("unknown link type: " + _type);
            }
          else
            {
              throw std::runtime_error ("failed to parse line: " + _line);
            }
        }
      _link_file.close ();
    }
  return 0;
}

int
MNM_IO_Multiclass::build_demand_multiclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
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

  /* build */
  TInt _O_ID, _D_ID;
  MNM_Origin_Multiclass *_origin;
  MNM_Destination_Multiclass *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open ())
    {
      // printf("Start build demand profile.\n");
      TFlt *_demand_vector_car
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt *_demand_vector_truck
        = (TFlt *) malloc (sizeof (TFlt) * _max_interval * _num_of_minute);
      TFlt _demand_car;
      TFlt _demand_truck;

      std::getline (_demand_file, _line); // skip the first line
      for (int i = 0; i < _num_OD; ++i)
        {
          std::getline (_demand_file, _line);
          _words = split (trim (_line), ' ');
          if (TInt (_words.size ()) == (_max_interval * 2 + 2))
            {
              _O_ID = TInt (std::stoi (_words[0]));
              _D_ID = TInt (std::stoi (_words[1]));
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
                  // ++k){ 	_demand_vector_car[j * _num_of_minute + k] =
                  // _demand_car; 	_demand_vector_truck[j * _num_of_minute
                  // + k] = _demand_truck;
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
                      printf ("Wrong init_demand_split\n");
                      exit (-1);
                    }
                }
              _origin = dynamic_cast<MNM_Origin_Multiclass *> (
                od_factory->get_origin (_O_ID));
              _dest = dynamic_cast<MNM_Destination_Multiclass *> (
                od_factory->get_destination (_D_ID));
              _origin->add_dest_demand_multiclass (_dest, _demand_vector_car,
                                                   _demand_vector_truck);
            }
          else
            {
              free (_demand_vector_car);
              free (_demand_vector_truck);
              throw std::runtime_error ("failed to build demand");
            }
        }
      free (_demand_vector_car);
      free (_demand_vector_truck);
      _demand_file.close ();
    }
  return 0;
}

int
MNM_IO_Multiclass::read_origin_car_label_ratio (const std::string &file_folder,
                                                MNM_ConfReader *conf_reader,
                                                MNM_OD_Factory *od_factory,
                                                const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  /* build */
  MNM_Origin_Multiclass *_origin;
  TInt _origin_ID;
  std::string _line;
  std::vector<std::string> _words;
  if (_file.is_open ())
    {
      /* read config */
      TInt _num_of_O = conf_reader->get_int ("num_of_O");
      TInt _num_of_vehicle_labels = conf_reader->get_int ("num_of_car_labels");

      if (_num_of_vehicle_labels <= 0)
        {
          return 0;
        }

      // printf("Start build Origin-Destination factory.\n");
      std::getline (_file, _line); // skip the first line
      // printf("Processing Origin node.\n");
      for (int i = 0; i < _num_of_O; ++i)
        {
          std::getline (_file, _line);
          _words = split (_line, ' ');
          if ((int) _words.size () == 1 + _num_of_vehicle_labels)
            { // check
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));

              _origin = dynamic_cast<MNM_Origin_Multiclass *> (
                od_factory->get_origin (_origin_ID));
              for (int j = 0; j < _num_of_vehicle_labels; ++j)
                {
                  _origin->m_car_label_ratio.push_back (
                    TFlt (std::stof (_words[1 + j])));
                }
            }
        }
    }
  else
    {
      printf ("No vehicle registration data\n");
    }
  _file.close ();
  return 0;
}

int
MNM_IO_Multiclass::read_origin_truck_label_ratio (
  const std::string &file_folder, MNM_ConfReader *conf_reader,
  MNM_OD_Factory *od_factory, const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  /* build */
  MNM_Origin_Multiclass *_origin;
  TInt _origin_ID;
  std::string _line;
  std::vector<std::string> _words;
  if (_file.is_open ())
    {
      /* read config */
      TInt _num_of_O = conf_reader->get_int ("num_of_O");
      TInt _num_of_vehicle_labels
        = conf_reader->get_int ("num_of_truck_labels");

      if (_num_of_vehicle_labels <= 0)
        {
          return 0;
        }

      // printf("Start build Origin-Destination factory.\n");
      std::getline (_file, _line); // skip the first line
      // printf("Processing Origin node.\n");
      for (int i = 0; i < _num_of_O; ++i)
        {
          std::getline (_file, _line);
          _words = split (_line, ' ');
          if ((int) _words.size () == 1 + _num_of_vehicle_labels)
            { // check
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));

              _origin = dynamic_cast<MNM_Origin_Multiclass *> (
                od_factory->get_origin (_origin_ID));
              for (int j = 0; j < _num_of_vehicle_labels; ++j)
                {
                  _origin->m_truck_label_ratio.push_back (
                    TFlt (std::stof (_words[1 + j])));
                }
            }
        }
    }
  else
    {
      printf ("No vehicle registration data\n");
    }
  _file.close ();
  return 0;
}

int
MNM_IO_Multiclass::build_link_toll_multiclass (const std::string &file_folder,
                                               MNM_ConfReader *conf_reader,
                                               MNM_Link_Factory *link_factory,
                                               const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;

  if (_file.is_open ())
    {
      TInt _num_of_tolled_link = conf_reader->get_int ("num_of_tolled_link");
      if (_num_of_tolled_link <= 0)
        {
          _file.close ();
          printf ("No tolled links.\n");
          return 0;
        }

      printf ("Start build link toll.\n");
      std::getline (_file, _line); // #link_ID toll_car toll_truck
      for (int i = 0; i < _num_of_tolled_link; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";

          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 3)
            {
              _link_ID = TInt (std::stoi (trim (_words[0])));
              link_factory->get_link (_link_ID)->m_toll
                = TFlt (std::stof (trim (_words[1])));
              dynamic_cast<MNM_Dlink_Multiclass *> (
                link_factory->get_link (_link_ID))
                ->m_toll_car
                = TFlt (std::stof (trim (_words[1])));
              dynamic_cast<MNM_Dlink_Multiclass *> (
                link_factory->get_link (_link_ID))
                ->m_toll_truck
                = TFlt (std::stof (trim (_words[2])));
            }
          else
            {
              printf ("Something wrong in build_link_toll_multiclass!\n");
              exit (-1);
            }
        }
      _file.close ();
      printf ("Finish build link toll.\n");
    }
  else
    {
      printf ("No tolled links.\n");
    }
  return 0;
}

///
/// Multiclass DTA
///

MNM_Dta_Multiclass::MNM_Dta_Multiclass (const std::string &file_folder)
    : MNM_Dta::MNM_Dta (file_folder)
{
  // Re-run the multiclass version of initialize();
  initialize ();

  m_queue_veh_map_car = std::unordered_map<TInt, std::deque<TInt> *> ();
  m_queue_veh_map_truck = std::unordered_map<TInt, std::deque<TInt> *> ();
}

MNM_Dta_Multiclass::~MNM_Dta_Multiclass ()
{
  for (auto _it = m_queue_veh_map_car.begin ();
       _it != m_queue_veh_map_car.end (); _it++)
    {
      _it->second->clear ();
      delete _it->second;
    }
  m_queue_veh_map_car.clear ();

  for (auto _it = m_queue_veh_map_truck.begin ();
       _it != m_queue_veh_map_truck.end (); _it++)
    {
      _it->second->clear ();
      delete _it->second;
    }
  m_queue_veh_map_truck.clear ();
}

int
MNM_Dta_Multiclass::initialize ()
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
  m_veh_factory = new MNM_Veh_Factory_Multiclass ();
  // printf("1\n");
  m_node_factory = new MNM_Node_Factory_Multiclass ();
  // printf("2\n");
  m_link_factory = new MNM_Link_Factory_Multiclass ();
  // printf("3\n");
  m_od_factory = new MNM_OD_Factory_Multiclass ();
  // printf("4\n");
  m_config = new MNM_ConfReader (m_file_folder + "/config.conf", "DTA");
  m_unit_time = m_config->get_int ("unit_time");
  m_flow_scalar = m_config->get_int ("flow_scalar");
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

  return 0;
}

int
MNM_Dta_Multiclass::build_from_files ()
{
  MNM_IO_Multiclass::build_node_factory_multiclass (m_file_folder, m_config,
                                                    m_node_factory);
  MNM_IO_Multiclass::build_link_factory_multiclass (m_file_folder, m_config,
                                                    m_link_factory);
  // MNM_IO_Multiclass::build_od_factory_multiclass(m_file_folder, m_config,
  // m_od_factory, m_node_factory);
  MNM_IO_Multiclass::build_od_factory (m_file_folder, m_config, m_od_factory,
                                       m_node_factory);
  m_graph = MNM_IO_Multiclass::build_graph (m_file_folder, m_config);
  MNM_IO_Multiclass::build_demand_multiclass (m_file_folder, m_config,
                                              m_od_factory);
  MNM_IO_Multiclass::read_origin_car_label_ratio (m_file_folder, m_config,
                                                  m_od_factory);
  MNM_IO_Multiclass::read_origin_truck_label_ratio (m_file_folder, m_config,
                                                    m_od_factory);
  MNM_IO_Multiclass::build_link_toll_multiclass (m_file_folder, m_config,
                                                 m_link_factory);
  // build_workzone();
  m_workzone = nullptr;
  set_statistics ();
  set_gridlock_recorder ();
  set_routing ();
  return 0;
}

int
MNM_Dta_Multiclass::pre_loading ()
{
  MNM_Dnode *_node;
  // printf("MNM: Prepare loading!\n");
  m_statistics->init_record ();
  if (m_gridlock_recorder != nullptr)
    m_gridlock_recorder->init_record ();
  for (auto _node_it = m_node_factory->m_node_map.begin ();
       _node_it != m_node_factory->m_node_map.end (); _node_it++)
    {
      _node = _node_it->second;
      _node->prepare_loading ();
    }

  // https://stackoverflow.com/questions/7443787/using-c-ifstream-extraction-operator-to-read-formatted-data-from-a-file
  std::ifstream _emission_file (m_file_folder + "/MNM_input_emission_linkID");
  int _link_ID;
  std::unordered_map<int, int> _emission_links = {};
  while (_emission_file >> _link_ID)
    {
      _emission_links.insert ({ _link_ID, 0 });
    }
  _emission_file.close ();

  std::deque<TInt> *_rec;
  for (auto _map_it : m_link_factory->m_link_map)
    {
      _rec = new std::deque<TInt> ();
      m_queue_veh_map.insert ({ _map_it.second->m_link_ID, _rec });
      _rec = new std::deque<TInt> ();
      m_queue_veh_map_car.insert ({ _map_it.second->m_link_ID, _rec });
      _rec = new std::deque<TInt> ();
      m_queue_veh_map_truck.insert ({ _map_it.second->m_link_ID, _rec });
      if (_emission_links.find (int (_map_it.second->m_link_ID))
          != _emission_links.end ())
        m_emission->register_link (_map_it.second);
    }

  printf ("Exiting MNM: Prepare loading!\n");
  return 0;
}

int
MNM_Dta_Multiclass::record_queue_vehicles ()
{
  TInt _tot_queue_size = 0, _queue_size = 0, _queue_size_car = 0,
       _queue_size_truck = 0;
  MNM_Dlink_Multiclass *_link;
  for (auto _map_it : m_link_factory->m_link_map)
    {
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (_map_it.second);
      _queue_size = _link->m_finished_array.size ();
      if (_link->m_link_type != MNM_TYPE_PQ_MULTICLASS)
        { // PQ not included
          _tot_queue_size += _queue_size;
        }
      m_queue_veh_map[_link->m_link_ID]->push_back (_queue_size);
      _queue_size_car = 0, _queue_size_truck = 0;
      for (auto *_v : _link->m_finished_array)
        {
          if (_v->get_class () == 0)
            _queue_size_car += 1;
          if (_v->get_class () == 1)
            _queue_size_truck += 1; // not divided by flow_scalar
        }
      m_queue_veh_map_car[_link->m_link_ID]->push_back (_queue_size_car);
      m_queue_veh_map_truck[_link->m_link_ID]->push_back (_queue_size_truck);
    }
  m_queue_veh_num.push_back (_tot_queue_size);
  return 0;
}

///
/// Multiclass DTA gradient utils
///

// All functions/API to python should be coded under this namespace
namespace MNM_DTA_GRADIENT
{
TFlt
get_link_inflow_car (MNM_Dlink_Multiclass *link, TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_car link is null");
    }
  if (link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, get_link_inflow_car link cumulative curve is not installed");
    }
  return link->m_N_in_car->get_result (end_time)
         - link->m_N_in_car->get_result (start_time);
}

TFlt
get_link_inflow_car (MNM_Dlink_Multiclass *link, TInt start_time, TInt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_car link is null");
    }
  if (link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, get_link_inflow_car link cumulative curve is not installed");
    }
  return link->m_N_in_car->get_result (TFlt (end_time))
         - link->m_N_in_car->get_result (TFlt (start_time));
}

TFlt
get_link_inflow_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                       TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_truck link is null");
    }
  if (link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, get_link_inflow_truck link cumulative curve is not installed");
    }
  return link->m_N_in_truck->get_result (end_time)
         - link->m_N_in_truck->get_result (start_time);
}

TFlt
get_link_inflow_truck (MNM_Dlink_Multiclass *link, TInt start_time,
                       TInt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow_truck link is null");
    }
  if (link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, get_link_inflow_truck link cumulative curve is not installed");
    }
  return link->m_N_in_truck->get_result (TFlt (end_time))
         - link->m_N_in_truck->get_result (TFlt (start_time));
}

TFlt
get_average_waiting_time_at_intersection (MNM_Dlink_Multiclass *link)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection link is null");
    }
  if (link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection link car in "
        "cumulative curve is not installed");
    }
  if (link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection link truck in "
        "cumulative curve is not installed");
    }
  TFlt _tot_vehs = 0;
  _tot_vehs = link->m_N_in_car->m_recorder.back ().second
              + link->m_N_in_truck->m_recorder.back ().second;

  return link->m_tot_wait_time_at_intersection / (_tot_vehs + 1e-6); // seconds
}

TFlt
get_average_waiting_time_at_intersection_car (MNM_Dlink_Multiclass *link)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection_car link is null");
    }
  if (link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection_car link car in "
        "cumulative curve is not installed");
    }
  TFlt _tot_vehs = 0;
  _tot_vehs = link->m_N_in_car->m_recorder.back ().second;

  return link->m_tot_wait_time_at_intersection_car
         / (_tot_vehs + 1e-6); // seconds
}

TFlt
get_average_waiting_time_at_intersection_truck (MNM_Dlink_Multiclass *link)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection_truck link is null");
    }
  if (link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, get_average_waiting_time_at_intersection_truck link truck in "
        "cumulative curve is not installed");
    }
  TFlt _tot_vehs = 0;
  _tot_vehs = link->m_N_in_truck->m_recorder.back ().second;

  return link->m_tot_wait_time_at_intersection_truck / (_tot_vehs + 1e-6);
}

TInt
get_is_spillback (MNM_Dlink_Multiclass *link) // 0 - no spillback, 1 - spillback
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_is_spillback link is null");
    }
  if (link->m_spill_back)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

TFlt
get_travel_time_from_FD_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                             TFlt unit_interval)
{
  TFlt _flow = link->m_N_in_car->get_result (start_time)
               - link->m_N_out_car->get_result (start_time);
  if (_flow < 0.)
    _flow = 0.;
  TFlt _tt = link->get_link_tt_from_flow_car (_flow);
  return _tt / unit_interval;
}

TFlt
get_travel_time_from_FD_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                               TFlt unit_interval)
{
  TFlt _flow = link->m_N_in_truck->get_result (start_time)
               - link->m_N_out_truck->get_result (start_time);
  if (_flow < 0.)
    _flow = 0.;
  TFlt _tt = link->get_link_tt_from_flow_truck (_flow);
  return _tt / unit_interval;
}

TFlt
get_travel_time_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                     TFlt unit_interval, TInt end_loading_timestamp)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_car link is null");
    }
  if (link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, get_travel_time_car link cumulative curve is not installed");
    }

  // TFlt fftt = link -> get_link_freeflow_tt_car() / unit_interval;
  TFlt fftt = TFlt (int (
    link->get_link_freeflow_tt_loading_car ())); // actual intervals in loading

  if (link->m_last_valid_time < 0)
    {
      link->m_last_valid_time
        = get_last_valid_time (link->m_N_in_car, link->m_N_out_car,
                               end_loading_timestamp);
    }
  IAssert (link->m_last_valid_time >= 0);

  return get_travel_time_from_cc (start_time, link->m_N_in_car,
                                  link->m_N_out_car, link->m_last_valid_time,
                                  fftt);
}

TFlt
get_travel_time_car_robust (MNM_Dlink_Multiclass *link, TFlt start_time,
                            TFlt end_time, TFlt unit_interval,
                            TInt end_loading_timestamp, TInt num_trials)
{
  IAssert (end_time > start_time);
  num_trials = num_trials > TInt (end_time - start_time)
                 ? TInt (end_time - start_time)
                 : num_trials;
  TFlt _delta = (end_time - start_time) / TFlt (num_trials);
  TFlt _ave_tt = TFlt (0);
  for (int i = 0; i < num_trials (); ++i)
    {
      _ave_tt += get_travel_time_car (link, start_time + TFlt (i) * _delta,
                                      unit_interval, end_loading_timestamp);
    }
  return _ave_tt / TFlt (num_trials);
}

TFlt
get_travel_time_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                       TFlt unit_interval, TInt end_loading_timestamp)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time_truck link is null");
    }
  if (link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, get_travel_time_truck link cumulative curve is not installed");
    }
  // printf("%.2f\n", start_time);

  // TFlt fftt = link -> get_link_freeflow_tt_truck() / unit_interval;
  TFlt fftt = TFlt (int (
    link
      ->get_link_freeflow_tt_loading_truck ())); // actual intervals in loading

  if (link->m_last_valid_time_truck < 0)
    {
      link->m_last_valid_time_truck
        = get_last_valid_time (link->m_N_in_truck, link->m_N_out_truck,
                               end_loading_timestamp);
    }
  IAssert (link->m_last_valid_time_truck >= 0);

  return get_travel_time_from_cc (start_time, link->m_N_in_truck,
                                  link->m_N_out_truck,
                                  link->m_last_valid_time_truck, fftt);
}

TFlt
get_travel_time_truck_robust (MNM_Dlink_Multiclass *link, TFlt start_time,
                              TFlt end_time, TFlt unit_interval,
                              TInt end_loading_timestamp, TInt num_trials)
{
  IAssert (end_time > start_time);
  num_trials = num_trials > TInt (end_time - start_time)
                 ? TInt (end_time - start_time)
                 : num_trials;
  TFlt _delta = (end_time - start_time) / TFlt (num_trials);
  TFlt _ave_tt = TFlt (0);
  for (int i = 0; i < num_trials (); ++i)
    {
      _ave_tt += get_travel_time_truck (link, start_time + TFlt (i) * _delta,
                                        unit_interval, end_loading_timestamp);
    }
  return _ave_tt / TFlt (num_trials);
}

TFlt
get_path_travel_time_car (MNM_Path *path, TFlt start_time,
                          MNM_Link_Factory *link_factory, TFlt unit_interval,
                          TInt end_loading_timestamp)
{
  if (path == nullptr)
    {
      throw std::runtime_error ("Error, get_path_travel_time_car path is null");
    }
  if (link_factory == nullptr)
    {
      throw std::runtime_error (
        "Error, get_path_travel_time_car link_factory is null");
    }
  int _end_time = int (round (start_time));
  MNM_Dlink_Multiclass *_link;
  for (auto _link_ID : path->m_link_vec)
    {
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (
        link_factory->get_link (_link_ID));
      // assume each link's travel time >= unit interval
      // use _end_time + 1 as start_time in cc to compute the link travel time
      // for vehicles arriving at the beginning of interval _end_time
      _end_time = _end_time
                  + MNM_Ults::round_up_time (
                    get_travel_time_car (_link, _end_time + 1, unit_interval,
                                         end_loading_timestamp));
    }
  return TFlt (_end_time - start_time); // # of unit intervals
}

TFlt
get_path_travel_time_car (MNM_Path *path, TFlt start_time,
                          std::unordered_map<TInt, TFlt *> &link_tt_map_car,
                          TInt end_loading_timestamp)
{
  return get_path_travel_time (path, start_time, link_tt_map_car,
                               end_loading_timestamp);
}

TFlt
get_path_travel_time_truck (MNM_Path *path, TFlt start_time,
                            MNM_Link_Factory *link_factory, TFlt unit_interval,
                            TInt end_loading_timestamp)
{
  if (path == nullptr)
    {
      throw std::runtime_error (
        "Error, get_path_travel_time_truck path is null");
    }
  if (link_factory == nullptr)
    {
      throw std::runtime_error (
        "Error, get_path_travel_time_truck link_factory is null");
    }
  int _end_time = int (round (start_time));
  MNM_Dlink_Multiclass *_link;
  for (auto _link_ID : path->m_link_vec)
    {
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (
        link_factory->get_link (_link_ID));
      // assume each link's travel time >= unit interval
      // use _end_time + 1 as start_time in cc to compute the link travel time
      // for vehicles arriving at the beginning of interval _end_time
      _end_time = _end_time
                  + MNM_Ults::round_up_time (
                    get_travel_time_truck (_link, _end_time + 1, unit_interval,
                                           end_loading_timestamp));
    }
  return TFlt (_end_time - start_time); // # of unit intervals
}

TFlt
get_path_travel_time_truck (MNM_Path *path, TFlt start_time,
                            std::unordered_map<TInt, TFlt *> &link_tt_map_truck,
                            TInt end_loading_timestamp)
{
  return get_path_travel_time (path, start_time, link_tt_map_truck,
                               end_loading_timestamp);
}

int
add_dar_records_car (std::vector<dar_record *> &record,
                     MNM_Dlink_Multiclass *link, std::set<MNM_Path *> pathset,
                     TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_car link is null");
    }
  if (link->m_N_in_tree_car == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_car link cumulative "
                                "curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_car->m_record)
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
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int if release_one_interval_biclass already set the
                  // correct assign interval for vehicle, then this is 15 min
                  // intervals
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
add_dar_records_truck (std::vector<dar_record *> &record,
                       MNM_Dlink_Multiclass *link, std::set<MNM_Path *> pathset,
                       TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_truck link is null");
    }
  if (link->m_N_in_tree_truck == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_truck link cumulative "
                                "curve tree is not installed");
    }
  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_truck->m_record)
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
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int if release_one_interval_biclass already set the
                  // correct assign interval for vehicle, then this is 15 min
                  // intervals
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
add_dar_records_car (std::vector<dar_record *> &record,
                     MNM_Dlink_Multiclass *link, std::set<TInt> pathID_set,
                     TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_car link is null");
    }
  if (link->m_N_in_tree_car == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_car link cumulative "
                                "curve tree is not installed");
    }

  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_car->m_record)
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
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int if release_one_interval_biclass already set the
                  // correct assign interval for vehicle, then this is 15 min
                  // intervals
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
add_dar_records_truck (std::vector<dar_record *> &record,
                       MNM_Dlink_Multiclass *link, std::set<TInt> pathID_set,
                       TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_truck link is null");
    }
  if (link->m_N_in_tree_truck == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_truck link cumulative "
                                "curve tree is not installed");
    }

  MNM_Path *_path;
  for (auto path_it : link->m_N_in_tree_truck->m_record)
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
                  // the count of 1 min intervals, the vehicles record this
                  // assign_int if release_one_interval_biclass already set the
                  // correct assign interval for vehicle, then this is 15 min
                  // intervals
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
add_dar_records_eigen_car (std::vector<Eigen::Triplet<double>> &record,
                           MNM_Dlink_Multiclass *link,
                           std::set<MNM_Path *> pathset, TFlt start_time,
                           TFlt end_time, int link_ind, int interval_ind,
                           int num_of_minute, int num_e_link, int num_e_path,
                           const double *f_ptr)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_eigen_car link is null");
    }
  if (link->m_N_in_tree_car == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen_car link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  int _x, _y;
  for (const auto &path_it : link->m_N_in_tree_car->m_record)
    {
      _path = path_it.first;
      // !!! assume all paths recorded in veh -> m_path are in pathset, even for
      // adaptive users, they use a nominal path in pathset if
      // (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second)
        {
          TFlt tmp_flow = depart_it.second->get_result (end_time)
                          - depart_it.second->get_result (start_time);
          if (tmp_flow > DBL_EPSILON)
            {
              _x = link_ind
                   + num_e_link * interval_ind; // # of links * # of intervals
              // !!! this assumes that m_path_ID starts from zero and is sorted,
              // which is usually done in python _y = _path -> m_path_ID +
              // num_e_path * int(depart_it.first / num_of_minute); // # of
              // paths * # of intervals if release_one_interval_biclass already
              // set the correct assign interval for vehicle
              _y
                = _path->m_path_ID
                  + num_e_path * depart_it.first; // # of paths * # of intervals
              // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
              // path_ID(), new_record -> assign_int(),
              //     new_record -> link_ID(), (float)new_record ->
              //     link_start_int(), (float) new_record -> flow());

              // https://eigen.tuxfamily.org/dox/classEigen_1_1Triplet.html
              // https://eigen.tuxfamily.org/dox/SparseUtil_8h_source.html
              // (row index, col index, value)
              // 0 in f is set to small value in python
              record.push_back (
                Eigen::Triplet<double> ((double) _x, (double) _y,
                                        tmp_flow () / f_ptr[_y]));
            }
        }
      // }
    }
  return 0;
}

int
add_dar_records_eigen_car (Eigen::SparseMatrix<double, Eigen::RowMajor> &mat,
                           MNM_Dlink_Multiclass *link,
                           std::set<MNM_Path *> pathset, TFlt start_time,
                           TFlt end_time, int link_ind, int interval_ind,
                           int num_of_minute, int num_e_link, int num_e_path,
                           const double *f_ptr)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_eigen_car link is null");
    }
  if (link->m_N_in_tree_car == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen_car link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  int _x, _y;
  for (const auto &path_it : link->m_N_in_tree_car->m_record)
    {
      _path = path_it.first;
      // !!! assume all paths recorded in veh -> m_path are in pathset, even for
      // adaptive users, they use a nominal path in pathset if
      // (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second)
        {
          TFlt tmp_flow = depart_it.second->get_result (end_time)
                          - depart_it.second->get_result (start_time);
          if (tmp_flow > DBL_EPSILON)
            {
              _x = link_ind
                   + num_e_link * interval_ind; // # of links * # of intervals
              // !!! this assumes that m_path_ID starts from zero and is sorted,
              // which is usually done in python _y = _path -> m_path_ID +
              // num_e_path * int(depart_it.first / num_of_minute); // # of
              // paths * # of intervals if release_one_interval_biclass already
              // set the correct assign interval for vehicle
              _y
                = _path->m_path_ID
                  + num_e_path * depart_it.first; // # of paths * # of intervals
              // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
              // path_ID(), new_record -> assign_int(),
              //     new_record -> link_ID(), (float)new_record ->
              //     link_start_int(), (float) new_record -> flow());

              // https://stackoverflow.com/questions/18154027/sparsematrix-construction-in-eigen
              // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3
              // insert() does not allow duplicates
              // coeffRef(i,j) allows duplicates, but slower
              // 0 in f is set to small value in python
              mat.insert (_x, _y) = tmp_flow () / f_ptr[_y];
            }
        }
      // }
    }
  return 0;
}

int
add_dar_records_eigen_truck (std::vector<Eigen::Triplet<double>> &record,
                             MNM_Dlink_Multiclass *link,
                             std::set<MNM_Path *> pathset, TFlt start_time,
                             TFlt end_time, int link_ind, int interval_ind,
                             int num_of_minute, int num_e_link, int num_e_path,
                             const double *f_ptr)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_eigen_truck link is null");
    }
  if (link->m_N_in_tree_truck == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen_truck link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  int _x, _y;
  for (const auto &path_it : link->m_N_in_tree_truck->m_record)
    {
      _path = path_it.first;
      // !!! assume all paths recorded in veh -> m_path are in pathset, even for
      // adaptive users, they use a nominal path in pathset if
      // (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second)
        {
          TFlt tmp_flow = depart_it.second->get_result (end_time)
                          - depart_it.second->get_result (start_time);
          if (tmp_flow > DBL_EPSILON)
            {
              _x = link_ind
                   + num_e_link * interval_ind; // # of links * # of intervals
              // !!! this assumes that m_path_ID starts from zero and is sorted,
              // which is usually done in python _y = _path -> m_path_ID +
              // num_e_path * int(depart_it.first / num_of_minute); // # of
              // paths * # of intervals if release_one_interval_biclass already
              // set the correct assign interval for vehicle
              _y
                = _path->m_path_ID
                  + num_e_path * depart_it.first; // # of paths * # of intervals
              // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
              // path_ID(), new_record -> assign_int(),
              //     new_record -> link_ID(), (float)new_record ->
              //     link_start_int(), (float) new_record -> flow());

              // https://eigen.tuxfamily.org/dox/classEigen_1_1Triplet.html
              // https://eigen.tuxfamily.org/dox/SparseUtil_8h_source.html
              // (row index, col index, value)
              // 0 in f is set to small value in python
              record.push_back (
                Eigen::Triplet<double> ((double) _x, (double) _y,
                                        tmp_flow () / f_ptr[_y]));
            }
        }
      // }
    }
  return 0;
}

int
add_dar_records_eigen_truck (Eigen::SparseMatrix<double, Eigen::RowMajor> &mat,
                             MNM_Dlink_Multiclass *link,
                             std::set<MNM_Path *> pathset, TFlt start_time,
                             TFlt end_time, int link_ind, int interval_ind,
                             int num_of_minute, int num_e_link, int num_e_path,
                             const double *f_ptr)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records_eigen_truck link is null");
    }
  if (link->m_N_in_tree_truck == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen_truck link "
                                "cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  int _x, _y;
  for (const auto &path_it : link->m_N_in_tree_truck->m_record)
    {
      _path = path_it.first;
      // !!! assume all paths recorded in veh -> m_path are in pathset, even for
      // adaptive users, they use a nominal path in pathset if
      // (pathset.find(_path) != pathset.end()) {
      for (auto depart_it : path_it.second)
        {
          TFlt tmp_flow = depart_it.second->get_result (end_time)
                          - depart_it.second->get_result (start_time);
          if (tmp_flow > DBL_EPSILON)
            {
              _x = link_ind
                   + num_e_link * interval_ind; // # of links * # of intervals
              // !!! this assumes that m_path_ID starts from zero and is sorted,
              // which is usually done in python _y = _path -> m_path_ID +
              // num_e_path * int(depart_it.first / num_of_minute); // # of
              // paths * # of intervals if release_one_interval_biclass already
              // set the correct assign interval for vehicle
              _y
                = _path->m_path_ID
                  + num_e_path * depart_it.first; // # of paths * # of intervals
              // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
              // path_ID(), new_record -> assign_int(),
              //     new_record -> link_ID(), (float)new_record ->
              //     link_start_int(), (float) new_record -> flow());

              // https://stackoverflow.com/questions/18154027/sparsematrix-construction-in-eigen
              // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3
              // insert() does not allow duplicates
              // coeffRef(i,j) allows duplicates, but slower
              // 0 in f is set to small value in python
              mat.insert (_x, _y) = tmp_flow () / f_ptr[_y];
            }
        }
      // }
    }
  return 0;
}

TFlt
get_departure_cc_slope_car (MNM_Dlink_Multiclass *link, TFlt start_time,
                            TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope_car link is null");
    }
  if (link->m_N_out_car == nullptr)
    {
      throw std::runtime_error ("Error, get_departure_cc_slope_car link "
                                "cumulative curve is not installed");
    }
  if (start_time > link->m_N_out_car->m_recorder.back ().first)
    {
      return 0;
    }
  int _delta = int (end_time) - int (start_time);
  IAssert (_delta > 0);
  TFlt _cc1, _cc2, _slope = 0.;
  for (int i = 0; i < _delta; i++)
    {
      _cc1 = link->m_N_out_car->get_result (TFlt (start_time + i));
      _cc2 = link->m_N_out_car->get_result (TFlt (start_time + i + 1));
      _slope += (_cc2 - _cc1);
    }
  // if (_slope <= 0) {
  // 	std::cout << "car in" << "\n";
  // 	std::cout << link -> m_N_in_car -> to_string() << "\n";
  // 	std::cout << "car out" << "\n";
  // 	std::cout << link -> m_N_out_car -> to_string() << "\n";
  // 	printf("debug");
  // }
  return _slope / _delta; // flow per unit interval
}

TFlt
get_departure_cc_slope_truck (MNM_Dlink_Multiclass *link, TFlt start_time,
                              TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope_truck link is null");
    }
  if (link->m_N_out_truck == nullptr)
    {
      throw std::runtime_error ("Error, get_departure_cc_slope_truck link "
                                "cumulative curve is not installed");
    }
  if (start_time > link->m_N_out_truck->m_recorder.back ().first)
    {
      return 0;
    }
  int _delta = int (end_time) - int (start_time);
  IAssert (_delta > 0);
  TFlt _cc1, _cc2, _slope = 0.;
  for (int i = 0; i < _delta; i++)
    {
      _cc1 = link->m_N_out_truck->get_result (TFlt (start_time + i));
      _cc2 = link->m_N_out_truck->get_result (TFlt (start_time + i + 1));
      _slope += (_cc2 - _cc1);
    }
  return _slope / _delta; // flow per unit interval
}

int
add_ltg_records_veh (std::vector<ltg_record *> &record,
                     MNM_Dlink_Multiclass *link, MNM_Path *path,
                     int depart_time, int start_time, TFlt gradient)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_ltg_records_veh link is null");
    }
  if (path == nullptr)
    {
      throw std::runtime_error ("Error, add_ltg_records_veh path is null");
    }
  if (!path->is_link_in (link->m_link_ID))
    {
      throw std::runtime_error (
        "Error, add_ltg_records_veh link is not in path");
    }

  auto new_record = new ltg_record ();
  new_record->path_ID = path->m_path_ID;
  // the count of 1 min intervals in terms of 5s intervals, the vehicles record
  // this assign_int
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

int
add_ltg_records_eigen_veh (std::vector<Eigen::Triplet<double>> &record,
                           MNM_Path *path, int depart_time, int start_time,
                           int link_ind, int assign_interval, int num_e_link,
                           int num_e_path, TFlt gradient)
{
  int _x, _y;
  _x = link_ind
       + num_e_link
           * int (start_time / assign_interval); // # of links * # of intervals
  // !!! this assumes that m_path_ID starts from zero and is sorted, which is
  // usually done in python
  _y = path->m_path_ID
       + num_e_path
           * int (depart_time / assign_interval); // # of paths * # of intervals
  record.push_back (
    Eigen::Triplet<double> ((double) _x, (double) _y, gradient));
  return 0;
}

} // end namespace MNM_DTA_GRADIENT

namespace MNM
{
int
print_vehicle_statistics (MNM_Veh_Factory_Multiclass *veh_factory)
{
  printf (
    "############################################### Vehicle Statistics ###############################################\n \
	Released Vehicle total %d, Enroute Vehicle Total %d, Finished Vehicle Total %d,\n \
	Total Travel Time: %.2f intervals,\n \
	Released Car Driving %d, Enroute Car Driving %d, Finished Car Driving %d,\n \
	Released Truck %d, Enroute Truck %d, Finished Truck %d,\n \
	Total Travel Time Car: %.2f intervals, Total Travel Time Truck: %.2f intervals\n \
	############################################### Vehicle Statistics ###############################################\n",
    veh_factory->m_num_veh (), veh_factory->m_enroute (),
    veh_factory->m_finished (), veh_factory->m_total_time (),
    veh_factory->m_num_car (), veh_factory->m_enroute_car (),
    veh_factory->m_finished_car (), veh_factory->m_num_truck (),
    veh_factory->m_enroute_truck (), veh_factory->m_finished_truck (),
    veh_factory->m_total_time_car (), veh_factory->m_total_time_truck ());
  return 0;
}

Path_Table *
build_pathset_multiclass (PNEGraph &graph, MNM_OD_Factory *od_factory,
                          MNM_Link_Factory *link_factory, TFlt min_path_length,
                          size_t MaxIter, TFlt vot, TFlt Mid_Scale,
                          TFlt Heavy_Scale, TInt buffer_length)
{
  // printf("11\n");
  // MaxIter: maximum iteration to find alternative shortest path, when MaxIter
  // = 0, just shortest path Mid_Scale and Heavy_Scale are different penalties
  // to the travel cost of links in existing paths
  IAssert (vot > 0);
  /* initialize data structure */
  TInt _dest_node_ID, _origin_node_ID;
  Path_Table *_path_table = new Path_Table ();
  for (auto _o_it = od_factory->m_origin_map.begin ();
       _o_it != od_factory->m_origin_map.end (); _o_it++)
    {
      _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
      std::unordered_map<TInt, MNM_Pathset *> *_new_map
        = new std::unordered_map<TInt, MNM_Pathset *> ();
      _path_table->insert (
        std::pair<TInt,
                  std::unordered_map<TInt, MNM_Pathset *> *> (_origin_node_ID,
                                                              _new_map));
      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          // assume build_demand is called before this function
          if (dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                ->m_demand_car.find (
                  dynamic_cast<MNM_Destination_Multiclass *> (_d_it->second))
              != dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                   ->m_demand_car.end ())
            {
              _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
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
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _free_cost_map.insert (
        std::pair<TInt, TFlt> (_link_it->first,
                               vot * _link_it->second->get_link_tt ()
                                 + _link_it->second->m_toll));
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
          if (dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                ->m_demand_car.find (
                  dynamic_cast<MNM_Destination_Multiclass *> (_d_it->second))
              == dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                   ->m_demand_car.end ())
            {
              continue;
            }
          _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
          _path = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                     _free_shortest_path_tree, graph);
          if (_path != nullptr)
            {
              if (_path->get_path_length (link_factory) > min_path_length)
                {
                  if (buffer_length > 0)
                    {
                      _path->allocate_buffer (buffer_length);
                    }
                  _path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                    ->second->m_path_vec.push_back (_path);
                }
            }
          else
            {
              printf ("No driving path found to connect origin node ID %d and "
                      "destination node ID %d\n",
                      _origin_node_ID (), _dest_node_ID ());
              exit (-1);
            }
        }
    }
  // printf("22\n");
  _mid_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());
  _heavy_cost_map.insert (_free_cost_map.begin (), _free_cost_map.end ());

  MNM_Dlink *_link;
  MNM_Path *_path_mid, *_path_heavy;
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
                      if (Mid_Scale > 1)
                        {
                          _mid_cost_map.find (_link_ID)->second
                            = vot * _link->get_link_tt () * Mid_Scale
                              + _link->m_toll;
                        }
                      if (Heavy_Scale > 1)
                        {
                          _heavy_cost_map.find (_link_ID)->second
                            = vot * _link->get_link_tt () * Heavy_Scale
                              + _link->m_toll;
                        }
                    }
                }
            }
        }

      for (auto _d_it = od_factory->m_destination_map.begin ();
           _d_it != od_factory->m_destination_map.end (); _d_it++)
        {
          _dest_node_ID = _d_it->second->m_dest_node->m_node_ID;
          if (Mid_Scale > 1)
            {
              MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                                  _mid_cost_map,
                                                  _mid_shortest_path_tree);
            }
          if (Heavy_Scale > 1)
            {
              MNM_Shortest_Path::all_to_one_FIFO (_dest_node_ID, graph,
                                                  _heavy_cost_map,
                                                  _heavy_shortest_path_tree);
            }
          for (auto _o_it = od_factory->m_origin_map.begin ();
               _o_it != od_factory->m_origin_map.end (); _o_it++)
            {
              if (dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                    ->m_demand_car.find (
                      dynamic_cast<MNM_Destination_Multiclass *> (
                        _d_it->second))
                  == dynamic_cast<MNM_Origin_Multiclass *> (_o_it->second)
                       ->m_demand_car.end ())
                {
                  continue;
                }
              _origin_node_ID = _o_it->second->m_origin_node->m_node_ID;
              _path_mid = nullptr;
              _path_heavy = nullptr;
              if (Mid_Scale > 1)
                {
                  _path_mid
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _mid_shortest_path_tree, graph);
                }
              if (Heavy_Scale > 1)
                {
                  _path_heavy
                    = MNM::extract_path (_origin_node_ID, _dest_node_ID,
                                         _heavy_shortest_path_tree, graph);
                }
              if (_path_mid != nullptr)
                {
                  if (!_path_table->find (_origin_node_ID)
                         ->second->find (_dest_node_ID)
                         ->second->is_in (_path_mid))
                    {
                      if (_path_mid->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_mid->allocate_buffer (buffer_length);
                            }
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
                      if (_path_heavy->get_path_length (link_factory)
                          > min_path_length)
                        {
                          if (buffer_length > 0)
                            {
                              _path_heavy->allocate_buffer (buffer_length);
                            }
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
      _CurIter += 1;
    }

  _mid_shortest_path_tree.clear ();
  _mid_cost_map.clear ();
  _heavy_shortest_path_tree.clear ();
  _heavy_cost_map.clear ();

  _free_cost_map.clear ();
  _free_shortest_path_tree.clear ();

  return _path_table;
}

} // end namespace MNM

///
/// Multiclass emissions
///

MNM_Cumulative_Emission_Multiclass::MNM_Cumulative_Emission_Multiclass (
  TFlt unit_time, TInt freq, TInt ev_label_car, TInt ev_label_truck)
    : MNM_Cumulative_Emission::MNM_Cumulative_Emission (unit_time, freq,
                                                        ev_label_car)
{
  m_fuel_truck = TFlt (0);
  m_CO2_truck = TFlt (0);
  m_HC_truck = TFlt (0);
  m_CO_truck = TFlt (0);
  m_NOX_truck = TFlt (0);
  m_VMT_truck = TFlt (0);
  m_VMT_ev_truck = TFlt (0);

  m_VHT_car = TFlt (0);
  m_VHT_truck = TFlt (0);

  m_ev_label_truck = ev_label_truck;

  m_car_set = std::unordered_set<MNM_Veh *> ();
  m_truck_set = std::unordered_set<MNM_Veh *> ();
}

MNM_Cumulative_Emission_Multiclass::~MNM_Cumulative_Emission_Multiclass () { ; }

// All convert_factors from MOVES
// Reference: MOVES default database - class 2b trucks with 4 tires
TFlt
MNM_Cumulative_Emission_Multiclass::calculate_fuel_rate_truck (TFlt v)
{
  TFlt _convert_factor = 1.0;
  if (v < 25)
    {
      _convert_factor = 1.53;
    }
  else if (v < 55)
    {
      _convert_factor = 1.50;
    }
  else
    {
      _convert_factor = 1.55;
    }
  TFlt _fuel_rate_car = calculate_fuel_rate (v);
  TFlt _fuel_rate_truck = _fuel_rate_car * _convert_factor;
  return _fuel_rate_truck;
}

TFlt
MNM_Cumulative_Emission_Multiclass::calculate_CO2_rate_truck (TFlt v)
{
  TFlt _convert_factor = 1.0;
  if (v < 25)
    {
      _convert_factor = 1.53;
    }
  else if (v < 55)
    {
      _convert_factor = 1.50;
    }
  else
    {
      _convert_factor = 1.55;
    }
  TFlt _CO2_rate_car = calculate_CO2_rate (v);
  TFlt _CO2_rate_truck = _CO2_rate_car * _convert_factor;
  return _CO2_rate_truck;
}

TFlt
MNM_Cumulative_Emission_Multiclass::calculate_HC_rate_truck (TFlt v)
{
  TFlt _convert_factor = 1.0;
  if (v < 25)
    {
      _convert_factor = 1.87;
    }
  else if (v < 55)
    {
      _convert_factor = 2.41;
    }
  else
    {
      _convert_factor = 2.01;
    }
  TFlt _HC_rate_car = calculate_HC_rate (v);
  TFlt _HC_rate_truck = _HC_rate_car * _convert_factor;
  return _HC_rate_truck;
}

TFlt
MNM_Cumulative_Emission_Multiclass::calculate_CO_rate_truck (TFlt v)
{
  TFlt _convert_factor = 1.0;
  if (v < 25)
    {
      _convert_factor = 3.97;
    }
  else if (v < 55)
    {
      _convert_factor = 2.67;
    }
  else
    {
      _convert_factor = 5.01;
    }
  TFlt _CO_rate_car = calculate_CO_rate (v);
  TFlt _CO_rate_truck = _CO_rate_car * _convert_factor;
  return _CO_rate_truck;
}

TFlt
MNM_Cumulative_Emission_Multiclass::calculate_NOX_rate_truck (TFlt v)
{
  TFlt _convert_factor = 1.0;
  if (v < 25)
    {
      _convert_factor = 7.32;
    }
  else if (v < 55)
    {
      _convert_factor = 6.03;
    }
  else
    {
      _convert_factor = 5.75;
    }
  TFlt _NOX_rate_car = calculate_NOX_rate (v);
  TFlt _NOX_rate_truck = _NOX_rate_car * _convert_factor;
  return _NOX_rate_truck;
}

int
MNM_Cumulative_Emission_Multiclass::update (MNM_Veh_Factory *veh_factory)
{
  // assume car truck the same speed on the same link when computing the
  // emissions possible to change to more accurate speeds for cars and trucks
  TFlt _v, _nonev_ct_car, _ev_ct_car, _nonev_ct_truck, _ev_ct_truck;
  TFlt _v_converted;
  std::vector<TFlt> _veh_ct;
  for (MNM_Dlink *link : m_link_vector)
    {
      MNM_Dlink_Multiclass *_mlink
        = dynamic_cast<MNM_Dlink_Multiclass *> (link);
      IAssert (_mlink != nullptr);
      _v = _mlink->m_length / _mlink->get_link_tt (); // m/s
      _v_converted = _v * TFlt (3600) / TFlt (1600);  // mile / hour
      _v_converted = MNM_Ults::max (_v_converted, TFlt (5));
      _v_converted = MNM_Ults::min (_v_converted, TFlt (65));

      _veh_ct = _mlink->get_link_flow_emission_car (m_ev_label);
      IAssert (_veh_ct.size () == 2);
      _nonev_ct_car = _veh_ct[0];
      _ev_ct_car = _veh_ct[1];

      _veh_ct = _mlink->get_link_flow_emission_truck (m_ev_label_truck);
      IAssert (_veh_ct.size () == 2);
      _nonev_ct_truck = _veh_ct[0];
      _ev_ct_truck = _veh_ct[1];

      // cars
      m_fuel += calculate_fuel_rate (_v_converted)
                * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_car;
      m_CO2 += calculate_CO2_rate (_v_converted)
               * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_car;
      m_HC += calculate_HC_rate (_v_converted)
              * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_car;
      m_CO += calculate_CO_rate (_v_converted)
              * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_car;
      m_NOX += calculate_NOX_rate (_v_converted)
               * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_car;
      m_VMT += (_v * m_unit_time / TFlt (1600)) * (_nonev_ct_car + _ev_ct_car);
      m_VMT_ev += (_v * m_unit_time / TFlt (1600)) * _ev_ct_car;

      // trucks
      m_fuel_truck += calculate_fuel_rate_truck (_v_converted)
                      * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_truck;
      m_CO2_truck += calculate_CO2_rate_truck (_v_converted)
                     * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_truck;
      m_HC_truck += calculate_HC_rate_truck (_v_converted)
                    * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_truck;
      m_CO_truck += calculate_CO_rate_truck (_v_converted)
                    * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_truck;
      m_NOX_truck += calculate_NOX_rate_truck (_v_converted)
                     * (_v * m_unit_time / TFlt (1600)) * _nonev_ct_truck;
      m_VMT_truck
        += (_v * m_unit_time / TFlt (1600)) * (_nonev_ct_truck + _ev_ct_truck);
      m_VMT_ev_truck += (_v * m_unit_time / TFlt (1600)) * _ev_ct_truck;

      // VHT (hours)
      m_VHT_car += m_unit_time * (_nonev_ct_car + _ev_ct_car) / 3600;
      m_VHT_truck += m_unit_time * (_nonev_ct_truck + _ev_ct_truck) / 3600;
    }
  // trips
  MNM_Veh *_veh;
  MNM_Veh_Multiclass *_veh_multiclass;
  for (auto pair_it : veh_factory->m_veh_map)
    {
      _veh = pair_it.second;
      _veh_multiclass = dynamic_cast<MNM_Veh_Multiclass *> (_veh);
      if (m_link_set.find (_veh_multiclass->m_current_link)
          != m_link_set.end ())
        {
          if (_veh_multiclass->m_class == 0)
            {
              m_car_set.insert (_veh_multiclass);
            }
          if (_veh_multiclass->m_class == 1)
            {
              m_truck_set.insert (_veh_multiclass);
            }
        }
    }

  return 0;
}

std::string
MNM_Cumulative_Emission_Multiclass::output ()
{
  std::string _s = "";

  _s += "The emission stats for cars are: \n";
  _s += "fuel: " + std::to_string (m_fuel ()) + " gallons, ";
  _s += "CO2: " + std::to_string (m_CO2 ()) + " g, ";
  _s += "HC: " + std::to_string (m_HC ()) + " g, ";
  _s += "CO: " + std::to_string (m_CO ()) + " g, ";
  _s += "NOX: " + std::to_string (m_NOX ()) + " g, ";
  _s += "Total Car VMT: " + std::to_string (m_VMT ()) + " miles, ";
  _s += "EV Car VMT: " + std::to_string (m_VMT_ev ()) + " miles, ";
  _s += "VHT: " + std::to_string (m_VHT_car ()) + " hours, ";
  _s += "number of trips: " + std::to_string (int (m_car_set.size ()))
        + " trips\n";

  _s += "The emission stats for trucks are: \n";
  _s += "fuel: " + std::to_string (m_fuel_truck ()) + " gallons, ";
  _s += "CO2: " + std::to_string (m_CO2_truck ()) + " g, ";
  _s += "HC: " + std::to_string (m_HC_truck ()) + " g, ";
  _s += "CO: " + std::to_string (m_CO_truck ()) + " g, ";
  _s += "NOX: " + std::to_string (m_NOX_truck ()) + " g, ";
  _s += "Total Truck VMT: " + std::to_string (m_VMT_truck ()) + " miles, ";
  _s += "EV Truck VMT: " + std::to_string (m_VMT_ev_truck ()) + " miles, ";
  _s += "VHT: " + std::to_string (m_VHT_truck ()) + " hours, ";
  _s += "number of trips: " + std::to_string (int (m_truck_set.size ()))
        + " trips\n";

  printf ("The emission stats for cars are: ");
  printf ("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, "
          "Total VMT: %lf miles, EV VMT: %lf miles, VHT: %lf hours, %d trips\n",
          m_fuel (), m_CO2 (), m_HC (), m_CO (), m_NOX (), m_VMT (),
          m_VMT_ev (), m_VHT_car (), int (m_car_set.size ()));

  printf ("The emission stats for trucks are: ");
  printf ("fuel: %lf gallons, CO2: %lf g, HC: %lf g, CO: %lf g, NOX: %lf g, "
          "Total VMT: %lf miles, EV VMT: %lf miles, VHT: %lf hours, %d trips\n",
          m_fuel_truck (), m_CO2_truck (), m_HC_truck (), m_CO_truck (),
          m_NOX_truck (), m_VMT_truck (), m_VMT_ev_truck (), m_VHT_truck (),
          int (m_truck_set.size ()));
  return _s;
}
