#include "dso.h"

MNM_Dso::MNM_Dso (std::string file_folder)
    : MNM_Due_Msa::MNM_Due_Msa (file_folder)
{
  m_link_congested = std::unordered_map<TInt, bool *> ();
  m_queue_dissipated_time = std::unordered_map<TInt, int *> ();
}

MNM_Dso::~MNM_Dso ()
{
  for (auto _it : m_link_congested)
    {
      delete _it.second;
    }
  m_link_congested.clear ();
  for (auto _it : m_queue_dissipated_time)
    {
      delete _it.second;
    }
  m_queue_dissipated_time.clear ();
}

TFlt
MNM_Dso::get_disutility (TFlt depart_time, TFlt tt)
{
  return tt;
}

int
MNM_Dso::build_link_cost_map (MNM_Dta *dta)
{
  MNM_Dlink *_link;
  for (auto _link_it : dta->m_link_factory->m_link_map)
    {
      if (m_link_congested.find (_link_it.first) == m_link_congested.end ())
        {
          m_link_congested[_link_it.first] = new bool[m_total_loading_inter];
        }
      std::cout << "********************** link " << _link_it.first ()
                << " **********************\n";
      for (int i = 0; i < m_total_loading_inter; i++)
        {
          _link = _link_it.second;
          // use i+1 as start_time in cc to compute link travel time for
          // vehicles arriving at the beginning of interval i, i+1 is the end of
          // the interval i, the beginning of interval i + 1
          m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time (_link, TFlt (i + 1), m_unit_time,
                             dta->m_current_loading_interval); // intervals
          m_link_cost_map[_link_it.first][i]
            = m_vot * m_link_tt_map[_link_it.first][i] + _link->m_toll;
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_link_cost_map[_link_it.first][i] << "\n";
          m_link_congested[_link_it.first][i]
            = m_link_tt_map[_link_it.first][i]
              > _link->get_link_freeflow_tt_loading ();
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", congested?: " << m_link_congested[_link_it.first][i] << "\n";
        }
    }
  return 0;
}

int
MNM_Dso::get_link_marginal_cost (MNM_Dta *dta)
{
  // suppose m_link_congested is constructed already in build_link_cost_map()
  MNM_Dlink *_link;
  int _total_loading_inter
    = m_total_loading_inter; // dta -> m_current_loading_interval;
  IAssert (_total_loading_inter > 0);
  int _link_fft; // intervals
  bool _flg;
  std::cout << "\n********************** Begin MNM_Dso::get_link_marginal_cost "
               "**********************\n";
  for (auto _link_it : dta->m_link_factory->m_link_map)
    {
      if (m_queue_dissipated_time.find (_link_it.first)
          == m_queue_dissipated_time.end ())
        {
          m_queue_dissipated_time[_link_it.first]
            = new int[_total_loading_inter];
        }
      _link = dynamic_cast<MNM_Dlink *> (_link_it.second);
      _link_fft = _link->get_link_freeflow_tt_loading ();
      // std::cout << "********************** get_link_queue_dissipated_time
      // link " << _link_it.first << " **********************\n";
      for (int i = 0; i < _total_loading_inter; i++)
        {
          // ************************** car **************************
          if (m_link_congested[_link_it.first][i])
            {
              if (i == _total_loading_inter - 1)
                {
                  m_queue_dissipated_time[_link_it.first][i]
                    = _total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < _total_loading_inter; k++)
                    {
                      if (m_link_congested[_link_it.first][k - 1]
                          && !m_link_congested[_link_it.first][k])
                        {
                          m_queue_dissipated_time[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time[_link_it.first][i]
                        = _total_loading_inter;
                    }
                }
            }
          else
            {
              if (MNM_Ults::approximate_equal (m_link_tt_map[_link_it.first][i],
                                               (float) _link_fft))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm *> (_link) != nullptr)
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
                      TFlt _outflow_rate = MNM_DTA_GRADIENT::
                        get_departure_cc_slope (_link,
                                                TFlt (i + (int) _link_fft),
                                                TFlt (i + (int) _link_fft
                                                      + 1)); // veh / 5s
                      TFlt _cap = dynamic_cast<MNM_Dlink_Ctm *> (_link)
                                    ->m_cell_array.back ()
                                    ->m_flow_cap
                                  * dta->m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * dta->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * dta->m_flow_scalar)))
                        {
                          if (i == _total_loading_inter - 1)
                            {
                              m_queue_dissipated_time[_link_it.first][i]
                                = _total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < _total_loading_inter; k++)
                                {
                                  if (m_link_congested[_link_it.first][k - 1]
                                      && !m_link_congested[_link_it.first][k])
                                    {
                                      m_queue_dissipated_time[_link_it.first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time[_link_it.first][i]
                                    = _total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq *> (_link) != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "MNM_Dso::get_link_marginal_cost, Link type not "
                        "implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time[_link_it.first][i] = i;
                  throw std::runtime_error ("MNM_Dso::get_link_marginal_cost, "
                                            "Link travel time less than fftt");
                }
              // m_queue_dissipated_time[_link_it.first][i] = i;
            }

          // reuse m_link_tt_map and m_link_cost_map
          m_link_tt_map[_link_it.first][i]
            = m_queue_dissipated_time[_link_it.first][i] - i + _link_fft;
          m_link_cost_map[_link_it.first][i] = m_link_tt_map[_link_it.first][i];
        }
    }
  std::cout << "********************** End MNM_Dso::get_link_marginal_cost "
               "**********************\n";
  return 0;
}