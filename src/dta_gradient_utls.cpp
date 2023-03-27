#include "dta_gradient_utls.h"

namespace MNM_DTA_GRADIENT
{
TFlt
get_link_inflow (MNM_Dlink *link, TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow link is null");
    }
  if (link->m_N_in == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow link "
                                "cummulative curve is not installed");
    }
  // printf("ss %d\n", link -> m_link_ID());
  // printf("%x\n", link -> m_N_in);
  return link->m_N_in->get_result (end_time)
         - link->m_N_in->get_result (start_time);
}

TFlt
get_link_inflow (MNM_Dlink *link, TInt start_time, TInt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow link is null");
    }
  if (link->m_N_in == nullptr)
    {
      throw std::runtime_error ("Error, get_link_inflow link "
                                "cummulative curve is not installed");
    }
  return link->m_N_in->get_result (TFlt (end_time))
         - link->m_N_in->get_result (TFlt (start_time));
}

TFlt
get_last_valid_time (MNM_Cumulative_Curve *N_in, MNM_Cumulative_Curve *N_out,
                     TInt end_loading_timestamp, const std::string &s)
{
  if (MNM_Ults::approximate_less_than (N_in->m_recorder.back ().second,
                                       N_out->m_recorder.back ().second)
      && !MNM_Ults::approximate_equal (N_in->m_recorder.back ().second,
                                       N_out->m_recorder.back ().second))
    {
      printf ("max in cc flow: %lf, max out cc flow: %lf\n",
              N_in->m_recorder.back ().second (),
              N_out->m_recorder.back ().second ());
      printf ("diff: %lf\n", N_in->m_recorder.back ().second
                               - N_out->m_recorder.back ().second);
      std::cout << s << std::endl;
      throw std::runtime_error ("invalid state of cumulative curve");
    }
  IAssert (end_loading_timestamp >= int (N_in->m_recorder.back ().first));
  IAssert (end_loading_timestamp >= int (N_out->m_recorder.back ().first));
  TFlt _cc_flow = N_out->m_recorder.back ().second;

  // link is empty after end_loading_timestamp
  if (MNM_Ults::approximate_equal (_cc_flow, N_in->m_recorder.back ().second))
    {
      return TFlt (end_loading_timestamp);
    }

  // link is not empty after end_loading_timestamp
  // first time reaching _cc_flow
  TFlt _last_valid_time = N_in->get_time (_cc_flow);
  IAssert (int (_last_valid_time) < int (end_loading_timestamp));
  if (_last_valid_time < 0)
    return TFlt (0);
  if (int (_last_valid_time) + 1 <= int (end_loading_timestamp))
    {
      for (int i = 1; i <= int (end_loading_timestamp) - int (_last_valid_time);
           ++i)
        {
          if (MNM_Ults::approximate_less_than (_cc_flow,
                                               N_in->get_result (
                                                 TFlt (_last_valid_time + i))))
            {
              return TFlt (_last_valid_time + i - 1);
            }
        }
    }
  throw std::runtime_error ("Error, MNM_DTA_GRADIENT::get_last_valid_time, "
                            "cannot find last valid time in N_in");
  return TFlt (-1);
}

TFlt
get_last_valid_time_bus (MNM_Cumulative_Curve *N_in,
                         MNM_Cumulative_Curve *N_out,
                         TInt end_loading_timestamp, const std::string &s)
{
  if (MNM_Ults::approximate_less_than (N_in->m_recorder.back ().second,
                                       N_out->m_recorder.back ().second)
      && !MNM_Ults::approximate_equal (N_in->m_recorder.back ().second,
                                       N_out->m_recorder.back ().second))
    {
      printf ("max in cc flow: %lf, max out cc flow: %lf\n",
              N_in->m_recorder.back ().second (),
              N_out->m_recorder.back ().second ());
      printf ("diff: %lf\n", N_in->m_recorder.back ().second
                               - N_out->m_recorder.back ().second);
      std::cout << s << std::endl;
      throw std::runtime_error ("invalid state of cumulative curve");
    }
  IAssert (end_loading_timestamp >= int (N_in->m_recorder.back ().first));
  IAssert (end_loading_timestamp >= int (N_out->m_recorder.back ().first));
  TFlt _cc_flow_out = N_out->m_recorder.back ().second;

  TFlt _last_valid_time = N_in->get_time (_cc_flow_out);
  TFlt _cc_flow = N_in->get_result (_last_valid_time);
  IAssert (!MNM_Ults::approximate_less_than (_cc_flow_out, _cc_flow));

  int _cc_floor = int (_cc_flow);
  int _cc_ceil = _cc_floor + 1;

  if (_cc_floor == 0 && !MNM_Ults::approximate_equal (_cc_flow, TFlt (1)))
    {
      return TFlt (0);
    }

  if (MNM_Ults::approximate_equal (_cc_flow, TFlt (_cc_floor)))
    {
      return _last_valid_time;
    }
  else if (MNM_Ults::approximate_equal (_cc_flow, TFlt (_cc_ceil)))
    {
      return _last_valid_time;
    }
  else
    {
      IAssert (MNM_Ults::approximate_less_than (TFlt (_cc_floor), _cc_flow));

      TFlt _last_valid_time_2 = N_in->get_time (TFlt (_cc_floor));
      TFlt _flow = N_in->get_result (TFlt (_last_valid_time_2));
      if (MNM_Ults::approximate_equal (_flow, TFlt (_cc_floor)))
        {
          return _last_valid_time_2;
        }
      else
        {
          return _last_valid_time;
        }
    }
}

TFlt
get_travel_time_from_cc (TFlt start_time, MNM_Cumulative_Curve *N_in,
                         MNM_Cumulative_Curve *N_out, TFlt last_valid_time,
                         TFlt fftt, bool rounding_up)
{
  if (last_valid_time <= 0)
    return fftt;
  if (start_time > last_valid_time)
    {
      start_time = last_valid_time;
    }

  TFlt _cc_flow = N_in->get_result (start_time);
  if (_cc_flow <= DBL_EPSILON)
    {
      return fftt;
    }

  // get the earliest time point in N_in that reaches the inflow == _cc_flow as
  // the true start_time TFlt _true_start_time = N_in -> get_time(_cc_flow);
  // IAssert(_true_start_time <= last_valid_time);

  // get the earliest time point in N_out that reaches the outflow == _cc_flow
  // as the end_time
  TFlt _end_time = N_out->get_time (_cc_flow, rounding_up);

  if (_end_time () < 0 || (_end_time - start_time < 0)
      || (_end_time - start_time < fftt))
    {
      return fftt; // in intervals
    }
  else
    {
      return (_end_time - start_time); // each interval is 5s
    }
}

TFlt
get_travel_time_from_FD (MNM_Dlink *link, TFlt start_time, TFlt unit_interval)
{
  TFlt _flow = link->m_N_in->get_result (start_time)
               - link->m_N_out->get_result (start_time);
  if (_flow < 0.)
    _flow = 0.;
  TFlt _tt = link->get_link_tt_from_flow (_flow);
  return _tt / unit_interval;
}

TFlt
get_travel_time (MNM_Dlink *link, TFlt start_time, TFlt unit_interval,
                 TInt end_loading_timestamp)
{
  // start_time actually means the timestamp used in CC to compute the link
  // travel time
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_travel_time link is null");
    }
  if (link->m_N_in == nullptr)
    {
      throw std::runtime_error (
        "Error, get_travel_time link cumulative curve is not installed");
    }

  // TFlt fftt = link->m_length/link->m_ffs/unit_interval;
  TFlt fftt = TFlt (
    int (link->get_link_freeflow_tt_loading ())); // actual intervals in loading

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
get_travel_time_robust (MNM_Dlink *link, TFlt start_time, TFlt end_time,
                        TFlt unit_interval, TInt end_loading_timestamp,
                        TInt num_trials)
{
  // start_time actually means the timestamp used in CC to compute the link
  // travel time
  num_trials = num_trials > TInt (end_time - start_time)
                 ? TInt (end_time - start_time)
                 : num_trials;
  TFlt _delta = (end_time - start_time) / TFlt (num_trials);
  TFlt _ave_tt = TFlt (0);
  // will not use end_time
  for (int i = 0; i < num_trials (); ++i)
    {
      _ave_tt += get_travel_time (link, start_time + TFlt (i) * _delta,
                                  unit_interval, end_loading_timestamp);
    }
  return _ave_tt / TFlt (num_trials);
}

TFlt
get_path_travel_time (MNM_Path *path, TFlt start_time,
                      MNM_Link_Factory *link_factory, TFlt unit_interval,
                      TInt end_loading_timestamp)
{
  // start_time means the actual departure time
  if (path == nullptr)
    {
      throw std::runtime_error ("Error, get_path_travel_time path is null");
    }
  if (link_factory == nullptr)
    {
      throw std::runtime_error (
        "Error, get_path_travel_time link link_factory is null");
    }
  int _end_time = int (round (start_time));
  MNM_Dlink *_link;
  for (auto _link_ID : path->m_link_vec)
    {
      _link = link_factory->get_link (_link_ID);
      // assume each link's travel time >= unit interval
      // use _end_time + 1 as start_time in cc to compute the link travel time
      // for vehicles arriving at the beginning of interval _end_time
      _end_time = _end_time
                  + MNM_Ults::round_up_time (
                    get_travel_time (_link, _end_time + 1, unit_interval,
                                     end_loading_timestamp));
    }
  return TFlt (_end_time - start_time); // # of unit intervals
}

TFlt
get_path_travel_time (MNM_Path *path, TFlt start_time,
                      std::unordered_map<TInt, TFlt *> &link_tt_map,
                      TInt end_loading_timestamp)
{
  // start_time means the actual departure time
  if (path == nullptr)
    {
      throw std::runtime_error ("Error, get_path_travel_time path is null");
    }
  int _cur_time = int (round (start_time));
  for (TInt _link_ID : path->m_link_vec)
    {
      _cur_time += MNM_Ults::round_up_time (
        link_tt_map[_link_ID][_cur_time < (int) end_loading_timestamp
                                ? _cur_time
                                : (int) end_loading_timestamp - 1]);
    }
  return TFlt (_cur_time - start_time);
}

TFlt
get_path_travel_cost (MNM_Path *path, TFlt start_time,
                      std::unordered_map<TInt, TFlt *> &link_tt_map,
                      std::unordered_map<TInt, TFlt *> &link_cost_map,
                      TInt end_loading_timestamp)
{
  // start_time means the actual departure time
  if (path == nullptr)
    {
      throw std::runtime_error ("Error, get_path_travel_time path is null");
    }
  int _cur_time = int (round (start_time));
  TFlt _cost = 0.;
  for (TInt _link_ID : path->m_link_vec)
    {
      _cost += link_cost_map[_link_ID][_cur_time < (int) end_loading_timestamp
                                         ? _cur_time
                                         : (int) end_loading_timestamp - 1];
      _cur_time += MNM_Ults::round_up_time (
        link_tt_map[_link_ID][_cur_time < (int) end_loading_timestamp
                                ? _cur_time
                                : (int) end_loading_timestamp - 1]);
    }
  return _cost;
}

int
add_dar_records (std::vector<dar_record *> &record, MNM_Dlink *link,
                 std::unordered_map<MNM_Path *, int> path_map, TFlt start_time,
                 TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records link is null");
    }
  if (link->m_N_in_tree == nullptr)
    {
      throw std::runtime_error (
        "Error, add_dar_records link cumulative curve tree is not installed");
    }
  MNM_Path *_path;
  for (const auto &path_it : link->m_N_in_tree->m_record)
    {
      _path = path_it.first;
      // if (std::find(pathset.begin(), pathset.end(), _path) != pathset.end())
      // {
      if (path_map.find (_path) != path_map.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  auto new_record = new dar_record ();
                  new_record->path_ID = path_it.first->m_path_ID;
                  new_record->assign_int = depart_it.first;
                  new_record->link_ID = link->m_link_ID;
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
add_dar_records_eigen (std::vector<Eigen::Triplet<double>> &record,
                       MNM_Dlink *link,
                       std::unordered_map<MNM_Path *, int> path_map,
                       TFlt start_time, TFlt end_time, int link_ind,
                       int interval_ind, int num_e_link, int num_e_path,
                       const double *f_ptr)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen link is null");
    }
  if (link->m_N_in_tree == nullptr)
    {
      throw std::runtime_error ("Error, add_dar_records_eigen link cumulative "
                                "curve tree is not installed");
    }
  MNM_Path *_path;
  int _x, _y;
  for (const auto &path_it : link->m_N_in_tree->m_record)
    {
      _path = path_it.first;
      // if (std::find(pathset.begin(), pathset.end(), _path) != pathset.end())
      // {
      auto _path_iter = path_map.find (_path);
      if (_path_iter != path_map.end ())
        {
          for (auto depart_it : path_it.second)
            {
              TFlt tmp_flow = depart_it.second->get_result (end_time)
                              - depart_it.second->get_result (start_time);
              if (tmp_flow > DBL_EPSILON)
                {
                  _x = link_ind
                       + num_e_link
                           * interval_ind; // # of links * # of intervals
                  _y = (*_path_iter).second
                       + num_e_path
                           * depart_it.first; // # of paths * # of intervals
                  // printf("Adding record, %d, %d, %d, %f, %f\n", new_record ->
                  // path_ID(), new_record -> assign_int(),
                  //     new_record -> link_ID(), (float)new_record ->
                  //     link_start_int(), (float) new_record -> flow());

                  // https://eigen.tuxfamily.org/dox/classEigen_1_1Triplet.html
                  // https://eigen.tuxfamily.org/dox/SparseUtil_8h_source.html
                  // (row index, col index, value)
                  record.push_back (
                    Eigen::Triplet<double> ((double) _x, (double) _y,
                                            tmp_flow () / f_ptr[_y]));
                }
            }
        }
    }
  return 0;
}

TFlt
get_departure_cc_slope (MNM_Dlink *link, TFlt start_time, TFlt end_time)
{
  if (link == nullptr)
    {
      throw std::runtime_error ("Error, get_departure_cc_slope link is null");
    }
  if (link->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, get_departure_cc_slope link cumulative curve is not installed");
    }
  if (start_time > link->m_N_out->m_recorder.back ().first)
    {
      return 0;
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
  // if (_slope <= 0) {
  // 	std::cout << "car in" << "\n";
  // 	std::cout << link -> m_N_in -> to_string() << "\n";
  // 	std::cout << "car out" << "\n";
  // 	std::cout << link -> m_N_out -> to_string() << "\n";
  // 	printf("debug");
  // }
  return _slope / _delta; // flow per unit interval
}

} // end namespace MNM_DTA_GRADIENT
