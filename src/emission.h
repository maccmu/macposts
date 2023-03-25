#pragma once

#include "dlink.h"
#include "ults.h"

#include <math.h>
#include <unordered_set>
#include <vector>

class MNM_Cumulative_Emission
{
public:
  // -1 is the default veh -> m_label value, use -2 ad ev label
  MNM_Cumulative_Emission (TFlt unit_time, TInt freq, TInt ev_label = -2);
  virtual ~MNM_Cumulative_Emission ();
  std::vector<MNM_Dlink *> m_link_vector;
  std::unordered_set<MNM_Dlink *> m_link_set;

  int register_link (MNM_Dlink *link);
  // v should be in mile/hour
  TFlt calculate_fuel_rate (TFlt v);
  TFlt calculate_fuel_rate_deprecated (TFlt v);
  TFlt calculate_CO2_rate (TFlt v);
  TFlt calculate_CO2_rate_deprecated (TFlt v);
  TFlt calculate_HC_rate (TFlt v);
  TFlt calculate_CO_rate (TFlt v);
  TFlt calculate_NOX_rate (TFlt v);

  virtual int update (MNM_Veh_Factory *veh_factory);
  virtual std::string output ();

  TFlt m_fuel;
  TFlt m_CO2;
  TFlt m_HC;
  TFlt m_CO;
  TFlt m_NOX;
  TFlt m_unit_time;
  TInt m_freq;
  TInt m_counter;
  TFlt m_VMT;
  TFlt m_VMT_ev;
  TInt m_ev_label;
};
