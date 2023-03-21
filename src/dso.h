#ifndef DSO_H
#define DSO_H

#include "due.h"

class MNM_Dso : public MNM_Due_Msa
{
public:
  MNM_Dso (std::string file_folder);

  virtual ~MNM_Dso () override;

  virtual TFlt get_disutility (TFlt depart_time, TFlt tt) override;

  virtual int build_link_cost_map (MNM_Dta *dta) override;

  int get_link_marginal_cost (MNM_Dta *dta);

  std::unordered_map<TInt, bool *> m_link_congested;

  // time-varying queue dissipated time
  std::unordered_map<TInt, int *> m_queue_dissipated_time;
};

#endif