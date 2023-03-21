#ifndef DUE_H
#define DUE_H

#include "dta.h"
#include "dta_gradient_utls.h"
#include "limits.h"
#include "path.h"
#include "realtime_dta.h"

class MNM_Due
{
public:
  MNM_Due (std::string file_folder);

  virtual ~MNM_Due ();

  virtual int initialize () { return 0; };

  MNM_Dta *run_dta (bool verbose);

  virtual int init_path_flow () { return 0; };

  virtual int update_path_table (MNM_Dta *dta, int iter) { return 0; };

  virtual int update_path_table_fixed_departure_time_choice (MNM_Dta *dta,
                                                             int iter)
  {
    return 0;
  };

  virtual int update_path_table_gp_fixed_departure_time_choice (MNM_Dta *dta,
                                                                int iter)
  {
    return 0;
  };

  TFlt compute_total_travel_time ();

  TFlt compute_merit_function ();

  TFlt compute_merit_function_fixed_departure_time_choice ();

  virtual TFlt get_disutility (TFlt depart_time, TFlt tt);

  TFlt get_tt (TFlt depart_time, MNM_Path *path);

  virtual int build_link_cost_map (MNM_Dta *dta);

  int update_path_table_cost (MNM_Dta *dta);

  int update_one_path_cost (MNM_Path *path, TInt o_node_ID, TInt d_node_ID,
                            MNM_Dta *dta);

  int update_demand_from_path_table (MNM_Dta *dta);

  TFlt compute_total_demand (MNM_Origin *orig, MNM_Destination *dest,
                             TInt total_assign_inter);

  std::string m_file_folder;
  TFlt m_unit_time;
  TInt m_total_loading_inter;
  Path_Table *m_path_table;
  // MNM_OD_Factory *m_od_factory;
  TInt m_total_assign_inter;
  MNM_ConfReader *m_dta_config;
  MNM_ConfReader *m_due_config;

  TFlt m_vot;
  TFlt m_early_penalty;
  TFlt m_late_penalty;
  TFlt m_target_time;
  TFlt m_step_size;

  std::unordered_map<TInt, TFlt *> m_link_tt_map;
  std::unordered_map<TInt, TFlt *> m_link_cost_map;
};

class MNM_Due_Msa : public MNM_Due
{
public:
  MNM_Due_Msa (std::string file_folder);

  virtual ~MNM_Due_Msa () override;

  virtual int initialize () override;

  virtual int init_path_flow () override;

  virtual int update_path_table (MNM_Dta *dta, int iter) override;

  virtual int update_path_table_fixed_departure_time_choice (MNM_Dta *dta,
                                                             int iter) override;

  virtual int
  update_path_table_gp_fixed_departure_time_choice (MNM_Dta *dta,
                                                    int iter) override;

  std::pair<MNM_Path *, TInt> get_best_route (TInt o_node_ID,
                                              MNM_TDSP_Tree *tdsp_tree);

  std::pair<MNM_Path *, TInt>
  get_best_route_for_single_interval (TInt interval, TInt o_node_ID,
                                      MNM_TDSP_Tree *tdsp_tree);

  MNM_Dta *m_base_dta;
};

#endif