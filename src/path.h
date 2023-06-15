#pragma once

#include "Snap.h"
#include "factory.h"
#include "shortest_path.h"

#include <deque>
#include <fstream>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

class MNM_Path
{
public:
  MNM_Path ();
  virtual ~MNM_Path ();
  std::string node_vec_to_string ();
  std::string link_vec_to_string ();
  std::string buffer_to_string ();
  std::deque<TInt> m_link_vec;
  std::deque<TInt> m_node_vec;
  TInt m_path_ID;
  TFlt m_p;
  TFlt *m_buffer;
  TInt m_buffer_length;
  // only used in multimodal
  int m_path_type = -1;

  std::set<TInt> m_link_set;
  virtual bool is_link_in (TInt link_ID);

  TFlt get_path_tt (MNM_Link_Factory *link_factory);
  TFlt get_path_fftt (MNM_Link_Factory *link_factory);
  TFlt get_path_length (MNM_Link_Factory *link_factory);
  int allocate_buffer (TInt length);
  int eliminate_cycles ();

  inline bool operator== (const MNM_Path &rhs)
  {
    if (m_link_vec.size () != rhs.m_link_vec.size ())
      return false;
    // compare link ID for MultiGraph
    for (size_t i = 0; i < rhs.m_link_vec.size (); ++i)
      {
        if (rhs.m_link_vec[i] != m_link_vec[i])
          return false;
      }
    return true;
  }

  // for storing time-dependent value for DUE
  std::vector<TFlt> m_travel_time_vec;
  std::vector<TFlt> m_travel_cost_vec;
  std::vector<TFlt> m_travel_disutility_vec;
  std::string time_vec_to_string ();
  std::string cost_vec_to_string ();
  std::string disutility_vec_to_string ();
};

struct LessByPathP
{
  bool operator() (const MNM_Path *lhs, const MNM_Path *rhs) const
  {
    return lhs->m_p >= rhs->m_p;
  }
};

class MNM_Pathset
{
public:
  MNM_Pathset ();
  virtual ~MNM_Pathset ();
  std::vector<MNM_Path *> m_path_vec;
  int normalize_p ();
  virtual bool is_in (MNM_Path *path);
};

// <O_node_ID, <D_node_ID, Pathset>>
typedef std::unordered_map<TInt, std::unordered_map<TInt, MNM_Pathset *> *>
  Path_Table;

namespace MNM
{
MNM_Path *extract_path (TInt origin_ID, TInt dest_ID,
                        std::unordered_map<TInt, TInt> &output_map,
                        PNEGraph &graph);
// one-shot cost
TFlt get_path_tt_snapshot(MNM_Path* path, const std::unordered_map<TInt, TFlt> &link_cost_map);
// time-dependent cost
TFlt get_path_tt(TFlt start_time, MNM_Path* path, const std::unordered_map<TInt, TFlt*> &link_cost_map, TInt max_interval);
Path_Table *build_pathset (PNEGraph &graph, MNM_OD_Factory *od_factory,
                           MNM_Link_Factory *link_factory,
                           TFlt min_path_length = 0.0, size_t MaxIter = 10,
                           TFlt vot = 3., TFlt Mid_Scale = 3,
                           TFlt Heavy_Scale = 6, TInt buffer_length = -1);
int save_path_table (const std::string &file_folder, Path_Table *path_table,
                     MNM_OD_Factory *m_od_factory, bool w_buffer = false,
                     bool w_cost = false);
int print_path_table (Path_Table *path_table, MNM_OD_Factory *m_od_factory,
                      bool w_buffer = false, bool w_cost = false);
Path_Table *build_shortest_pathset (PNEGraph &graph, MNM_OD_Factory *od_factory,
                                    MNM_Link_Factory *link_factory);
int allocate_path_table_buffer (Path_Table *path_table, TInt num);
int normalize_path_table_p (Path_Table *path_table);
int copy_p_to_buffer (Path_Table *path_table, TInt col);
int copy_buffer_to_p (Path_Table *path_table, TInt col);
int get_ID_path_mapping (std::unordered_map<TInt, MNM_Path *> &map,
                         Path_Table *path_table);
MNM_Pathset *get_pathset (Path_Table *path_table, TInt origin_node_ID,
                          TInt dest_node_ID);
}
