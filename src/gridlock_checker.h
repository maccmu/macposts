#include "factory.h"
#include "vehicle.h"
#include <deque>
#include <set>
#include <unordered_map>

class MNM_Gridlock_Checker
{
public:
  MNM_Gridlock_Checker (PNEGraph graph, MNM_Link_Factory *link_factory);
  ~MNM_Gridlock_Checker ();

  bool is_gridlocked ();
  int initialize ();
  MNM_Veh *get_last_veh (MNM_Dlink *link);
  bool static has_cycle (PNEGraph graph);

  MNM_Link_Factory *m_link_factory;
  std::unordered_map<TInt, MNM_Veh *> m_link_veh_map;
  PNEGraph m_full_graph;
  PNEGraph m_gridlock_graph;
};

class MNM_Gridlock_Link_Recorder
{
public:
  MNM_Gridlock_Link_Recorder (const std::string &file_folder,
                              MNM_ConfReader *record_config);
  virtual ~MNM_Gridlock_Link_Recorder ();
  int init_record ();
  virtual int save_one_link (TInt loading_interval, MNM_Dlink *link);
  int post_record ();

  MNM_ConfReader *m_config;
  std::ofstream m_record_file;
};