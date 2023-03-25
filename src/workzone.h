#pragma once

#include "Snap.h"
#include "dlink.h"
#include "factory.h"
#include <algorithm>

struct Link_Workzone
{
  TInt m_link_ID;
  // TInt m_start_timestamp;
  // TInt m_end_timestamp;
};

class MNM_Workzone
{
public:
  MNM_Workzone (MNM_Node_Factory *node_factory, MNM_Link_Factory *link_factory,
                PNEGraph graph);
  ~MNM_Workzone ();

  MNM_Link_Factory *m_link_factory;
  MNM_Node_Factory *m_node_factory;
  PNEGraph m_graph;
  int init_workzone ();
  int update_workzone (TInt timestamp);
  int add_workzone_link (TInt link_ID);
  int delete_workzone_link (TInt link_ID);
  std::vector<MNM_Dlink *> m_disabled_link;
  std::vector<Link_Workzone> m_workzone_list;
};
