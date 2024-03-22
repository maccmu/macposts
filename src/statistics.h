#pragma once

#include "Snap.h"
#include "dlink.h"
#include "dnode.h"
#include "enum.h"
#include "factory.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class MNM_Statistics
{
public:
  MNM_Statistics (const std::string &file_folder, MNM_ConfReader *conf_reader,
                  MNM_ConfReader *record_config, MNM_OD_Factory *od_factory,
                  MNM_Node_Factory *node_factory,
                  MNM_Link_Factory *link_factory);
  virtual ~MNM_Statistics ();

  MNM_ConfReader *m_self_config;

  /* may or may not be initialized */
  std::unordered_map<TInt, TFlt> m_load_interval_volume;
  std::unordered_map<TInt, TFlt> m_record_interval_volume;
  std::unordered_map<TInt, TFlt> m_record_interval_tt;
  std::unordered_map<TInt, TFlt> m_load_interval_tt;

  std::vector<MNM_Dlink *> m_link_order;
  // since iteration of the unordered_map is not guaranteed the same order,
  // so we keep the order first

  /* universal function */
  virtual int record_loading_interval_condition (TInt timestamp);
  virtual int record_record_interval_condition (TInt timestamp);
  virtual int update_record (TInt timestamp) { return 0; };
  virtual int init_record ();
  virtual int post_record ();

protected:
  int init_record_value ();
  bool m_record_volume;
  bool m_record_tt;
  std::string m_file_folder;
  Record_type m_record_type;

  MNM_ConfReader *m_global_config;
  MNM_OD_Factory *m_od_factory;
  MNM_Node_Factory *m_node_factory;
  MNM_Link_Factory *m_link_factory;

  std::ofstream m_load_interval_volume_file;
  std::ofstream m_record_interval_volume_file;
  std::ofstream m_load_interval_tt_file;
  std::ofstream m_record_interval_tt_file;
};

class MNM_Statistics_Lrn : public MNM_Statistics
{
public:
  MNM_Statistics_Lrn (const std::string &file_folder,
                      MNM_ConfReader *conf_reader,
                      MNM_ConfReader *record_config, MNM_OD_Factory *od_factory,
                      MNM_Node_Factory *node_factory,
                      MNM_Link_Factory *link_factory);
  virtual ~MNM_Statistics_Lrn () override;
  virtual int update_record (TInt timestamp) override;
  virtual int init_record () override;
  TInt m_n;

  // private:
  std::unordered_map<TInt, TFlt> m_to_be_volume;
  std::unordered_map<TInt, TFlt> m_to_be_tt;
};
