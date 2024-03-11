#pragma once

#include "common.h"
#include "enum.h"
#include "factory.h"
#include "path.h"
#include "ults.h"
#include "vms.h"
#include "workzone.h"
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class MNM_Node_Factory;

namespace macposts
{
  namespace io
  {
    macposts::Graph build_graph(const std::string &file_folder,
                                MNM_ConfReader *conf_reader);
  }
}

class MNM_IO
{
public:
  static int build_node_factory (const std::string &file_folder,
                                 MNM_ConfReader *conf_reader,
                                 MNM_Node_Factory *node_factory,
                                 const std::string &file_name
                                 = "MNM_input_node");
  static int build_link_factory (const std::string &file_folder,
                                 MNM_ConfReader *conf_reader,
                                 MNM_Link_Factory *link_factory,
                                 const std::string &file_name
                                 = "MNM_input_link");
  static int build_od_factory (const std::string &file_folder,
                               MNM_ConfReader *conf_reader,
                               MNM_OD_Factory *od_factory,
                               MNM_Node_Factory *node_factory,
                               const std::string &file_name = "MNM_input_od");
  static int build_od_factory (const std::string &file_folder,
                               MNM_ConfReader *conf_reader,
                               MNM_OD_Factory *od_factory,
                               const std::string &file_name = "MNM_input_od");
  static int hook_up_od_node (const std::string &file_folder,
                              MNM_ConfReader *conf_reader,
                              MNM_OD_Factory *od_factory,
                              MNM_Node_Factory *node_factory,
                              const std::string &file_name = "MNM_input_od");
  static PNEGraph build_graph (const std::string &file_folder,
                               MNM_ConfReader *conf_reader);
  static int build_demand (const std::string &file_folder,
                           MNM_ConfReader *conf_reader,
                           MNM_OD_Factory *od_factory,
                           const std::string &file_name = "MNM_input_demand");
  static Path_Table *load_path_table (const std::string &file_name,
                                      const PNEGraph &graph, TInt num_path,
                                      bool w_buffer = false, bool w_ID = false);
  static Path_Table *load_path_table (const std::string &file_name,
                                      const macposts::Graph &graph,
                                      TInt num_path, bool w_buffer = false,
                                      bool w_ID = false);
  static int build_vms_facotory (const std::string &file_folder, PNEGraph graph,
                                 TInt num_vms, MNM_Vms_Factory *vms_factory,
                                 const std::string &file_name
                                 = "MNM_input_vms");
  static int build_vms_facotory (const std::string &file_folder,
                                 const macposts::Graph &graph, TInt num_vms,
                                 MNM_Vms_Factory *vms_factory,
                                 const std::string &file_name
                                 = "MNM_input_vms");
  static int read_int_float (const std::string &file_name,
                             std::unordered_map<TInt, TFlt> *reader);
  static int read_int (const std::string &file_name, std::vector<TInt> *reader);
  static int read_float (const std::string &file_name,
                         std::vector<TFlt *> *reader);
  static int
  build_workzone_list (const std::string &file_folder, MNM_Workzone *workzone,
                       const std::string &file_name = "MNM_input_workzone");
  static int dump_cumulative_curve (const std::string &file_folder,
                                    MNM_Link_Factory *link_factory,
                                    const std::string &file_name = "cc_record");
  static int
  build_link_toll (const std::string &file_folder, MNM_ConfReader *conf_reader,
                   MNM_Link_Factory *link_factory,
                   const std::string &file_name = "MNM_input_link_toll");
  static int read_td_link_cost (const std::string &file_folder,
                                std::unordered_map<TInt, TFlt *> &td_link_cost,
                                const TInt num_rows, const TInt num_timestamps,
                                const std::string &file_name
                                = "tdsp_link_cost");
  static int read_td_node_cost (
    const std::string &file_folder,
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &td_node_cost,
    const TInt num_rows, const TInt num_timestamps,
    const std::string &file_name = "tdsp_node_cost");

  static int read_origin_vehicle_label_ratio (const std::string &file_folder,
                                              MNM_ConfReader *conf_reader,
                                              MNM_OD_Factory *od_factory,
                                              const std::string &file_name
                                              = "MNM_input_origin_vehicle_label");

  static int read_vehicle_tracking_setting (const std::string &file_folder,
                                              std::vector<std::pair<int, int>> *od_pair_tracked,
                                              std::vector<int> *interval_tracked,
                                              const std::string &od_tracking_file_name
                                              = "MNM_input_od_tracking",
                                              const std::string &interval_tracking_file_name
                                              = "MNM_input_interval_tracking");

  // private:
  static std::vector<std::string> split (const std::string &text, char sep);
  static std::string inline &ltrim (std::string &s)
  {
    s.erase (s.begin (), std::find_if (s.begin (), s.end (), [] (int c) {
               return !std::isspace (c);
             }));
    return s;
  }

  // trim from end
  static std::string inline &rtrim (std::string &s)
  {
    s.erase (std::find_if (s.rbegin (), s.rend (),
                           [] (int c) { return !std::isspace (c); })
               .base (),
             s.end ());
    return s;
  }

  // trim from both ends
  static std::string inline &trim (std::string &s) { return ltrim (rtrim (s)); }
};
