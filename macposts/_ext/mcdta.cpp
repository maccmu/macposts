// Bi-class DTA

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <set>
#include <unordered_map>

#include "utils.h"
#include <Snap.h>
#include <multiclass.h>
#include <multimodal.h>

namespace py = pybind11;
using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

namespace macposts
{
namespace mcdta
{
class Mcdta
{
public:
  Mcdta ();
  ~Mcdta ();
  int initialize (const std::string &folder);
  bool check_input_files ();
  int generate_shortest_pathsets (const std::string &folder, int max_iter,
                                  double vot, double mid_scale,
                                  double heavy_scale, double min_path_tt = 0.);
  int install_cc ();
  int install_cc_tree ();
  int run_whole (bool verbose = false);
  // FIXME: This returns a Numpy array for consistency, but it should really be
  // better to use a plain list.
  py::array_t<int> get_all_links ();
  int register_links (py::array_t<int> links);
  // FIXME: This returns a Numpy array for consistency, but it should really be
  // better to use a plain list.
  py::array_t<int> get_registered_links ();
  int get_cur_loading_interval ();
  py::array_t<double> get_travel_stats ();
  std::string print_emission_stats ();
  int print_simulation_results (const std::string &folder,
                                int cong_frequency = 180);

  int build_link_cost_map (bool with_congestion_indicator = false);
  int get_link_queue_dissipated_time ();
  int update_tdsp_tree ();
  py::array_t<int> get_lowest_cost_path (int start_interval, int o_node_ID,
                                         int d_node_ID);

  py::array_t<double> get_car_link_fftt (py::array_t<int> link_IDs);
  py::array_t<double> get_truck_link_fftt (py::array_t<int> link_IDs);

  py::array_t<double> get_car_link_tt (py::array_t<double> start_intervals,
                                       bool return_inf = false);
  py::array_t<double>
  get_car_link_tt_robust (py::array_t<double> start_intervals,
                          py::array_t<double> end_intervals,
                          int num_trials = 180, bool return_inf = false);
  py::array_t<double> get_truck_link_tt (py::array_t<double> start_intervals,
                                         bool return_inf = false);
  py::array_t<double>
  get_truck_link_tt_robust (py::array_t<double> start_intervals,
                            py::array_t<double> end_intervals,
                            int num_trials = 180, bool return_inf = false);

  py::array_t<double> get_car_link_speed (py::array_t<double> start_intervals);
  py::array_t<double>
  get_truck_link_speed (py::array_t<double> start_intervals);

  py::array_t<double> get_link_car_inflow (py::array_t<int> start_intervals,
                                           py::array_t<int> end_intervals);
  py::array_t<double> get_link_truck_inflow (py::array_t<int> start_intervals,
                                             py::array_t<int> end_intervals);

  int register_paths (py::array_t<int> paths);

  std::vector<bool> check_registered_links_in_registered_paths ();
  py::array_t<bool> are_registered_links_in_registered_paths ();
  py::array_t<int> generate_paths_to_cover_registered_links ();
  int save_path_table (const std::string &folder);

  py::array_t<double> get_car_link_out_cc (int link_ID);
  py::array_t<double> get_car_link_in_cc (int link_ID);
  py::array_t<double> get_truck_link_out_cc (int link_ID);
  py::array_t<double> get_truck_link_in_cc (int link_ID);

  py::array_t<double> get_enroute_and_queue_veh_stats_agg ();
  py::array_t<double> get_queue_veh_each_link (py::array_t<int> useful_links,
                                               py::array_t<int> intervals);

  double get_car_link_out_num (int link_ID, double time);
  double get_truck_link_out_num (int link_ID, double time);

  py::array_t<double> get_car_dar_matrix (py::array_t<int> start_intervals,
                                          py::array_t<int> end_intervals);
  py::array_t<double> get_truck_dar_matrix (py::array_t<int> start_intervals,
                                            py::array_t<int> end_intervals);

  int save_car_dar_matrix (py::array_t<int> start_intervals,
                           py::array_t<int> end_intervals,
                           py::array_t<double> f, const std::string &file_name);
  int save_truck_dar_matrix (py::array_t<int> start_intervals,
                             py::array_t<int> end_intervals,
                             py::array_t<double> f,
                             const std::string &file_name);

  SparseMatrixR get_complete_car_dar_matrix (py::array_t<int> start_intervals,
                                             py::array_t<int> end_intervals,
                                             int num_intervals,
                                             py::array_t<double> f);
  SparseMatrixR get_complete_truck_dar_matrix (py::array_t<int> start_intervals,
                                               py::array_t<int> end_intervals,
                                               int num_intervals,
                                               py::array_t<double> f);

  py::array_t<double> get_waiting_time_at_intersections ();
  py::array_t<double> get_waiting_time_at_intersections_car ();
  py::array_t<double> get_waiting_time_at_intersections_truck ();
  py::array_t<int> get_link_spillback ();
  py::array_t<double>
  get_avg_link_on_path_tt_car (py::array_t<int> link_IDs,
                               py::array_t<double> start_intervals);
  py::array_t<double>
  get_avg_link_on_path_tt_truck (py::array_t<int> link_IDs,
                                 py::array_t<double> start_intervals);
  // assume build_link_cost_map() is invoked before
  py::array_t<double> get_path_tt_car (py::array_t<int> link_IDs,
                                       py::array_t<int> start_intervals);
  py::array_t<double> get_path_tt_truck (py::array_t<int> link_IDs,
                                         py::array_t<int> start_intervals);
  py::array_t<double>
  get_registered_path_tt_car (py::array_t<int> start_intervals);
  py::array_t<double>
  get_registered_path_tt_truck (py::array_t<int> start_intervals);

  py::array_t<double> get_car_ltg_matrix (py::array_t<int> start_intervals,
                                          int threshold_timestamp);
  py::array_t<double> get_truck_ltg_matrix (py::array_t<int> start_intervals,
                                            int threshold_timestamp);

  SparseMatrixR get_complete_car_ltg_matrix (py::array_t<int> start_intervals,
                                             int threshold_timestamp,
                                             int num_intervals);
  SparseMatrixR get_complete_truck_ltg_matrix (py::array_t<int> start_intervals,
                                               int threshold_timestamp,
                                               int num_intervals);

  int delete_all_agents ();

  MNM_Dta_Multiclass *m_mcdta;
  std::vector<MNM_Dlink_Multiclass *> m_link_vec;
  std::vector<MNM_Path *> m_path_vec;
  std::set<MNM_Path *> m_path_set;
  std::unordered_map<TInt, MNM_Path *> m_ID_path_mapping;

  // time-varying link tt
  std::unordered_map<TInt, TFlt *> m_link_tt_map;
  std::unordered_map<TInt, TFlt *> m_link_tt_map_truck;

  // time-varying link cost
  std::unordered_map<TInt, TFlt *> m_link_cost_map;
  std::unordered_map<TInt, TFlt *> m_link_cost_map_truck;

  // time-varying indicator
  std::unordered_map<TInt, bool *> m_link_congested_car;
  std::unordered_map<TInt, bool *> m_link_congested_truck;

  // time-varying queue dissipated time
  std::unordered_map<TInt, int *> m_queue_dissipated_time_car;
  std::unordered_map<TInt, int *> m_queue_dissipated_time_truck;

  std::unordered_map<TInt, MNM_TDSP_Tree *> m_tdsp_tree_map;
};

void
init (py::module &m)
{
  py::class_<Mcdta> (m, "Mcdta")
    .def (py::init<> ())
    .def ("initialize", &Mcdta::initialize)
    .def ("check_input_files", &Mcdta::check_input_files)
    .def ("generate_shortest_pathsets", &Mcdta::generate_shortest_pathsets)
    .def ("run_whole", &Mcdta::run_whole, py::arg ("verbose") = false)
    .def ("install_cc", &Mcdta::install_cc)
    .def ("install_cc_tree", &Mcdta::install_cc_tree)
    .def ("get_travel_stats", &Mcdta::get_travel_stats)
    .def ("print_emission_stats", &Mcdta::print_emission_stats)
    .def ("get_cur_loading_interval", &Mcdta::get_cur_loading_interval)
    .def_property_readonly ("links", &Mcdta::get_all_links, "IDs of all links.")
    .def ("print_simulation_results", &Mcdta::print_simulation_results)

    .def ("build_link_cost_map", &Mcdta::build_link_cost_map)
    .def ("get_link_queue_dissipated_time",
          &Mcdta::get_link_queue_dissipated_time)
    .def ("update_tdsp_tree", &Mcdta::update_tdsp_tree)
    .def ("get_lowest_cost_path", &Mcdta::get_lowest_cost_path)

    .def ("register_links", &Mcdta::register_links)
    .def ("register_paths", &Mcdta::register_paths)
    .def_property_readonly ("registered_links", &Mcdta::get_registered_links,
                            "IDs of previously registered links.")
    .def ("are_registered_links_in_registered_paths",
          &Mcdta::are_registered_links_in_registered_paths)
    .def ("generate_paths_to_cover_registered_links",
          &Mcdta::generate_paths_to_cover_registered_links)

    .def ("get_car_link_fftt", &Mcdta::get_car_link_fftt)
    .def ("get_truck_link_fftt", &Mcdta::get_truck_link_fftt)

    .def ("get_car_link_tt", &Mcdta::get_car_link_tt)
    .def ("get_car_link_tt_robust", &Mcdta::get_car_link_tt_robust)
    .def ("get_truck_link_tt", &Mcdta::get_truck_link_tt)
    .def ("get_truck_link_tt_robust", &Mcdta::get_truck_link_tt_robust)
    .def ("get_car_link_out_num", &Mcdta::get_car_link_out_num)
    .def ("get_truck_link_out_num", &Mcdta::get_truck_link_out_num)
    .def ("get_car_link_in_cc", &Mcdta::get_car_link_in_cc)
    .def ("get_truck_link_in_cc", &Mcdta::get_truck_link_in_cc)
    .def ("get_car_link_out_cc", &Mcdta::get_car_link_out_cc)
    .def ("get_truck_link_out_cc", &Mcdta::get_truck_link_out_cc)
    .def ("get_car_link_speed", &Mcdta::get_car_link_speed)
    .def ("get_truck_link_speed", &Mcdta::get_truck_link_speed)
    .def ("get_link_car_inflow", &Mcdta::get_link_car_inflow)
    .def ("get_link_truck_inflow", &Mcdta::get_link_truck_inflow)
    .def ("get_enroute_and_queue_veh_stats_agg",
          &Mcdta::get_enroute_and_queue_veh_stats_agg)
    .def ("get_queue_veh_each_link", &Mcdta::get_queue_veh_each_link)

    .def ("get_car_dar_matrix", &Mcdta::get_car_dar_matrix)
    .def ("get_truck_dar_matrix", &Mcdta::get_truck_dar_matrix)
    .def ("get_complete_car_dar_matrix", &Mcdta::get_complete_car_dar_matrix)
    .def ("get_complete_truck_dar_matrix",
          &Mcdta::get_complete_truck_dar_matrix)
    .def ("save_car_dar_matrix", &Mcdta::save_car_dar_matrix)
    .def ("save_truck_dar_matrix", &Mcdta::save_truck_dar_matrix)

    // For scenarios in McKees Rocks project:
    .def ("get_waiting_time_at_intersections",
          &Mcdta::get_waiting_time_at_intersections)
    .def ("get_waiting_time_at_intersections_car",
          &Mcdta::get_waiting_time_at_intersections_car)
    .def ("get_waiting_time_at_intersections_truck",
          &Mcdta::get_waiting_time_at_intersections_truck)
    .def ("get_link_spillback", &Mcdta::get_link_spillback)
    .def ("get_avg_link_on_path_tt_car", &Mcdta::get_avg_link_on_path_tt_car)
    .def ("get_avg_link_on_path_tt_truck",
          &Mcdta::get_avg_link_on_path_tt_truck)

    // with build_link_cost_map()
    .def ("get_path_tt_car", &Mcdta::get_path_tt_car)
    .def ("get_path_tt_truck", &Mcdta::get_path_tt_truck)
    .def ("get_registered_path_tt_car", &Mcdta::get_registered_path_tt_car)
    .def ("get_registered_path_tt_truck", &Mcdta::get_registered_path_tt_truck)

    .def ("get_car_ltg_matrix", &Mcdta::get_car_ltg_matrix)
    .def ("get_truck_ltg_matrix", &Mcdta::get_truck_ltg_matrix)

    .def ("get_complete_car_ltg_matrix", &Mcdta::get_complete_car_ltg_matrix)
    .def ("get_complete_truck_ltg_matrix",
          &Mcdta::get_complete_truck_ltg_matrix)

    .def ("delete_all_agents", &Mcdta::delete_all_agents);
}

Mcdta::Mcdta ()
{
  m_mcdta = nullptr;
  m_link_vec = std::vector<MNM_Dlink_Multiclass *> ();
  m_path_vec = std::vector<MNM_Path *> ();
  m_path_set = std::set<MNM_Path *> ();
  m_ID_path_mapping = std::unordered_map<TInt, MNM_Path *> ();

  m_link_tt_map = std::unordered_map<TInt, TFlt *> ();
  m_link_tt_map_truck = std::unordered_map<TInt, TFlt *> ();

  m_link_cost_map = std::unordered_map<TInt, TFlt *> ();
  m_link_cost_map_truck = std::unordered_map<TInt, TFlt *> ();

  m_link_congested_car = std::unordered_map<TInt, bool *> ();
  m_link_congested_truck = std::unordered_map<TInt, bool *> ();

  m_queue_dissipated_time_car = std::unordered_map<TInt, int *> ();
  m_queue_dissipated_time_truck = std::unordered_map<TInt, int *> ();

  m_tdsp_tree_map = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
}

Mcdta::~Mcdta ()
{
  if (m_mcdta != nullptr)
    {
      delete m_mcdta;
    }
  m_link_vec.clear ();
  m_path_vec.clear ();

  for (auto _tt_it : m_link_tt_map)
    {
      delete _tt_it.second;
    }
  m_link_tt_map.clear ();

  for (auto _tt_it : m_link_tt_map_truck)
    {
      delete _tt_it.second;
    }
  m_link_tt_map_truck.clear ();

  for (auto _cost_it : m_link_cost_map)
    {
      delete _cost_it.second;
    }
  m_link_cost_map.clear ();

  for (auto _cost_it : m_link_cost_map_truck)
    {
      delete _cost_it.second;
    }
  m_link_cost_map_truck.clear ();

  for (auto _it : m_link_congested_car)
    {
      delete _it.second;
    }
  m_link_congested_car.clear ();

  for (auto _it : m_link_congested_truck)
    {
      delete _it.second;
    }
  m_link_congested_truck.clear ();

  for (auto _it : m_queue_dissipated_time_car)
    {
      delete _it.second;
    }
  m_queue_dissipated_time_car.clear ();

  for (auto _it : m_queue_dissipated_time_truck)
    {
      delete _it.second;
    }
  m_queue_dissipated_time_truck.clear ();

  if (!m_tdsp_tree_map.empty ())
    {
      for (auto _it : m_tdsp_tree_map)
        {
          delete _it.second;
        }
      m_tdsp_tree_map.clear ();
    }
}

int
Mcdta::initialize (const std::string &folder)
{
  m_mcdta = new MNM_Dta_Multiclass (folder);
  m_mcdta->build_from_files ();
  m_mcdta->hook_up_node_and_link ();
  m_mcdta->is_ok ();
  IAssert (m_mcdta->m_config->get_string ("routing_type") == "Biclass_Hybrid"
           || m_mcdta->m_config->get_string ("routing_type")
                == "Biclass_Hybrid_ColumnGeneration"
           || m_mcdta->m_config->get_string ("routing_type") == "Hybrid"
           || m_mcdta->m_config->get_string ("routing_type") == "Adaptive");
  if (MNM_Routing_Fixed *_routing
      = dynamic_cast<MNM_Routing_Fixed *> (m_mcdta->m_routing))
    {
      MNM::get_ID_path_mapping (m_ID_path_mapping, _routing->m_path_table);
      return 0;
    }
  if (MNM_Routing_Hybrid *_routing
      = dynamic_cast<MNM_Routing_Hybrid *> (m_mcdta->m_routing))
    {
      // printf("start load ID path mapping\n");
      MNM::get_ID_path_mapping (m_ID_path_mapping,
                                _routing->m_routing_fixed->m_path_table);
      return 0;
      // printf("mapping size %d\n", m_ID_path_mapping.size());
    }
  if (MNM_Routing_Biclass_Hybrid *_routing
      = dynamic_cast<MNM_Routing_Biclass_Hybrid *> (m_mcdta->m_routing))
    {
      printf ("MNM_Routing_Biclass_Hybrid start load ID path mapping\n");
      MNM::get_ID_path_mapping (m_ID_path_mapping,
                                _routing->m_routing_fixed_car->m_path_table);
      printf ("MNM_Routing_Biclass_Hybrid mapping size %d\n",
              (int) m_ID_path_mapping.size ());
      return 0;
    }
  // TODO: Check if the other routing types are really unsupported -- they work
  // fine in our test cases at least.
  return -1;
}

bool
Mcdta::check_input_files ()
{
  return m_mcdta->is_ok ();
}

int
Mcdta::generate_shortest_pathsets (const std::string &folder, int max_iter,
                                   double vot, double mid_scale,
                                   double heavy_scale, double min_path_tt)
{
  // m_mcdta = new MNM_Dta_Multiclass(folder);
  // m_mcdta -> build_from_files();
  // m_mcdta -> hook_up_node_and_link();
  // m_mcdta -> is_ok();

  IAssert (m_mcdta != nullptr);
  // this is based on travel time
  Path_Table *_driving_path_table
    = MNM::build_pathset_multiclass (m_mcdta->m_graph, m_mcdta->m_od_factory,
                                     m_mcdta->m_link_factory, min_path_tt,
                                     max_iter, vot, mid_scale, heavy_scale,
                                     2
                                       * m_mcdta->m_config->get_int (
                                         "max_interval"));
  printf ("driving pathset generated\n");
  MNM::save_driving_path_table (folder, _driving_path_table, "path_table",
                                "path_table_buffer", true);
  printf ("driving pathset saved\n");

  for (auto _it : *_driving_path_table)
    {
      for (auto _it_it : *_it.second)
        {
          delete _it_it.second;
        }
      _it.second->clear ();
      delete _it.second;
    }
  _driving_path_table->clear ();
  delete _driving_path_table;

  return 0;
}

int
Mcdta::install_cc ()
{
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      m_link_vec[i]->install_cumulative_curve_multiclass ();
    }
  return 0;
}

int
Mcdta::install_cc_tree ()
{
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      m_link_vec[i]->install_cumulative_curve_tree_multiclass ();
    }
  return 0;
}

int
Mcdta::run_whole (bool verbose)
{
  m_mcdta->pre_loading ();
  m_mcdta->loading (verbose);
  return 0;
}

py::array_t<int>
Mcdta::get_all_links ()
{
  if (!m_mcdta)
    throw std::runtime_error ("MCDTA uninitialized");
  auto &link_map = m_mcdta->m_link_factory->m_link_map;
  auto results = py::array_t<int> (link_map.size ());
  auto results_buf = results.request ();
  int *results_ptr = static_cast<int *> (results_buf.ptr);
  int idx = 0;
  for (auto link : m_mcdta->m_link_factory->m_link_map)
    results_ptr[idx++] = link.first;
  results.attr ("sort") ();
  return results;
}

int
Mcdta::register_links (py::array_t<int> links)
{
  if (m_link_vec.size () > 0)
    {
      printf ("Warning, Mcdta::register_links, link exists\n");
      m_link_vec.clear ();
    }
  auto links_buf = links.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Number of dimensions must be one");
    }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mcdta->m_link_factory->get_link (TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          if (std::find (m_link_vec.begin (), m_link_vec.end (), _link)
              != m_link_vec.end ())
            {
              throw std::runtime_error (
                "Error, Mcdta::register_links, link does not exist");
            }
          else
            {
              m_link_vec.push_back (_mclink);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mcdta::register_links: link type is not multiclass");
        }
    }
  return 0;
}

py::array_t<int>
Mcdta::get_registered_links ()
{
  auto results = py::array_t<int> (m_link_vec.size ());
  auto results_buf = results.request ();
  int *results_ptr = static_cast<int *> (results_buf.ptr);
  for (int idx = 0; idx < m_link_vec.size (); idx++)
    results_ptr[idx] = (int) m_link_vec[idx]->m_link_ID;
  return results;
}

int
Mcdta::get_cur_loading_interval ()
{
  return m_mcdta->m_current_loading_interval ();
}

std::string
Mcdta::print_emission_stats ()
{
  return m_mcdta->m_emission->output ();
}

int
Mcdta::print_simulation_results (const std::string &folder, int cong_frequency)
{
  // cong_frequency: number of 5-s interval, 180 means 15 minutes
  bool output_link_cong;
  if (cong_frequency > 0)
    {
      output_link_cong
        = true; // if true output link congestion level every cong_frequency
    }
  else
    {
      output_link_cong = false;
    }

  MNM_Dlink *_link;
  MNM_Dlink_Multiclass *_link_m;
  std::string _str1;
  TInt _current_inter = m_mcdta->m_current_loading_interval;
  std::ofstream _vis_file2;
  if (output_link_cong)
    {
      _vis_file2.open (folder + "/driving_link_cong_raw.txt",
                       std::ofstream::out);
      if (!_vis_file2.is_open ())
        {
          throw std::runtime_error ("failed to open _vis_file2");
        }

      _str1 = "timestamp (intervals), driving_link_ID, car_inflow, "
              "truck_inflow, car_tt (s), truck_tt (s), car_fftt (s), "
              "truck_fftt (s), car_speed (mph), truck_speed (mph)\n";
      _vis_file2 << _str1;

      TInt _iter = 0;
      while (_iter + cong_frequency <= _current_inter)
        {
          if (_iter % cong_frequency == 0 || _iter == _current_inter - 1)
            {
              printf ("Current loading interval: %d\n", int (_iter));
              for (auto _link_it : m_mcdta->m_link_factory->m_link_map)
                {
                  _link = _link_it.second;
                  _link_m = dynamic_cast<MNM_Dlink_Multiclass *> (_link);
                  _str1 = std::to_string (int (_iter)) + " ";
                  _str1 += std::to_string (_link->m_link_ID ()) + " ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::
                               get_link_inflow_car (_link_m, _iter,
                                                    _iter + cong_frequency))
                           + " ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::
                               get_link_inflow_truck (_link_m, _iter,
                                                      _iter + cong_frequency))
                           + " ";
                  // _str1 +=
                  // std::to_string(MNM_DTA_GRADIENT::get_travel_time_car(_link_m,
                  // TFlt(_iter + 1), m_mcdta -> m_unit_time, m_mcdta ->
                  // m_current_loading_interval) * m_mcdta -> m_unit_time) + "
                  // "; _str1 +=
                  // std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m,
                  // TFlt(_iter + 1), m_mcdta -> m_unit_time, m_mcdta ->
                  // m_current_loading_interval) * m_mcdta -> m_unit_time) + "
                  // ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_car_robust (
                               _link_m, TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_mcdta->m_unit_time,
                               m_mcdta->m_current_loading_interval)
                             * m_mcdta->m_unit_time)
                           + " "; // seconds
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_truck_robust (
                               _link_m, TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_mcdta->m_unit_time,
                               m_mcdta->m_current_loading_interval)
                             * m_mcdta->m_unit_time)
                           + " ";
                  _str1 += std::to_string (_link_m->get_link_freeflow_tt_car ())
                           + " ";
                  _str1
                    += std::to_string (_link_m->get_link_freeflow_tt_truck ())
                       + " ";
                  // _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time_car(_link_m,
                  // TFlt(_iter + 1), m_mcdta -> m_unit_time, m_mcdta ->
                  // m_current_loading_interval) * m_mcdta -> m_unit_time) *
                  // 3600 / 1600) + " "; _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m,
                  // TFlt(_iter + 1), m_mcdta -> m_unit_time, m_mcdta ->
                  // m_current_loading_interval) * m_mcdta -> m_unit_time) *
                  // 3600 / 1600) + "\n";
                  _str1 += std::to_string (
                             _link_m->m_length
                             / (MNM_DTA_GRADIENT::get_travel_time_car_robust (
                                  _link_m, TFlt (_iter + 1),
                                  TFlt (_iter + cong_frequency + 1),
                                  m_mcdta->m_unit_time,
                                  m_mcdta->m_current_loading_interval)
                                * m_mcdta->m_unit_time)
                             * 3600 / 1600)
                           + " "; // mph
                  _str1 += std::to_string (
                             _link_m->m_length
                             / (MNM_DTA_GRADIENT::get_travel_time_truck_robust (
                                  _link_m, TFlt (_iter + 1),
                                  TFlt (_iter + cong_frequency + 1),
                                  m_mcdta->m_unit_time,
                                  m_mcdta->m_current_loading_interval)
                                * m_mcdta->m_unit_time)
                             * 3600 / 1600)
                           + "\n";
                  _vis_file2 << _str1;
                }
            }
          _iter += 1;
        }

      // // save cc of some links
      // _str = "\n\n **************************** driving link cc
      // ****************************"; for (auto _link_it :
      // m_mcdta->m_link_factory->m_link_map) {
      //     _link = _link_it.second;
      //     if (_link->m_link_ID() == 4) {
      //         _link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
      //         _str += "\nlink_ID: " + std::to_string(_link->m_link_ID());
      //         _str +="\nm_N_in_car: \n";
      //         _str += _link_m->m_N_in_car->to_string();
      //         _str +="\nm_N_out_car: \n";
      //         _str += _link_m->m_N_out_car->to_string();
      //         _str +="\nm_N_in_truck: \n";
      //         _str += _link_m->m_N_in_truck->to_string();
      //         _str +="\nm_N_out_truck: \n";
      //         _str += _link_m->m_N_out_truck->to_string();
      //         _vis_file2 << _str;
      //     }
      // }

      if (_vis_file2.is_open ())
        _vis_file2.close ();
    }
  return 0;
}

py::array_t<double>
Mcdta::get_travel_stats ()
{
  // finished
  TInt _count_car = 0, _count_truck = 0;
  TFlt _tot_tt_car = 0.0, _tot_tt_truck = 0.0;

  auto *_veh_factory
    = dynamic_cast<MNM_Veh_Factory_Multiclass *> (m_mcdta->m_veh_factory);
  _count_car = _veh_factory->m_finished_car;
  _count_truck = _veh_factory->m_finished_truck;
  _tot_tt_car = _veh_factory->m_total_time_car * m_mcdta->m_unit_time / 3600.0;
  _tot_tt_truck
    = _veh_factory->m_total_time_truck * m_mcdta->m_unit_time / 3600.0;

  // unfinished
  MNM_Veh_Multiclass *_veh;
  int _end_time = get_cur_loading_interval ();
  for (auto _map_it : m_mcdta->m_veh_factory->m_veh_map)
    {
      _veh = dynamic_cast<MNM_Veh_Multiclass *> (_map_it.second);
      IAssert (_veh->m_finish_time < 0);
      if (_veh->m_class == 0)
        {
          _count_car += 1;
          _tot_tt_car
            += (_end_time - _veh->m_start_time) * m_mcdta->m_unit_time / 3600.0;
        }
      else
        {
          _count_truck += 1;
          _tot_tt_truck
            += (_end_time - _veh->m_start_time) * m_mcdta->m_unit_time / 3600.0;
        }
    }

  // // for vehicles not deleted
  // MNM_Veh_Multiclass* _veh;
  // int _end_time = get_cur_loading_interval();
  // for (auto _map_it : m_mcdta -> m_veh_factory -> m_veh_map){
  //     _veh = dynamic_cast<MNM_Veh_Multiclass *>(_map_it.second);
  //     if (_veh -> m_class == 0){
  //         _count_car += 1;
  //         if (_veh -> m_finish_time > 0) {
  //             _tot_tt_car += (_veh -> m_finish_time - _veh -> m_start_time) *
  //             m_mcdta -> m_unit_time / 3600.0;
  //         }
  //         else {
  //             _tot_tt_car += (_end_time - _veh -> m_start_time) * m_mcdta ->
  //             m_unit_time / 3600.0;
  //         }
  //     }
  //     else {
  //         _count_truck += 1;
  //         if (_veh -> m_finish_time > 0) {
  //             _tot_tt_truck += (_veh -> m_finish_time - _veh -> m_start_time)
  //             * m_mcdta -> m_unit_time / 3600.0;
  //         }
  //         else {
  //             _tot_tt_truck += (_end_time - _veh -> m_start_time) * m_mcdta
  //             -> m_unit_time / 3600.0;
  //         }
  //     }
  // }

  // printf("\n\nTotal car: %d, Total truck: %d, Total car tt: %.2f hours, Total
  // truck tt: %.2f hours\n\n",
  //        int(_count_car/m_mcdta -> m_flow_scalar), int(_count_truck/m_mcdta
  //        -> m_flow_scalar), float(_tot_tt_car/m_mcdta -> m_flow_scalar),
  //        float(_tot_tt_truck/m_mcdta -> m_flow_scalar));
  // m_mcdta -> m_emission -> output();

  // for all released vehicles
  int new_shape[1] = { 4 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  result_ptr[0] = _count_car / m_mcdta->m_flow_scalar;
  result_ptr[1] = _count_truck / m_mcdta->m_flow_scalar;
  result_ptr[2] = _tot_tt_car / m_mcdta->m_flow_scalar;
  result_ptr[3] = _tot_tt_truck / m_mcdta->m_flow_scalar;

  return result;
}

int
Mcdta::build_link_cost_map (bool with_congestion_indicator)
{
  MNM_Dlink_Multiclass *_link;
  // TODO: what if not hybrid routing, better way to get vot
  TFlt _vot = dynamic_cast<MNM_Routing_Biclass_Hybrid *> (m_mcdta->m_routing)
                ->m_routing_adaptive->m_vot
              * m_mcdta->m_unit_time; // money / second -> money / interval
  for (auto _link_it : m_mcdta->m_link_factory->m_link_map)
    {
      // #pragma omp task
      _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);

      if (m_link_tt_map.find (_link_it.first) == m_link_tt_map.end ())
        {
          m_link_tt_map[_link_it.first] = new TFlt[get_cur_loading_interval ()];
        }
      if (m_link_cost_map.find (_link_it.first) == m_link_cost_map.end ())
        {
          m_link_cost_map[_link_it.first]
            = new TFlt[get_cur_loading_interval ()];
        }
      if (m_link_tt_map_truck.find (_link_it.first)
          == m_link_tt_map_truck.end ())
        {
          m_link_tt_map_truck[_link_it.first]
            = new TFlt[get_cur_loading_interval ()];
        }
      if (m_link_cost_map_truck.find (_link_it.first)
          == m_link_cost_map_truck.end ())
        {
          m_link_cost_map_truck[_link_it.first]
            = new TFlt[get_cur_loading_interval ()];
        }

      if (with_congestion_indicator)
        {
          if (m_link_congested_car.find (_link_it.first)
              == m_link_congested_car.end ())
            {
              m_link_congested_car[_link_it.first]
                = new bool[get_cur_loading_interval ()];
            }
          if (m_link_congested_truck.find (_link_it.first)
              == m_link_congested_truck.end ())
            {
              m_link_congested_truck[_link_it.first]
                = new bool[get_cur_loading_interval ()];
            }
        }

      std::cout << "********************** build_link_cost_map link "
                << _link->m_link_ID () << " **********************\n";
      for (int i = 0; i < get_cur_loading_interval (); i++)
        {
          m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time_car (_link, TFlt (i + 1), m_mcdta->m_unit_time,
                                 get_cur_loading_interval ()); // intervals

          m_link_cost_map[_link_it.first][i]
            = _vot * m_link_tt_map[_link_it.first][i] + _link->m_toll_car;

          m_link_tt_map_truck[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time_truck (_link, TFlt (i + 1), m_mcdta->m_unit_time,
                                   get_cur_loading_interval ()); // intervals

          m_link_cost_map_truck[_link_it.first][i]
            = _vot * m_link_tt_map_truck[_link_it.first][i]
              + _link->m_toll_truck;

          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_link_tt_map[_link_it.first][i] << "\n"; std::cout <<
          // "car in" << "\n"; std::cout << _link -> m_N_in_car -> to_string()
          // << "\n"; std::cout << "car out" << "\n"; std::cout << _link ->
          // m_N_out_car -> to_string() << "\n"; std::cout << "truck in" <<
          // "\n"; std::cout << _link -> m_N_in_truck -> to_string() << "\n";
          // std::cout << "truck out" << "\n";
          // std::cout << _link -> m_N_out_truck -> to_string() << "\n";
          if (with_congestion_indicator)
            {
              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " << m_link_tt_map[_link_it.first][i]
              // << ", fftt: " << _link -> get_link_freeflow_tt_car() /
              // m_unit_time << "\n"; std::cout << "truck, interval: " << i <<
              // ", link: " << _link_it.first << ", tt: " <<
              // m_link_tt_map_truck[_link_it.first][i] << ", fftt: " << _link
              // -> get_link_freeflow_tt_truck() / m_unit_time << "\n";
              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " << m_link_tt_map[_link_it.first][i]
              // << ", fftt: " << _link -> get_link_freeflow_tt_loading_car() <<
              // "\n"; std::cout << "truck, interval: " << i << ", link: " <<
              // _link_it.first << ", tt: " <<
              // m_link_tt_map_truck[_link_it.first][i] << ", fftt: " << _link
              // -> get_link_freeflow_tt_loading_truck() << "\n";

              m_link_congested_car[_link_it.first][i]
                = m_link_tt_map[_link_it.first][i]
                  > _link->get_link_freeflow_tt_loading_car ();

              m_link_congested_truck[_link_it.first][i]
                = m_link_tt_map_truck[_link_it.first][i]
                  > _link->get_link_freeflow_tt_loading_truck ();

              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", congested?: " <<
              // m_link_congested_car[_link_it.first][i] << "\n"; std::cout <<
              // "truck, interval: " << i << ", link: " << _link_it.first << ",
              // congested?: " << m_link_congested_car[_link_it.first][i] <<
              // "\n";
            }
        }
    }
  return 0;
}

int
Mcdta::get_link_queue_dissipated_time ()
{
  // suppose m_link_congested_car and m_link_congested_truck are constructed
  // already in build_link_cost_map()
  MNM_Dlink_Multiclass *_link;
  int _total_loading_inter = get_cur_loading_interval ();
  IAssert (_total_loading_inter > 0);

  bool _flg;
  std::cout << "\n********************** Begin get_link_queue_dissipated_time "
               "**********************\n";
  for (auto _link_it : m_mcdta->m_link_factory->m_link_map)
    {
      if (m_queue_dissipated_time_car.find (_link_it.first)
          == m_queue_dissipated_time_car.end ())
        {
          m_queue_dissipated_time_car[_link_it.first]
            = new int[_total_loading_inter];
        }
      if (m_queue_dissipated_time_truck.find (_link_it.first)
          == m_queue_dissipated_time_truck.end ())
        {
          m_queue_dissipated_time_truck[_link_it.first]
            = new int[_total_loading_inter];
        }
      // std::cout << "********************** get_link_queue_dissipated_time
      // link " << _link_it.first << " **********************\n";
      for (int i = 0; i < _total_loading_inter; i++)
        {
          // ************************** car **************************
          if (m_link_congested_car[_link_it.first][i])
            {
              if (i == _total_loading_inter - 1)
                {
                  m_queue_dissipated_time_car[_link_it.first][i]
                    = _total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < _total_loading_inter; k++)
                    {
                      if (m_link_congested_car[_link_it.first][k - 1]
                          && !m_link_congested_car[_link_it.first][k])
                        {
                          m_queue_dissipated_time_car[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time_car[_link_it.first][i]
                        = _total_loading_inter;
                    }
                }
            }
          else
            {
              _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
              if (MNM_Ults::
                    approximate_equal (m_link_tt_map[_link_it.first][i],
                                       (float) _link
                                         ->get_link_freeflow_tt_loading_car ()))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                      != nullptr)
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
                      TFlt _outflow_rate
                        = MNM_DTA_GRADIENT::get_departure_cc_slope_car (
                          _link,
                          TFlt (
                            i
                            + (int) _link->get_link_freeflow_tt_loading_car ()),
                          TFlt (
                            i + (int) _link->get_link_freeflow_tt_loading_car ()
                            + 1)); // veh / 5s
                      TFlt _cap
                        = dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                            ->m_cell_array.back ()
                            ->m_flow_cap_car
                          * m_mcdta->m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * m_mcdta
                                                             ->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * m_mcdta
                                                             ->m_flow_scalar)))
                        {
                          if (i == _total_loading_inter - 1)
                            {
                              m_queue_dissipated_time_car[_link_it.first][i]
                                = _total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < _total_loading_inter; k++)
                                {
                                  if (m_link_congested_car[_link_it.first]
                                                          [k - 1]
                                      && !m_link_congested_car[_link_it.first]
                                                              [k])
                                    {
                                      m_queue_dissipated_time_car[_link_it
                                                                    .first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time_car[_link_it.first][i]
                                    = _total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time_car[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link)
                           != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time_car[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "Mcdta::get_link_queue_dissipated_time, Link type "
                        "not implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time_car[_link_it.first][i] = i;
                  throw std::runtime_error (
                    "Mcdta::get_link_queue_dissipated_time, Link travel "
                    "time less than fftt");
                }
              // m_queue_dissipated_time_car[_link_it.first][i] = i;
            }

          // ************************** truck **************************
          if (m_link_congested_truck[_link_it.first][i])
            {
              if (i == _total_loading_inter - 1)
                {
                  m_queue_dissipated_time_truck[_link_it.first][i]
                    = _total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < _total_loading_inter; k++)
                    {
                      if (m_link_congested_truck[_link_it.first][k - 1]
                          && !m_link_congested_truck[_link_it.first][k])
                        {
                          m_queue_dissipated_time_truck[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time_truck[_link_it.first][i]
                        = _total_loading_inter;
                    }
                }
            }
          else
            {
              _link = dynamic_cast<MNM_Dlink_Multiclass *> (_link_it.second);
              if (
                MNM_Ults::
                  approximate_equal (m_link_tt_map_truck[_link_it.first][i],
                                     (float) _link
                                       ->get_link_freeflow_tt_loading_truck ()))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                      != nullptr)
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
                      TFlt _outflow_rate
                        = MNM_DTA_GRADIENT::get_departure_cc_slope_truck (
                          _link,
                          TFlt (i
                                + (int) _link
                                    ->get_link_freeflow_tt_loading_truck ()),
                          TFlt (
                            i
                            + (int) _link->get_link_freeflow_tt_loading_truck ()
                            + 1)); // veh / 5s
                      TFlt _cap
                        = dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                            ->m_cell_array.back ()
                            ->m_flow_cap_truck
                          * m_mcdta->m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * m_mcdta
                                                             ->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * m_mcdta
                                                             ->m_flow_scalar)))
                        {
                          if (i == _total_loading_inter - 1)
                            {
                              m_queue_dissipated_time_truck[_link_it.first][i]
                                = _total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < _total_loading_inter; k++)
                                {
                                  if (m_link_congested_truck[_link_it.first]
                                                            [k - 1]
                                      && !m_link_congested_truck[_link_it.first]
                                                                [k])
                                    {
                                      m_queue_dissipated_time_truck[_link_it
                                                                      .first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time_truck[_link_it.first]
                                                               [i]
                                    = _total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time_truck[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link)
                           != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time_truck[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "Mcdta::get_link_queue_dissipated_time, Link type "
                        "not implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time_truck[_link_it.first][i] = i;
                  throw std::runtime_error (
                    "Mcdta::get_link_queue_dissipated_time, Link travel "
                    "time less than fftt");
                }
              // m_queue_dissipated_time_truck[_link_it.first][i] = i;
            }
        }
    }
  std::cout << "********************** End get_link_queue_dissipated_time "
               "**********************\n";
  return 0;
}

int
Mcdta::update_tdsp_tree ()
{
  // build_link_cost_map() should be called first before this function
  if (!m_tdsp_tree_map.empty ())
    {
      for (auto _it : m_tdsp_tree_map)
        {
          delete _it.second;
        }
      m_tdsp_tree_map.clear ();
    }

  MNM_Destination *_dest;
  TInt _dest_node_ID;
  MNM_TDSP_Tree *_tdsp_tree;

  for (auto _d_it : m_mcdta->m_od_factory->m_destination_map)
    {
      _dest = _d_it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;
      _tdsp_tree
        = new MNM_TDSP_Tree (_dest_node_ID, m_mcdta->m_graph,
                             m_mcdta->m_config->get_int ("assign_frq")
                               * m_mcdta->m_config->get_int ("max_interval"));
      _tdsp_tree->initialize ();
      _tdsp_tree->update_tree (m_link_cost_map, m_link_tt_map);
      m_tdsp_tree_map.insert (
        std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
      _tdsp_tree = nullptr;
      IAssert (m_tdsp_tree_map.find (_dest_node_ID)->second != nullptr);
    }
  return 0;
}

py::array_t<int>
Mcdta::get_lowest_cost_path (int start_interval, int o_node_ID, int d_node_ID)
{
  // get lowest cost path departing at start_interval
  // input interval is in the loading intervals
  IAssert (start_interval + m_mcdta->m_config->get_int ("assign_frq")
           <= m_mcdta->m_config->get_int ("assign_frq")
                * m_mcdta->m_config->get_int (
                  "max_interval")); // tdsp_tree -> m_max_interval =
                                    // total_loading_interval

  // IAssert(start_interval < m_mcdta -> m_config -> get_int("max_interval") *
  // m_mcdta -> m_config -> get_int("assign_frq"));

  TFlt _cur_best_cost = TFlt (std::numeric_limits<double>::infinity ());
  TInt _cur_best_time = -1;
  TInt _cur_best_assign_col = -1;
  TFlt _tmp_tt, _tmp_cost;
  TInt _assign_inter;
  MNM_Path *_path;
  MNM_Path *_cur_best_path = nullptr;
  int _num_col;
  bool _exist;
  Path_Table *_path_table;
  MNM_Pathset *_path_set;
  MNM_TDSP_Tree *_tdsp_tree;

  _path_table = dynamic_cast<MNM_Routing_Biclass_Hybrid *> (m_mcdta->m_routing)
                  ->m_routing_fixed_car->m_path_table;
  _path_set = nullptr;
  if (_path_table->find (o_node_ID) != _path_table->end ()
      && _path_table->find (o_node_ID)->second->find (d_node_ID)
           != _path_table->find (o_node_ID)->second->end ())
    {
      _path_set
        = _path_table->find (o_node_ID)->second->find (d_node_ID)->second;
    }
  IAssert (_path_set != nullptr);

  _tdsp_tree = m_tdsp_tree_map.find (d_node_ID)->second;

  for (int i = start_interval; i < start_interval + 1; ++i)
    { // tdsp_tree -> m_max_interval = total_loading_interval

      _path = new MNM_Path ();
      _tmp_tt = _tdsp_tree->get_tdsp (o_node_ID, i, m_link_tt_map, _path);
      IAssert (_tmp_tt > 0);
      _path->eliminate_cycles ();

      _tmp_cost
        = _tdsp_tree
            ->m_dist[o_node_ID][i < (int) _tdsp_tree->m_max_interval
                                  ? i
                                  : (int) _tdsp_tree->m_max_interval - 1];

      _assign_inter = (int) i / m_mcdta->m_config->get_int ("assign_frq");
      if (_assign_inter >= m_mcdta->m_config->get_int ("assign_frq")
                             * m_mcdta->m_config->get_int ("max_interval"))
        _assign_inter = m_mcdta->m_config->get_int ("assign_frq")
                          * m_mcdta->m_config->get_int ("max_interval")
                        - 1;

      if (_tmp_cost < _cur_best_cost)
        {
          _cur_best_cost = _tmp_cost;
          _cur_best_time = i;
          _cur_best_assign_col = _assign_inter;
          if (_cur_best_path != nullptr)
            delete _cur_best_path;
          _cur_best_path = _path;
        }
      else
        {
          delete _path;
        }
    }
  IAssert (_cur_best_time >= 0 && _cur_best_assign_col >= 0);
  IAssert (_cur_best_path != nullptr);

  _exist = _path_set->is_in (_cur_best_path);
  _num_col = (int) _cur_best_path->m_node_vec.size ();

  int new_shape[2] = {
    4, _num_col
  }; // row: _exist, cost, driving path node vec, driving path link vec
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  for (int i = 0; i < _num_col; ++i)
    {
      if (i == 0)
        {
          result_ptr[i + _num_col * 0] = (int) _exist;
          result_ptr[i + _num_col * 1] = (int) _cur_best_cost;
        }
      else
        {
          result_ptr[i + _num_col * 0] = -1;
          result_ptr[i + _num_col * 1] = -1;
        }
      if (i < _num_col - 1)
        {
          result_ptr[i + _num_col * 3] = _path->m_link_vec[i];
        }
      else
        {
          result_ptr[i + _num_col * 3] = -1;
        }
      result_ptr[i + _num_col * 2] = _path->m_node_vec[i];
    }

  return result;
}

py::array_t<double>
Mcdta::get_waiting_time_at_intersections ()
{
  int new_shape[1] = { (int) m_link_vec.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection (
          m_link_vec[i]) (); // seconds
    }

  return result;
}

py::array_t<double>
Mcdta::get_waiting_time_at_intersections_car ()
{
  int new_shape[1] = { (int) m_link_vec.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection_car (
          m_link_vec[i]) (); // seconds
    }

  return result;
}

py::array_t<double>
Mcdta::get_waiting_time_at_intersections_truck ()
{
  int new_shape[1] = { (int) m_link_vec.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection_truck (
          m_link_vec[i]) (); // seconds
    }

  return result;
}

py::array_t<int>
Mcdta::get_link_spillback ()
{
  int new_shape[1] = { (int) m_link_vec.size () };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      result_ptr[i] = MNM_DTA_GRADIENT::get_is_spillback (m_link_vec[i]) ();
    }

  return result;
}

py::array_t<double>
Mcdta::get_avg_link_on_path_tt_car (py::array_t<int> link_IDs,
                                    py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();
  int num_link = links_buf.shape[0];

  int new_shape[1] = { num_link };
  // the output is an array with size of num_link, each element being the
  // average link travel time
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  double *start_ptr = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mcdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          double avg_tt = 0;
          for (int t = 0; t < num_int; ++t)
            {
              if (start_ptr[t] >= get_cur_loading_interval ())
                {
                  throw std::runtime_error (
                    "Error, Mcdta::get_avg_link_on_path_tt_car, input "
                    "start intervals exceeds the total loading intervals - 1");
                }
              double _tmp
                = MNM_DTA_GRADIENT::
                    get_travel_time_car (_mclink, TFlt (start_ptr[t] + 1),
                                         m_mcdta->m_unit_time,
                                         m_mcdta->m_current_loading_interval) ()
                  * m_mcdta->m_unit_time;
              if (_tmp
                  > TT_UPPER_BOUND * (_mclink->m_length / _mclink->m_ffs_car))
                {
                  _tmp
                    = TT_UPPER_BOUND * _mclink->m_length / _mclink->m_ffs_car;
                }
              avg_tt += _tmp; // seconds
            }
          avg_tt /= num_int;
          result_ptr[i] = avg_tt;
        }
      else
        {
          throw std::runtime_error ("Mcdta::get_avg_link_on_path_tt_car: "
                                    "link type is not multiclass");
        }
    }

  return result;
}

py::array_t<double>
Mcdta::get_avg_link_on_path_tt_truck (py::array_t<int> link_IDs,
                                      py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();
  int num_link = links_buf.shape[0];

  int new_shape[1] = { num_link };
  // the output is an array with size of num_link, each element being the
  // average link travel time
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  double *start_ptr = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mcdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          double avg_tt = 0;
          for (int t = 0; t < num_int; ++t)
            {
              if (start_ptr[t] >= get_cur_loading_interval ())
                {
                  throw std::runtime_error (
                    "Error, Mcdta::get_avg_link_on_path_tt_truck, input "
                    "start intervals exceeds the total loading intervals - 1");
                }
              double _tmp
                = MNM_DTA_GRADIENT::
                    get_travel_time_truck (_mclink, TFlt (start_ptr[t] + 1),
                                           m_mcdta->m_unit_time,
                                           m_mcdta
                                             ->m_current_loading_interval) ()
                  * m_mcdta->m_unit_time;
              if (_tmp
                  > TT_UPPER_BOUND * (_mclink->m_length / _mclink->m_ffs_truck))
                {
                  _tmp
                    = TT_UPPER_BOUND * _mclink->m_length / _mclink->m_ffs_truck;
                }
              avg_tt += _tmp; // seconds
            }
          avg_tt /= num_int;
          result_ptr[i] = avg_tt;
        }
      else
        {
          throw std::runtime_error ("Mcdta::get_avg_link_on_path_tt_truck: "
                                    "link type is not multiclass");
        }
    }

  return result;
}

py::array_t<double>
Mcdta::get_path_tt_car (py::array_t<int> link_IDs,
                        py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();

  int new_shape[2] = { 2, num_int };
  // the output is an array with size of 2 * num_int
  // in first row, each element being the average path travel time departing at
  // start_interval, in second row, each element being the average path travel
  // cost departing at start_interval,
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  int *start_ptr = (int *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  MNM_Path *_path = new MNM_Path ();
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mcdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          _path->m_link_vec.push_back (links_ptr[i]);
          _path->m_node_vec.push_back (_link->m_from_node->m_node_ID);
          if (i == links_buf.shape[0] - 1)
            {
              _path->m_node_vec.push_back (_link->m_to_node->m_node_ID);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mcdta::get_path_tt_car: link type is not multiclass");
        }
    }

  for (int t = 0; t < num_int; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_path_tt_car, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      // MNM_DTA_GRADIENT::get_path_travel_time will process link travel time
      // and the start_time for cc result_ptr[t] =
      // MNM_DTA_GRADIENT::get_path_travel_time_car(_path, TFlt(start_ptr[t]),
      // m_mcdta -> m_link_factory, m_mcdta -> m_unit_time,
      // get_cur_loading_interval()) * m_mcdta -> m_unit_time;  // seconds
      // assume build_link_cost_map() is invoked before
      result_ptr[t]
        = MNM_DTA_GRADIENT::
            get_path_travel_time_car (_path, TFlt (start_ptr[t]), m_link_tt_map,
                                      get_cur_loading_interval ())
          * m_mcdta->m_unit_time; // seconds
      result_ptr[t + num_int]
        = MNM_DTA_GRADIENT::get_path_travel_cost (_path, TFlt (start_ptr[t]),
                                                  m_link_tt_map,
                                                  m_link_cost_map,
                                                  get_cur_loading_interval ());
    }
  delete _path;
  return result;
}

py::array_t<double>
Mcdta::get_path_tt_truck (py::array_t<int> link_IDs,
                          py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();

  int new_shape[2] = { 2, num_int };
  // the output is an array with size of 2 * num_int
  // in first row, each element being the average path travel time departing at
  // start_interval, in second row, each element being the average path travel
  // cost departing at start_interval,
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  int *start_ptr = (int *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  MNM_Path *_path = new MNM_Path ();
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mcdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          _path->m_link_vec.push_back (links_ptr[i]);
          _path->m_node_vec.push_back (_link->m_from_node->m_node_ID);
          if (i == links_buf.shape[0] - 1)
            {
              _path->m_node_vec.push_back (_link->m_to_node->m_node_ID);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mcdta::get_path_tt_truck: link type is not multiclass");
        }
    }

  for (int t = 0; t < num_int; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_path_tt_truck, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      // MNM_DTA_GRADIENT::get_path_travel_time will process link travel time
      // and the start_time for cc result_ptr[t] =
      // MNM_DTA_GRADIENT::get_path_travel_time_truck(_path, TFlt(start_ptr[t]),
      // m_mcdta -> m_link_factory, m_mcdta -> m_unit_time,
      // get_cur_loading_interval()) * m_mcdta -> m_unit_time;  // seconds
      // assume build_link_cost_map() is invoked before
      result_ptr[t] = MNM_DTA_GRADIENT::
                        get_path_travel_time_truck (_path, TFlt (start_ptr[t]),
                                                    m_link_tt_map_truck,
                                                    get_cur_loading_interval ())
                      * m_mcdta->m_unit_time; // seconds
      result_ptr[t + num_int]
        = MNM_DTA_GRADIENT::get_path_travel_cost (_path, TFlt (start_ptr[t]),
                                                  m_link_tt_map_truck,
                                                  m_link_cost_map_truck,
                                                  get_cur_loading_interval ());
    }
  delete _path;
  return result;
}

py::array_t<double>
Mcdta::get_registered_path_tt_car (py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mcdta::get_registered_path_tt_car, "
                                "input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec.size (), l };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_registered_path_tt_car, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec.size (); ++i)
        {
          // MNM_DTA_GRADIENT::get_path_travel_time will process link travel
          // time and the start_time for cc result_ptr[i * l + t] =
          // MNM_DTA_GRADIENT::get_path_travel_time_car(m_path_vec[i],
          // TFlt(start_ptr[t]), m_mcdta -> m_link_factory, m_mcdta ->
          // m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time;
          // // seconds assume build_link_cost_map() is invoked before
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::
                get_path_travel_time_car (m_path_vec[i], TFlt (start_ptr[t]),
                                          m_link_tt_map,
                                          get_cur_loading_interval ())
              * m_mcdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_registered_path_tt_truck (py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_registered_path_tt_truck, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec.size (), l };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_registered_path_tt_truck, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec.size (); ++i)
        {
          // MNM_DTA_GRADIENT::get_path_travel_time will process link travel
          // time and the start_time for cc result_ptr[i * l + t] =
          // MNM_DTA_GRADIENT::get_path_travel_time_truck(m_path_vec[i],
          // TFlt(start_ptr[t]), m_mcdta -> m_link_factory, m_mcdta ->
          // m_unit_time, get_cur_loading_interval()) * m_mcdta -> m_unit_time;
          // // seconds assume build_link_cost_map() is invoked before
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::
                get_path_travel_time_truck (m_path_vec[i], TFlt (start_ptr[t]),
                                            m_link_tt_map_truck,
                                            get_cur_loading_interval ())
              * m_mcdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_car_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i] = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (start_ptr[i]))
                        ->get_link_freeflow_tt_loading_car ()
                      * m_mcdta->m_unit_time; // seconds
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i] = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (start_ptr[i]))
                        ->get_link_freeflow_tt_loading_truck ()
                      * m_mcdta->m_unit_time; // seconds
    }
  return result;
}

// unit: m_mcdta -> m_unit_time (eg: 5 seconds)
py::array_t<double>
Mcdta::get_car_link_tt (py::array_t<double> start_intervals, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_car_link_tt, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                 m_mcdta->m_unit_time,
                                 m_mcdta->m_current_loading_interval) ();
          if (_tmp * m_mcdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec[i]->m_length / m_link_vec[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec[i]->m_length
                         / m_link_vec[i]->m_ffs_car / m_mcdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mcdta->m_unit_time;
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_car_link_tt_robust (py::array_t<double> start_intervals,
                               py::array_t<double> end_intervals,
                               int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_tt_robust, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (end_ptr[t] <= start_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mcdta::get_car_link_tt_robust, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_car_link_tt_robust, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_car_link_tt_robust, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car_robust (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                        TFlt (end_ptr[t] + 1),
                                        m_mcdta->m_unit_time,
                                        m_mcdta->m_current_loading_interval,
                                        num_trials) ();
          if (_tmp * m_mcdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec[i]->m_length / m_link_vec[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec[i]->m_length
                         / m_link_vec[i]->m_ffs_car / m_mcdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mcdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_tt (py::array_t<double> start_intervals, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_truck_link_tt, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                   m_mcdta->m_unit_time,
                                   m_mcdta->m_current_loading_interval) ();
          if (_tmp * m_mcdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec[i]->m_length / m_link_vec[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec[i]->m_length
                         / m_link_vec[i]->m_ffs_truck / m_mcdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mcdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_tt_robust (py::array_t<double> start_intervals,
                                 py::array_t<double> end_intervals,
                                 int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_tt_robust, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (end_ptr[t] <= start_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mcdta::get_truck_link_tt_robust, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_truck_link_tt_robust, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_truck_link_tt_robust, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck_robust (m_link_vec[i],
                                          TFlt (start_ptr[t] + 1),
                                          TFlt (end_ptr[t] + 1),
                                          m_mcdta->m_unit_time,
                                          m_mcdta->m_current_loading_interval,
                                          num_trials) ();
          if (_tmp * m_mcdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec[i]->m_length / m_link_vec[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec[i]->m_length
                         / m_link_vec[i]->m_ffs_truck / m_mcdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mcdta->m_unit_time; // second
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_car_link_speed (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_speed, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_car_link_speed, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tt
            = MNM_DTA_GRADIENT::
                get_travel_time_car (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                     m_mcdta->m_unit_time,
                                     m_mcdta->m_current_loading_interval) ()
              * m_mcdta->m_unit_time; // seconds
          result_ptr[i * l + t]
            = (m_link_vec[i]->m_length) / _tt * 3600 / 1600; // mile per hour
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_speed (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_speed, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_truck_link_speed, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tt
            = MNM_DTA_GRADIENT::
                get_travel_time_truck (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                       m_mcdta->m_unit_time,
                                       m_mcdta->m_current_loading_interval) ()
              * m_mcdta->m_unit_time; // seconds
          result_ptr[i * l + t]
            = (m_link_vec[i]->m_length) / _tt * 3600 / 1600; // mile per hour
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_link_car_inflow (py::array_t<int> start_intervals,
                            py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_link_car_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_link_car_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (end_ptr[t] <= start_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_car_inflow, end time is smaller than "
            "or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_car_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_car_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_car (m_link_vec[i],
                                                     TFlt (start_ptr[t]),
                                                     TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_link_truck_inflow (py::array_t<int> start_intervals,
                              py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_link_truck_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_link_truck_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (end_ptr[t] <= start_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_truck_inflow, end time is smaller than "
            "or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_truck_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_link_truck_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_truck (m_link_vec[i],
                                                       TFlt (start_ptr[t]),
                                                       TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

int
Mcdta::register_paths (py::array_t<int> paths)
{
  if (m_path_vec.size () > 0)
    {
      printf ("Warning, Mcdta::register_paths, path exists\n");
      m_path_vec.clear ();
      m_path_set.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Mcdta::register_paths: Number of dimensions must be one");
    }
  int *paths_ptr = (int *) paths_buf.ptr;
  TInt _path_ID;
  for (int i = 0; i < paths_buf.shape[0]; i++)
    {
      _path_ID = TInt (paths_ptr[i]);
      // printf("registering path %d, %d\n", _path_ID(),
      // (int)m_ID_path_mapping.size());
      if (m_ID_path_mapping.find (_path_ID) == m_ID_path_mapping.end ())
        {
          throw std::runtime_error ("Mcdta::register_paths: No such path");
        }
      else
        {
          m_path_vec.push_back (m_ID_path_mapping[_path_ID]);
        }
    }
  m_path_set = std::set<MNM_Path *> (m_path_vec.begin (), m_path_vec.end ());
  return 0;
}

std::vector<bool>
Mcdta::check_registered_links_in_registered_paths ()
{
  std::vector<bool> _link_existing = std::vector<bool> ();
  if (m_link_vec.empty ())
    {
      printf ("Warning, Mcdta::check_registered_links_in_registered_paths, "
              "no link registered\n");
      return _link_existing;
    }
  for (size_t k = 0; k < m_link_vec.size (); ++k)
    {
      _link_existing.push_back (false);
    }
  if (m_path_vec.empty ())
    {
      printf ("Warning, Mcdta::check_registered_links_in_registered_paths, "
              "no path registered\n");
      return _link_existing;
    }
  for (auto *_path : m_path_vec)
    {
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          if (!_link_existing[i])
            {
              _link_existing[i] = _path->is_link_in (m_link_vec[i]->m_link_ID);
            }
        }
      if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                       [] (bool v) { return v; }))
        {
          break;
        }
    }
  if (std::any_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return !v; }))
    {
      printf ("Warning: some observed driving links in m_link_vec are not "
              "covered by generated paths in m_path_vec!\n");
    }
  return _link_existing;
}

py::array_t<bool>
Mcdta::are_registered_links_in_registered_paths ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing
    = check_registered_links_in_registered_paths ();

  int new_shape[1] = { (int) _link_existing.size () };
  auto result = py::array_t<bool> (new_shape);
  auto result_buf = result.request ();
  bool *result_ptr = (bool *) result_buf.ptr;

  for (size_t v = 0; v < _link_existing.size (); ++v)
    {
      result_ptr[v] = _link_existing[v];
    }
  return result;
}

py::array_t<int>
Mcdta::generate_paths_to_cover_registered_links ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing
    = check_registered_links_in_registered_paths ();

  int new_shape[1] = { 1 + (int) _link_existing.size () };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return v; }))
    {
      printf ("All links in m_link_vec are covered by paths in m_path_vec!\n");

      result_ptr[0] = 0; // indicate whether the path table is updated
      for (size_t v = 0; v < _link_existing.size (); ++v)
        {
          result_ptr[v + 1] = (int) _link_existing[v];
        }
      return result;
    }

  PNEGraph reversed_graph = MNM_Ults::reverse_graph (m_mcdta->m_graph);
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  for (auto _link_it : m_mcdta->m_link_factory->m_link_map)
    {
      _cost_map.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               dynamic_cast<MNM_Dlink_Multiclass *> (
                                 _link_it.second)
                                 ->get_link_freeflow_tt_car ()));
    }
  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TInt> _shortest_path_tree_reversed
    = std::unordered_map<TInt, TInt> ();
  TInt _from_node_ID, _to_node_ID;
  MNM_Origin_Multiclass *_origin;
  MNM_Destination_Multiclass *_dest;
  MNM_Path *_path_1, *_path_2, *_path;
  std::vector<std::pair<TInt, MNM_Origin *>> pair_ptrs_1
    = std::vector<std::pair<TInt, MNM_Origin *>> ();
  std::vector<std::pair<MNM_Destination *, TFlt *>> pair_ptrs_2
    = std::vector<std::pair<MNM_Destination *, TFlt *>> ();

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      if (!_link_existing[i])
        {
          // generate new path including this link
          _from_node_ID
            = m_mcdta->m_graph->GetEI (m_link_vec[i]->m_link_ID).GetSrcNId ();
          _to_node_ID
            = m_mcdta->m_graph->GetEI (m_link_vec[i]->m_link_ID).GetDstNId ();

          // path from origin to from_node_ID
          if (!_shortest_path_tree.empty ())
            {
              _shortest_path_tree.clear ();
            }
          if (dynamic_cast<MNM_DMOND_Multiclass *> (
                m_mcdta->m_node_factory->get_node (_from_node_ID))
              != nullptr)
            {
              _path_1 = new MNM_Path ();
              _path_1->m_node_vec.push_back (_from_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_from_node_ID,
                                                  m_mcdta->m_graph, _cost_map,
                                                  _shortest_path_tree);
            }

          // path from to_node_ID to destination
          if (!_shortest_path_tree_reversed.empty ())
            {
              _shortest_path_tree_reversed.clear ();
            }
          if (dynamic_cast<MNM_DMDND_Multiclass *> (
                m_mcdta->m_node_factory->get_node (_to_node_ID))
              != nullptr)
            {
              _path_2 = new MNM_Path ();
              _path_2->m_node_vec.push_back (_to_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_to_node_ID, reversed_graph,
                                                  _cost_map,
                                                  _shortest_path_tree_reversed);
            }

          _origin = nullptr;
          _dest = nullptr;
          bool _flg = false;

          if (!pair_ptrs_1.empty ())
            {
              pair_ptrs_1.clear ();
            }
          for (const auto &p : m_mcdta->m_od_factory->m_origin_map)
            {
              pair_ptrs_1.emplace_back (p);
            }
          std::random_shuffle (std::begin (pair_ptrs_1),
                               std::end (pair_ptrs_1));
          for (auto _it : pair_ptrs_1)
            {
              _origin = dynamic_cast<MNM_Origin_Multiclass *> (_it.second);
              if (_origin->m_demand_car.empty ())
                {
                  continue;
                }

              if (!pair_ptrs_2.empty ())
                {
                  pair_ptrs_2.clear ();
                }
              for (const auto &p : _origin->m_demand_car)
                {
                  pair_ptrs_2.emplace_back (p);
                }
              std::random_shuffle (std::begin (pair_ptrs_2),
                                   std::end (pair_ptrs_2));
              for (auto _it_it : pair_ptrs_2)
                {
                  _dest
                    = dynamic_cast<MNM_Destination_Multiclass *> (_it_it.first);
                  if (_shortest_path_tree
                          .find (_origin->m_origin_node->m_node_ID)
                          ->second
                        != -1
                      && _shortest_path_tree_reversed
                             .find (_dest->m_dest_node->m_node_ID)
                             ->second
                           != -1)
                    {
                      _flg = true;
                      break;
                    }
                }
              if (_flg)
                {
                  break;
                }
            }

          if (!_flg)
            {
              printf ("Cannot generate path covering this link\n");
              // exit(-1);
              continue;
            }
          IAssert (_origin != nullptr && _dest != nullptr);

          if (!_shortest_path_tree.empty ())
            {
              _path_1 = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                           _from_node_ID, _shortest_path_tree,
                                           m_mcdta->m_graph);
            }
          if (!_shortest_path_tree_reversed.empty ())
            {
              _path_2
                = MNM::extract_path (_dest->m_dest_node->m_node_ID, _to_node_ID,
                                     _shortest_path_tree_reversed,
                                     reversed_graph);
            }

          // merge the paths to a complete path
          _path = new MNM_Path ();
          _path->m_link_vec = _path_1->m_link_vec;
          _path->m_link_vec.push_back (m_link_vec[i]->m_link_ID);
          _path->m_link_vec.insert (_path->m_link_vec.end (),
                                    _path_2->m_link_vec.rbegin (),
                                    _path_2->m_link_vec.rend ());
          _path->m_node_vec = _path_1->m_node_vec;
          _path->m_node_vec.insert (_path->m_node_vec.end (),
                                    _path_2->m_node_vec.rbegin (),
                                    _path_2->m_node_vec.rend ());
          _path->allocate_buffer (
            2 * m_mcdta->m_config->get_int ("max_interval"));
          delete _path_1;
          delete _path_2;
          // add this new path to path table
          dynamic_cast<MNM_Routing_Biclass_Hybrid *> (m_mcdta->m_routing)
            ->m_routing_fixed_car->m_path_table
            ->find (_origin->m_origin_node->m_node_ID)
            ->second->find (_dest->m_dest_node->m_node_ID)
            ->second->m_path_vec.push_back (_path);
          m_path_vec.push_back (_path);
          _link_existing[i] = true;

          // check if this new path cover other links
          for (size_t j = 0; j < m_link_vec.size (); ++j)
            {
              if (!_link_existing[j])
                {
                  _link_existing[j]
                    = _path->is_link_in (m_link_vec[j]->m_link_ID);
                }
            }
          if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                           [] (bool v) { return v; }))
            {
              printf ("All links in m_link_vec are covered by paths in "
                      "m_path_vec!\n");
              break;
            }
        }
    }

  if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return v; }))
    {
      printf ("All links in m_link_vec are covered by paths in m_path_vec!\n");
    }
  else
    {
      printf ("Mcdta::generate_paths_to_cover_registered_links, NOT all "
              "links in m_link_vec are covered by paths in m_path_vec!\n");
      // exit(-1);
    }

  MNM::save_path_table (m_mcdta->m_file_folder,
                        dynamic_cast<MNM_Routing_Biclass_Hybrid *> (
                          m_mcdta->m_routing)
                          ->m_routing_fixed_car->m_path_table,
                        m_mcdta->m_od_factory, true, false);

  result_ptr[0] = 1; // indicate whether the path table is updated
  for (size_t v = 0; v < _link_existing.size (); ++v)
    {
      result_ptr[v + 1] = (int) _link_existing[v];
    }

  _link_existing.clear ();
  _cost_map.clear ();
  _shortest_path_tree.clear ();
  _shortest_path_tree_reversed.clear ();
  reversed_graph.Clr ();
  pair_ptrs_1.clear ();
  pair_ptrs_2.clear ();
  return result;
}

int
Mcdta::save_path_table (const std::string &folder)
{
  // write updated path table to file
  MNM::save_path_table (folder,
                        dynamic_cast<MNM_Routing_Biclass_Hybrid *> (
                          m_mcdta->m_routing)
                          ->m_routing_fixed_car->m_path_table,
                        m_mcdta->m_od_factory, true, false);
  return 0;
}

py::array_t<double>
Mcdta::get_car_link_out_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_out_car->m_recorder;
  int new_shape[2] = { (int) _record.size (), 2 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      result_ptr[i * 2] = _record[i].first ();
      result_ptr[i * 2 + 1] = _record[i].second ();
    }
  return result;
}

py::array_t<double>
Mcdta::get_car_link_in_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_in_car->m_recorder;
  int new_shape[2] = { (int) _record.size (), 2 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      result_ptr[i * 2] = _record[i].first ();
      result_ptr[i * 2 + 1] = _record[i].second ();
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_out_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_out_truck->m_recorder;
  int new_shape[2] = { (int) _record.size (), 2 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      result_ptr[i * 2] = _record[i].first ();
      result_ptr[i * 2 + 1] = _record[i].second ();
    }
  return result;
}

py::array_t<double>
Mcdta::get_truck_link_in_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_in_truck->m_recorder;
  int new_shape[2] = { (int) _record.size (), 2 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      result_ptr[i * 2] = _record[i].first ();
      result_ptr[i * 2 + 1] = _record[i].second ();
    }
  return result;
}

py::array_t<double>
Mcdta::get_enroute_and_queue_veh_stats_agg ()
{
  int _tot_interval = get_cur_loading_interval ();
  int new_shape[2] = { _tot_interval, 3 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  if ((int) m_mcdta->m_enroute_veh_num.size () != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mcdta::get_enroute_and_queue_veh_stats_agg, enroute "
        "vehicle missed for some intervals");
    }
  else if ((int) m_mcdta->m_queue_veh_num.size ()
           != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mcdta::get_enroute_and_queue_veh_stats_agg, queuing "
        "vehicle missed for some intervals");
    }
  else
    {
      for (int i = 0; i < _tot_interval; ++i)
        {
          result_ptr[i * 3]
            = (m_mcdta->m_enroute_veh_num[i]()) / (m_mcdta->m_flow_scalar);
          result_ptr[i * 3 + 1]
            = (m_mcdta->m_queue_veh_num[i]()) / (m_mcdta->m_flow_scalar);
          result_ptr[i * 3 + 2]
            = (m_mcdta->m_enroute_veh_num[i]() - m_mcdta->m_queue_veh_num[i]())
              / (m_mcdta->m_flow_scalar);
        }
    }
  return result;
}

py::array_t<double>
Mcdta::get_queue_veh_each_link (py::array_t<int> useful_links,
                                py::array_t<int> intervals)
{
  auto intervals_buf = intervals.request ();
  if (intervals_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mcdta::get_queue_veh_each_link, "
                                "input (intervals) dimension mismatch");
    }
  auto links_buf = useful_links.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mcdta::get_queue_veh_each_link, "
                                "input (useful_links) dimension mismatch");
    }
  int num_intervals = intervals_buf.shape[0];
  int num_links = links_buf.shape[0];
  int new_shape[2] = { num_links, num_intervals };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *intervals_ptr = (double *) intervals_buf.ptr;
  double *links_ptr = (double *) links_buf.ptr;

  for (int t = 0; t < num_intervals; ++t)
    {
      if (intervals_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mcdta::get_queue_veh_each_link, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (int i = 0; i < num_links; ++i)
        {
          if (m_mcdta->m_queue_veh_map.find (links_ptr[i])
              == m_mcdta->m_queue_veh_map.end ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_queue_veh_each_link, can't find link "
                "ID");
            }
          // not divided by flow_scalar in the original version
          result_ptr[i * num_intervals + t]
            = (*(m_mcdta->m_queue_veh_map[links_ptr[i]]))[intervals_ptr[t]]
              / m_mcdta->m_flow_scalar;
        }
    }
  return result;
}

double
Mcdta::get_car_link_out_num (int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("%d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_link_out_num, cc not installed");
    }
  // printf("1\n");
  TFlt result = _link->m_N_out_car->get_result (TFlt (time));
  // printf("%lf\n", result());
  return result ();
}

double
Mcdta::get_truck_link_out_num (int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mcdta->m_link_factory->get_link (
      TInt (link_ID));
  if (_link->m_N_out_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_link_out_num, cc not installed");
    }
  TFlt result = _link->m_N_out_truck->get_result (TFlt (time));
  return result ();
}

py::array_t<double>
Mcdta::get_car_dar_matrix (py::array_t<int> start_intervals,
                           py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_car_dar_matrix, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //   m_link_vec[i] -> m_N_in_tree -> print_out();
  // }

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      // printf("Current processing time: %d\n", t);
      std::cout << "************ Car DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::get_car_dar_matrix, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_car_dar_matrix, input start intervals "
                "exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_car_dar_matrix, input end intervals "
                "exceeds the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records_car (_record, m_link_vec[i],
                                                 m_path_set,
                                                 TFlt (start_ptr[t]),
                                                 TFlt (end_ptr[t]));
        }
    }
  // _record.size() = num_timesteps x num_links x num_path x
  // num_assign_timesteps path_ID, assign_time, link_ID, start_int, flow
  int new_shape[2] = { (int) _record.size (), 5 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  dar_record *tmp_record;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      tmp_record = _record[i];
      result_ptr[i * 5 + 0] = (double) tmp_record->path_ID ();
      // the count of 1 min interval
      result_ptr[i * 5 + 1] = (double) tmp_record->assign_int ();
      result_ptr[i * 5 + 2] = (double) tmp_record->link_ID ();
      // the count of unit time interval (5s)
      result_ptr[i * 5 + 3] = (double) tmp_record->link_start_int ();
      result_ptr[i * 5 + 4] = tmp_record->flow ();
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mcdta::get_truck_dar_matrix (py::array_t<int> start_intervals,
                             py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_dar_matrix, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //   m_link_vec[i] -> m_N_in_tree -> print_out();
  // }

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      std::cout << "************ Truck DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::get_truck_dar_matrix, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_truck_dar_matrix, input start intervals "
                "exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_truck_dar_matrix, input end intervals "
                "exceeds the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records_truck (_record, m_link_vec[i],
                                                   m_path_set,
                                                   TFlt (start_ptr[t]),
                                                   TFlt (end_ptr[t]));
        }
    }
  // path_ID, assign_time, link_ID, start_int, flow
  int new_shape[2] = { (int) _record.size (), 5 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  dar_record *tmp_record;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      tmp_record = _record[i];
      result_ptr[i * 5 + 0] = (double) tmp_record->path_ID ();
      // the count of 1 min interval
      result_ptr[i * 5 + 1] = (double) tmp_record->assign_int ();
      result_ptr[i * 5 + 2] = (double) tmp_record->link_ID ();
      // the count of unit time interval (5s)
      result_ptr[i * 5 + 3] = (double) tmp_record->link_start_int ();
      result_ptr[i * 5 + 4] = tmp_record->flow ();
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

int
Mcdta::save_car_dar_matrix (py::array_t<int> start_intervals,
                            py::array_t<int> end_intervals,
                            py::array_t<double> f, const std::string &file_name)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 720, ...] with increment of ass_freq
  auto f_buf = f.request ();
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::save_car_dar_matrix, input path flow mismatch");
    }
  double *f_ptr = (double *) f_buf.ptr;

  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::save_car_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::save_car_dar_matrix, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //   m_link_vec[i] -> m_N_in_tree -> print_out();
  // }

  std::ofstream _file;
  std::string _str;
  _file.open (file_name, std::ofstream::out);
  if (!_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + file_name);
    }

  int _num_path = m_path_vec.size ();
  int _num_link = m_link_vec.size ();
  int _x, _y;

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      std::cout << "************ Car DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::save_car_dar_matrix, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::save_car_dar_matrix, input start intervals "
                "exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::save_car_dar_matrix, input end intervals "
                "exceeds the total loading intervals");
            }
          IAssert (_record.empty ());
          MNM_DTA_GRADIENT::add_dar_records_car (_record, m_link_vec[i],
                                                 m_path_set,
                                                 TFlt (start_ptr[t]),
                                                 TFlt (end_ptr[t]));
          for (size_t j = 0; j < _record.size (); ++j)
            {
              dar_record *tmp_record = _record[j];

              _x = i + _num_link * t; // # of links * # of intervals
              // assume path_ID starts from zero
              _y = tmp_record->path_ID
                   + _num_path
                       * tmp_record
                           ->assign_int (); // # of paths * # of intervals
              _str = std::to_string (_x) + ",";
              _str += std::to_string (_y) + ",";
              _str += std::to_string (tmp_record->flow () / f_ptr[_y]) + "\n";

              // _str = std::to_string(tmp_record -> path_ID()) + ","
              // // the count of 15 min interval
              // _str += std::to_string(tmp_record -> assign_int()) + ",";
              // _str += std::to_string(tmp_record -> link_ID()) + ",";
              // // the count of unit time interval (5s)
              // _str += std::to_string(tmp_record -> link_start_int()) + ",";
              // _str += std::to_string(tmp_record -> flow()) + "\n";

              _file << _str;
              delete _record[j];
            }
          // for (size_t i = 0; i < _record.size(); ++i){
          //     delete _record[i];
          // }
          _record.clear ();
        }
    }

  if (_file.is_open ())
    _file.close ();
  return 0;
}

int
Mcdta::save_truck_dar_matrix (py::array_t<int> start_intervals,
                              py::array_t<int> end_intervals,
                              py::array_t<double> f,
                              const std::string &file_name)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 720, ...] with increment of ass_freq
  auto f_buf = f.request ();
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::save_truck_dar_matrix, input path flow mismatch");
    }
  double *f_ptr = (double *) f_buf.ptr;

  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::save_truck_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::save_truck_dar_matrix, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //   m_link_vec[i] -> m_N_in_tree -> print_out();
  // }

  std::ofstream _file;
  std::string _str;
  _file.open (file_name, std::ofstream::out);
  if (!_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + file_name);
    }

  int _num_path = m_path_vec.size ();
  int _num_link = m_link_vec.size ();
  int _x, _y;

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      std::cout << "************ Truck DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::save_truck_dar_matrix, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::save_truck_dar_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::save_truck_dar_matrix, input end intervals "
                "exceeds the total loading intervals");
            }
          IAssert (_record.empty ());
          MNM_DTA_GRADIENT::add_dar_records_truck (_record, m_link_vec[i],
                                                   m_path_set,
                                                   TFlt (start_ptr[t]),
                                                   TFlt (end_ptr[t]));
          for (size_t j = 0; j < _record.size (); ++j)
            {
              dar_record *tmp_record = _record[j];

              _x = i + _num_link * t; // # of links * # of intervals
              // assume path_ID starts from zero
              _y = tmp_record->path_ID
                   + _num_path
                       * tmp_record
                           ->assign_int (); // # of paths * # of intervals
              _str = std::to_string (_x) + ",";
              _str += std::to_string (_y) + ",";
              _str += std::to_string (tmp_record->flow () / f_ptr[_y]) + "\n";

              // _str = std::to_string(tmp_record -> path_ID()) + ","
              // // the count of 15 min interval
              // _str += std::to_string(tmp_record -> assign_int()) + ",";
              // _str += std::to_string(tmp_record -> link_ID()) + ",";
              // // the count of unit time interval (5s)
              // _str += std::to_string(tmp_record -> link_start_int()) + ",";
              // _str += std::to_string(tmp_record -> flow()) + "\n";

              _file << _str;
              delete _record[j];
            }
          // for (size_t i = 0; i < _record.size(); ++i){
          //     delete _record[i];
          // }
          _record.clear ();
        }
    }

  if (_file.is_open ())
    _file.close ();
  return 0;
}

SparseMatrixR
Mcdta::get_complete_car_dar_matrix (py::array_t<int> start_intervals,
                                    py::array_t<int> end_intervals,
                                    int num_intervals, py::array_t<double> f)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 720, ...] with increment of ass_freq
  int _num_e_path = m_path_vec.size ();
  int _num_e_link = m_link_vec.size ();
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  auto f_buf = f.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_car_dar_matrix, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_car_dar_matrix, input length mismatch");
    }
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_car_dar_matrix, input path flow "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  double *f_ptr = (double *) f_buf.ptr;

  int _num_of_minute
    = int (m_mcdta->m_config->get_int ("assign_frq")
           / m_mcdta->m_assign_freq); // 15 min, release interval

  // std::vector<Eigen::Triplet<double>> _record;
  // // pre-allocate sufficient space for dar
  // // _record.reserve(int(_num_e_link * num_intervals * _num_e_path *
  // num_intervals * 0.3)); _record.reserve(int(1e7));

  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //     for (int t = 0; t < l; ++t){
  //         if (end_ptr[t] <= start_ptr[t]){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_car_dar_matrix, end time is smaller
  //             than or equal to start time");
  //         }
  //         if (start_ptr[t] >= get_cur_loading_interval()){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_car_dar_matrix, input start intervals
  //             exceeds the total loading intervals - 1");
  //         }
  //         if (end_ptr[t] > get_cur_loading_interval()){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_car_dar_matrix, input end intervals
  //             exceeds the total loading intervals");
  //         }
  //         MNM_DTA_GRADIENT::add_dar_records_eigen_car(_record, m_link_vec[i],
  //         m_path_set,
  //                                                     TFlt(start_ptr[t]),
  //                                                     TFlt(end_ptr[t]), i, t,
  //                                                     _num_of_minute,
  //                                                     _num_e_link,
  //                                                     _num_e_path, f_ptr);
  //     }
  // }
  // // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // // dar matrix rho
  // SparseMatrixR mat(num_intervals * _num_e_link, num_intervals *
  // _num_e_path);
  // //
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#acc35051d698e3973f1de5b9b78dbe345
  // mat.setFromTriplets(_record.begin(), _record.end());

  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3
  // dar matrix rho
  SparseMatrixR mat (num_intervals * _num_e_link, num_intervals * _num_e_path);
  mat.reserve (Eigen::VectorXi::Constant (num_intervals * _num_e_link, 2000));
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_car_dar_matrix, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_car_dar_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_car_dar_matrix, input end "
                "intervals exceeds the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records_eigen_car (mat, m_link_vec[i],
                                                       m_path_set,
                                                       TFlt (start_ptr[t]),
                                                       TFlt (end_ptr[t]), i, t,
                                                       _num_of_minute,
                                                       _num_e_link, _num_e_path,
                                                       f_ptr);
        }
    }
  mat.makeCompressed ();

  return mat;
}

SparseMatrixR
Mcdta::get_complete_truck_dar_matrix (py::array_t<int> start_intervals,
                                      py::array_t<int> end_intervals,
                                      int num_intervals, py::array_t<double> f)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 720, ...] with increment of ass_freq
  int _num_e_path = m_path_vec.size ();
  int _num_e_link = m_link_vec.size ();
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  auto f_buf = f.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_truck_dar_matrix, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_truck_dar_matrix, input length "
        "mismatch");
    }
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_truck_dar_matrix, input path flow "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  double *f_ptr = (double *) f_buf.ptr;

  int _num_of_minute
    = int (m_mcdta->m_config->get_int ("assign_frq")
           / m_mcdta->m_assign_freq); // 15 min, release interval

  // std::vector<Eigen::Triplet<double>> _record;
  // // pre-allocate sufficient space for dar
  // // _record.reserve(int(_num_e_link * num_intervals * _num_e_path *
  // num_intervals * 0.3)); _record.reserve(int(1e7));

  // for (size_t i = 0; i<m_link_vec.size(); ++i){
  //     for (int t = 0; t < l; ++t){
  //         if (end_ptr[t] <= start_ptr[t]){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_truck_dar_matrix, end time is smaller
  //             than or equal to start time");
  //         }
  //         if (start_ptr[t] >= get_cur_loading_interval()){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_truck_dar_matrix, input start intervals
  //             exceeds the total loading intervals - 1");
  //         }
  //         if (end_ptr[t] > get_cur_loading_interval()){
  //             throw std::runtime_error("Error,
  //             Mcdta::get_complete_truck_dar_matrix, input end intervals
  //             exceeds the total loading intervals");
  //         }
  //         MNM_DTA_GRADIENT::add_dar_records_eigen_truck(_record,
  //         m_link_vec[i], m_path_set,
  //                                                       TFlt(start_ptr[t]),
  //                                                       TFlt(end_ptr[t]), i,
  //                                                       t, _num_of_minute,
  //                                                       _num_e_link,
  //                                                       _num_e_path, f_ptr);
  //     }
  // }
  // // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // // dar matrix rho
  // SparseMatrixR mat(num_intervals * _num_e_link, num_intervals *
  // _num_e_path);
  // //
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#acc35051d698e3973f1de5b9b78dbe345
  // mat.setFromTriplets(_record.begin(), _record.end());

  // http://eigen.tuxfamily.org/dox/group__TutorialSparse.html#title3
  // dar matrix rho
  SparseMatrixR mat (num_intervals * _num_e_link, num_intervals * _num_e_path);
  mat.reserve (Eigen::VectorXi::Constant (num_intervals * _num_e_link, 2000));
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_truck_dar_matrix, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_truck_dar_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_truck_dar_matrix, input end "
                "intervals exceeds the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records_eigen_truck (mat, m_link_vec[i],
                                                         m_path_set,
                                                         TFlt (start_ptr[t]),
                                                         TFlt (end_ptr[t]), i,
                                                         t, _num_of_minute,
                                                         _num_e_link,
                                                         _num_e_path, f_ptr);
        }
    }
  mat.makeCompressed ();

  return mat;
}

py::array_t<double>
Mcdta::get_car_ltg_matrix (py::array_t<int> start_intervals,
                           int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mcdta::build_link_cost_map() and
  // Mcdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mcdta::get_car_ltg_matrix_driving, "
                                "input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec)
    {
      // check if the path does not include any link in m_link_vec
      _flg = false;
      for (auto *_link : m_link_vec)
        {
          if (_path->is_link_in (_link->m_link_ID))
            {
              _flg = true;
              break;
            }
        }
      if (!_flg)
        {
          continue;
        }

      for (int t = 0; t < l; ++t)
        {
          // printf("Current processing time: %d\n", t);
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_car_ltg_matrix_driving, input start "
                "intervals exceeds the total loading intervals - 1");
            }

          _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1,
          _t_depart_lift_up = -1, _t_depart_prime = -1;
          // trace one additional veh departing from origin of path at
          // start_ptr[t]
          _t_depart = start_ptr[t];
          for (TInt _link_ID : _path->m_link_vec)
            {
              // arrival and departure time of original perturbation vehicle
              _t_arrival = _t_depart;
              _t_depart
                = _t_arrival
                  + MNM_Ults::round_up_time (
                    m_link_tt_map[_link_ID]
                                 [_t_arrival < get_cur_loading_interval ()
                                    ? _t_arrival
                                    : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mcdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                           != nullptr);
                  IAssert (_t_depart_lift_up >= 0); // from its upstream link
                  _t_arrival_lift_up = _t_depart_lift_up;
                }
              IAssert (_t_arrival_lift_up >= _t_arrival);

              IAssert (_link->m_last_valid_time > 0);
              if (_t_arrival_lift_up
                    > int (round (_link->m_last_valid_time - 1))
                  || _t_arrival_lift_up >= threshold_timestamp)
                {
                  break;
                }

              // departure time of new perturbation vehicle
              _t_depart_prime
                = _t_arrival_lift_up
                  + MNM_Ults::round_up_time (
                    m_link_tt_map[_link_ID]
                                 [_t_arrival_lift_up
                                      < get_cur_loading_interval ()
                                    ? _t_arrival_lift_up
                                    : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_car ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_car() / m_mcdta -> m_unit_time;
              _t_depart_lift_up
                = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_link_congested_car[_link_ID][_t_arrival_lift_up])
                {
                  if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                      == _t_arrival_lift_up)
                    {
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               == _t_depart_lift_up);
                    }
                  else
                    {
                      // critical state where subgradient applies
                      IAssert (m_queue_dissipated_time_car[_link_ID]
                                                          [_t_arrival_lift_up]
                               > _t_arrival_lift_up);
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               < _t_depart_lift_up);
                    }
                }
              else
                {
                  IAssert (
                    m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                    > _t_arrival_lift_up);
                  IAssert (_t_arrival_lift_up + MNM_Ults::round_up_time (_fftt)
                           < _t_depart_lift_up);
                }

              if (_t_depart_lift_up < _t_depart_prime)
                {
                  throw std::runtime_error ("invalid state");
                  // _t_depart_lift_up can be equal to _t_depart_prime, when the
                  // arrival curve is horizontal
                }

              if (_t_depart_prime < get_cur_loading_interval () - 1
                  && std::find_if (m_link_vec.begin (), m_link_vec.end (),
                                   [&_link_ID] (
                                     const MNM_Dlink_Multiclass *_l) {
                                     return _l->m_link_ID == _link_ID;
                                   })
                       != m_link_vec.end ())
                {
                  if (m_link_congested_car[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time > 0);
                      if (m_queue_dissipated_time_car[_link_ID]
                                                     [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_queue_dissipated_time_car[_link_ID]
                                                         [_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_link_tt_map[_link_ID]
                                             [_t_queue_dissipated_valid - 1
                                                  < get_cur_loading_interval ()
                                                ? _t_queue_dissipated_valid - 1
                                                : get_cur_loading_interval ()
                                                    - 1]);
                        }
                      // TODO: debug
                      if (_t_depart_lift_up_valid
                          > get_cur_loading_interval () - 1)
                        {
                          _t_depart_lift_up_valid
                            = get_cur_loading_interval () - 1;
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout << "\nError, Mcdta::get_car_ltg_matrix"
                                    << "\n";
                          std::cout << "interval: " << start_ptr[t]
                                    << ", link: " << _link_ID << "\n";
                          std::cout << "car in"
                                    << "\n";
                          std::cout << _link->m_N_in_car->to_string () << "\n";
                          std::cout << "car out"
                                    << "\n";
                          std::cout << _link->m_N_out_car->to_string ()
                                    << "\n\n";
                          std::cout
                            << "last valid time: " << _link->m_last_valid_time
                            << "\n";
                          std::cout << "_t_arrival: " << _t_arrival << "\n";
                          std::cout << "_t_depart: " << _t_depart << "\n";
                          std::cout
                            << "_t_arrival_lift_up: " << _t_arrival_lift_up
                            << "\n";
                          std::cout << "_t_depart_prime: " << _t_depart_prime
                                    << "\n";
                          std::cout
                            << "m_queue_dissipated_time_car[_link_ID][_t_"
                               "arrival_lift_up]: "
                            << m_queue_dissipated_time_car[_link_ID]
                                                          [_t_arrival_lift_up]
                            << "\n";
                          std::cout << "_t_queue_dissipated_valid: "
                                    << _t_queue_dissipated_valid << "\n";
                          std::cout
                            << "_t_depart_lift_up: " << _t_depart_lift_up
                            << "\n";
                          std::cout << "_t_depart_lift_up_valid: "
                                    << _t_depart_lift_up_valid << "\n";
                          std::cout << "_fftt: " << _fftt << "\n";
                          std::cout
                            << "m_link_tt_map[_link_ID][_t_queue_dissipated_"
                               "valid]: "
                            << m_link_tt_map[_link_ID]
                                            [_t_queue_dissipated_valid - 1
                                                 < get_cur_loading_interval ()
                                               ? _t_queue_dissipated_valid - 1
                                               : get_cur_loading_interval ()
                                                   - 1]
                            << "\n";
                          std::cout << "get_cur_loading_interval(): "
                                    << get_cur_loading_interval () << "\n";
                          throw std::runtime_error ("invalid state");
                        }
                      if (_t_depart_prime < _t_depart_lift_up_valid)
                        {
                          _gradient
                            = MNM_DTA_GRADIENT::get_departure_cc_slope_car (
                              _link, TFlt (_t_depart_prime),
                              TFlt (_t_depart_lift_up_valid + 1));
                          if (_gradient > DBL_EPSILON)
                            {
                              _gradient
                                = m_mcdta->m_unit_time / _gradient; // seconds
                              for (int t_prime = _t_arrival_lift_up;
                                   t_prime < _t_queue_dissipated_valid;
                                   ++t_prime)
                                {
                                  MNM_DTA_GRADIENT::
                                    add_ltg_records_veh (_record, _link, _path,
                                                         start_ptr[t], t_prime,
                                                         _gradient);
                                }
                            }

                          // for (int t_prime = _t_arrival_lift_up; t_prime <
                          // _t_queue_dissipated_valid; ++t_prime) {
                          //     _gradient =
                          //     MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link,
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mcdta -> m_unit_time /
                          //         _gradient;  // seconds
                          //         MNM_DTA_GRADIENT::add_ltg_records_veh(_record,
                          //         _link, _path, start_ptr[t], t_prime,
                          //         _gradient);
                          //     }
                          // }
                        }
                    }
                }
            }
        }
    }

  // _record.size() = num_timesteps x num_links x num_path x
  // num_assign_timesteps path_ID, assign_time, link_ID, start_int, gradient
  int new_shape[2] = { (int) _record.size (), 5 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  ltg_record *tmp_record;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      tmp_record = _record[i];
      result_ptr[i * 5 + 0] = (double) tmp_record->path_ID ();
      // the count of 1 min interval
      result_ptr[i * 5 + 1] = (double) tmp_record->assign_int;
      result_ptr[i * 5 + 2] = (double) tmp_record->link_ID ();
      // the count of unit time interval (5s)
      result_ptr[i * 5 + 3] = (double) tmp_record->link_start_int;
      result_ptr[i * 5 + 4] = tmp_record->gradient ();
      // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f,
      // time interval (5 s): %f, gradient: %f\n",
      //         result_ptr[i * 5 + 0], result_ptr[i * 5 + 1], result_ptr[i * 5
      //         + 2], result_ptr[i * 5 + 3], result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mcdta::get_truck_ltg_matrix (py::array_t<int> start_intervals,
                             int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mcdta::build_link_cost_map() and
  // Mcdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_truck_ltg_matrix, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec)
    {
      // check if the path does not include any link in m_link_vec
      _flg = false;
      for (auto *_link : m_link_vec)
        {
          if (_path->is_link_in (_link->m_link_ID))
            {
              _flg = true;
              break;
            }
        }
      if (!_flg)
        {
          continue;
        }

      for (int t = 0; t < l; ++t)
        {
          // printf("Current processing time: %d\n", t);
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_truck_ltg_matrix, input start intervals "
                "exceeds the total loading intervals - 1");
            }

          _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1,
          _t_depart_lift_up = -1, _t_depart_prime = -1;
          // trace one additional veh departing from origin of path at
          // start_ptr[t]
          _t_depart = start_ptr[t];
          for (TInt _link_ID : _path->m_link_vec)
            {
              // arrival and departure time of original perturbation vehicle
              _t_arrival = _t_depart;
              _t_depart
                = _t_arrival
                  + MNM_Ults::round_up_time (
                    m_link_tt_map_truck[_link_ID]
                                       [_t_arrival < get_cur_loading_interval ()
                                          ? _t_arrival
                                          : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mcdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                           != nullptr);
                  IAssert (_t_depart_lift_up >= 0); // from its upstream link
                  _t_arrival_lift_up = _t_depart_lift_up;
                }
              IAssert (_t_arrival_lift_up >= _t_arrival);

              IAssert (_link->m_last_valid_time > 0);
              if (_t_arrival_lift_up
                    > int (round (_link->m_last_valid_time - 1))
                  || _t_arrival_lift_up >= threshold_timestamp)
                {
                  break;
                }

              // departure time of new perturbation vehicle
              _t_depart_prime
                = _t_arrival_lift_up
                  + MNM_Ults::round_up_time (
                    m_link_tt_map_truck[_link_ID]
                                       [_t_arrival_lift_up
                                            < get_cur_loading_interval ()
                                          ? _t_arrival_lift_up
                                          : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_truck ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
              _t_depart_lift_up
                = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_link_congested_truck[_link_ID][_t_arrival_lift_up])
                {
                  if (m_queue_dissipated_time_truck[_link_ID]
                                                   [_t_arrival_lift_up]
                      == _t_arrival_lift_up)
                    {
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               == _t_depart_lift_up);
                    }
                  else
                    {
                      // critical state where subgradient applies
                      IAssert (m_queue_dissipated_time_truck[_link_ID]
                                                            [_t_arrival_lift_up]
                               > _t_arrival_lift_up);
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               < _t_depart_lift_up);
                    }
                }
              else
                {
                  IAssert (
                    m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]
                    > _t_arrival_lift_up);
                  IAssert (_t_arrival_lift_up + MNM_Ults::round_up_time (_fftt)
                           < _t_depart_lift_up);
                }

              if (_t_depart_lift_up < _t_depart_prime)
                {
                  throw std::runtime_error ("invalid state");
                  // _t_depart_lift_up can be equal to _t_depart_prime, when the
                  // arrival curve is horizontal
                }

              if (_t_depart_prime < get_cur_loading_interval () - 1
                  && std::find_if (m_link_vec.begin (), m_link_vec.end (),
                                   [&_link_ID] (
                                     const MNM_Dlink_Multiclass *_l) {
                                     return _l->m_link_ID == _link_ID;
                                   })
                       != m_link_vec.end ())
                {
                  if (m_link_congested_truck[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time > 0);
                      if (m_queue_dissipated_time_truck[_link_ID]
                                                       [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_queue_dissipated_time_truck[_link_ID]
                                                           [_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_link_tt_map_truck
                                  [_link_ID]
                                  [_t_queue_dissipated_valid - 1
                                       < get_cur_loading_interval ()
                                     ? _t_queue_dissipated_valid - 1
                                     : get_cur_loading_interval () - 1]);
                        }
                      // TODO: debug
                      if (_t_depart_lift_up_valid
                          > get_cur_loading_interval () - 1)
                        {
                          _t_depart_lift_up_valid
                            = get_cur_loading_interval () - 1;
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout << "\nError, Mcdta::get_truck_ltg_matrix"
                                    << "\n";
                          std::cout << "interval: " << start_ptr[t]
                                    << ", link: " << _link_ID << "\n";
                          std::cout << "truck in"
                                    << "\n";
                          std::cout << _link->m_N_in_truck->to_string ()
                                    << "\n";
                          std::cout << "truck out"
                                    << "\n";
                          std::cout << _link->m_N_out_truck->to_string ()
                                    << "\n\n";
                          std::cout
                            << "last valid time: " << _link->m_last_valid_time
                            << "\n";
                          std::cout << "_t_arrival: " << _t_arrival << "\n";
                          std::cout << "_t_depart: " << _t_depart << "\n";
                          std::cout
                            << "_t_arrival_lift_up: " << _t_arrival_lift_up
                            << "\n";
                          std::cout << "_t_depart_prime: " << _t_depart_prime
                                    << "\n";
                          std::cout
                            << "m_queue_dissipated_time_truck[_link_ID][_t_"
                               "arrival_lift_up]: "
                            << m_queue_dissipated_time_truck[_link_ID]
                                                            [_t_arrival_lift_up]
                            << "\n";
                          std::cout << "_t_queue_dissipated_valid: "
                                    << _t_queue_dissipated_valid << "\n";
                          std::cout
                            << "_t_depart_lift_up: " << _t_depart_lift_up
                            << "\n";
                          std::cout << "_t_depart_lift_up_valid: "
                                    << _t_depart_lift_up_valid << "\n";
                          std::cout << "_fftt: " << _fftt << "\n";
                          std::cout
                            << "m_link_tt_map_truck[_link_ID][_t_queue_"
                               "dissipated_valid]: "
                            << m_link_tt_map_truck
                                 [_link_ID][_t_queue_dissipated_valid - 1
                                                < get_cur_loading_interval ()
                                              ? _t_queue_dissipated_valid - 1
                                              : get_cur_loading_interval () - 1]
                            << "\n";
                          std::cout << "get_cur_loading_interval(): "
                                    << get_cur_loading_interval () << "\n";
                          throw std::runtime_error ("invalid state");
                        }
                      if (_t_depart_prime < _t_depart_lift_up_valid)
                        {
                          _gradient
                            = MNM_DTA_GRADIENT::get_departure_cc_slope_truck (
                              _link, TFlt (_t_depart_prime),
                              TFlt (_t_depart_lift_up_valid + 1));
                          if (_gradient > DBL_EPSILON)
                            {
                              _gradient
                                = m_mcdta->m_unit_time / _gradient; // seconds
                              for (int t_prime = _t_arrival_lift_up;
                                   t_prime < _t_queue_dissipated_valid;
                                   ++t_prime)
                                {
                                  MNM_DTA_GRADIENT::
                                    add_ltg_records_veh (_record, _link, _path,
                                                         start_ptr[t], t_prime,
                                                         _gradient);
                                }
                            }

                          // for (int t_prime = _t_arrival_lift_up; t_prime <
                          // _t_queue_dissipated_valid; ++t_prime) {
                          //     _gradient =
                          //     MNM_DTA_GRADIENT::get_departure_cc_slope_truck(_link,
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mcdta -> m_unit_time /
                          //         _gradient;  // seconds
                          //         MNM_DTA_GRADIENT::add_ltg_records_veh(_record,
                          //         _link, _path, start_ptr[t], t_prime,
                          //         _gradient);
                          //     }
                          // }
                        }
                    }
                }
            }
        }
    }

  // _record.size() = num_timesteps x num_links x num_path x
  // num_assign_timesteps path_ID, assign_time, link_ID, start_int, gradient
  int new_shape[2] = { (int) _record.size (), 5 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  ltg_record *tmp_record;
  for (size_t i = 0; i < _record.size (); ++i)
    {
      tmp_record = _record[i];
      result_ptr[i * 5 + 0] = (double) tmp_record->path_ID ();
      // the count of 1 min interval
      result_ptr[i * 5 + 1] = (double) tmp_record->assign_int;
      result_ptr[i * 5 + 2] = (double) tmp_record->link_ID ();
      // the count of unit time interval (5s)
      result_ptr[i * 5 + 3] = (double) tmp_record->link_start_int;
      result_ptr[i * 5 + 4] = tmp_record->gradient ();
      // printf("path ID: %f, departure assign interval (5 s): %f, link ID: %f,
      // time interval (5 s): %f, gradient: %f\n",
      //         result_ptr[i * 5 + 0], result_ptr[i * 5 + 1], result_ptr[i * 5
      //         + 2], result_ptr[i * 5 + 3], result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

SparseMatrixR
Mcdta::get_complete_car_ltg_matrix (py::array_t<int> start_intervals,
                                    int threshold_timestamp, int num_intervals)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mcdta::build_link_cost_map() and
  // Mcdta::get_link_queue_dissipated_time() are invoked already
  int _num_e_path = m_path_vec.size ();
  int _num_e_link = m_link_vec.size ();
  int _assign_interval = int (m_mcdta->m_config->get_int ("assign_frq"));
  std::vector<Eigen::Triplet<double>> _record;
  // pre-allocate sufficient space for dar
  // _record.reserve(int(_num_e_link * num_intervals * _num_e_path *
  // num_intervals));
  _record.reserve (int (1e9));

  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_truck_ltg_matrix, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec)
    {
      // check if the path does not include any link in m_link_vec
      _flg = false;
      for (auto *_link : m_link_vec)
        {
          if (_path->is_link_in (_link->m_link_ID))
            {
              _flg = true;
              break;
            }
        }
      if (!_flg)
        {
          continue;
        }

      for (int t = 0; t < l; ++t)
        {
          // printf("Current processing time: %d\n", t);
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_truck_ltg_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }

          _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1,
          _t_depart_lift_up = -1, _t_depart_prime = -1;
          // trace one additional veh departing from origin of path at
          // start_ptr[t]
          _t_depart = start_ptr[t];
          for (TInt _link_ID : _path->m_link_vec)
            {
              // arrival and departure time of original perturbation vehicle
              _t_arrival = _t_depart;
              _t_depart
                = _t_arrival
                  + MNM_Ults::round_up_time (
                    m_link_tt_map[_link_ID]
                                 [_t_arrival < get_cur_loading_interval ()
                                    ? _t_arrival
                                    : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mcdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                           != nullptr);
                  IAssert (_t_depart_lift_up >= 0); // from its upstream link
                  _t_arrival_lift_up = _t_depart_lift_up;
                }
              IAssert (_t_arrival_lift_up >= _t_arrival);

              IAssert (_link->m_last_valid_time > 0);
              if (_t_arrival_lift_up
                    > int (round (_link->m_last_valid_time - 1))
                  || _t_arrival_lift_up >= threshold_timestamp)
                {
                  break;
                }

              // departure time of new perturbation vehicle
              _t_depart_prime
                = _t_arrival_lift_up
                  + MNM_Ults::round_up_time (
                    m_link_tt_map[_link_ID]
                                 [_t_arrival_lift_up
                                      < get_cur_loading_interval ()
                                    ? _t_arrival_lift_up
                                    : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_car ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_car() / m_mcdta -> m_unit_time;
              _t_depart_lift_up
                = m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_link_congested_car[_link_ID][_t_arrival_lift_up])
                {
                  if (m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                      == _t_arrival_lift_up)
                    {
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               == _t_depart_lift_up);
                    }
                  else
                    {
                      // critical state where subgradient applies
                      IAssert (m_queue_dissipated_time_car[_link_ID]
                                                          [_t_arrival_lift_up]
                               > _t_arrival_lift_up);
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               < _t_depart_lift_up);
                    }
                }
              else
                {
                  IAssert (
                    m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                    > _t_arrival_lift_up);
                  IAssert (_t_arrival_lift_up + MNM_Ults::round_up_time (_fftt)
                           < _t_depart_lift_up);
                }

              if (_t_depart_lift_up < _t_depart_prime)
                {
                  throw std::runtime_error ("invalid state");
                  // _t_depart_lift_up can be equal to _t_depart_prime, when the
                  // arrival curve is horizontal
                }

              auto _itr
                = std::find_if (m_link_vec.begin (), m_link_vec.end (),
                                [&_link_ID] (const MNM_Dlink_Multiclass *_l) {
                                  return _l->m_link_ID == _link_ID;
                                });
              if (_t_depart_prime < get_cur_loading_interval () - 1
                  && _itr != m_link_vec.end ())
                {
                  int link_ind = std::distance (m_link_vec.begin (), _itr);

                  if (m_link_congested_car[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time > 0);
                      if (m_queue_dissipated_time_car[_link_ID]
                                                     [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_queue_dissipated_time_car[_link_ID]
                                                         [_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_link_tt_map[_link_ID]
                                             [_t_queue_dissipated_valid - 1
                                                  < get_cur_loading_interval ()
                                                ? _t_queue_dissipated_valid - 1
                                                : get_cur_loading_interval ()
                                                    - 1]);
                        }
                      // TODO: debug
                      if (_t_depart_lift_up_valid
                          > get_cur_loading_interval () - 1)
                        {
                          _t_depart_lift_up_valid
                            = get_cur_loading_interval () - 1;
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout << "\nError, Mcdta::get_car_ltg_matrix"
                                    << "\n";
                          std::cout << "interval: " << start_ptr[t]
                                    << ", link: " << _link_ID << "\n";
                          std::cout << "car in"
                                    << "\n";
                          std::cout << _link->m_N_in_car->to_string () << "\n";
                          std::cout << "car out"
                                    << "\n";
                          std::cout << _link->m_N_out_car->to_string ()
                                    << "\n\n";
                          std::cout
                            << "last valid time: " << _link->m_last_valid_time
                            << "\n";
                          std::cout << "_t_arrival: " << _t_arrival << "\n";
                          std::cout << "_t_depart: " << _t_depart << "\n";
                          std::cout
                            << "_t_arrival_lift_up: " << _t_arrival_lift_up
                            << "\n";
                          std::cout << "_t_depart_prime: " << _t_depart_prime
                                    << "\n";
                          std::cout
                            << "m_queue_dissipated_time_car[_link_ID][_t_"
                               "arrival_lift_up]: "
                            << m_queue_dissipated_time_car[_link_ID]
                                                          [_t_arrival_lift_up]
                            << "\n";
                          std::cout << "_t_queue_dissipated_valid: "
                                    << _t_queue_dissipated_valid << "\n";
                          std::cout
                            << "_t_depart_lift_up: " << _t_depart_lift_up
                            << "\n";
                          std::cout << "_t_depart_lift_up_valid: "
                                    << _t_depart_lift_up_valid << "\n";
                          std::cout << "_fftt: " << _fftt << "\n";
                          std::cout
                            << "m_link_tt_map[_link_ID][_t_queue_dissipated_"
                               "valid]: "
                            << m_link_tt_map[_link_ID]
                                            [_t_queue_dissipated_valid - 1
                                                 < get_cur_loading_interval ()
                                               ? _t_queue_dissipated_valid - 1
                                               : get_cur_loading_interval ()
                                                   - 1]
                            << "\n";
                          std::cout << "get_cur_loading_interval(): "
                                    << get_cur_loading_interval () << "\n";
                          throw std::runtime_error ("invalid state");
                        }
                      if (_t_depart_prime < _t_depart_lift_up_valid)
                        {
                          _gradient
                            = MNM_DTA_GRADIENT::get_departure_cc_slope_car (
                              _link, TFlt (_t_depart_prime),
                              TFlt (_t_depart_lift_up_valid + 1));
                          if (_gradient > DBL_EPSILON)
                            {
                              _gradient
                                = m_mcdta->m_unit_time / _gradient; // seconds
                              int _tmp
                                = int (_t_arrival_lift_up / _assign_interval);
                              int _ct = 0;
                              for (int t_prime = _t_arrival_lift_up;
                                   t_prime < _t_queue_dissipated_valid;
                                   ++t_prime)
                                {
                                  if (int (t_prime / _assign_interval) == _tmp)
                                    {
                                      _ct += 1;
                                    }
                                  else
                                    {
                                      MNM_DTA_GRADIENT::
                                        add_ltg_records_eigen_veh (
                                          _record, _path, start_ptr[t],
                                          t_prime - 1, link_ind,
                                          _assign_interval, _num_e_link,
                                          _num_e_path, _gradient * _ct);
                                      _ct = 1;
                                      _tmp = int (t_prime / _assign_interval);
                                    }
                                  if (t_prime == _t_queue_dissipated_valid - 1)
                                    {
                                      MNM_DTA_GRADIENT::
                                        add_ltg_records_eigen_veh (
                                          _record, _path, start_ptr[t], t_prime,
                                          link_ind, _assign_interval,
                                          _num_e_link, _num_e_path,
                                          _gradient * _ct);
                                    }

                                  // MNM_DTA_GRADIENT::add_ltg_records_eigen_veh(_record,
                                  // _path, start_ptr[t], t_prime, link_ind,
                                  //                                             _assign_interval, _num_e_link, _num_e_path, _gradient);
                                }
                            }

                          // for (int t_prime = _t_arrival_lift_up; t_prime <
                          // _t_queue_dissipated_valid; ++t_prime) {
                          //     _gradient =
                          //     MNM_DTA_GRADIENT::get_departure_cc_slope_car(_link,
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mcdta -> m_unit_time /
                          //         _gradient;  // seconds
                          //         MNM_DTA_GRADIENT::add_ltg_records_eigen_veh(_record,
                          //         _path, start_ptr[t], t_prime, link_ind,
                          //                                                     _assign_interval, _num_e_link, _num_e_path, _gradient);
                          //     }
                          // }
                        }
                    }
                }
            }
        }
    }

  // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // dar matrix rho
  SparseMatrixR mat (num_intervals * _num_e_link, num_intervals * _num_e_path);
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#acc35051d698e3973f1de5b9b78dbe345
  mat.setFromTriplets (_record.begin (), _record.end ());
  return mat;
}

SparseMatrixR
Mcdta::get_complete_truck_ltg_matrix (py::array_t<int> start_intervals,
                                      int threshold_timestamp,
                                      int num_intervals)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mcdta::build_link_cost_map() and
  // Mcdta::get_link_queue_dissipated_time() are invoked already
  int _num_e_path = m_path_vec.size ();
  int _num_e_link = m_link_vec.size ();
  int _assign_interval = int (m_mcdta->m_config->get_int ("assign_frq"));
  std::vector<Eigen::Triplet<double>> _record;
  // pre-allocate sufficient space for dar
  // _record.reserve(int(_num_e_link * num_intervals * _num_e_path *
  // num_intervals));
  _record.reserve (int (1e9));

  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mcdta::get_complete_truck_ltg_matrix, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec)
    {
      // check if the path does not include any link in m_link_vec
      _flg = false;
      for (auto *_link : m_link_vec)
        {
          if (_path->is_link_in (_link->m_link_ID))
            {
              _flg = true;
              break;
            }
        }
      if (!_flg)
        {
          continue;
        }

      for (int t = 0; t < l; ++t)
        {
          // printf("Current processing time: %d\n", t);
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mcdta::get_complete_truck_ltg_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }

          _t_arrival = -1, _t_depart = -1, _t_arrival_lift_up = -1,
          _t_depart_lift_up = -1, _t_depart_prime = -1;
          // trace one additional veh departing from origin of path at
          // start_ptr[t]
          _t_depart = start_ptr[t];
          for (TInt _link_ID : _path->m_link_vec)
            {
              // arrival and departure time of original perturbation vehicle
              _t_arrival = _t_depart;
              _t_depart
                = _t_arrival
                  + MNM_Ults::round_up_time (
                    m_link_tt_map_truck[_link_ID]
                                       [_t_arrival < get_cur_loading_interval ()
                                          ? _t_arrival
                                          : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mcdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multiclass *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multiclass *> (_link)
                           != nullptr);
                  IAssert (_t_depart_lift_up >= 0); // from its upstream link
                  _t_arrival_lift_up = _t_depart_lift_up;
                }
              IAssert (_t_arrival_lift_up >= _t_arrival);

              IAssert (_link->m_last_valid_time > 0);
              if (_t_arrival_lift_up
                    > int (round (_link->m_last_valid_time - 1))
                  || _t_arrival_lift_up >= threshold_timestamp)
                {
                  break;
                }

              // departure time of new perturbation vehicle
              _t_depart_prime
                = _t_arrival_lift_up
                  + MNM_Ults::round_up_time (
                    m_link_tt_map_truck[_link_ID]
                                       [_t_arrival_lift_up
                                            < get_cur_loading_interval ()
                                          ? _t_arrival_lift_up
                                          : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mcdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_truck ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mcdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_truck() / m_mcdta -> m_unit_time;
              _t_depart_lift_up
                = m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_link_congested_truck[_link_ID][_t_arrival_lift_up])
                {
                  if (m_queue_dissipated_time_truck[_link_ID]
                                                   [_t_arrival_lift_up]
                      == _t_arrival_lift_up)
                    {
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               == _t_depart_lift_up);
                    }
                  else
                    {
                      // critical state where subgradient applies
                      IAssert (m_queue_dissipated_time_truck[_link_ID]
                                                            [_t_arrival_lift_up]
                               > _t_arrival_lift_up);
                      IAssert (_t_arrival_lift_up
                                 + MNM_Ults::round_up_time (_fftt)
                               < _t_depart_lift_up);
                    }
                }
              else
                {
                  IAssert (
                    m_queue_dissipated_time_truck[_link_ID][_t_arrival_lift_up]
                    > _t_arrival_lift_up);
                  IAssert (_t_arrival_lift_up + MNM_Ults::round_up_time (_fftt)
                           < _t_depart_lift_up);
                }

              if (_t_depart_lift_up < _t_depart_prime)
                {
                  throw std::runtime_error ("invalid state");
                  // _t_depart_lift_up can be equal to _t_depart_prime, when the
                  // arrival curve is horizontal
                }

              auto _itr
                = std::find_if (m_link_vec.begin (), m_link_vec.end (),
                                [&_link_ID] (const MNM_Dlink_Multiclass *_l) {
                                  return _l->m_link_ID == _link_ID;
                                });
              if (_t_depart_prime < get_cur_loading_interval () - 1
                  && _itr != m_link_vec.end ())
                {
                  int link_ind = std::distance (m_link_vec.begin (), _itr);

                  if (m_link_congested_truck[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time > 0);
                      if (m_queue_dissipated_time_truck[_link_ID]
                                                       [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_queue_dissipated_time_truck[_link_ID]
                                                           [_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_link_tt_map_truck
                                  [_link_ID]
                                  [_t_queue_dissipated_valid - 1
                                       < get_cur_loading_interval ()
                                     ? _t_queue_dissipated_valid - 1
                                     : get_cur_loading_interval () - 1]);
                        }
                      // TODO: debug
                      if (_t_depart_lift_up_valid
                          > get_cur_loading_interval () - 1)
                        {
                          _t_depart_lift_up_valid
                            = get_cur_loading_interval () - 1;
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout << "\nError, Mcdta::get_truck_ltg_matrix"
                                    << "\n";
                          std::cout << "interval: " << start_ptr[t]
                                    << ", link: " << _link_ID << "\n";
                          std::cout << "truck in"
                                    << "\n";
                          std::cout << _link->m_N_in_truck->to_string ()
                                    << "\n";
                          std::cout << "truck out"
                                    << "\n";
                          std::cout << _link->m_N_out_truck->to_string ()
                                    << "\n\n";
                          std::cout
                            << "last valid time: " << _link->m_last_valid_time
                            << "\n";
                          std::cout << "_t_arrival: " << _t_arrival << "\n";
                          std::cout << "_t_depart: " << _t_depart << "\n";
                          std::cout
                            << "_t_arrival_lift_up: " << _t_arrival_lift_up
                            << "\n";
                          std::cout << "_t_depart_prime: " << _t_depart_prime
                                    << "\n";
                          std::cout
                            << "m_queue_dissipated_time_truck[_link_ID][_t_"
                               "arrival_lift_up]: "
                            << m_queue_dissipated_time_truck[_link_ID]
                                                            [_t_arrival_lift_up]
                            << "\n";
                          std::cout << "_t_queue_dissipated_valid: "
                                    << _t_queue_dissipated_valid << "\n";
                          std::cout
                            << "_t_depart_lift_up: " << _t_depart_lift_up
                            << "\n";
                          std::cout << "_t_depart_lift_up_valid: "
                                    << _t_depart_lift_up_valid << "\n";
                          std::cout << "_fftt: " << _fftt << "\n";
                          std::cout
                            << "m_link_tt_map_truck[_link_ID][_t_queue_"
                               "dissipated_valid]: "
                            << m_link_tt_map_truck
                                 [_link_ID][_t_queue_dissipated_valid - 1
                                                < get_cur_loading_interval ()
                                              ? _t_queue_dissipated_valid - 1
                                              : get_cur_loading_interval () - 1]
                            << "\n";
                          std::cout << "get_cur_loading_interval(): "
                                    << get_cur_loading_interval () << "\n";
                          throw std::runtime_error ("invalid state");
                        }
                      if (_t_depart_prime < _t_depart_lift_up_valid)
                        {
                          _gradient
                            = MNM_DTA_GRADIENT::get_departure_cc_slope_truck (
                              _link, TFlt (_t_depart_prime),
                              TFlt (_t_depart_lift_up_valid + 1));
                          if (_gradient > DBL_EPSILON)
                            {
                              _gradient
                                = m_mcdta->m_unit_time / _gradient; // seconds
                              int _tmp
                                = int (_t_arrival_lift_up / _assign_interval);
                              int _ct = 0;
                              for (int t_prime = _t_arrival_lift_up;
                                   t_prime < _t_queue_dissipated_valid;
                                   ++t_prime)
                                {
                                  if (int (t_prime / _assign_interval) == _tmp)
                                    {
                                      _ct += 1;
                                    }
                                  else
                                    {
                                      MNM_DTA_GRADIENT::
                                        add_ltg_records_eigen_veh (
                                          _record, _path, start_ptr[t],
                                          t_prime - 1, link_ind,
                                          _assign_interval, _num_e_link,
                                          _num_e_path, _gradient * _ct);
                                      _ct = 1;
                                      _tmp = int (t_prime / _assign_interval);
                                    }
                                  if (t_prime == _t_queue_dissipated_valid - 1)
                                    {
                                      MNM_DTA_GRADIENT::
                                        add_ltg_records_eigen_veh (
                                          _record, _path, start_ptr[t], t_prime,
                                          link_ind, _assign_interval,
                                          _num_e_link, _num_e_path,
                                          _gradient * _ct);
                                    }

                                  // MNM_DTA_GRADIENT::add_ltg_records_eigen_veh(_record,
                                  // _path, start_ptr[t], t_prime, link_ind,
                                  //                                             _assign_interval, _num_e_link, _num_e_path, _gradient);
                                }
                            }

                          // for (int t_prime = _t_arrival_lift_up; t_prime <
                          // _t_queue_dissipated_valid; ++t_prime) {
                          //     _gradient =
                          //     MNM_DTA_GRADIENT::get_departure_cc_slope_truck(_link,
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mcdta -> m_unit_time /
                          //         _gradient;  // seconds
                          //         MNM_DTA_GRADIENT::add_ltg_records_eigen_veh(_record,
                          //         _path, start_ptr[t], t_prime, link_ind,
                          //                                                     _assign_interval, _num_e_link, _num_e_path, _gradient);
                          //     }
                          // }
                        }
                    }
                }
            }
        }
    }

  // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
  // dar matrix rho
  SparseMatrixR mat (num_intervals * _num_e_link, num_intervals * _num_e_path);
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SparseMatrix.html#acc35051d698e3973f1de5b9b78dbe345
  mat.setFromTriplets (_record.begin (), _record.end ());
  return mat;
}

int
Mcdta::delete_all_agents ()
{
  // invoke it after simulation, may help save some memory
  delete m_mcdta->m_veh_factory;
  m_mcdta->m_veh_factory = nullptr;
  return 0;
}

}
}
