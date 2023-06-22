// Single class DTA

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <unordered_map>
#include <vector>

#include "utils.h"
#include <Snap.h>
#include <dso.h>
#include <dta.h>
#include <due.h>
#include <multimodal.h>
#include <delivery_traffic.h>
#include <ev_traffic.h>

namespace py = pybind11;
using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

namespace macposts
{
namespace dta
{
class Dta
{
public:
  Dta ();
  ~Dta ();
  int initialize (const std::string &folder);
  bool check_input_files ();
  int generate_shortest_pathsets (const std::string &folder, int max_iter,
                                  double vot, double mid_scale,
                                  double heavy_scale, double min_path_tt = 0.);
  int install_cc ();
  int install_cc_tree ();
  int run_whole (bool verbose = false);
  int run_due (int max_iter, const std::string &folder, bool verbose = true,
               bool with_dtc = false, const std::string &method = "MSA");
  int run_dso (int max_iter, const std::string &folder, bool verbose = true,
               bool with_dtc = false, const std::string &method = "MSA");
  int run_dnl_delivery_traffic(const std::string &folder, bool verbose = false, bool skip_check = false, int cong_frequency = 180);
  int run_dnl_electrified_traffic(const std::string &folder, bool verbose = false, bool skip_check = false, int cong_frequency = 180);
  py::array_t<double> get_travel_stats ();
  std::string print_emission_stats ();
  int print_simulation_results (const std::string &folder,
                                int cong_frequency = 180);
  // FIXME: This returns a Numpy array for consistency, but it should really be
  // better to use a plain list.
  py::array_t<int> get_all_links ();
  int register_links (py::array_t<int> links);
  // FIXME: This returns a Numpy array for consistency, but it should really be
  // better to use a plain list.
  py::array_t<int> get_registered_links ();
  int register_paths (py::array_t<int> paths);
  std::vector<bool> check_registered_links_in_registered_paths ();
  py::array_t<bool> are_registered_links_in_registered_paths ();
  py::array_t<int> generate_paths_to_cover_registered_links ();
  int save_path_table (const std::string &folder);
  int get_cur_loading_interval ();
  int build_link_cost_map (bool with_congestion_indicator = false);
  int get_link_queue_dissipated_time ();
  py::array_t<double> get_link_inflow (py::array_t<int> start_intervals,
                                       py::array_t<int> end_intervals);
  py::array_t<double> get_link_tt_FD (py::array_t<int> start_intervals);
  py::array_t<double> get_link_tt (py::array_t<int> start_intervals,
                                   bool return_inf = false);
  py::array_t<double> get_link_tt_robust (py::array_t<double> start_intervals,
                                          py::array_t<double> end_intervals,
                                          int num_trials = 180,
                                          bool return_inf = false);

  // assume build_link_cost_map() is invoked before
  py::array_t<double> get_path_tt (py::array_t<int> link_IDs,
                                   py::array_t<int> start_intervals);
  py::array_t<double> get_registered_path_tt (py::array_t<int> start_intervals);

  py::array_t<double> get_link_in_cc (int link_ID);
  py::array_t<double> get_link_out_cc (int link_ID);
  py::array_t<double> get_dar_matrix (py::array_t<int> link_start_intervals,
                                      py::array_t<int> link_end_intervals);
  int save_dar_matrix (py::array_t<int> link_start_intervals,
                       py::array_t<int> link_end_intervals,
                       py::array_t<double> f, const std::string &file_name);
  SparseMatrixR get_complete_dar_matrix (py::array_t<int> start_intervals,
                                         py::array_t<int> end_intervals,
                                         int num_intervals,
                                         py::array_t<double> f);

  int delete_all_agents ();

  MNM_Dta *m_dta;
  std::vector<MNM_Dlink *> m_link_vec;
  std::vector<MNM_Path *> m_path_vec;
  std::unordered_map<MNM_Path *, int> m_path_map;
  // std::unordered_map<MNM_Dlink*, int> m_link_map;
  std::unordered_map<TInt, MNM_Path *> m_ID_path_mapping;

  std::unordered_map<TInt, TFlt *> m_link_tt_map;
  std::unordered_map<TInt, TFlt *> m_link_cost_map;
  std::unordered_map<TInt, bool *> m_link_congested;

  // time-varying queue dissipated time
  std::unordered_map<TInt, int *> m_queue_dissipated_time;
};

void
init (py::module &m)
{
  py::class_<Dta> (m, "Dta")
    .def (py::init<> ())
    .def ("initialize", &Dta::initialize)
    .def ("check_input_files", &Dta::check_input_files)
    .def ("run_whole", &Dta::run_whole, py::arg ("verbose") = false)
    .def ("run_due", &Dta::run_due)
    .def ("run_dso", &Dta::run_dso)
    .def("run_dnl_delivery_traffic", &Dta::run_dnl_delivery_traffic)
    .def("run_dnl_electrified_traffic", &Dta::run_dnl_electrified_traffic)
    .def ("install_cc", &Dta::install_cc)
    .def ("install_cc_tree", &Dta::install_cc_tree)
    .def ("get_travel_stats", &Dta::get_travel_stats)
    .def ("print_emission_stats", &Dta::print_emission_stats)
    .def ("get_cur_loading_interval", &Dta::get_cur_loading_interval)
    .def ("print_simulation_results", &Dta::print_simulation_results)
    .def_property_readonly ("links", &Dta::get_all_links, "IDs of all links.")
    .def ("register_links", &Dta::register_links)
    .def_property_readonly ("registered_links", &Dta::get_registered_links,
                            "IDs of previously registered links.")
    .def ("register_paths", &Dta::register_paths)
    .def ("are_registered_links_in_registered_paths",
          &Dta::are_registered_links_in_registered_paths)
    .def ("generate_paths_to_cover_registered_links",
          &Dta::generate_paths_to_cover_registered_links)
    .def ("get_link_tt_FD", &Dta::get_link_tt_FD)
    .def ("get_link_tt", &Dta::get_link_tt)
    .def ("get_link_tt_robust", &Dta::get_link_tt_robust)

    .def ("build_link_cost_map", &Dta::build_link_cost_map)
    // with build_link_cost_map()
    .def ("get_path_tt", &Dta::get_path_tt)
    .def ("get_registered_path_tt", &Dta::get_registered_path_tt)

    .def ("get_link_inflow", &Dta::get_link_inflow)
    .def ("get_link_in_cc", &Dta::get_link_in_cc)
    .def ("get_link_out_cc", &Dta::get_link_out_cc)
    .def ("get_dar_matrix", &Dta::get_dar_matrix)
    .def ("get_complete_dar_matrix", &Dta::get_complete_dar_matrix)
    .def ("save_dar_matrix", &Dta::save_dar_matrix)
    .def ("delete_all_agents", &Dta::delete_all_agents);
}

Dta::Dta ()
{
  m_dta = nullptr;
  m_link_vec = std::vector<MNM_Dlink *> ();
  m_path_vec = std::vector<MNM_Path *> ();
  m_path_map = std::unordered_map<MNM_Path *, int> ();
  m_ID_path_mapping = std::unordered_map<TInt, MNM_Path *> ();
  // m_link_map = std::unordered_map<MNM_Dlink*, int>();

  m_link_tt_map = std::unordered_map<TInt, TFlt *> ();
  m_link_cost_map = std::unordered_map<TInt, TFlt *> ();
  m_link_congested = std::unordered_map<TInt, bool *> ();

  m_queue_dissipated_time = std::unordered_map<TInt, int *> ();
}

Dta::~Dta ()
{
  if (m_dta != nullptr)
    {
      delete m_dta;
    }
  m_link_vec.clear ();
  m_path_vec.clear ();
  // m_link_map.clear();
  m_ID_path_mapping.clear ();

  for (auto _tt_it : m_link_tt_map)
    {
      delete _tt_it.second;
    }
  m_link_tt_map.clear ();

  for (auto _cost_it : m_link_cost_map)
    {
      delete _cost_it.second;
    }
  m_link_cost_map.clear ();

  for (auto _it : m_link_congested)
    {
      delete _it.second;
    }
  m_link_congested.clear ();

  for (auto _it : m_queue_dissipated_time)
    {
      delete _it.second;
    }
  m_queue_dissipated_time.clear ();
}

int
Dta::initialize (const std::string &folder)
{
  m_dta = new MNM_Dta (folder);
  m_dta->build_from_files ();
  m_dta->hook_up_node_and_link ();
  m_dta->is_ok ();
  Assert (m_dta->m_config->get_string ("routing_type") == "Due"
           || m_dta->m_config->get_string ("routing_type") == "Hybrid"
           || m_dta->m_config->get_string ("routing_type") == "Fixed"
           || m_dta->m_config->get_string ("routing_type") == "Adaptive");
  // printf("start load ID path mapping 0\n");
  if (MNM_Routing_Fixed *_routing
      = dynamic_cast<MNM_Routing_Fixed *> (m_dta->m_routing))
    {
      MNM::get_ID_path_mapping (m_ID_path_mapping, _routing->m_path_table);
      return 0;
    }
  if (MNM_Routing_Hybrid *_routing
      = dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing))
    {
      // printf("start load ID path mapping\n");
      MNM::get_ID_path_mapping (m_ID_path_mapping,
                                _routing->m_routing_fixed->m_path_table);
      // printf("mapping size %d\n", m_ID_path_mapping.size());
      return 0;
    }
  // TODO: Check if the other routing types are really unsupported -- they work
  // fine in our test cases at least.
  return -1;
}

bool
Dta::check_input_files ()
{
  return m_dta->is_ok ();
}

int
Dta::generate_shortest_pathsets (const std::string &folder, int max_iter,
                                 double vot, double mid_scale,
                                 double heavy_scale, double min_path_tt)
{
  m_dta = new MNM_Dta (folder);
  m_dta->build_from_files ();
  m_dta->hook_up_node_and_link ();

  Path_Table *_driving_path_table
    = MNM::build_pathset (m_dta->m_graph, m_dta->m_od_factory,
                          m_dta->m_link_factory, min_path_tt, max_iter, vot,
                          mid_scale, heavy_scale,
                          m_dta->m_config->get_int ("max_interval"));
  printf ("driving pathset generated\n");
  MNM::save_driving_path_table (folder, _driving_path_table, "path_table",
                                "path_table_buffer", true);
  printf ("driving pathset saved\n");

  if (!_driving_path_table->empty ())
    {
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
    }
  delete _driving_path_table;

  return 0;
}

int
Dta::install_cc ()
{
  // for (size_t i = 0; i < m_link_vec.size (); ++i)
  //   {
  //     m_link_vec[i]->install_cumulative_curve ();
  //   }
  for (auto _link_it : m_dta -> m_link_factory -> m_link_map) {
    _link_it.second -> install_cumulative_curve();
  }
  return 0;
}

int
Dta::install_cc_tree ()
{
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      m_link_vec[i]->install_cumulative_curve_tree ();
    }
  return 0;
}

int
Dta::run_whole (bool verbose)
{
  m_dta->pre_loading ();
  m_dta->loading (verbose);
  return 0;
}

int
Dta::run_due (int max_iter, const std::string &folder, bool verbose,
              bool with_dtc, const std::string &method)
{
  Assert (m_dta == nullptr);
  MNM_ConfReader *_config
    = new MNM_ConfReader (folder + "/config.conf", "STAT");
  std::string _rec_folder = _config->get_string ("rec_folder");
  delete _config;

  MNM_Due *_due = new MNM_Due_Msa (folder);
  _due->initialize (); // create and set m_buffer[i] = 0
  _due->init_path_flow ();

  std::string _gap_file_name = folder + "/" + _rec_folder + "/gap_iteration";
  std::ofstream _gap_file;
  _gap_file.open (_gap_file_name, std::ofstream::out);
  if (!_gap_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + _gap_file_name);
    }

  TFlt _gap, _total_tt;
  for (int i = 0; i < max_iter; ++i)
    {
      printf ("---------- Iteration %d ----------\n", i);

      // DNL using dta.cpp, new dta is built from scratch
      m_dta = _due->run_dta (verbose);

      // time-dependent link cost
      build_link_cost_map (false);
      // TODO: whole map will be copied
      _due->m_link_tt_map = m_link_tt_map;
      _due->m_link_cost_map = m_link_cost_map;
      // _due -> build_link_cost_map(m_dta);

      _total_tt = _due->compute_total_travel_time ();

      _due->update_path_table_cost (m_dta);

      // path flow are saved in _rec_folder
      // MNM::print_path_table(_due -> m_path_table, m_dta->m_od_factory, true,
      // true);
      MNM::save_path_table (folder + "/" + _rec_folder, _due->m_path_table,
                            m_dta->m_od_factory, true, true);

      // calculate gap
      if (with_dtc)
        {
          // with departure time choice
          _gap = _due->compute_merit_function ();
        }
      else
        {
          // fixed departure time choice
          _gap = _due->compute_merit_function_fixed_departure_time_choice ();
        }

      printf ("GAP = %lf, total tt = %lf\n", (float) _gap, (float) _total_tt);
      _gap_file << std::to_string (_gap) + " " + std::to_string (_total_tt)
                     + "\n";

      // search for the lowest disutility route and update path flow
      if (with_dtc)
        {
          // with departure time choice
          _due->update_path_table (m_dta, i);
        }
      else
        {
          if (method == "MSA")
            {
              // fixed departure time choice
              _due->update_path_table_fixed_departure_time_choice (m_dta, i);
            }
          else if (method == "GP")
            {
              // gradient projection
              _due->update_path_table_gp_fixed_departure_time_choice (m_dta, i);
            }
          else
            {
              throw std::runtime_error ("unsupported method");
            }
        }

      dynamic_cast<MNM_Routing_Fixed *> (m_dta->m_routing)->m_path_table
        = nullptr;
      delete m_dta;
      m_dta = nullptr;
    }

  _gap_file.close ();

  _due->m_link_tt_map = std::unordered_map<TInt, TFlt *> ();
  _due->m_link_cost_map = std::unordered_map<TInt, TFlt *> ();
  delete _due;
  printf ("Dta::run_due, finished\n");
  return 0;
}

int
Dta::run_dso (int max_iter, const std::string &folder, bool verbose,
              bool with_dtc, const std::string &method)
{
  Assert (m_dta == nullptr);
  MNM_ConfReader *_config
    = new MNM_ConfReader (folder + "/config.conf", "STAT");
  std::string _rec_folder = _config->get_string ("rec_folder");
  delete _config;

  MNM_Dso *_dso = new MNM_Dso (folder);
  _dso->initialize (); // create and set m_buffer[i] = 0
  _dso->init_path_flow ();

  std::string _gap_file_name = folder + "/" + _rec_folder + "/gap_iteration";
  std::ofstream _gap_file;
  _gap_file.open (_gap_file_name, std::ofstream::out);
  if (!_gap_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + _gap_file_name);
    }

  TFlt _gap, _total_tt;
  for (int i = 0; i < max_iter; ++i)
    {
      printf ("---------- Iteration %d ----------\n", i);

      // DNL using dta.cpp, new dta is built from scratch
      m_dta = _dso->run_dta (verbose);

      // time-dependent link cost
      build_link_cost_map (true);
      // TODO: whole map will be copied
      _dso->m_link_tt_map = m_link_tt_map;
      _dso->m_link_cost_map = m_link_cost_map;
      _dso->m_link_congested = m_link_congested;
      // _dso -> build_link_cost_map(m_dta);

      _total_tt = _dso->compute_total_travel_time ();

      _dso->get_link_marginal_cost (m_dta);

      _dso->update_path_table_cost (m_dta);

      // path flow are saved in _rec_folder
      // MNM::print_path_table(_dso -> m_path_table, m_dta->m_od_factory, true,
      // true);
      MNM::save_path_table (folder + "/" + _rec_folder, _dso->m_path_table,
                            m_dta->m_od_factory, true, true);

      // calculate gap
      if (with_dtc)
        {
          // with departure time choice
          _gap = _dso->compute_merit_function ();
        }
      else
        {
          // fixed departure time choice
          _gap = _dso->compute_merit_function_fixed_departure_time_choice ();
        }

      printf ("GAP = %lf, total tt = %lf\n", (float) _gap, (float) _total_tt);
      _gap_file << std::to_string (_gap) + " " + std::to_string (_total_tt)
                     + "\n";

      // search for the lowest disutility route and update path flow
      if (with_dtc)
        {
          // with departure time choice
          _dso->update_path_table (m_dta, i);
        }
      else
        {
          if (method == "MSA")
            {
              // fixed departure time choice
              _dso->update_path_table_fixed_departure_time_choice (m_dta, i);
            }
          else if (method == "GP")
            {
              // gradient projection
              _dso->update_path_table_gp_fixed_departure_time_choice (m_dta, i);
            }
          else
            {
              throw std::runtime_error ("unsupported method");
            }
        }

      dynamic_cast<MNM_Routing_Fixed *> (m_dta->m_routing)->m_path_table
        = nullptr;
      delete m_dta;
      m_dta = nullptr;
    }

  _gap_file.close ();

  _dso->m_link_tt_map = std::unordered_map<TInt, TFlt *> ();
  _dso->m_link_cost_map = std::unordered_map<TInt, TFlt *> ();
  _dso->m_link_congested = std::unordered_map<TInt, bool *> ();
  delete _dso;
  printf ("Dta::run_dso, finished\n");
  return 0;
}

int 
Dta::run_dnl_delivery_traffic(const std::string &folder, bool verbose, bool skip_check, int cong_frequency)
{
  Assert(m_dta == nullptr);
  MNM_ConfReader *_config = new MNM_ConfReader(folder + "/config.conf", "STAT");
  std::string _rec_folder = _config -> get_string("rec_folder");
  delete _config;

  m_dta = new MNM_Dta_Delivery(folder);
  printf("================================ DTA set! =================================\n");
  m_dta -> build_from_files();
  printf("========================= Finished initialization! ========================\n");
  m_dta -> hook_up_node_and_link();
  MNM_Dlink *_link;
	for (auto _link_it : m_dta -> m_link_factory -> m_link_map){
		_link = _link_it.second;
		_link -> install_cumulative_curve();
	}
  printf("====================== Finished node and link hook-up! ====================\n");
  if (!skip_check) {
    m_dta -> is_ok();
    printf("============================ DTA is OK to run! ============================\n");
  }
  m_dta -> pre_loading();
  printf("========================== Finished pre_loading! ==========================\n");

	printf("\n\n\n====================================== Start loading! =======================================\n");
  m_dta -> loading(verbose);
  printf("\n====================================== Finished loading! =======================================\n\n\n");

  // Output total travels and travel time, before divided by flow_scalar
	TInt _finished_car, _released_car, _enroute_car, _released_delivery_car;
	TFlt _tot_tt_car;
	MNM_Veh_Factory_Delivery *_veh_factory = dynamic_cast<MNM_Veh_Factory_Delivery*>(m_dta -> m_veh_factory);

	_finished_car = _veh_factory -> m_finished;
	_released_car = _veh_factory -> m_num_veh;
	_enroute_car = _veh_factory -> m_enroute;
	_released_delivery_car = _veh_factory -> m_veh_delivery;
	_tot_tt_car = _veh_factory -> m_total_time * m_dta -> m_unit_time / 3600.0;
	for (auto _map_it : m_dta -> m_veh_factory -> m_veh_map){
		if (_map_it.second -> m_finish_time > 0) {
			throw std::runtime_error("Finished vehicles are deleted and remaining vehicles are all enroute");
		}
		else {
			_tot_tt_car += (m_dta -> m_current_loading_interval - _map_it.second -> m_start_time) * m_dta -> m_unit_time / 3600.0;
		}
	}
  // divided by flow_scalar
  _finished_car = _finished_car / m_dta -> m_flow_scalar;
  _released_car = _released_car / m_dta -> m_flow_scalar;
  _enroute_car = _enroute_car / m_dta -> m_flow_scalar;
  _released_delivery_car = _released_delivery_car / m_dta -> m_flow_scalar;
  _tot_tt_car = _tot_tt_car / m_dta -> m_flow_scalar;

  std::string _str;
	std::ofstream _vis_file;
	_vis_file.open(folder + "/" + _rec_folder + "/statistics_and_emission.txt", std::ofstream::out);
	if (! _vis_file.is_open()){
		throw std::runtime_error("Error happens when open _vis_file\n");
	}
	_str = "Total released car: " + std::to_string(int(_released_car)) + "\n" 
	       + "Total finished car: " + std::to_string(int(_finished_car)) + "\n" 
		     + "Total enroute car: " + std::to_string(int(_enroute_car)) + "\n" 
		     + "Total released delivery car: " + std::to_string(int(_released_delivery_car)) + "\n"
		     + "Total car tt: " + std::to_string(float(_tot_tt_car)) + " hours\n\n";

	_str += print_emission_stats();
	std::cout << _str << std::endl;
	_vis_file << _str;
	if (_vis_file.is_open()) _vis_file.close();

  print_simulation_results(folder + "/" + _rec_folder, cong_frequency);

  printf("Finished DNL!\n");
  return 0;
}

int 
Dta::run_dnl_electrified_traffic(const std::string &folder, bool verbose, bool skip_check, int cong_frequency)
{
  Assert (m_dta == nullptr);
  MNM_ConfReader *_config = new MNM_ConfReader(folder + "/config.conf", "STAT");
  std::string _rec_folder = _config -> get_string("rec_folder");
	delete _config;

	m_dta = new MNM_Dta_EV(folder);
	printf("================================ DTA set! =================================\n");
	m_dta -> build_from_files();
	printf("========================= Finished initialization! ========================\n");
	m_dta -> hook_up_node_and_link();

  MNM_Dlink *_link;
	for (auto _link_it : m_dta -> m_link_factory -> m_link_map){
		_link = _link_it.second;
		_link -> install_cumulative_curve();
	}
	printf("====================== Finished node and link hook-up! ====================\n");

  if (!skip_check) {
    m_dta -> is_ok();
    printf("============================ DTA is OK to run! ============================\n");
  }

	m_dta -> pre_loading();
	printf("========================== Finished pre_loading! ==========================\n");

	printf("\n\n\n====================================== Start loading! =======================================\n");
  m_dta -> loading(verbose);
  printf("\n====================================== Finished loading! =======================================\n\n\n");

	// Output total travels and travel time, before divided by flow_scalar
	TInt _finished_car, _released_car, _enroute_car, _released_delivery_car, _released_electrified_car;
	TFlt _tot_tt_car;
	MNM_Veh_Factory_EV *_veh_factory = dynamic_cast<MNM_Veh_Factory_EV*>(m_dta -> m_veh_factory);

	_finished_car = _veh_factory -> m_finished;
	_released_car = _veh_factory -> m_num_veh;
	_enroute_car = _veh_factory -> m_enroute;
	_released_delivery_car = _veh_factory -> m_veh_delivery;
	_released_electrified_car = _veh_factory -> m_veh_electrified;
	_tot_tt_car = _veh_factory -> m_total_time * m_dta -> m_unit_time / 3600.0;
	for (auto _map_it : m_dta -> m_veh_factory -> m_veh_map){
		if (_map_it.second -> m_finish_time > 0) {
			throw std::runtime_error("Finished vehicles are deleted and remaining vehicles are all enroute");
		}
		else {
			_tot_tt_car += (m_dta -> m_current_loading_interval - _map_it.second -> m_start_time) * m_dta -> m_unit_time / 3600.0;
		}
	}
  // divided by flow_scalar
  _finished_car = _finished_car / m_dta -> m_flow_scalar;
  _released_car = _released_car / m_dta -> m_flow_scalar;
  _enroute_car = _enroute_car / m_dta -> m_flow_scalar;
  _released_delivery_car = _released_delivery_car / m_dta -> m_flow_scalar;
  _released_electrified_car = _released_electrified_car / m_dta -> m_flow_scalar;
  _tot_tt_car = _tot_tt_car / m_dta -> m_flow_scalar;

  std::string _str;
	std::ofstream _vis_file;
	_vis_file.open(folder + "/" + _rec_folder + "/statistics_and_emission.txt", std::ofstream::out);
	if (! _vis_file.is_open()){
		throw std::runtime_error("Error happens when open _vis_file\n");
	}
	_str = "Total released car: " + std::to_string(int(_released_car)) + "\n" 
	      + "Total finished car: " + std::to_string(int(_finished_car)) + "\n" 
        + "Total enroute car: " + std::to_string(int(_enroute_car)) + "\n" 
        + "Total released delivery car: " + std::to_string(int(_released_delivery_car)) + "\n"
        + "Total released electrified car: " + std::to_string(int(_released_electrified_car)) + "\n"
        + "Total car tt: " + std::to_string(float(_tot_tt_car)) + " hours\n\n";

	_str += print_emission_stats();
	std::cout << _str << std::endl;
	_vis_file << _str;
	if (_vis_file.is_open()) _vis_file.close();

  print_simulation_results(folder + "/" + _rec_folder, cong_frequency);

	// std::ofstream _vis_file2;
	// if (output_link_cong){
	// 	_vis_file2.open(folder + "/" + _rec_folder + "/link_cong_raw.txt", std::ofstream::out);
	// 	if (! _vis_file2.is_open()){
  //     throw std::runtime_error("Error happens when open _vis_file2\n");
  //   }
	// 	TInt _iter = 0;
	// 	_str = "timestamp(intervals) link_ID car_inflow car_tt(s) car_fftt(s) car_speed(mph)\n";
	// 	_vis_file2 << _str;
  //   while (_iter < m_dta -> m_current_loading_interval){
  //       if (_iter % cong_frequency == 0){
  //           // printf("Current loading interval: %d\n", int(_iter));
  //           for (auto _link_it : m_dta -> m_link_factory -> m_link_map){
  //               _link = _link_it.second;
  //               _str = std::to_string(int(_iter)) + " ";
  //               _str += std::to_string(_link -> m_link_ID()) + " ";
  //               _str += std::to_string(MNM_DTA_GRADIENT::get_link_inflow(_link, _iter, _iter + cong_frequency)) + " ";
  //               _str += std::to_string(MNM_DTA_GRADIENT::get_travel_time(_link, TFlt(_iter + 1), m_dta -> m_unit_time, m_dta -> m_current_loading_interval) * m_dta -> m_unit_time) + " ";
  //               _str += std::to_string(_link -> get_link_freeflow_tt()) + " ";
  //               _str += std::to_string(_link -> m_length/(MNM_DTA_GRADIENT::get_travel_time(_link, TFlt(_iter + 1), m_dta -> m_unit_time, m_dta -> m_current_loading_interval) * m_dta -> m_unit_time) * 3600 / 1600) + "\n";
  //               _vis_file2 << _str;
  //           }
  //       }
  //       _iter += 1;
  //   }
	// 	if (_vis_file2.is_open()) _vis_file2.close();
	// }

	MNM_IO_EV::save_charging_station_record(folder + "/" + _rec_folder, m_dta ->m_node_factory);

  printf("Finished DNL!\n");
  return 0;
}

int
Dta::get_cur_loading_interval ()
{
  return m_dta->m_current_loading_interval ();
}

py::array_t<int>
Dta::get_all_links ()
{
  if (!m_dta)
    throw std::runtime_error ("DTA uninitialized");
  auto &link_map = m_dta->m_link_factory->m_link_map;
  auto results = py::array_t<int> (link_map.size ());
  auto results_buf = results.request ();
  int *results_ptr = static_cast<int *> (results_buf.ptr);
  int idx = 0;
  for (auto link : m_dta->m_link_factory->m_link_map)
    results_ptr[idx++] = link.first;
  results.attr ("sort") ();
  return results;
}

py::array_t<double>
Dta::get_travel_stats ()
{
  // finished
  TInt _count_car = 0;
  TFlt _tot_tt_car = 0.0;

  auto *_veh_factory = dynamic_cast<MNM_Veh_Factory *> (m_dta->m_veh_factory);
  _count_car = _veh_factory->m_finished;
  _tot_tt_car = _veh_factory->m_total_time * m_dta->m_unit_time / 3600.0;

  // unfinished
  MNM_Veh *_veh;
  int _end_time = get_cur_loading_interval ();
  for (auto _map_it : m_dta->m_veh_factory->m_veh_map)
    {
      _veh = _map_it.second;
      IAssert (_veh->m_finish_time < 0);
      _count_car += 1;
      _tot_tt_car
        += (_end_time - _veh->m_start_time) * m_dta->m_unit_time / 3600.0;
    }

  // for all released vehicles
  int new_shape[1] = { 4 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  result_ptr[0] = _count_car / m_dta->m_flow_scalar; // released vehicles
  result_ptr[1]
    = _tot_tt_car / m_dta->m_flow_scalar; // VHT of released vehicles
  result_ptr[2]
    = _veh_factory->m_enroute / m_dta->m_flow_scalar; // enroute vehicles
  result_ptr[3]
    = _veh_factory->m_finished / m_dta->m_flow_scalar; // finished vehicles

  return result;
}

std::string
Dta::print_emission_stats ()
{
  return m_dta->m_emission->output ();
}

int
Dta::print_simulation_results (const std::string &folder, int cong_frequency)
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
  std::string _str1;
  TInt _current_inter = m_dta->m_current_loading_interval;
  std::ofstream _vis_file2;
  if (output_link_cong)
    {
      _vis_file2.open (folder + "/driving_link_cong_raw.txt",
                       std::ofstream::out);
      if (!_vis_file2.is_open ())
        {
          throw std::runtime_error ("failed to open _vis_file2");
        }

      _str1 = "timestamp(intervals) driving_link_ID vehicle_inflow vehicle_outflow "
              "vehicle_tt(s) vehicle_fftt(s) vehicle_freeflow_speed(mph) vehicle_speed(mph)\n";
      _vis_file2 << _str1;

      TInt _iter = 0;
      while (_iter + cong_frequency <= _current_inter)
        {
          if (_iter % cong_frequency == 0 || _iter == _current_inter - 1)
            {
              // printf ("Current loading interval: %d\n", int (_iter));
              for (auto _link_it : m_dta->m_link_factory->m_link_map)
                {
                  _link = _link_it.second;
                  _str1 = std::to_string (int (_iter)) + " ";
                  _str1 += std::to_string (_link->m_link_ID ()) + " ";
                  _str1
                    += std::to_string (
                         MNM_DTA_GRADIENT::get_link_inflow (_link, _iter,
                                                            _iter
                                                              + cong_frequency))
                       + " ";
                  _str1
                    += std::to_string (
                         MNM_DTA_GRADIENT::get_link_outflow (_link, _iter,
                                                            _iter
                                                              + cong_frequency))
                       + " ";
                  // _str1 +=
                  // std::to_string(MNM_DTA_GRADIENT::get_travel_time(_link,
                  // TFlt(_iter + 1), m_dta -> m_unit_time, m_dta ->
                  // m_current_loading_interval) * m_dta -> m_unit_time) + " ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_robust (
                               _link, TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_dta->m_unit_time,
                               m_dta->m_current_loading_interval)
                             * m_dta->m_unit_time)
                           + " "; // seconds
                  // _str1 += std::to_string (_link->get_link_freeflow_tt ()) + " ";  // seconds
                  _str1 += std::to_string (_link->get_link_freeflow_tt_loading () * m_dta->m_unit_time) + " ";  // seconds
                  // _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time(_link,
                  // TFlt(_iter + 1), m_dta -> m_unit_time, m_dta ->
                  // m_current_loading_interval) * m_dta -> m_unit_time) * 3600
                  // / 1600) + " ";
                  // _str1 += std::to_string (
                  //            _link->m_length
                  //            / _link->get_link_freeflow_tt ()
                  //            * 3600 / 1600)
                  //          + " "; // mph
                  _str1 += std::to_string (
                             _link->m_length
                             / (_link->get_link_freeflow_tt_loading () * m_dta -> m_unit_time)
                             * 3600 / 1600)
                           + " "; // mph
                  _str1 += std::to_string (
                             _link->m_length
                             / (MNM_DTA_GRADIENT::get_travel_time_robust (
                                  _link, TFlt (_iter + 1),
                                  TFlt (_iter + cong_frequency + 1),
                                  m_dta->m_unit_time,
                                  m_dta->m_current_loading_interval)
                                * m_dta->m_unit_time)
                             * 3600 / 1600)
                           + "\n"; // mph
                  _vis_file2 << _str1;
                }
            }
          _iter += 1;
        }

      // // save cc of some links
      // _str = "\n\n **************************** driving link cc
      // ****************************"; for (auto _link_it :
      // m_dta->m_link_factory->m_link_map) {
      //     _link = _link_it.second;
      //     if (_link->m_link_ID() == 4) {
      //         _str += "\nlink_ID: " + std::to_string(_link->m_link_ID());
      //         _str +="\nm_N_in: \n";
      //         _str += _link->m_N_in->to_string();
      //         _str +="\nm_N_out: \n";
      //         _str += _link->m_N_out->to_string();
      //         _vis_file2 << _str;
      //     }
      // }

      if (_vis_file2.is_open ())
        _vis_file2.close ();
    }
  std::cout << "Finish printing simulation results" << std::endl;
  return 0;
}

int
Dta::build_link_cost_map (bool with_congestion_indicator)
{
  MNM_Dlink *_link;
  TFlt _vot;
  if (m_dta->m_config->get_string ("routing_type") == "Due")
    {
      MNM_ConfReader *_tmp_conf
        = new MNM_ConfReader (m_dta->m_file_folder + "/config.conf", "DUE");
      _vot = _tmp_conf->get_float ("vot") / 3600.
             * m_dta->m_unit_time; // money / hour -> money / interval
      delete _tmp_conf;
    }
  else if (m_dta->m_config->get_string ("routing_type") == "Hybrid")
    {
      _vot = dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing)
               ->m_routing_adaptive->m_vot
             * m_dta->m_unit_time; // money / second -> money / interval
    }
  else
    {
      throw std::runtime_error ("unsupported routing type");
    }

  for (auto _link_it : m_dta->m_link_factory->m_link_map)
    {
      // #pragma omp task
      _link = _link_it.second;
      if (m_link_tt_map.find (_link_it.first) == m_link_tt_map.end ())
        {
          m_link_tt_map[_link_it.first] = new TFlt[get_cur_loading_interval ()];
        }
      if (m_link_cost_map.find (_link_it.first) == m_link_cost_map.end ())
        {
          m_link_cost_map[_link_it.first]
            = new TFlt[get_cur_loading_interval ()];
        }
      if (with_congestion_indicator)
        {
          // std::cout << "car, interval: " << i << ", link: " << _link_it.first
          // << ", tt: " << m_link_tt_map[_link_it.first][i] << ", fftt: " <<
          // _link -> get_link_freeflow_tt_car() / m_unit_time << "\n";
          if (m_link_congested.find (_link_it.first) == m_link_congested.end ())
            {
              m_link_congested[_link_it.first]
                = new bool[get_cur_loading_interval ()];
            }
        }

      std::cout << "********************** build_link_cost_map link "
                << _link->m_link_ID () << " **********************\n";
      for (int i = 0; i < get_cur_loading_interval (); i++)
        {
          m_link_tt_map[_link_it.first][i] = MNM_DTA_GRADIENT::
            get_travel_time (_link, TFlt (i + 1), m_dta->m_unit_time,
                             get_cur_loading_interval ()); // intervals
          m_link_cost_map[_link_it.first][i]
            = _vot * m_link_tt_map[_link_it.first][i] + _link->m_toll;
          // std::cout << "interval: " << i << ", link: " << _link_it.first <<
          // ", tt: " << m_link_tt_map[_link_it.first][i] << "\n"; std::cout <<
          // "car in" << "\n"; std::cout << _link -> m_N_in -> to_string() <<
          // "\n"; std::cout << "car out" << "\n"; std::cout << _link -> m_N_out
          // -> to_string() << "\n";
          if (with_congestion_indicator)
            {
              m_link_congested[_link_it.first][i]
                = m_link_tt_map[_link_it.first][i]
                  > _link->get_link_freeflow_tt_loading ();
              // std::cout << "car, interval: " << i << ", link: " <<
              // _link_it.first << ", congested?: " <<
              // m_link_congested[_link_it.first][i] << "\n";
            }
        }
    }

  return 0;
}

int
Dta::get_link_queue_dissipated_time ()
{
  // suppose m_link_congested is constructed already in build_link_cost_map()
  MNM_Dlink *_link;
  int _total_loading_inter = get_cur_loading_interval ();
  IAssert (_total_loading_inter > 0);

  bool _flg;
  std::cout << "\n********************** Begin get_link_queue_dissipated_time "
               "**********************\n";
  for (auto _link_it : m_dta->m_link_factory->m_link_map)
    {
      if (m_queue_dissipated_time.find (_link_it.first)
          == m_queue_dissipated_time.end ())
        {
          m_queue_dissipated_time[_link_it.first]
            = new int[_total_loading_inter];
        }
      // std::cout << "********************** get_link_queue_dissipated_time
      // link " << _link_it.first << " **********************\n";
      for (int i = 0; i < _total_loading_inter; i++)
        {
          // ************************** car **************************
          if (m_link_congested[_link_it.first][i])
            {
              if (i == _total_loading_inter - 1)
                {
                  m_queue_dissipated_time[_link_it.first][i]
                    = _total_loading_inter;
                }
              else
                {
                  _flg = false;
                  for (int k = i + 1; k < _total_loading_inter; k++)
                    {
                      if (m_link_congested[_link_it.first][k - 1]
                          && !m_link_congested[_link_it.first][k])
                        {
                          m_queue_dissipated_time[_link_it.first][i] = k;
                          _flg = true;
                          break;
                        }
                    }
                  if (!_flg)
                    {
                      m_queue_dissipated_time[_link_it.first][i]
                        = _total_loading_inter;
                    }
                }
            }
          else
            {
              _link = dynamic_cast<MNM_Dlink *> (_link_it.second);
              if (MNM_Ults::
                    approximate_equal (m_link_tt_map[_link_it.first][i],
                                       (float) _link
                                         ->get_link_freeflow_tt_loading ()))
                {
                  // based on subgradient paper, when out flow = capacity and
                  // link tt = fftt, this is critical state where the
                  // subgradient applies
                  if (dynamic_cast<MNM_Dlink_Ctm *> (_link) != nullptr)
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
                        = MNM_DTA_GRADIENT::get_departure_cc_slope (
                          _link,
                          TFlt (i
                                + (int) _link->get_link_freeflow_tt_loading ()),
                          TFlt (i + (int) _link->get_link_freeflow_tt_loading ()
                                + 1)); // veh / 5s
                      TFlt _cap = dynamic_cast<MNM_Dlink_Ctm *> (_link)
                                    ->m_cell_array.back ()
                                    ->m_flow_cap
                                  * m_dta->m_unit_time; // veh / 5s
                      if (MNM_Ults::approximate_equal (_outflow_rate
                                                         * m_dta->m_flow_scalar,
                                                       floor (
                                                         _cap
                                                         * m_dta
                                                             ->m_flow_scalar)))
                        {
                          if (i == _total_loading_inter - 1)
                            {
                              m_queue_dissipated_time[_link_it.first][i]
                                = _total_loading_inter;
                            }
                          else
                            {
                              // to compute lift up time for the departure cc
                              _flg = false;
                              for (int k = i + 1; k < _total_loading_inter; k++)
                                {
                                  if (m_link_congested[_link_it.first][k - 1]
                                      && !m_link_congested[_link_it.first][k])
                                    {
                                      m_queue_dissipated_time[_link_it.first][i]
                                        = k;
                                      _flg = true;
                                      break;
                                    }
                                }
                              if (!_flg)
                                {
                                  m_queue_dissipated_time[_link_it.first][i]
                                    = _total_loading_inter;
                                }
                            }
                        }
                      else
                        {
                          // TODO: boundary condition
                          m_queue_dissipated_time[_link_it.first][i] = i;
                        }
                    }
                  else if (dynamic_cast<MNM_Dlink_Pq *> (_link) != nullptr)
                    {
                      // PQ link as OD connectors always has sufficient capacity
                      m_queue_dissipated_time[_link_it.first][i] = i;
                    }
                  else
                    {
                      throw std::runtime_error (
                        "Dta::get_link_queue_dissipated_time, Link type "
                        "not implemented");
                    }
                }
              else
                {
                  // m_queue_dissipated_time[_link_it.first][i] = i;
                  throw std::runtime_error (
                    "Dta::get_link_queue_dissipated_time, Link travel time "
                    "less than fftt");
                }
              // m_queue_dissipated_time[_link_it.first][i] = i;
            }
        }
    }
  std::cout << "********************** End get_link_queue_dissipated_time "
               "**********************\n";
  return 0;
}

int
Dta::register_links (py::array_t<int> links)
{
  if (m_link_vec.size () > 0)
    {
      printf ("Warning, Dta::register_links, link exists\n");
      m_link_vec.clear ();
    }
  // https://people.duke.edu/~ccc14/cspy/18G_C++_Python_pybind11.html#Using-numpy-arrays-as-function-arguments-and-return-values
  // https://www.linyuanshi.me/post/pybind11-array/
  // The properties of the numpy array can be obtained by calling its request()
  // method
  auto links_buf = links.request ();
  if (links_buf.ndim != 1)
    { // dimensions
      throw std::runtime_error ("Number of dimensions must be one");
    }
  // obtain the pointer with the type cast to access and modify the elements of
  // the array
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_dta->m_link_factory->get_link (TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (std::find (m_link_vec.begin (), m_link_vec.end (), _link)
          != m_link_vec.end ())
        {
          throw std::runtime_error (
            "Error, Dta::register_links, link does not exist");
        }
      else
        {
          m_link_vec.push_back (_link);
          // m_link_map.insert(std::make_pair(_link, i));
        }
    }
  return 0;
}

py::array_t<int>
Dta::get_registered_links ()
{
  auto results = py::array_t<int> (m_link_vec.size ());
  auto results_buf = results.request ();
  int *results_ptr = static_cast<int *> (results_buf.ptr);
  for (int idx = 0; idx < m_link_vec.size (); idx++)
    results_ptr[idx] = (int) m_link_vec[idx]->m_link_ID;
  return results;
}

int
Dta::register_paths (py::array_t<int> paths)
{
  if (m_path_vec.size () > 0)
    {
      printf ("Warning, Dta::register_paths, path exists\n");
      m_path_vec.clear ();
      m_path_map.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Dta::register_paths: Number of dimensions must be one");
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
          throw std::runtime_error ("Dta::register_paths: No such path");
        }
      else
        {
          m_path_vec.push_back (m_ID_path_mapping[_path_ID]);
          m_path_map.insert (std::make_pair (m_ID_path_mapping[_path_ID], i));
        }
    }
  // m_path_set = std::set<MNM_Path*> (m_path_vec.begin(), m_path_vec.end());
  return 0;
}

std::vector<bool>
Dta::check_registered_links_in_registered_paths ()
{
  std::vector<bool> _link_existing = std::vector<bool> ();
  if (m_link_vec.empty ())
    {
      printf ("Warning, Dta::check_registered_links_in_registered_paths, "
              "no link registered\n");
      return _link_existing;
    }
  for (size_t k = 0; k < m_link_vec.size (); ++k)
    {
      _link_existing.push_back (false);
    }
  if (m_path_vec.empty ())
    {
      printf ("Warning, Dta::check_registered_links_in_registered_paths, "
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
      printf ("Warning: some observed bus links in m_link_vec are not covered "
              "by generated paths in m_path_vec!\n");
    }
  return _link_existing;
}

py::array_t<bool>
Dta::are_registered_links_in_registered_paths ()
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
Dta::generate_paths_to_cover_registered_links ()
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

  PNEGraph reversed_graph = MNM_Ults::reverse_graph (m_dta->m_graph);
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  for (auto _link_it : m_dta->m_link_factory->m_link_map)
    {
      _cost_map.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               _link_it.second->get_link_freeflow_tt ()));
    }
  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TInt> _shortest_path_tree_reversed
    = std::unordered_map<TInt, TInt> ();
  TInt _from_node_ID, _to_node_ID;
  MNM_Origin *_origin;
  MNM_Destination *_dest;
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
            = m_dta->m_graph->GetEI (m_link_vec[i]->m_link_ID).GetSrcNId ();
          _to_node_ID
            = m_dta->m_graph->GetEI (m_link_vec[i]->m_link_ID).GetDstNId ();

          // path from origin to from_node_ID
          if (!_shortest_path_tree.empty ())
            {
              _shortest_path_tree.clear ();
            }
          if (dynamic_cast<MNM_DMOND *> (
                m_dta->m_node_factory->get_node (_from_node_ID))
              != nullptr)
            {
              _path_1 = new MNM_Path ();
              _path_1->m_node_vec.push_back (_from_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_from_node_ID, m_dta->m_graph,
                                                  _cost_map,
                                                  _shortest_path_tree);
            }

          // path from to_node_ID to destination
          if (!_shortest_path_tree_reversed.empty ())
            {
              _shortest_path_tree_reversed.clear ();
            }
          if (dynamic_cast<MNM_DMDND *> (
                m_dta->m_node_factory->get_node (_to_node_ID))
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
          for (const auto &p : m_dta->m_od_factory->m_origin_map)
            {
              pair_ptrs_1.emplace_back (p);
            }
          std::random_shuffle (std::begin (pair_ptrs_1),
                               std::end (pair_ptrs_1));
          for (auto _it : pair_ptrs_1)
            {
              _origin = _it.second;
              if (_origin->m_demand.empty ())
                {
                  continue;
                }

              if (!pair_ptrs_2.empty ())
                {
                  pair_ptrs_2.clear ();
                }
              for (const auto &p : _origin->m_demand)
                {
                  pair_ptrs_2.emplace_back (p);
                }
              std::random_shuffle (std::begin (pair_ptrs_2),
                                   std::end (pair_ptrs_2));
              for (auto _it_it : pair_ptrs_2)
                {
                  _dest = _it_it.first;
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
          Assert (_origin != nullptr && _dest != nullptr);

          if (!_shortest_path_tree.empty ())
            {
              _path_1 = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                           _from_node_ID, _shortest_path_tree,
                                           m_dta->m_graph);
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
          _path->allocate_buffer (m_dta->m_config->get_int ("max_interval"));
          delete _path_1;
          delete _path_2;

          // add this new path to path table
          dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing)
            ->m_routing_fixed->m_path_table
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
      throw std::runtime_error (
        "failed when generate paths to cover registered links");
    }

  MNM::save_path_table (m_dta->m_file_folder,
                        dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing)
                          ->m_routing_fixed->m_path_table,
                        m_dta->m_od_factory, true, false);

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
Dta::save_path_table (const std::string &folder)
{
  // write updated path table to file
  Path_Table *_path_table;
  if (dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing) != nullptr)
    {
      _path_table = dynamic_cast<MNM_Routing_Hybrid *> (m_dta->m_routing)
                      ->m_routing_fixed->m_path_table;
    }
  else if (dynamic_cast<MNM_Routing_Fixed *> (m_dta->m_routing) != nullptr)
    {
      _path_table
        = dynamic_cast<MNM_Routing_Fixed *> (m_dta->m_routing)->m_path_table;
    }
  else
    {
      throw std::runtime_error ("invalid routing type");
    }
  MNM::save_path_table (folder, _path_table, m_dta->m_od_factory, true, false);
  return 0;
}

py::array_t<double>
Dta::get_link_inflow (py::array_t<int> start_intervals,
                      py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_link_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Dta::get_link_inflow, input length mismatch");
    }
  // number of time steps from input
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };
  // creat a new py::array_t<double> as output, here ndim == 2
  auto result = py::array_t<double> (new_shape);
  // request() method of py::array_t()
  auto result_buf = result.request ();
  // obtain the pointer to manipulate the created array
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (end_ptr[t] <= start_ptr[t])
        {
          throw std::runtime_error ("Error, Dta::get_link_inflow, end time "
                                    "is smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_link_inflow, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_link_inflow, input end intervals exceeds the "
            "total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          // matrix is stored as a row-major array
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow (m_link_vec[i],
                                                 TFlt (start_ptr[t]),
                                                 TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  // return the created array
  return result;
}

py::array_t<double>
Dta::get_link_tt_FD (py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_link_tt_FD, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      // if (start_ptr[t] >= get_cur_loading_interval()){
      //     throw std::runtime_error("Error, Dta::get_link_tt_FD, input
      //     start intervals exceeds the total loading intervals - 1");
      // }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          // // use start_ptr[t] + 1 as start_time in cc to compute link travel
          // time for vehicles arriving at the beginning of interval
          // start_ptr[t]
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_travel_time_from_FD (m_link_vec[i],
                                                         TFlt (start_ptr[t]),
                                                         m_dta->m_unit_time) ()
              * m_dta->m_unit_time; // second
        }
    }
  return result;
}

py::array_t<double>
Dta::get_link_tt (py::array_t<int> start_intervals, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_link_tt, input start intervals exceeds the "
            "total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          // use start_ptr[t] + 1 as start_time in cc to compute link travel
          // time for vehicles arriving at the beginning of interval
          // start_ptr[t]
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::
                get_travel_time (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                 m_dta->m_unit_time,
                                 m_dta->m_current_loading_interval) ()
              * m_dta->m_unit_time; // second
          if (result_ptr[i * l + t]
              > TT_UPPER_BOUND * m_link_vec[i]->m_length / m_link_vec[i]->m_ffs)
            {
              if (return_inf)
                {
                  result_ptr[i * l + t]
                    = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  result_ptr[i * l + t] = TT_UPPER_BOUND
                                          * m_link_vec[i]->m_length
                                          / m_link_vec[i]->m_ffs;
                }
            }
        }
    }
  return result;
}

py::array_t<double>
Dta::get_link_tt_robust (py::array_t<double> start_intervals,
                         py::array_t<double> end_intervals, int num_trials,
                         bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Dta::get_link_tt_robust, input length mismatch");
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
            "Error, Dta::get_link_tt_robust, end time is smaller than or "
            "equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_link_tt_robust, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_link_tt_robust, input end intervals exceeds "
            "the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_robust (m_link_vec[i], TFlt (start_ptr[t] + 1),
                                    TFlt (end_ptr[t] + 1), m_dta->m_unit_time,
                                    m_dta->m_current_loading_interval,
                                    num_trials) ();
          result_ptr[i * l + t] = _tmp * m_dta->m_unit_time; // second
          if (result_ptr[i * l + t]
              > TT_UPPER_BOUND * m_link_vec[i]->m_length / m_link_vec[i]->m_ffs)
            {
              if (return_inf)
                {
                  result_ptr[i * l + t]
                    = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  result_ptr[i * l + t] = TT_UPPER_BOUND
                                          * m_link_vec[i]->m_length
                                          / m_link_vec[i]->m_ffs;
                }
            }
        }
    }
  return result;
}

py::array_t<double>
Dta::get_registered_path_tt (py::array_t<int> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_registered_path_tt, input dimension mismatch");
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
            "Error, Dta::get_registered_path_tt, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec.size (); ++i)
        {
          // MNM_DTA_GRADIENT::get_path_travel_time will process link travel
          // time and the start_time for cc result_ptr[i * l + t] =
          // MNM_DTA_GRADIENT::get_path_travel_time(
          //         m_path_vec[i], TFlt(start_ptr[t]), m_dta -> m_link_factory,
          //         m_dta -> m_unit_time)() * m_dta -> m_unit_time;  // seconds
          // assume build_link_cost_map() is invoked before
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::
                get_path_travel_time (m_path_vec[i], TFlt (start_ptr[t]),
                                      m_link_tt_map,
                                      get_cur_loading_interval ()) ()
              * m_dta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Dta::get_path_tt (py::array_t<int> link_IDs, py::array_t<int> start_intervals)
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
      _link = m_dta->m_link_factory->get_link (TInt (links_ptr[i]));
      _path->m_link_vec.push_back (links_ptr[i]);
      _path->m_node_vec.push_back (_link->m_from_node->m_node_ID);
      if (i == links_buf.shape[0] - 1)
        {
          _path->m_node_vec.push_back (_link->m_to_node->m_node_ID);
        }
    }

  for (int t = 0; t < num_int; ++t)
    {
      // std::cout << "t: " << t << ", time: " << start_ptr[t] << ", duration: "
      // << get_cur_loading_interval() << std::endl;
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Dta::get_path_tt, input start intervals exceeds the "
            "total loading intervals - 1");
        }
      // MNM_DTA_GRADIENT::get_path_travel_time will process link travel time
      // and the start_time for cc result_ptr[t] =
      // MNM_DTA_GRADIENT::get_path_travel_time(_path, TFlt(start_ptr[t]), m_dta
      // -> m_link_factory, m_dta -> m_unit_time, get_cur_loading_interval()) *
      // m_dta -> m_unit_time;  // seconds assume build_link_cost_map() is
      // invoked before
      result_ptr[t]
        = MNM_DTA_GRADIENT::get_path_travel_time (_path, TFlt (start_ptr[t]),
                                                  m_link_tt_map,
                                                  get_cur_loading_interval ())
          * m_dta->m_unit_time; // seconds
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
Dta::get_link_in_cc (int link_ID)
{
  if (m_dta->m_link_factory->get_link (TInt (link_ID))->m_N_in == nullptr)
    {
      throw std::runtime_error ("Error, Dta::get_link_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = m_dta->m_link_factory->get_link (TInt (link_ID))->m_N_in->m_recorder;
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
Dta::get_link_out_cc (int link_ID)
{
  if (m_dta->m_link_factory->get_link (TInt (link_ID))->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, Dta::get_link_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = m_dta->m_link_factory->get_link (TInt (link_ID))->m_N_out->m_recorder;
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
Dta::get_dar_matrix (py::array_t<int> start_intervals,
                     py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Dta::get_dar_matrix, input length mismatch");
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
      std::cout << "************ DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Dta::get_dar_matrix, end time is smaller than or "
                "equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::get_dar_matrix, input start intervals exceeds "
                "the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::get_dar_matrix, input end intervals exceeds "
                "the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records (_record, m_link_vec[i], m_path_map,
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
      // the count of 15 min interval
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
Dta::save_dar_matrix (py::array_t<int> start_intervals,
                      py::array_t<int> end_intervals, py::array_t<double> f,
                      const std::string &file_name)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 540, ...] with increment of ass_freq
  auto f_buf = f.request ();
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::save_dar_matrix, input path flow mismatch");
    }
  double *f_ptr = (double *) f_buf.ptr;

  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::save_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Dta::save_dar_matrix, input length mismatch");
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

  int _num_path = m_path_map.size ();
  int _num_link = m_link_vec.size ();
  int _x, _y;

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      std::cout << "************ DAR link " << m_link_vec[i]->m_link_ID ()
                << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Dta::save_dar_matrix, end time is smaller than or "
                "equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::save_dar_matrix, input start intervals "
                "exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::save_dar_matrix, input end intervals exceeds "
                "the total loading intervals");
            }
          Assert (_record.empty ());
          MNM_DTA_GRADIENT::add_dar_records (_record, m_link_vec[i], m_path_map,
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
Dta::get_complete_dar_matrix (py::array_t<int> start_intervals,
                              py::array_t<int> end_intervals, int num_intervals,
                              py::array_t<double> f)
{
  // start_intervals and end_intervals are like [0, 180, 360, ...] and [180,
  // 360, 720, ...] with increment of ass_freq
  int _num_e_path = m_path_map.size ();
  int _num_e_link = m_link_vec.size ();
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  auto f_buf = f.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_complete_dar_matrix, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Dta::get_complete_dar_matrix, input length mismatch");
    }
  if (f_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Dta::get_complete_dar_matrix, input path flow mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  double *f_ptr = (double *) f_buf.ptr;

  std::vector<Eigen::Triplet<double>> _record;
  // pre-allocate sufficient space for dar
  _record.reserve (int (1e9));

  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Dta::get_complete_dar_matrix, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::get_complete_dar_matrix, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Dta::get_complete_dar_matrix, input end intervals "
                "exceeds the total loading intervals");
            }
          MNM_DTA_GRADIENT::add_dar_records_eigen (_record, m_link_vec[i],
                                                   m_path_map,
                                                   TFlt (start_ptr[t]),
                                                   TFlt (end_ptr[t]), i, t,
                                                   _num_e_link, _num_e_path,
                                                   f_ptr);
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
Dta::delete_all_agents ()
{
  // invoke it after simulation, may help save some memory
  delete m_dta->m_veh_factory;
  m_dta->m_veh_factory = nullptr;
  return 0;
}

}
}
