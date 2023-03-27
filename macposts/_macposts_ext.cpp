#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <set>
#include <unordered_map>
#include <vector>

#include <Snap.h>
#include <dso.h>
#include <dta.h>
#include <due.h>
#include <multiclass.h>
#include <multimodal.h>

namespace py = pybind11;
using SparseMatrixR = Eigen::SparseMatrix<double, Eigen::RowMajor>;

const TFlt TT_UPPER_BOUND = 20;

//
// Time-dependent shortest path (TDSP)
//

class Tdsp
{
public:
  Tdsp ();
  ~Tdsp ();
  int initialize (const std::string &folder, int max_interval,
                  int num_rows_link_file, int num_rows_node_file);
  int read_td_cost_txt (const std::string &folder,
                        const std::string &link_tt_file_name = "td_link_tt",
                        const std::string &node_tt_file_name = "td_node_tt",
                        const std::string &link_cost_file_name = "td_link_cost",
                        const std::string &node_cost_file_name
                        = "td_node_cost");
  int read_td_cost_py (py::array_t<double> td_link_tt_py,
                       py::array_t<double> td_link_cost_py,
                       py::array_t<double> td_node_tt_py,
                       py::array_t<double> td_node_cost_py);
  int read_td_link_cost (py::array_t<double> td_link_cost_py,
                         std::unordered_map<TInt, TFlt *> &td_link_cost);
  int read_td_node_cost (
    py::array_t<double> td_node_cost_py,
    std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &td_node_cost);

  int build_tdsp_tree (int dest_node_ID);

  py::array_t<double> extract_tdsp (int origin_node_ID, int timestamp);

  TInt m_num_rows_link_file, m_num_rows_node_file, m_dest_node_ID,
    m_max_interval;
  PNEGraph m_graph;
  MNM_TDSP_Tree *m_tdsp_tree;

  std::unordered_map<TInt, TFlt *> m_td_link_tt;
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> m_td_node_tt;
  std::unordered_map<TInt, TFlt *> m_td_link_cost;
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> m_td_node_cost;
};

Tdsp::Tdsp ()
{
  m_dest_node_ID = -1;
  m_max_interval = -1;
  m_tdsp_tree = nullptr;

  m_td_link_tt = std::unordered_map<TInt, TFlt *> ();
  m_td_node_tt = std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> ();
  m_td_link_cost = std::unordered_map<TInt, TFlt *> ();
  m_td_node_cost
    = std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> ();

  m_num_rows_link_file = -1;
  m_num_rows_node_file = -1;

  m_graph = nullptr;
}

Tdsp::~Tdsp ()
{
  delete m_tdsp_tree;

  for (auto _it : m_td_link_tt)
    {
      free (_it.second);
    }
  m_td_link_tt.clear ();
  for (auto _it : m_td_node_tt)
    {
      for (auto _it_it : _it.second)
        {
          free (_it_it.second);
        }
      _it.second.clear ();
    }
  m_td_node_tt.clear ();

  for (auto _it : m_td_link_cost)
    {
      free (_it.second);
    }
  m_td_link_cost.clear ();
  for (auto _it : m_td_node_cost)
    {
      for (auto _it_it : _it.second)
        {
          free (_it_it.second);
        }
      _it.second.clear ();
    }
  m_td_node_cost.clear ();

  m_graph->Clr ();
}

int
Tdsp::initialize (const std::string &folder, int max_interval,
                  int num_rows_link_file, int num_rows_node_file)
{
  m_max_interval = max_interval;

  m_num_rows_link_file = num_rows_link_file;
  m_num_rows_node_file = num_rows_node_file;

  MNM_ConfReader *conf_reader
    = new MNM_ConfReader (folder + "/config.conf", "Network");
  m_graph = MNM_IO::build_graph (folder, conf_reader);

  delete conf_reader;
  return 0;
}

int
Tdsp::read_td_cost_txt (const std::string &folder,
                        const std::string &link_tt_file_name,
                        const std::string &node_tt_file_name,
                        const std::string &link_cost_file_name,
                        const std::string &node_cost_file_name)
{
  // external input with plain text
  MNM_IO::read_td_link_cost (folder, m_td_link_tt, m_num_rows_link_file,
                             m_max_interval, link_tt_file_name);
  printf ("Complete reading link tt, size: %d\n", (int) m_td_link_tt.size ());
  MNM_IO::read_td_link_cost (folder, m_td_link_cost, m_num_rows_link_file,
                             m_max_interval, link_cost_file_name);
  printf ("Complete reading link cost, size: %d\n",
          (int) m_td_link_cost.size ());
  if (m_num_rows_node_file != -1)
    {
      MNM_IO::read_td_node_cost (folder, m_td_node_tt, m_num_rows_node_file,
                                 m_max_interval, node_tt_file_name);
      printf ("Complete reading node tt, size: %d\n",
              (int) m_td_node_tt.size ());
      MNM_IO::read_td_node_cost (folder, m_td_node_cost, m_num_rows_node_file,
                                 m_max_interval, node_cost_file_name);
      printf ("Complete reading node cost, size: %d\n",
              (int) m_td_node_cost.size ());
    }
  return 0;
}

int
Tdsp::read_td_cost_py (py::array_t<double> td_link_tt_py,
                       py::array_t<double> td_link_cost_py,
                       py::array_t<double> td_node_tt_py,
                       py::array_t<double> td_node_cost_py)
{
  // external input with numpy array
  read_td_link_cost (td_link_tt_py, m_td_link_tt);
  printf ("Complete reading link tt, size: %d\n", (int) m_td_link_tt.size ());
  read_td_link_cost (td_link_cost_py, m_td_link_cost);
  printf ("Complete reading link cost, size: %d\n",
          (int) m_td_link_cost.size ());
  if (m_num_rows_node_file != -1)
    {
      read_td_node_cost (td_node_tt_py, m_td_node_tt);
      printf ("Complete reading node tt, size: %d\n",
              (int) m_td_node_tt.size ());
      read_td_node_cost (td_node_cost_py, m_td_node_cost);
      printf ("Complete reading node cost, size: %d\n",
              (int) m_td_node_cost.size ());
    }
  return 0;
}

int
Tdsp::read_td_link_cost (py::array_t<double> td_link_cost_py,
                         std::unordered_map<TInt, TFlt *> &td_link_cost)
{
  auto start_buf = td_link_cost_py.request ();
  if (start_buf.ndim != 2)
    {
      throw std::runtime_error (
        "Error, Tdsp::read_td_link_cost, input dimension must be 2");
    }
  if (start_buf.shape[0] != m_num_rows_link_file
      || start_buf.shape[1] != m_max_interval + 1)
    {
      throw std::runtime_error (
        "Error, Tdsp::read_td_link_cost, input length mismatch");
    }
  double *start_ptr = (double *) start_buf.ptr;

  TInt _link_ID;
  TFlt _cost;
  TFlt *_cost_vector;
  for (int i = 0; i < m_num_rows_link_file; ++i)
    {
      _link_ID = TInt ((int) start_ptr[i * (m_max_interval + 1)]);
      if (td_link_cost.find (_link_ID) == td_link_cost.end ())
        {
          TFlt *_cost_vector_tmp
            = (TFlt *) malloc (sizeof (TFlt) * m_max_interval);
          td_link_cost.insert (
            std::pair<TInt, TFlt *> (_link_ID, _cost_vector_tmp));
        }
      _cost_vector = td_link_cost.find (_link_ID)->second;
      for (int j = 0; j < m_max_interval; ++j)
        {
          _cost = TFlt (start_ptr[i * (m_max_interval + 1) + j + 1]);
          // printf("Tdsp::read_td_link_cost, %d, %f\n", j, _cost());
          _cost_vector[j] = _cost;
        }
    }

  return 0;
}

int
Tdsp::read_td_node_cost (
  py::array_t<double> td_node_cost_py,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &td_node_cost)
{
  auto start_buf = td_node_cost_py.request ();
  if (start_buf.ndim != 2)
    {
      throw std::runtime_error (
        "Error, Tdsp::read_td_node_cost, input dimension must be 2");
    }
  if (start_buf.shape[0] != m_num_rows_node_file
      || start_buf.shape[1] != m_max_interval + 3)
    {
      throw std::runtime_error (
        "Error, Tdsp::read_td_node_cost, input length mismatch");
    }
  double *start_ptr = (double *) start_buf.ptr;

  TInt _in_link_ID, _out_link_ID;
  TFlt _cost;
  TFlt *_cost_vector;
  for (int i = 0; i < m_num_rows_node_file; ++i)
    {
      _in_link_ID = TInt ((int) start_ptr[i * (m_max_interval + 3) + 1]);
      _out_link_ID = TInt ((int) start_ptr[i * (m_max_interval + 3) + 2]);
      if (td_node_cost.find (_in_link_ID) == td_node_cost.end ())
        {
          td_node_cost.insert (
            std::pair<TInt,
                      std::unordered_map<TInt, TFlt *>> (_in_link_ID,
                                                         std::unordered_map<
                                                           TInt, TFlt *> ()));
        }
      if (td_node_cost.find (_in_link_ID)->second.find (_out_link_ID)
          == td_node_cost.find (_in_link_ID)->second.end ())
        {
          TFlt *_cost_vector_tmp
            = (TFlt *) malloc (sizeof (TFlt) * m_max_interval);
          td_node_cost.find (_in_link_ID)
            ->second.insert (
              std::pair<TInt, TFlt *> (_out_link_ID, _cost_vector_tmp));
        }
      _cost_vector
        = td_node_cost.find (_in_link_ID)->second.find (_out_link_ID)->second;
      for (int j = 0; j < m_max_interval; ++j)
        {
          _cost = TFlt (start_ptr[i * (m_max_interval + 3) + j + 3]);
          _cost_vector[j] = _cost;
        }
    }

  return 0;
}

int
Tdsp::build_tdsp_tree (int dest_node_ID)
{
  m_dest_node_ID = dest_node_ID;
  if (m_tdsp_tree != nullptr)
    {
      delete m_tdsp_tree;
      m_tdsp_tree = nullptr;
    }
  m_tdsp_tree = new MNM_TDSP_Tree (dest_node_ID, m_graph, m_max_interval);
  printf ("Init TDSP tree\n");
  m_tdsp_tree->initialize ();
  printf ("Update tree\n");
  if (m_num_rows_node_file != -1)
    {
      m_tdsp_tree->update_tree (m_td_link_cost, m_td_node_cost, m_td_link_tt,
                                m_td_node_tt);
    }
  else
    {
      m_tdsp_tree->update_tree (m_td_link_cost, m_td_link_tt);
    }
  return 0;
}

py::array_t<double>
Tdsp::extract_tdsp (int origin_node_ID, int timestamp)
{
  IAssert (timestamp < m_max_interval && timestamp >= 0);
  TFlt tmp_cost;
  MNM_Path *_path;
  std::string _str;

  IAssert (m_tdsp_tree->m_dist.find (origin_node_ID)
           != m_tdsp_tree->m_dist.end ());
  IAssert (m_tdsp_tree->m_dist[origin_node_ID] != nullptr);

  printf ("get travel cost to dest\n");
  tmp_cost = m_tdsp_tree
               ->m_dist[origin_node_ID][timestamp < m_tdsp_tree->m_max_interval
                                          ? timestamp
                                          : m_tdsp_tree->m_max_interval - 1];
  printf ("At time %d, minimum cost is %f\n", timestamp, tmp_cost ());
  _path = new MNM_Path ();
  TFlt _tt;
  if (m_num_rows_node_file != -1)
    {
      _tt = m_tdsp_tree->get_tdsp (origin_node_ID, timestamp, m_td_link_tt,
                                   m_td_node_tt, _path);
    }
  else
    {
      _tt = m_tdsp_tree->get_tdsp (origin_node_ID, timestamp, m_td_link_tt,
                                   _path);
    }
  printf ("travel time: %f\n", _tt ());
  printf ("number of nodes: %d\n", int (_path->m_node_vec.size ()));
  _str = _path->node_vec_to_string ();
  std::cout << "path: " << _str << "\n";

  int new_shape[2] = { (int) _path->m_node_vec.size (), 4 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  for (int i = 0; i < new_shape[0]; ++i)
    {
      result_ptr[i * new_shape[1]] = _path->m_node_vec[i];
      if (i < new_shape[0] - 1)
        {
          result_ptr[i * new_shape[1] + 1] = _path->m_link_vec[i];
        }
      else
        {
          result_ptr[i * new_shape[1] + 1] = -1;
        }
      if (i == 0)
        {
          result_ptr[i * new_shape[1] + 2] = tmp_cost;
          result_ptr[i * new_shape[1] + 3] = _tt;
        }
      else
        {
          result_ptr[i * new_shape[1] + 2] = -1;
          result_ptr[i * new_shape[1] + 3] = -1;
        }
    }
  delete _path;
  return result;
}

//
// Single class DTA
//

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
  IAssert (m_dta->m_config->get_string ("routing_type") == "Due"
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
  std::runtime_error ("Dta:: Routing type not implemented in API");
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
  for (size_t i = 0; i < m_link_vec.size (); ++i)
    {
      m_link_vec[i]->install_cumulative_curve ();
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
  IAssert (m_dta == nullptr);
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
  IAssert (m_dta == nullptr);
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

      _str1 = "timestamp (intervals), driving_link_ID, vehicle_inflow, "
              "vehicle_tt (s), vehicle_fftt (s), vehicle_speed (mph)\n";
      _vis_file2 << _str1;

      TInt _iter = 0;
      while (_iter + cong_frequency <= _current_inter)
        {
          if (_iter % cong_frequency == 0 || _iter == _current_inter - 1)
            {
              printf ("Current loading interval: %d\n", int (_iter));
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
                  _str1
                    += std::to_string (_link->get_link_freeflow_tt ()) + " ";
                  // _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time(_link,
                  // TFlt(_iter + 1), m_dta -> m_unit_time, m_dta ->
                  // m_current_loading_interval) * m_dta -> m_unit_time) * 3600
                  // / 1600) + " ";
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
          IAssert (_origin != nullptr && _dest != nullptr);

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
  // 360, 720, ...] with increment of ass_freq
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
          IAssert (_record.empty ());
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

//
// Bi-class DTA
//

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
  printf ("xxx\n");
  std::runtime_error ("Mcdta:: Routing type not implemented in API");
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

//
// Multimodal DTA
//

class Mmdta
{
public:
  Mmdta ();
  ~Mmdta ();
  int initialize (const std::string &folder);
  int initialize_mmdue (const std::string &folder);
  int generate_shortest_pathsets (const std::string &folder, int max_iter,
                                  double mid_scale, double heavy_scale,
                                  double min_path_tt = 0.);
  bool check_input_files ();

  int install_cc ();
  int install_cc_tree ();

  int run_whole (bool verbose = false);
  int run_mmdue (const std::string &folder, bool verbose = true);
  int run_mmdta_adaptive (const std::string &folder, int cong_frequency = 180,
                          bool verbose = true);

  int register_links_driving (py::array_t<int> links_driving);
  int register_links_walking (py::array_t<int> links_walking);
  int register_links_bus (py::array_t<int> links_bus);
  py::array_t<double> get_links_overlapped_bus_driving ();
  int register_links_bus_driving (py::array_t<int> links_bus_driving);

  int get_cur_loading_interval ();
  py::array_t<double> get_travel_stats ();
  std::string print_emission_stats ();
  int print_simulation_results (const std::string &folder,
                                int cong_frequency = 180);

  py::array_t<int> get_od_mode_connectivity ();
  int generate_init_mode_demand_file (const std::string &file_folder);

  py::array_t<double> get_car_link_fftt (py::array_t<int> link_IDs);
  py::array_t<double> get_truck_link_fftt (py::array_t<int> link_IDs);
  py::array_t<double> get_bus_link_fftt (py::array_t<int> link_IDs);
  py::array_t<double> get_walking_link_fftt (py::array_t<int> link_IDs);

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
  py::array_t<double> get_bus_link_tt (py::array_t<double> start_intervals,
                                       bool return_inf = false,
                                       bool return_bus_time = false);
  py::array_t<double>
  get_bus_link_tt_robust (py::array_t<double> start_intervals,
                          py::array_t<double> end_intervals,
                          int num_trials = 180, bool return_inf = false,
                          bool return_bus_time = false);
  // for boarding links, this includes bus waiting time
  py::array_t<double>
  get_passenger_walking_link_tt (py::array_t<double> start_intervals);
  py::array_t<double>
  get_passenger_walking_link_tt_robust (py::array_t<double> start_intervals,
                                        py::array_t<double> end_intervals,
                                        int num_trials = 180);
  py::array_t<double>
  get_bus_driving_link_tt_car (py::array_t<double> start_intervals,
                               bool return_inf = false);
  py::array_t<double> get_bus_driving_link_tt_car_robust (
    py::array_t<double> start_intervals, py::array_t<double> end_intervals,
    int num_trials = 180, bool return_inf = false);
  py::array_t<double>
  get_bus_driving_link_tt_truck (py::array_t<double> start_intervals,
                                 bool return_inf = false);
  py::array_t<double> get_bus_driving_link_tt_truck_robust (
    py::array_t<double> start_intervals, py::array_t<double> end_intervals,
    int num_trials = 180, bool return_inf = false);

  py::array_t<double> get_car_link_speed (py::array_t<double> start_intervals);
  py::array_t<double>
  get_truck_link_speed (py::array_t<double> start_intervals);
  py::array_t<double> get_bus_link_speed (py::array_t<double> start_intervals,
                                          bool return_inf = false,
                                          bool return_bus_time = false);

  py::array_t<double> get_link_car_inflow (py::array_t<int> start_intervals,
                                           py::array_t<int> end_intervals);
  py::array_t<double> get_link_truck_inflow (py::array_t<int> start_intervals,
                                             py::array_t<int> end_intervals);
  py::array_t<double> get_link_bus_inflow (py::array_t<int> start_intervals,
                                           py::array_t<int> end_intervals);
  py::array_t<double> get_busstop_bus_inflow (py::array_t<int> start_intervals,
                                              py::array_t<int> end_intervals);
  py::array_t<double>
  get_link_bus_passenger_inflow (py::array_t<int> start_intervals,
                                 py::array_t<int> end_intervals);
  py::array_t<double>
  get_link_walking_passenger_inflow (py::array_t<int> start_intervals,
                                     py::array_t<int> end_intervals);

  py::array_t<double> get_car_link_out_cc (int link_ID);
  py::array_t<double> get_car_link_in_cc (int link_ID);
  py::array_t<double> get_truck_link_out_cc (int link_ID);
  py::array_t<double> get_truck_link_in_cc (int link_ID);
  py::array_t<double> get_bus_link_out_passenger_cc (int link_ID);
  py::array_t<double> get_bus_link_in_passenger_cc (int link_ID);
  py::array_t<double> get_bus_link_to_busstop_in_cc (int link_ID);
  py::array_t<double> get_bus_link_to_busstop_out_cc (int link_ID);
  py::array_t<double> get_bus_link_from_busstop_in_cc (int link_ID);
  py::array_t<double> get_bus_link_from_busstop_out_cc (int link_ID);
  py::array_t<double> get_walking_link_out_cc (int link_ID);
  py::array_t<double> get_walking_link_in_cc (int link_ID);

  py::array_t<double> get_enroute_and_queue_veh_stats_agg ();
  py::array_t<double> get_enroute_and_queue_passenger_stats_agg ();
  py::array_t<double> get_queue_veh_each_link (py::array_t<int> useful_links,
                                               py::array_t<int> intervals);
  py::array_t<double>
  get_queue_passenger_each_link (py::array_t<int> useful_links,
                                 py::array_t<int> intervals);

  double get_car_link_out_num (int link_ID, double time);
  double get_truck_link_out_num (int link_ID, double time);
  double get_passenger_link_out_num (int link_ID, double time);
  double get_bus_stop_arrival_num (int busstop_ID, double time);
  double get_bus_stop_departure_num (int busstop_ID, double time);

  int register_paths (py::array_t<int> paths);
  int register_paths_driving (py::array_t<int> paths);
  int register_paths_bustransit (py::array_t<int> paths);
  int register_paths_pnr (py::array_t<int> paths);
  int register_paths_bus (py::array_t<int> paths);

  std::vector<bool> check_registered_links_in_registered_paths_driving ();
  std::vector<bool> check_registered_links_in_registered_paths_bus ();
  std::vector<bool> check_registered_links_in_registered_paths_walking ();

  py::array_t<bool> are_registered_links_in_registered_paths_driving ();
  py::array_t<bool> are_registered_links_in_registered_paths_bus ();
  py::array_t<bool> are_registered_links_in_registered_paths_walking ();

  py::array_t<int> generate_paths_to_cover_registered_links_driving ();
  py::array_t<int> generate_paths_to_cover_registered_links_bus_walking ();

  int save_passenger_path_table (const std::string &file_folder);
  int save_mode_path_table (const std::string &file_folder);

  py::array_t<int> link_seq_to_node_seq_driving (py::array_t<int> link_IDs);
  py::array_t<int> link_seq_to_node_seq_bustransit (py::array_t<int> link_IDs);
  py::array_t<int> node_seq_to_link_seq_driving (py::array_t<int> node_IDs);
  py::array_t<int> node_seq_to_link_seq_bustransit (py::array_t<int> node_IDs);

  // with m_mmdue -> m_link_tt_map and m_mmdue -> m_transitlink_tt_map
  py::array_t<double>
  get_passenger_path_cost_driving (py::array_t<int> link_IDs,
                                   py::array_t<double> start_intervals);
  py::array_t<double>
  get_passenger_path_cost_bus (py::array_t<int> link_IDs,
                               py::array_t<double> start_intervals);
  py::array_t<double>
  get_passenger_path_cost_pnr (py::array_t<int> link_IDs_driving,
                               py::array_t<int> link_IDs_bustransit,
                               py::array_t<double> start_intervals);

  py::array_t<double> get_path_tt_car (py::array_t<int> link_IDs,
                                       py::array_t<double> start_intervals);
  py::array_t<double> get_path_tt_truck (py::array_t<int> link_IDs,
                                         py::array_t<double> start_intervals);

  // with m_mmdue -> m_link_tt_map_truck
  py::array_t<double>
  get_registered_path_tt_truck (py::array_t<double> start_intervals);

  // with m_mmdue -> m_link_tt_map and m_mmdue -> m_transitlink_tt_map
  py::array_t<double>
  get_registered_path_tt_driving (py::array_t<double> start_intervals);
  py::array_t<double>
  get_registered_path_tt_bustransit (py::array_t<double> start_intervals);
  py::array_t<double>
  get_registered_path_tt_pnr (py::array_t<double> start_intervals);

  py::array_t<double> get_registered_path_distance_driving ();
  py::array_t<double> get_registered_path_distance_bustransit ();
  py::array_t<double> get_registered_path_distance_pnr ();

  // with m_mmdue -> m_link_tt_map and m_mmdue -> m_transitlink_cost_map
  py::array_t<double>
  get_registered_path_cost_driving (py::array_t<double> start_intervals);
  py::array_t<double>
  get_registered_path_cost_bustransit (py::array_t<double> start_intervals);
  py::array_t<double>
  get_registered_path_cost_pnr (py::array_t<double> start_intervals);

  int update_tdsp_tree ();
  py::array_t<int> get_lowest_cost_path (int start_interval, int o_node_ID,
                                         int d_node_ID);

  int build_link_cost_map (bool with_congestion_indicator = false);
  int get_link_queue_dissipated_time ();
  int build_link_cost_map_snapshot (int start_interval,
                                    bool in_simulation = false);
  int update_snapshot_route_table (int start_interval);
  py::array_t<int> get_lowest_cost_path_snapshot (int start_interval,
                                                  int o_node_ID, int d_node_ID);

  py::array_t<double> get_waiting_time_at_intersections ();
  py::array_t<double> get_waiting_time_at_intersections_car ();
  py::array_t<double> get_waiting_time_at_intersections_truck ();
  py::array_t<int> get_link_spillback ();

  py::array_t<double>
  get_car_dar_matrix_driving (py::array_t<int> start_intervals,
                              py::array_t<int> end_intervals);
  py::array_t<double>
  get_truck_dar_matrix_driving (py::array_t<int> start_intervals,
                                py::array_t<int> end_intervals);
  py::array_t<double> get_car_dar_matrix_pnr (py::array_t<int> start_intervals,
                                              py::array_t<int> end_intervals);
  py::array_t<double>
  get_bus_dar_matrix_bustransit_link (py::array_t<int> start_intervals,
                                      py::array_t<int> end_intervals);
  py::array_t<double>
  get_bus_dar_matrix_driving_link (py::array_t<int> start_intervals,
                                   py::array_t<int> end_intervals);
  py::array_t<double>
  get_passenger_dar_matrix_bustransit (py::array_t<int> start_intervals,
                                       py::array_t<int> end_intervals);
  py::array_t<double>
  get_passenger_dar_matrix_pnr (py::array_t<int> start_intervals,
                                py::array_t<int> end_intervals);

  py::array_t<double>
  get_car_dar_matrix_bus_driving_link (py::array_t<int> start_intervals,
                                       py::array_t<int> end_intervals);
  py::array_t<double>
  get_truck_dar_matrix_bus_driving_link (py::array_t<int> start_intervals,
                                         py::array_t<int> end_intervals);
  py::array_t<double> get_passenger_dar_matrix_bustransit_bus_link (
    py::array_t<int> start_intervals, py::array_t<int> end_intervals);
  py::array_t<double>
  get_passenger_dar_matrix_pnr_bus_link (py::array_t<int> start_intervals,
                                         py::array_t<int> end_intervals);

  // py::array_t<double>
  // get_passenger_bus_link_flow_relationship(py::array_t<int>start_intervals,
  // py::array_t<int>end_intervals);

  py::array_t<double>
  get_car_ltg_matrix_driving (py::array_t<int> start_intervals,
                              int threshold_timestamp);
  py::array_t<double> get_car_ltg_matrix_pnr (py::array_t<int> start_intervals,
                                              int threshold_timestamp);
  py::array_t<double>
  get_truck_ltg_matrix_driving (py::array_t<int> start_intervals,
                                int threshold_timestamp);
  py::array_t<double>
  get_passenger_ltg_matrix_bustransit (py::array_t<int> start_intervals,
                                       int threshold_timestamp);
  py::array_t<double>
  get_passenger_ltg_matrix_pnr (py::array_t<int> start_intervals,
                                int threshold_timestamp);

  int delete_all_agents ();

  MNM_MM_Due *m_mmdue;
  MNM_Dta_Multimodal *m_mmdta;
  bool m_is_mmdta_new;

  std::vector<MNM_Dlink_Multiclass *> m_link_vec_driving;
  std::vector<MNM_Walking_Link *> m_link_vec_walking;
  std::vector<MNM_Bus_Link *> m_link_vec_bus;
  std::vector<MNM_Dlink_Multiclass *> m_link_vec_bus_driving;

  std::vector<MNM_Path *> m_path_vec_driving;
  std::vector<MNM_Path *> m_path_vec_bus;
  std::vector<MNM_Path *> m_path_vec_bustransit;
  std::vector<MNM_Path *> m_path_vec_pnr;

  std::set<MNM_Path *> m_path_set_driving;
  std::set<MNM_Path *> m_path_set_bustransit;
  std::set<MNM_Path *> m_path_set_pnr;
  std::set<MNM_Path *> m_path_set_bus;

  std::set<TInt> m_pathID_set_driving;
  std::set<TInt> m_pathID_set_bustransit;
  std::set<TInt> m_pathID_set_pnr;
  std::set<TInt> m_pathID_set_bus;

  TInt m_num_path_driving;
  TInt m_num_path_bustransit;
  TInt m_num_path_pnr;
  TInt m_num_path_bus;

  // all paths from all modes
  std::vector<MNM_Path *> m_path_vec;
  std::set<MNM_Path *> m_path_set;
  std::unordered_map<TInt, std::pair<MNM_Path *, MNM_Passenger_Path_Base *>>
    m_ID_path_mapping;

  std::unordered_map<TInt, MNM_TDSP_Tree *> m_tdsp_tree_map_driving;
  std::unordered_map<TInt, MNM_TDSP_Tree *> m_tdsp_tree_map_bus;
};

Mmdta::Mmdta ()
{
  m_mmdta = nullptr;
  m_mmdue = nullptr;
  m_is_mmdta_new = false;

  m_num_path_driving = TInt (0);
  m_num_path_bustransit = TInt (0);
  m_num_path_pnr = TInt (0);
  m_num_path_bus = TInt (0);

  m_link_vec_driving = std::vector<MNM_Dlink_Multiclass *> ();
  m_link_vec_walking = std::vector<MNM_Walking_Link *> ();
  m_link_vec_bus = std::vector<MNM_Bus_Link *> ();
  m_link_vec_bus_driving = std::vector<MNM_Dlink_Multiclass *> ();

  m_path_vec_driving = std::vector<MNM_Path *> ();
  m_path_vec_bustransit = std::vector<MNM_Path *> ();
  m_path_vec_pnr = std::vector<MNM_Path *> ();
  m_path_vec_bus = std::vector<MNM_Path *> ();

  m_path_set_driving = std::set<MNM_Path *> ();
  m_path_set_bustransit = std::set<MNM_Path *> ();
  m_path_set_pnr = std::set<MNM_Path *> ();
  m_path_set_bus = std::set<MNM_Path *> ();

  m_pathID_set_driving = std::set<TInt> ();
  m_pathID_set_bustransit = std::set<TInt> ();
  m_pathID_set_pnr = std::set<TInt> ();
  m_pathID_set_bus = std::set<TInt> ();

  // all paths from all modes
  m_path_vec = std::vector<MNM_Path *> ();
  m_path_set = std::set<MNM_Path *> ();
  m_ID_path_mapping
    = std::unordered_map<TInt,
                         std::pair<MNM_Path *, MNM_Passenger_Path_Base *>> ();

  m_tdsp_tree_map_driving = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
  m_tdsp_tree_map_bus = std::unordered_map<TInt, MNM_TDSP_Tree *> ();
}

Mmdta::~Mmdta ()
{
  if (m_is_mmdta_new)
    {
      delete m_mmdta;
    }
  if (m_mmdue != nullptr)
    {
      delete m_mmdue;
    }

  m_link_vec_driving.clear ();
  m_link_vec_walking.clear ();
  m_link_vec_bus.clear ();
  m_link_vec_bus_driving.clear ();

  m_path_vec_driving.clear ();
  m_path_vec_bustransit.clear ();
  m_path_vec_pnr.clear ();
  m_path_vec_bus.clear ();

  m_path_set_driving.clear ();
  m_path_set_bustransit.clear ();
  m_path_set_pnr.clear ();
  m_path_set_bus.clear ();

  m_pathID_set_driving.clear ();
  m_pathID_set_bustransit.clear ();
  m_pathID_set_pnr.clear ();
  m_pathID_set_bus.clear ();

  m_path_vec.clear ();
  m_path_set.clear ();
  m_ID_path_mapping.clear ();

  if (!m_tdsp_tree_map_driving.empty ())
    {
      for (auto _it : m_tdsp_tree_map_driving)
        {
          delete _it.second;
        }
      m_tdsp_tree_map_driving.clear ();
    }
  if (!m_tdsp_tree_map_bus.empty ())
    {
      for (auto _it : m_tdsp_tree_map_bus)
        {
          delete _it.second;
        }
      m_tdsp_tree_map_bus.clear ();
    }
}

int
Mmdta::generate_shortest_pathsets (const std::string &folder, int max_iter,
                                   double mid_scale, double heavy_scale,
                                   double min_path_tt)
{
  m_mmdue = new MNM_MM_Due (folder);
  m_mmdue->initialize ();

  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (),
                 driving)
      != m_mmdue->m_mode_vec.end ())
    {
      Path_Table *_driving_path_table
        = MNM::build_shortest_driving_pathset (m_mmdue->m_mmdta->m_graph,
                                               m_mmdue->m_mmdta->m_od_factory,
                                               m_mmdue->m_od_mode_connectivity,
                                               m_mmdue->m_mmdta->m_link_factory,
                                               min_path_tt, max_iter, mid_scale,
                                               heavy_scale,
                                               2
                                                 * m_mmdue
                                                     ->m_total_assign_inter);
      printf ("driving pathset generated\n");
      MNM::save_driving_path_table (folder, _driving_path_table,
                                    "driving_path_table",
                                    "driving_path_table_buffer", true);
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
    }
  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (),
                 transit)
      != m_mmdue->m_mode_vec.end ())
    {
      Path_Table *_bustransit_path_table = MNM::
        build_shortest_bustransit_pathset (m_mmdue->m_mmdta
                                             ->m_bus_transit_graph,
                                           m_mmdue->m_mmdta->m_od_factory,
                                           m_mmdue->m_od_mode_connectivity,
                                           m_mmdue->m_mmdta
                                             ->m_transitlink_factory,
                                           min_path_tt, max_iter, mid_scale,
                                           heavy_scale,
                                           m_mmdue->m_total_assign_inter);
      printf ("bus transit pathset generated\n");
      MNM::save_bustransit_path_table (folder, _bustransit_path_table,
                                       "bustransit_path_table",
                                       "bustransit_path_table_buffer", true);
      printf ("bus transit pathset saved\n");

      for (auto _it : *_bustransit_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      _bustransit_path_table->clear ();
      delete _bustransit_path_table;
    }
  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (), pnr)
      != m_mmdue->m_mode_vec.end ())
    {
      PnR_Path_Table *_pnr_path_table
        = MNM::build_shortest_pnr_pathset (m_mmdue->m_mmdta->m_graph,
                                           m_mmdue->m_mmdta
                                             ->m_bus_transit_graph,
                                           m_mmdue->m_mmdta->m_od_factory,
                                           m_mmdue->m_od_mode_connectivity,
                                           m_mmdue->m_mmdta->m_link_factory,
                                           m_mmdue->m_mmdta
                                             ->m_transitlink_factory,
                                           min_path_tt, max_iter, mid_scale,
                                           heavy_scale,
                                           m_mmdue->m_total_assign_inter);
      printf ("pnr pathset generated\n");
      MNM::save_pnr_path_table (folder, _pnr_path_table, "pnr_path_table",
                                "pnr_path_table_buffer", true);
      printf ("pnr pathset saved\n");

      for (auto _it : *_pnr_path_table)
        {
          for (auto _it_it : *_it.second)
            {
              delete _it_it.second;
            }
          _it.second->clear ();
          delete _it.second;
        }
      _pnr_path_table->clear ();
      delete _pnr_path_table;
    }

  return 0;
}

int
Mmdta::initialize (const std::string &folder)
{
  m_mmdue = new MNM_MM_Due (folder);
  m_mmdue->initialize ();
  m_mmdue->init_passenger_path_table ();
  // m_mmdue -> m_passenger_path_table are created, at least one path for each
  // mode for each OD pair, if connected m_mmdue -> m_driving_path_table,
  // m_mmdue -> m_pnr_path_table, and m_mmdue -> m_bus_path_table are all
  // nullptr m_mmdue -> m_truck_path_table and m_mmdue -> m_bus_path_table are
  // pointing to the corresponding attributes of m_mmdue -> m_mmdta However, the
  // attributes of m_mmdue -> m_mmdta are based on the input files
  IAssert (m_mmdue->m_mmdta_config->get_string ("routing_type")
             == "Multimodal_Hybrid"
           || m_mmdue->m_mmdta_config->get_string ("routing_type")
                == "Multimodal_Hybrid_ColumnGeneration"
           || m_mmdue->m_mmdta_config->get_string ("routing_type")
                == "Multimodal_DUE_FixedPath"
           || m_mmdue->m_mmdta_config->get_string ("routing_type")
                == "Multimodal_DUE_ColumnGeneration");
  IAssert (m_mmdue->m_passenger_path_table != nullptr
           && !m_mmdue->m_passenger_path_table->empty ());

  m_mmdta = m_mmdue->m_mmdta;
  m_is_mmdta_new = false; // indicate m_mmdta is either created with new method
                          // or just the m_mmdue -> m_mmdta

  //    m_mmdta = new MNM_Dta_Multimodal(folder);
  //    m_mmdta -> build_from_files();
  //    m_mmdta -> hook_up_node_and_link();
  //    m_mmdta -> find_connected_pnr_parkinglot_for_destination();
  //    m_mmdta -> is_ok();
  //    m_is_mmdta_new = true;

  auto *_tmp_conf
    = new MNM_ConfReader (m_mmdue->m_file_folder + "/config.conf", "FIXED");
  if (_tmp_conf->get_int ("num_driving_path") > 0)
    {
      m_num_path_driving = _tmp_conf->get_int ("num_driving_path");
    }
  if (_tmp_conf->get_int ("num_bustransit_path") > 0)
    {
      m_num_path_bustransit = _tmp_conf->get_int ("num_bustransit_path");
    }
  if (_tmp_conf->get_int ("num_pnr_path") > 0)
    {
      m_num_path_pnr = _tmp_conf->get_int ("num_pnr_path");
    }
  if (_tmp_conf->get_int ("num_bus_routes") > 0)
    {
      m_num_path_bus = _tmp_conf->get_int ("num_bus_routes");
    }
  delete _tmp_conf;

  if (MNM_Routing_Multimodal_Hybrid *_routing
      = dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing))
    {
      // !!!!!! make sure path_IDs across all modes are unique
      printf ("MNM_Routing_Multimodal_Hybrid start load ID path mapping\n");
      // car and truck share the same path_table
      // NOTE: be careful with the construction of m_mmdue ->
      // m_passenger_path_table in MNM::build_existing_passenger_pathset() the
      // path in passenger_path_table and that in path_table from m_mmdta SHOULD
      // BE in the same memory address because the registered link's cc_tree
      // stored the path in path_table in m_mmdta This is to ensure the
      // correctness of DAR extraction
      MNM::get_ID_path_mapping_all_mode (m_ID_path_mapping,
                                         _routing->m_routing_fixed_car
                                           ->m_path_table,
                                         _routing->m_routing_bus_fixed
                                           ->m_bus_path_table,
                                         _routing->m_routing_car_pnr_fixed
                                           ->m_pnr_path_table,
                                         _routing->m_routing_passenger_fixed
                                           ->m_bustransit_path_table,
                                         m_mmdue->m_passenger_path_table,
                                         m_num_path_driving,
                                         m_num_path_bustransit, m_num_path_pnr,
                                         m_num_path_bus);
      printf ("MNM_Routing_Multimodal_Hybrid mapping size %d\n",
              (int) m_ID_path_mapping.size ());
      return 0;
    }

  printf ("xxx\n");
  throw std::runtime_error ("Mmdta:: Routing type not implemented in API");
  return -1;
}

int
Mmdta::initialize_mmdue (const std::string &folder)
{
  m_mmdue = new MNM_MM_Due (folder);
  printf ("================================ DUE set! "
          "=================================\n");

  m_mmdue->initialize ();
  printf ("========================= Finished initialization! "
          "========================\n");

  m_mmdue->init_passenger_path_table ();
  m_mmdue->init_passenger_path_flow ();
  printf ("====================== Finished initialize passenger path flow! "
          "====================\n");
  return 0;
}

bool
Mmdta::check_input_files ()
{
  return m_mmdta->is_ok ();
}

int
Mmdta::install_cc ()
{
  // car and truck
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      m_link_vec_driving[i]->install_cumulative_curve_multiclass ();
    }
  // passenger
  for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
    {
      m_link_vec_walking[i]->install_cumulative_curve ();
    }
  // bus and passenger
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      // passenger
      m_link_vec_bus[i]->install_cumulative_curve ();
      // bus
      m_link_vec_bus[i]->m_from_busstop->install_cumulative_curve_multiclass ();
      m_link_vec_bus[i]->m_to_busstop->install_cumulative_curve_multiclass ();
    }
  // truck traversing bus links
  for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
    {
      m_link_vec_bus_driving[i]->install_cumulative_curve_multiclass ();
    }
  return 0;
}

int
Mmdta::install_cc_tree ()
{
  // car and truck
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      m_link_vec_driving[i]->install_cumulative_curve_tree_multiclass ();
    }
  // passenger
  for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
    {
      m_link_vec_walking[i]->install_cumulative_curve_tree ();
    }
  // bus and passenger
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      m_link_vec_bus[i]->install_cumulative_curve_tree ();
    }
  // truck traversing bus links
  for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
    {
      m_link_vec_bus_driving[i]->install_cumulative_curve_tree_multiclass ();
    }
  return 0;
}

int
Mmdta::run_whole (bool verbose)
{
  IAssert (m_mmdue->m_mmdta_config->get_string ("routing_type")
           == "Multimodal_Hybrid");
  m_mmdta->pre_loading ();
  m_mmdta->loading (verbose);
  return 0;
}

int
Mmdta::run_mmdue (const std::string &folder, bool verbose)
{
  IAssert (m_mmdue->m_mmdta_config->get_string ("routing_type")
             == "Multimodal_DUE_FixedPath"
           || m_mmdue->m_mmdta_config->get_string ("routing_type")
                == "Multimodal_DUE_ColumnGeneration");

  MNM_ConfReader *config = new MNM_ConfReader (folder + "/config.conf", "STAT");
  std::string rec_folder = config->get_string ("rec_folder");

  MNM_Dta_Multimodal *mmdta;

  std::string gap_file_name = folder + "/" + rec_folder + "/gap_iteration";
  std::ofstream gap_file;
  gap_file.open (gap_file_name, std::ofstream::out);
  if (!gap_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + gap_file_name);
    }

  std::string emission_file_name = folder + "/" + rec_folder + "/emission";
  std::ofstream emission_file;
  emission_file.open (emission_file_name, std::ofstream::out);
  if (!emission_file.is_open ())
    {
      throw std::runtime_error ("failed to open file: " + emission_file_name);
    }

  TFlt gap;
  for (int i = 0; i < m_mmdue->m_max_iter; ++i)
    {
      printf ("---------- Iteration %d ----------\n", i);

      // DNL using dta, new dta is built from scratch
      mmdta = m_mmdue->run_mmdta (verbose);

      // update time dependent cost and save existing path table
      m_mmdue->build_link_cost_map (mmdta);
      m_mmdue->update_path_table_cost (mmdta);

      MNM::save_driving_path_table (folder, m_mmdue->m_driving_path_table,
                                    "driving_path_table",
                                    "driving_path_table_buffer", true);
      MNM::save_bustransit_path_table (folder, m_mmdue->m_bustransit_path_table,
                                       "bustransit_path_table",
                                       "bustransit_path_table_buffer", true);
      MNM::save_pnr_path_table (folder, m_mmdue->m_pnr_path_table,
                                "pnr_path_table", "pnr_path_table_buffer",
                                true);

      // calculate gap
      // with departure time choice
      // gap = m_mmdue -> compute_merit_function(mmdta);
      // fixed departure time choice
      gap = m_mmdue->compute_merit_function_fixed_departure_time_choice (mmdta);
      printf ("\n\n*******************GAP = %lf*******************\n\n",
              (float) gap);
      gap_file << std::to_string (gap) + "\n";

      // search for the lowest disutility route and update path flow
      // with departure time choice
      // m_mmdue->update_path_table(mmdta, i);
      // fixed departure time choice
      // m_mmdue->update_path_table_fixed_departure_time_choice(mmdta, i);
      // gradient projection
      m_mmdue->update_path_table_gp_fixed_departure_time_choice (mmdta, i);

      if (i == m_mmdue->m_max_iter - 1)
        {
          TInt _count_car, _count_car_pnr, _count_truck, _count_bus,
            _count_passenger, _count_passenger_pnr;
          TFlt _tot_tt_car, _tot_tt_truck, _tot_tt_bus, _tot_tt_passenger;
          _count_car
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_finished_car;
          _count_car_pnr
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_finished_car_pnr;
          _count_truck
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_finished_truck;
          _count_bus
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_finished_bus;
          _tot_tt_car
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_total_time_car
              * mmdta->m_unit_time / 3600.0;
          _tot_tt_truck
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_total_time_truck
              * mmdta->m_unit_time / 3600.0;
          _tot_tt_bus
            = dynamic_cast<MNM_Veh_Factory_Multimodal *> (mmdta->m_veh_factory)
                ->m_total_time_bus
              * mmdta->m_unit_time / 3600.0;
          printf ("\nTotal driving car: %d, Total pnr car:, %d, Total truck: "
                  "%d, Total bus: %d, Total car tt: %.2f hours, Total truck "
                  "tt: %.2f hours, Total bus tt: %.2f hours\n",
                  int (_count_car), int (_count_car_pnr), int (_count_truck),
                  int (_count_bus), float (_tot_tt_car), float (_tot_tt_truck),
                  float (_tot_tt_bus));
          _count_passenger = mmdta->m_passenger_factory->m_finished_passenger;
          _count_passenger_pnr
            = mmdta->m_passenger_factory->m_finished_passenger_pnr;
          _tot_tt_passenger = mmdta->m_passenger_factory->m_total_time_passenger
                              * mmdta->m_unit_time / 3600.0;
          printf ("Total passenger: %d, Total pnr passenger: %d, Total Total "
                  "tt: %.2f hours\n",
                  int (_count_passenger), int (_count_passenger_pnr),
                  float (_tot_tt_passenger));

          // print to terminal
          // freopen("CON", "w", stdout);
          // print to file
          // freopen((folder + "/" + rec_folder + "/output.log").c_str(), "w",
          // stdout); mmdta -> m_emission -> output();
          emission_file << mmdta->m_emission->output ();
        }
      delete mmdta;
    }

  gap_file.close ();
  emission_file.close ();

  delete config;
  printf ("====================== Finished mmdue! ====================\n");

  return 0;
}

int
Mmdta::run_mmdta_adaptive (const std::string &folder, int cong_frequency,
                           bool verbose)
{
  IAssert (m_mmdue->m_mmdta_config->get_string ("routing_type")
           == "Multimodal_Hybrid");

  m_mmdta = m_mmdue->run_mmdta_adaptive (verbose);
  m_is_mmdta_new = false;

  // m_mmdue -> build_link_cost_map(m_mmdta, false);

  MNM_ConfReader *config = new MNM_ConfReader (folder + "/config.conf", "STAT");
  std::string rec_folder = config->get_string ("rec_folder");

  print_simulation_results (folder + "/" + rec_folder, cong_frequency);

  if (cong_frequency > 0)
    {
      m_mmdue->save_od_demand_split (m_mmdta, folder + "/" + rec_folder);

      TFlt _tot_demand = 0;
      for (auto _o_it : m_mmdue->m_passenger_demand)
        {
          for (auto _d_it : _o_it.second)
            {
              for (int i = 0; i < m_mmdue->m_total_assign_inter; ++i)
                {
                  _tot_demand += _d_it.second[i];
                }
            }
        }
      for (auto _it : m_mmdue->m_mode_share)
        {
          m_mmdue->m_mode_share.find (_it.first)->second
            = _it.second / _tot_demand;
        }

      std::string _str;
      std::ofstream _vis_file4;
      _vis_file4.open (folder + "/" + rec_folder + "/mode_share.txt",
                       std::ofstream::out);
      if (!_vis_file4.is_open ())
        {
          throw std::runtime_error ("failed to open_vis_file4");
        }
      _str = "driving, bus_transit, pnr\n";
      if (m_mmdue->m_mode_share.find (driving) != m_mmdue->m_mode_share.end ())
        {
          _str += std::to_string (m_mmdue->m_mode_share.find (driving)->second)
                  + " ";
        }
      else
        {
          _str += "0 ";
        }
      if (m_mmdue->m_mode_share.find (transit) != m_mmdue->m_mode_share.end ())
        {
          _str += std::to_string (m_mmdue->m_mode_share.find (transit)->second)
                  + " ";
        }
      else
        {
          _str += "0 ";
        }
      if (m_mmdue->m_mode_share.find (pnr) != m_mmdue->m_mode_share.end ())
        {
          _str
            += std::to_string (m_mmdue->m_mode_share.find (pnr)->second) + "\n";
        }
      else
        {
          _str += "0\n";
        }

      _vis_file4 << _str;
      if (_vis_file4.is_open ())
        _vis_file4.close ();

      // freopen((folder + "/" + rec_folder + "/emission_output.log").c_str(),
      // "w", stdout); m_mmdta -> m_emission -> output();
    }
  delete config;

  return 0;
}

int
Mmdta::register_links_driving (py::array_t<int> links_driving)
{
  if (m_link_vec_driving.size () > 0)
    {
      printf ("Warning, Mmdta::register_links_driving, link exists\n");
      m_link_vec_driving.clear ();
    }
  auto links_buf = links_driving.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Number of dimensions must be one");
    }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_link_factory->get_link (TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          if (std::find (m_link_vec_driving.begin (), m_link_vec_driving.end (),
                         _link)
              != m_link_vec_driving.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::register_links_driving, link does not "
                "exist");
            }
          else
            {
              m_link_vec_driving.push_back (_mclink);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mmdta::register_links_driving: link type is not multiclass");
        }
    }
  return 0;
}

int
Mmdta::register_links_walking (py::array_t<int> links_walking)
{
  if (m_link_vec_walking.size () > 0)
    {
      printf ("Warning, Mmdta::register_links_walking, link exists\n");
      m_link_vec_walking.clear ();
    }
  auto links_buf = links_walking.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Number of dimensions must be one");
    }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Transit_Link *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_transitlink_factory->get_transit_link (
        TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (MNM_Walking_Link *_wlink = dynamic_cast<MNM_Walking_Link *> (_link))
        {
          if (std::find (m_link_vec_walking.begin (), m_link_vec_walking.end (),
                         _wlink)
              != m_link_vec_walking.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::register_links_walking, link does not "
                "exist");
            }
          else
            {
              m_link_vec_walking.push_back (_wlink);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mmdta::register_links_walking: link type is not walking");
        }
    }
  return 0;
}

int
Mmdta::register_links_bus (py::array_t<int> links_bus)
{
  if (m_link_vec_bus.size () > 0)
    {
      printf ("Warning, Mmdta::register_links_bus, link exists\n");
      m_link_vec_bus.clear ();
    }
  auto links_buf = links_bus.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Number of dimensions must be one");
    }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Transit_Link *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_transitlink_factory->get_transit_link (
        TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (MNM_Bus_Link *_blink = dynamic_cast<MNM_Bus_Link *> (_link))
        {
          if (std::find (m_link_vec_bus.begin (), m_link_vec_bus.end (), _blink)
              != m_link_vec_bus.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::register_links_bus, link does not exist");
            }
          else
            {
              m_link_vec_bus.push_back (_blink);
            }
        }
      else
        {
          throw std::runtime_error (
            "Mmdta::register_links_bus: link type is not bus");
        }
    }
  return 0;
}

py::array_t<double>
Mmdta::get_links_overlapped_bus_driving ()
{
  if (m_link_vec_bus.empty ())
    {
      throw std::runtime_error ("bus links do not exist");
    }
  struct _record
  {
    TInt bus_link_ID;
    TInt driving_link_ID;
    TFlt length_proportion;
  };
  std::vector<_record *> _overlapped_driving_link_records
    = std::vector<_record *> ();
  for (auto _bus_link : m_link_vec_bus)
    {
      size_t i = 0;
      for (auto _driving_link : _bus_link->m_overlapped_driving_link_vec)
        {
          auto new_record = new _record ();
          new_record->bus_link_ID = _bus_link->m_link_ID;
          new_record->driving_link_ID = _driving_link->m_link_ID;
          new_record->length_proportion
            = _bus_link->m_overlapped_driving_link_length_portion_vec[i];
          _overlapped_driving_link_records.push_back (new_record);
          i++;
        }
    }

  int new_shape[2] = { (int) _overlapped_driving_link_records.size (), 3 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  _record *tmp_record;
  for (size_t i = 0; i < _overlapped_driving_link_records.size (); ++i)
    {
      tmp_record = _overlapped_driving_link_records[i];
      result_ptr[i * 3 + 0] = (double) tmp_record->bus_link_ID ();
      result_ptr[i * 3 + 1] = (double) tmp_record->driving_link_ID ();
      result_ptr[i * 3 + 2] = (double) tmp_record->length_proportion ();
    }

  for (size_t i = 0; i < _overlapped_driving_link_records.size (); ++i)
    {
      delete _overlapped_driving_link_records[i];
    }
  _overlapped_driving_link_records.clear ();
  return result;
}

int
Mmdta::register_links_bus_driving (py::array_t<int> links_bus_driving)
{
  if (!m_link_vec_bus_driving.empty ())
    {
      printf ("Warning, Mmdta::register_links_bus_driving, link exists\n");
      m_link_vec_bus_driving.clear ();
    }
  auto links_buf = links_bus_driving.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Number of dimensions must be one");
    }
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_link_factory->get_link (TInt (links_ptr[i]));
      // printf("%d\n", links_ptr[i]);
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          if (std::find (m_link_vec_bus_driving.begin (),
                         m_link_vec_bus_driving.end (), _link)
              != m_link_vec_bus_driving.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::register_links_bus_driving, link does not "
                "exist");
            }
          else
            {
              m_link_vec_bus_driving.push_back (_mclink);
            }
        }
      else
        {
          throw std::runtime_error ("Mmdta::register_links_bus_driving: "
                                    "link type is not multiclass");
        }
    }
  return 0;
}

int
Mmdta::get_cur_loading_interval ()
{
  return m_mmdta->m_current_loading_interval ();
}

std::string
Mmdta::print_emission_stats ()
{
  return m_mmdta->m_emission->output ();
}

int
Mmdta::print_simulation_results (const std::string &folder, int cong_frequency)
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
  MNM_Transit_Link *_transit_link;
  // MNM_Walking_Link *_walking_link;
  std::string _str1;
  std::string _str2;
  TInt _current_inter = m_mmdta->m_current_loading_interval;
  std::ofstream _vis_file2;
  std::ofstream _vis_file3;
  if (output_link_cong)
    {
      _vis_file2.open (folder + "/driving_link_cong_raw.txt",
                       std::ofstream::out);
      if (!_vis_file2.is_open ())
        {
          throw std::runtime_error ("failed to open _vis_file2");
        }
      _vis_file3.open (folder + "/transit_link_cong_raw.txt",
                       std::ofstream::out);
      if (!_vis_file3.is_open ())
        {
          throw std::runtime_error ("failed to open _vis_file3");
        }

      _str1 = "timestamp (intervals), driving_link_ID, car_inflow, "
              "truck_inflow, car_tt (s), truck_tt (s), car_fftt (s), "
              "truck_fftt (s), car_speed (mph), truck_speed (mph)\n";
      _str2 = "timestamp (intervals), bus_transit_link_ID, "
              "bus_transit_link_type, passenger_inflow, tt (s), fftt (s)\n";
      _vis_file2 << _str1;
      _vis_file3 << _str2;

      TInt _iter = 0;
      while (_iter + cong_frequency <= _current_inter)
        {
          if (_iter % cong_frequency == 0 || _iter == _current_inter - 1)
            {
              printf ("Current loading interval: %d\n", int (_iter));
              for (auto _link_it : m_mmdta->m_link_factory->m_link_map)
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
                  // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                  // m_current_loading_interval) * m_mmdta -> m_unit_time) + "
                  // "; _str1 +=
                  // std::to_string(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m,
                  // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                  // m_current_loading_interval) * m_mmdta -> m_unit_time) + "
                  // ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_car_robust (
                               _link_m, TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_mmdta->m_unit_time,
                               m_mmdta->m_current_loading_interval)
                             * m_mmdta->m_unit_time)
                           + " ";
                  _str1 += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_truck_robust (
                               _link_m, TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_mmdta->m_unit_time,
                               m_mmdta->m_current_loading_interval)
                             * m_mmdta->m_unit_time)
                           + " ";
                  _str1 += std::to_string (_link_m->get_link_freeflow_tt_car ())
                           + " ";
                  _str1
                    += std::to_string (_link_m->get_link_freeflow_tt_truck ())
                       + " ";
                  // _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time_car(_link_m,
                  // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                  // m_current_loading_interval) * m_mmdta -> m_unit_time) *
                  // 3600 / 1600) + " "; _str1 += std::to_string(_link_m ->
                  // m_length/(MNM_DTA_GRADIENT::get_travel_time_truck(_link_m,
                  // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                  // m_current_loading_interval) * m_mmdta -> m_unit_time) *
                  // 3600 / 1600) + "\n";
                  _str1 += std::to_string (
                             _link_m->m_length
                             / (MNM_DTA_GRADIENT::get_travel_time_car_robust (
                                  _link_m, TFlt (_iter + 1),
                                  TFlt (_iter + cong_frequency + 1),
                                  m_mmdta->m_unit_time,
                                  m_mmdta->m_current_loading_interval)
                                * m_mmdta->m_unit_time)
                             * 3600 / 1600)
                           + " ";
                  _str1 += std::to_string (
                             _link_m->m_length
                             / (MNM_DTA_GRADIENT::get_travel_time_truck_robust (
                                  _link_m, TFlt (_iter + 1),
                                  TFlt (_iter + cong_frequency + 1),
                                  m_mmdta->m_unit_time,
                                  m_mmdta->m_current_loading_interval)
                                * m_mmdta->m_unit_time)
                             * 3600 / 1600)
                           + "\n";
                  _vis_file2 << _str1;
                }
              for (auto _link_it :
                   m_mmdta->m_transitlink_factory->m_transit_link_map)
                {
                  _transit_link = _link_it.second;
                  _str2 = std::to_string (int (_iter)) + " ";
                  _str2 += std::to_string (_transit_link->m_link_ID ()) + " ";
                  _str2 += std::to_string (_transit_link->m_link_type) + " ";
                  _str2
                    += std::to_string (
                         MNM_DTA_GRADIENT::
                           get_link_inflow_passenger (_transit_link,
                                                      TFlt (_iter),
                                                      TFlt (_iter
                                                            + cong_frequency)))
                       + " ";
                  if (_transit_link->m_link_type == MNM_TYPE_WALKING_MULTIMODAL)
                    {
                      // _str2 +=
                      // std::to_string(MNM_DTA_GRADIENT::get_travel_time_walking(dynamic_cast<MNM_Walking_Link*>(_transit_link),
                      // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                      // m_current_loading_interval) * m_mmdta -> m_unit_time) +
                      // " ";
                      _str2
                        += std::to_string (
                             MNM_DTA_GRADIENT::get_travel_time_walking_robust (
                               dynamic_cast<MNM_Walking_Link *> (_transit_link),
                               TFlt (_iter + 1),
                               TFlt (_iter + cong_frequency + 1),
                               m_mmdta->m_unit_time,
                               m_mmdta->m_current_loading_interval)
                             * m_mmdta->m_unit_time)
                           + " ";
                      _str2
                        += std::to_string (
                             dynamic_cast<MNM_Walking_Link *> (_transit_link)
                               ->m_fftt)
                           + "\n";
                    }
                  else
                    {
                      // _str2 +=
                      // std::to_string(MNM_DTA_GRADIENT::get_travel_time_bus(dynamic_cast<MNM_Bus_Link*>(_transit_link),
                      // TFlt(_iter + 1), m_mmdta -> m_unit_time, m_mmdta ->
                      // m_current_loading_interval, m_mmdta -> m_explicit_bus,
                      // true,  !m_mmdta -> m_explicit_bus) * m_mmdta ->
                      // m_unit_time) + " ";
                      _str2 += std::to_string (
                                 MNM_DTA_GRADIENT::get_travel_time_bus_robust (
                                   dynamic_cast<MNM_Bus_Link *> (_transit_link),
                                   TFlt (_iter + 1),
                                   TFlt (_iter + cong_frequency + 1),
                                   m_mmdta->m_unit_time,
                                   m_mmdta->m_current_loading_interval,
                                   cong_frequency, m_mmdta->m_explicit_bus,
                                   true, !m_mmdta->m_explicit_bus)
                                 * m_mmdta->m_unit_time)
                               + " ";
                      _str2 += std::to_string (
                                 dynamic_cast<MNM_Bus_Link *> (_transit_link)
                                   ->m_fftt)
                               + "\n";
                    }
                  _vis_file3 << _str2;
                }
            }
          _iter += 1;
        }

      // save cc of some links
      // _str1 = "\n\n **************************** driving link cc
      // ****************************"; _vis_file2 << _str1; for (size_t i = 0;
      // i < m_link_vec_driving.size(); ++i) {
      //     _link_m = m_link_vec_driving[i];
      //     _str1 = "\n\n------ link_ID: " +
      //     std::to_string(_link_m->m_link_ID()) + " ------\n"; _str1 +=
      //     "\nm_N_in_car: \n"; _str1 += _link_m->m_N_in_car->to_string();
      //     _str1 += "\nm_N_out_car: \n";
      //     _str1 += _link_m->m_N_out_car->to_string();
      //     _str1 += "\nm_N_in_truck: \n";
      //     _str1 += _link_m->m_N_in_truck->to_string();
      //     _str1 += "\nm_N_out_truck: \n";
      //     _str1 += _link_m->m_N_out_truck->to_string();
      //     _vis_file2 << _str1;
      // }

      // _str1 = "\n\n **************************** driving link cc
      // ****************************"; _vis_file2 << _str1; for (auto _link_it
      // : m_mmdta->m_link_factory->m_link_map) {
      //     _link = _link_it.second;
      //     if (_link->m_link_ID() == 4) {
      //         _link_m = dynamic_cast<MNM_Dlink_Multiclass *>(_link);
      //         _str1 += "\nlink_ID: " + std::to_string(_link->m_link_ID());
      //         _str1 +="\nm_N_in_car: \n";
      //         _str1 += _link_m->m_N_in_car->to_string();
      //         _str1 +="\nm_N_out_car: \n";
      //         _str1 += _link_m->m_N_out_car->to_string();
      //         _str1 +="\nm_N_in_truck: \n";
      //         _str1 += _link_m->m_N_in_truck->to_string();
      //         _str1 +="\nm_N_out_truck: \n";
      //         _str1 += _link_m->m_N_out_truck->to_string();
      //         _vis_file2 << _str1;
      //     }
      // }

      if (_vis_file2.is_open ())
        _vis_file2.close ();
      if (_vis_file3.is_open ())
        _vis_file3.close ();
    }
  return 0;
}

py::array_t<double>
Mmdta::get_travel_stats ()
{
  // finished
  TInt _count_car = 0, _count_pnr_car = 0, _count_truck = 0, _count_bus = 0,
       _count_passenger = 0;
  TFlt _tot_tt_car = 0.0, _tot_tt_truck = 0.0, _tot_tt_bus = 0.0,
       _tot_tt_passenger = 0.0;

  auto *_veh_factory
    = dynamic_cast<MNM_Veh_Factory_Multimodal *> (m_mmdta->m_veh_factory);
  _count_car = _veh_factory->m_finished_car;
  _count_pnr_car = _veh_factory->m_finished_car_pnr;
  _count_truck = _veh_factory->m_finished_truck;
  _count_bus = _veh_factory->m_finished_bus;
  _count_passenger = m_mmdta->m_passenger_factory->m_finished_passenger;

  _tot_tt_car = _veh_factory->m_total_time_car * m_mmdta->m_unit_time / 3600.0;
  _tot_tt_truck
    = _veh_factory->m_total_time_truck * m_mmdta->m_unit_time / 3600.0;
  _tot_tt_bus = _veh_factory->m_total_time_bus * m_mmdta->m_unit_time / 3600.0;
  _tot_tt_passenger = m_mmdta->m_passenger_factory->m_total_time_passenger
                      * m_mmdta->m_unit_time / 3600.0;

  // unfinished
  MNM_Veh_Multimodal *_veh;
  MNM_Passenger *_passenger;
  int _end_time = get_cur_loading_interval ();

  for (auto _map_it : m_mmdta->m_veh_factory->m_veh_map)
    {
      _veh = dynamic_cast<MNM_Veh_Multimodal *> (_map_it.second);
      IAssert (_veh->m_finish_time < 0);
      if (_veh->m_class == 0)
        {
          if (_veh->get_ispnr ())
            {
              _count_pnr_car += 1;
            }
          else
            {
              _count_car += 1;
            }
          _tot_tt_car
            += (_end_time - _veh->m_start_time) * m_mmdta->m_unit_time / 3600.0;
        }
      else
        {
          if (_veh->m_bus_route_ID == TInt (-1))
            {
              _count_truck += 1;
            }
          else
            {
              _count_bus += 1;
            }
          _tot_tt_bus
            += (_end_time - _veh->m_start_time) * m_mmdta->m_unit_time / 3600.0;
        }
    }

  for (auto _map_it : m_mmdta->m_passenger_factory->m_passenger_map)
    {
      _passenger = _map_it.second;
      IAssert (_passenger->m_finish_time < 0);
      _count_passenger += 1;
      _tot_tt_passenger += (_end_time - _passenger->m_start_time)
                           * m_mmdta->m_unit_time / 3600.0;
    }

  // // for vehicles and passenger not deleted
  // MNM_Veh_Multimodal *_veh;
  // MNM_Passenger *_passenger;
  // int _end_time = get_cur_loading_interval();

  // for (auto _map_it : m_mmdta -> m_veh_factory -> m_veh_map){
  //     _veh = dynamic_cast<MNM_Veh_Multimodal *>(_map_it.second);
  //     if (_veh -> m_class == 0){
  //         if (_veh -> get_ispnr()) {
  //             _count_pnr_car += 1;
  //         }
  //         else {
  //             _count_car += 1;
  //         }
  //         if (_veh -> m_finish_time > 0) {
  //             _tot_tt_car += (_veh -> m_finish_time - _veh -> m_start_time) *
  //             m_mmdta -> m_unit_time / 3600.0;
  //         }
  //         else {
  //             _tot_tt_car += (_end_time - _veh -> m_start_time) * m_mmdta ->
  //             m_unit_time / 3600.0;
  //         }
  //     }
  //     else {
  //         if (_veh -> m_bus_route_ID == TInt(-1)) {
  //             _count_truck += 1;
  //             if (_veh -> m_finish_time > 0) {
  //                 _tot_tt_truck += (_veh -> m_finish_time - _veh ->
  //                 m_start_time) * m_mmdta -> m_unit_time / 3600.0;
  //             }
  //             else {
  //                 _tot_tt_truck += (_end_time - _veh -> m_start_time) *
  //                 m_mmdta -> m_unit_time / 3600.0;
  //             }
  //         }
  //         else {
  //             _count_bus += 1;
  //             if (_veh -> m_finish_time > 0) {
  //                 _tot_tt_bus += (_veh -> m_finish_time - _veh ->
  //                 m_start_time) * m_mmdta -> m_unit_time / 3600.0;
  //             }
  //             else {
  //                 _tot_tt_bus += (_end_time - _veh -> m_start_time) * m_mmdta
  //                 -> m_unit_time / 3600.0;
  //             }
  //         }

  //     }
  // }

  // for (auto _map_it : m_mmdta -> m_passenger_factory -> m_passenger_map){
  //     if (_map_it.second -> m_finish_time > 0) {
  //         _passenger = _map_it.second;
  //         _count_passenger += 1;
  //         if (_passenger -> m_finish_time > 0) {
  //             _tot_tt_passenger += (_passenger -> m_finish_time - _passenger
  //             -> m_start_time) * m_mmdta -> m_unit_time / 3600.0;
  //         }
  //         else {
  //             _tot_tt_passenger += (_end_time - _passenger -> m_start_time) *
  //             m_mmdta -> m_unit_time / 3600.0;
  //         }
  //     }
  // }

  // printf("\n\nTotal car: %d, Total truck: %d, Total bus : %d, Total
  // passenger: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total
  // bus tt: %.2f hours, Total passenger tt: %.2f hours\n\n",
  //         int(_count_car/m_mmdta -> m_flow_scalar), int(_count_truck/m_mmdta
  //         -> m_flow_scalar), int(_count_bus/m_mmdta -> m_flow_scalar),
  //         int(_count_passenger), float(_tot_tt_car/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_truck/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_bus/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_passenger));
  // m_mmdta -> m_emission -> output();

  // add flow_scalar to passenger
  // printf("\n\nTotal car: %d, Total truck: %d, Total bus : %d, Total
  // passenger: %d, Total car tt: %.2f hours, Total truck tt: %.2f hours, Total
  // bus tt: %.2f hours, Total passenger tt: %.2f hours\n\n",
  //         int(_count_car/m_mmdta -> m_flow_scalar), int(_count_truck/m_mmdta
  //         -> m_flow_scalar), int(_count_bus/m_mmdta -> m_flow_scalar),
  //         int(_count_passenger/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_car/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_truck/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_bus/m_mmdta -> m_flow_scalar),
  //         float(_tot_tt_passenger/m_mmdta -> m_flow_scalar));
  // m_mmdta -> m_emission -> output();

  // for all released vehicles and passengers
  int new_shape[1] = { 9 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  result_ptr[0] = _count_car / m_mmdta->m_flow_scalar;
  result_ptr[1] = _count_pnr_car / m_mmdta->m_flow_scalar;
  result_ptr[2] = _count_truck / m_mmdta->m_flow_scalar;
  result_ptr[3] = _count_bus / m_mmdta->m_flow_scalar;
  // result_ptr[4] = _count_passenger;
  // add flow_scalar to passenger
  result_ptr[4] = _count_passenger / m_mmdta->m_flow_scalar;
  result_ptr[5] = _tot_tt_car / m_mmdta->m_flow_scalar;   // hours
  result_ptr[6] = _tot_tt_truck / m_mmdta->m_flow_scalar; // hours
  result_ptr[7] = _tot_tt_bus / m_mmdta->m_flow_scalar;   // hours
  // result_ptr[8] = _tot_tt_passenger;  // hours
  // add flow_scalar to passenger
  result_ptr[8] = _tot_tt_passenger / m_mmdta->m_flow_scalar; // hours
  return result;
}

int
Mmdta::register_paths (py::array_t<int> paths)
{
  if (m_path_vec.size () > 0)
    {
      printf ("Warning, Mmdta::register_paths, path exists\n");
      m_path_vec.clear ();
      m_path_set.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Mmdta::register_paths: Number of dimensions must be one");
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
          throw std::runtime_error ("Mmdta::register_paths: No such path");
        }
      else
        {
          m_path_vec.push_back (m_ID_path_mapping[_path_ID].first);
          // if (m_ID_path_mapping[_path_ID].first -> m_path_type == driving) {
          //     m_path_vec_driving.push_back(m_ID_path_mapping[_path_ID].first);
          //     m_pathID_set_driving.insert(_path_ID);
          // }
          // else if (m_ID_path_mapping[_path_ID].first -> m_path_type ==
          // transit) {
          //     m_path_vec_bustransit.push_back(m_ID_path_mapping[_path_ID].first);
          //     m_pathID_set_bustransit.insert(_path_ID);
          // }
          // else if (m_ID_path_mapping[_path_ID].first -> m_path_type == pnr) {
          //     m_path_vec_pnr.push_back(m_ID_path_mapping[_path_ID].first);
          //     m_pathID_set_pnr.insert(_path_ID);
          // }
          // else if (m_ID_path_mapping[_path_ID].first -> m_path_type ==
          // bus_route) {
          //     m_path_vec_bus.push_back(m_ID_path_mapping[_path_ID].first);
          //     m_pathID_set_bus.insert(_path_ID);
          // }
        }
    }
  // m_path_set_driving = std::set<MNM_Path*>(m_path_vec_driving.begin(),
  // m_path_vec_driving.end()); m_path_set_bustransit =
  // std::set<MNM_Path*>(m_path_vec_bustransit.begin(),
  // m_path_vec_bustransit.end()); m_path_set_pnr =
  // std::set<MNM_Path*>(m_path_vec_pnr.begin(), m_path_vec_pnr.end());
  // m_path_set_bus = std::set<MNM_Path*>(m_path_vec_bus.begin(),
  // m_path_vec_bus.end());
  m_path_set = std::set<MNM_Path *> (m_path_vec.begin (), m_path_vec.end ());
  // if (m_path_set_driving.size() != m_path_vec_driving.size() ||
  // m_pathID_set_driving.size() != m_path_vec_driving.size()) {
  //     printf("repeated driving paths\n");
  //     exit(-1);
  // }
  // if (m_path_set_bustransit.size() != m_path_vec_bustransit.size() ||
  // m_pathID_set_bustransit.size() != m_path_vec_bustransit.size()) {
  //     printf("repeated bustransit paths\n");
  //     exit(-1);
  // }
  // if (m_path_set_pnr.size() != m_path_vec_pnr.size() ||
  // m_pathID_set_pnr.size() != m_path_vec_pnr.size()) {
  //     printf("repeated pnr paths\n");
  //     exit(-1);
  // }
  // if (m_path_set_bus.size() != m_path_vec_bus.size() ||
  // m_pathID_set_bus.size() != m_path_vec_bus.size()) {
  //     printf("repeated bus routes\n");
  //     exit(-1);
  // }
  if (m_path_set.size () != m_path_vec.size ()
      || int (m_path_vec.size ())
           != m_num_path_driving + m_num_path_bustransit + m_num_path_pnr
                + m_num_path_bus)
    {
      throw std::runtime_error ("repeated paths");
    }
  return 0;
}

int
Mmdta::register_paths_driving (py::array_t<int> paths)
{
  if (m_path_vec_driving.size () > 0)
    {
      printf ("Warning, Mmdta::register_paths_driving, path exists\n");
      m_path_vec_driving.clear ();
      m_path_set_driving.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Mmdta::register_paths_driving: Number of dimensions must be one");
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
          throw std::runtime_error (
            "Mmdta::register_paths_driving: No such path");
        }
      else
        {
          if (m_ID_path_mapping[_path_ID].first->m_path_type != driving)
            {
              throw std::runtime_error (
                "Mmdta::register_paths_driving: Wrong path type");
            }
          m_path_vec_driving.push_back (m_ID_path_mapping[_path_ID].first);
          m_pathID_set_driving.insert (_path_ID);
        }
    }
  m_path_set_driving = std::set<MNM_Path *> (m_path_vec_driving.begin (),
                                             m_path_vec_driving.end ());
  if (m_path_set_driving.size () != m_path_vec_driving.size ()
      || m_pathID_set_driving.size () != m_path_vec_driving.size ()
      || int (m_path_vec_driving.size ()) != m_num_path_driving)
    {
      throw std::runtime_error ("repeated driving paths");
    }
  return 0;
}

int
Mmdta::register_paths_bustransit (py::array_t<int> paths)
{
  if (m_path_vec_bustransit.size () > 0)
    {
      printf ("Warning, Mmdta::register_paths_bustransit, path exists\n");
      m_path_vec_bustransit.clear ();
      m_path_set_bustransit.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error ("Mmdta::register_paths_bustransit: Number "
                                "of dimensions must be one");
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
          throw std::runtime_error (
            "Mmdta::register_paths_bustransit: No such path");
        }
      else
        {
          if (m_ID_path_mapping[_path_ID].first->m_path_type != transit)
            {
              throw std::runtime_error (
                "Mmdta::register_paths_bustransit: Wrong path type");
            }
          m_path_vec_bustransit.push_back (m_ID_path_mapping[_path_ID].first);
          m_pathID_set_bustransit.insert (_path_ID);
        }
    }
  m_path_set_bustransit = std::set<MNM_Path *> (m_path_vec_bustransit.begin (),
                                                m_path_vec_bustransit.end ());
  if (m_path_set_bustransit.size () != m_path_vec_bustransit.size ()
      || m_pathID_set_bustransit.size () != m_path_vec_bustransit.size ()
      || int (m_path_vec_bustransit.size ()) != m_num_path_bustransit)
    {
      throw std::runtime_error ("repeated bus transit paths");
    }
  return 0;
}

int
Mmdta::register_paths_pnr (py::array_t<int> paths)
{
  if (m_path_vec_pnr.size () > 0)
    {
      printf ("Warning, Mmdta::register_paths_pnr, path exists\n");
      m_path_vec_pnr.clear ();
      m_path_set_pnr.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Mmdta::register_paths_pnr: Number of dimensions must be one");
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
          throw std::runtime_error ("Mmdta::register_paths_pnr: No such path");
        }
      else
        {
          if (m_ID_path_mapping[_path_ID].first->m_path_type != pnr)
            {
              throw std::runtime_error (
                "Mmdta::register_paths_pnr: Wrong path type");
            }
          m_path_vec_pnr.push_back (m_ID_path_mapping[_path_ID].first);
          m_pathID_set_pnr.insert (_path_ID);
        }
    }
  m_path_set_pnr
    = std::set<MNM_Path *> (m_path_vec_pnr.begin (), m_path_vec_pnr.end ());
  if (m_path_set_pnr.size () != m_path_vec_pnr.size ()
      || m_pathID_set_pnr.size () != m_path_vec_pnr.size ()
      || int (m_path_vec_pnr.size ()) != m_num_path_pnr)
    {
      throw std::runtime_error ("repeated pnr paths");
    }
  return 0;
}

int
Mmdta::register_paths_bus (py::array_t<int> paths)
{
  if (m_path_vec_bus.size () > 0)
    {
      printf ("Warning, Mmdta::register_paths_bus, path exists\n");
      m_path_vec_bus.clear ();
      m_path_set_bus.clear ();
    }
  auto paths_buf = paths.request ();
  if (paths_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Mmdta::register_paths_bus: Number of dimensions must be one");
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
          throw std::runtime_error ("Mmdta::register_paths_bus: No such path");
        }
      else
        {
          if (m_ID_path_mapping[_path_ID].first->m_path_type != bus_route)
            {
              throw std::runtime_error (
                "Mmdta::register_paths_bus: Wrong path type");
            }
          m_path_vec_bus.push_back (m_ID_path_mapping[_path_ID].first);
          m_pathID_set_bus.insert (_path_ID);
        }
    }
  m_path_set_bus
    = std::set<MNM_Path *> (m_path_vec_bus.begin (), m_path_vec_bus.end ());
  if (m_path_set_bus.size () != m_path_vec_bus.size ()
      || m_pathID_set_bus.size () != m_path_vec_bus.size ()
      || int (m_path_vec_bus.size ()) != m_num_path_bus)
    {
      throw std::runtime_error ("repeated bus paths");
    }
  return 0;
}

std::vector<bool>
Mmdta::check_registered_links_in_registered_paths_driving ()
{
  // only check for m_path_vec_driving because truck exists
  std::vector<bool> _link_existing = std::vector<bool> ();
  if (m_link_vec_driving.empty ())
    {
      printf ("Warning, "
              "Mmdta::check_registered_links_in_registered_paths_driving, "
              "no link registered\n");
      return _link_existing;
    }
  for (size_t k = 0; k < m_link_vec_driving.size (); ++k)
    {
      _link_existing.push_back (false);
    }
  // if (m_path_vec_driving.empty() && m_path_vec_pnr.empty()){
  if (m_path_vec_driving.empty ())
    {
      printf ("Warning, "
              "Mmdta::check_registered_links_in_registered_paths_driving, "
              "no path registered\n");
      return _link_existing;
    }
  for (auto *_path : m_path_vec_driving)
    {
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          if (!_link_existing[i])
            {
              _link_existing[i]
                = _path->is_link_in (m_link_vec_driving[i]->m_link_ID);
            }
        }
      if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                       [] (bool v) { return v; }))
        {
          break;
        }
    }
  // if (std::any_of(_link_existing.cbegin(), _link_existing.cend(), [](bool
  // v){return !v;})) {
  //     for (auto* _path : m_path_vec_pnr) {
  //         for (size_t i = 0; i < m_link_vec_driving.size(); ++i) {
  //             if (!_link_existing[i]) {
  //                 _link_existing[i] = _path ->
  //                 is_link_in(m_link_vec_driving[i] -> m_link_ID);
  //             }
  //         }
  //         if (std::all_of(_link_existing.cbegin(), _link_existing.cend(),
  //         [](bool v){return v;})) {
  //             break;
  //         }
  //     }
  // }
  if (std::any_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return !v; }))
    {
      // printf("Warning: some observed driving links in m_link_vec_driving are
      // not covered by generated paths in m_path_vec_driving and
      // m_path_vec_pnr!\n");
      printf ("Warning: some observed driving links in m_link_vec_driving are "
              "not covered by generated paths in m_path_vec_driving!\n");
    }
  return _link_existing;
}

std::vector<bool>
Mmdta::check_registered_links_in_registered_paths_bus ()
{
  std::vector<bool> _link_existing = std::vector<bool> ();
  if (m_link_vec_bus.empty ())
    {
      printf ("Warning, Mmdta::check_registered_links_in_registered_paths_bus, "
              "no link registered\n");
      return _link_existing;
    }
  for (size_t k = 0; k < m_link_vec_bus.size (); ++k)
    {
      _link_existing.push_back (false);
    }
  if (m_path_vec_bustransit.empty () && m_path_vec_pnr.empty ())
    {
      printf ("Warning, Mmdta::check_registered_links_in_registered_paths_bus, "
              "no path registered\n");
      return _link_existing;
    }
  for (auto *_path : m_path_vec_bustransit)
    {
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          if (!_link_existing[i])
            {
              _link_existing[i]
                = _path->is_link_in (m_link_vec_bus[i]->m_link_ID);
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
      for (auto *_path : m_path_vec_pnr)
        {
          for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
            {
              if (!_link_existing[i])
                {
                  _link_existing[i]
                    = _path->is_link_in (m_link_vec_bus[i]->m_link_ID);
                }
            }
          if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                           [] (bool v) { return v; }))
            {
              break;
            }
        }
    }
  if (std::any_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return !v; }))
    {
      printf (
        "Warning: some observed bus links in m_link_vec_bus are not covered by "
        "generated paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    }
  return _link_existing;
}

std::vector<bool>
Mmdta::check_registered_links_in_registered_paths_walking ()
{
  std::vector<bool> _link_existing = std::vector<bool> ();
  if (m_link_vec_walking.empty ())
    {
      printf ("Warning, "
              "Mmdta::check_registered_links_in_registered_paths_walking, "
              "no link registered\n");
      return _link_existing;
    }
  for (size_t k = 0; k < m_link_vec_walking.size (); ++k)
    {
      _link_existing.push_back (false);
    }
  if (m_path_vec_bustransit.empty () && m_path_vec_pnr.empty ())
    {
      printf ("Warning, "
              "Mmdta::check_registered_links_in_registered_paths_walking, "
              "no path registered\n");
      return _link_existing;
    }
  for (auto *_path : m_path_vec_bustransit)
    {
      for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
        {
          if (!_link_existing[i])
            {
              _link_existing[i]
                = _path->is_link_in (m_link_vec_walking[i]->m_link_ID);
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
      for (auto *_path : m_path_vec_pnr)
        {
          for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
            {
              if (!_link_existing[i])
                {
                  _link_existing[i]
                    = _path->is_link_in (m_link_vec_walking[i]->m_link_ID);
                }
            }
          if (std::all_of (_link_existing.cbegin (), _link_existing.cend (),
                           [] (bool v) { return v; }))
            {
              break;
            }
        }
    }
  if (std::any_of (_link_existing.cbegin (), _link_existing.cend (),
                   [] (bool v) { return !v; }))
    {
      printf ("Warning: some observed walking links in m_link_vec_walking are "
              "not covered by generated paths in m_path_vec_bustransit and "
              "m_path_vec_pnr!\n");
    }
  return _link_existing;
}

py::array_t<bool>
Mmdta::are_registered_links_in_registered_paths_driving ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing
    = check_registered_links_in_registered_paths_driving ();

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

py::array_t<bool>
Mmdta::are_registered_links_in_registered_paths_bus ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing
    = check_registered_links_in_registered_paths_bus ();

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

py::array_t<bool>
Mmdta::are_registered_links_in_registered_paths_walking ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing
    = check_registered_links_in_registered_paths_walking ();

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
Mmdta::generate_paths_to_cover_registered_links_driving ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing_driving
    = check_registered_links_in_registered_paths_driving ();

  int new_shape[1] = { 1 + (int) _link_existing_driving.size () };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  if (std::all_of (_link_existing_driving.cbegin (),
                   _link_existing_driving.cend (), [] (bool v) { return v; }))
    {
      printf ("driving_path_table NOT updated! All links in m_link_vec_driving "
              "are covered by paths in m_path_vec_driving!\n");

      result_ptr[0] = 0; // indicate whether the path table is updated
      for (size_t v = 0; v < _link_existing_driving.size (); ++v)
        {
          result_ptr[v + 1] = (int) _link_existing_driving[v];
        }
      return result;
    }

  PNEGraph reversed_graph = MNM_Ults::reverse_graph (m_mmdta->m_graph);
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  for (auto _link_it : m_mmdta->m_link_factory->m_link_map)
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
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  MNM_Path *_path_1, *_path_2, *_path;
  std::vector<std::pair<TInt, MNM_Origin *>> pair_ptrs_1
    = std::vector<std::pair<TInt, MNM_Origin *>> ();
  std::vector<std::pair<MNM_Destination *, TFlt *>> pair_ptrs_2
    = std::vector<std::pair<MNM_Destination *, TFlt *>> ();

  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      if (!_link_existing_driving[i])
        {
          // generate new path including this link
          _from_node_ID
            = m_mmdta->m_graph->GetEI (m_link_vec_driving[i]->m_link_ID)
                .GetSrcNId ();
          _to_node_ID
            = m_mmdta->m_graph->GetEI (m_link_vec_driving[i]->m_link_ID)
                .GetDstNId ();

          // path from origin to from_node_ID
          if (!_shortest_path_tree.empty ())
            {
              _shortest_path_tree.clear ();
            }
          if (dynamic_cast<MNM_DMOND_Multiclass *> (
                m_mmdta->m_node_factory->get_node (_from_node_ID))
              != nullptr)
            {
              _path_1 = new MNM_Path ();
              _path_1->m_node_vec.push_back (_from_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_from_node_ID,
                                                  m_mmdta->m_graph, _cost_map,
                                                  _shortest_path_tree);
            }

          // path from to_node_ID to destination
          if (!_shortest_path_tree_reversed.empty ())
            {
              _shortest_path_tree_reversed.clear ();
            }
          if (dynamic_cast<MNM_DMDND_Multiclass *> (
                m_mmdta->m_node_factory->get_node (_to_node_ID))
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
          for (const auto &p : m_mmdta->m_od_factory->m_origin_map)
            {
              pair_ptrs_1.emplace_back (p);
            }
          std::random_shuffle (std::begin (pair_ptrs_1),
                               std::end (pair_ptrs_1));
          for (auto _it : pair_ptrs_1)
            {
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (_it.second);
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
                    = dynamic_cast<MNM_Destination_Multimodal *> (_it_it.first);
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
              printf ("Cannot generate driving path covering this link\n");
              // probably this is for pnr mode
              // exit(-1);
              continue;
            }
          IAssert (_origin != nullptr && _dest != nullptr);

          if (!_shortest_path_tree.empty ())
            {
              _path_1 = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                           _from_node_ID, _shortest_path_tree,
                                           m_mmdta->m_graph);
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
          _path->m_link_vec.push_back (m_link_vec_driving[i]->m_link_ID);
          _path->m_link_vec.insert (_path->m_link_vec.end (),
                                    _path_2->m_link_vec.rbegin (),
                                    _path_2->m_link_vec.rend ());
          _path->m_node_vec = _path_1->m_node_vec;
          _path->m_node_vec.insert (_path->m_node_vec.end (),
                                    _path_2->m_node_vec.rbegin (),
                                    _path_2->m_node_vec.rend ());
          _path->allocate_buffer (
            2 * m_mmdta->m_config->get_int ("max_interval"));
          delete _path_1;
          delete _path_2;
          // add this new path to path table, not the passenger_path_table
          dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
            ->m_routing_fixed_car->m_path_table
            ->find (_origin->m_origin_node->m_node_ID)
            ->second->find (_dest->m_dest_node->m_node_ID)
            ->second->m_path_vec.push_back (_path);
          m_path_vec_driving.push_back (_path);
          _link_existing_driving[i] = true;

          // check if this new path cover other links
          for (size_t j = 0; j < m_link_vec_driving.size (); ++j)
            {
              if (!_link_existing_driving[j])
                {
                  _link_existing_driving[j]
                    = _path->is_link_in (m_link_vec_driving[j]->m_link_ID);
                }
            }
          if (std::all_of (_link_existing_driving.cbegin (),
                           _link_existing_driving.cend (),
                           [] (bool v) { return v; }))
            {
              printf ("All links in m_link_vec_driving are covered by paths in "
                      "m_path_vec_driving!\n");
              break;
            }
        }
    }

  if (std::all_of (_link_existing_driving.cbegin (),
                   _link_existing_driving.cend (), [] (bool v) { return v; }))
    {
      printf ("driving_path_table updated! All links in m_link_vec_driving are "
              "covered by paths in m_path_vec_driving!\n");
    }
  else
    {
      printf ("Mmdta::generate_paths_to_cover_registered_links_driving, "
              "NOT all links in m_link_vec_driving are covered by paths in "
              "m_path_vec_driving!\n");
      // exit(-1);
    }

  MNM::save_driving_path_table (m_mmdta->m_file_folder,
                                dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (
                                  m_mmdta->m_routing)
                                  ->m_routing_fixed_car->m_path_table,
                                "driving_path_table",
                                "driving_path_table_buffer", true);

  result_ptr[0] = 1; // indicate whether the path table is updated
  for (size_t v = 0; v < _link_existing_driving.size (); ++v)
    {
      result_ptr[v + 1] = (int) _link_existing_driving[v];
    }

  _link_existing_driving.clear ();
  _cost_map.clear ();
  _shortest_path_tree.clear ();
  _shortest_path_tree_reversed.clear ();
  reversed_graph.Clr ();
  pair_ptrs_1.clear ();
  pair_ptrs_2.clear ();
  return result;
}

py::array_t<int>
Mmdta::generate_paths_to_cover_registered_links_bus_walking ()
{
  // used in Python API, check the input files before the DODE
  std::vector<bool> _link_existing_bus
    = check_registered_links_in_registered_paths_bus ();
  std::vector<bool> _link_existing_walking
    = check_registered_links_in_registered_paths_walking ();

  int new_shape[1] = { 1 + (int) _link_existing_bus.size ()
                       + (int) _link_existing_walking.size () };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  if (std::all_of (_link_existing_bus.cbegin (), _link_existing_bus.cend (),
                   [] (bool v) { return v; })
      && std::all_of (_link_existing_walking.cbegin (),
                      _link_existing_walking.cend (),
                      [] (bool v) { return v; }))
    {
      printf ("bustransit_path_table and pnr_path_table NOT updated! All links "
              "in m_link_vec_bus and m_link_vec_walking are covered by paths "
              "in m_path_vec_bustransit and m_path_vec_pnr!\n");

      result_ptr[0] = 0; // indicate whether the path table is updated
      for (size_t v = 0; v < _link_existing_bus.size (); ++v)
        {
          result_ptr[v + 1] = (int) _link_existing_bus[v];
        }
      for (size_t v = _link_existing_bus.size ();
           v < _link_existing_bus.size () + _link_existing_walking.size (); ++v)
        {
          result_ptr[v + 1]
            = (int) _link_existing_walking[v - _link_existing_bus.size ()];
        }
      return result;
    }

  PNEGraph reversed_graph
    = MNM_Ults::reverse_graph (m_mmdta->m_bus_transit_graph);
  std::unordered_map<TInt, TFlt> _cost_map_driving
    = std::unordered_map<TInt, TFlt> ();
  for (auto _link_it : m_mmdta->m_link_factory->m_link_map)
    {
      _cost_map_driving.insert (
        std::pair<TInt, TFlt> (_link_it.first,
                               dynamic_cast<MNM_Dlink_Multiclass *> (
                                 _link_it.second)
                                 ->get_link_freeflow_tt_car ()));
    }
  std::unordered_map<TInt, TFlt> _cost_map = std::unordered_map<TInt, TFlt> ();
  for (auto _link_it : m_mmdta->m_transitlink_factory->m_transit_link_map)
    {
      _cost_map.insert (
        std::pair<TInt, TFlt> (_link_it.first, _link_it.second->m_fftt));
    }
  std::unordered_map<TInt, TInt> _shortest_path_tree
    = std::unordered_map<TInt, TInt> ();
  std::unordered_map<TInt, TInt> _shortest_path_tree_reversed
    = std::unordered_map<TInt, TInt> ();
  TInt _from_node_ID, _to_node_ID;
  MNM_Origin_Multimodal *_origin;
  MNM_Destination_Multimodal *_dest;
  MNM_Path *_path_1, *_path_2, *_path;
  MNM_PnR_Path *_pnr_path;
  TInt _mid_parking_lot_ID = -1;
  TInt _mid_dest_node_ID = -1;
  bool _is_bustransit, _is_pnr;
  std::vector<std::pair<TInt, MNM_Origin *>> pair_ptrs_1
    = std::vector<std::pair<TInt, MNM_Origin *>> ();
  std::vector<std::pair<MNM_Destination *, TFlt *>> pair_ptrs_2
    = std::vector<std::pair<MNM_Destination *, TFlt *>> ();

  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      if (!_link_existing_bus[i])
        {
          _is_bustransit = false;
          _is_pnr = false;

          // generate new path including this link
          _from_node_ID
            = m_mmdta->m_bus_transit_graph->GetEI (m_link_vec_bus[i]->m_link_ID)
                .GetSrcNId ();
          _to_node_ID
            = m_mmdta->m_bus_transit_graph->GetEI (m_link_vec_bus[i]->m_link_ID)
                .GetDstNId ();

          // path from origin to from_node_ID
          if (!_shortest_path_tree.empty ())
            {
              _shortest_path_tree.clear ();
            }
          if (m_mmdta->m_bus_transit_graph->GetNI (_from_node_ID).GetInDeg ()
              == 0)
            {
              _path_1 = new MNM_Path ();
              _path_1->m_node_vec.push_back (_from_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_from_node_ID,
                                                  m_mmdta->m_bus_transit_graph,
                                                  _cost_map,
                                                  _shortest_path_tree);
            }

          // path from to_node_ID to destination
          if (!_shortest_path_tree_reversed.empty ())
            {
              _shortest_path_tree_reversed.clear ();
            }
          if (m_mmdta->m_bus_transit_graph->GetNI (_from_node_ID).GetOutDeg ()
              == 0)
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
          for (const auto &p : m_mmdta->m_od_factory->m_origin_map)
            {
              pair_ptrs_1.emplace_back (p);
            }
          std::random_shuffle (std::begin (pair_ptrs_1),
                               std::end (pair_ptrs_1));
          for (auto _it : pair_ptrs_1)
            {
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (_it.second);
              if (_origin->m_demand_passenger_bus.empty ())
                {
                  continue;
                }

              if (!pair_ptrs_2.empty ())
                {
                  pair_ptrs_2.clear ();
                }
              for (const auto &p : _origin->m_demand_passenger_bus)
                {
                  pair_ptrs_2.emplace_back (p);
                }
              std::random_shuffle (std::begin (pair_ptrs_2),
                                   std::end (pair_ptrs_2));
              for (auto _it_it : pair_ptrs_2)
                {
                  _dest
                    = dynamic_cast<MNM_Destination_Multimodal *> (_it_it.first);
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
                      _is_bustransit = true;
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
              // PnR path
              IAssert (!pair_ptrs_1.empty ());
              for (auto _it : pair_ptrs_1)
                {
                  _origin = dynamic_cast<MNM_Origin_Multimodal *> (_it.second);
                  if (_origin->m_demand_pnr_car.empty ())
                    {
                      continue;
                    }

                  if (!pair_ptrs_2.empty ())
                    {
                      pair_ptrs_2.clear ();
                    }
                  for (const auto &p : _origin->m_demand_pnr_car)
                    {
                      pair_ptrs_2.emplace_back (p);
                    }
                  std::random_shuffle (std::begin (pair_ptrs_2),
                                       std::end (pair_ptrs_2));
                  for (auto _it_it : pair_ptrs_2)
                    {
                      _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                        _it_it.first);
                      if (!_dest->m_connected_pnr_parkinglot_vec.empty ())
                        {
                          for (auto *_parking_lot :
                               _dest->m_connected_pnr_parkinglot_vec)
                            {
                              if (_shortest_path_tree
                                      .find (_parking_lot->m_node_ID)
                                      ->second
                                    != -1
                                  && _shortest_path_tree_reversed
                                         .find (_dest->m_dest_node->m_node_ID)
                                         ->second
                                       != -1)
                                {
                                  _mid_dest_node_ID = _parking_lot->m_node_ID;
                                  _mid_parking_lot_ID = _parking_lot->m_ID;
                                  _flg = true;
                                  _is_pnr = true;
                                  break;
                                }
                            }
                        }
                      if (_flg)
                        {
                          break;
                        }
                    }
                  if (_flg)
                    {
                      break;
                    }
                }
            }

          if (!_flg)
            {
              printf (
                "Cannot generate bustransit or PnR path covering this link\n");
              // exit(-1);
              continue;
            }
          IAssert (_origin != nullptr && _dest != nullptr);
          if (_is_pnr)
            {
              IAssert (_mid_dest_node_ID > -1);
            }

          if (!_shortest_path_tree.empty ())
            {
              if (_is_bustransit)
                {
                  _path_1
                    = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                         _from_node_ID, _shortest_path_tree,
                                         m_mmdta->m_bus_transit_graph);
                }
              else if (_is_pnr)
                {
                  _path_1 = MNM::extract_path (_mid_dest_node_ID, _from_node_ID,
                                               _shortest_path_tree,
                                               m_mmdta->m_bus_transit_graph);
                }
              else
                {
                  printf ("Cannot generate bustransit or PnR path covering "
                          "this link\n");
                  // exit(-1);
                  continue;
                }
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
          _path->m_link_vec.push_back (m_link_vec_bus[i]->m_link_ID);
          _path->m_link_vec.insert (_path->m_link_vec.end (),
                                    _path_2->m_link_vec.rbegin (),
                                    _path_2->m_link_vec.rend ());
          _path->m_node_vec = _path_1->m_node_vec;
          _path->m_node_vec.insert (_path->m_node_vec.end (),
                                    _path_2->m_node_vec.rbegin (),
                                    _path_2->m_node_vec.rend ());
          delete _path_1;
          delete _path_2;

          // add this new path to path table, not the passenger_path_table
          if (_is_bustransit)
            {
              _path->allocate_buffer (
                m_mmdta->m_config->get_int ("max_interval"));
              dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
                ->m_routing_passenger_fixed->m_bustransit_path_table
                ->find (_origin->m_origin_node->m_node_ID)
                ->second->find (_dest->m_dest_node->m_node_ID)
                ->second->m_path_vec.push_back (_path);
              m_path_vec_bustransit.push_back (_path);
            }
          else if (_is_pnr)
            {
              _shortest_path_tree.clear ();
              MNM_Shortest_Path::all_to_one_FIFO (_mid_dest_node_ID,
                                                  m_mmdta->m_graph,
                                                  _cost_map_driving,
                                                  _shortest_path_tree);
              IAssert (
                _shortest_path_tree.find (_origin->m_origin_node->m_node_ID)
                  ->second
                != -1);
              _path_1
                = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                     _mid_dest_node_ID, _shortest_path_tree,
                                     m_mmdta->m_graph);
              _pnr_path = new MNM_PnR_Path ((int) m_path_vec_pnr.size (),
                                            _mid_parking_lot_ID,
                                            _mid_dest_node_ID, _path_1, _path);
              _pnr_path->allocate_buffer (
                m_mmdta->m_config->get_int ("max_interval"));
              delete _path_1;
              delete _path;
              dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
                ->m_routing_car_pnr_fixed->m_pnr_path_table
                ->find (_origin->m_origin_node->m_node_ID)
                ->second->find (_dest->m_dest_node->m_node_ID)
                ->second->m_path_vec.push_back (_pnr_path);
              m_path_vec_pnr.push_back (_pnr_path);
            }
          else
            {
              printf (
                "Cannot generate bustransit or PnR path covering this link\n");
              // exit(-1);
              continue;
            }
          _link_existing_bus[i] = true;

          // check if this new path cover other links
          for (size_t j = 0; j < m_link_vec_bus.size (); ++j)
            {
              if (!_link_existing_bus[j])
                {
                  if (_is_bustransit)
                    {
                      _link_existing_bus[j]
                        = _path->is_link_in (m_link_vec_bus[j]->m_link_ID);
                    }
                  else if (_is_pnr)
                    {
                      _link_existing_bus[j]
                        = _pnr_path->is_link_in (m_link_vec_bus[j]->m_link_ID);
                    }
                }
            }
          for (size_t j = 0; j < m_link_vec_walking.size (); ++j)
            {
              if (!_link_existing_walking[j])
                {
                  if (_is_bustransit)
                    {
                      _link_existing_walking[j]
                        = _path->is_link_in (m_link_vec_walking[j]->m_link_ID);
                    }
                  else if (_is_pnr)
                    {
                      _link_existing_walking[j] = _pnr_path->is_link_in (
                        m_link_vec_walking[j]->m_link_ID);
                    }
                }
            }
          if (std::all_of (_link_existing_bus.cbegin (),
                           _link_existing_bus.cend (),
                           [] (bool v) { return v; }))
            {
              printf ("All links in m_link_vec_bus are covered by paths in "
                      "m_path_vec_bustransit and m_path_vec_pnr!\n");
              break;
            }
        }
    }

  if (std::all_of (_link_existing_bus.cbegin (), _link_existing_bus.cend (),
                   [] (bool v) { return v; }))
    {
      printf (
        "bustransit_path_table updated! All links in m_link_vec_bus are "
        "covered by paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
    }
  else
    {
      printf ("Mmdta::generate_paths_to_cover_registered_links_bus_walking,"
              " NOT all links in m_link_vec_bus are covered by paths in "
              "m_path_vec_bustransit and m_path_vec_pnr!\n");
      // exit(-1);
    }

  for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
    {
      if (!_link_existing_walking[i])
        {
          _is_bustransit = false;
          _is_pnr = false;

          // generate new path including this link
          _from_node_ID = m_mmdta->m_bus_transit_graph
                            ->GetEI (m_link_vec_walking[i]->m_link_ID)
                            .GetSrcNId ();
          _to_node_ID = m_mmdta->m_bus_transit_graph
                          ->GetEI (m_link_vec_walking[i]->m_link_ID)
                          .GetDstNId ();

          // path from origin to from_node_ID
          if (!_shortest_path_tree.empty ())
            {
              _shortest_path_tree.clear ();
            }
          if (m_mmdta->m_bus_transit_graph->GetNI (_from_node_ID).GetInDeg ()
              == 0)
            {
              _path_1 = new MNM_Path ();
              _path_1->m_node_vec.push_back (_from_node_ID);
            }
          else
            {
              MNM_Shortest_Path::all_to_one_FIFO (_from_node_ID,
                                                  m_mmdta->m_bus_transit_graph,
                                                  _cost_map,
                                                  _shortest_path_tree);
            }

          // path from to_node_ID to destination
          if (!_shortest_path_tree_reversed.empty ())
            {
              _shortest_path_tree_reversed.clear ();
            }
          if (m_mmdta->m_bus_transit_graph->GetNI (_from_node_ID).GetOutDeg ()
              == 0)
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
          for (const auto &p : m_mmdta->m_od_factory->m_origin_map)
            {
              pair_ptrs_1.emplace_back (p);
            }
          std::random_shuffle (std::begin (pair_ptrs_1),
                               std::end (pair_ptrs_1));
          for (auto _it : pair_ptrs_1)
            {
              _origin = dynamic_cast<MNM_Origin_Multimodal *> (_it.second);
              if (_origin->m_demand_passenger_bus.empty ())
                {
                  continue;
                }

              if (!pair_ptrs_2.empty ())
                {
                  pair_ptrs_2.clear ();
                }
              for (const auto &p : _origin->m_demand_passenger_bus)
                {
                  pair_ptrs_2.emplace_back (p);
                }
              std::random_shuffle (std::begin (pair_ptrs_2),
                                   std::end (pair_ptrs_2));
              for (auto _it_it : pair_ptrs_2)
                {
                  _dest
                    = dynamic_cast<MNM_Destination_Multimodal *> (_it_it.first);
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
                      _is_bustransit = true;
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
              // PnR path
              IAssert (!pair_ptrs_1.empty ());
              for (auto _it : pair_ptrs_1)
                {
                  _origin = dynamic_cast<MNM_Origin_Multimodal *> (_it.second);
                  if (_origin->m_demand_pnr_car.empty ())
                    {
                      continue;
                    }

                  if (!pair_ptrs_2.empty ())
                    {
                      pair_ptrs_2.clear ();
                    }
                  for (const auto &p : _origin->m_demand_pnr_car)
                    {
                      pair_ptrs_2.emplace_back (p);
                    }
                  std::random_shuffle (std::begin (pair_ptrs_2),
                                       std::end (pair_ptrs_2));
                  for (auto _it_it : pair_ptrs_2)
                    {
                      _dest = dynamic_cast<MNM_Destination_Multimodal *> (
                        _it_it.first);
                      if (!_dest->m_connected_pnr_parkinglot_vec.empty ())
                        {
                          for (auto *_parking_lot :
                               _dest->m_connected_pnr_parkinglot_vec)
                            {
                              if (_shortest_path_tree
                                      .find (_parking_lot->m_node_ID)
                                      ->second
                                    != -1
                                  && _shortest_path_tree_reversed
                                         .find (_dest->m_dest_node->m_node_ID)
                                         ->second
                                       != -1)
                                {
                                  _mid_dest_node_ID = _parking_lot->m_node_ID;
                                  _mid_parking_lot_ID = _parking_lot->m_ID;
                                  _flg = true;
                                  _is_pnr = true;
                                  break;
                                }
                            }
                        }
                      if (_flg)
                        {
                          break;
                        }
                    }
                  if (_flg)
                    {
                      break;
                    }
                }
            }

          if (!_flg)
            {
              printf (
                "Cannot generate bustransit or PnR path covering this link\n");
              // exit(-1);
              continue;
            }
          IAssert (_origin != nullptr && _dest != nullptr);
          if (_is_pnr)
            {
              IAssert (_mid_dest_node_ID > -1);
            }

          if (!_shortest_path_tree.empty ())
            {
              if (_is_bustransit)
                {
                  _path_1
                    = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                         _from_node_ID, _shortest_path_tree,
                                         m_mmdta->m_bus_transit_graph);
                }
              else if (_is_pnr)
                {
                  _path_1 = MNM::extract_path (_mid_dest_node_ID, _from_node_ID,
                                               _shortest_path_tree,
                                               m_mmdta->m_bus_transit_graph);
                }
              else
                {
                  printf ("Cannot generate bustransit or PnR path covering "
                          "this link\n");
                  // exit(-1);
                  continue;
                }
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
          _path->m_link_vec.push_back (m_link_vec_walking[i]->m_link_ID);
          _path->m_link_vec.insert (_path->m_link_vec.end (),
                                    _path_2->m_link_vec.rbegin (),
                                    _path_2->m_link_vec.rend ());
          _path->m_node_vec = _path_1->m_node_vec;
          _path->m_node_vec.insert (_path->m_node_vec.end (),
                                    _path_2->m_node_vec.rbegin (),
                                    _path_2->m_node_vec.rend ());
          delete _path_1;
          delete _path_2;

          // add this new path to path table, not the passenger_path_table
          if (_is_bustransit)
            {
              _path->allocate_buffer (
                m_mmdta->m_config->get_int ("max_interval"));
              dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
                ->m_routing_passenger_fixed->m_bustransit_path_table
                ->find (_origin->m_origin_node->m_node_ID)
                ->second->find (_dest->m_dest_node->m_node_ID)
                ->second->m_path_vec.push_back (_path);
              m_path_vec_bustransit.push_back (_path);
            }
          else if (_is_pnr)
            {
              _shortest_path_tree.clear ();
              MNM_Shortest_Path::all_to_one_FIFO (_mid_dest_node_ID,
                                                  m_mmdta->m_graph,
                                                  _cost_map_driving,
                                                  _shortest_path_tree);
              IAssert (
                _shortest_path_tree.find (_origin->m_origin_node->m_node_ID)
                  ->second
                != -1);
              _path_1
                = MNM::extract_path (_origin->m_origin_node->m_node_ID,
                                     _mid_dest_node_ID, _shortest_path_tree,
                                     m_mmdta->m_graph);
              _pnr_path = new MNM_PnR_Path ((int) m_path_vec_pnr.size (),
                                            _mid_parking_lot_ID,
                                            _mid_dest_node_ID, _path_1, _path);
              _pnr_path->allocate_buffer (
                m_mmdta->m_config->get_int ("max_interval"));
              delete _path_1;
              delete _path;
              dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (m_mmdta->m_routing)
                ->m_routing_car_pnr_fixed->m_pnr_path_table
                ->find (_origin->m_origin_node->m_node_ID)
                ->second->find (_dest->m_dest_node->m_node_ID)
                ->second->m_path_vec.push_back (_pnr_path);
              m_path_vec_pnr.push_back (_pnr_path);
            }
          else
            {
              printf (
                "Cannot generate bustransit or PnR path covering this link\n");
              // exit(-1);
              continue;
            }
          _link_existing_walking[i] = true;

          // check if this new path cover other links
          for (size_t j = 0; j < m_link_vec_walking.size (); ++j)
            {
              if (!_link_existing_walking[j])
                {
                  if (_is_bustransit)
                    {
                      _link_existing_walking[j]
                        = _path->is_link_in (m_link_vec_walking[j]->m_link_ID);
                    }
                  else if (_is_pnr)
                    {
                      _link_existing_walking[j] = _pnr_path->is_link_in (
                        m_link_vec_walking[j]->m_link_ID);
                    }
                }
            }
          if (std::all_of (_link_existing_walking.cbegin (),
                           _link_existing_walking.cend (),
                           [] (bool v) { return v; }))
            {
              printf ("All links in m_link_vec_walking are covered by paths in "
                      "m_path_vec_bustransit and m_path_vec_pnr!\n");
              break;
            }
        }
    }

  if (std::all_of (_link_existing_bus.cbegin (), _link_existing_bus.cend (),
                   [] (bool v) { return v; })
      && std::all_of (_link_existing_walking.cbegin (),
                      _link_existing_walking.cend (),
                      [] (bool v) { return v; }))
    {
      printf ("bustransit_path_table and/or pnr_path_table updated! All links "
              "in m_link_vec_bus and m_link_vec_walking are covered by paths "
              "in m_path_vec_bustransit and m_path_vec_pnr!\n");
    }
  else
    {
      printf (
        "Mmdta::generate_paths_to_cover_registered_links_bus_walking, NOT "
        "all links in m_link_vec_bus and m_link_vec_walking are covered by "
        "paths in m_path_vec_bustransit and m_path_vec_pnr!\n");
      // exit(-1);
    }

  MNM::save_bustransit_path_table (m_mmdta->m_file_folder,
                                   dynamic_cast<MNM_Routing_Multimodal_Hybrid
                                                  *> (m_mmdta->m_routing)
                                     ->m_routing_passenger_fixed
                                     ->m_bustransit_path_table,
                                   "bustransit_path_table",
                                   "bustransit_path_table_buffer", true);
  MNM::save_pnr_path_table (m_mmdta->m_file_folder,
                            dynamic_cast<MNM_Routing_Multimodal_Hybrid *> (
                              m_mmdta->m_routing)
                              ->m_routing_car_pnr_fixed->m_pnr_path_table,
                            "pnr_path_table", "pnr_path_table_buffer", true);

  result_ptr[0] = 1; // indicate whether the path table is updated
  for (size_t v = 0; v < _link_existing_bus.size (); ++v)
    {
      result_ptr[v + 1] = (int) _link_existing_bus[v];
    }
  for (size_t v = _link_existing_bus.size ();
       v < _link_existing_bus.size () + _link_existing_walking.size (); ++v)
    {
      result_ptr[v + 1]
        = (int) _link_existing_walking[v - _link_existing_bus.size ()];
    }

  _link_existing_bus.clear ();
  _link_existing_walking.clear ();
  _cost_map_driving.clear ();
  _cost_map.clear ();
  _shortest_path_tree.clear ();
  _shortest_path_tree_reversed.clear ();
  reversed_graph.Clr ();
  pair_ptrs_1.clear ();
  pair_ptrs_2.clear ();
  return result;
}

int
Mmdta::save_passenger_path_table (const std::string &file_folder)
{
  MNM::save_passenger_path_table (m_mmdue->m_passenger_path_table, file_folder,
                                  std::string ("passenger_path_table"),
                                  std::string ("passenger_path_table_buffer"),
                                  true, true);

  printf ("Mmdta::save_passenger_path_table, Finish saving passenger path "
          "table\n");
  return 0;
}

int
Mmdta::save_mode_path_table (const std::string &file_folder)
{
  m_mmdue->passenger_path_table_to_multimodal_path_table (m_mmdta);
  if (m_mmdue->m_driving_path_table == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::save_mode_path_table, null driving path table");
    }
  if (m_mmdue->m_bustransit_path_table == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::save_mode_path_table, null bustransit path table");
    }
  if (m_mmdue->m_pnr_path_table == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::save_mode_path_table, null pnr path table");
    }

  MNM::save_driving_path_table (file_folder, m_mmdue->m_driving_path_table,
                                "driving_path_table",
                                "driving_path_table_buffer", true);
  MNM::save_bustransit_path_table (file_folder,
                                   m_mmdue->m_bustransit_path_table,
                                   "bustransit_path_table",
                                   "bustransit_path_table_buffer", true);
  MNM::save_pnr_path_table (file_folder, m_mmdue->m_pnr_path_table,
                            "pnr_path_table", "pnr_path_table_buffer", true);
  return 0;
}

py::array_t<int>
Mmdta::link_seq_to_node_seq_driving (py::array_t<int> link_IDs)
{
  auto links_buf = link_IDs.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::link_seq_to_node_seq_driving, link_IDs input "
        "dimension mismatch");
    }
  int m = links_buf.shape[0];
  int *links_ptr = (int *) links_buf.ptr;

  int new_shape[1] = { m + 1 };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  result_ptr[0] = m_mmdta->m_graph->GetEI (links_ptr[0]).GetSrcNId ();
  for (int i = 0; i < m; ++i)
    {
      result_ptr[i + 1] = m_mmdta->m_graph->GetEI (links_ptr[i]).GetDstNId ();
    }

  return result;
}

py::array_t<int>
Mmdta::link_seq_to_node_seq_bustransit (py::array_t<int> link_IDs)
{
  auto links_buf = link_IDs.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::link_seq_to_node_seq_bustransit, link_IDs input "
        "dimension mismatch");
    }
  int m = links_buf.shape[0];
  int *links_ptr = (int *) links_buf.ptr;

  int new_shape[1] = { m + 1 };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  result_ptr[0]
    = m_mmdta->m_bus_transit_graph->GetEI (links_ptr[0]).GetSrcNId ();
  for (int i = 0; i < m; ++i)
    {
      result_ptr[i + 1]
        = m_mmdta->m_bus_transit_graph->GetEI (links_ptr[i]).GetDstNId ();
    }

  return result;
}

py::array_t<int>
Mmdta::node_seq_to_link_seq_driving (py::array_t<int> node_IDs)
{
  auto nodes_buf = node_IDs.request ();
  if (nodes_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::node_seq_to_link_seq_driving, node_IDs input "
        "dimension mismatch");
    }
  int m = nodes_buf.shape[0];
  if (m < 2)
    {
      throw std::runtime_error (
        "Error, Mmdta::node_seq_to_link_seq_driving, node_IDs length must "
        "not be less than 2");
    }
  int *nodes_ptr = (int *) nodes_buf.ptr;

  int new_shape[1] = { m - 1 };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  for (int i = 0; i < m - 1; ++i)
    {
      result_ptr[i] = m_mmdta->m_graph->GetEId (nodes_ptr[i], nodes_ptr[i + 1]);
      if (result_ptr[i] < 0)
        {
          throw std::runtime_error (
            "Error, Mmdta::node_seq_to_link_seq_driving, link does not "
            "exist");
        }
    }

  return result;
}

py::array_t<int>
Mmdta::node_seq_to_link_seq_bustransit (py::array_t<int> node_IDs)
{
  auto nodes_buf = node_IDs.request ();
  if (nodes_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::node_seq_to_link_seq_bustransit, node_IDs input "
        "dimension mismatch");
    }
  int m = nodes_buf.shape[0];
  if (m < 2)
    {
      throw std::runtime_error (
        "Error, Mmdta::node_seq_to_link_seq_bustransit, node_IDs length "
        "must not be less than 2");
    }
  int *nodes_ptr = (int *) nodes_buf.ptr;

  int new_shape[1] = { m - 1 };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  for (int i = 0; i < m - 1; ++i)
    {
      result_ptr[i]
        = m_mmdta->m_bus_transit_graph->GetEId (nodes_ptr[i], nodes_ptr[i + 1]);
      if (result_ptr[i] < 0)
        {
          throw std::runtime_error (
            "Error, Mmdta::node_seq_to_link_seq_bustransit, link does not "
            "exist");
        }
    }

  return result;
}

py::array_t<double>
Mmdta::get_passenger_path_cost_driving (py::array_t<int> link_IDs,
                                        py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_driving, start_intervals "
        "input dimension mismatch");
    }
  int l = start_buf.shape[0];
  double *start_ptr = (double *) start_buf.ptr;

  auto links_buf = link_IDs.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_driving, link_IDs input "
        "dimension mismatch");
    }
  int m = links_buf.shape[0];
  int *links_ptr = (int *) links_buf.ptr;

  auto *_path = new MNM_Path ();
  _path->m_node_vec.push_back (
    m_mmdta->m_graph->GetEI (links_ptr[0]).GetSrcNId ());
  for (int i = 0; i < m; ++i)
    {
      _path->m_link_vec.push_back (links_ptr[i]);
      _path->m_node_vec.push_back (
        m_mmdta->m_graph->GetEI (links_ptr[i]).GetDstNId ());
    }
  TInt _dest_node_ID = _path->m_node_vec.back ();
  auto *_dest = dynamic_cast<MNM_Destination_Multimodal *> (
    ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (_dest_node_ID))->m_dest);

  auto *_p_path = new MNM_Passenger_Path_Driving (
    driving, _path, m_mmdue->m_vot, m_mmdue->m_early_penalty,
    m_mmdue->m_late_penalty, m_mmdue->m_target_time, 1,
    m_mmdue->m_carpool_cost_multiplier, 0.0, _dest->m_parking_lot,
    m_mmdue->m_parking_lot_to_destination_walking_time);
  _path = nullptr;
  IAssert (_p_path->m_path != nullptr);

  int new_shape[2]
    = { 3, l }; // rows: car travel time, truck travel time, car travel cost
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  TFlt _tt;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_path_cost_driving, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      _tt = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                      m_mmdue->m_link_tt_map,
                                      m_mmdue->m_transitlink_cost_map);
      // _tt = _p_path -> get_travel_time(TFlt(start_ptr[t]), m_mmdta);
      result_ptr[t] = _tt () * m_mmdta->m_unit_time;
      result_ptr[l + t]
        = _p_path->get_travel_time_truck (TFlt (start_ptr[t]), m_mmdta,
                                          m_mmdue->m_link_tt_map_truck) ()
          * m_mmdta->m_unit_time;
      result_ptr[2 * l + t]
        = _p_path->get_travel_cost_with_tt (TFlt (start_ptr[t]), _tt,
                                            m_mmdta) ();
    }

  delete _p_path;
  return result;
}

py::array_t<double>
Mmdta::get_passenger_path_cost_bus (py::array_t<int> link_IDs,
                                    py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_bus, start_intervals input "
        "dimension mismatch");
    }
  int l = start_buf.shape[0];
  double *start_ptr = (double *) start_buf.ptr;

  auto links_buf = link_IDs.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_bus, link_IDs input "
        "dimension mismatch");
    }
  int m = links_buf.shape[0];
  int *links_ptr = (int *) links_buf.ptr;

  auto *_path = new MNM_Path ();
  _path->m_node_vec.push_back (
    m_mmdta->m_bus_transit_graph->GetEI (links_ptr[0]).GetSrcNId ());
  for (int i = 0; i < m; ++i)
    {
      _path->m_link_vec.push_back (links_ptr[i]);
      _path->m_node_vec.push_back (
        m_mmdta->m_bus_transit_graph->GetEI (links_ptr[i]).GetDstNId ());
    }

  auto *_p_path
    = new MNM_Passenger_Path_Bus (transit, _path, m_mmdue->m_vot,
                                  m_mmdue->m_early_penalty,
                                  m_mmdue->m_late_penalty,
                                  m_mmdue->m_target_time, m_mmdue->m_bus_fare,
                                  m_mmdue->m_bus_inconvenience);
  _path = nullptr;
  IAssert (_p_path->m_path != nullptr);

  int new_shape[2] = { 2, l }; // rows: travel time, travel cost
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  TFlt _tt;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_path_cost_bus, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      _tt = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                      m_mmdue->m_link_tt_map,
                                      m_mmdue->m_transitlink_tt_map);
      // _tt = _p_path -> get_travel_time(TFlt(start_ptr[t]), m_mmdta);
      result_ptr[t] = _tt () * m_mmdta->m_unit_time;
      result_ptr[l + t] = _p_path->get_travel_cost_with_tt (TFlt (start_ptr[t]),
                                                            _tt, m_mmdta) ();
    }

  delete _p_path;
  return result;
}

py::array_t<double>
Mmdta::get_passenger_path_cost_pnr (py::array_t<int> link_IDs_driving,
                                    py::array_t<int> link_IDs_bustransit,
                                    py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_pnr, start_intervals input "
        "dimension mismatch");
    }
  int l = start_buf.shape[0];
  double *start_ptr = (double *) start_buf.ptr;

  auto links_buf_driving = link_IDs_driving.request ();
  if (links_buf_driving.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_pnr, link_IDs_driving input "
        "dimension mismatch");
    }
  int m_driving = links_buf_driving.shape[0];
  int *links_ptr_driving = (int *) links_buf_driving.ptr;

  auto links_buf_bustransit = link_IDs_bustransit.request ();
  if (links_buf_bustransit.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_path_cost_pnr, link_IDs_bustransit "
        "input dimension mismatch");
    }
  int m_bustransit = links_buf_bustransit.shape[0];
  int *links_ptr_bustransit = (int *) links_buf_bustransit.ptr;

  auto *_path_driving = new MNM_Path ();
  _path_driving->m_node_vec.push_back (
    m_mmdta->m_graph->GetEI (links_ptr_driving[0]).GetSrcNId ());
  for (int i = 0; i < m_driving; ++i)
    {
      _path_driving->m_link_vec.push_back (links_ptr_driving[i]);
      _path_driving->m_node_vec.push_back (
        m_mmdta->m_graph->GetEI (links_ptr_driving[i]).GetDstNId ());
    }
  TInt _mid_dest_node_ID = _path_driving->m_node_vec.back ();
  auto *_mid_dest = dynamic_cast<MNM_Destination_Multimodal *> (
    ((MNM_DMDND *) m_mmdta->m_node_factory->get_node (_mid_dest_node_ID))
      ->m_dest);
  IAssert (_mid_dest->m_parking_lot != nullptr);

  auto *_path_bustransit = new MNM_Path ();
  _path_bustransit->m_node_vec.push_back (
    m_mmdta->m_bus_transit_graph->GetEI (links_ptr_bustransit[0]).GetSrcNId ());
  for (int i = 0; i < m_bustransit; ++i)
    {
      _path_bustransit->m_link_vec.push_back (links_ptr_bustransit[i]);
      _path_bustransit->m_node_vec.push_back (
        m_mmdta->m_bus_transit_graph->GetEI (links_ptr_bustransit[i])
          .GetDstNId ());
    }

  auto *_path_pnr
    = new MNM_PnR_Path (0, _mid_dest->m_parking_lot->m_ID, _mid_dest_node_ID,
                        _path_driving, _path_bustransit);
  _path_driving = nullptr;
  _path_bustransit = nullptr;

  auto *_p_path
    = new MNM_Passenger_Path_PnR (pnr, _path_pnr, m_mmdue->m_vot,
                                  m_mmdue->m_early_penalty,
                                  m_mmdue->m_late_penalty,
                                  m_mmdue->m_target_time, 0.0,
                                  _mid_dest->m_parking_lot, m_mmdue->m_bus_fare,
                                  m_mmdue->m_pnr_inconvenience);
  _path_pnr = nullptr;
  IAssert (_p_path->m_path != nullptr);

  int new_shape[2] = { 2, l }; // rows: travel time, travel cost
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  TFlt _tt;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_path_cost_pnr, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      _tt = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                      m_mmdue->m_link_tt_map,
                                      m_mmdue->m_transitlink_tt_map);
      // _tt = _p_path -> get_travel_time(TFlt(start_ptr[t]), m_mmdta);
      result_ptr[t] = _tt () * m_mmdta->m_unit_time;
      result_ptr[l + t] = _p_path->get_travel_cost_with_tt (TFlt (start_ptr[t]),
                                                            _tt, m_mmdta) ();
    }

  delete _p_path;
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_tt_truck (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_tt_truck, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_truck, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_driving.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_truck, invalid path");
            }
          _p_path = m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
                      ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)
                   == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_truck, invalid "
                "passenger path");
            }
          // double _tmp = dynamic_cast<MNM_Passenger_Path_Driving*>(_p_path) ->
          // get_travel_time_truck(TFlt(start_ptr[t]), m_mmdta)() * m_mmdta ->
          // m_unit_time;
          double _tmp
            = dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)
                ->get_travel_time_truck (TFlt (start_ptr[t]), m_mmdta,
                                         m_mmdue->m_link_tt_map_truck) ()
              * m_mmdta->m_unit_time;
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_distance_driving ()
{
  int new_shape[2] = { (int) m_path_vec_driving.size (), 1 };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;

  for (size_t i = 0; i < m_path_vec_driving.size (); ++i)
    {
      if (m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
          == m_ID_path_mapping.end ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_distance_driving, invalid "
            "path");
        }
      _p_path = m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
                  ->second.second;
      if (_p_path == nullptr
          || dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path) == nullptr)
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_distance_driving, invalid "
            "passenger path");
        }

      double _tmp = _p_path->get_length (m_mmdta) () / 1600; // miles
      result_ptr[i] = _tmp;
    }

  return result;
}

py::array_t<double>
Mmdta::get_registered_path_distance_bustransit ()
{
  int new_shape[2] = { (int) m_path_vec_bustransit.size (), 1 };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;

  for (size_t i = 0; i < m_path_vec_bustransit.size (); ++i)
    {
      if (m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
          == m_ID_path_mapping.end ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_distance_bustransit, "
            "invalid path");
        }
      _p_path = m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
                  ->second.second;
      if (_p_path == nullptr
          || dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path) == nullptr)
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_distance_bustransit, "
            "invalid passenger path");
        }
      double _tmp = _p_path->get_length (m_mmdta) () / 1600; // miles
      result_ptr[i] = _tmp;
    }

  return result;
}

py::array_t<double>
Mmdta::get_registered_path_distance_pnr ()
{
  int new_shape[2] = { (int) m_path_vec_pnr.size (), 1 };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;

  for (size_t i = 0; i < m_path_vec_pnr.size (); ++i)
    {
      if (m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)
          == m_ID_path_mapping.end ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_pnr, invalid path");
        }
      _p_path
        = m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)->second.second;
      if (_p_path == nullptr
          || dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path) == nullptr)
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_pnr, invalid passenger "
            "path");
        }
      double _tmp = _p_path->get_length (m_mmdta) () / 1600; // miles
      result_ptr[i] = _tmp;
    }

  return result;
}

py::array_t<double>
Mmdta::get_registered_path_tt_driving (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_tt_driving, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_driving, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_driving.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_driving, invalid "
                "path");
            }
          _p_path = m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
                      ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)
                   == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_driving, invalid "
                "passenger path");
            }
          double _tmp
            = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ()
              * m_mmdta->m_unit_time;
          // double _tmp = _p_path ->get_travel_time(TFlt(start_ptr[t]),
          // m_mmdta)() * m_mmdta -> m_unit_time;
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_tt_bustransit (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_tt_bustransit, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_bustransit.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_bustransit, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_bustransit.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_bustransit, invalid "
                "path");
            }
          _p_path
            = m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
                ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path) == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_bustransit, invalid "
                "passenger path");
            }
          double _tmp
            = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ()
              * m_mmdta->m_unit_time;
          // double _tmp = _p_path ->get_travel_time(TFlt(start_ptr[t]),
          // m_mmdta)() * m_mmdta -> m_unit_time;
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_tt_pnr (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mmdta::get_registered_path_tt_pnr, "
                                "input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_pnr.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_tt_pnr, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_pnr.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_pnr, invalid path");
            }
          _p_path = m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)
                      ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path) == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_tt_pnr, invalid "
                "passenger path");
            }
          double _tmp
            = _p_path->get_travel_time (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ()
              * m_mmdta->m_unit_time;
          // double _tmp = _p_path ->get_travel_time(TFlt(start_ptr[t]),
          // m_mmdta)() * m_mmdta -> m_unit_time;
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_cost_driving (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_cost_driving, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_cost_driving, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_driving.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_driving, invalid "
                "path");
            }
          _p_path = m_ID_path_mapping.find (m_path_vec_driving[i]->m_path_ID)
                      ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)
                   == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_driving, invalid "
                "passenger path");
            }
          // double _tmp = _p_path ->get_travel_cost(TFlt(start_ptr[t]),
          // m_mmdta)();
          double _tmp
            = _p_path->get_travel_cost (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ();
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_cost_bustransit (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_cost_bustransit, input "
        "dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_bustransit.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_cost_bustransit, input "
            "start intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_bustransit.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_bustransit, "
                "invalid path");
            }
          _p_path
            = m_ID_path_mapping.find (m_path_vec_bustransit[i]->m_path_ID)
                ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path) == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_bustransit, "
                "invalid passenger path");
            }
          // double _tmp = _p_path ->get_travel_cost(TFlt(start_ptr[t]),
          // m_mmdta)();
          double _tmp
            = _p_path->get_travel_cost (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ();
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_registered_path_cost_pnr (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_registered_path_cost_pnr, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_path_vec_pnr.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  MNM_Passenger_Path_Base *_p_path;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_registered_path_cost_pnr, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_path_vec_pnr.size (); ++i)
        {
          if (m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)
              == m_ID_path_mapping.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_pnr, invalid path");
            }
          _p_path = m_ID_path_mapping.find (m_path_vec_pnr[i]->m_path_ID)
                      ->second.second;
          if (_p_path == nullptr
              || dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path) == nullptr)
            {
              throw std::runtime_error (
                "Error, Mmdta::get_registered_path_cost_pnr, invalid "
                "passenger path");
            }
          // double _tmp = _p_path ->get_travel_cost(TFlt(start_ptr[t]),
          // m_mmdta)();
          double _tmp
            = _p_path->get_travel_cost (TFlt (start_ptr[t]), m_mmdta,
                                        m_mmdue->m_link_tt_map,
                                        m_mmdue->m_transitlink_tt_map) ();
          result_ptr[i * l + t] = _tmp;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_path_tt_car (py::array_t<int> link_IDs,
                        py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();
  int num_link = links_buf.shape[0];

  int new_shape[1] = { num_link };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  double *start_ptr = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          double avg_tt = 0;
          for (int t = 0; t < num_int; ++t)
            {
              if (start_ptr[t] >= get_cur_loading_interval ())
                {
                  throw std::runtime_error (
                    "Error, Mmdta::get_path_tt_car, input start intervals "
                    "exceeds the total loading intervals - 1");
                }
              double _tmp
                = MNM_DTA_GRADIENT::
                    get_travel_time_car (_mclink, TFlt (start_ptr[t] + 1),
                                         m_mmdta->m_unit_time,
                                         m_mmdta->m_current_loading_interval) ()
                  * m_mmdta->m_unit_time;
              if (_tmp * m_mmdta->m_unit_time
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
          throw std::runtime_error (
            "Mmdta::get_path_tt_car: link type is not multiclass");
        }
    }

  return result;
}

py::array_t<double>
Mmdta::get_path_tt_truck (py::array_t<int> link_IDs,
                          py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  int num_int = start_buf.shape[0];

  auto links_buf = link_IDs.request ();
  int num_link = links_buf.shape[0];

  int new_shape[1] = { num_link };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  double *start_ptr = (double *) start_buf.ptr;
  int *links_ptr = (int *) links_buf.ptr;
  MNM_Dlink *_link;
  for (int i = 0; i < links_buf.shape[0]; i++)
    {
      _link = m_mmdta->m_link_factory->get_link (TInt (links_ptr[i]));
      if (MNM_Dlink_Multiclass *_mclink
          = dynamic_cast<MNM_Dlink_Multiclass *> (_link))
        {
          double avg_tt = 0;
          for (int t = 0; t < num_int; ++t)
            {
              if (start_ptr[t] >= get_cur_loading_interval ())
                {
                  throw std::runtime_error (
                    "Error, Mmdta::get_path_tt_truck, input start "
                    "intervals exceeds the total loading intervals - 1");
                }
              double _tmp
                = MNM_DTA_GRADIENT::
                    get_travel_time_truck (_mclink, TFlt (start_ptr[t] + 1),
                                           m_mmdta->m_unit_time,
                                           m_mmdta
                                             ->m_current_loading_interval) ()
                  * m_mmdta->m_unit_time;
              if (_tmp * m_mmdta->m_unit_time
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
          throw std::runtime_error (
            "Mmdta::get_path_tt_truck: link type is not multiclass");
        }
    }

  return result;
}

int
Mmdta::update_tdsp_tree ()
{
  // m_mmdue -> build_link_cost_map(m_mmdta, false) should be called first
  // before this function
  if (!m_tdsp_tree_map_driving.empty ())
    {
      for (auto _it : m_tdsp_tree_map_driving)
        {
          delete _it.second;
        }
      m_tdsp_tree_map_driving.clear ();
    }
  if (!m_tdsp_tree_map_bus.empty ())
    {
      for (auto _it : m_tdsp_tree_map_bus)
        {
          delete _it.second;
        }
      m_tdsp_tree_map_bus.clear ();
    }

  MNM_Destination *_dest;
  TInt _dest_node_ID;
  MNM_TDSP_Tree *_tdsp_tree;

  for (auto _d_it : m_mmdta->m_od_factory->m_destination_map)
    {
      _dest = _d_it.second;
      _dest_node_ID = _dest->m_dest_node->m_node_ID;

      // for driving
      _tdsp_tree = new MNM_TDSP_Tree (_dest_node_ID, m_mmdta->m_graph,
                                      m_mmdue->m_total_loading_inter);
      _tdsp_tree->initialize ();
      _tdsp_tree->update_tree (m_mmdue->m_link_cost_map,
                               m_mmdue->m_link_tt_map);
      m_tdsp_tree_map_driving.insert (
        std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
      _tdsp_tree = nullptr;
      IAssert (m_tdsp_tree_map_driving.find (_dest_node_ID)->second != nullptr);

      // for bus transit
      if (m_mmdta->m_bus_transit_graph->IsNode (_dest_node_ID))
        {
          _tdsp_tree
            = new MNM_TDSP_Tree (_dest_node_ID, m_mmdta->m_bus_transit_graph,
                                 m_mmdue->m_total_loading_inter);
          _tdsp_tree->initialize ();
          _tdsp_tree->update_tree (m_mmdue->m_transitlink_cost_map,
                                   m_mmdue->m_transitlink_tt_map);
          m_tdsp_tree_map_bus.insert (
            std::pair<TInt, MNM_TDSP_Tree *> (_dest_node_ID, _tdsp_tree));
          _tdsp_tree = nullptr;
          IAssert (m_tdsp_tree_map_bus.find (_dest_node_ID)->second != nullptr);
        }
    }
  return 0;
}

py::array_t<int>
Mmdta::get_lowest_cost_path (int start_interval, int o_node_ID, int d_node_ID)
{
  // get lowest cost path departing at start_interval
  IAssert (start_interval
           < m_mmdue->m_total_assign_inter
               * m_mmdue->m_mmdta_config->get_int ("assign_frq"));
  IAssert (
    m_mmdue->m_passenger_demand.find (o_node_ID)
      != m_mmdue->m_passenger_demand.end ()
    && m_mmdue->m_passenger_demand.find (o_node_ID)->second.find (d_node_ID)
         != m_mmdue->m_passenger_demand.find (o_node_ID)->second.end ());

  MNM_Passenger_Path_Base *_p_path;
  MNM_Path *_path;
  TInt _mode;
  TFlt _cost;
  int _best_time_col, _best_assign_col, _num_col;
  bool _exist;
  MNM_Passenger_Pathset *_path_set_driving;
  MNM_Passenger_Pathset *_path_set_bus;
  MNM_Passenger_Pathset *_path_set_pnr;
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt>, int>
    _path_result;

  _path_set_driving = nullptr;
  _path_set_bus = nullptr;
  _path_set_pnr = nullptr;
  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (),
                 driving)
        != m_mmdue->m_mode_vec.end ()
      && m_mmdue->m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (driving)
           ->second)
    {
      _path_set_driving = m_mmdue->m_passenger_path_table->find (o_node_ID)
                            ->second->find (d_node_ID)
                            ->second->find (driving)
                            ->second;
    }
  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (),
                 transit)
        != m_mmdue->m_mode_vec.end ()
      && m_mmdue->m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (transit)
           ->second)
    {
      _path_set_bus = m_mmdue->m_passenger_path_table->find (o_node_ID)
                        ->second->find (d_node_ID)
                        ->second->find (transit)
                        ->second;
    }
  if (std::find (m_mmdue->m_mode_vec.begin (), m_mmdue->m_mode_vec.end (), pnr)
        != m_mmdue->m_mode_vec.end ()
      && m_mmdue->m_od_mode_connectivity.find (o_node_ID)
           ->second.find (d_node_ID)
           ->second.find (pnr)
           ->second)
    {
      _path_set_pnr = m_mmdue->m_passenger_path_table->find (o_node_ID)
                        ->second->find (d_node_ID)
                        ->second->find (pnr)
                        ->second;
    }

  _path_result
    = m_mmdue->get_best_path_for_single_interval (start_interval, o_node_ID,
                                                  d_node_ID,
                                                  m_tdsp_tree_map_driving,
                                                  m_tdsp_tree_map_bus, m_mmdta);
  _p_path = std::get<0> (_path_result.first);
  _cost = std::get<2> (_path_result.first);
  _mode = _path_result.second;
  _best_time_col = std::get<1> (_path_result.first);
  _best_assign_col
    = (int) _best_time_col / m_mmdue->m_mmdta_config->get_int ("assign_frq");
  if (_best_assign_col >= m_mmdue->m_total_assign_inter)
    _best_assign_col = m_mmdue->m_total_assign_inter - 1;

  _exist = false;
  _path = nullptr;
  if (_mode == driving && _path_set_driving != nullptr)
    {
      _exist = _path_set_driving->is_in (_p_path);
      _path = dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)->m_path;
      _num_col = (int) _path->m_node_vec.size ();
    }
  else if (_mode == transit && _path_set_bus != nullptr)
    {
      _exist = _path_set_bus->is_in (_p_path);
      _path = dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path)->m_path;
      _num_col = (int) _path->m_link_vec.size ();
    }
  else if (_mode == pnr && _path_set_pnr != nullptr)
    {
      _exist = _path_set_pnr->is_in (_p_path);
      _path = dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)->m_path;
      _num_col = std::max (int (dynamic_cast<MNM_PnR_Path *> (_path)
                                  ->m_driving_path->m_node_vec.size ()),
                           int (dynamic_cast<MNM_PnR_Path *> (_path)
                                  ->m_transit_path->m_link_vec.size ()));
    }
  else
    {
      throw std::runtime_error ("unsupported mode");
    }
  IAssert (_path != nullptr);

  int new_shape[2] = {
    4, _num_col
  }; // row: _exist, _mode, driving path node vec, transit path link vec
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  for (int i = 0; i < _num_col; ++i)
    {
      if (i == 0)
        {
          result_ptr[i + _num_col * 0] = (int) _exist;
          result_ptr[i + _num_col * 1] = (int) _mode;
        }
      else
        {
          result_ptr[i + _num_col * 0] = -1;
          result_ptr[i + _num_col * 1] = -1;
        }

      if (_mode == driving)
        {
          result_ptr[i + _num_col * 2] = _path->m_node_vec[i];
          result_ptr[i + _num_col * 3] = -1;
        }
      else if (_mode == transit)
        {
          result_ptr[i + _num_col * 2] = -1;
          result_ptr[i + _num_col * 3] = _path->m_link_vec[i];
        }
      else if (_mode == pnr)
        {
          if (i < int (dynamic_cast<MNM_PnR_Path *> (_path)
                         ->m_driving_path->m_node_vec.size ()))
            {
              result_ptr[i + _num_col * 2]
                = dynamic_cast<MNM_PnR_Path *> (_path)
                    ->m_driving_path->m_node_vec[i];
            }
          else
            {
              result_ptr[i + _num_col * 2] = -1;
            }

          if (i < int (dynamic_cast<MNM_PnR_Path *> (_path)
                         ->m_transit_path->m_link_vec.size ()))
            {
              result_ptr[i + _num_col * 3]
                = dynamic_cast<MNM_PnR_Path *> (_path)
                    ->m_transit_path->m_link_vec[i];
            }
          else
            {
              result_ptr[i + _num_col * 3] = -1;
            }
        }
    }

  return result;
}

int
Mmdta::build_link_cost_map (bool with_congestion_indicator)
{
  m_mmdue->build_link_cost_map (m_mmdta, with_congestion_indicator);
  return 0;
}

int
Mmdta::get_link_queue_dissipated_time ()
{
  // build_link_cost_map should be called first
  m_mmdue->get_link_queue_dissipated_time (m_mmdta);
  return 0;
}

int
Mmdta::build_link_cost_map_snapshot (int start_interval, bool in_simulation)
{
  m_mmdue->build_link_cost_map_snapshot (m_mmdta, start_interval,
                                         in_simulation);
  return 0;
}

int
Mmdta::update_snapshot_route_table (int start_interval)
{
  // build_link_cost_map_snapshot should be called first
  m_mmdue->update_snapshot_route_table (m_mmdta, start_interval);
  return 0;
}

py::array_t<int>
Mmdta::get_lowest_cost_path_snapshot (int start_interval, int o_node_ID,
                                      int d_node_ID)
{
  std::pair<std::tuple<MNM_Passenger_Path_Base *, TInt, TFlt, bool>, int> _best
    = m_mmdue->get_lowest_cost_path_snapshot (start_interval, o_node_ID,
                                              d_node_ID, m_mmdta);

  int _mode = _best.second;
  MNM_Passenger_Path_Base *_p_path = std::get<0> (_best.first);
  bool _exist = std::get<3> (_best.first);

  MNM_Path *_path = nullptr;
  int _num_col;

  if (_mode == driving)
    {
      _path = dynamic_cast<MNM_Passenger_Path_Driving *> (_p_path)->m_path;
      _num_col = (int) _path->m_node_vec.size ();
    }
  else if (_mode == transit)
    {
      _path = dynamic_cast<MNM_Passenger_Path_Bus *> (_p_path)->m_path;
      _num_col = (int) _path->m_link_vec.size ();
    }
  else if (_mode == pnr)
    {
      _path = dynamic_cast<MNM_Passenger_Path_PnR *> (_p_path)->m_path;
      _num_col = std::max (int (dynamic_cast<MNM_PnR_Path *> (_path)
                                  ->m_driving_path->m_node_vec.size ()),
                           int (dynamic_cast<MNM_PnR_Path *> (_path)
                                  ->m_transit_path->m_link_vec.size ()));
    }
  else
    {
      throw std::runtime_error ("unsupported mode");
    }
  IAssert (_path != nullptr);

  int new_shape[2] = {
    4, _num_col
  }; // row: _exist, _mode, driving path node vec, transit path link vec
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;

  for (int i = 0; i < _num_col; ++i)
    {
      if (i == 0)
        {
          result_ptr[i + _num_col * 0] = (int) _exist;
          result_ptr[i + _num_col * 1] = (int) _mode;
        }
      else
        {
          result_ptr[i + _num_col * 0] = -1;
          result_ptr[i + _num_col * 1] = -1;
        }

      if (_mode == driving)
        {
          result_ptr[i + _num_col * 2] = _path->m_node_vec[i];
          result_ptr[i + _num_col * 3] = -1;
        }
      else if (_mode == transit)
        {
          result_ptr[i + _num_col * 2] = -1;
          result_ptr[i + _num_col * 3] = _path->m_link_vec[i];
        }
      else if (_mode == pnr)
        {
          if (i < int (dynamic_cast<MNM_PnR_Path *> (_path)
                         ->m_driving_path->m_node_vec.size ()))
            {
              result_ptr[i + _num_col * 2]
                = dynamic_cast<MNM_PnR_Path *> (_path)
                    ->m_driving_path->m_node_vec[i];
            }
          else
            {
              result_ptr[i + _num_col * 2] = -1;
            }

          if (i < int (dynamic_cast<MNM_PnR_Path *> (_path)
                         ->m_transit_path->m_link_vec.size ()))
            {
              result_ptr[i + _num_col * 3]
                = dynamic_cast<MNM_PnR_Path *> (_path)
                    ->m_transit_path->m_link_vec[i];
            }
          else
            {
              result_ptr[i + _num_col * 3] = -1;
            }
        }
    }

  return result;
}

py::array_t<int>
Mmdta::get_od_mode_connectivity ()
{
  int _num_col = 5;
  int _num_OD = m_mmdue->m_mmdta_config->get_int ("OD_pair_passenger");
  int new_shape[2]
    = { _num_OD, _num_col }; // O_node, D_node, driving, bustransit, pnr

  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;
  int i = 0;
  for (const auto &_o_it : m_mmdue->m_od_mode_connectivity)
    {
      result_ptr[i * _num_col] = _o_it.first;
      for (const auto &_d_it : _o_it.second)
        {
          result_ptr[i * _num_col + 1] = _d_it.first;
          for (auto _mode_it : _d_it.second)
            {
              if (_mode_it.first == driving)
                {
                  result_ptr[i * _num_col + 2] = (int) _mode_it.second;
                }
              else if (_mode_it.first == transit)
                {
                  result_ptr[i * _num_col + 3] = (int) _mode_it.second;
                }
              else if (_mode_it.first == pnr)
                {
                  result_ptr[i * _num_col + 4] = (int) _mode_it.second;
                }
              else
                {
                  throw std::runtime_error (
                    "Error, Mmdta::get_od_mode_connectivity, mode not "
                    "implemented");
                }
            }
          i++;
        }
    }
  return result;
}

int
Mmdta::generate_init_mode_demand_file (const std::string &file_folder)
{
  MNM::generate_init_mode_demand_file (m_mmdue, file_folder, "driving_demand",
                                       "bustransit_demand", "pnr_demand");
  return 0;
}

py::array_t<double>
Mmdta::get_car_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i] = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mmdta->m_link_factory->get_link (start_ptr[i]))
                        ->get_link_freeflow_tt_loading_car ()
                      * m_mmdue->m_unit_time; // seconds
    }
  return result;
}

py::array_t<double>
Mmdta::get_truck_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i] = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mmdta->m_link_factory->get_link (start_ptr[i]))
                        ->get_link_freeflow_tt_loading_truck ()
                      * m_mmdue->m_unit_time; // seconds
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i]
        = dynamic_cast<MNM_Bus_Link *> (
            m_mmdta->m_transitlink_factory->get_transit_link (start_ptr[i]))
            ->m_fftt; // seconds
    }
  return result;
}

py::array_t<double>
Mmdta::get_walking_link_fftt (py::array_t<int> link_IDs)
{
  auto start_buf = link_IDs.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_walking_link_fftt, input dimension mismatch");
    }
  int l = start_buf.shape[0];

  auto result = py::array_t<double> (l);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  int *start_ptr = (int *) start_buf.ptr;

  for (int i = 0; i < l; ++i)
    {
      result_ptr[i]
        = dynamic_cast<MNM_Walking_Link *> (
            m_mmdta->m_transitlink_factory->get_transit_link (start_ptr[i]))
            ->m_fftt; // seconds
    }
  return result;
}

// unit: m_mmdta -> m_unit_time (eg: 5 seconds)
py::array_t<double>
Mmdta::get_car_link_tt (py::array_t<double> start_intervals, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_car_link_tt, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car (m_link_vec_driving[i], TFlt (start_ptr[t] + 1),
                                 m_mmdta->m_unit_time,
                                 m_mmdta->m_current_loading_interval) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_driving[i]->m_length
                     / m_link_vec_driving[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_driving[i]->m_length
                         / m_link_vec_driving[i]->m_ffs_car
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_car_link_tt_robust (py::array_t<double> start_intervals,
                               py::array_t<double> end_intervals,
                               int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_tt_robust, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= end_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mmdta::get_car_link_tt_robust, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_car_link_tt_robust, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_car_link_tt_robust, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car_robust (m_link_vec_driving[i],
                                        TFlt (start_ptr[t] + 1),
                                        TFlt (end_ptr[t] + 1),
                                        m_mmdta->m_unit_time,
                                        m_mmdta->m_current_loading_interval,
                                        num_trials) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_driving[i]->m_length
                     / m_link_vec_driving[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_driving[i]->m_length
                         / m_link_vec_driving[i]->m_ffs_car
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_truck_link_tt (py::array_t<double> start_intervals, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_truck_link_tt, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck (m_link_vec_driving[i],
                                   TFlt (start_ptr[t] + 1),
                                   m_mmdta->m_unit_time,
                                   m_mmdta->m_current_loading_interval) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_driving[i]->m_length
                     / m_link_vec_driving[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_driving[i]->m_length
                         / m_link_vec_driving[i]->m_ffs_truck
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_truck_link_tt_robust (py::array_t<double> start_intervals,
                                 py::array_t<double> end_intervals,
                                 int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_tt_robust, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= end_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mmdta::get_truck_link_tt_robust, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_truck_link_tt_robust, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_truck_link_tt_robust, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck_robust (m_link_vec_driving[i],
                                          TFlt (start_ptr[t] + 1),
                                          TFlt (end_ptr[t] + 1),
                                          m_mmdta->m_unit_time,
                                          m_mmdta->m_current_loading_interval,
                                          num_trials) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_driving[i]->m_length
                     / m_link_vec_driving[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_driving[i]->m_length
                         / m_link_vec_driving[i]->m_ffs_truck
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_link_tt (py::array_t<double> start_intervals, bool return_inf,
                        bool return_bus_time)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_tt, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_link_tt, input start intervals exceeds "
            "the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_bus (m_link_vec_bus[i], TFlt (start_ptr[t] + 1),
                                 m_mmdta->m_unit_time,
                                 m_mmdta->m_current_loading_interval,
                                 m_mmdta->m_explicit_bus, return_inf,
                                 return_bus_time) ();
          // if (std::isinf(_tmp)) {
          //     result_ptr[i * l + t] =
          //     std::numeric_limits<double>::quiet_NaN();
          // }
          // else {
          //     result_ptr[i * l + t] = _tmp * m_mmdta -> m_unit_time;  //
          //     seconds
          // }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_link_tt_robust (py::array_t<double> start_intervals,
                               py::array_t<double> end_intervals,
                               int num_trials, bool return_inf,
                               bool return_bus_time)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_tt_robust, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_tt_robust, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };

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
            "Error, Mmdta::get_bus_link_tt_robust, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_link_tt_robust, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_link_tt_robust, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_bus_robust (m_link_vec_bus[i],
                                        TFlt (start_ptr[t] + 1),
                                        TFlt (end_ptr[t] + 1),
                                        m_mmdta->m_unit_time,
                                        m_mmdta->m_current_loading_interval,
                                        num_trials, m_mmdta->m_explicit_bus,
                                        return_inf, return_bus_time) ();
          // if (std::isinf(_tmp)) {
          //     result_ptr[i * l + t] =
          //     std::numeric_limits<double>::quiet_NaN();
          // }
          // else {
          //     result_ptr[i * l + t] = _tmp * m_mmdta -> m_unit_time;  //
          //     seconds
          // }
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_passenger_walking_link_tt (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_walking_link_tt, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_walking.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_walking_link_tt, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_walking (m_link_vec_walking[i],
                                     TFlt (start_ptr[t] + 1),
                                     m_mmdta->m_unit_time,
                                     m_mmdta->m_current_loading_interval) ();
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_passenger_walking_link_tt_robust (
  py::array_t<double> start_intervals, py::array_t<double> end_intervals,
  int num_trials)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_walking_link_tt_robust, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_walking_link_tt_robust, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_walking.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= end_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_walking_link_tt_robust, end time "
            "is smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_walking_link_tt_robust, input "
            "start intervals exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_passenger_walking_link_tt_robust, input end "
            "intervals exceeds the total loading intervals");
        }
      // use start_ptr[t] + 1 as start_time in cc to compute link travel time
      // for vehicles arriving at the beginning of interval start_ptr[t]
      for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_walking_robust (m_link_vec_walking[i],
                                            TFlt (start_ptr[t] + 1),
                                            TFlt (end_ptr[t] + 1),
                                            m_mmdta->m_unit_time,
                                            m_mmdta->m_current_loading_interval,
                                            num_trials) ();
          result_ptr[i * l + t] = _tmp * m_mmdta->m_unit_time; // seconds
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_driving_link_tt_car (py::array_t<double> start_intervals,
                                    bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_car, input dimension "
        "mismatch");
    }
  if (m_link_vec_bus_driving.empty ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_car, m_link_vec_bus_driving "
        "is empty");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_car, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car (m_link_vec_bus_driving[i],
                                 TFlt (start_ptr[t] + 1), m_mmdta->m_unit_time,
                                 m_mmdta->m_current_loading_interval) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_bus_driving[i]->m_length
                     / m_link_vec_bus_driving[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_bus_driving[i]->m_length
                         / m_link_vec_bus_driving[i]->m_ffs_car
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t]
            = _tmp
              * m_mmdta
                  ->m_unit_time; // seconds, the length portion is in python
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_driving_link_tt_car_robust (py::array_t<double> start_intervals,
                                           py::array_t<double> end_intervals,
                                           int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_car_robust, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_car_robust, input length "
        "mismatch");
    }
  if (m_link_vec_bus_driving.empty ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_car_robust, "
        "m_link_vec_bus_driving is empty");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= end_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_car_robust, end time is "
            "smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_car_robust, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_car_robust, input end "
            "intervals exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_car_robust (m_link_vec_bus_driving[i],
                                        TFlt (start_ptr[t] + 1),
                                        TFlt (end_ptr[t] + 1),
                                        m_mmdta->m_unit_time,
                                        m_mmdta->m_current_loading_interval,
                                        num_trials) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_bus_driving[i]->m_length
                     / m_link_vec_bus_driving[i]->m_ffs_car))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_bus_driving[i]->m_length
                         / m_link_vec_bus_driving[i]->m_ffs_car
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t]
            = _tmp
              * m_mmdta
                  ->m_unit_time; // seconds, the length portion is in python
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_driving_link_tt_truck (py::array_t<double> start_intervals,
                                      bool return_inf)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt, input dimension mismatch");
    }
  if (m_link_vec_bus_driving.empty ())
    {
      throw std::runtime_error ("Error, Mmdta::get_bus_driving_link_tt, "
                                "m_link_vec_bus_driving is empty");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck (m_link_vec_bus_driving[i],
                                   TFlt (start_ptr[t] + 1),
                                   m_mmdta->m_unit_time,
                                   m_mmdta->m_current_loading_interval) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_bus_driving[i]->m_length
                     / m_link_vec_bus_driving[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_bus_driving[i]->m_length
                         / m_link_vec_bus_driving[i]->m_ffs_truck
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t]
            = _tmp
              * m_mmdta
                  ->m_unit_time; // seconds, the length portion is in python
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_driving_link_tt_truck_robust (
  py::array_t<double> start_intervals, py::array_t<double> end_intervals,
  int num_trials, bool return_inf)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_robust, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_driving_link_tt_robust, input length "
        "mismatch");
    }
  if (m_link_vec_bus_driving.empty ())
    {
      throw std::runtime_error ("Error, Mmdta::get_bus_driving_link_tt_robust, "
                                "m_link_vec_bus_driving is empty");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  double *end_ptr = (double *) end_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= end_ptr[t])
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_robust, end time is "
            "smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_robust, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_driving_link_tt_robust, input end "
            "intervals exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
        {
          double _tmp = MNM_DTA_GRADIENT::
            get_travel_time_truck_robust (m_link_vec_bus_driving[i],
                                          TFlt (start_ptr[t] + 1),
                                          TFlt (end_ptr[t] + 1),
                                          m_mmdta->m_unit_time,
                                          m_mmdta->m_current_loading_interval,
                                          num_trials) ();
          if (_tmp * m_mmdta->m_unit_time
              > TT_UPPER_BOUND
                  * (m_link_vec_bus_driving[i]->m_length
                     / m_link_vec_bus_driving[i]->m_ffs_truck))
            {
              if (return_inf)
                {
                  _tmp = std::numeric_limits<double>::infinity ();
                }
              else
                {
                  _tmp = TT_UPPER_BOUND * m_link_vec_bus_driving[i]->m_length
                         / m_link_vec_bus_driving[i]->m_ffs_truck
                         / m_mmdta->m_unit_time;
                }
            }
          result_ptr[i * l + t]
            = _tmp
              * m_mmdta
                  ->m_unit_time; // seconds, the length portion is in python
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_car_link_speed (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_speed, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_car_link_speed, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      // use start_ptr[t] + 1 as start_time in cc to compute link travel time
      // for vehicles arriving at the beginning of interval start_ptr[t]
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tt
            = MNM_DTA_GRADIENT::
                get_travel_time_car (m_link_vec_driving[i],
                                     TFlt (start_ptr[t] + 1),
                                     m_mmdta->m_unit_time,
                                     m_mmdta->m_current_loading_interval) ()
              * m_mmdta->m_unit_time; // seconds
          result_ptr[i * l + t] = (m_link_vec_driving[i]->m_length) / _tt * 3600
                                  / 1600; // mile per hour
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_truck_link_speed (py::array_t<double> start_intervals)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_speed, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_truck_link_speed, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      // use start_ptr[t] + 1 as start_time in cc to compute link travel time
      // for vehicles arriving at the beginning of interval start_ptr[t]
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          double _tt
            = MNM_DTA_GRADIENT::
                get_travel_time_truck (m_link_vec_driving[i],
                                       TFlt (start_ptr[t] + 1),
                                       m_mmdta->m_unit_time,
                                       m_mmdta->m_current_loading_interval) ()
              * m_mmdta->m_unit_time; // seconds
          result_ptr[i * l + t] = (m_link_vec_driving[i]->m_length) / _tt * 3600
                                  / 1600; // mile per hour
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_bus_link_speed (py::array_t<double> start_intervals, bool return_inf,
                           bool return_bus_time)
{
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_speed, input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };

  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  double *start_ptr = (double *) start_buf.ptr;
  for (int t = 0; t < l; ++t)
    {
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_bus_link_speed, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      // use start_ptr[t] + 1 as start_time in cc to compute link travel time
      // for vehicles arriving at the beginning of interval start_ptr[t]
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          double _tt
            = MNM_DTA_GRADIENT::
                get_travel_time_bus (m_link_vec_bus[i], TFlt (start_ptr[t] + 1),
                                     m_mmdta->m_unit_time,
                                     m_mmdta->m_current_loading_interval,
                                     m_mmdta->m_explicit_bus, return_inf,
                                     return_bus_time) ()
              * m_mmdta->m_unit_time; // seconds
          if (std::isinf (_tt))
            {
              result_ptr[i * l + t] = 0;
            }
          else
            {
              result_ptr[i * l + t] = (m_link_vec_bus[i]->m_length) / _tt * 3600
                                      / 1600; // mile per hour
            }
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_link_car_inflow (py::array_t<int> start_intervals,
                            py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_car_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_car_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };
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
            "Error, Mmdta::get_link_car_inflow, end time is smaller than "
            "or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_car_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_car_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_car (m_link_vec_driving[i],
                                                     TFlt (start_ptr[t]),
                                                     TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_link_truck_inflow (py::array_t<int> start_intervals,
                              py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_truck_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_truck_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_driving.size (), l };
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
            "Error, Mmdta::get_link_truck_inflow, end time is smaller than "
            "or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_truck_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_truck_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_truck (m_link_vec_driving[i],
                                                       TFlt (start_ptr[t]),
                                                       TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_link_bus_inflow (py::array_t<int> start_intervals,
                            py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_bus_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_bus_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };
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
            "Error, Mmdta::get_link_bus_inflow, end time is smaller than "
            "or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_bus_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_bus_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_bus (m_link_vec_bus[i],
                                                     TFlt (start_ptr[t]),
                                                     TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_busstop_bus_inflow (py::array_t<int> start_intervals,
                               py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_busstop_bus_inflow, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_busstop_bus_inflow, input length mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };
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
            "Error, Mmdta::get_busstop_bus_inflow, end time is smaller "
            "than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_busstop_bus_inflow, input start intervals "
            "exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_busstop_bus_inflow, input end intervals "
            "exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_busstop_inflow_bus (m_link_vec_bus[i]
                                                          ->m_to_busstop,
                                                        TFlt (start_ptr[t]),
                                                        TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_link_bus_passenger_inflow (py::array_t<int> start_intervals,
                                      py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_bus_passenger_inflow, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_bus_passenger_inflow, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_bus.size (), l };
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
            "Error, Mmdta::get_link_bus_passenger_inflow, end time is "
            "smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_bus_passenger_inflow, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_bus_passenger_inflow, input end "
            "intervals exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
        {
          result_ptr[i * l + t]
            = MNM_DTA_GRADIENT::get_link_inflow_passenger (m_link_vec_bus[i],
                                                           TFlt (start_ptr[t]),
                                                           TFlt (
                                                             end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_link_walking_passenger_inflow (py::array_t<int> start_intervals,
                                          py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_walking_passenger_inflow, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_link_walking_passenger_inflow, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int new_shape[2] = { (int) m_link_vec_walking.size (), l };
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
            "Error, Mmdta::get_link_walking_passenger_inflow, end time is "
            "smaller than or equal to start time");
        }
      if (start_ptr[t] >= get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_walking_passenger_inflow, input start "
            "intervals exceeds the total loading intervals - 1");
        }
      if (end_ptr[t] > get_cur_loading_interval ())
        {
          throw std::runtime_error (
            "Error, Mmdta::get_link_walking_passenger_inflow, input end "
            "intervals exceeds the total loading intervals");
        }
      for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
        {
          result_ptr[i * l + t] = MNM_DTA_GRADIENT::
            get_link_inflow_passenger (m_link_vec_walking[i],
                                       TFlt (start_ptr[t]),
                                       TFlt (end_ptr[t])) ();
          // printf("i %d, t %d, %f\n", i, t, result_ptr[i * l + t]);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_car_link_out_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("driving link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_out_cc, cc not installed");
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
Mmdta::get_car_link_in_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("driving link: %d\n", _link -> m_link_ID());
  if (_link->m_N_in_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_in_cc, cc not installed");
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
Mmdta::get_truck_link_out_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("driving link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_out_cc, cc not installed");
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
Mmdta::get_truck_link_in_cc (int link_ID)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("driving link: %d\n", _link -> m_link_ID());
  if (_link->m_N_in_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_in_cc, cc not installed");
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
Mmdta::get_bus_link_out_passenger_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  // printf("bus link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_out_passenger_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_out->m_recorder;
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
Mmdta::get_bus_link_in_passenger_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  // printf("bus link: %d\n", _link -> m_link_ID());
  if (_link->m_N_in == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_in_passenger_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_in->m_recorder;
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
Mmdta::get_bus_link_to_busstop_in_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  if (_link->m_to_busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_to_busstop_in_cc, busstop does not "
        "exist");
    }
  // printf("bus link: %d, busstop: %d\n", _link -> m_link_ID(), _link ->
  // m_to_busstop -> m_busstop_ID());
  if (_link->m_to_busstop->m_N_in_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_to_busstop_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = _link->m_to_busstop->m_N_in_bus->m_recorder;
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
Mmdta::get_bus_link_to_busstop_out_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  if (_link->m_to_busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_to_busstop_out_cc, busstop does not "
        "exist");
    }
  // printf("bus link: %d, busstop: %d\n", _link -> m_link_ID(), _link ->
  // m_to_busstop -> m_busstop_ID());
  if (_link->m_to_busstop->m_N_out_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_to_busstop_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = _link->m_to_busstop->m_N_out_bus->m_recorder;
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
Mmdta::get_bus_link_from_busstop_in_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  if (_link->m_from_busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_from_busstop_in_cc, busstop does not "
        "exist");
    }
  // printf("bus link: %d, busstop: %d\n", _link -> m_link_ID(), _link ->
  // m_from_busstop -> m_busstop_ID());
  if (_link->m_from_busstop->m_N_in_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_from_busstop_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = _link->m_from_busstop->m_N_in_bus->m_recorder;
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
Mmdta::get_bus_link_from_busstop_out_cc (int link_ID)
{
  MNM_Bus_Link *_link
    = (MNM_Bus_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  if (_link->m_from_busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_from_busstop_out_cc, busstop does not "
        "exist");
    }
  // printf("bus link: %d, busstop: %d\n", _link -> m_link_ID(), _link ->
  // m_from_busstop -> m_busstop_ID());
  if (_link->m_from_busstop->m_N_out_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_link_from_busstop_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record
    = _link->m_from_busstop->m_N_out_bus->m_recorder;
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
Mmdta::get_walking_link_out_cc (int link_ID)
{
  MNM_Walking_Link *_link
    = (MNM_Walking_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  // printf("walking link: %d\n", _link -> m_link_ID());
  if (_link->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_walking_link_out_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_out->m_recorder;
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
Mmdta::get_walking_link_in_cc (int link_ID)
{
  MNM_Walking_Link *_link
    = (MNM_Walking_Link *) m_mmdta->m_transitlink_factory->get_transit_link (
      TInt (link_ID));
  // printf("walking link: %d\n", _link -> m_link_ID());
  if (_link->m_N_in == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_walking_link_in_cc, cc not installed");
    }
  std::deque<std::pair<TFlt, TFlt>> _record = _link->m_N_in->m_recorder;
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
Mmdta::get_waiting_time_at_intersections ()
{
  int new_shape[1] = { (int) m_link_vec_driving.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection (
          m_link_vec_driving[i]) (); // seconds
    }
  return result;
}

py::array_t<double>
Mmdta::get_waiting_time_at_intersections_car ()
{
  int new_shape[1] = { (int) m_link_vec_driving.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection_car (
          m_link_vec_driving[i]) (); // seconds
    }
  return result;
}

py::array_t<double>
Mmdta::get_waiting_time_at_intersections_truck ()
{
  int new_shape[1] = { (int) m_link_vec_driving.size () };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_average_waiting_time_at_intersection_truck (
          m_link_vec_driving[i]) (); // seconds
    }
  return result;
}

py::array_t<int>
Mmdta::get_link_spillback ()
{
  int new_shape[1] = { (int) m_link_vec_driving.size () };
  auto result = py::array_t<int> (new_shape);
  auto result_buf = result.request ();
  int *result_ptr = (int *) result_buf.ptr;
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      result_ptr[i]
        = MNM_DTA_GRADIENT::get_is_spillback (m_link_vec_driving[i]) ();
    }
  return result;
}

py::array_t<double>
Mmdta::get_enroute_and_queue_veh_stats_agg ()
{
  int _tot_interval = get_cur_loading_interval ();
  int new_shape[2] = { _tot_interval, 3 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  if ((int) m_mmdta->m_enroute_veh_num.size () != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_enroute_and_queue_veh_stats_agg, enroute "
        "vehicle missed for some intervals");
    }
  else if ((int) m_mmdta->m_queue_veh_num.size ()
           != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_enroute_and_queue_veh_stats_agg, queuing "
        "vehicle missed for some intervals");
    }
  else
    {
      for (int i = 0; i < _tot_interval; ++i)
        {
          result_ptr[i * 3]
            = (m_mmdta->m_enroute_veh_num[i]()) / (m_mmdta->m_flow_scalar);
          result_ptr[i * 3 + 1]
            = (m_mmdta->m_queue_veh_num[i]()) / (m_mmdta->m_flow_scalar);
          result_ptr[i * 3 + 2]
            = (m_mmdta->m_enroute_veh_num[i]() - m_mmdta->m_queue_veh_num[i]())
              / (m_mmdta->m_flow_scalar);
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_enroute_and_queue_passenger_stats_agg ()
{
  int _tot_interval = get_cur_loading_interval ();
  int new_shape[2] = { _tot_interval, 3 };
  auto result = py::array_t<double> (new_shape);
  auto result_buf = result.request ();
  double *result_ptr = (double *) result_buf.ptr;

  if ((int) m_mmdta->m_enroute_passenger_num.size ()
      != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_enroute_and_queue_passenger_stats_agg, enroute "
        "passenger missed for some intervals");
    }
  else if ((int) m_mmdta->m_enroute_passenger_num.size ()
           != get_cur_loading_interval ())
    {
      throw std::runtime_error (
        "Error, Mmdta::get_enroute_and_queue_passenger_stats_agg, queuing "
        "passenger missed for some intervals");
    }
  else
    {
      for (int i = 0; i < _tot_interval; ++i)
        {
          // result_ptr[i * 3] =  m_mmdta -> m_enroute_passenger_num[i]();
          // result_ptr[i * 3 + 1] =  m_mmdta -> m_queue_passenger_num[i]();
          // result_ptr[i * 3 + 2] =  m_mmdta -> m_enroute_passenger_num[i]() -
          // m_mmdta -> m_queue_passenger_num[i]();

          // add flow_scalar to passenger
          result_ptr[i * 3]
            = m_mmdta->m_enroute_passenger_num[i]() / m_mmdta->m_flow_scalar;
          result_ptr[i * 3 + 1]
            = m_mmdta->m_queue_passenger_num[i]() / m_mmdta->m_flow_scalar;
          result_ptr[i * 3 + 2] = (m_mmdta->m_enroute_passenger_num[i]()
                                   - m_mmdta->m_queue_passenger_num[i]())
                                  / m_mmdta->m_flow_scalar;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_queue_veh_each_link (py::array_t<int> useful_links,
                                py::array_t<int> intervals)
{
  auto intervals_buf = intervals.request ();
  if (intervals_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mmdta::get_queue_veh_each_link, "
                                "input (intervals) dimension mismatch");
    }
  auto links_buf = useful_links.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mmdta::get_queue_veh_each_link, "
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
            "Error, Mmdta::get_queue_veh_each_link, input intervals "
            "exceeds the total loading intervals - 1");
        }
      for (int i = 0; i < num_links; ++i)
        {
          if (m_mmdta->m_queue_veh_map.find (links_ptr[i])
              == m_mmdta->m_queue_veh_map.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_queue_veh_each_link, can't find link "
                "ID");
            }
          result_ptr[i * num_intervals + t]
            = (*(m_mmdta->m_queue_veh_map[links_ptr[i]]))[intervals_ptr[t]]
              / m_mmdta->m_flow_scalar;
        }
    }
  return result;
}

py::array_t<double>
Mmdta::get_queue_passenger_each_link (py::array_t<int> useful_links,
                                      py::array_t<int> intervals)
{
  auto intervals_buf = intervals.request ();
  if (intervals_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_queue_passenger_each_link, input (intervals) "
        "dimension mismatch");
    }
  auto links_buf = useful_links.request ();
  if (links_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_queue_passenger_each_link, input (useful_links) "
        "dimension mismatch");
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
            "Error, Mmdta::get_queue_passenger_each_link, input intervals "
            "exceeds the total loading intervals - 1");
        }
      for (int i = 0; i < num_links; ++i)
        {
          if (m_mmdta->m_queue_passenger_map.find (links_ptr[i])
              == m_mmdta->m_queue_passenger_map.end ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_queue_passenger_each_link, can't find "
                "link ID");
            }
          // result_ptr[i * num_intervals + t] = (*(m_mmdta ->
          // m_queue_passenger_map[links_ptr[i]]))[intervals_ptr[t]]; add
          // flow_scalar to passenger
          result_ptr[i * num_intervals + t]
            = (*(
                m_mmdta->m_queue_passenger_map[links_ptr[i]]))[intervals_ptr[t]]
              / m_mmdta->m_flow_scalar;
        }
    }
  return result;
}

double
Mmdta::get_car_link_out_num (int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  // printf("%d\n", _link -> m_link_ID());
  if (_link->m_N_out_car == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_link_out_num, cc not installed");
    }
  // printf("1\n");
  TFlt result = _link->m_N_out_car->get_result (TFlt (time));
  // printf("%lf\n", result());
  return result ();
}

double
Mmdta::get_truck_link_out_num (int link_ID, double time)
{
  MNM_Dlink_Multiclass *_link
    = (MNM_Dlink_Multiclass *) m_mmdta->m_link_factory->get_link (
      TInt (link_ID));
  if (_link->m_N_out_truck == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_link_out_num, cc not installed");
    }
  TFlt result = _link->m_N_out_truck->get_result (TFlt (time));
  return result ();
}

double
Mmdta::get_passenger_link_out_num (int link_ID, double time)
{
  MNM_Transit_Link *_link
    = m_mmdta->m_transitlink_factory->get_transit_link (TInt (link_ID));
  if (_link->m_N_out == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_link_out_num, cc not installed");
    }
  TFlt result = _link->m_N_out->get_result (TFlt (time));
  return result ();
}

double
Mmdta::get_bus_stop_arrival_num (int busstop_ID, double time)
{
  MNM_Busstop_Virtual *_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
    m_mmdta->m_busstop_factory->get_busstop (TInt (busstop_ID)));
  if (_busstop == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_stop_arrival_num, virtual busstop invalid");
    }
  if (_busstop->m_N_in_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_stop_arrival_num, cc not installed");
    }
  TFlt result = _busstop->m_N_in_bus->get_result (TFlt (time));
  return result ();
}

double
Mmdta::get_bus_stop_departure_num (int busstop_ID, double time)
{
  MNM_Busstop_Virtual *_busstop = dynamic_cast<MNM_Busstop_Virtual *> (
    m_mmdta->m_busstop_factory->get_busstop (TInt (busstop_ID)));
  if (_busstop == nullptr)
    {
      throw std::runtime_error ("Error, Mmdta::get_bus_stop_departure_num, "
                                "virtual busstop invalid");
    }
  if (_busstop->m_N_out_bus == nullptr)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_stop_departure_num, cc not installed");
    }
  TFlt result = _busstop->m_N_out_bus->get_result (TFlt (time));
  return result ();
}

py::array_t<double>
Mmdta::get_car_dar_matrix_driving (py::array_t<int> start_intervals,
                                   py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mmdta::get_car_dar_matrix_driving, "
                                "input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_dar_matrix_driving, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
  //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      // printf("Current processing time: %d\n", t);
      std::cout << "************ Car Driving DAR link "
                << m_link_vec_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_driving, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_driving, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_driving, input end "
                "intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_car(_record,
          // m_link_vec_driving[i], m_path_set_driving, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_car (_record, m_link_vec_driving[i],
                                                 m_pathID_set_driving,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_truck_dar_matrix_driving (py::array_t<int> start_intervals,
                                     py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_dar_matrix_driving, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_dar_matrix_driving, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
  //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      std::cout << "************ Truck Driving DAR link "
                << m_link_vec_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_driving, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_driving, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_driving, input end "
                "intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_truck(_record,
          // m_link_vec_driving[i], m_path_set_driving, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_truck (_record,
                                                   m_link_vec_driving[i],
                                                   m_pathID_set_driving,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_car_dar_matrix_pnr (py::array_t<int> start_intervals,
                               py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_dar_matrix_pnr, input dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_dar_matrix_pnr, input length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
  //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      // printf("Current processing time: %d\n", t);
      std::cout << "************ Car PnR DAR link "
                << m_link_vec_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_pnr, end time is smaller "
                "than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_pnr, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_pnr, input end intervals "
                "exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_car(_record,
          // m_link_vec_driving[i], m_path_set_pnr, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_car (_record, m_link_vec_driving[i],
                                                 m_pathID_set_pnr,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_bus_dar_matrix_bustransit_link (py::array_t<int> start_intervals,
                                           py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_dar_matrix_bustransit_link, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_dar_matrix_bustransit_link, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
  //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      std::cout << "************ Bus Transit DAR link "
                << m_link_vec_bus[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_bustransit_link, end "
                "time smaller than start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_bustransit_link, input "
                "start intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_bustransit_link, input "
                "end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_bus(_record, m_link_vec_bus[i],
          // m_path_set_bus, TFlt(start_ptr[t]), TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_bus (_record, m_link_vec_bus[i],
                                                 m_pathID_set_bus,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_bus_dar_matrix_driving_link (py::array_t<int> start_intervals,
                                        py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_dar_matrix_driving_link, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_bus_dar_matrix_driving_link, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  for (size_t i = 0; i < m_link_vec_driving.size (); ++i)
    {
      std::cout << "************ Bus Driving DAR link "
                << m_link_vec_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_driving_link, end time "
                "is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_driving_link, input "
                "start intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_bus_dar_matrix_driving_link, input end "
                "intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_truck(_record,
          // m_link_vec_driving[i], m_path_set_bus, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_truck (_record,
                                                   m_link_vec_driving[i],
                                                   m_pathID_set_bus,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_passenger_dar_matrix_bustransit (py::array_t<int> start_intervals,
                                            py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_bustransit, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_bustransit, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
  //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
  // }
  // for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
  //   m_link_vec_walking[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      std::cout << "************ Passenger Bus Transit DAR bus link "
                << m_link_vec_bus[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, input "
                "start intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, input "
                "end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_bus[i], m_path_set_bustransit, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_bus[i],
                                                       m_pathID_set_bustransit,
                                                       TFlt (start_ptr[t]),
                                                       TFlt (end_ptr[t]));
        }
    }
  for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
    {
      std::cout << "************ Passenger Bus Transit DAR walking link "
                << m_link_vec_walking[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, input "
                "start intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_bustransit, input "
                "end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_walking[i], m_path_set_bustransit, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_walking[i],
                                                       m_pathID_set_bustransit,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_passenger_dar_matrix_pnr (py::array_t<int> start_intervals,
                                     py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_pnr, input dimension "
        "mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_pnr, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
  //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
  // }
  // for (size_t i = 0; i<m_link_vec_walking.size(); ++i){
  //   m_link_vec_walking[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      std::cout << "************ Passenger PnR DAR bus link "
                << m_link_vec_bus[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, input end "
                "intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_bus[i], m_path_set_pnr, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_bus[i],
                                                       m_pathID_set_pnr,
                                                       TFlt (start_ptr[t]),
                                                       TFlt (end_ptr[t]));
        }
    }
  for (size_t i = 0; i < m_link_vec_walking.size (); ++i)
    {
      std::cout << "************ Passenger PnR DAR walking link "
                << m_link_vec_walking[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, end time is "
                "smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, input start "
                "intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr, input end "
                "intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_walking[i], m_path_set_pnr, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_walking[i],
                                                       m_pathID_set_pnr,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_car_dar_matrix_bus_driving_link (py::array_t<int> start_intervals,
                                            py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_dar_matrix_bus_driving_link, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_dar_matrix_bus_driving_link, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
  //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
    {
      std::cout << "************ Car Driving DAR bus driving link "
                << m_link_vec_bus_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_bus_driving_link, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_bus_driving_link, input "
                "start intervals exceeds the total loading intervals - 1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_car_dar_matrix_bus_driving_link, input "
                "end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_car(_record,
          // m_link_vec_bus_driving[i], m_path_set_driving, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_car (_record,
                                                 m_link_vec_bus_driving[i],
                                                 m_pathID_set_driving,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_truck_dar_matrix_bus_driving_link (py::array_t<int> start_intervals,
                                              py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_dar_matrix_bus_driving_link, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_dar_matrix_bus_driving_link, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_driving.size(); ++i){
  //   m_link_vec_driving[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus_driving.size (); ++i)
    {
      std::cout << "************ Truck Driving DAR bus driving link "
                << m_link_vec_bus_driving[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_bus_driving_link, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_bus_driving_link, "
                "input start intervals exceeds the total loading intervals - "
                "1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_truck_dar_matrix_bus_driving_link, "
                "input end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_truck(_record,
          // m_link_vec_bus_driving[i], m_path_set_driving, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_truck (_record,
                                                   m_link_vec_bus_driving[i],
                                                   m_pathID_set_driving,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_passenger_dar_matrix_bustransit_bus_link (
  py::array_t<int> start_intervals, py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_bustransit_bus_link, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_bustransit_bus_link, input "
        "length mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
  //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      std::cout << "************ Passenger Bus Transit DAR bus link "
                << m_link_vec_bus[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, "
                "Mmdta::get_passenger_dar_matrix_bustransit_bus_link, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, "
                "Mmdta::get_passenger_dar_matrix_bustransit_bus_link, "
                "input start intervals exceeds the total loading intervals - "
                "1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, "
                "Mmdta::get_passenger_dar_matrix_bustransit_bus_link, "
                "input end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_bus[i], m_path_set_bustransit, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_bus[i],
                                                       m_pathID_set_bustransit,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

py::array_t<double>
Mmdta::get_passenger_dar_matrix_pnr_bus_link (py::array_t<int> start_intervals,
                                              py::array_t<int> end_intervals)
{
  auto start_buf = start_intervals.request ();
  auto end_buf = end_intervals.request ();
  if (start_buf.ndim != 1 || end_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_pnr_bus_link, input "
        "dimension mismatch");
    }
  if (start_buf.shape[0] != end_buf.shape[0])
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_dar_matrix_pnr_bus_link, input length "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;
  int *end_ptr = (int *) end_buf.ptr;
  std::vector<dar_record *> _record = std::vector<dar_record *> ();
  // for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
  //   m_link_vec_bus[i] -> m_N_in_tree -> print_out();
  // }
  for (size_t i = 0; i < m_link_vec_bus.size (); ++i)
    {
      std::cout << "************ Passenger PnR DAR bus link "
                << m_link_vec_bus[i]->m_link_ID () << " ************\n";
      for (int t = 0; t < l; ++t)
        {
          if (end_ptr[t] <= start_ptr[t])
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr_bus_link, end "
                "time is smaller than or equal to start time");
            }
          if (start_ptr[t] >= get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr_bus_link, "
                "input start intervals exceeds the total loading intervals - "
                "1");
            }
          if (end_ptr[t] > get_cur_loading_interval ())
            {
              throw std::runtime_error (
                "Error, Mmdta::get_passenger_dar_matrix_pnr_bus_link, "
                "input end intervals exceeds the total loading intervals");
            }
          // MNM_DTA_GRADIENT::add_dar_records_passenger(_record,
          // m_link_vec_bus[i], m_path_set_pnr, TFlt(start_ptr[t]),
          // TFlt(end_ptr[t]));
          MNM_DTA_GRADIENT::add_dar_records_passenger (_record,
                                                       m_link_vec_bus[i],
                                                       m_pathID_set_pnr,
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
      //        printf("path ID: %f, departure assign interval (1 min): %f, link
      //        ID: %f, time interval (5 s): %f, flow: %f\n",
      //               result_ptr[i * 5 + 0], result_ptr[i * 5 + 1],
      //               result_ptr[i * 5 + 2], result_ptr[i * 5 + 3],
      //               result_ptr[i * 5 + 4]);
    }
  for (size_t i = 0; i < _record.size (); ++i)
    {
      delete _record[i];
    }
  _record.clear ();
  return result;
}

// py::array_t<double>
// Mmdta::get_passenger_bus_link_flow_relationship(py::array_t<int>start_intervals,
// py::array_t<int>end_intervals)
// {
//     auto start_buf = start_intervals.request();
//     auto end_buf = end_intervals.request();
//     if (start_buf.ndim != 1 || end_buf.ndim != 1){
//         throw std::runtime_error("Error,
//         Mmdta::get_passenger_bus_link_flow_relationship, input dimension
//         mismatch");
//     }
//     if (start_buf.shape[0] != end_buf.shape[0]){
//         throw std::runtime_error("Error,
//         Mmdta::get_passenger_bus_link_flow_relationship, input length
//         mismatch");
//     }
//     int l = start_buf.shape[0];
//     int *start_ptr = (int *) start_buf.ptr;
//     int *end_ptr = (int *) end_buf.ptr;

//     int new_shape [1] = { (int) m_link_vec_bus.size() * l};
//     auto result = py::array_t<double>(new_shape);
//     auto result_buf = result.request();
//     double *result_ptr = (double *) result_buf.ptr;

//     double _small_number = 1e-4;
//     for (int t = 0; t < l; ++t){
//         if (end_ptr[t] <= start_ptr[t]){
//             throw std::runtime_error("Error,
//             Mmdta::get_passenger_bus_link_flow_relationship, end time is
//             smaller than or equal to start time");
//         }
//         if (start_ptr[t] >= get_cur_loading_interval()){
//             throw std::runtime_error("Error,
//             Mmdta::get_passenger_bus_link_flow_relationship, input start
//             intervals exceeds the total loading intervals - 1");
//         }
//         if (end_ptr[t] > get_cur_loading_interval()){
//             throw std::runtime_error("Error,
//             Mmdta::get_passenger_bus_link_flow_relationship, input end
//             intervals exceeds the total loading intervals");
//         }
//         for (size_t i = 0; i<m_link_vec_bus.size(); ++i){
//             TFlt _passenger_count =
//             MNM_DTA_GRADIENT::get_link_inflow_passenger(m_link_vec_bus[i],
//             TFlt(start_ptr[t]), TFlt(end_ptr[t])); TFlt _bus_count =
//             MNM_DTA_GRADIENT::get_link_inflow_bus(m_link_vec_bus[i],
//             TFlt(start_ptr[t]), TFlt(end_ptr[t])); if (_passenger_count <=
//             _small_number) {
//                 // _passenger_count = _small_number;
//                 result_ptr[i + t*l] = (double)1.;
//             }
//             else if (_bus_count <= _small_number) {
//                 // _passenger_count = _small_number;
//                 result_ptr[i + t*l] = (double)0.;
//             }
//             else {
//                 result_ptr[i + t*l] = (double)_bus_count / _passenger_count;
//             }
//         }
//     }
//     return result;
// }

py::array_t<double>
Mmdta::get_car_ltg_matrix_driving (py::array_t<int> start_intervals,
                                   int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mmdta::build_link_cost_map() and
  // Mmdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error ("Error, Mmdta::get_car_ltg_matrix_driving, "
                                "input dimension mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec_driving)
    {
      // check if the path does not include any link in m_link_vec_driving
      _flg = false;
      for (auto *_link : m_link_vec_driving)
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
                "Error, Mmdta::get_car_ltg_matrix_driving, input start "
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
                    m_mmdue
                      ->m_link_tt_map[_link_ID]
                                     [_t_arrival < get_cur_loading_interval ()
                                        ? _t_arrival
                                        : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mmdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multimodal *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
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
                    m_mmdue
                      ->m_link_tt_map[_link_ID]
                                     [_t_arrival_lift_up
                                          < get_cur_loading_interval ()
                                        ? _t_arrival_lift_up
                                        : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mmdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_car ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mmdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_car() / m_mmdue -> m_unit_time;
              _t_depart_lift_up
                = m_mmdue
                    ->m_queue_dissipated_time_car[_link_ID][_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_mmdue->m_link_congested_car[_link_ID][_t_arrival_lift_up])
                {
                  if (m_mmdue->m_queue_dissipated_time_car[_link_ID]
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
                      IAssert (
                        m_mmdue->m_queue_dissipated_time_car[_link_ID]
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
                    m_mmdue->m_queue_dissipated_time_car[_link_ID]
                                                        [_t_arrival_lift_up]
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
                  && std::find_if (m_link_vec_driving.begin (),
                                   m_link_vec_driving.end (),
                                   [&_link_ID] (
                                     const MNM_Dlink_Multiclass *_l) {
                                     return _l->m_link_ID == _link_ID;
                                   })
                       != m_link_vec_driving.end ())
                {
                  if (m_mmdue
                        ->m_link_congested_car[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time > 0);
                      if (m_mmdue
                            ->m_queue_dissipated_time_car[_link_ID]
                                                         [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_mmdue->m_queue_dissipated_time_car
                                [_link_ID][_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_mmdue->m_link_tt_map
                                  [_link_ID]
                                  [_t_queue_dissipated_valid - 1
                                       < get_cur_loading_interval ()
                                     ? _t_queue_dissipated_valid - 1
                                     : get_cur_loading_interval () - 1]);
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout
                            << "\nError, Mmdta::get_car_ltg_matrix_driving"
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
                          std::cout << "m_mmdue -> "
                                       "m_queue_dissipated_time_car[_link_ID][_"
                                       "t_arrival_lift_up]: "
                                    << m_mmdue->m_queue_dissipated_time_car
                                         [_link_ID][_t_arrival_lift_up]
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
                            << "m_mmdue -> "
                               "m_link_tt_map[_link_ID][_t_queue_dissipated_"
                               "valid]: "
                            << m_mmdue->m_link_tt_map
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
                            = MNM_DTA_GRADIENT::get_departure_cc_slope_car (
                              _link, TFlt (_t_depart_prime),
                              TFlt (_t_depart_lift_up_valid + 1));
                          if (_gradient > DBL_EPSILON)
                            {
                              _gradient
                                = m_mmdue->m_unit_time / _gradient; // seconds
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
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_tt_map[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mmdue -> m_unit_time /
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
Mmdta::get_car_ltg_matrix_pnr (py::array_t<int> start_intervals,
                               int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mmdta::build_link_cost_map() and
  // Mmdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_car_ltg_matrix_pnr, input dimension mismatch");
    }
  // int l = start_buf.shape[0];
  // int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  // bool _flg;
  // TFlt _fftt, _gradient;
  // int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
  // _cost_map_index;

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
Mmdta::get_truck_ltg_matrix_driving (py::array_t<int> start_intervals,
                                     int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mmdta::build_link_cost_map() and
  // Mmdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_truck_ltg_matrix_driving, input dimension "
        "mismatch");
    }
  int l = start_buf.shape[0];
  int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  bool _flg;
  TFlt _fftt, _gradient;
  int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
    _t_depart_prime, _t_queue_dissipated_valid, _t_depart_lift_up_valid;
  for (auto *_path : m_path_vec_driving)
    {
      // check if the path does not include any link in m_link_vec_driving
      _flg = false;
      for (auto *_link : m_link_vec_driving)
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
                "Error, Mmdta::get_truck_ltg_matrix_driving, input start "
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
                    m_mmdue->m_link_tt_map_truck
                      [_link_ID][_t_arrival < get_cur_loading_interval ()
                                   ? _t_arrival
                                   : get_cur_loading_interval () - 1]);

              // arrival time of the new perturbation vehicle
              auto *_link = dynamic_cast<MNM_Dlink_Multiclass *> (
                m_mmdta->m_link_factory->get_link (_link_ID));
              if (dynamic_cast<MNM_Dlink_Pq_Multimodal *> (_link) != nullptr)
                {
                  _t_arrival_lift_up
                    = _t_arrival; // for last pq, _t_arrival_lift_up >=
                                  // get_cur_loading_interval()
                }
              else
                {
                  IAssert (dynamic_cast<MNM_Dlink_Ctm_Multimodal *> (_link)
                           != nullptr);
                  IAssert (_t_depart_lift_up >= 0); // from its upstream link
                  _t_arrival_lift_up = _t_depart_lift_up;
                }
              IAssert (_t_arrival_lift_up >= _t_arrival);

              IAssert (_link->m_last_valid_time_truck > 0);
              if (_t_arrival_lift_up
                    > int (round (_link->m_last_valid_time_truck - 1))
                  || _t_arrival_lift_up >= threshold_timestamp)
                {
                  break;
                }

              // departure time of new perturbation vehicle
              _t_depart_prime
                = _t_arrival_lift_up
                  + MNM_Ults::round_up_time (
                    m_mmdue->m_link_tt_map_truck
                      [_link_ID]
                      [_t_arrival_lift_up < get_cur_loading_interval ()
                         ? _t_arrival_lift_up
                         : get_cur_loading_interval () - 1]);

              // arrival time of the NEXT new perturbation for the NEXT link
              _fftt = dynamic_cast<MNM_Dlink_Multiclass *> (
                        m_mmdta->m_link_factory->get_link (_link_ID))
                        ->get_link_freeflow_tt_loading_truck ();
              // _fftt = dynamic_cast<MNM_Dlink_Ctm_Multiclass*>(m_mmdta ->
              // m_link_factory -> get_link(_link_ID)) ->
              // get_link_freeflow_tt_truck() / m_mmdue -> m_unit_time;
              _t_depart_lift_up
                = m_mmdue->m_queue_dissipated_time_truck[_link_ID]
                                                        [_t_arrival_lift_up]
                  + MNM_Ults::round_up_time (_fftt);

              if (!m_mmdue
                     ->m_link_congested_truck[_link_ID][_t_arrival_lift_up])
                {
                  if (m_mmdue->m_queue_dissipated_time_truck[_link_ID]
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
                      IAssert (
                        m_mmdue
                          ->m_queue_dissipated_time_truck[_link_ID]
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
                    m_mmdue->m_queue_dissipated_time_truck[_link_ID]
                                                          [_t_arrival_lift_up]
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
                  && std::find_if (m_link_vec_driving.begin (),
                                   m_link_vec_driving.end (),
                                   [&_link_ID] (
                                     const MNM_Dlink_Multiclass *_l) {
                                     return _l->m_link_ID == _link_ID;
                                   })
                       != m_link_vec_driving.end ())
                {
                  if (m_mmdue
                        ->m_link_congested_truck[_link_ID][_t_arrival_lift_up]
                      && _t_depart_lift_up > _t_depart_prime)
                    {
                      IAssert (_link->m_last_valid_time_truck > 0);
                      if (m_mmdue
                            ->m_queue_dissipated_time_truck[_link_ID]
                                                           [_t_arrival_lift_up]
                          <= int (round (_link->m_last_valid_time_truck - 1)))
                        {
                          _t_queue_dissipated_valid
                            = m_mmdue->m_queue_dissipated_time_truck
                                [_link_ID][_t_arrival_lift_up];
                          _t_depart_lift_up_valid = _t_depart_lift_up;
                        }
                      else
                        {
                          _t_queue_dissipated_valid
                            = int (round (_link->m_last_valid_time_truck));
                          _t_depart_lift_up_valid
                            = _t_queue_dissipated_valid - 1
                              + MNM_Ults::round_up_time (
                                m_mmdue->m_link_tt_map_truck
                                  [_link_ID]
                                  [_t_queue_dissipated_valid - 1
                                       < get_cur_loading_interval ()
                                     ? _t_queue_dissipated_valid - 1
                                     : get_cur_loading_interval () - 1]);
                        }
                      IAssert (_t_depart_lift_up_valid
                               <= get_cur_loading_interval () - 1);
                      IAssert (_t_arrival_lift_up < _t_queue_dissipated_valid);
                      if (_t_depart_prime > _t_depart_lift_up_valid)
                        {
                          std::cout << "\nError, "
                                       "Mmdta::get_truck_ltg_matrix_driving"
                                    << "\n";
                          std::cout << "interval: " << start_ptr[t]
                                    << ", link: " << _link_ID << "\n";
                          std::cout << "car in"
                                    << "\n";
                          std::cout << _link->m_N_in_truck->to_string ()
                                    << "\n";
                          std::cout << "car out"
                                    << "\n";
                          std::cout << _link->m_N_out_truck->to_string ()
                                    << "\n\n";
                          std::cout << "last valid time: "
                                    << _link->m_last_valid_time_truck << "\n";
                          std::cout << "_t_arrival: " << _t_arrival << "\n";
                          std::cout << "_t_depart: " << _t_depart << "\n";
                          std::cout
                            << "_t_arrival_lift_up: " << _t_arrival_lift_up
                            << "\n";
                          std::cout << "_t_depart_prime: " << _t_depart_prime
                                    << "\n";
                          std::cout << "m_mmdue -> "
                                       "m_queue_dissipated_time_truck[_link_ID]"
                                       "[_t_arrival_lift_up]: "
                                    << m_mmdue->m_queue_dissipated_time_truck
                                         [_link_ID][_t_arrival_lift_up]
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
                            << "m_mmdue -> "
                               "m_link_tt_map_truck[_link_ID][_t_queue_"
                               "dissipated_valid]: "
                            << m_mmdue->m_link_tt_map_truck
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
                                = m_mmdue->m_unit_time / _gradient; // seconds
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
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1])),
                          //                                                             TFlt(t_prime + MNM_Ults::round_up_time(m_mmdue -> m_link_tt_map_truck[_link_ID][t_prime < get_cur_loading_interval() ? t_prime : get_cur_loading_interval() - 1]) + 1)
                          //                                                             );
                          //     if (_gradient > DBL_EPSILON) {
                          //         _gradient = m_mmdue -> m_unit_time /
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
Mmdta::get_passenger_ltg_matrix_bustransit (py::array_t<int> start_intervals,
                                            int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mmdta::build_link_cost_map() and
  // Mmdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_ltg_matrix_bustransit, input "
        "dimension mismatch");
    }
  // int l = start_buf.shape[0];
  // int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  // bool _flg;
  // TFlt _fftt, _gradient;
  // int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
  // _cost_map_index;

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
Mmdta::get_passenger_ltg_matrix_pnr (py::array_t<int> start_intervals,
                                     int threshold_timestamp)
{
  // input: intervals in which the agents are released for each path, 1 min
  // interval = 12 5-s intervals assume Mmdta::build_link_cost_map() and
  // Mmdta::get_link_queue_dissipated_time() are invoked already
  auto start_buf = start_intervals.request ();
  if (start_buf.ndim != 1)
    {
      throw std::runtime_error (
        "Error, Mmdta::get_passenger_ltg_matrix_pnr, input dimension "
        "mismatch");
    }
  // int l = start_buf.shape[0];
  // int *start_ptr = (int *) start_buf.ptr;

  std::vector<ltg_record *> _record = std::vector<ltg_record *> ();
  // bool _flg;
  // TFlt _fftt, _gradient;
  // int _t_arrival, _t_depart, _t_arrival_lift_up, _t_depart_lift_up,
  // _cost_map_index;

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

int
Mmdta::delete_all_agents ()
{
  // invoke it after simulation, may help save some memory
  delete m_mmdta->m_veh_factory;
  delete m_mmdta->m_passenger_factory;
  m_mmdta->m_veh_factory = nullptr;
  m_mmdta->m_passenger_factory = nullptr;
  return 0;
}

//
// Entry point
//

PYBIND11_MODULE (_macposts_ext, m)
{
  m.def ("set_random_state", &MNM_Ults::set_random_state,
         R"pbdoc(Set the random state.

Note that this only affects the random number generator used during network
simulation.

)pbdoc");

  py::class_<Tdsp> (m, "Tdsp")
    .def (py::init<> ())
    .def ("initialize", &Tdsp::initialize)
    .def ("read_td_cost_txt", &Tdsp::read_td_cost_txt)
    .def ("read_td_cost_py", &Tdsp::read_td_cost_py)
    .def ("build_tdsp_tree", &Tdsp::build_tdsp_tree)
    .def ("extract_tdsp", &Tdsp::extract_tdsp);

  py::class_<Dta> (m, "Dta")
    .def (py::init<> ())
    .def ("initialize", &Dta::initialize)
    .def ("check_input_files", &Dta::check_input_files)
    .def ("run_whole", &Dta::run_whole, py::arg ("verbose") = false)
    .def ("run_due", &Dta::run_due)
    .def ("run_dso", &Dta::run_dso)
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

  py::class_<Mmdta> (m, "Mmdta")
    .def (py::init<> ())
    .def ("initialize", &Mmdta::initialize)
    .def ("run_whole", &Mmdta::run_whole, py::arg ("verbose") = false)
    .def ("initialize_mmdue", &Mmdta::initialize_mmdue)
    .def ("generate_shortest_pathsets", &Mmdta::generate_shortest_pathsets)
    .def ("check_input_files", &Mmdta::check_input_files)
    .def ("run_mmdue", &Mmdta::run_mmdue)
    .def ("run_mmdta_adaptive", &Mmdta::run_mmdta_adaptive)
    .def ("install_cc", &Mmdta::install_cc)
    .def ("install_cc_tree", &Mmdta::install_cc_tree)
    .def ("get_travel_stats", &Mmdta::get_travel_stats)
    .def ("print_emission_stats", &Mmdta::print_emission_stats)
    .def ("get_cur_loading_interval", &Mmdta::get_cur_loading_interval)
    .def ("print_simulation_results", &Mmdta::print_simulation_results)

    .def ("register_links_driving", &Mmdta::register_links_driving)
    .def ("register_links_bus", &Mmdta::register_links_bus)
    .def ("register_links_walking", &Mmdta::register_links_walking)

    .def ("get_links_overlapped_bus_driving",
          &Mmdta::get_links_overlapped_bus_driving)
    .def ("register_links_bus_driving", &Mmdta::register_links_bus_driving)

    .def ("register_paths", &Mmdta::register_paths)
    .def ("register_paths_driving", &Mmdta::register_paths_driving)
    .def ("register_paths_bustransit", &Mmdta::register_paths_bustransit)
    .def ("register_paths_pnr", &Mmdta::register_paths_pnr)
    .def ("register_paths_bus", &Mmdta::register_paths_bus)

    .def ("are_registered_links_in_registered_paths_driving",
          &Mmdta::are_registered_links_in_registered_paths_driving)
    .def ("are_registered_links_in_registered_paths_bus",
          &Mmdta::are_registered_links_in_registered_paths_bus)
    .def ("are_registered_links_in_registered_paths_walking",
          &Mmdta::are_registered_links_in_registered_paths_walking)

    .def ("generate_paths_to_cover_registered_links_driving",
          &Mmdta::generate_paths_to_cover_registered_links_driving)
    .def ("generate_paths_to_cover_registered_links_bus_walking",
          &Mmdta::generate_paths_to_cover_registered_links_bus_walking)

    .def ("get_od_mode_connectivity", &Mmdta::get_od_mode_connectivity)
    .def ("generate_init_mode_demand_file",
          &Mmdta::generate_init_mode_demand_file)

    .def ("get_car_link_fftt", &Mmdta::get_car_link_fftt)
    .def ("get_truck_link_fftt", &Mmdta::get_truck_link_fftt)
    .def ("get_bus_link_fftt", &Mmdta::get_bus_link_fftt)
    .def ("get_walking_link_fftt", &Mmdta::get_walking_link_fftt)

    .def ("get_car_link_tt", &Mmdta::get_car_link_tt)
    .def ("get_car_link_tt_robust", &Mmdta::get_car_link_tt_robust)
    .def ("get_truck_link_tt", &Mmdta::get_truck_link_tt)
    .def ("get_truck_link_tt_robust", &Mmdta::get_truck_link_tt_robust)
    .def ("get_bus_link_tt", &Mmdta::get_bus_link_tt)
    .def ("get_bus_link_tt_robust", &Mmdta::get_bus_link_tt_robust)
    .def ("get_passenger_walking_link_tt",
          &Mmdta::get_passenger_walking_link_tt)
    .def ("get_passenger_walking_link_tt_robust",
          &Mmdta::get_passenger_walking_link_tt_robust)
    .def ("get_bus_driving_link_tt_car", &Mmdta::get_bus_driving_link_tt_car)
    .def ("get_bus_driving_link_tt_car_robust",
          &Mmdta::get_bus_driving_link_tt_car_robust)
    .def ("get_bus_driving_link_tt_truck",
          &Mmdta::get_bus_driving_link_tt_truck)
    .def ("get_bus_driving_link_tt_truck_robust",
          &Mmdta::get_bus_driving_link_tt_truck_robust)

    .def ("get_link_car_inflow", &Mmdta::get_link_car_inflow)
    .def ("get_link_truck_inflow", &Mmdta::get_link_truck_inflow)
    .def ("get_link_bus_inflow", &Mmdta::get_link_bus_inflow)
    .def ("get_busstop_bus_inflow", &Mmdta::get_busstop_bus_inflow)
    .def ("get_link_bus_passenger_inflow",
          &Mmdta::get_link_bus_passenger_inflow)
    .def ("get_link_walking_passenger_inflow",
          &Mmdta::get_link_walking_passenger_inflow)

    .def ("get_car_link_out_num", &Mmdta::get_car_link_out_num)
    .def ("get_truck_link_out_num", &Mmdta::get_truck_link_out_num)
    .def ("get_passenger_link_out_num", &Mmdta::get_passenger_link_out_num)
    .def ("get_bus_stop_arrival_num", &Mmdta::get_bus_stop_arrival_num)
    .def ("get_bus_stop_departure_num", &Mmdta::get_bus_stop_departure_num)

    .def ("get_car_link_out_cc", &Mmdta::get_car_link_out_cc)
    .def ("get_car_link_in_cc", &Mmdta::get_car_link_in_cc)
    .def ("get_truck_link_out_cc", &Mmdta::get_truck_link_out_cc)
    .def ("get_truck_link_in_cc", &Mmdta::get_truck_link_in_cc)
    .def ("get_bus_link_out_passenger_cc",
          &Mmdta::get_bus_link_out_passenger_cc)
    .def ("get_bus_link_in_passenger_cc", &Mmdta::get_bus_link_in_passenger_cc)
    .def ("get_bus_link_to_busstop_in_cc",
          &Mmdta::get_bus_link_to_busstop_in_cc)
    .def ("get_bus_link_to_busstop_out_cc",
          &Mmdta::get_bus_link_to_busstop_out_cc)
    .def ("get_bus_link_from_busstop_in_cc",
          &Mmdta::get_bus_link_from_busstop_in_cc)
    .def ("get_bus_link_from_busstop_out_cc",
          &Mmdta::get_bus_link_from_busstop_out_cc)
    .def ("get_walking_link_out_cc", &Mmdta::get_walking_link_out_cc)
    .def ("get_walking_link_in_cc", &Mmdta::get_walking_link_in_cc)

    .def ("get_car_link_speed", &Mmdta::get_car_link_speed)
    .def ("get_truck_link_speed", &Mmdta::get_truck_link_speed)
    .def ("get_bus_link_speed", &Mmdta::get_bus_link_speed)

    .def ("save_passenger_path_table", &Mmdta::save_passenger_path_table)
    .def ("save_mode_path_table", &Mmdta::save_mode_path_table)

    .def ("link_seq_to_node_seq_driving", &Mmdta::link_seq_to_node_seq_driving)
    .def ("link_seq_to_node_seq_bustransit",
          &Mmdta::link_seq_to_node_seq_bustransit)
    .def ("node_seq_to_link_seq_driving", &Mmdta::node_seq_to_link_seq_driving)
    .def ("node_seq_to_link_seq_bustransit",
          &Mmdta::node_seq_to_link_seq_bustransit)

    .def ("update_tdsp_tree", &Mmdta::update_tdsp_tree)
    .def ("get_lowest_cost_path", &Mmdta::get_lowest_cost_path)

    .def ("build_link_cost_map", &Mmdta::build_link_cost_map)
    .def ("get_link_queue_dissipated_time",
          &Mmdta::get_link_queue_dissipated_time)
    .def ("build_link_cost_map_snapshot", &Mmdta::build_link_cost_map_snapshot)
    .def ("update_snapshot_route_table", &Mmdta::update_snapshot_route_table)
    .def ("get_lowest_cost_path_snapshot",
          &Mmdta::get_lowest_cost_path_snapshot)

    .def ("get_passenger_path_cost_driving",
          &Mmdta::get_passenger_path_cost_driving)
    .def ("get_passenger_path_cost_bus", &Mmdta::get_passenger_path_cost_bus)
    .def ("get_passenger_path_cost_pnr", &Mmdta::get_passenger_path_cost_pnr)

    .def ("get_path_tt_car", &Mmdta::get_path_tt_car)
    .def ("get_path_tt_truck", &Mmdta::get_path_tt_truck)
    .def ("get_registered_path_tt_truck", &Mmdta::get_registered_path_tt_truck)
    .def ("get_registered_path_tt_driving",
          &Mmdta::get_registered_path_tt_driving)
    .def ("get_registered_path_tt_bustransit",
          &Mmdta::get_registered_path_tt_bustransit)
    .def ("get_registered_path_tt_pnr", &Mmdta::get_registered_path_tt_pnr)

    .def ("get_registered_path_distance_driving",
          &Mmdta::get_registered_path_distance_driving)
    .def ("get_registered_path_distance_bustransit",
          &Mmdta::get_registered_path_distance_bustransit)
    .def ("get_registered_path_distance_pnr",
          &Mmdta::get_registered_path_distance_pnr)

    .def ("get_registered_path_cost_driving",
          &Mmdta::get_registered_path_cost_driving)
    .def ("get_registered_path_cost_bustransit",
          &Mmdta::get_registered_path_cost_bustransit)
    .def ("get_registered_path_cost_pnr", &Mmdta::get_registered_path_cost_pnr)

    .def ("get_enroute_and_queue_veh_stats_agg",
          &Mmdta::get_enroute_and_queue_veh_stats_agg)
    .def ("get_enroute_and_queue_passenger_stats_agg",
          &Mmdta::get_enroute_and_queue_passenger_stats_agg)
    .def ("get_queue_veh_each_link", &Mmdta::get_queue_veh_each_link)
    .def ("get_queue_passenger_each_link",
          &Mmdta::get_queue_passenger_each_link)

    .def ("get_car_dar_matrix_driving", &Mmdta::get_car_dar_matrix_driving)
    .def ("get_truck_dar_matrix_driving", &Mmdta::get_truck_dar_matrix_driving)
    .def ("get_car_dar_matrix_pnr", &Mmdta::get_car_dar_matrix_pnr)
    .def ("get_bus_dar_matrix_bustransit_link",
          &Mmdta::get_bus_dar_matrix_bustransit_link)
    .def ("get_bus_dar_matrix_driving_link",
          &Mmdta::get_bus_dar_matrix_driving_link)
    .def ("get_passenger_dar_matrix_bustransit",
          &Mmdta::get_passenger_dar_matrix_bustransit)
    .def ("get_passenger_dar_matrix_pnr", &Mmdta::get_passenger_dar_matrix_pnr)
    .def ("get_car_dar_matrix_bus_driving_link",
          &Mmdta::get_car_dar_matrix_bus_driving_link)
    .def ("get_truck_dar_matrix_bus_driving_link",
          &Mmdta::get_truck_dar_matrix_bus_driving_link)
    .def ("get_passenger_dar_matrix_bustransit_bus_link",
          &Mmdta::get_passenger_dar_matrix_bustransit_bus_link)
    .def ("get_passenger_dar_matrix_pnr_bus_link",
          &Mmdta::get_passenger_dar_matrix_pnr_bus_link)
    // .def("get_passenger_bus_link_flow_relationship",
    // &Mmdta::get_passenger_bus_link_flow_relationship)

    .def ("get_car_ltg_matrix_driving", &Mmdta::get_car_ltg_matrix_driving)
    .def ("get_car_ltg_matrix_pnr", &Mmdta::get_car_ltg_matrix_pnr)
    .def ("get_truck_ltg_matrix_driving", &Mmdta::get_truck_ltg_matrix_driving)
    .def ("get_passenger_ltg_matrix_bustransit",
          &Mmdta::get_passenger_ltg_matrix_bustransit)
    .def ("get_passenger_ltg_matrix_pnr", &Mmdta::get_passenger_ltg_matrix_pnr)

    .def ("get_waiting_time_at_intersections",
          &Mmdta::get_waiting_time_at_intersections)
    .def ("get_waiting_time_at_intersections_car",
          &Mmdta::get_waiting_time_at_intersections_car)
    .def ("get_waiting_time_at_intersections_truck",
          &Mmdta::get_waiting_time_at_intersections_truck)
    .def ("get_link_spillback", &Mmdta::get_link_spillback)

    .def ("delete_all_agents", &Mmdta::delete_all_agents);
}
