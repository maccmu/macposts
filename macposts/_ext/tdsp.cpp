// Time-dependent shortest path (TDSP)

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <unordered_map>
#include <vector>

#include <common.h>
#include <io.h>
#include <shortest_path.h>

namespace py = pybind11;

namespace macposts
{
namespace tdsp
{
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
  macposts::Graph m_graph;
  MNM_TDSP_Tree *m_tdsp_tree;

  std::unordered_map<TInt, TFlt *> m_td_link_tt;
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> m_td_node_tt;
  std::unordered_map<TInt, TFlt *> m_td_link_cost;
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> m_td_node_cost;
};

void
init (py::module &m)
{
  py::class_<Tdsp> (m, "Tdsp")
    .def (py::init<> ())
    .def ("initialize", &Tdsp::initialize)
    .def ("read_td_cost_txt", &Tdsp::read_td_cost_txt)
    .def ("read_td_cost_py", &Tdsp::read_td_cost_py)
    .def ("build_tdsp_tree", &Tdsp::build_tdsp_tree)
    .def ("extract_tdsp", &Tdsp::extract_tdsp);
}

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
}

Tdsp::~Tdsp ()
{
  delete m_tdsp_tree;

  for (auto _it : m_td_link_tt)
    {
      delete[] _it.second;
    }
  m_td_link_tt.clear ();
  for (auto _it : m_td_node_tt)
    {
      for (auto _it_it : _it.second)
        {
          delete[] _it_it.second;
        }
      _it.second.clear ();
    }
  m_td_node_tt.clear ();

  for (auto _it : m_td_link_cost)
    {
      delete[] _it.second;
    }
  m_td_link_cost.clear ();
  for (auto _it : m_td_node_cost)
    {
      for (auto _it_it : _it.second)
        {
          delete[] _it_it.second;
        }
      _it.second.clear ();
    }
  m_td_node_cost.clear ();
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
          double *_cost_vector_tmp = new double[m_max_interval]();
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
          double *_cost_vector_tmp = new double[m_max_interval]();
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
  Assert (m_tdsp_tree->m_dist[origin_node_ID] != nullptr);

  printf ("get travel cost to dest\n");
  tmp_cost = m_tdsp_tree
               ->m_dist[origin_node_ID][timestamp < m_tdsp_tree->m_max_interval
                                          ? timestamp
                                          : m_tdsp_tree->m_max_interval - 1];
  printf ("At time %d, minimum cost is %f\n", timestamp, tmp_cost);
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
  printf ("travel time: %f\n", _tt);
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
}
}
