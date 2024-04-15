#include "io.h"
#include <cstring>

using macposts::graph::Direction;

int
MNM_IO::build_node_factory (const std::string &file_folder,
                            MNM_ConfReader *conf_reader,
                            MNM_Node_Factory *node_factory,
                            const std::string &file_name)
{
  /* find file */
  std::string _node_file_name = file_folder + "/" + file_name;
  std::ifstream _node_file;
  _node_file.open (_node_file_name, std::ios::in);

  /* read confid */
  TInt _num_of_node = conf_reader->get_int ("num_of_node");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");

  /* read file */
  std::string _line;
  std::vector<std::string> _words;
  TInt _node_ID;
  std::string _type;

  if (_node_file.is_open ())
    {
      for (int i = 0; i < _num_of_node;)
        {
          std::getline (_node_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          // if (_words.size() == 2) {
          if (_words.size () >= 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _node_ID = TInt (std::stoi (_words[0]));
              _type = trim (_words[1]);
              if (_type == "FWJ")
                {
                  node_factory->make_node (_node_ID, MNM_TYPE_FWJ,
                                           _flow_scalar);
                  continue;
                }
              if (_type == "GRJ")
                {
                  node_factory->make_node (_node_ID, MNM_TYPE_GRJ,
                                           _flow_scalar);
                  continue;
                }
              if (_type == "DMOND")
                {
                  node_factory->make_node (_node_ID, MNM_TYPE_ORIGIN,
                                           _flow_scalar);
                  continue;
                }
              if (_type == "DMDND")
                {
                  node_factory->make_node (_node_ID, MNM_TYPE_DEST,
                                           _flow_scalar);
                  continue;
                }
              throw std::runtime_error ("unknown node type: " + _type);
            }
          else
            {
              throw std::runtime_error ("failed to parse line: " + _line);
            }
        }
      _node_file.close ();
    }
  return 0;
}

int
MNM_IO::build_link_factory (const std::string &file_folder,
                            MNM_ConfReader *conf_reader,
                            MNM_Link_Factory *link_factory,
                            const std::string &file_name)
{
  /* find file */
  std::string _link_file_name = file_folder + "/" + file_name;
  std::ifstream _link_file;
  _link_file.open (_link_file_name, std::ios::in);

  /* read config */
  TInt _num_of_link = conf_reader->get_int ("num_of_link");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TFlt _unit_time = conf_reader->get_float ("unit_time");

  /* read file */
  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;
  TFlt _lane_hold_cap;
  TFlt _lane_flow_cap;
  TInt _number_of_lane;
  TFlt _length;
  TFlt _ffs;
  std::string _type;

  if (_link_file.is_open ())
    {
      for (int i = 0; i < _num_of_link;)
        {
          std::getline (_link_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          // if (_words.size() == 7) {
          if (_words.size () >= 7)
            {
              // std::cout << "Processing: " << _line << "\n";
              _link_ID = TInt (std::stoi (_words[0]));
              _type = trim (_words[1]);
              _length = TFlt (std::stod (_words[2]));
              _ffs = TFlt (std::stod (_words[3]));
              _lane_flow_cap = TFlt (std::stod (_words[4]));
              _lane_hold_cap = TFlt (std::stod (_words[5]));
              _number_of_lane = TInt (std::stoi (_words[6]));

              /* unit conversion */
              _length = _length * TFlt (1600);
              _ffs = _ffs * TFlt (1600) / TFlt (3600);
              _lane_flow_cap = _lane_flow_cap / TFlt (3600);
              _lane_hold_cap = _lane_hold_cap / TFlt (1600);

              /* build */
              if (_type == "PQ")
                {
                  link_factory->make_link (_link_ID, MNM_TYPE_PQ,
                                           _lane_hold_cap, _lane_flow_cap,
                                           _number_of_lane, _length, _ffs,
                                           _unit_time, _flow_scalar);
                  continue;
                }
              if (_type == "CTM")
                {
                  link_factory->make_link (_link_ID, MNM_TYPE_CTM,
                                           _lane_hold_cap, _lane_flow_cap,
                                           _number_of_lane, _length, _ffs,
                                           _unit_time, _flow_scalar);
                  continue;
                }
              if (_type == "LQ")
                {
                  link_factory->make_link (_link_ID, MNM_TYPE_LQ,
                                           _lane_hold_cap, _lane_flow_cap,
                                           _number_of_lane, _length, _ffs,
                                           _unit_time, _flow_scalar);
                  continue;
                }
              if (_type == "LTM")
                {
                  link_factory->make_link (_link_ID, MNM_TYPE_LTM,
                                           _lane_hold_cap, _lane_flow_cap,
                                           _number_of_lane, _length, _ffs,
                                           _unit_time, _flow_scalar);
                  continue;
                }
              throw std::runtime_error ("unknown link type: " + _type);
            }
          else
            {
              throw std::runtime_error ("failed to parse line: " + _line);
            }
        }
      _link_file.close ();
    }
  return 0;
}

int
MNM_IO::build_od_factory (const std::string &file_folder,
                          MNM_ConfReader *conf_reader,
                          MNM_OD_Factory *od_factory,
                          MNM_Node_Factory *node_factory,
                          const std::string &file_name)
{
  /* find file */
  std::string _od_file_name = file_folder + "/" + file_name;
  std::ifstream _od_file;
  _od_file.open (_od_file_name, std::ios::in);

  /* read config */
  TInt _num_of_O = conf_reader->get_int ("num_of_O");
  TInt _num_of_D = conf_reader->get_int ("num_of_D");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _frequency = conf_reader->get_int ("assign_frq");

  /* build */
  TInt _dest_ID, _origin_ID, _node_ID;
  std::string _line;
  std::vector<std::string> _words;
  MNM_Origin *_origin;
  MNM_Destination *_dest;
  if (_od_file.is_open ())
    {
      for (int i = 0; i < _num_of_O;)
        {
          std::getline (_od_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _origin = od_factory->make_origin (_origin_ID, _max_interval,
                                                 _flow_scalar, _frequency);

              /* hook up */
              _origin->m_origin_node
                = (MNM_DMOND *) node_factory->get_node (_node_ID);
              ((MNM_DMOND *) node_factory->get_node (_node_ID))
                ->hook_up_origin (_origin);
            }
        }

      for (int i = 0; i < _num_of_D;)
        {
          std::getline (_od_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _dest_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _dest = od_factory->make_destination (_dest_ID);
              _dest->m_flow_scalar = _flow_scalar;
              /* hook up */
              _dest->m_dest_node
                = (MNM_DMDND *) node_factory->get_node (_node_ID);
              ((MNM_DMDND *) node_factory->get_node (_node_ID))
                ->hook_up_destination (_dest);
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO::hook_up_od_node (const std::string &file_folder,
                         MNM_ConfReader *conf_reader,
                         MNM_OD_Factory *od_factory,
                         MNM_Node_Factory *node_factory,
                         const std::string &file_name)
{
  /* find file */
  std::string _od_file_name = file_folder + "/" + file_name;
  std::ifstream _od_file;
  _od_file.open (_od_file_name, std::ios::in);

  /* read config */
  TInt _num_of_O = conf_reader->get_int ("num_of_O");
  TInt _num_of_D = conf_reader->get_int ("num_of_D");

  /* build */
  TInt _dest_ID, _origin_ID, _node_ID;
  std::string _line;
  std::vector<std::string> _words;
  MNM_Origin *_origin;
  MNM_Destination *_dest;
  if (_od_file.is_open ())
    {
      for (int i = 0; i < _num_of_O;)
        {
          std::getline (_od_file, _line);
          _words = split (_line, ' ');
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _origin = od_factory->get_origin (_origin_ID);

              /* hook up */
              _origin->m_origin_node
                = (MNM_DMOND *) node_factory->get_node (_node_ID);
              ((MNM_DMOND *) node_factory->get_node (_node_ID))
                ->hook_up_origin (_origin);
            }
        }

      for (int i = 0; i < _num_of_D;)
        {
          std::getline (_od_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _dest_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              _dest = od_factory->get_destination (_dest_ID);

              /* hook up */
              _dest->m_dest_node
                = (MNM_DMDND *) node_factory->get_node (_node_ID);
              ((MNM_DMDND *) node_factory->get_node (_node_ID))
                ->hook_up_destination (_dest);
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO::build_od_factory (const std::string &file_folder,
                          MNM_ConfReader *conf_reader,
                          MNM_OD_Factory *od_factory,
                          const std::string &file_name)
{
  /* find file */
  std::string _od_file_name = file_folder + "/" + file_name;
  std::ifstream _od_file;
  _od_file.open (_od_file_name, std::ios::in);

  /* read config */
  TInt _num_of_O = conf_reader->get_int ("num_of_O");
  TInt _num_of_D = conf_reader->get_int ("num_of_D");
  TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _frequency = conf_reader->get_int ("assign_frq");

  /* build */
  TInt _dest_ID, _origin_ID, _node_ID;
  std::string _line;
  std::vector<std::string> _words;
  if (_od_file.is_open ())
    {
      for (int i = 0; i < _num_of_O;)
        {
          std::getline (_od_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              od_factory->make_origin (_origin_ID, _max_interval, _flow_scalar,
                                       _frequency);
            }
        }

      for (int i = 0; i < _num_of_D;)
        {
          std::getline (_od_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (_words.size () == 2)
            {
              // std::cout << "Processing: " << _line << "\n";
              _dest_ID = TInt (std::stoi (_words[0]));
              _node_ID = TInt (std::stoi (_words[1]));
              od_factory->make_destination (_dest_ID);
            }
        }
    }
  _od_file.close ();
  return 0;
}

int
MNM_IO::read_origin_vehicle_label_ratio (const std::string &file_folder,
                                         MNM_ConfReader *conf_reader,
                                         MNM_OD_Factory *od_factory,
                                         const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  /* build */
  MNM_Origin *_origin;
  TInt _origin_ID;
  std::string _line;
  std::vector<std::string> _words;
  if (_file.is_open ())
    {
      /* read config */
      TInt _num_of_O = conf_reader->get_int ("num_of_O");
      TInt _num_of_vehicle_labels
        = conf_reader->get_int ("num_of_vehicle_labels");

      if (_num_of_vehicle_labels <= 0)
        {
          return 0;
        }

      // printf("Start build Origin-Destination factory.\n");
      std::getline (_file, _line); // skip the first line
      // printf("Processing Origin node.\n");
      for (int i = 0; i < _num_of_O; ++i)
        {
          std::getline (_file, _line);
          _words = split (_line, ' ');
          if ((int) _words.size () == 1 + _num_of_vehicle_labels)
            { // check
              // std::cout << "Processing: " << _line << "\n";
              _origin_ID = TInt (std::stoi (_words[0]));

              _origin = od_factory->get_origin (_origin_ID);
              for (int j = 0; j < _num_of_vehicle_labels; ++j)
                {
                  _origin->m_vehicle_label_ratio.push_back (
                    TFlt (std::stof (_words[1 + j])));
                }
            }
        }
    }
  else
    {
      printf ("No vehicle registration data\n");
    }
  _file.close ();
  return 0;
}

int
MNM_IO::read_vehicle_tracking_setting (
  const std::string &file_folder,
  std::vector<std::pair<int, int>> *od_pair_tracked,
  std::vector<int> *interval_tracked, const std::string &od_tracking_file_name,
  const std::string &interval_tracking_file_name)
{
  int _origin_ID, _dest_ID;
  std::string _line;
  std::vector<std::string> _words;

  /* find file */
  std::string _od_pair_tracked_file_name
    = file_folder + "/" + od_tracking_file_name;
  std::ifstream _od_pair_tracked_file;
  _od_pair_tracked_file.open (_od_pair_tracked_file_name, std::ios::in);

  if (_od_pair_tracked_file.is_open ())
    {
      std::getline (_od_pair_tracked_file, _line);
      while (std::getline (_od_pair_tracked_file, _line))
        {
          _words = split (_line, ' ');
          if ((int) _words.size () == 2)
            {
              _origin_ID = std::stoi (_words[0]);
              _dest_ID = std::stoi (_words[1]);
              od_pair_tracked->push_back (
                std::make_pair (_origin_ID, _dest_ID));
            }
          else
            {
              throw std::runtime_error ("Wrong OD tracking file");
            }
        }
    }
  else
    {
      printf ("No OD pair tracking file\n");
    }
  _od_pair_tracked_file.close ();

  std::string _interval_tracked_file_name
    = file_folder + "/" + interval_tracking_file_name;
  std::ifstream _interval_tracked_file;
  _interval_tracked_file.open (_interval_tracked_file_name, std::ios::in);

  if (_interval_tracked_file.is_open ())
    {
      std::getline (_interval_tracked_file, _line);
      while (std::getline (_interval_tracked_file, _line))
        {
          _words = split (_line, ' ');
          if ((int) _words.size () == 1)
            {
              interval_tracked->push_back (std::stoi (_words[0]));
            }
          else
            {
              throw std::runtime_error ("Wrong interval tracking file");
            }
        }
    }
  else
    {
      printf ("No interval tracking file\n");
    }
  _interval_tracked_file.close ();
  return 0;
}

macposts::Graph
MNM_IO::build_graph (const std::string &file_folder,
                     MNM_ConfReader *conf_reader)
{
  /* find file */
  std::string _network_name = conf_reader->get_string ("network_name");
  std::string _graph_file_name = file_folder + "/" + _network_name;
  std::ifstream _graph_file;
  _graph_file.open (_graph_file_name, std::ios::in);

  TInt _num_of_link = conf_reader->get_int ("num_of_link");

  macposts::Graph _graph;

  int _link_ID, _from_ID, _to_ID;
  std::string _line;
  std::vector<std::string> _words;
  for (int i = 0; i < _num_of_link;)
    {
      std::getline (_graph_file, _line);
      _line = MNM_IO::trim (_line);
      if (_line.empty () || _line[0] == '#')
        {
          continue;
        }
      ++i;
      _words = MNM_IO::split (_line, ' ');
      if (_words.size () == 3)
        {
          // std::cout << "Processing: " << _line << "\n";
          _link_ID = TInt (std::stoi (_words[0]));
          _from_ID = TInt (std::stoi (_words[1]));
          _to_ID = TInt (std::stoi (_words[2]));
          try
            {
              _graph.add_node (_from_ID);
            }
          // FIXME: This could be overly generic. Maybe we should use a more
          // specific exception type.
          catch (const std::runtime_error &)
            {
            }
          try
            {
              _graph.add_node (_to_ID);
            }
          // FIXME: This could be overly generic. Maybe we should use a more
          // specific exception type.
          catch (const std::runtime_error &)
            {
            }
          _graph.add_link (_from_ID, _to_ID, _link_ID);
        }
    }
  assert ((std::ptrdiff_t) _graph.size_links () == _num_of_link);
  return _graph;
}

int
MNM_IO::build_demand (const std::string &file_folder,
                      MNM_ConfReader *conf_reader, MNM_OD_Factory *od_factory,
                      const std::string &file_name)
{
  /* find file */
  std::string _demand_file_name = file_folder + "/" + file_name;
  std::ifstream _demand_file;
  _demand_file.open (_demand_file_name, std::ios::in);

  /* read config */
  TInt _max_interval = conf_reader->get_int ("max_interval");
  TInt _num_OD = conf_reader->get_int ("OD_pair");

  /* build */
  TInt _O_ID, _D_ID;
  MNM_Origin *_origin;
  MNM_Destination *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_demand_file.is_open ())
    {
      // printf("Start build demand profile.\n");
      double *_demand_vector = new double[_max_interval]();
      for (int i = 0; i < _num_OD;)
        {
          std::getline (_demand_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          // if (TInt(_words.size()) == (_max_interval + 2)) {
          if (TInt (_words.size ()) >= (_max_interval + 2))
            {
              _O_ID = TInt (std::stoi (_words[0]));
              _D_ID = TInt (std::stoi (_words[1]));
              memset (_demand_vector, 0x0, sizeof (TFlt) * _max_interval);
              for (int j = 0; j < _max_interval; ++j)
                {
                  _demand_vector[j] = TFlt (std::stod (_words[j + 2]));
                }
              _origin = od_factory->get_origin (_O_ID);
              _dest = od_factory->get_destination (_D_ID);
              _origin->add_dest_demand (_dest, _demand_vector);
            }
          else
            {
              delete[] _demand_vector;
              throw std::runtime_error ("failed to build demand");
            }
        }
      delete[] _demand_vector;
      _demand_file.close ();
    }
  return 0;
}

int MNM_IO::build_td_adaptive_ratio (const std::string &file_folder,
                                    MNM_ConfReader *conf_reader,
                                    MNM_OD_Factory *od_factory,
                                    const std::string &file_name)
{
  /* find file */
  std::string _ratio_file_name = file_folder + "/" + file_name;
  std::ifstream _ratio_file;
  _ratio_file.open (_ratio_file_name, std::ios::in);

  /* read config */
  TInt _max_interval = conf_reader->get_int ("max_interval");

  /* build */
  TInt _O_ID, _D_ID;
  MNM_Origin *_origin;
  MNM_Destination *_dest;
  std::string _line;
  std::vector<std::string> _words;
  if (_ratio_file.is_open ())
    {
      // printf("Start build demand profile.\n");
      TFlt *_ratio_vector = (TFlt *) malloc (sizeof (TFlt) * _max_interval);
      std::getline (_ratio_file, _line);
      while (std::getline (_ratio_file, _line))
        {
          // std::cout << "Processing: " << _line << "\n";
          _line = trim (_line);
          _words = split (_line, ' ');
          // if (TInt(_words.size()) == (_max_interval + 2)) {
          if (TInt (_words.size ()) >= (_max_interval + 2))
            {
              _O_ID = TInt (std::stoi (_words[0]));
              _D_ID = TInt (std::stoi (_words[1]));
              memset (_ratio_vector, 0x0, sizeof (TFlt) * _max_interval);
              for (int j = 0; j < _max_interval; ++j)
                {
                  _ratio_vector[j] = TFlt (std::stod (_words[j + 2]));
                }
              _origin = od_factory->get_origin (_O_ID);
              _dest = od_factory->get_destination (_D_ID);
              _origin->add_dest_adaptive_ratio (_dest, _ratio_vector);
            }
          else
            {
              free (_ratio_vector);
              throw std::runtime_error ("failed to build time-dependent adaptive ratio");
            }
        }
      free (_ratio_vector);
      _ratio_file.close ();
    }
  else {
    printf("No time-dependent adaptive ratio file\n");
  }
  return 0;
}

int
MNM_IO::build_link_td_attribute (const std::string &file_folder,
                                MNM_Link_Factory *link_factory,
                                const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _interval, _link_ID;
  TFlt _lane_hold_cap;
  TFlt _lane_flow_cap;
  TInt _number_of_lane;
  TFlt _length;
  TFlt _ffs;
  std::string _type;
  TFlt _toll;

  auto td_link_attribute_table
    = dynamic_cast<MNM_Link_Factory *> (link_factory)
        ->m_td_link_attribute_table;

  if (_file.is_open ())
    {
      printf ("Start build time-dependent link attribute.\n");
      std::getline (_file, _line); // #link_ID toll_car toll_truck
      int i = 0;
      while (std::getline (_file, _line))
        {
          // std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";

          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 13)
            {
              _interval = TInt (std::stoi (_words[0]));
              _link_ID = TInt (std::stoi (_words[1]));
              _type = trim (_words[2]);
              _length = TFlt (std::stod (_words[3]));
              _ffs = TFlt (std::stod (_words[4]));
              _lane_flow_cap = TFlt (
                std::stod (_words[5])); // flow capacity (vehicles/hour/lane)
              _lane_hold_cap = TFlt (
                std::stod (_words[6])); // jam density (vehicles/mile/lane)
              _number_of_lane = TInt (std::stoi (_words[7]));
              _toll = TFlt (std::stoi (_words[8]));

              /* unit conversion */
              // mile -> meter, hour -> second
              _length = _length * TFlt (1600);  // m
              _ffs = _ffs * TFlt (1600) / TFlt (3600); // m/s
              _lane_flow_cap
                = _lane_flow_cap / TFlt (3600); // vehicles/s/lane
              _lane_hold_cap
                = _lane_hold_cap / TFlt (1600); // vehicles/m/lane

              if (td_link_attribute_table->find (_interval)
                  == td_link_attribute_table->end ())
                {
                  td_link_attribute_table->insert (
                    std::make_pair (_interval,
                                    new std::unordered_map<
                                      int, td_link_attribute_row *> ()));
                }
              if (td_link_attribute_table->find (_interval)->second->find (
                    _link_ID)
                  == td_link_attribute_table->find (_interval)->second->end ())
                {
                  td_link_attribute_table->find (_interval)->second->insert (
                    std::make_pair (_link_ID, new td_link_attribute_row ()));
                }

              auto *_td_row = td_link_attribute_table->find (_interval)
                                ->second->find (_link_ID)
                                ->second;
              _td_row->link_type = _type;
              _td_row->length = _length;
              _td_row->FFS = _ffs;
              _td_row->Cap = _lane_flow_cap;
              _td_row->RHOJ = _lane_hold_cap;
              _td_row->Lane = _number_of_lane;
              _td_row->toll = _toll;
            }
          else
            {
              std::cout << _line << std::endl;
              throw std::runtime_error ("failed to parse line: " + _line);
            }
          ++i;
        }
      _file.close ();
      printf ("Finish build time-dependent link attribute.\n");
    }
  else
    {
      printf ("No time-dependent link attribute.\n");
    }
  return 0;
}

Path_Table *
MNM_IO::load_path_table (const std::string &file_name,
                         const macposts::Graph &graph, TInt num_path,
                         bool w_buffer, bool w_ID)
{
  if (w_ID)
    {
      throw std::runtime_error (
        "Error, MNM_IO::load_path_table, with ID loading not implemented");
    }
  printf ("Loading Path Table for Driving!\n");
  TInt Num_Path = num_path;
  printf ("Number of path %d\n", Num_Path);

  if (Num_Path <= 0)
    {
      printf ("Finish Loading Path Table for Driving, which is nullptr!\n");
      return nullptr;
    }

  std::ifstream _path_table_file, _buffer_file;
  std::string _buffer_file_name;
  if (w_buffer)
    {
      _buffer_file_name = file_name + "_buffer";
      _buffer_file.open (_buffer_file_name, std::ios::in);
    }
  _path_table_file.open (file_name, std::ios::in);
  Path_Table *_path_table = new Path_Table ();

  /* read file */
  std::string _line, _buffer_line;
  std::vector<std::string> _words, _buffer_words;
  TInt _origin_node_ID, _dest_node_ID, _node_ID;
  std::unordered_map<TInt, MNM_Pathset *> *_new_map;
  MNM_Pathset *_pathset;
  MNM_Path *_path;
  TInt _from_ID, _to_ID, _link_ID;
  TInt _path_ID_counter = 0;
  if (_path_table_file.is_open ())
    {
      for (int i = 0; i < Num_Path;)
        {
          std::getline (_path_table_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          if (w_buffer)
            {
              while (1)
                {
                  std::getline (_buffer_file, _buffer_line);
                  _buffer_line = trim (_buffer_line);
                  if (!_line.empty () && _line[0] != '#')
                    {
                      break;
                    }
                }
              _buffer_words = split (_buffer_line, ' ');
            }
          // std::cout << "Processing: " << _line << "\n";
          _words = split (_line, ' ');
          if (_words.size () >= 2)
            {
              _origin_node_ID = TInt (std::stoi (_words[0]));
              _dest_node_ID = TInt (std::stoi (_words.back ()));
              if (_path_table->find (_origin_node_ID) == _path_table->end ())
                {
                  _new_map = new std::unordered_map<TInt, MNM_Pathset *> ();
                  _path_table->insert (
                    std::pair<TInt, std::unordered_map<TInt, MNM_Pathset *>
                                      *> (_origin_node_ID, _new_map));
                }
              if (_path_table->find (_origin_node_ID)
                    ->second->find (_dest_node_ID)
                  == _path_table->find (_origin_node_ID)->second->end ())
                {
                  _pathset = new MNM_Pathset ();
                  _path_table->find (_origin_node_ID)
                    ->second->insert (
                      std::pair<TInt, MNM_Pathset *> (_dest_node_ID, _pathset));
                }
              _path = new MNM_Path ();
              _path->m_path_ID = _path_ID_counter;
              _path_ID_counter += 1;
              for (std::string _s_node_ID : _words)
                {
                  _node_ID = TInt (std::stoi (_s_node_ID));
                  _path->m_node_vec.push_back (_node_ID);
                }
              for (size_t j = 0; j < _path->m_node_vec.size () - 1; ++j)
                {
                  _from_ID = _path->m_node_vec[j];
                  _to_ID = _path->m_node_vec[j + 1];
                  for (auto &&c : graph.connections (graph.get_node (_from_ID),
                                                     Direction::Outgoing))
                    {
                      if (graph.get_id (graph.get_endpoints (c).second)
                          == _to_ID)
                        {
                          _link_ID = graph.get_id (c);
                          break;
                        }
                    };
                  _path->m_link_vec.push_back (_link_ID);
                }

              if (w_buffer && (_buffer_words.size () > 0))
                {
                  TInt _buffer_len = TInt (_buffer_words.size ());
                  // printf("Buffer len %d\n", _buffer_len());
                  _path->allocate_buffer (_buffer_len);
                  for (int j = 0; j < _buffer_len; ++j)
                    {
                      _path->m_buffer[j]
                        = TFlt (std::stof (trim (_buffer_words[j])));
                    }
                }

              _path_table->find (_origin_node_ID)
                ->second->find (_dest_node_ID)
                ->second->m_path_vec.push_back (_path);
            }
        }
      _path_table_file.close ();
      if (w_buffer)
        {
          _buffer_file.close ();
        }
    }
  else
    {
      throw std::runtime_error ("failed to open path table file");
    }
  printf ("Finish Loading Path Table for Driving!\n");
  // printf("path table %p\n", _path_table);
  // printf("path table %s\n", _path_table -> find(100283) -> second ->
  // find(150153) -> second
  //                           -> m_path_vec.front() -> node_vec_to_string());
  return _path_table;
}

int
MNM_IO::build_vms_facotory (const std::string &file_folder,
                            const macposts::Graph &graph, TInt num_vms,
                            MNM_Vms_Factory *vms_factory,
                            const std::string &file_name)
{
  /* find file */
  std::string _vms_file_name = file_folder + "/" + file_name;
  std::ifstream _vms_file;
  _vms_file.open (_vms_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _vms_ID, _link_ID;
  if (_vms_file.is_open ())
    {
      // printf("Start build demand profile.\n");
      for (int i = 0; i < num_vms;)
        {
          std::getline (_vms_file, _line);
          _line = trim (_line);
          if (_line.empty () || _line[0] == '#')
            {
              continue;
            }
          ++i;
          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 2)
            {
              _vms_ID = TInt (std::stoi (trim (_words[0])));
              _link_ID = TInt (std::stoi (trim (_words[1])));
              vms_factory->make_link_vms (_vms_ID, _link_ID, graph);
            }
          else
            {
              throw std::runtime_error ("failed to build vms");
            }
        }
      _vms_file.close ();
    }
  return 0;
}

int
MNM_IO::read_int_float (const std::string &file_name,
                        std::unordered_map<TInt, TFlt> *reader)
{
  /* find file */
  std::ifstream _file;
  _file.open (file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _int;
  TFlt _float;
  if (_file.is_open ())
    {
      printf ("Start build read_int_float.\n");
      std::getline (_file, _line);
      std::cout << "Processing: " << _line << "\n";
      TInt num_record = TInt (std::stoi (trim (_line)));
      printf ("Total is %d\n", num_record);
      for (int i = 0; i < num_record; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 2)
            {
              _int = TInt (std::stoi (trim (_words[0])));
              _float = TFlt (std::stof (trim (_words[1])));
              reader->insert (std::pair<TInt, TFlt> (_int, _float));
            }
          else
            {
              throw std::runtime_error ("failed to build read_int_float");
            }
        }
      _file.close ();
    }
  return 0;
}

int
MNM_IO::read_int (const std::string &file_name, std::vector<TInt> *reader)
{
  /* find file */
  std::ifstream _file;
  _file.open (file_name, std::ios::in);

  std::string _line;
  TInt _int;
  if (_file.is_open ())
    {
      printf ("Start read int file.\n");
      std::getline (_file, _line);
      std::cout << "Processing: " << _line << "\n";
      TInt num_record = TInt (std::stoi (trim (_line)));
      printf ("Total is %d\n", num_record);
      for (int i = 0; i < num_record; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _int = TInt (std::stoi (trim (_line)));
          reader->push_back (_int);
        }
      _file.close ();
    }
  return 0;
}

int
MNM_IO::read_float (const std::string &file_name, std::vector<TFlt *> *reader)
{
  /* find file */
  std::ifstream _file;
  _file.open (file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TFlt _flt;
  TInt _len;
  TFlt *_tmp_flt;
  if (_file.is_open ())
    {
      printf ("Start read float file.\n");
      std::getline (_file, _line);
      std::cout << "Processing: " << _line << "\n";
      TInt num_record = TInt (std::stoi (trim (_line)));
      printf ("Total is %d\n", num_record);
      for (int i = 0; i < num_record; ++i)
        {
          std::getline (_file, _line);
          _words = split (_line, ' ');
          _len = TInt (_words.size ());
          _tmp_flt = new double[_len]();
          // std::cout << "Processing: " << _line << "\n";
          for (int j = 0; j < _len; ++j)
            {
              _flt = TFlt (std::stof (trim (_words[j])));
              _tmp_flt[j] = _flt;
            }
          reader->push_back (_tmp_flt);
        }
      _file.close ();
    }
  return 0;
}

int
MNM_IO::build_workzone_list (const std::string &file_folder,
                             MNM_Workzone *workzone,
                             const std::string &file_name)
{
  /* find file */
  std::string _workzone_file_name = file_folder + "/" + file_name;
  std::ifstream _workzone_file;
  _workzone_file.open (_workzone_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;

  if (_workzone_file.is_open ())
    {
      printf ("Start build workzone profile.\n");
      std::getline (_workzone_file, _line);
      TInt num_workzone = TInt (std::stoi (trim (_line)));
      for (int i = 0; i < num_workzone; ++i)
        {
          std::getline (_workzone_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 1)
            {
              _link_ID = TInt (std::stoi (trim (_words[0])));
              Link_Workzone _w = { _link_ID };
              workzone->m_workzone_list.push_back (_w);
            }
          else
            {
              throw std::runtime_error ("failed to build workzone");
            }
        }
      _workzone_file.close ();
    }

  return 0;
}

int
MNM_IO::dump_cumulative_curve (const std::string &file_folder,
                               MNM_Link_Factory *link_factory,
                               const std::string &file_name)
{
  /* find file */
  std::string _cc_file_name = file_folder + "/" + file_name;
  std::ofstream _cc_file;
  _cc_file.open (_cc_file_name, std::ios::out);

  MNM_Dlink *_link;
  for (auto _link_it = link_factory->m_link_map.begin ();
       _link_it != link_factory->m_link_map.end (); _link_it++)
    {
      _link = _link_it->second;
      std::string _temp_s = std::to_string (_link->m_link_ID) + ",";
      if (_link->m_N_in != nullptr)
        {
          std::string _temp_s_in
            = _temp_s + "in," + _link->m_N_in->to_string () + "\n";
          _cc_file << _temp_s_in;
        }
      if (_link->m_N_out != nullptr)
        {
          std::string _temp_s_out
            = _temp_s + "out," + _link->m_N_out->to_string () + "\n";
          _cc_file << _temp_s_out;
        }
    }
  _cc_file.close ();
  return 0;
}

int
MNM_IO::build_link_toll (const std::string &file_folder,
                         MNM_ConfReader *conf_reader,
                         MNM_Link_Factory *link_factory,
                         const std::string &file_name)
{
  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;

  if (_file.is_open ())
    {
      TInt _num_of_tolled_link = conf_reader->get_int ("num_of_tolled_link");
      if (_num_of_tolled_link <= 0)
        {
          _file.close ();
          printf ("No tolled links.\n");
          return 0;
        }

      printf ("Start build link toll.\n");
      std::getline (_file, _line); // #link_ID toll
      for (int i = 0; i < _num_of_tolled_link; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";

          _words = split (_line, ' ');
          if (TInt (_words.size ()) == 2)
            {
              _link_ID = TInt (std::stoi (trim (_words[0])));
              link_factory->get_link (_link_ID)->m_toll
                = TFlt (std::stof (trim (_words[1])));
            }
          else
            {
              throw std::runtime_error ("failed to build link toll");
            }
        }
      _file.close ();
      printf ("Finish build link toll.\n");
    }
  else
    {
      printf ("No tolled links.\n");
    }
  return 0;
}

std::vector<std::string>
MNM_IO::split (const std::string &text, char sep)
{
  std::vector<std::string> tokens;
  // Skip leading separators if any
  std::size_t start = text.find_first_not_of (sep, 0);
  std::size_t end;
  while ((end = text.find (sep, start)) != std::string::npos)
    {
      tokens.push_back (text.substr (start, end - start));
      // Skip successive separators in between
      start = text.find_first_not_of (sep, end + 1);
    }
  if (start < text.size ())
    tokens.push_back (text.substr (start));
  return tokens;
}

int
MNM_IO::read_td_link_cost (const std::string &file_folder,
                           std::unordered_map<TInt, TFlt *> &td_link_cost,
                           const TInt num_rows, const TInt num_timestamps,
                           const std::string &file_name)
{
  if (!td_link_cost.empty ())
    {
      for (auto _it : td_link_cost)
        {
          memset (_it.second, 0x0, sizeof (TFlt) * num_timestamps);
        }
    }

  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _link_ID;
  TFlt _cost;
  TFlt *_cost_vector;

  if (_file.is_open ())
    {
      std::cout << "Start reading " << file_name << "\n";
      std::getline (_file, _line); // skip the header
      for (int i = 0; i < num_rows; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _words = split (_line, ' ');
          if (TInt (_words.size ()) == num_timestamps + 1)
            {
              _link_ID = TInt (std::stoi (trim (_words[0])));
              if (td_link_cost.find (_link_ID) == td_link_cost.end ())
                {
                  double *_cost_vector_tmp = new double[num_timestamps]();
                  td_link_cost.insert (
                    std::pair<TInt, TFlt *> (_link_ID, _cost_vector_tmp));
                }
              _cost_vector = td_link_cost.find (_link_ID)->second;
              for (int j = 0; j < num_timestamps; ++j)
                {
                  _cost = TFlt (std::stof (trim (_words[1 + j])));
                  _cost_vector[j] = _cost;
                }
            }
          else
            {
              throw std::runtime_error ("failed to read line: " + _line);
            }
        }
      _file.close ();
    }
  else
    {
      throw std::runtime_error ("failed to open file: " + _file_name);
    }
  return 0;
}

int
MNM_IO::read_td_node_cost (
  const std::string &file_folder,
  std::unordered_map<TInt, std::unordered_map<TInt, TFlt *>> &td_node_cost,
  const TInt num_rows, const TInt num_timestamps, const std::string &file_name)
{
  if (!td_node_cost.empty ())
    {
      for (auto _it : td_node_cost)
        {
          for (auto _it_it : _it.second)
            {
              memset (_it_it.second, 0x0, sizeof (TFlt) * num_timestamps);
            }
        }
    }

  /* find file */
  std::string _file_name = file_folder + "/" + file_name;
  std::ifstream _file;
  _file.open (_file_name, std::ios::in);

  std::string _line;
  std::vector<std::string> _words;
  TInt _in_link_ID, _out_link_ID;
  TFlt _cost;
  TFlt *_cost_vector;

  if (_file.is_open ())
    {
      std::cout << "Start reading " << file_name << "\n";
      std::getline (_file, _line); // skip the header
      for (int i = 0; i < num_rows; ++i)
        {
          std::getline (_file, _line);
          // std::cout << "Processing: " << _line << "\n";
          _words = split (_line, ' ');
          if (TInt (_words.size ()) == num_timestamps + 3)
            {
              _in_link_ID = TInt (std::stoi (trim (_words[1])));
              _out_link_ID = TInt (std::stoi (trim (_words[2])));
              if (td_node_cost.find (_in_link_ID) == td_node_cost.end ())
                {
                  td_node_cost.insert (
                    std::pair<TInt, std::unordered_map<
                                      TInt, TFlt *>> (_in_link_ID,
                                                      std::unordered_map<
                                                        TInt, TFlt *> ()));
                }
              if (td_node_cost.find (_in_link_ID)->second.find (_out_link_ID)
                  == td_node_cost.find (_in_link_ID)->second.end ())
                {
                  double *_cost_vector_tmp = new double[num_timestamps]();
                  td_node_cost.find (_in_link_ID)
                    ->second.insert (
                      std::pair<TInt, TFlt *> (_out_link_ID, _cost_vector_tmp));
                }
              _cost_vector = td_node_cost.find (_in_link_ID)
                               ->second.find (_out_link_ID)
                               ->second;
              for (int j = 0; j < num_timestamps; ++j)
                {
                  _cost = TFlt (std::stof (trim (_words[3 + j])));
                  _cost_vector[j] = _cost;
                }
            }
          else
            {
              throw std::runtime_error ("failed to read line: " + _line);
            }
        }
      _file.close ();
    }
  else
    {
      throw std::runtime_error ("failed to open file: " + _file_name);
    }
  return 0;
}
