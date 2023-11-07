#include "statistics.h"

MNM_Statistics::MNM_Statistics (const std::string &file_folder,
                                MNM_ConfReader *conf_reader,
                                MNM_ConfReader *record_config,
                                MNM_OD_Factory *od_factory,
                                MNM_Node_Factory *node_factory,
                                MNM_Link_Factory *link_factory)
{
  m_file_folder = file_folder;
  m_global_config = conf_reader;
  m_self_config = record_config;
  m_od_factory = od_factory;
  m_node_factory = node_factory;
  m_link_factory = link_factory;

  // link volume per record interval, <link ID, volume>
  m_record_interval_volume = std::unordered_map<TInt, TFlt> ();
  // link volume per loading interval, <link ID, volume>
  m_load_interval_volume = std::unordered_map<TInt, TFlt> ();
  // link travel time per record interval, <link ID, tt>
  m_record_interval_tt = std::unordered_map<TInt, TFlt> ();
  // link travel time per loading interval, <link ID, tt>
  m_load_interval_tt = std::unordered_map<TInt, TFlt> ();

  // store links in a fixed order
  m_link_order = std::vector<MNM_Dlink *> ();

  if (m_load_interval_volume_file.is_open ())
    m_load_interval_volume_file.close ();
  if (m_record_interval_volume_file.is_open ())
    m_record_interval_volume_file.close ();
  if (m_load_interval_tt_file.is_open ())
    m_load_interval_tt_file.close ();
  if (m_record_interval_tt_file.is_open ())
    m_record_interval_tt_file.close ();

  init_record_value ();
}

MNM_Statistics::~MNM_Statistics ()
{
  delete m_self_config;
  m_record_interval_volume.clear ();
  m_load_interval_volume.clear ();
  m_record_interval_tt.clear ();
  m_load_interval_tt.clear ();
}

int
MNM_Statistics::init_record_value ()
{
  switch (m_self_config->get_int ("rec_volume"))
    {
    case 1:
      m_record_volume = true;
      break;
    case 0:
      m_record_volume = false;
      break;
    default:
      throw std::runtime_error ("invalid value for rec_volume");
    }
  switch (m_self_config->get_int ("rec_tt"))
    {
    case 1:
      m_record_tt = true;
      break;
    case 0:
      m_record_tt = false;
      break;
    default:
      throw std::runtime_error ("invalid value for rec_tt");
    }
  return 0;
}

int
MNM_Statistics::init_record ()
{
  TInt _link_ID;
  std::string _file_name;
  if (m_record_volume)
    {
      for (auto _link_it : m_link_factory->m_link_map)
        {
          _link_ID = _link_it.first;
          m_load_interval_volume.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
          m_record_interval_volume.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
        }

      if (m_self_config->get_int ("volume_load_automatic_rec") == 1
          || m_self_config->get_int ("volume_record_automatic_rec") == 1)
        {
          std::string _str = "Interval ";
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _str += std::to_string (_link_it.first) + " ";
            }
          _str.pop_back ();
          _str += "\n";

          if (m_self_config->get_int ("volume_load_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_load_interval_volume";
              m_load_interval_volume_file.open (_file_name, std::ofstream::out);
              if (!m_load_interval_volume_file.is_open ())
                {
                  throw std::runtime_error ("failed to open file: "
                                            + _file_name);
                }
              m_load_interval_volume_file << _str;
            }

          if (m_self_config->get_int ("volume_record_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_record_interval_volume";
              m_record_interval_volume_file.open (_file_name,
                                                  std::ofstream::out);
              if (!m_record_interval_volume_file.is_open ())
                {
                  throw std::runtime_error ("failed to open file: "
                                            + _file_name);
                }
              m_record_interval_volume_file << _str;
            }
        }
    }

  if (m_record_tt)
    {
      for (auto _link_it : m_link_factory->m_link_map)
        {
          _link_ID = _link_it.first;
          m_load_interval_tt.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
          m_record_interval_tt.insert (
            std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
        }

      if (m_self_config->get_int ("tt_load_automatic_rec") == 1
          || m_self_config->get_int ("tt_record_automatic_rec") == 1)
        {
          std::string _str = "Interval ";
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _str += std::to_string (_link_it.first) + " ";
            }
          _str.pop_back ();
          _str += "\n";

          if (m_self_config->get_int ("tt_load_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_load_interval_tt";
              m_load_interval_tt_file.open (_file_name, std::ofstream::out);
              if (!m_load_interval_tt_file.is_open ())
                {
                  throw std::runtime_error ("failed to open file: "
                                            + _file_name);
                }
              m_load_interval_tt_file << _str;
            }

          if (m_self_config->get_int ("tt_record_automatic_rec") == 1)
            {
              _file_name = m_file_folder + "/"
                           + m_self_config->get_string ("rec_folder")
                           + "/MNM_output_record_interval_tt";
              m_record_interval_tt_file.open (_file_name, std::ofstream::out);
              if (!m_record_interval_tt_file.is_open ())
                {
                  throw std::runtime_error ("failed to open file: "
                                            + _file_name);
                }
              m_record_interval_tt_file << _str;
            }
        }
    }

  // store links in a fixed order in a vector, same as the order of creating the
  // head of the record file
  // https://stackoverflow.com/questions/18301302/is-forauto-i-unordered-map-guaranteed-to-have-the-same-order-every-time
  // The iteration order of unordered associative containers can only change
  // when rehashing as a result of a mutating operation (as described in
  // C++11 23.2.5/8). You are not modifying the container between iterations, so
  // the order will not change.
  for (auto _link_it : m_link_factory->m_link_map)
    {
      m_link_order.push_back (_link_it.second);
    }
  return 0;
}

int
MNM_Statistics::record_loading_interval_condition (TInt timestamp)
{
  std::string _str;
  TFlt _flow, _tt;
  if (m_record_volume && m_load_interval_volume_file.is_open ())
    {
      _str = std::to_string(timestamp) + " ";
      for (auto _link : m_link_order)
        {
          _flow = m_load_interval_volume.find (_link->m_link_ID)->second;
          _str += std::to_string (_flow) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_load_interval_volume_file << _str;
    }

  _str.clear ();
  if (m_record_tt && m_load_interval_tt_file.is_open ())
    {
      _str = std::to_string(timestamp) + " ";
      for (auto _link : m_link_order)
        {
          _tt = m_load_interval_tt.find (_link->m_link_ID)->second;
          _str += std::to_string (_tt) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_load_interval_tt_file << _str;
    }

  return 0;
}

int
MNM_Statistics::record_record_interval_condition (TInt timestamp)
{
  std::string _str;
  TFlt _flow, _tt;
  if (m_record_volume && m_record_interval_volume_file.is_open ())
    {
      _str = std::to_string(timestamp) + " ";
      for (auto _link : m_link_order)
        {
          _flow = m_record_interval_volume.find (_link->m_link_ID)->second;
          _str += std::to_string (_flow) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_record_interval_volume_file << _str;
    }
  _str.clear ();

  if (m_record_tt && m_record_interval_tt_file.is_open ())
    {
      _str = std::to_string(timestamp) + " ";
      for (auto _link : m_link_order)
        {
          _tt = m_record_interval_tt.find (_link->m_link_ID)->second;
          _str += std::to_string (_tt) + " ";
        }
      _str.pop_back ();
      _str += "\n";
      m_record_interval_tt_file << _str;
    }
  return 0;
}

int
MNM_Statistics::post_record ()
{
  if (m_record_volume)
    {
      if (m_load_interval_volume_file.is_open ()) m_load_interval_volume_file.close ();
      if (m_record_interval_volume_file.is_open ()) m_record_interval_volume_file.close ();
    }
  if (m_record_tt)
    {
      if (m_load_interval_tt_file.is_open ()) m_load_interval_tt_file.close ();
      if (m_record_interval_tt_file.is_open ()) m_record_interval_tt_file.close ();
    }
  return 0;
}
/**************************************************************************
                              LRN
**************************************************************************/

MNM_Statistics_Lrn::MNM_Statistics_Lrn (const std::string &file_folder,
                                        MNM_ConfReader *conf_reader,
                                        MNM_ConfReader *record_config,
                                        MNM_OD_Factory *od_factory,
                                        MNM_Node_Factory *node_factory,
                                        MNM_Link_Factory *link_factory)
    : MNM_Statistics::MNM_Statistics (file_folder, conf_reader, record_config,
                                      od_factory, node_factory, link_factory)
{
  // number of loading intervals to be averaged
  m_n = record_config->get_int ("rec_mode_para");
  // temporarily storing aggregated m_n-interval volume, <link ID, volume>
  m_to_be_volume = std::unordered_map<TInt, TFlt> ();
  // temporarily storing aggregated m_n-interval tt, <link ID, tt>
  m_to_be_tt = std::unordered_map<TInt, TFlt> ();
}

MNM_Statistics_Lrn::~MNM_Statistics_Lrn ()
{
  m_to_be_volume.clear ();
  m_to_be_tt.clear ();
}

int
MNM_Statistics_Lrn::update_record (TInt timestamp)
{
  MNM_Dlink *_link;
  TFlt _flow, _tt;
  if (m_record_volume)
    {
      if ((timestamp) % m_n == 0 || timestamp == 0)
        {
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _link = _link_it.second;
              _flow = _link->get_link_flow ();
              m_load_interval_volume.find (_link->m_link_ID)->second = _flow;
              if (timestamp == 0)
                {
                  m_record_interval_volume.find (_link->m_link_ID)->second
                    = _flow;
                }
              else
                {
                  m_record_interval_volume.find (_link->m_link_ID)->second
                    = m_to_be_volume.find (_link->m_link_ID)->second
                      + _flow / TFlt (m_n);
                }
              // reset
              m_to_be_volume.find (_link->m_link_ID)->second = TFlt (0);
            }
        }
      else
        {
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _link = _link_it.second;
              _flow = _link->get_link_flow ();
              m_load_interval_volume.find (_link->m_link_ID)->second = _flow;
              m_to_be_volume.find (_link->m_link_ID)->second
                += _flow / TFlt (m_n);
            }
        }
    }
  if (m_record_tt)
    {
      if ((timestamp) % m_n == 0 || timestamp == 0)
        {
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _link = _link_it.second;
              _tt = _link->get_link_tt (); // seconds
              m_load_interval_tt.find (_link->m_link_ID)->second = _tt;
              if (timestamp == 0)
                {
                  m_record_interval_tt.find (_link->m_link_ID)->second = _tt;
                }
              else
                {
                  m_record_interval_tt.find (_link->m_link_ID)->second
                    = m_to_be_tt.find (_link->m_link_ID)->second
                      + _tt / TFlt (m_n);
                }
              // reset
              m_to_be_tt.find (_link->m_link_ID)->second = TFlt (0);
            }
        }
      else
        {
          for (auto _link_it : m_link_factory->m_link_map)
            {
              _link = _link_it.second;
              _tt = _link->get_link_tt (); // seconds
              m_load_interval_tt.find (_link->m_link_ID)->second = _tt;
              m_to_be_tt.find (_link->m_link_ID)->second += _tt / TFlt (m_n);
            }
        }
    }

  MNM_Statistics::record_loading_interval_condition (timestamp);

  if ((timestamp) % m_n == 0 || timestamp == 0)
    {
      MNM_Statistics::record_record_interval_condition (timestamp);
    }
  return 0;
}

int
MNM_Statistics_Lrn::init_record ()
{
  MNM_Statistics::init_record ();
  TInt _link_ID;
  if (m_record_volume)
    {
      for (auto _link_it : m_link_factory->m_link_map)
        {
          _link_ID = _link_it.first;
          m_to_be_volume.insert (
            std::pair<TInt, TFlt> (_link_ID,
                                   TFlt (0))); // intermediate variable to
                                               // compute average quantities
        }
    }
  if (m_record_tt)
    {
      for (auto _link_it : m_link_factory->m_link_map)
        {
          _link_ID = _link_it.first;
          m_to_be_tt.insert (std::pair<TInt, TFlt> (_link_ID, TFlt (0)));
        }
    }
  return 0;
}
