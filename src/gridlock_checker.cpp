#include "gridlock_checker.h"

using macposts::graph::Direction;

MNM_Gridlock_Checker::MNM_Gridlock_Checker (macposts::Graph &graph,
                                            MNM_Link_Factory *link_factory)
    : m_full_graph (graph)
{
  m_link_factory = link_factory;
  m_link_veh_map = std::unordered_map<TInt, MNM_Veh *> ();
}

MNM_Gridlock_Checker::~MNM_Gridlock_Checker () { m_link_veh_map.clear (); }

int
MNM_Gridlock_Checker::initialize ()
{
  for (auto _map_it : m_link_factory->m_link_map)
    {
      m_link_veh_map.insert ({ _map_it.first, get_last_veh (_map_it.second) });
    }
  return 0;
}

MNM_Veh *
MNM_Gridlock_Checker::get_last_veh (MNM_Dlink *link)
{
  if (link->m_finished_array.size () == 0)
    {
      return NULL;
    }
  return link->m_finished_array.front ();
}

bool
MNM_Gridlock_Checker::has_cycle (macposts::Graph &graph)
{
  std::deque<TInt> _link_queue = std::deque<TInt> ();
  std::set<TInt> _remained_link_set = std::set<TInt> ();
  for (const auto &l : graph.links ())
    {
      _remained_link_set.insert (graph.get_id (l));
    }
  TInt _temp_link_ID, _temp_link_ID2;
  TInt _temp_dest_node_ID;
  while (_remained_link_set.size () != 0)
    {
      _link_queue.push_back (*(_remained_link_set.begin ()));
      while (_link_queue.size () != 0)
        {
          _temp_link_ID = _link_queue.front ();
          _remained_link_set.erase (_temp_link_ID);
          _link_queue.pop_front ();
          _temp_dest_node_ID
            = graph.get_id (graph.get_endpoints (_temp_link_ID).second);
          for (const auto &l :
               graph.connections (_temp_dest_node_ID, Direction::Outgoing))
            {
              _temp_link_ID2 = graph.get_id (l);
              if (_remained_link_set.find (_temp_link_ID2)
                  != _remained_link_set.end ())
                {
                  _link_queue.push_back (_temp_link_ID2);
                }
              else
                {
                  return false;
                }
            }
        }
    }
  return true;
}

bool
MNM_Gridlock_Checker::is_gridlocked ()
{
  return false;
}

MNM_Gridlock_Link_Recorder::MNM_Gridlock_Link_Recorder (
  const std::string &file_folder, MNM_ConfReader *record_config)
{
  m_config = record_config;
  if (m_record_file.is_open ())
    m_record_file.close ();
  m_record_file.open (file_folder + "/"
                        + record_config->get_string ("rec_folder")
                        + "/possible_gridlocked_links",
                      std::ofstream::out);
  if (!m_record_file.is_open ())
    {
      throw std::runtime_error ("failed to open m_record_file");
    }
}

MNM_Gridlock_Link_Recorder::~MNM_Gridlock_Link_Recorder () { delete m_config; }

int
MNM_Gridlock_Link_Recorder::init_record ()
{
  std::string _str
    = "loading_interval link_ID flow incoming_flow finished_flow\n";
  m_record_file << _str;
  return 0;
}

int
MNM_Gridlock_Link_Recorder::save_one_link (TInt loading_interval,
                                           MNM_Dlink *link)
{
  if (link->get_link_flow () > 0)
    {
      std::string _str
        = std::to_string (loading_interval) + " "
          + std::to_string (link->m_link_ID) + " "
          + std::to_string (link->get_link_flow ()) + " "
          + std::to_string ((int) link->m_incoming_array.size ()) + " "
          + std::to_string ((int) link->m_finished_array.size ()) + "\n";
      m_record_file << _str;

      printf ("Current Link %d:, traffic flow %.4f, incoming %d, finished %d\n",
              link->m_link_ID, link->get_link_flow (),
              (int) link->m_incoming_array.size (),
              (int) link->m_finished_array.size ());
      link->print_info ();
    }
  return 0;
}

int
MNM_Gridlock_Link_Recorder::post_record ()
{
  if (m_record_file.is_open ())
    m_record_file.close ();
  return 0;
}
