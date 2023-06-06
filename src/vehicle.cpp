#include "vehicle.h"

MNM_Veh::MNM_Veh (TInt ID, TInt start_time)
{
  m_veh_ID = ID;
  m_current_link = nullptr;
  m_next_link = nullptr;
  m_start_time = start_time;
  m_finish_time = -1;
  m_miles_traveled = 0.;
  m_assign_interval = -1;
  m_path = nullptr;
  m_class = TInt (0);
  m_bus_route_ID = TInt (-1);
  m_pnr = false;
}

MNM_Veh::~MNM_Veh ()
{
  m_current_link = nullptr;
  m_next_link = nullptr;
  m_path = nullptr;
}

int
MNM_Veh::set_current_link (MNM_Dlink *link)
{
  m_current_link = link;
  return 0;
}

MNM_Dlink *
MNM_Veh::get_current_link ()
{
  return m_current_link;
}

MNM_Dlink *
MNM_Veh::get_next_link ()
{
  return m_next_link;
}

bool
MNM_Veh::has_next_link ()
{
  return (m_next_link != nullptr);
}

MNM_Destination *
MNM_Veh::get_destination ()
{
  return m_dest;
}

int
MNM_Veh::set_destination (MNM_Destination *dest)
{
  m_dest = dest;
  return 0;
}

int
MNM_Veh::set_next_link (MNM_Dlink *link)
{
  m_next_link = link;
  return 0;
}

int
MNM_Veh::finish (TInt finish_time)
{
  m_finish_time = finish_time;
  return 0;
}

MNM_Origin *
MNM_Veh::get_origin ()
{
  return m_origin;
}

int
MNM_Veh::set_origin (MNM_Origin *origin)
{
  m_origin = origin;
  return 0;
}

int 
MNM_Veh::update_miles_traveled(MNM_Dlink *link)
{
  if (dynamic_cast<MNM_Dlink_Pq*>(link) == nullptr) {
    m_miles_traveled += link -> m_length / 1600.;  // meter -> mile
  }
  return 0;
}