#pragma once

#include "Snap.h"
#include "dlink.h"
#include "enum.h"
#include "od.h"

#include <deque>

class MNM_Destination;
class MNM_Origin;
class MNM_Dlink;
class MNM_Path;

class MNM_Veh
{
public:
  MNM_Veh (TInt ID, TInt start_time);
  virtual ~MNM_Veh ();
  TInt m_veh_ID;
  int set_current_link (MNM_Dlink *link);
  MNM_Dlink *get_current_link ();
  MNM_Dlink *get_next_link ();
  int set_next_link (MNM_Dlink *link);
  bool has_next_link ();
  MNM_Destination *get_destination ();
  int set_destination (MNM_Destination *dest);
  int finish (TInt finish_time);
  MNM_Origin *get_origin ();
  int set_origin (MNM_Origin *origin);

  virtual TInt get_class ()
  {
    return m_class;
  }; // virtual getter for derived class
  virtual TInt get_bus_route_ID ()
  {
    return m_bus_route_ID;
  }; // virtual getter for derived class
  virtual bool get_ispnr ()
  {
    return m_pnr;
  }; // virtual getter for derived class
  virtual TInt get_label ()
  {
    return m_label;
  }; // virtual getter for derived class

  virtual int update_miles_traveled(MNM_Dlink *link);
  
     // private:
  Vehicle_type m_type;
  MNM_Dlink *m_current_link;
  TInt m_start_time;
  TInt m_finish_time;
  MNM_Dlink *m_next_link;
  MNM_Destination *m_dest;
  MNM_Origin *m_origin;
  // m_path will only be used in Fixed routing (didn't find a better way to
  // encode) m_path for adaptive routing is just nominal, not exactly the actual
  // path
  MNM_Path *m_path;
  TInt m_assign_interval;

  TFlt m_miles_traveled;

  TInt m_class;
  // m_bus_route only used in multimodal loading
  TInt m_bus_route_ID;
  // m_pnr only used in multimodal loading
  bool m_pnr;

  // m_label only used with registration data
  TInt m_label = -1;
};

// class MNM_Veh_Det :: public MNM_Veh
// {
// public:
//   MNM_Veh_Det(TInt ID, TInt start_time);
//   ~MNM_Veh_Det();
// };

// class MNM_Veh_Adp :: public MNM_Veh
// {
// public:
//   MNM_Veh_Adp(TInt ID, TInt start_time);
//   ~MNM_Veh_Adp();
// };
