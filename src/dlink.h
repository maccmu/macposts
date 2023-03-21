#ifndef DLINK_H
#define DLINK_H

#include "enum.h"
#include "limits.h"
#include "ults.h"
#include "vehicle.h"

#include <algorithm>
#include <deque>
#include <unordered_map>
#include <vector>

class MNM_Veh;
class MNM_Dnode;
class MNM_Path;

/**************************************************************************
                          Cumulative curve
**************************************************************************/
class MNM_Cumulative_Curve
{
public:
  MNM_Cumulative_Curve ();
  ~MNM_Cumulative_Curve ();
  // <timestamp, flow>
  std::deque<std::pair<TFlt, TFlt>> m_recorder;
  int add_record (std::pair<TFlt, TFlt> r);
  int add_increment (std::pair<TFlt, TFlt> r);
  TFlt get_result (TFlt time);
  TFlt get_approximated_result (TFlt time);
  TFlt get_time (TFlt result, bool rounding_up = false);
  std::string to_string ();
  int shrink (TInt number);

private:
  int arrange ();
  int arrange2 ();
};

// will majorly used to construct DAR matrix
class MNM_Tree_Cumulative_Curve
{
public:
  MNM_Tree_Cumulative_Curve ();
  ~MNM_Tree_Cumulative_Curve ();
  std::unordered_map<MNM_Path *,
                     std::unordered_map<TInt, MNM_Cumulative_Curve *>>
    m_record;
  int add_flow (TFlt timestamp, TFlt flow, MNM_Path *path, TInt departing_int);
  int print_out ();
};

/**************************************************************************
                          Dlink family
**************************************************************************/

class MNM_Dlink
{
public:
  MNM_Dlink (TInt ID, TInt number_of_lane, TFlt length, TFlt ffs);
  virtual ~MNM_Dlink ();

  virtual int evolve (TInt timestamp) { return 0; };
  virtual TFlt get_link_supply () { return TFlt (0); };
  virtual int clear_incoming_array (TInt timestamp) { return 0; };
  virtual void print_info (){};
  int hook_up_node (MNM_Dnode *from, MNM_Dnode *to);
  virtual TFlt get_link_flow () { return TFlt (0); };
  virtual std::vector<TFlt> get_link_flow_emission (TInt ev_label)
  {
    std::vector<TFlt> _r = { TFlt (0), TFlt (0) };
    return _r;
  };
  virtual TFlt get_link_tt () { return TFlt (0); };
  virtual TFlt get_link_tt_from_flow (TFlt flow) { return TFlt (0); };
  virtual TFlt get_link_freeflow_tt (); // seconds
  virtual TInt get_link_freeflow_tt_loading ()
  {
    return TInt (-1);
  }; // intervals

  int install_cumulative_curve ();
  int install_cumulative_curve_tree ();

  // protected:
  DLink_type m_link_type;
  TInt m_link_ID;
  MNM_Dnode *m_from_node;
  MNM_Dnode *m_to_node;
  TInt m_number_of_lane;
  TFlt m_length;
  TFlt m_ffs;
  std::deque<MNM_Veh *> m_finished_array;
  std::deque<MNM_Veh *> m_incoming_array;

  MNM_Cumulative_Curve *m_N_in;
  MNM_Cumulative_Curve *m_N_out;
  TFlt m_last_valid_time = TFlt (-1);
  MNM_Tree_Cumulative_Curve *m_N_in_tree;
  MNM_Tree_Cumulative_Curve *m_N_out_tree;

  TFlt m_toll = 0.;

  // protected:
  virtual int move_veh_queue (std::deque<MNM_Veh *> *from_queue,
                              std::deque<MNM_Veh *> *to_queue,
                              TInt number_tomove);
};

class MNM_Dlink_Ctm : public MNM_Dlink
{
public:
  MNM_Dlink_Ctm (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                 TInt number_of_lane, TFlt length, TFlt ffs, TFlt unit_time,
                 TFlt flow_scalar);
  virtual ~MNM_Dlink_Ctm () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;
  virtual TFlt get_link_flow () override;
  virtual std::vector<TFlt> get_link_flow_emission (TInt ev_label) override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow (TFlt flow) override;
  virtual TInt get_link_freeflow_tt_loading () override; // intervals

  // private:
  class Ctm_Cell;
  int init_cell_array (TFlt unit_time, TFlt std_cell_length,
                       TFlt lane_hold_cap_last_cell);
  int update_out_veh ();
  int move_last_cell ();
  TInt m_num_cells;
  TFlt m_lane_hold_cap;
  TFlt m_lane_flow_cap;
  TFlt m_flow_scalar;
  TFlt m_wave_ratio;
  TFlt m_last_wave_ratio;
  std::vector<Ctm_Cell *> m_cell_array;
};

class MNM_Dlink_Ctm::Ctm_Cell
{
public:
  Ctm_Cell (TFlt hold_cap, TFlt flow_cap, TFlt flow_scalar, TFlt wave_ratio);
  ~Ctm_Cell ();
  TFlt get_demand ();
  TFlt get_supply ();

  TInt m_volume;
  TFlt m_flow_scalar;
  TFlt m_hold_cap;
  TFlt m_flow_cap;
  TFlt m_wave_ratio;
  TInt m_out_veh;
  std::deque<MNM_Veh *> m_veh_queue;
};

/**************************************************************************
                          Point Queue
**************************************************************************/
class MNM_Dlink_Pq : public MNM_Dlink
{
public:
  MNM_Dlink_Pq (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                TInt number_of_lane, TFlt length, TFlt ffs, TFlt unit_time,
                TFlt flow_scalar);
  virtual ~MNM_Dlink_Pq () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;
  virtual TFlt get_link_flow () override;
  virtual std::vector<TFlt> get_link_flow_emission (TInt ev_label) override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow (TFlt flow) override;
  virtual TInt get_link_freeflow_tt_loading () override; // intervals

  // private:
  std::deque<std::pair<MNM_Veh *, TInt>> m_veh_queue;
  TInt m_volume; // vehicle number, without the flow scalar
  TFlt m_lane_hold_cap;
  TFlt m_lane_flow_cap;
  TFlt m_flow_scalar;
  TFlt m_hold_cap;
  TInt m_max_stamp;
  TFlt m_unit_time;
};

/**************************************************************************
                          Link Queue
**************************************************************************/
class MNM_Dlink_Lq : public MNM_Dlink
{
public:
  MNM_Dlink_Lq (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                TInt number_of_lane, TFlt length, TFlt ffs, TFlt unit_time,
                TFlt flow_scalar);
  virtual ~MNM_Dlink_Lq () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;
  virtual TFlt get_link_flow () override;
  virtual std::vector<TFlt> get_link_flow_emission (TInt ev_label) override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow (TFlt flow) override;
  virtual TInt get_link_freeflow_tt_loading () override; // intervals

  // private:
  std::deque<MNM_Veh *> m_veh_queue;
  TInt m_volume; // vehicle number, without the flow scalar
  TFlt m_lane_hold_cap;
  TFlt m_lane_flow_cap;
  TFlt m_flow_scalar;
  TFlt m_hold_cap;
  TFlt m_C;   // maximum free flow capacity
  TFlt m_k_c; // maximum free flow density
  TFlt m_unit_time;

  TFlt get_flow_from_density (TFlt density);
  TFlt get_demand ();
};

/**************************************************************************
                          Link Transmission model
**************************************************************************/

class MNM_Dlink_Ltm : public MNM_Dlink
{
public:
  MNM_Dlink_Ltm (TInt ID, TFlt lane_hold_cap, TFlt lane_flow_cap,
                 TInt number_of_lane, TFlt length, TFlt ffs, TFlt unit_time,
                 TFlt flow_scalar);
  virtual ~MNM_Dlink_Ltm () override;
  virtual int evolve (TInt timestamp) override;
  virtual TFlt get_link_supply () override;
  virtual int clear_incoming_array (TInt timestamp) override;
  virtual void print_info () override;
  virtual TFlt get_link_flow () override;
  virtual std::vector<TFlt> get_link_flow_emission (TInt ev_label) override;
  virtual TFlt get_link_tt () override;
  virtual TFlt get_link_tt_from_flow (TFlt flow) override;
  virtual TInt get_link_freeflow_tt_loading () override; // intervals

  // private:
  std::deque<MNM_Veh *> m_veh_queue;
  MNM_Cumulative_Curve m_N_in2;
  MNM_Cumulative_Curve m_N_out2;
  TInt m_volume; // vehicle number, without the flow scalar
  TFlt m_lane_hold_cap;
  TFlt m_lane_flow_cap;
  TFlt m_flow_scalar;
  TFlt m_hold_cap;
  TFlt m_unit_time;
  TInt m_current_timestamp; // used only in clear incoming array
  TFlt m_w;
  TFlt m_previous_finished_flow;
  TInt m_record_size;

  TFlt get_demand ();
};

#endif
