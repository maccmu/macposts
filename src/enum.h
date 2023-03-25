#pragma once

enum DNode_type
{
  MNM_TYPE_ORIGIN,
  MNM_TYPE_DEST,
  MNM_TYPE_FWJ,
  MNM_TYPE_GRJ
};
enum DLink_type
{
  MNM_TYPE_CTM,
  MNM_TYPE_PQ,
  MNM_TYPE_LQ,
  MNM_TYPE_LTM
};
enum Dta_type
{
  MNM_TYPE_RANDOM,
  MNM_TYPE_BOSTON,
  MNM_TYPE_HYBRID
};
enum Vehicle_type
{
  MNM_TYPE_ADAPTIVE,
  MNM_TYPE_STATIC
};
enum Record_type
{
  MNM_TYPE_LRN
};

enum DNode_type_multiclass
{
  MNM_TYPE_ORIGIN_MULTICLASS,
  MNM_TYPE_DEST_MULTICLASS,
  MNM_TYPE_FWJ_MULTICLASS
};
enum DLink_type_multiclass
{
  MNM_TYPE_CTM_MULTICLASS,
  MNM_TYPE_LQ_MULTICLASS,
  MNM_TYPE_PQ_MULTICLASS
};

enum DNode_type_multimodal
{
  MNM_TYPE_ORIGIN_MULTIMODAL,
  MNM_TYPE_DEST_MULTIMODAL,
  MNM_TYPE_FWJ_MULTIMODAL
};
enum DLink_type_multimodal
{
  MNM_TYPE_CTM_MULTIMODAL,
  MNM_TYPE_PQ_MULTIMODAL,
  MNM_TYPE_BUS_MULTIMODAL,
  MNM_TYPE_WALKING_MULTIMODAL
};

enum MMDue_mode
{
  driving,
  transit,
  pnr,
  rnd,
  rh,
  bus_route
};
enum MMDue_mode_driving
{
  solo,
  cp2
};
enum MMDue_mode_transit
{
  bus,
  metro
};
enum MMDue_mode_pnr
{
  p1,
  p2,
  p3
};
enum MMDue_mode_rh
{
  rh_only,
  rh_metro
};
