#pragma once

#include "multiclass.h"

// multiclass vehicles
// car and truck both have subclasses, which may use different graph for routing

class MNM_Origin_Multiclass_Subclass : public MNM_Origin_Multiclass
{
public:
    MNM_Origin_Multiclass_Subclass (TInt ID, TInt max_interval, TFlt flow_scalar,
                            TInt frequency);
    virtual ~MNM_Origin_Multiclass_Subclass () override;
    virtual TInt generate_label (TInt veh_class) override;
    virtual int release (MNM_Veh_Factory *veh_factory,
                        TInt current_interval) override;
    virtual int release_one_interval (TInt current_interval,
                                    MNM_Veh_Factory *veh_factory,
                                    TInt assign_interval,
                                    TFlt adaptive_ratio) override;

    virtual int release_one_interval_biclass (TInt current_interval,
                                            MNM_Veh_Factory *veh_factory,
                                            TInt assign_interval,
                                            TFlt adaptive_ratio_car,
                                            TFlt adaptive_ratio_truck) override;

    int add_dest_demand_multiclass_subclass (MNM_Destination_Multiclass *dest, int mainclass_label, int subclass_label, 
                                            TFlt *demand);
    int get_dest_demand_multiclass();

    int add_dest_adaptive_ratio_multiclass (MNM_Destination_Multiclass *dest,
                                            TFlt *ad_ratio_car, TFlt *ad_ratio_truck);

    // <dest, <subclass_label, demand>>
    std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>>  m_demand_car_subclass;
    std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>>  m_demand_truck_subclass;
};


class MNM_IO_Multiclass_Subclass : public MNM_IO_Multiclass
{
public:
  static int build_demand_subclass (const std::string &file_folder,
                                    MNM_ConfReader *conf_reader,
                                    MNM_OD_Factory *od_factory,
                                    int mainclass_label,
                                    const std::string &file_name
                                    = "MNM_input_demand_car");
};