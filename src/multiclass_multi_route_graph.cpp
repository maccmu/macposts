#include "multiclass_multi_route_graph.h"
#include <cfloat>


MNM_Origin_Multiclass_Subclass::MNM_Origin_Multiclass_Subclass (TInt ID, TInt max_interval,
                                              TFlt flow_scalar, TInt frequency)
    : MNM_Origin_Multiclass::MNM_Origin_Multiclass (ID, max_interval, flow_scalar, frequency)
{
    m_demand_car_subclass = std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>> ();
    m_demand_truck_subclass = std::unordered_map<MNM_Destination_Multiclass * , std::unordered_map<int, TFlt *>> ();
    m_adaptive_ratio_car = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
    m_adaptive_ratio_truck = std::unordered_map<MNM_Destination_Multiclass *, TFlt *> ();
    m_car_label_ratio = std::vector<TFlt> ();
    m_truck_label_ratio = std::vector<TFlt> ();
}

MNM_Origin_Multiclass_Subclass::~MNM_Origin_Multiclass_Subclass ()
{
    for (auto _it : m_demand_car_subclass)
    {
        for (auto _demand_it : _it.second)
        {
            delete[] _demand_it.second;
        }
        _it.second.clear ();
    }
    m_demand_car_subclass.clear ();

    for (auto _it : m_demand_truck_subclass)
    {
        for (auto _demand_it : _it.second)
        {
            delete[] _demand_it.second;
        }
        _it.second.clear ();
    }
    m_demand_truck_subclass.clear ();
}

int 
MNM_Origin_Multiclass_Subclass::add_dest_demand_multiclass_subclass (MNM_Destination_Multiclass *dest, int mainclass_label, int subclass_label, 
                                                                     TFlt *demand)
{
    // split (15-mins demand) to (15 * 1-minute demand)
    double *_demand = new double[m_max_assign_interval * 15]();
    for (int i = 0; i < m_max_assign_interval * 15; ++i)
    {
        _demand[i] = TFlt (demand[i]);
    }

    if (mainclass_label == 0) {
        m_demand_car_subclass.insert ({dest, std::unordered_map<int, TFlt*> ()});
        m_demand_car_subclass[dest].insert (std::make_pair(subclass_label, _demand));
    } 
    else if (mainclass_label == 1) {
        m_demand_truck_subclass.insert ({dest, std::unordered_map<int, TFlt*> ()});
        m_demand_truck_subclass[dest].insert (std::make_pair(subclass_label, _demand));
    } 
    else {
        std::cout << "Error: mainclass_label is not 0 or 1" << std::endl;
        throw std::runtime_error("Invalid mainclass_label");
    }
    return 0;
}

int 
MNM_Origin_Multiclass_Subclass::get_dest_demand_multiclass ()
{ 
    double *_demand;
    for (auto _it : m_demand_car_subclass)
    {   
        _demand = new double[m_max_assign_interval * 15]();
        for (auto _demand_it : _it.second)
        { 
            for (int i = 0; i < m_max_assign_interval * 15; ++i)
            {
                _demand[i] += TFlt (_demand_it.second[i]);
            }
        }
        m_demand_car.insert ({ _it.first, _demand});
    }
    for (auto _it : m_demand_truck_subclass)
    {   
        _demand = new double[m_max_assign_interval * 15]();
        for (auto _demand_it : _it.second)
        { 
            for (int i = 0; i < m_max_assign_interval * 15; ++i)
            {
                _demand[i] += TFlt (_demand_it.second[i]);
            }
        }
        m_demand_truck.insert ({ _it.first, _demand});
    }
    return 0;
}


int
MNM_IO_Multiclass_Subclass::build_demand_subclass (const std::string &file_folder,
                                            MNM_ConfReader *conf_reader,
                                            MNM_OD_Factory *od_factory,
                                            int mainclass_label,
                                            const std::string &file_name)
{
    /* find file */
    std::string _demand_file_name = file_folder + "/" + file_name;
    std::ifstream _demand_file;
    _demand_file.open (_demand_file_name, std::ios::in);

    /* read config */
    TFlt _flow_scalar = conf_reader->get_float ("flow_scalar");
    TInt _unit_time = conf_reader->get_int ("unit_time");
    TInt _num_of_minute = int (conf_reader->get_int ("assign_frq"))
                        / (60 / _unit_time); // the releasing strategy is
                                                // assigning vehicles per 1 minute
    TInt _max_interval = conf_reader->get_int ("max_interval");
    TInt _num_OD = conf_reader->get_int ("OD_pair");
    TInt _init_demand_split = conf_reader->get_int ("init_demand_split");

    int _num_subclass;
    if (mainclass_label == 0) {
        _num_subclass = conf_reader->get_int ("num_subclass_car");
    }
    else if (mainclass_label == 1) {
        _num_subclass = conf_reader->get_int ("num_subclass_truck");
    }

    /* build */
    TInt _O_ID, _D_ID;
    MNM_Origin_Multiclass *_origin;
    MNM_Destination_Multiclass *_dest;
    std::string _line;
    std::vector<std::string> _words;
    if (_demand_file.is_open ())
    {
        // printf("Start build demand profile.\n");
        double *_demand_vector = new double[_max_interval * _num_of_minute]();
        TFlt _demand;

        for (int i = 0; i < _num_OD;)
        {
            std::getline (_demand_file, _line);
            _line = trim (_line);
            if (_line.empty () || _line[0] == '#')
            continue;
            ++i;
            _words = split (_line, ' ');
            if (TInt (_words.size ()) == (_max_interval * _num_subclass + 2))
            {
                _O_ID = TInt (std::stoi (_words[0]));
                _D_ID = TInt (std::stoi (_words[1]));
                memset (_demand_vector, 0x0, sizeof (TFlt) * _max_interval * _num_of_minute);
                // the releasing strategy is assigning vehicles per 1 minute, so
                // disaggregate 15-min demand into 1-min demand
                for (int j = 0; j < _max_interval; ++j)
                {
                    if (_init_demand_split == 0)
                    {
                        for (int q = 0; q < _num_subclass; ++q) {
                            _demand = TFlt (std::stod (_words[j * _num_sub
                            _demand = TFlt (std::stod (_words[j * _num_subclass + 2 + q]));
                            m_demand_car_sub
                        }
                        _demand_car = TFlt (std::stod (_words[j + 2]));
                        _demand_vector_car[j * _num_of_minute] = _demand_car;
                    }
                    else if (_init_demand_split == 1)
                    {
                        // find suitable releasing interval so that the
                        // agent-based DNL is feasible
                        for (int p = 0; p < _num_of_minute; ++p)
                        {
                            _demand_car = TFlt (std::stod (_words[j + 2]))
                                        / TFlt (_num_of_minute - p);
                            // if (round(_demand_car * _flow_scalar) >= 1){
                            if (floor (_demand_car * _flow_scalar) >= 1)
                            {
                                for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                    _demand_vector_car[j * _num_of_minute + k]
                                    = _demand_car;
                                }
                                break;
                            }
                        }
                        for (int p = 0; p < _num_of_minute; ++p)
                        {
                            _demand_truck
                            = TFlt (std::stod (_words[j + _max_interval + 2]))
                                / TFlt (_num_of_minute - p);
                            // if (round(_demand_truck * _flow_scalar) >= 1){
                            if (floor (_demand_truck * _flow_scalar) >= 1)
                            {
                                for (int k = 0; k < _num_of_minute - p; ++k)
                                {
                                    _demand_vector_truck[j * _num_of_minute + k]
                                    = _demand_truck;
                                }
                                break;
                            }
                        }
                    }
                    else
                    {
                        throw std::runtime_error ("wrong init_demand_split");
                    }
                }
                _origin = dynamic_cast<MNM_Origin_Multiclass *> (
                od_factory->get_origin (_O_ID));
                _dest = dynamic_cast<MNM_Destination_Multiclass *> (
                od_factory->get_destination (_D_ID));
                _origin->add_dest_demand_multiclass (_dest, _demand_vector_car,
                                                    _demand_vector_truck);
            }
            else
            {
                delete[] _demand_vector_car;
                delete[] _demand_vector_truck;
                throw std::runtime_error ("failed to build demand");
            }
        }
        delete[] _demand_vector_car;
        delete[] _demand_vector_truck;
        _demand_file.close ();
    }
    return 0;
}