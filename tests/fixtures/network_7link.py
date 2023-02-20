"""A toy network with seven links.

"""

import pytest


@pytest.fixture(scope="session")
def network_7link(tmp_path_factory):
    config = """\
[DTA]
network_name = Snap_graph
unit_time = 5
total_interval = -1
assign_frq = 180
start_assign_interval = 0
max_interval = 10
flow_scalar = 10
num_of_link = 7
num_of_node = 6
num_of_O = 1
num_of_D = 1
OD_pair = 1

adaptive_ratio = 0.5
routing_type = Hybrid

[STAT]
rec_mode = LRn
rec_mode_para = 12
rec_folder = record
rec_volume = 1
volume_load_automatic_rec = 0
volume_record_automatic_rec = 0
rec_tt = 1
tt_load_automatic_rec = 0
tt_record_automatic_rec = 0

[HYBRID]
route_frq = 180

[FIXED]
path_file_name = path_table
num_path = 3
choice_portion = Buffer
buffer_length = 10
route_frq = 180

[ADAPTIVE]
route_frq = 180
"""
    graph = """\
1 1 2
2 2 3
3 2 4
4 3 5
5 4 5
6 3 4
7 5 6
"""
    links = """\
1 PQ 1 99999 99999 99999 1
2 CTM 1.55 35 600 40 2
3 CTM 1.55 35 600 40 2
4 CTM 1.55 35 600 40 1
5 CTM 1.55 35 600 40 1
6 CTM 1.55 35 600 40 1
7 PQ 1 99999 99999 99999 1
"""
    nodes = """\
1 DMOND
2 FWJ
3 FWJ
4 FWJ
5 FWJ
6 DMDND
"""
    ods = """\
# origins
1 1
# destination
1 6
"""
    demands = """\
1 1 100 100 300 100 200 100 200 100 100 100
"""
    path_table = """\
1 2 3 5 6
1 2 4 5 6
1 2 3 4 5 6
"""
    path_table_buffer = """\
30 30 30 30 30 10 10 10 10 33
40 40 40 40 40 10 10 10 10 33
40 40 40 40 40 80 80 80 80 34
"""
    base_dir = tmp_path_factory.mktemp("network_sevenlink")
    for name, contents in [
        ("config.conf", config),
        ("Snap_graph", graph),
        ("path_table", path_table),
        ("path_table_buffer", path_table_buffer),
        ("MNM_input_demand", demands),
        ("MNM_input_link", links),
        ("MNM_input_node", nodes),
        ("MNM_input_od", ods),
    ]:
        with (base_dir / name).open("w") as f:
            f.write(contents)
    return base_dir


@pytest.fixture(scope="session")
def network_7link_mc(tmp_path_factory):
    config = """\
[DTA]
network_name = Snap_graph
unit_time = 5
total_interval = -1
assign_frq = 180
start_assign_interval = 0
max_interval = 10
flow_scalar = 10
num_of_link = 7
num_of_node = 6
num_of_O = 1
num_of_D = 1
OD_pair = 1
adaptive_ratio_car = 0.5
adaptive_ratio_truck = 0.5
routing_type = Biclass_Hybrid

[STAT]
rec_mode = LRn
rec_mode_para = 12
rec_folder = record
rec_volume = 1
volume_load_automatic_rec = 0
volume_record_automatic_rec = 0
rec_tt = 1
tt_load_automatic_rec = 0
tt_record_automatic_rec = 0

[HYBRID]
route_frq = 180

[FIXED]
path_file_name = path_table
num_path = 3
choice_portion = Buffer
buffer_length = 10
route_frq = 180

[ADAPTIVE]
route_frq = 180
"""
    graph = """\
1 1 2
2 2 3
3 2 4
4 3 5
5 4 5
6 3 4
7 5 6
"""
    links = """\
1 PQ 1 99999 99999 99999 1 99999 99999 99999 2.1
2 CTM 1.55 35 600 40 2 25 200 20 2.1
3 CTM 1.55 35 600 40 2 25 200 20 2.1
4 CTM 1.55 35 600 40 1 25 200 20 2.1
5 CTM 1.55 35 600 40 1 25 200 20 2.1
6 CTM 1.55 35 600 40 1 25 200 20 2.1
7 PQ 1 99999 99999 99999 1 99999 99999 99999 2.1
"""
    nodes = """\
1 DMOND 2.1
2 FWJ 2.1
3 FWJ 2.1
4 FWJ 2.1
5 FWJ 2.1
6 DMDND 2.1
"""
    ods = """\
# origins
1 1
# destination
1 6
"""
    demands = """\
1 1 100 100 300 100 200 100 200 100 100 100 10 9 30 7 12 7 8 10 9 11
"""
    path_table = """\
1 2 3 5 6
1 2 4 5 6
1 2 3 4 5 6
"""
    path_table_buffer = """\
30 30 30 30 30 10 10 10 10 33 30 30 30 30 30 10 10 10 10 33
40 40 40 40 40 10 10 10 10 33 40 40 40 40 40 10 10 10 10 33
40 40 40 40 40 80 80 80 80 34 40 40 40 40 40 80 80 80 80 33
"""
    base_dir = tmp_path_factory.mktemp("network_sevenlink")
    for name, contents in [
        ("config.conf", config),
        ("Snap_graph", graph),
        ("path_table", path_table),
        ("path_table_buffer", path_table_buffer),
        ("MNM_input_demand", demands),
        ("MNM_input_link", links),
        ("MNM_input_node", nodes),
        ("MNM_input_od", ods),
    ]:
        with (base_dir / name).open("w") as f:
            f.write(contents)
    return base_dir
