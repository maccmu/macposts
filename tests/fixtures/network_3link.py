"""A toy network with three links."""

import pytest


@pytest.fixture(scope="session")
def network_3link(tmp_path_factory):
    config = """\
[DTA]
network_name = Snap_graph
total_interval = 200
unit_time = 5
assign_frq = 20
start_assign_interval = 0
max_interval = 10
flow_scalar = 1
num_of_link = 3
num_of_node = 4
num_of_O = 1
num_of_D = 1
OD_pair = 1

routing_type = Adaptive

[STAT]
rec_mode = LRn
rec_mode_para = 5
rec_folder = record

rec_volume = 1
volume_load_automatic_rec = 0
volume_record_automatic_rec = 0

rec_tt = 1
tt_load_automatic_rec = 0
tt_record_automatic_rec = 0

[ADAPTIVE]
route_frq = 20
"""
    graph = """\
#e f t
 2 1 2
 3 2 3
 4 3 4
"""
    nodes = """\
1 DMOND
2 FWJ
3 FWJ
4 DMDND
"""
    links = """\
2 PQ 1   99999 99999 99999 1
3 LQ 0.4 45    2200  200   1
4 PQ 1   99999 99999 99999 1
"""
    ods = """\
# origins
1 1
# destinations
1 4
"""
    demands = """\
1 1 100 100 100 100 100 0 0 0 0 0
"""
    base_dir = tmp_path_factory.mktemp("network_3link")
    for name, contents in [
        ("config.conf", config),
        ("Snap_graph", graph),
        ("MNM_input_demand", demands),
        ("MNM_input_link", links),
        ("MNM_input_node", nodes),
        ("MNM_input_od", ods),
    ]:
        with (base_dir / name).open("w") as f:
            f.write(contents)
    return base_dir


@pytest.fixture(scope="session")
def network_3link_mc(tmp_path_factory):
    config = """\
[DTA]
network_name = Snap_graph
total_interval = 200
unit_time = 5
assign_frq = 20
start_assign_interval = 0
max_interval = 10
flow_scalar = 1
num_of_link = 3
num_of_node = 4
num_of_O = 1
num_of_D = 1
OD_pair = 1

routing_type = Adaptive

[STAT]
rec_mode = LRn
rec_mode_para = 5
rec_folder = record

rec_volume = 1
volume_load_automatic_rec = 0
volume_record_automatic_rec = 0

rec_tt = 1
tt_load_automatic_rec = 0
tt_record_automatic_rec = 0

[ADAPTIVE]
route_frq = 20
"""
    graph = """\
#e f t
 2 1 2
 3 2 3
 4 3 4
"""
    nodes = """\
1 DMOND 2.1
2 FWJ 2.1
3 FWJ 2.1
4 DMDND 2.1
"""
    links = """\
2 PQ 1   99999 99999 99999 1 99999 99999 99999 2.1
3 LQ 0.4 45    2200  200   1 25    1000  100   2.1
4 PQ 1   99999 99999 99999 1 99999 99999 99999 2.1
"""
    ods = """\
# origins
1 1
# destinations
1 4
"""
    demands = """\
1 1 100 100 100 100 100 0 0 0 0 0 20 20 20 20 20 0 0 0 0 0
"""
    base_dir = tmp_path_factory.mktemp("network_3link_mc")
    for name, contents in [
        ("config.conf", config),
        ("Snap_graph", graph),
        ("MNM_input_demand", demands),
        ("MNM_input_link", links),
        ("MNM_input_node", nodes),
        ("MNM_input_od", ods),
    ]:
        with (base_dir / name).open("w") as f:
            f.write(contents)
    return base_dir
