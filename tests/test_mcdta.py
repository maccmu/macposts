import shutil
import macposts
import numpy as np
import platform
import pytest
from .conftest import SEED, NUM_REPRO_RUNS


@pytest.mark.xfail(
    platform.system() == "Darwin",
    reason="failed for unknown reasons on Darwin platform (GH-28)",
)
@pytest.mark.parametrize("network", ["network_3link_mc", "network_7link_mc"])
def test_reproducibility(network, request):
    car_in_ccs, car_out_ccs = None, None
    truck_in_ccs, truck_out_ccs = None, None
    network = request.getfixturevalue(network)
    for _ in range(NUM_REPRO_RUNS):
        macposts.set_random_state(SEED)
        mcdta = macposts.Mcdta.from_files(network)
        mcdta.register_links()
        mcdta.install_cc()
        mcdta.run_whole()
        car_in_ccs_ = mcdta.get_car_in_ccs()
        car_out_ccs_ = mcdta.get_car_out_ccs()
        truck_in_ccs_ = mcdta.get_truck_in_ccs()
        truck_out_ccs_ = mcdta.get_truck_out_ccs()
        if car_in_ccs is not None:
            assert np.all(car_in_ccs == car_in_ccs_)
            assert np.all(car_out_ccs == car_out_ccs_)
            assert np.all(truck_in_ccs == truck_in_ccs_)
            assert np.all(truck_out_ccs == truck_out_ccs_)
        car_in_ccs, car_out_ccs = car_in_ccs_, car_out_ccs_
        truck_in_ccs, truck_out_ccs = truck_in_ccs_, truck_out_ccs_


def test_3link_mc(network_3link_mc):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    mcdta = macposts.Mcdta.from_files(network_3link_mc)
    assert links == list(mcdta.links)
    mcdta.register_links(links)
    assert links == list(mcdta.registered_links)

    mcdta.install_cc()
    mcdta.run_whole()

    in_cc = mcdta.get_car_in_ccs([links[0]])
    in_cc_ = mcdta.get_car_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)
    in_cc = mcdta.get_truck_in_ccs([links[0]])
    in_cc_ = mcdta.get_truck_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)

    car_in_ccs = mcdta.get_car_in_ccs()
    car_out_ccs = mcdta.get_car_out_ccs(links)
    truck_in_ccs = mcdta.get_truck_in_ccs(links)
    truck_out_ccs = mcdta.get_truck_out_ccs()
    assert car_in_ccs.shape == (241, 3)
    assert car_out_ccs.shape == car_in_ccs.shape
    assert truck_in_ccs.shape == car_in_ccs.shape
    assert truck_out_ccs.shape == truck_out_ccs.shape
    assert np.isclose(car_in_ccs[-1, 0], 500)
    assert np.isclose(truck_in_ccs[-1, 0], 100)
    assert np.isclose(car_out_ccs[0, 0], 0)
    assert np.isclose(truck_out_ccs[0, 0], 0)


def test_7link_mc(network_7link_mc):
    macposts.set_random_state(SEED)
    links = list(range(1, 8))

    mcdta = macposts.Mcdta()
    mcdta.initialize(str(network_7link_mc))
    assert links == list(mcdta.links)
    mcdta.register_links(links)
    assert links == list(mcdta.registered_links)

    mcdta.install_cc()
    mcdta.run_whole()

    in_cc = mcdta.get_car_in_ccs([links[0]])
    in_cc_ = mcdta.get_car_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)
    in_cc = mcdta.get_truck_in_ccs([links[0]])
    in_cc_ = mcdta.get_truck_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)

    car_in_ccs = mcdta.get_car_in_ccs()
    car_out_ccs = mcdta.get_car_out_ccs(links)
    truck_in_ccs = mcdta.get_truck_in_ccs(links)
    truck_out_ccs = mcdta.get_truck_out_ccs()
    assert car_in_ccs.shape == (mcdta.get_cur_loading_interval() + 1, 7)
    assert car_out_ccs.shape == car_in_ccs.shape
    assert truck_in_ccs.shape == car_in_ccs.shape
    assert truck_out_ccs.shape == truck_out_ccs.shape
    assert np.isclose(car_in_ccs[0, 0], 0)


def test_demand_split_keeps_sub_vehicle_demand(network_7link_mc, tmp_path):
    """init_demand_split=1 must not discard demand below one vehicle a minute.

    The disaggregation loop looks for a releasing window wide enough that every
    minute carries at least one (flow-scalar-inflated) vehicle. When even the
    whole interval packed into a single minute stays under one vehicle, it used
    to fall out of the loop having written nothing, silently dropping that OD's
    demand. Here flow_scalar = 10 and the demand is 0.09 per interval, i.e. 0.9
    vehicles -- always under the threshold, so every interval took that path.
    """
    net = tmp_path / "net"
    shutil.copytree(network_7link_mc, net)
    config = (net / "config.conf").read_text()
    assert "init_demand_split = 0" in config
    (net / "config.conf").write_text(
        config.replace("init_demand_split = 0", "init_demand_split = 1")
    )
    (net / "MNM_input_demand").write_text("1 1 " + " ".join(["0.09"] * 20) + "\n")

    macposts.set_random_state(SEED)
    mcdta = macposts.Mcdta.from_files(net)
    mcdta.register_links()
    mcdta.install_cc()
    mcdta.run_whole()

    total_car = mcdta.get_car_in_ccs([1])[-1, 0]
    total_truck = mcdta.get_truck_in_ccs([1])[-1, 0]
    assert total_car > 0
    assert total_truck > 0
