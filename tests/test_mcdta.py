import macposts
import numpy as np
import platform
import pytest
from .conftest import SEED


@pytest.mark.xfail(
    platform.system() == "Darwin",
    reason="failed for unknown reasons on Darwin platform",
)
@pytest.mark.parametrize("network", ["network_3link_mc", "network_7link_mc"])
def test_reproducibility(network, request):
    car_in_ccs, car_out_ccs = None, None
    truck_in_ccs, truck_out_ccs = None, None
    network = request.getfixturevalue(network)
    for _ in range(10):
        macposts.set_random_state(SEED)
        mcdta = macposts.Mcdta.from_files(network)
        mcdta.register_links()
        mcdta.install_cc()
        mcdta.run_whole()
        car_in_ccs_ = mcdta.get_car_in_ccs()[1]
        car_out_ccs_ = mcdta.get_car_out_ccs()[1]
        truck_in_ccs_ = mcdta.get_truck_in_ccs()[1]
        truck_out_ccs_ = mcdta.get_truck_out_ccs()[1]
        if car_in_ccs is not None:
            assert np.allclose(car_in_ccs, car_in_ccs_)
            assert np.allclose(car_out_ccs, car_out_ccs_)
            assert np.allclose(truck_in_ccs, truck_in_ccs_)
            assert np.allclose(truck_out_ccs, truck_out_ccs_)
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
