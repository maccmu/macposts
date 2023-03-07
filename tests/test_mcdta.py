import macposts
import numpy as np
from .conftest import SEED


def test_3link_mc(network_3link_mc):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    mcdta = macposts.Mcdta.from_files(network_3link_mc)
    mcdta.register_links(links)
    assert links == list(mcdta.registered_links)

    mcdta.install_cc()
    mcdta.run_whole()

    _, in_cc = mcdta.get_car_in_ccs(links[0])
    in_cc_ = mcdta.get_car_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)
    _, in_cc = mcdta.get_truck_in_ccs(links[0])
    in_cc_ = mcdta.get_truck_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)

    ticks, car_in_ccs = mcdta.get_car_in_ccs()
    ticks_, car_out_ccs = mcdta.get_car_out_ccs(links)
    assert np.all(ticks == ticks_)
    ticks, truck_in_ccs = mcdta.get_truck_in_ccs(links)
    ticks_, truck_out_ccs = mcdta.get_truck_out_ccs()

    assert car_in_ccs.shape == (201, 3)
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
    mcdta.register_links(links)
    assert links == list(mcdta.registered_links)

    mcdta.install_cc()
    mcdta.run_whole()

    _, in_cc = mcdta.get_car_in_ccs(links[0])
    in_cc_ = mcdta.get_car_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)
    _, in_cc = mcdta.get_truck_in_ccs(links[0])
    in_cc_ = mcdta.get_truck_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)

    ticks, car_in_ccs = mcdta.get_car_in_ccs(links)
    ticks_, car_out_ccs = mcdta.get_car_out_ccs()
    assert np.all(ticks == ticks_)
    assert car_in_ccs.shape[1] == 7
    assert car_out_ccs.shape == car_in_ccs.shape
    assert np.isclose(car_in_ccs[0, 0], 0)
