import macposts
import numpy as np
from .conftest import SEED


def test_3link_mc(network_3link_mc):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    mcdta = macposts.Mcdta()
    mcdta.initialize(str(network_3link_mc))
    mcdta.register_links(links)
    mcdta.install_cc()
    mcdta.run_whole()

    car_in_ccs = np.hstack([mcdta.get_car_link_in_cc(link) for link in links])[
        :, 1::2
    ]
    car_out_ccs = np.hstack(
        [mcdta.get_car_link_out_cc(link) for link in links]
    )[:, 1::2]
    truck_in_ccs = np.hstack(
        [mcdta.get_truck_link_in_cc(link) for link in links]
    )[:, 1::2]
    truck_out_ccs = np.hstack(
        [mcdta.get_truck_link_out_cc(link) for link in links]
    )[:, 1::2]

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
    mcdta.install_cc()
    mcdta.run_whole()

    car_in_ccs = np.hstack([mcdta.get_car_link_in_cc(link) for link in links])[
        :, 1::2
    ]
    car_out_ccs = np.hstack(
        [mcdta.get_car_link_out_cc(link) for link in links]
    )[:, 1::2]
    assert car_in_ccs.shape[1] == 7
    assert car_out_ccs.shape == car_in_ccs.shape
    assert np.isclose(car_in_ccs[0, 0], 0)
