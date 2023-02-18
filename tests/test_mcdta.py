import macposts
import numpy as np
from .conftest import SEED


def test_sevenlink_mc(network_sevenlink_mc):
    macposts.set_random_state(SEED)
    links = list(range(1, 8))

    mcdta = macposts.Mcdta()
    mcdta.initialize(str(network_sevenlink_mc))
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
