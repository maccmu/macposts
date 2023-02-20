import macposts
import numpy as np
from .conftest import SEED


def test_3link(network_3link):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    dta = macposts.Dta()
    dta.initialize(str(network_3link))
    dta.register_links(links)
    dta.install_cc()
    dta.run_whole()

    in_ccs = np.hstack([dta.get_link_in_cc(link) for link in links])[:, 1::2]
    out_ccs = np.hstack([dta.get_link_out_cc(link) for link in links])[:, 1::2]
    assert in_ccs.shape == (201, 3)
    assert out_ccs.shape == in_ccs.shape
    assert np.isclose(in_ccs[-1, 0], 500)
    assert np.isclose(out_ccs[0, 0], 0)
    assert np.allclose(out_ccs[:, :-1], in_ccs[:, 1:])


def test_7link(network_7link):
    macposts.set_random_state(SEED)
    links = list(range(1, 8))

    dta = macposts.Dta()
    dta.initialize(str(network_7link))
    dta.register_links(links)
    dta.install_cc()
    dta.run_whole()

    in_ccs = np.hstack([dta.get_link_in_cc(link) for link in links])[:, 1::2]
    out_ccs = np.hstack([dta.get_link_out_cc(link) for link in links])[:, 1::2]
    assert in_ccs.shape[1] == 7
    assert out_ccs.shape == in_ccs.shape
    assert np.isclose(out_ccs[0, 0], 0)
