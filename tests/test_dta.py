import macposts
import numpy as np
from .conftest import SEED


def test_3link(network_3link):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    dta = macposts.Dta.from_files(network_3link)
    dta.register_links(links)
    assert links == list(dta.registered_links)

    dta.install_cc()
    dta.run_whole()

    _, in_cc = dta.get_in_ccs(links[0])
    in_cc_ = dta.get_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)

    ticks, in_ccs = dta.get_in_ccs()
    ticks_, out_ccs = dta.get_out_ccs(links)
    assert np.all(ticks == ticks_)
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
    assert links == list(dta.registered_links)

    dta.install_cc()
    dta.run_whole()

    _, in_cc = dta.get_in_ccs(links[0])
    in_cc_ = dta.get_link_in_cc(links[0])[:, 1::2]
    assert np.all(in_cc == in_cc_)

    ticks, in_ccs = dta.get_in_ccs(links)
    ticks_, out_ccs = dta.get_out_ccs()
    assert np.all(ticks == ticks_)
    assert in_ccs.shape[1] == 7
    assert out_ccs.shape == in_ccs.shape
    assert np.isclose(out_ccs[0, 0], 0)
