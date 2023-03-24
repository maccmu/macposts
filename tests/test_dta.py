import macposts
import numpy as np
import platform
import pytest
from .conftest import SEED


@pytest.mark.xfail(
    platform.system() == "Darwin",
    reason="failed for unknown reasons on Darwin platform",
)
@pytest.mark.parametrize("network", ["network_3link", "network_7link"])
def test_reproducibility(network, request):
    in_ccs, out_ccs = None, None
    network = request.getfixturevalue(network)
    for _ in range(10):
        macposts.set_random_state(SEED)
        dta = macposts.Dta.from_files(network)
        dta.register_links()
        dta.install_cc()
        dta.run_whole()
        in_ccs_ = dta.get_in_ccs()
        out_ccs_ = dta.get_out_ccs()
        if in_ccs is not None:
            assert np.allclose(in_ccs, in_ccs_)
            assert np.allclose(out_ccs, out_ccs_)
        in_ccs, out_ccs = in_ccs_, out_ccs_


def test_3link(network_3link):
    macposts.set_random_state(SEED)
    links = [2, 3, 4]

    dta = macposts.Dta.from_files(network_3link)
    assert links == list(dta.links)
    dta.register_links(links)
    assert links == list(dta.registered_links)

    dta.install_cc()
    dta.run_whole()

    in_cc = dta.get_in_ccs([links[0]])
    in_cc_ = dta.get_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)

    in_ccs = dta.get_in_ccs()
    out_ccs = dta.get_out_ccs(links)
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
    assert links == list(dta.links)
    dta.register_links(links)
    assert links == list(dta.registered_links)

    dta.install_cc()
    dta.run_whole()

    in_cc = dta.get_in_ccs([links[0]])
    in_cc_ = dta.get_link_in_cc(links[0])
    ticks = in_cc_[:, 0].astype(int)
    in_cc_ = in_cc_[:, 1]
    assert np.all(in_cc[ticks, 0] == in_cc_)

    in_ccs = dta.get_in_ccs(links)
    out_ccs = dta.get_out_ccs()
    assert in_ccs.shape[1] == 7
    assert out_ccs.shape == in_ccs.shape
    assert np.isclose(out_ccs[0, 0], 0)
