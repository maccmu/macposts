import pytest
import macposts
from macposts.graph import Direction


def test_basics():
    g = macposts.Graph()
    num_nodes = 10
    for idx in range(num_nodes):
        g.add_node(idx)
    for idx, (f, t) in enumerate(
        zip(range(0, num_nodes - 1), range(1, num_nodes))
    ):
        g.add_link(f, t, idx)

    assert g.size_nodes == num_nodes
    assert g.size_links == num_nodes - 1
    for idx in range(g.size_nodes):
        assert g.get_id(g.get_node(idx)) == idx
    for idx in range(g.size_links):
        assert g.get_id(g.get_link(idx)) == idx

    for idx in range(g.size_links):
        src, dst = g.get_endpoints(idx)
        assert src == idx
        assert dst == idx + 1

    assert len(g.nodes()) == g.size_nodes
    for idx in range(num_nodes):
        assert idx in g.nodes()

    assert len(g.links()) == g.size_links
    for idx in range(num_nodes - 1):
        assert idx in g.links()

    for idx in range(g.size_nodes):
        assert (
            len(g.neighbors(idx, Direction.Outgoing)) == 1
            or idx == g.size_nodes - 1
        )
        if idx < g.size_nodes - 1:
            assert g.neighbors(idx, Direction.Outgoing) == [idx + 1]
        assert len(g.neighbors(idx, Direction.Incoming)) == 1 or idx == 0
        if idx > 0:
            assert g.neighbors(idx, Direction.Incoming) == [idx - 1]

    for idx in range(g.size_nodes):
        assert (
            len(g.connections(idx, Direction.Outgoing)) == 1
            or idx == g.size_nodes - 1
        )
        if idx < g.size_nodes - 1:
            assert g.connections(idx, Direction.Outgoing) == [idx]
        assert len(g.connections(idx, Direction.Incoming)) == 1 or idx == 0
        if idx > 0:
            assert g.connections(idx, Direction.Incoming) == [idx - 1]

    with pytest.raises(IndexError):
        g.get_node(num_nodes)
    with pytest.raises(IndexError):
        g.get_link(num_nodes - 1)
    with pytest.raises(RuntimeError):
        g.add_node(0)
    with pytest.raises(RuntimeError):
        g.add_link(0, 1, 0)


def test_parallel_links():
    g = macposts.Graph()
    g.add_node(0)
    g.add_node(1)
    g.add_link(0, 1, 0)
    g.add_link(0, 1, 1)
    g.add_link(0, 1, 2)

    assert len(g.neighbors(0, Direction.Incoming)) == 0
    assert len(g.neighbors(0, Direction.Outgoing)) == 1
    assert g.neighbors(0, Direction.Outgoing) == [1]
    assert len(g.neighbors(1, Direction.Incoming)) == 1
    assert g.neighbors(1, Direction.Incoming) == [0]
    assert len(g.neighbors(1, Direction.Outgoing)) == 0

    assert len(g.connections(0, Direction.Incoming)) == 0
    assert len(g.connections(0, Direction.Outgoing)) == 3
    for idx in range(3):
        assert idx in g.connections(0, Direction.Outgoing)
    assert len(g.connections(1, Direction.Incoming)) == 3
    for idx in range(3):
        assert idx in g.connections(1, Direction.Incoming)
    assert len(g.connections(1, Direction.Outgoing)) == 0


def test_cycles():
    g = macposts.Graph()
    g.add_node(0)
    g.add_node(1)
    g.add_link(0, 1, 0)
    g.add_link(1, 0, 1)

    assert g.neighbors(0, Direction.Incoming) == [1]
    assert g.neighbors(0, Direction.Outgoing) == [1]
    assert g.neighbors(1, Direction.Incoming) == [0]
    assert g.neighbors(1, Direction.Outgoing) == [0]

    assert g.connections(0, Direction.Incoming) == [1]
    assert g.connections(0, Direction.Outgoing) == [0]
    assert g.connections(1, Direction.Incoming) == [0]
    assert g.connections(1, Direction.Outgoing) == [1]
