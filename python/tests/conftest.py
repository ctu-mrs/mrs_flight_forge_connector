import pytest


@pytest.fixture(scope="session")
def sim():
    """A live simulator connection; the whole sim-marked suite skips when none is reachable."""
    flightforge = pytest.importorskip("flightforge")
    try:
        simulator = flightforge.Simulator()
    except Exception as error:
        pytest.skip(f"no simulator reachable: {error}")
    yield simulator
    simulator.close()


@pytest.fixture()
def drone(sim):
    d = sim.spawn_drone((0.0, 0.0, 200.0), "x500")
    yield d
    sim.remove_drone(d)
    d.close()
