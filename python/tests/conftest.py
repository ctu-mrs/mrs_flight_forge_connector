import pytest

from fake_simulator import FakeSimulator


@pytest.fixture(scope="session")
def sim():
    """A live simulator on FLIGHTFORGE_PORT (default 8600); the sim-marked suite skips when none is reachable."""
    flightforge = pytest.importorskip("flightforge")
    try:
        simulator = flightforge.Simulator()
    except flightforge.FlightForgeConnectionError as error:
        pytest.skip(f"no simulator reachable: {error}")
    yield simulator
    simulator.close()


@pytest.fixture()
def robot(sim):
    r = sim.spawn_robot((0.0, 0.0, 2.0), "x500", label="test")
    yield r
    sim.remove_robot(r)
    r.close()


@pytest.fixture()
def fake():
    """A factory for local fake simulators, closed after the test."""
    servers = []

    def make(handler=None):
        server = FakeSimulator(handler)
        servers.append(server)
        return server

    yield make
    for server in servers:
        server.close()
