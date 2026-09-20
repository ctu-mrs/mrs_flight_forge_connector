"""The connector's serializable_shared.h and serializable_schema.h must be
byte-identical mirrors of the plugin's. Runs only where the plugin repo sits
next to the connector checkout (the development worktree); everywhere else it
skips."""

from pathlib import Path

import pytest

CONNECTOR_DIR = Path(__file__).resolve().parents[2] / "include" / "flight_forge_connector" / "serialization"
PLUGIN_DIR = Path(__file__).resolve().parents[3] / "FlightForgePlugin" / "Source" / "MessageSerialization" / "Public"


@pytest.mark.parametrize("name", ["serializable_shared.h", "serializable_schema.h"])
def test_header_mirrors_plugin(name):
    plugin_header = PLUGIN_DIR / name
    if not plugin_header.exists():
        pytest.skip("plugin repo not present next to the connector")

    connector_header = CONNECTOR_DIR / name
    assert connector_header.read_bytes() == plugin_header.read_bytes(), (
        f"{name} has drifted from the plugin's copy - re-mirror it (cp {plugin_header} {connector_header})"
    )
