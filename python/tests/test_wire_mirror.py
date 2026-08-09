"""The connector's serializable_shared.h must be a byte-identical mirror of the
plugin's. Runs only where the plugin repo sits next to the connector checkout
(the development worktree); everywhere else it skips."""

from pathlib import Path

import pytest

CONNECTOR_HEADER = Path(__file__).resolve().parents[2] / "include" / "flight_forge_connector" / "serialization" / "serializable_shared.h"

PLUGIN_HEADER_CANDIDATES = [
    Path(__file__).resolve().parents[3] / "FlightForgePlugin" / "Source" / "MessageSerialization" / "Public" / "serializable_shared.h",
]


def test_header_mirrors_plugin():
    plugin_header = next((p for p in PLUGIN_HEADER_CANDIDATES if p.exists()), None)
    if plugin_header is None:
        pytest.skip("plugin repo not present next to the connector")

    assert CONNECTOR_HEADER.read_bytes() == plugin_header.read_bytes(), (
        "serializable_shared.h has drifted from the plugin's copy - re-mirror it "
        f"(cp {plugin_header} {CONNECTOR_HEADER})"
    )
