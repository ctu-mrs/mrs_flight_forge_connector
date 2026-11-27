"""The generated bindings must be what the plugin's schema produces. Runs only where the plugin repo sits next to
the connector checkout (the development worktree); everywhere else it skips."""

import filecmp
import os
import subprocess
from pathlib import Path

import pytest

CONNECTOR = Path(__file__).resolve().parents[2]
PLUGIN = CONNECTOR.parent
GENERATE = PLUGIN / "tools" / "generate_wire.sh"
FLATC_CACHE = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "flightforge-flatc"


def test_cpp_bindings_mirror_the_plugin():
    plugin = PLUGIN / "FlightForgePlugin" / "Source" / "MessageSerialization" / "Public" / "Generated"
    if not plugin.is_dir():
        pytest.skip("plugin repo not present next to the connector")

    connector = CONNECTOR / "include" / "flight_forge_connector" / "wire"
    comparison = filecmp.dircmp(plugin, connector)
    assert not comparison.left_only and not comparison.right_only
    _, mismatch, errors = filecmp.cmpfiles(plugin, connector, comparison.common_files, shallow=False)
    assert not mismatch and not errors, f"drifted from the plugin: {mismatch + errors}"


def test_python_bindings_are_generated_fresh():
    if not GENERATE.exists():
        pytest.skip("plugin repo not present next to the connector")
    if not list(FLATC_CACHE.glob("*/bin/flatc")):
        pytest.skip("the pinned flatc is not built yet; tools/generate_wire.sh builds it")

    environment = dict(os.environ, FF_WIRE_CHECK="1", FF_CONNECTOR_DIR=str(CONNECTOR))
    result = subprocess.run([str(GENERATE)], env=environment, capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stderr
