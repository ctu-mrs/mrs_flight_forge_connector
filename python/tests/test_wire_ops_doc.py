"""docs/wire-1.0-ops.md in the plugin repo must map every 0.x op and name every 1.0 request; skips without the plugin repo."""

import re
from pathlib import Path

import pytest

pytest.importorskip("flatbuffers")

from flightforge.wire.RequestBody import RequestBody
from flightforge.wire.ResponseBody import ResponseBody

SHARED_HEADER = Path(__file__).resolve().parents[2] / "include" / "flight_forge_connector" / "serialization" / "serializable_shared.h"
OPS_DOC = Path(__file__).resolve().parents[3] / "docs" / "wire-1.0-ops.md"


def members(union):
    return {name for name, value in vars(union).items() if isinstance(value, int) and name != "NONE"}


@pytest.fixture(scope="module")
def doc():
    if not OPS_DOC.exists():
        pytest.skip("plugin repo not present next to the connector")
    return OPS_DOC.read_text()


def test_every_old_op_is_mapped(doc):
    ops = re.findall(r"FF_OP\(\w+, \w+, (\w+)\);", SHARED_HEADER.read_text())
    assert len(ops) > 70
    missing = [op for op in ops if f"| `{op}` |" not in doc]
    assert not missing, f"not in {OPS_DOC.name}: {missing}"


def test_every_request_is_named(doc):
    missing = sorted(name for name in members(RequestBody) if f"`{name}`" not in doc)
    assert not missing, f"not in {OPS_DOC.name}: {missing}"


def test_doc_names_only_real_messages(doc):
    named = set(re.findall(r"`(\w+(?:Request|Response))`", doc))
    unknown = sorted(named - members(RequestBody) - members(ResponseBody))
    assert not unknown, f"{OPS_DOC.name} names messages the schema does not have: {unknown}"
