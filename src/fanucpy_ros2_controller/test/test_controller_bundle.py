# Copyright 2026 Muhammad Ureed Hussain
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import hashlib
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BUNDLE_ROOT = PACKAGE_ROOT / "fanuc_driver"


def _manifest_entries():
    entries = {}
    manifest = BUNDLE_ROOT / "SHA256SUMS"
    for line in manifest.read_text(encoding="utf-8").splitlines():
        digest, filename = line.split("  ", maxsplit=1)
        entries[filename] = digest
    return entries


def test_controller_bundle_contains_only_reviewed_files():
    expected = {
        "SHA256SUMS",
        "mappdk.ls",
        "mappdk_cmd.kl",
        "mappdk_comm.kl",
        "mappdk_logger.kl",
        "mappdk_logger.pc",
        "mappdk_move.ls",
        "mappdk_movel.ls",
        "mappdk_server.kl",
        "mappdk_server.pc",
        "mappdk_utils.kl",
    }
    assert {
        path.name for path in BUNDLE_ROOT.iterdir() if path.is_file()
    } == expected


def test_controller_bundle_matches_manifest():
    entries = _manifest_entries()
    assert len(entries) == 10
    for filename, expected_digest in entries.items():
        contents = (BUNDLE_ROOT / filename).read_bytes()
        assert hashlib.sha256(contents).hexdigest() == expected_digest


def test_supplied_mappdk_command_capabilities_are_explicit():
    command_source = (BUNDLE_ROOT / "mappdk_cmd.kl").read_text(
        encoding="utf-8"
    )
    for command in (
        "curpos",
        "curjpos",
        "ins_pwr",
        "movej",
        "movep",
        "mappdkcall",
        "setrdo",
        "getrdo",
        "setdout",
        "getdout",
        "setsysvar",
    ):
        assert f"= '{command}'" in command_source

    for extension in ("setregint", "setregflt", "getreg", "setpr", "getpr"):
        assert extension not in command_source
