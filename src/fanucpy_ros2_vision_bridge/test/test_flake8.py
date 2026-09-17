# Copyright 2026 Muhammad Ureed Hussain
# SPDX-License-Identifier: Apache-2.0

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


ROOT = Path(__file__).parents[1]
CONFIG = Path(__file__).with_name("ament_flake8.ini")


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check Python source against the workspace style rules."""
    return_code, errors = main_with_errors(
        argv=["--config", str(CONFIG), str(ROOT)]
    )
    assert return_code == 0, "\n".join(errors)
