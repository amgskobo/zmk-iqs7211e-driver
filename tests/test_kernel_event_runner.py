# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
"""Run the inexpensive event-runner admission controls in canonical host/CI tests."""
import importlib.util
from pathlib import Path


def load_tests(loader, tests, pattern):
    path = Path(__file__).parent/'kernel/events/test_runner.py'
    spec = importlib.util.spec_from_file_location('kernel_event_runner_controls', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return loader.loadTestsFromModule(module)
