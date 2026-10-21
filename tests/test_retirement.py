"""Prevent retired timing/stack/FLOW instrumentation from re-entering the driver."""
from pathlib import Path
import unittest


def retirement_errors(root):
    errors = []
    forbidden = ("IQS7211E_REPORT_TIMING", "IQS7211E_WORKQUEUE_STACK_USAGE", "IQS_TIMING_",
                 "iqs7211e_timing", "diagnostic_log_ms", "FLOW irq=", "work queue stack high-water")
    for path in [root / "Kconfig", *(root / "src").glob("*")]:
        if path.is_file():
            for token in forbidden:
                if token in path.read_text():
                    errors.append(f"{path.name}: {token}")
    return errors


class RetirementTests(unittest.TestCase):
    def test_no_retired_collectors_or_configuration(self):
        self.assertEqual(retirement_errors(Path(__file__).resolve().parents[1]), [])

    def test_boot_recovery_state_is_retained(self):
        source = (Path(__file__).resolve().parents[1] / "src/iqs7211e.c").read_text()
        self.assertIn("atomic_set(&data->boot_irq_seen, 1)", source)
        self.assertIn("atomic_get(&data->boot_irq_seen)", source)


if __name__ == "__main__":
    unittest.main()
