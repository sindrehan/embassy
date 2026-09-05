"""Host-only checks for the hardware runner; no probes are accessed."""

import argparse
import io
import json
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest.mock import patch

import run


class SelectionTests(unittest.TestCase):
    def test_full_selector_required(self):
        for value in [
            "1366:1015",
            "1366:1015:",
            "zz:1015:123",
            "1234:1015:123",
            "1366:1015:abc",
        ]:
            with (
                self.subTest(value=value),
                self.assertRaises(argparse.ArgumentTypeError),
            ):
                run.probe_selector(value)
        self.assertEqual(run.probe_selector("1366:0101:000123"), "1366:0101:000123")

    def test_same_serial_with_different_padding(self):
        self.assertTrue(run.same_probe("1366:0101:000123", "1366:0101:123"))
        self.assertFalse(run.same_probe("1366:0101:123", "1366:0101:124"))

    def test_time_driver_filter(self):
        self.assertEqual(run.select_tests(["lptmr_time"], "onboard", "tpm"), [])
        self.assertEqual(
            run.select_tests(["adc", "adc"], "link", "tpm"), [("adc", run.TESTS["adc"])]
        )
        names = [name for name, _ in run.select_tests([], "low-power", "lptmr")]
        self.assertIn("lptmr_time", names)
        self.assertNotIn("low_leakage", names)

    def test_unknown_name_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown tests: typo"):
            run.select_tests(["typo"], "onboard", "tpm")

    def test_invalid_invocations_do_not_build(self):
        for args in [
            [],
            ["--group", "link", "--probe", "1366:0101:123"],
            ["--probe", "1366:0101:123", "--peer-probe", "1366:0101:000123"],
            ["lptmr_time", "--probe", "1366:0101:123", "--time-driver", "tpm"],
        ]:
            with (
                self.subTest(args=args),
                patch.object(sys, "argv", ["run.py", *args]),
                patch.object(run, "build") as build,
                redirect_stderr(io.StringIO()),
            ):
                with self.assertRaises(SystemExit) as error:
                    run.main()
                self.assertEqual(error.exception.code, 2)
                build.assert_not_called()


class ExecutionTests(unittest.TestCase):
    def test_command_exit_status(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "command.log"
            self.assertIn(
                "hello", run.command([sys.executable, "-c", "print('hello')"], log, 5)
            )
            with self.assertRaisesRegex(RuntimeError, "exited 7"):
                run.command([sys.executable, "-c", "raise SystemExit(7)"], log, 5)

    def test_command_timeout(self):
        with (
            tempfile.TemporaryDirectory() as folder,
            self.assertRaises(subprocess.TimeoutExpired),
        ):
            run.command(
                [sys.executable, "-c", "import time; time.sleep(30)"],
                Path(folder) / "log",
                0.1,
            )

    def test_build_uses_cargo_artifacts(self):
        artifact = {
            "reason": "compiler-artifact",
            "target": {"name": "adc"},
            "executable": "/tmp/adc",
        }
        output = (
            "Command: cargo\n"
            + json.dumps(artifact)
            + "\n"
            + json.dumps({"reason": "build-finished"})
        )
        with (
            tempfile.TemporaryDirectory() as folder,
            patch.object(run, "command", return_value=output) as command,
            patch.object(run.shutil, "copy2") as copy,
        ):
            saved = Path(folder) / "firmware" / "adc.elf"
            self.assertEqual(run.build("lptmr", True, Path(folder)), {"adc": saved})
            copy.assert_called_once_with("/tmp/adc", saved)
            argv = command.call_args.args[0]
            self.assertIn("--no-default-features", argv)
            self.assertIn("time-driver-lptmr,vlpr", argv)

    def test_verdict_breakpoint_uses_function_entry(self):
        script = run.gdb_commands(2331)
        self.assertIn("hbreak *((unsigned long)&hil_test_passed & 0xfffffffe)", script)
        self.assertTrue(
            any("$pc" in line and "&hil_test_passed" in line for line in script)
        )
        self.assertLess(script.index("quit 1"), script.index('printf "HIL PASS\\n"'))

    def check_link_cleanup(self, error, exit_code, status, name="spi_link_master"):
        with tempfile.TemporaryDirectory() as folder:
            argv = [
                "run.py",
                name,
                "--time-driver",
                "tpm",
                "--probe",
                "1366:0101:123",
                "--peer-probe",
                "1366:1015:456",
            ]
            artifacts = {
                name: Path(name)
                for name in ["park", "spi_link_master", "spi_link_dma", "spi_link_slave"]
            }
            with (
                patch.object(sys, "argv", argv),
                patch.object(run, "ROOT", Path(folder)),
                patch.object(run, "build", return_value=artifacts),
                patch.object(run, "program") as program,
            ):
                with (
                    patch.object(run, "run_gdb", side_effect=error),
                    redirect_stdout(io.StringIO()),
                ):
                    self.assertEqual(run.main(), exit_code)
                last = program.call_args_list[-2:]
                self.assertEqual(
                    [call.args[1] for call in last],
                    ["1366:0101:123", "1366:1015:456"],
                )
                self.assertTrue(all(call.args[2] == Path("park") for call in last))
            report = json.loads(next(Path(folder).rglob("summary.json")).read_text())
            self.assertEqual(report["results"][0]["status"], status)
            return report

    def test_link_failure_still_parks_both_boards_and_reports(self):
        report = self.check_link_cleanup(RuntimeError("fault"), 1, "FAIL")
        self.assertEqual(report["results"][0]["error"], "fault")

    def test_link_success_parks_both_boards(self):
        self.check_link_cleanup(None, 0, "PASS")

    def test_dma_link_success_parks_both_boards(self):
        self.check_link_cleanup(None, 0, "PASS", "spi_link_dma")

    def test_interrupted_link_parks_both_boards_and_reports(self):
        report = self.check_link_cleanup(KeyboardInterrupt(), 130, "FAIL")
        self.assertEqual(report["results"][0]["error"], "interrupted")
        self.assertEqual(report["error"], "interrupted")


if __name__ == "__main__":
    unittest.main()
