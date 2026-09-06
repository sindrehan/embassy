#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5,<4"]
# ///
"""Run FRDM-KL82Z hardware tests with explicit probes and GDB or UART verdicts."""

import argparse
import binascii
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CHIP = "MKL82Z128VLK7"


@dataclass(frozen=True)
class Test:
    group: str
    timeout: int = 10
    drivers: tuple[str, ...] = ("tpm", "lptmr")
    serial: bool = False


TESTS = {
    "adc": Test("onboard"),
    "adc_lifecycle": Test("onboard"),
    "adc_dedicated": Test("onboard"),
    "i2c": Test("onboard"),
    "intmux": Test("onboard"),
    "lpuart": Test("onboard"),
    "lpuart_dma": Test("onboard"),
    "lpuart_lifecycle": Test("onboard"),
    "lpuart_traits": Test("onboard"),
    "lpuart_suspend": Test("onboard"),
    "lpuart_buffered": Test("onboard"),
    "dma_lifecycle": Test("onboard"),
    "adc_low_power": Test("low-power"),
    "watchdog": Test("serial", serial=True),
    "watchdog_reset": Test("serial", serial=True),
    "rom_bootloader": Test("serial", serial=True),
    "vlps_pll": Test("low-power"),
    "sleep_modes": Test("low-power", 25),
    "lptmr_time": Test("low-power", 90, ("lptmr",)),
    "low_leakage": Test("low-leakage", 25, ("tpm",)),
    "spis_lifecycle": Test("isolated"),
    "spi_dma_in_place": Test("isolated"),
    "spi_low_power": Test("isolated"),
    "spi_lifecycle": Test("isolated"),
    "spi_slow": Test("isolated"),
    "shared_irq": Test("isolated"),
    "gpio": Test("loopback"),
    "spi": Test("loopback"),
    "spi_link_master": Test("link", 25),
    "spi_link_dma": Test("link", 40),
}

LINK_TESTS = {"spi_link_master", "spi_link_dma"}


def probe_selector(value):
    parts = value.split(":")
    if len(parts) != 3 or not all(parts):
        raise argparse.ArgumentTypeError(
            "use a full VID:PID:SERIAL selector from probe-rs list"
        )
    try:
        vendor, product = (int(part, 16) for part in parts[:2])
    except ValueError as error:
        raise argparse.ArgumentTypeError("VID and PID must be hexadecimal") from error
    if vendor != 0x1366 or not 0 <= product <= 0xFFFF or not parts[2].isdecimal():
        raise argparse.ArgumentTypeError(
            "this runner requires a J-Link with a numeric serial number"
        )
    return value


def same_probe(first, second):
    return int(first.split(":")[2]) == int(second.split(":")[2])


def select_tests(names, group, driver):
    selected = names or [name for name, test in TESTS.items() if test.group == group]
    unknown = set(selected) - TESTS.keys()
    if unknown:
        raise ValueError(f"unknown tests: {', '.join(sorted(unknown))}")
    return [
        (name, TESTS[name])
        for name in dict.fromkeys(selected)
        if driver in TESTS[name].drivers
    ]


def stop_process(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def command(argv, log, timeout):
    """Capture output and bound both execution and cleanup of the child process group."""
    with log.open("w") as output:
        output.write(f"Command: {argv!r}\n")
        output.flush()
        process = subprocess.Popen(
            argv,
            cwd=ROOT,
            stdout=output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            code = process.wait(timeout=timeout)
        finally:
            stop_process(process)
    if code != 0:
        raise RuntimeError(f"command exited {code}; see {log}")
    return log.read_text()


def build(driver, vlpr, directory):
    features = [f"time-driver-{driver}"] + (["vlpr"] if vlpr else [])
    output = command(
        [
            "cargo",
            "build",
            "--release",
            "--bins",
            "--no-default-features",
            "--features",
            ",".join(features),
            "--message-format=json-render-diagnostics",
        ],
        directory / "build.log",
        180,
    )
    artifacts = {}
    firmware = directory / "firmware"
    firmware.mkdir()
    for line in output.splitlines():
        if not line.startswith("{"):
            continue
        item = json.loads(line)
        if item.get("reason") == "compiler-artifact" and item.get("executable"):
            name = item["target"]["name"]
            destination = firmware / f"{name}.elf"
            shutil.copy2(item["executable"], destination)
            artifacts[name] = destination
    return artifacts


def probe_args(args, probe):
    return [
        "--chip",
        CHIP,
        "--protocol",
        "swd",
        "--connect-under-reset",
        "--probe",
        probe,
        "--speed",
        str(args.speed),
    ]


def program(args, probe, binary, prefix, start=False):
    command(
        [args.probe_rs, "download", *probe_args(args, probe), str(binary)],
        prefix.with_suffix(".flash.log"),
        30,
    )
    if start:
        command(
            [args.probe_rs, "reset", *probe_args(args, probe)],
            prefix.with_suffix(".reset.log"),
            15,
        )


def gdb_commands(port, reset=True):
    return [
        "set confirm off",
        "set pagination off",
        "set remotetimeout 5",
        f"target remote 127.0.0.1:{port}",
        *(["monitor reset"] if reset else []),
        "set language c",
        "hbreak *((unsigned long)&hil_test_passed & 0xfffffffe)",
        "hbreak HardFault",
        "continue",
        "if (((unsigned long)$pc & 0xfffffffe) != ((unsigned long)&hil_test_passed & 0xfffffffe))",
        'printf "HIL FAIL: stopped before completion\\n"',
        "bt",
        "quit 1",
        "end",
        'printf "HIL PASS\\n"',
        "detach",
        "quit 0",
    ]


def gdb_server(args):
    serial = str(int(args.probe.split(":")[2]))
    argv = [
        args.jlink,
        "-device",
        "MKL82Z128xxx7",
        "-if",
        "SWD",
        "-speed",
        str(args.speed),
        "-endian",
        "little",
        "-USB",
        serial,
        "-localhostonly",
        "1",
        "-singlerun",
        "-timeout",
        "5000",
        "-nogui",
        "-port",
        str(args.port),
        "-swoport",
        str(args.port + 1),
        "-telnetport",
        str(args.port + 2),
        "-RTTTelnetPort",
        str(args.port + 3),
    ]
    ready_message = "Waiting for GDB connection"
    if args.debug_server == "probe-rs":
        argv = [
            args.probe_rs,
            "gdb",
            *probe_args(args, args.probe),
            "--reset-halt",
            "--gdb-connection-string",
            f"127.0.0.1:{args.port}",
        ]
        ready_message = "Firing up GDB stub"
    return argv, ready_message


def run_gdb(args, binary, timeout, prefix):
    argv, ready_message = gdb_server(args)
    server_log = prefix.with_suffix(".server.log")
    script = prefix.with_suffix(".gdb")
    script.write_text(
        "\n".join(gdb_commands(args.port, args.debug_server == "jlink")) + "\n"
    )
    with server_log.open("w") as output:
        server = subprocess.Popen(
            argv, stdout=output, stderr=subprocess.STDOUT, start_new_session=True
        )
        try:
            deadline = time.monotonic() + 12
            # Do not probe the TCP socket: closing that connection terminates a single-run server.
            while ready_message not in server_log.read_text():
                if server.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError(
                        f"debug server did not become ready; see {server_log}"
                    )
                time.sleep(0.05)
            result = command(
                [args.gdb, "-nx", "-q", "-batch", str(binary), "-x", str(script)],
                prefix.with_suffix(".gdb.log"),
                timeout + 5,
            )
            if "HIL PASS" not in result.splitlines():
                raise RuntimeError("GDB did not reach the completion breakpoint")
        finally:
            # Let J-Link restore the probe before killing an unresponsive server.
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop_process(server)


def valid_ping(packet):
    return (
        len(packet) == 10
        and packet[:2] == b"\x5a\xa7"
        and packet[5] == ord("P")
        and binascii.crc_hqx(packet[:8], 0) == int.from_bytes(packet[8:], "little")
    )


def run_serial(args, name, binary, timeout, prefix):
    import serial

    with serial.Serial(
        args.serial, 115200, timeout=0.1, write_timeout=1, exclusive=True
    ) as uart:
        uart.reset_input_buffer()
        program(args, args.probe, binary, prefix)
        uart.reset_input_buffer()
        command(
            [args.probe_rs, "reset", *probe_args(args, args.probe)],
            prefix.with_suffix(".reset.log"),
            15,
        )
        deadline = time.monotonic() + timeout
        received = bytearray()
        entered_rom = False
        armed_at = None
        with prefix.with_suffix(".serial.log").open("wb") as log:
            while time.monotonic() < deadline:
                data = uart.read(256)
                received.extend(data)
                del received[:-4096]
                log.write(data)
                log.flush()
                if name == "rom_bootloader":
                    entered_rom |= b"ROM entry\r\n" in received
                    if entered_rom:
                        for offset in range(len(received) - 9):
                            if valid_ping(received[offset : offset + 10]):
                                return
                        uart.write(b"\x5a\xa6")
                elif name == "watchdog_reset":
                    if armed_at is None and b"WDOG ARMED\r\n" in received:
                        armed_at = time.monotonic()
                    if b"WDOG RESET OK\r\n" in received:
                        if armed_at is None:
                            raise RuntimeError("watchdog reset arrived without arming")
                        elapsed = time.monotonic() - armed_at
                        if not 0.2 <= elapsed <= 2:
                            raise RuntimeError(
                                f"unexpected watchdog reset delay: {elapsed:.3f}s"
                            )
                        log.write(f"Reset observed after {elapsed:.3f}s\n".encode())
                        return
                elif b"WATCHDOG OK\r\n" in received:
                    return
            raise RuntimeError(
                f"UART completion timed out; see {prefix.with_suffix('.serial.log')}"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tests", nargs="*", help="test names; overrides --group")
    parser.add_argument(
        "--group",
        choices=sorted({test.group for test in TESTS.values()}),
        default="onboard",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="list all tests without building or accessing probes",
    )
    parser.add_argument(
        "--probe", type=probe_selector, help="explicit J-Link VID:PID:SERIAL selector"
    )
    parser.add_argument(
        "--peer-probe",
        type=probe_selector,
        help="second board; parked before tests, slave for --group link",
    )
    parser.add_argument(
        "--park",
        action="store_true",
        help="flash the idle fixture and release the probe",
    )
    parser.add_argument(
        "--time-driver", choices=["tpm", "lptmr", "both"], default="both"
    )
    parser.add_argument(
        "--vlpr",
        action="store_true",
        help="run sleep_modes and watchdog with the VLPR clock configuration",
    )
    parser.add_argument("--probe-rs", default="probe-rs")
    parser.add_argument(
        "--serial", help="UART device connected to PTB16/PTB17 (115200 baud)"
    )
    parser.add_argument(
        "--debug-server", choices=["jlink", "probe-rs"], default="jlink"
    )
    parser.add_argument("--jlink", default="JLinkGDBServerCLExe")
    parser.add_argument("--gdb", default="arm-none-eabi-gdb")
    parser.add_argument("--port", type=int, default=2331)
    parser.add_argument("--speed", type=int, default=1000, help="SWD speed in kHz")
    args = parser.parse_args()
    if args.list:
        for name, test in TESTS.items():
            print(
                f"{name:22} {test.group:12} {','.join(test.drivers):10} {test.timeout}s"
            )
        return 0
    if not args.probe:
        parser.error("--probe is required; no board is selected implicitly")
    if args.peer_probe and same_probe(args.probe, args.peer_probe):
        parser.error("the main and peer probes must select different boards")
    if not 1024 <= args.port <= 65532 or args.speed <= 0:
        parser.error("invalid port or SWD speed")
    drivers = ["tpm", "lptmr"] if args.time_driver == "both" else [args.time_driver]
    try:
        selections = [
            (driver, select_tests(args.tests, args.group, driver)) for driver in drivers
        ]
    except ValueError as error:
        parser.error(str(error))
    if not args.park and not any(tests for _, tests in selections):
        parser.error("no tests match this time driver")
    if (
        not args.park
        and not args.serial
        and any(test.serial for _, tests in selections for _, test in tests)
    ):
        parser.error(
            "serial tests require --serial; use the UART on the selected target"
        )
    if (
        not args.park
        and any(name in LINK_TESTS for _, tests in selections for name, _ in tests)
        and not args.peer_probe
    ):
        parser.error("the link test requires --peer-probe for the slave")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    directory = ROOT / "target" / "hil" / stamp
    directory.mkdir(parents=True)
    report = {
        "probe": args.probe,
        "peer_probe": args.peer_probe,
        "vlpr": args.vlpr,
        "results": [],
    }
    failed = False
    interrupted = False
    try:
        for driver, tests in selections:
            if not tests and not args.park:
                continue
            folder = directory / driver
            folder.mkdir()
            print(f"Building {driver} tests...", flush=True)
            artifacts = build(driver, args.vlpr, folder)
            if args.peer_probe:
                program(
                    args,
                    args.peer_probe,
                    artifacts["park"],
                    folder / "park-peer",
                    start=True,
                )
            if args.park:
                program(
                    args, args.probe, artifacts["park"], folder / "park", start=True
                )
                print(f"Parked {args.probe}", flush=True)
                break
            for name, test in tests:
                prefix = folder / name
                print(f"RUN  {driver}/{name}", flush=True)
                started = time.monotonic()
                result = {"driver": driver, "test": name, "status": "FAIL"}
                try:
                    if name in LINK_TESTS:
                        program(
                            args,
                            args.probe,
                            artifacts["park"],
                            folder / "park-master",
                            start=True,
                        )
                        program(
                            args,
                            args.peer_probe,
                            artifacts["spi_link_slave"],
                            folder / "slave",
                            start=True,
                        )
                    if test.serial:
                        run_serial(args, name, artifacts[name], test.timeout, prefix)
                    else:
                        program(args, args.probe, artifacts[name], prefix)
                        run_gdb(args, artifacts[name], test.timeout, prefix)
                    result["status"] = "PASS"
                except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                    result["error"] = str(error)
                    failed = True
                except KeyboardInterrupt:
                    result["error"] = "interrupted"
                    raise
                finally:
                    if name in LINK_TESTS:
                        for label, probe in [
                            ("master", args.probe),
                            ("peer", args.peer_probe),
                        ]:
                            try:
                                program(
                                    args,
                                    probe,
                                    artifacts["park"],
                                    folder / f"park-{label}-end",
                                    start=True,
                                )
                            except (
                                OSError,
                                RuntimeError,
                                subprocess.TimeoutExpired,
                            ) as error:
                                result["status"] = "FAIL"
                                result.setdefault("cleanup_errors", []).append(
                                    str(error)
                                )
                                failed = True
                    result["seconds"] = round(time.monotonic() - started, 3)
                    report["results"].append(result)
                    print(
                        f"{result['status']:4} {driver}/{name}"
                        + (f": {result['error']}" if "error" in result else ""),
                        flush=True,
                    )
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        report["error"] = str(error)
        failed = True
        print(f"ERROR: {error}", file=sys.stderr)
    except KeyboardInterrupt:
        report["error"] = "interrupted"
        interrupted = True
    finally:
        (directory / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        print(f"Logs: {directory}", flush=True)
    return 130 if interrupted else int(failed)


if __name__ == "__main__":
    sys.exit(main())
