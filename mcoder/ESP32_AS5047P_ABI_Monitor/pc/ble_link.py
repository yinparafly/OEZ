# -*- coding: utf-8 -*-
"""BLE link for OEZ-ABI (Nordic UART) - Windows-friendly poll + dump ACK."""

from __future__ import annotations

import asyncio
import queue
import sys
import threading
import time
from typing import Callable

NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"


def bleak_available() -> bool:
    try:
        import bleak  # noqa: F401

        return True
    except ImportError:
        return False


def _find_char(services, uuid: str):
    u = uuid.lower()
    for svc in services:
        for ch in svc.characteristics:
            if str(ch.uuid).lower() == u:
                return ch
    return None


async def scan_oez_devices(timeout_s: float = 5.0) -> list[tuple[str, str]]:
    from bleak import BleakScanner

    found: dict[str, str] = {}
    devices = await BleakScanner.discover(
        timeout=max(2.0, timeout_s * 0.6),
    )
    for d in devices:
        name = d.name or ""
        if "OEZ-ABI" in name or "OEZ" in name.upper():
            found[d.address] = name or "OEZ-ABI"
    if not found:
        devices2 = await BleakScanner.discover(timeout=max(2.0, timeout_s * 0.6))
        for d in devices2:
            name = d.name or ""
            if "OEZ" in name.upper() or "ABI" in name.upper():
                found[d.address] = name or d.address
    return [(a, n) for a, n in found.items()]


def scan_oez_devices_sync(timeout_s: float = 5.0) -> list[tuple[str, str]]:
    return asyncio.run(scan_oez_devices(timeout_s))


class BleLink(threading.Thread):
    def __init__(
        self,
        address: str,
        out_q: queue.Queue,
        stop_evt: threading.Event,
        on_status: Callable[[str], None] | None = None,
    ):
        super().__init__(daemon=True)
        self.address = address
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.on_status = on_status
        self.error: str | None = None
        self._tx_q: queue.Queue[str] = queue.Queue()
        self._opened = False
        self._rx_count = 0
        self._notify_count = 0
        self._notify_ok = False
        self._dumping = False
        self._ack_needed = False
        self._dump_lines = 0
        self._last_telem_mono = 0.0
        self._last_l_emit = 0.0  # UI ?? ?10Hz
        self._pending_l: str | None = None
        self._dump_batch: list[str] = []

    def send(self, cmd: str) -> None:
        line = cmd.strip()
        if not line:
            return
        up = line.upper()
        if self._dumping:
            allow = (
                up.startswith("DUMP ACK")
                or up == "ACK"
                or up.startswith("DUMP NEXT")
                or up.startswith("DUMP ABORT")
                or up.startswith("MONITOR")
                or up.startswith("TIME")
                or up.startswith("ABI")
                or up.startswith("LOG DUMP")
                or up.startswith("LOG CLEAR")
                or up == "READ"
                or up.startswith("LOG READ")
            )
            if not allow:
                return
        self._tx_q.put(line + "\n")

    @property
    def is_open(self) -> bool:
        return self.is_alive() and self._opened

    def _status(self, msg: str) -> None:
        if self.on_status:
            try:
                self.on_status(msg)
            except Exception:  # noqa: BLE001
                pass

    def run(self) -> None:
        try:
            asyncio.run(self._async_main())
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.out_q.put(("__error__", str(exc)))
            self._status(f"BLE error: {exc}")
        finally:
            self._opened = False

    def _ingest_text(self, text: str, *, via: str) -> None:
        if not text:
            return
        for line in text.replace("\r", "").split("\n"):
            line = line.strip()
            if line:
                self._on_rx_line(line, via=via)

    def _flush_dump_batch(self) -> None:
        if self._dump_batch:
            self.out_q.put(("__dump_batch__", list(self._dump_batch)))
            self._dump_batch.clear()

    def _on_rx_line(self, line: str, *, via: str = "n") -> None:
        self._rx_count += 1

        if line.startswith("L,"):
            self._last_telem_mono = time.monotonic()
            if self._dumping:
                self._flush_dump_batch()
                self._dumping = False
                self._ack_needed = False
                self._status("BLE live telem OK")
            # ?? 10Hz??????????????? UI ??
            now = time.monotonic()
            if now - self._last_l_emit < 0.1:
                self._pending_l = line
                return
            self._last_l_emit = now
            self._pending_l = None
            self.out_q.put(("__raw__", line))
            return

        if line.startswith("# DUMP BUSY"):
            self._dumping = True
            self._dump_lines = 0
            self._dump_batch.clear()
            self._status("BLE dump preparing...")
        elif line.startswith("# AUTO DUMP READY"):
            self._dumping = False
            self._ack_needed = False
            self._status("BLE dump ready - host will pull")
        elif line.startswith("# AUTO DUMP BEGIN") or line.startswith("# LOG DUMP"):
            self._dumping = True
            self._dump_lines = 0
            self._dump_batch.clear()
            self._status("BLE dump receiving...")
        elif line.startswith("# BLE dump mode"):
            self._dumping = True
            self._dump_lines = 0
            self._status("BLE dump receiving...")
        elif (
            line.startswith("# AUTO DUMP END")
            or line.startswith("I END")
            or line.startswith("D END")
        ):
            self._flush_dump_batch()
            if line.startswith("# AUTO DUMP END") or line.startswith("I END"):
                self._dumping = False
                self._ack_needed = False
                self._status(f"BLE dump done, lines {self._dump_lines}")

        if self._dumping and line.startswith("D,"):
            self._dump_lines += 1
            self._dump_batch.append(line)
            self._ack_needed = True
            if len(self._dump_batch) >= 16:
                self._flush_dump_batch()
            return
        if self._dumping and line.startswith("I,"):
            self._ack_needed = True
        elif self._dumping and line.startswith("# DUMP PROG"):
            self._ack_needed = True
            self.out_q.put(("__raw__", line))
            return
        elif line.startswith("# KEEPALIVE"):
            return

        self.out_q.put(("__raw__", line))

    async def _async_main(self) -> None:
        from bleak import BleakClient

        self._status(f"BLE connecting {self.address} ...")
        kwargs: dict = {"timeout": 25.0}
        if sys.platform == "win32":
            kwargs["winrt"] = {"use_cached_services": False}

        async with BleakClient(self.address, **kwargs) as client:
            if not client.is_connected:
                raise RuntimeError("BLE not connected")

            try:
                if hasattr(client, "exchange_mtu"):
                    await client.exchange_mtu(247)
            except Exception:  # noqa: BLE001
                pass

            tx = _find_char(client.services, NUS_TX)
            rx = _find_char(client.services, NUS_RX)
            if tx is None or rx is None:
                svcs = [str(s.uuid) for s in client.services]
                raise RuntimeError(
                    "NUS not found. "
                    f"GATT={svcs}. "
                    "Flash ABI monitor firmware; on Windows ignore device in Bluetooth settings then retry."
                )

            self._opened = True
            self._dumping = False
            self._status("BLE connected (NUS OK)")
            buf = ""

            def _on_notify(_char, data: bytearray) -> None:
                nonlocal buf
                self._notify_count += 1
                try:
                    buf += bytes(data).decode("utf-8", errors="ignore")
                except Exception:  # noqa: BLE001
                    return
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r")
                    if line:
                        self._on_rx_line(line, via="notify")
                if len(buf) > 8192:
                    buf = ""

            try:
                await client.start_notify(tx, _on_notify)
                self._notify_ok = True
            except Exception as exc:  # noqa: BLE001
                self._notify_ok = False
                self._status(f"Notify unavailable, poll mode: {exc}")

            await asyncio.sleep(0.25)
            unix_ms = int(time.time() * 1000)
            for cmd in (f"TIME {unix_ms}\n", "BLE RATE 10\n", "ABI?\n"):
                try:
                    await client.write_gatt_char(rx, cmd.encode("utf-8"), response=False)
                    await asyncio.sleep(0.08)
                except Exception:  # noqa: BLE001
                    pass

            last_status = 0.0
            last_poll = 0.0
            last_val = b""

            while not self.stop_evt.is_set():
                if self._ack_needed:
                    self._ack_needed = False
                    try:
                        await client.write_gatt_char(rx, b"DUMP ACK\n", response=False)
                    except Exception:  # noqa: BLE001
                        pass

                try:
                    while True:
                        if self._ack_needed:
                            break
                        tx_line = self._tx_q.get_nowait()
                        data = tx_line.encode("utf-8")
                        for i in range(0, len(data), 160):
                            await client.write_gatt_char(
                                rx, data[i : i + 160], response=False
                            )
                            await asyncio.sleep(0.015)
                except queue.Empty:
                    pass

                if not client.is_connected:
                    raise RuntimeError("BLE disconnected")

                now = time.monotonic()
                # ?????????? L
                if self._pending_l is not None and (now - self._last_l_emit) >= 0.1:
                    self.out_q.put(("__raw__", self._pending_l))
                    self._pending_l = None
                    self._last_l_emit = now
                # Dump: always poll fast (Windows notify often drops D lines)
                poll_period = 0.045 if self._dumping else (0.12 if sys.platform == "win32" else 0.25)
                if now - last_poll >= poll_period:
                    last_poll = now
                    try:
                        val = bytes(await client.read_gatt_char(tx))
                        if val and val != last_val:
                            last_val = val
                            self._ingest_text(
                                val.decode("utf-8", errors="ignore"), via="read"
                            )
                            if self._ack_needed:
                                self._ack_needed = False
                                try:
                                    await client.write_gatt_char(
                                        rx, b"DUMP ACK\n", response=False
                                    )
                                except Exception:  # noqa: BLE001
                                    pass
                    except Exception:  # noqa: BLE001
                        pass

                if now - last_status > 2.0:
                    last_status = now
                    tnow = time.monotonic()
                    age = (
                        f"{tnow - self._last_telem_mono:.1f}s"
                        if self._last_telem_mono
                        else "none"
                    )
                    self._status(
                        f"BLE OK | notify={self._notify_count} rx={self._rx_count} "
                        f"telem_age={age}"
                        + (f" | dump {self._dump_lines}" if self._dumping else "")
                    )
                await asyncio.sleep(0.005 if self._dumping else 0.02)

            try:
                await client.stop_notify(tx)
            except Exception:  # noqa: BLE001
                pass
