#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 BLE UART（Nordic UART Service）链路，供 encoder_monitor 使用。

依赖: pip install bleak
设备名默认: OEZ-RPM
"""

from __future__ import annotations

import asyncio
import queue
import threading
from typing import Callable

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC → ESP
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP → PC
DEFAULT_NAME_PREFIX = "OEZ-RPM"


def bleak_available() -> bool:
    try:
        import bleak  # noqa: F401

        return True
    except ImportError:
        return False


async def scan_oez_devices(timeout_s: float = 5.0) -> list[tuple[str, str]]:
    from bleak import BleakScanner

    found: dict[str, str] = {}
    devices = await BleakScanner.discover(timeout=timeout_s)
    for d in devices:
        name = d.name or ""
        if name.startswith(DEFAULT_NAME_PREFIX) or "OEZ" in name.upper():
            found[d.address] = name or d.address
    return sorted([(a, n) for a, n in found.items()], key=lambda x: x[1])


def scan_oez_devices_sync(timeout_s: float = 5.0) -> list[tuple[str, str]]:
    return asyncio.run(scan_oez_devices(timeout_s))


def _find_char(services, uuid: str):
    u = uuid.lower()
    for svc in services:
        for ch in svc.characteristics:
            if str(ch.uuid).lower() == u:
                return ch
    return None


class BleLink(threading.Thread):
    """与 SerialLink 同接口。"""

    def __init__(
        self,
        address: str,
        out_q: queue.Queue,
        stop_evt: threading.Event,
        *,
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

    def send(self, cmd: str) -> None:
        line = cmd.strip()
        if not line:
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
        finally:
            self._opened = False

    async def _async_main(self) -> None:
        from bleak import BleakClient

        self._status(f"BLE 连接 {self.address} …")
        # Windows 会缓存旧 GATT；刷固件后必须禁用缓存，否则看不到 NUS
        async with BleakClient(
            self.address,
            timeout=20.0,
            winrt={"use_cached_services": False},
        ) as client:
            if not client.is_connected:
                raise RuntimeError("BLE 未连接")

            tx = _find_char(client.services, NUS_TX)
            rx = _find_char(client.services, NUS_RX)
            if tx is None or rx is None:
                svcs = [str(s.uuid) for s in client.services]
                raise RuntimeError(
                    "未找到 Nordic UART 服务（NUS）。"
                    f"当前 GATT={svcs}。"
                    "请确认已烧录 BLE 固件；Windows 可在「设置→蓝牙」中忽略该设备后重连。"
                )

            self._opened = True
            self._status("BLE 已连接（NUS OK）")
            buf = ""

            def _on_notify(_char, data: bytearray) -> None:
                nonlocal buf
                try:
                    buf += bytes(data).decode("utf-8", errors="ignore")
                except Exception:  # noqa: BLE001
                    return
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r")
                    if not line:
                        continue
                    self._rx_count += 1
                    self.out_q.put(("__raw__", line))

            try:
                await client.start_notify(tx, _on_notify)
            except Exception as exc:  # noqa: BLE001
                self._status(f"BLE notify 订阅失败，改用读轮询: {exc}")

            await asyncio.sleep(0.3)
            await client.write_gatt_char(rx, b"BLE?\n", response=False)

            last_status = 0.0
            last_poll = 0.0
            last_val = b""
            while not self.stop_evt.is_set():
                try:
                    while True:
                        tx_line = self._tx_q.get_nowait()
                        data = tx_line.encode("utf-8")
                        for i in range(0, len(data), 160):
                            await client.write_gatt_char(
                                rx, data[i : i + 160], response=False
                            )
                            await asyncio.sleep(0.005)
                except queue.Empty:
                    pass
                if not client.is_connected:
                    raise RuntimeError("BLE 断开")

                now = asyncio.get_event_loop().time()
                # Notify 若无数据：50ms 读 TX 特征值（固件 setValue 后可读）
                if now - last_poll >= 0.05:
                    last_poll = now
                    try:
                        val = bytes(await client.read_gatt_char(tx))
                        if val and val != last_val:
                            last_val = val
                            chunk = val.decode("utf-8", errors="ignore")
                            for line in chunk.replace("\r", "").split("\n"):
                                line = line.strip()
                                if line:
                                    self._rx_count += 1
                                    self.out_q.put(("__raw__", line))
                    except Exception:  # noqa: BLE001
                        pass

                if now - last_status > 2.0:
                    last_status = now
                    self._status(f"BLE 已连接 | 收包 {self._rx_count}")
                await asyncio.sleep(0.01)

            try:
                await client.stop_notify(tx)
            except Exception:  # noqa: BLE001
                pass
