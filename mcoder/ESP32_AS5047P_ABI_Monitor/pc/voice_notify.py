# -*- coding: utf-8 -*-
"""PC 喇叭语音提醒（Windows System.Speech）。

- 测试播报：speak_test("已连接") → 「测试，测试，已连接」
- 正式提醒：speak("请加油") / speak("探测到了") → 直接说内容
"""
from __future__ import annotations

import subprocess
import threading

_lock = threading.Lock()
_last: dict[str, float] = {}


def _sapi_speak(utterance: str) -> None:
    print(f"[语音] {utterance}", flush=True)
    safe = utterance.replace("'", "''")
    ps = (
        "Add-Type -AssemblyName System.Speech; "
        "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
        "$s.Rate = 0; $s.Volume = 100; "
        f"$s.Speak('{safe}')"
    )
    try:
        subprocess.run(
            ["powershell", "-NoProfile", "-Command", ps],
            check=False,
            capture_output=True,
            timeout=60,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[语音失败] {exc}", flush=True)


def speak(text: str, *, block: bool = False, dedupe_s: float = 1.5) -> None:
    """正式提醒：直接播报 text（如「探测到了」「请加油」）。"""
    import time

    now = time.monotonic()
    with _lock:
        t0 = _last.get(text, 0.0)
        if dedupe_s > 0 and (now - t0) < dedupe_s:
            return
        _last[text] = now

    def _run() -> None:
        _sapi_speak(text)

    if block:
        _run()
    else:
        threading.Thread(target=_run, daemon=True).start()


def speak_test(text: str, *, block: bool = False) -> None:
    """测试播报：前缀「测试，测试，」+ 内容。"""
    utterance = f"测试，测试，{text}"

    def _run() -> None:
        _sapi_speak(utterance)

    if block:
        _run()
    else:
        threading.Thread(target=_run, daemon=True).start()
