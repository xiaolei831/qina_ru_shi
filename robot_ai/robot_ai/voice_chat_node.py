"""Portable voice assistant node for ROS2.

Pipeline per turn:
    mic -> VAD -> ASR -> local tools -> LLM -> TTS -> speaker

The tools are intentionally implemented inside this package so the package can
be moved as one unit. They do not execute arbitrary shell commands.
"""
from __future__ import annotations

import collections
import datetime as _dt
import html
import json
import os
import re
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional
from urllib.parse import quote_plus, urlparse

import numpy as np
import requests
import sounddevice as sd
from bs4 import BeautifulSoup

import dashscope
from dashscope import Generation
from dashscope.audio.asr import (
    Recognition,
    RecognitionCallback,
    TranscriptionResult,
)
from dashscope.audio.qwen_omni.omni_realtime import (
    AudioFormat as OmniAudioFormat,
    MultiModality,
    OmniRealtimeCallback,
    OmniRealtimeConversation,
)
from dashscope.audio.tts_v2 import (
    AudioFormat,
    ResultCallback,
    SpeechSynthesizer,
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


SYSTEM_PROMPT = (
    "你是一个运行在香橙派上的语音助手。你可以根据本地工具结果回答时间、天气、"
    "网页资料、工作区文件和代码问题。回答要自然、简短，默认 1-2 句话；"
    "如果用户要求详细解释，再展开。不要编造工具没有提供的信息。"
    "不要写抒情、拟人、陪伴式废话。不要使用 markdown、列表、代码块或表情符号。"
)
TTS_FORMAT = AudioFormat.MP3_22050HZ_MONO_256KBPS


def _rms(block: np.ndarray) -> int:
    return int(np.sqrt(np.mean(np.square(block.astype(np.int64)))))


def _device_opens(idx: Optional[int], want_input: bool, sample_rate: int) -> bool:
    try:
        if want_input:
            with sd.InputStream(samplerate=sample_rate, blocksize=480,
                                dtype="int16", channels=1, device=idx):
                return True
        with sd.OutputStream(samplerate=22050, blocksize=512,
                             dtype="int16", channels=1, device=idx):
            return True
    except Exception:
        return False


def _pick_device(name_substr: str, want_input: bool, sample_rate: int = 16000) -> Optional[int]:
    """Find a sounddevice index, preferring requested devices that really open."""
    devices = sd.query_devices()
    candidates = []
    for i, d in enumerate(devices):
        if name_substr and name_substr in d["name"]:
            if want_input and d["max_input_channels"] > 0:
                candidates.append(i)
            if not want_input and d["max_output_channels"] > 0:
                candidates.append(i)
    for i in candidates:
        name = devices[i]["name"]
        if "hw:" not in name and name_substr in name and _device_opens(i, want_input, sample_rate):
            return i
    if candidates:
        for i in candidates:
            if _device_opens(i, want_input, sample_rate):
                return i
    for i, d in enumerate(devices):
        if "UAC" in d["name"]:
            if want_input and d["max_input_channels"] > 0:
                if _device_opens(i, want_input, sample_rate):
                    return i
            if not want_input and d["max_output_channels"] > 0:
                if _device_opens(i, want_input, sample_rate):
                    return i
    fallback_names = ("pulse", "default", "sysdefault")
    for preferred in fallback_names:
        for i, d in enumerate(devices):
            if preferred in d["name"]:
                if want_input and d["max_input_channels"] > 0 and _device_opens(i, True, sample_rate):
                    return i
                if not want_input and d["max_output_channels"] > 0 and _device_opens(i, False, sample_rate):
                    return i
    for i, d in enumerate(devices):
        if want_input and d["max_input_channels"] > 0 and _device_opens(i, True, sample_rate):
            return i
        if not want_input and d["max_output_channels"] > 0 and _device_opens(i, False, sample_rate):
            return i
    return None


def record_utterance(in_idx: int, sr: int, silence_thr: int, speech_thr: int,
                     silence_end_s: float, pre_pad_s: float,
                     min_utt_s: float, max_utt_s: float,
                     speech_start_blocks: int, min_recorded_rms: int,
                     log) -> bytes:
    """Record one utterance and return PCM S16LE mono bytes."""
    block = int(sr * 0.03)
    pre_roll = collections.deque(maxlen=int(pre_pad_s / 0.03) + 2)
    recorded = bytearray()
    max_frames = int(max_utt_s * sr / block)
    silence_blocks_to_end = max(1, int(silence_end_s / 0.03))

    with sd.InputStream(samplerate=sr, blocksize=block, dtype="int16",
                        channels=1, device=in_idx) as stream:
        state = "warmup"
        silence_count = 0
        speech_count = 0
        warmup_blocks = max(1, int(1.5 / 0.03))
        log("[听...]")
        for _ in range(max_frames * 3):
            data, _ = stream.read(block)
            rms = _rms(data)
            pcm = data.tobytes()

            if state == "warmup":
                warmup_blocks -= 1
                if warmup_blocks <= 0:
                    state = "waiting"
                continue

            if state == "waiting":
                pre_roll.append(pcm)
                if rms > speech_thr:
                    speech_count += 1
                    if speech_count >= speech_start_blocks:
                        for chunk in pre_roll:
                            recorded.extend(chunk)
                        state = "speaking"
                        silence_count = 0
                        log("(说话中)")
                else:
                    speech_count = 0
            elif state == "speaking":
                recorded.extend(pcm)
                if rms < silence_thr:
                    silence_count += 1
                    if silence_count >= silence_blocks_to_end:
                        break
                else:
                    silence_count = 0

            if len(recorded) >= max_utt_s * sr * 2:
                break

    if len(recorded) < int(min_utt_s * sr * 2):
        return b""
    if _rms(np.frombuffer(recorded, dtype=np.int16)) < min_recorded_rms:
        log("(能量过低，忽略)")
        return b""
    return bytes(recorded)


class _AsrCb(RecognitionCallback):
    def __init__(self):
        self.text = ""
        self.done = False
        self.err = None

    def on_open(self): pass
    def on_close(self): self.done = True
    def on_event(self, result): pass
    def on_error(self, msg): self.err = msg

    def on_change(self, tr: TranscriptionResult):
        s = tr.get_sentence()
        if isinstance(s, dict):
            self.text = s.get("text", "")
        elif s:
            self.text = str(s)

    def on_complete(self): self.done = True


def asr(pcm: bytes, model: str, sr: int, log) -> str:
    cb = _AsrCb()
    rec = Recognition(model=model, format="pcm", sample_rate=sr, callback=cb)
    rec.start()
    try:
        chunk = 3200
        for i in range(0, len(pcm), chunk):
            rec.send_audio_frame(pcm[i:i + chunk])
            time.sleep(0.02)
    finally:
        rec.stop()
    for _ in range(50):
        if cb.done or cb.err:
            break
        time.sleep(0.1)
    if cb.err:
        log(f"[ASR err] {cb.err}")
    return cb.text.strip()


class _OmniAsrCb(OmniRealtimeCallback):
    def __init__(self):
        self.done = threading.Event()
        self.text_parts = []
        self.transcript_parts = []
        self.err = None

    def on_open(self): pass

    def on_close(self, close_status_code, close_msg):
        self.done.set()

    def on_event(self, message):
        etype = message.get("type", "")
        if etype == "error":
            self.err = message.get("error") or message
            self.done.set()
            return
        if etype in (
            "conversation.item.input_audio_transcription.completed",
            "input_audio_buffer.transcription.completed",
        ):
            text = message.get("transcript") or message.get("text")
            if text:
                self.transcript_parts.append(str(text))
        elif etype in ("response.text.delta", "response.output_text.delta"):
            delta = message.get("delta")
            if delta:
                self.text_parts.append(str(delta))
        elif etype in ("response.done", "session.finished"):
            self.done.set()


def omni_asr(pcm: bytes, model: str, voice: str,
             sample_rate: int, max_tokens: int, log) -> str:
    """Transcribe one utterance with Qwen Omni realtime."""
    import base64

    cb = _OmniAsrCb()
    conv = OmniRealtimeConversation(model=model, callback=cb, api_key=dashscope.api_key)
    conv.connect()
    try:
        conv.update_session(
            output_modalities=[MultiModality.TEXT],
            voice=voice,
            input_audio_format=OmniAudioFormat.PCM_16000HZ_MONO_16BIT,
            enable_input_audio_transcription=True,
            enable_turn_detection=False,
            max_response_output_tokens=max_tokens,
        )
        chunk_bytes = max(3200, sample_rate // 10 * 2)
        for i in range(0, len(pcm), chunk_bytes):
            chunk = pcm[i:i + chunk_bytes]
            conv.append_audio(base64.b64encode(chunk).decode("ascii"))
            time.sleep(0.01)
        conv.commit()
        conv.create_response(
            instructions="请只转写用户语音，不要回答问题。",
            output_modalities=[MultiModality.TEXT],
        )
        if not cb.done.wait(45):
            log("[Omni ASR err] timeout")
            conv.cancel_response()
        if cb.err:
            log(f"[Omni ASR err] {cb.err}")
    finally:
        try:
            conv.close()
        except Exception:
            pass
    transcript = "".join(cb.transcript_parts).strip()
    fallback = "".join(cb.text_parts).strip()
    return transcript or fallback


def _http_get(url: str, *, params=None, timeout: float = 8.0) -> requests.Response:
    headers = {
        "User-Agent": (
            "Mozilla/5.0 (X11; Linux aarch64) AppleWebKit/537.36 "
            "KHTML, like Gecko Chrome/120 Safari/537.36"
        )
    }
    r = requests.get(url, params=params, timeout=timeout, headers=headers)
    r.raise_for_status()
    return r


def _compact_text(text: str, limit: int = 1800) -> str:
    text = html.unescape(text)
    text = re.sub(r"\s+", " ", text).strip()
    if len(text) > limit:
        return text[:limit] + "..."
    return text


def _is_noise_transcript(text: str) -> bool:
    cleaned = re.sub(r"[\s，。！？,.!?（）()]+", "", text)
    if not cleaned:
        return True
    return cleaned in {
        "嗯", "哦", "啊", "呃", "呐", "额", "唔",
        "嗯嗯", "哦哦", "无语音内容", "没有语音内容",
    }


def _safe_root(path: str) -> Path:
    return Path(os.path.expanduser(path)).resolve()


def _safe_join(root: Path, raw_path: str) -> Optional[Path]:
    raw_path = raw_path.strip().strip("'\"“”‘’")
    if not raw_path:
        return None
    p = Path(os.path.expanduser(raw_path))
    if not p.is_absolute():
        p = root / p
    p = p.resolve()
    try:
        p.relative_to(root)
    except ValueError:
        return None
    return p


class AssistantTools:
    def __init__(self, workspace_root: str, default_location: str,
                 enable_web: bool, log):
        self.workspace_root = _safe_root(workspace_root)
        self.default_location = default_location
        self.enable_web = enable_web
        self.log = log

    def run(self, user_text: str) -> str:
        parts = []
        lower = user_text.lower()
        if self._wants_time(user_text):
            parts.append(self.time_now())
        if self._wants_weather(user_text):
            parts.append(self.weather(self._extract_location(user_text)))
        if self._wants_workspace(user_text):
            parts.append(self.workspace_answer(user_text))
        if self._wants_web(user_text, lower):
            query = self._extract_search_query(user_text)
            parts.append(self.web_search(query))
        context = "\n\n".join(p for p in parts if p)
        if context:
            self.log(f"[tools] {', '.join(self._tool_names(user_text, lower))}")
        return context

    def _tool_names(self, text: str, lower: str) -> list[str]:
        names = []
        if self._wants_time(text):
            names.append("time")
        if self._wants_weather(text):
            names.append("weather")
        if self._wants_workspace(text):
            names.append("workspace")
        if self._wants_web(text, lower):
            names.append("web")
        return names

    @staticmethod
    def _wants_time(text: str) -> bool:
        return any(k in text for k in ("几点", "时间", "现在时间", "当前时间", "今天几号", "日期"))

    @staticmethod
    def _wants_weather(text: str) -> bool:
        return any(k in text for k in ("天气", "温度", "下雨", "降雨", "空气质量"))

    @staticmethod
    def _wants_workspace(text: str) -> bool:
        return any(k in text for k in (
            "工作空间", "功能包", "代码", "源码", "文件", "目录", "工程",
            "阅读", "读取", "打开", "看一下", "看下", "看代码", "读代码"
        ))

    @staticmethod
    def _wants_web(text: str, lower: str) -> bool:
        web_words = ("上网", "联网", "搜索", "查资料", "查一下", "查一查", "最新",
                     "新闻", "资料", "百科", "google", "百度")
        if any(k in text for k in web_words) or any(k in lower for k in ("http://", "https://")):
            return not any(k in text for k in ("工作空间", "本地文件", "当前代码"))
        return False

    def time_now(self) -> str:
        now = _dt.datetime.now().astimezone()
        return (
            "[工具: 当前时间]\n"
            f"{now.strftime('%Y-%m-%d %H:%M:%S %Z')}，星期{self._weekday_cn(now.weekday())}。"
        )

    @staticmethod
    def _weekday_cn(idx: int) -> str:
        return "一二三四五六日"[idx]

    def _extract_location(self, text: str) -> str:
        m = re.search(r"([\u4e00-\u9fffA-Za-z0-9_-]{2,20})(?:的)?天气", text)
        if m and m.group(1) not in ("现在", "今天", "当前", "一下", "查询"):
            return m.group(1)
        return self.default_location

    def weather(self, location: str) -> str:
        if not self.enable_web:
            return "[工具: 天气]\n联网工具已关闭，不能查询实时天气。"
        try:
            url = f"https://wttr.in/{quote_plus(location)}"
            r = _http_get(url, params={"format": "j1", "lang": "zh-cn"}, timeout=10)
            data = r.json()
            cur = data["current_condition"][0]
            area = data.get("nearest_area", [{}])[0]
            area_name = location
            if area.get("areaName"):
                area_name = area["areaName"][0].get("value", location)
            desc = cur.get("lang_zh", cur.get("weatherDesc", [{"value": ""}]))[0].get("value", "")
            temp = cur.get("temp_C", "")
            feels = cur.get("FeelsLikeC", "")
            humidity = cur.get("humidity", "")
            wind = cur.get("windspeedKmph", "")
            return (
                "[工具: 天气]\n"
                f"地点: {area_name}\n天气: {desc}\n温度: {temp}°C，体感: {feels}°C，"
                f"湿度: {humidity}%，风速: {wind} km/h。"
            )
        except Exception as e:
            return f"[工具: 天气]\n查询 {location} 天气失败: {type(e).__name__}: {e}"

    def _extract_search_query(self, text: str) -> str:
        query = text
        for w in ("帮我", "请", "上网", "联网", "搜索", "查资料", "查一下", "查一查", "百度", "google"):
            query = query.replace(w, "")
        query = query.strip(" ，。！？?：:")
        return query or text

    def web_search(self, query: str) -> str:
        if not self.enable_web:
            return "[工具: 网页搜索]\n联网工具已关闭，不能搜索网页。"
        results = []
        errors = []
        try:
            r = _http_get("https://cn.bing.com/search", params={"q": query}, timeout=10)
            soup = BeautifulSoup(r.text, "html.parser")
            for item in soup.select("li.b_algo")[:5]:
                title_el = item.select_one("h2")
                link_el = title_el.select_one("a") if title_el else None
                snippet_el = item.select_one(".b_caption p") or item.select_one("p")
                title = _compact_text(title_el.get_text(" ", strip=True), 120) if title_el else ""
                href = link_el.get("href", "") if link_el else ""
                snippet = _compact_text(snippet_el.get_text(" ", strip=True), 260) if snippet_el else ""
                if title and href:
                    results.append((title, href, snippet))
                if len(results) >= 3:
                    break
        except Exception as e:
            errors.append(f"bing {type(e).__name__}: {e}")
        if not results:
            try:
                r = _http_get("https://www.baidu.com/s", params={"wd": query}, timeout=10)
                soup = BeautifulSoup(r.text, "html.parser")
                if "captcha" in r.url:
                    errors.append("baidu captcha")
                for item in soup.select(".result, .c-container")[:5]:
                    title_el = item.select_one("h3") or item.select_one("a")
                    if not title_el:
                        continue
                    title = _compact_text(title_el.get_text(" ", strip=True), 120)
                    href_el = title_el.find("a") if title_el.name != "a" else title_el
                    href = href_el.get("href", "") if href_el else ""
                    snippet = _compact_text(item.get_text(" ", strip=True), 260)
                    if title:
                        results.append((title, href, snippet))
                    if len(results) >= 3:
                        break
            except Exception as e:
                errors.append(f"baidu {type(e).__name__}: {e}")
        for url, params in (
            ("https://duckduckgo.com/html/", {"q": query}),
            ("https://lite.duckduckgo.com/lite/", {"q": query}),
        ):
            try:
                r = _http_get(url, params=params, timeout=10)
                soup = BeautifulSoup(r.text, "html.parser")
                links = soup.select("a.result__a") or soup.select("a[href]")
                for a in links:
                    title = _compact_text(a.get_text(" ", strip=True), 120)
                    href = a.get("href", "")
                    if not title or not href or href.startswith("#"):
                        continue
                    parent = a.find_parent("div")
                    snippet = ""
                    if parent:
                        sn = parent.select_one(".result__snippet")
                        if sn:
                            snippet = _compact_text(sn.get_text(" ", strip=True), 240)
                    host = urlparse(href).netloc
                    if "duckduckgo.com" in host and "uddg=" in href:
                        continue
                    results.append((title, href, snippet))
                    if len(results) >= 3:
                        break
                if results:
                    break
            except Exception as e:
                errors.append(f"{type(e).__name__}: {e}")
        if not results:
            try:
                r = _http_get(
                    "https://zh.wikipedia.org/w/api.php",
                    params={
                        "action": "query",
                        "list": "search",
                        "srsearch": query,
                        "format": "json",
                        "utf8": 1,
                    },
                    timeout=10,
                )
                data = r.json()
                for item in data.get("query", {}).get("search", [])[:3]:
                    title = item.get("title", "")
                    snippet = _compact_text(BeautifulSoup(item.get("snippet", ""), "html.parser").get_text(" "), 240)
                    href = "https://zh.wikipedia.org/wiki/" + quote_plus(title.replace(" ", "_"))
                    if title:
                        results.append((title, href, snippet))
            except Exception as e:
                errors.append(f"wikipedia {type(e).__name__}: {e}")
        if not results:
            try:
                r = _http_get("https://www.baidu.com/s", params={"wd": query}, timeout=10)
                soup = BeautifulSoup(r.text, "html.parser")
                for item in soup.select(".result, .c-container")[:3]:
                    title_el = item.select_one("h3") or item.select_one("a")
                    if not title_el:
                        continue
                    title = _compact_text(title_el.get_text(" ", strip=True), 120)
                    href_el = title_el.find("a") if title_el.name != "a" else title_el
                    href = href_el.get("href", "") if href_el else ""
                    snippet = _compact_text(item.get_text(" ", strip=True), 260)
                    if title:
                        results.append((title, href, snippet))
            except Exception as e:
                errors.append(f"baidu {type(e).__name__}: {e}")
        if not results:
            return "[工具: 网页搜索]\n没有拿到搜索结果。" + (" 错误: " + "; ".join(errors) if errors else "")
        lines = [f"[工具: 网页搜索]\n查询: {query}"]
        for i, (title, href, snippet) in enumerate(results, 1):
            lines.append(f"{i}. {title}\n来源: {href}\n摘要: {snippet or '无摘要'}")
        return "\n".join(lines)

    def workspace_answer(self, text: str) -> str:
        root = self.workspace_root
        if not root.exists():
            return f"[工具: 工作区]\n工作区不存在: {root}"
        if any(k in text for k in ("目录", "结构", "有哪些文件", "文件列表")):
            return self.workspace_tree()
        path = self._guess_file_path(text)
        if path:
            return self.read_file(path)
        if any(k in text for k in ("搜索", "查找", "包含")):
            query = self._extract_code_query(text)
            return self.search_files(query)
        return self.workspace_overview()

    def _guess_file_path(self, text: str) -> Optional[Path]:
        patterns = [
            r"([A-Za-z0-9_./-]+\.(?:py|yaml|yml|xml|launch|txt|md|json|sh|cpp|hpp|c|h))",
            r"[\"“']([^\"”']+)[\"”']",
        ]
        for pat in patterns:
            m = re.search(pat, text)
            if m:
                p = _safe_join(self.workspace_root, m.group(1))
                if p and p.exists():
                    return p
                found = self._find_by_name(m.group(1))
                if found:
                    return found
        return None

    def _find_by_name(self, name: str) -> Optional[Path]:
        target = Path(name).name
        for p in self._iter_files(max_files=800):
            if p.name == target:
                return p
        return None

    def _iter_files(self, max_files: int = 300):
        skip_dirs = {"build", "install", "log", "logs", ".git", "__pycache__"}
        count = 0
        for dirpath, dirnames, filenames in os.walk(self.workspace_root):
            dirnames[:] = [d for d in dirnames if d not in skip_dirs and not d.startswith(".")]
            for name in sorted(filenames):
                if name.endswith((".pyc", ".wav", ".mp3")):
                    continue
                p = Path(dirpath) / name
                count += 1
                if count > max_files:
                    return
                yield p

    def workspace_tree(self) -> str:
        lines = [f"[工具: 工作区目录]\n根目录: {self.workspace_root}"]
        for p in self._iter_files(max_files=120):
            try:
                lines.append(str(p.relative_to(self.workspace_root)))
            except ValueError:
                pass
        return "\n".join(lines)

    def workspace_overview(self) -> str:
        important = [
            "src/robot_ai/setup.py",
            "src/robot_ai/package.xml",
            "src/robot_ai/launch/voice_chat.launch.py",
            "src/robot_ai/config/params.yaml",
            "src/robot_ai/robot_ai/robot_ai_node.py",
        ]
        parts = [self.workspace_tree()]
        for rel in important:
            p = _safe_join(self.workspace_root, rel)
            if p and p.exists():
                parts.append(self.read_file(p, limit=1200))
        return "\n\n".join(parts)

    def read_file(self, path: Path, limit: int = 5000) -> str:
        try:
            path = path.resolve()
            path.relative_to(self.workspace_root)
            data = path.read_text(encoding="utf-8", errors="replace")
            rel = path.relative_to(self.workspace_root)
            return f"[工具: 读取文件]\n文件: {rel}\n内容:\n{data[:limit]}"
        except Exception as e:
            return f"[工具: 读取文件]\n读取失败: {type(e).__name__}: {e}"

    def _extract_code_query(self, text: str) -> str:
        text = re.sub(r".*(搜索|查找|包含)", "", text)
        return text.strip(" ，。！？?：:")[:80] or "voice_chat"

    def search_files(self, query: str) -> str:
        hits = []
        for p in self._iter_files(max_files=500):
            try:
                data = p.read_text(encoding="utf-8", errors="replace")
            except Exception:
                continue
            if query in data or query.lower() in data.lower():
                rel = p.relative_to(self.workspace_root)
                line_no = 1
                for line in data.splitlines():
                    if query in line or query.lower() in line.lower():
                        hits.append(f"{rel}:{line_no}: {_compact_text(line, 160)}")
                        break
                    line_no += 1
            if len(hits) >= 10:
                break
        if not hits:
            return f"[工具: 搜索文件]\n没有在工作区找到: {query}"
        return "[工具: 搜索文件]\n" + "\n".join(hits)


class MemoryStore:
    def __init__(self, path: str, log):
        self.path = Path(os.path.expanduser(path)).resolve()
        self.log = log
        self.items = self._load()

    def _load(self) -> list[dict]:
        try:
            if not self.path.exists():
                return []
            data = json.loads(self.path.read_text(encoding="utf-8"))
            return data if isinstance(data, list) else []
        except Exception as e:
            self.log(f"[memory err] load failed: {type(e).__name__}: {e}")
            return []

    def _save(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.path.write_text(
                json.dumps(self.items[-200:], ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
        except Exception as e:
            self.log(f"[memory err] save failed: {type(e).__name__}: {e}")

    def remember_from_text(self, text: str) -> Optional[str]:
        patterns = [
            r"帮我记住[，,:： ]*(.+)",
            r"你要记得[，,:： ]*(.+)",
            r"记住[，,:： ]*(.+)",
        ]
        for pat in patterns:
            m = re.search(pat, text)
            if not m:
                continue
            content = m.group(1).strip(" ，。！？?：:")
            if len(content) < 2:
                return None
            self.items.append({
                "time": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
                "content": content,
            })
            self._save()
            return content
        return None

    def context(self, limit: int = 12) -> str:
        if not self.items:
            return ""
        lines = ["[长期记忆]"]
        for item in self.items[-limit:]:
            content = item.get("content", "")
            if content:
                lines.append(f"- {content}")
        return "\n".join(lines)

    @staticmethod
    def is_memory_query(text: str) -> bool:
        return any(k in text for k in ("你记得", "还记得", "你记住", "我之前说", "我的信息"))


class StrongMemoryStore:
    def __init__(self, path: str, assistant_name: str, log):
        self.path = Path(os.path.expanduser(path)).resolve()
        self.assistant_name = assistant_name
        self.log = log
        self.data = self._load()

    def _empty(self) -> dict:
        return {
            "assistant": {"name": self.assistant_name},
            "user": {},
            "preferences": [],
            "facts": [],
            "events": [],
        }

    def _load(self) -> dict:
        try:
            if not self.path.exists():
                return self._empty()
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if isinstance(data, list):
                migrated = self._empty()
                migrated["events"] = data[-200:]
                return migrated
            if isinstance(data, dict):
                base = self._empty()
                base.update(data)
                base.setdefault("assistant", {})
                base["assistant"]["name"] = base["assistant"].get("name") or self.assistant_name
                base.setdefault("user", {})
                base.setdefault("preferences", [])
                base.setdefault("facts", [])
                base.setdefault("events", [])
                return base
        except Exception as e:
            self.log(f"[memory err] load failed: {type(e).__name__}: {e}")
        return self._empty()

    def _save(self) -> None:
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.data["preferences"] = self.data.get("preferences", [])[-100:]
            self.data["facts"] = self.data.get("facts", [])[-100:]
            self.data["events"] = self.data.get("events", [])[-200:]
            self.path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception as e:
            self.log(f"[memory err] save failed: {type(e).__name__}: {e}")

    def remember_from_text(self, text: str) -> Optional[str]:
        content = self._extract_remember_content(text)
        if not content:
            return None
        self._apply_profile(content)
        bucket = "preferences" if self._looks_like_preference(content) else "facts"
        item = {
            "time": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "content": content,
        }
        if not any(x.get("content") == content for x in self.data.get(bucket, [])):
            self.data.setdefault(bucket, []).append(item)
        self.data.setdefault("events", []).append(item)
        self._save()
        return content

    def _extract_remember_content(self, text: str) -> Optional[str]:
        for trigger in (
            "\u5e2e\u6211\u8bb0\u4f4f",
            "\u8bf7\u8bb0\u4f4f",
            "\u4f60\u8981\u8bb0\u5f97",
            "\u8bb0\u4f4f",
        ):
            if trigger in text:
                content = text.split(trigger, 1)[1].strip(" \t\r\n,，。！？?:：")
                return content if len(content) >= 2 else None
        return None

    def _apply_profile(self, content: str) -> None:
        user = self.data.setdefault("user", {})
        for pat in (
            r"(?:\u6211\u7684\u540d\u5b57\u53eb|\u6211\u53eb)([\u4e00-\u9fffA-Za-z0-9_-]{1,24})",
            r"(?:\u6211\u7684\u6635\u79f0\u53eb|\u6211\u7684\u6635\u79f0\u662f)([\u4e00-\u9fffA-Za-z0-9_-]{1,24})",
        ):
            m = re.search(pat, content)
            if m:
                user["name"] = m.group(1)
        if "\u52a9\u624b\u540d\u5b57" in content or "\u4f60\u7684\u540d\u5b57" in content:
            m = re.search(r"(?:\u53eb|\u662f)([\u4e00-\u9fffA-Za-z0-9_-]{1,24})", content)
            if m:
                self.data.setdefault("assistant", {})["name"] = m.group(1)

    @staticmethod
    def _looks_like_preference(content: str) -> bool:
        return any(k in content for k in (
            "\u559c\u6b22", "\u4e0d\u559c\u6b22", "\u504f\u597d",
            "\u4e60\u60ef", "\u4ee5\u540e", "\u56de\u7b54\u65f6"
        ))

    def context(self, limit: int = 12) -> str:
        lines = ["[\u957f\u671f\u8bb0\u5fc6]"]
        assistant = self.data.get("assistant", {})
        user = self.data.get("user", {})
        if assistant.get("name"):
            lines.append(f"\u52a9\u624b\u540d\u5b57: {assistant['name']}")
        if user.get("name"):
            lines.append(f"\u7528\u6237\u540d\u5b57: {user['name']}")
        for label, key in (("\u504f\u597d", "preferences"), ("\u4e8b\u5b9e", "facts")):
            items = self.data.get(key, [])[-limit:]
            if items:
                lines.append(label + ":")
            for item in items:
                content = item.get("content", "")
                if content:
                    lines.append(f"- {content}")
        return "\n".join(lines) if len(lines) > 1 else ""

    def identity_reply(self, text: str) -> Optional[str]:
        assistant_name = self.data.get("assistant", {}).get("name") or self.assistant_name
        user_name = self.data.get("user", {}).get("name")
        if any(k in text for k in ("\u4f60\u53eb\u4ec0\u4e48", "\u4f60\u662f\u8c01", "\u4f60\u7684\u540d\u5b57")):
            return f"\u6211\u53eb{assistant_name}\uff0c\u662f\u8fd0\u884c\u5728\u5730\u74dc\u6d3e\u4e0a\u7684AI\u52a9\u624b\u3002"
        if any(k in text for k in ("\u6211\u53eb\u4ec0\u4e48", "\u6211\u7684\u540d\u5b57", "\u4f60\u8bb0\u5f97\u6211\u53eb")):
            if user_name:
                return f"\u4f60\u53eb{user_name}\u3002"
            return "\u6211\u8fd8\u6ca1\u6709\u8bb0\u4f4f\u4f60\u7684\u540d\u5b57\u3002\u4f60\u53ef\u4ee5\u8bf4\uff1a\u8bb0\u4f4f\u6211\u7684\u540d\u5b57\u53eb\u67d0\u67d0\u3002"
        return None

    def remember_interaction(self, user_text: str, reply: str) -> None:
        if len(user_text) < 4:
            return
        self.data.setdefault("events", []).append({
            "time": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "user": user_text[:300],
            "assistant": reply[:300],
        })
        self._save()

    @staticmethod
    def is_memory_query(text: str) -> bool:
        return any(k in text for k in (
            "\u4f60\u8bb0\u5f97", "\u8fd8\u8bb0\u5f97", "\u4f60\u8bb0\u4f4f",
            "\u6211\u4e4b\u524d\u8bf4", "\u6211\u7684\u4fe1\u606f", "\u8bb0\u5fc6"
        ))


def chat(history: list, user_text: str, tool_context: str,
         model: str, max_tokens: int, log) -> str:
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]
    messages.extend(history)
    if tool_context:
        messages.append({
            "role": "system",
            "content": "以下是本轮本地工具返回的真实结果，请优先依据它回答:\n" + tool_context,
        })
    messages.append({"role": "user", "content": user_text})
    t0 = time.time()
    r = Generation.call(model=model, messages=messages,
                        max_tokens=max_tokens, result_format="message")
    dt = time.time() - t0
    if r.status_code != 200:
        log(f"[LLM err] code={r.status_code} msg={getattr(r, 'message', r)}")
        if tool_context:
            return _compact_text(tool_context, 240)
        return "抱歉，我刚才处理失败了，能再说一遍吗？"
    text = r.output.choices[0].message.content.strip()
    log(f"[LLM {dt:.2f}s] {text[:100]}")
    return text


class _TtsCb(ResultCallback):
    def __init__(self):
        self.queue = collections.deque()
        self.done = False
        self.err = None
        self._sentinel = object()
        self.lock = threading.Lock()

    def on_open(self): pass

    def on_close(self):
        with self.lock:
            self.queue.append(self._sentinel)
        self.done = True

    def on_data(self, data: bytes):
        with self.lock:
            self.queue.append(data)

    def on_event(self, result): pass

    def on_complete(self):
        with self.lock:
            self.queue.append(self._sentinel)
        self.done = True

    def on_error(self, msg):
        self.err = msg
        with self.lock:
            self.queue.append(self._sentinel)
        self.done = True

    def chunks(self):
        while True:
            with self.lock:
                item = self.queue.popleft() if self.queue else None
            if item is None:
                if self.done:
                    return
                time.sleep(0.01)
                continue
            if item is self._sentinel:
                return
            yield item


def speak(text: str, out_idx: int, model: str, voice: str,
          play_rate: int, log) -> None:
    if not text:
        return
    cb = _TtsCb()
    synth = SpeechSynthesizer(model=model, voice=voice,
                              format=TTS_FORMAT, callback=cb)
    synth.streaming_call(text)
    synth.streaming_complete()

    mp3_all = bytearray()
    for chunk in cb.chunks():
        if chunk:
            mp3_all.extend(bytes(chunk))
    if not mp3_all:
        if cb.err:
            log(f"[tts err] {cb.err}")
        return

    p = subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error",
         "-i", "pipe:0",
         "-f", "s16le", "-acodec", "pcm_s16le",
         "-ar", str(play_rate), "-ac", "1", "pipe:1"],
        input=bytes(mp3_all), capture_output=True,
    )
    pcm = p.stdout
    if not pcm:
        log(f"[tts err] ffmpeg: {p.stderr.decode('utf-8','replace')[:200]}")
        return

    arr = np.frombuffer(pcm, dtype=np.int16)
    stream = None
    try:
        stream = sd.OutputStream(samplerate=play_rate, dtype="int16",
                                 channels=1, device=out_idx, latency="high")
        stream.start()
        stream.write(arr)
    except Exception as e:
        log(f"[play err] {type(e).__name__}: {e}")
    finally:
        if stream:
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass


class VoiceChatNode(Node):
    def __init__(self):
        super().__init__('robot_ai_node')

        self._declare_params()
        self.load_params()
        dashscope.api_key = self.api_key

        self.pub_asr = self.create_publisher(String, '/voice/asr_text', 10)
        self.pub_reply = self.create_publisher(String, '/voice/llm_reply', 10)
        self.srv_say = self.create_service(SetBool, '/voice/say', self._handle_say)

        self.tools = AssistantTools(
            self.workspace_root, self.default_location,
            self.enable_web_tools, self._log,
        )
        self.memory = StrongMemoryStore(self.memory_file, self.assistant_name, self._log)
        self.in_idx = _pick_device(self.input_substr, want_input=True, sample_rate=self.sample_rate)
        self.out_idx = _pick_device(self.output_substr, want_input=False, sample_rate=self.tts_play_rate)
        self.get_logger().info(
            f"input idx={self.in_idx} output idx={self.out_idx} "
            f"(device match={self.input_substr})"
        )
        self.get_logger().info(
            f"ASR={self.asr_backend}/{self.asr_model} LLM={self.llm_model} "
            f"TTS={self.tts_model}/{self.tts_voice}"
        )
        self.get_logger().info(
            f"tools web={self.enable_web_tools} workspace={self.workspace_root} "
            f"location={self.default_location}"
        )
        self.get_logger().info(f"memory file={self.memory_file}")
        self.get_logger().info(
            f"VAD: speech>{self.speech_thr} silence<{self.silence_thr} "
            f"end_after={self.silence_end_s}s"
        )
        self.get_logger().info("Ready. Speak now. Ctrl+C to quit.")

    def _declare_params(self):
        self.declare_parameter('input_device', 'UAC')
        self.declare_parameter('output_device', 'UAC')
        self.declare_parameter('dashscope_api_key', '')
        self.declare_parameter('asr_backend', 'omni')
        self.declare_parameter('asr_model', 'paraformer-realtime-v2')
        self.declare_parameter('omni_asr_model', 'qwen3-omni-flash-realtime')
        self.declare_parameter('omni_voice', 'Cherry')
        self.declare_parameter('llm_model', 'qwen3-235b-a22b-instruct-2507')
        self.declare_parameter('tts_model', 'cosyvoice-v1')
        self.declare_parameter('tts_voice', 'longxiaochun')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('tts_play_rate', 22050)
        self.declare_parameter('silence_threshold', 260)
        self.declare_parameter('speech_threshold', 520)
        self.declare_parameter('silence_end_s', 0.8)
        self.declare_parameter('pre_speech_pad_s', 0.3)
        self.declare_parameter('min_utterance_s', 0.4)
        self.declare_parameter('max_utterance_s', 12.0)
        self.declare_parameter('max_tokens', 512)
        self.declare_parameter('max_history_turns', 6)
        self.declare_parameter('enable_web_tools', True)
        self.declare_parameter('default_location', 'Shanghai')
        self.declare_parameter('workspace_root', '')
        self.declare_parameter('memory_file', '')
        self.declare_parameter('assistant_name', '小青')
        self.declare_parameter('speech_start_blocks', 4)
        self.declare_parameter('min_recorded_rms', 260)

    def load_params(self):
        g = lambda n: self.get_parameter(n).value
        self.input_substr = g('input_device')
        self.output_substr = g('output_device')
        self.api_key = g('dashscope_api_key') or os.environ.get('DASHSCOPE_API_KEY', '')
        self.asr_backend = g('asr_backend')
        self.asr_model = g('asr_model')
        self.omni_asr_model = g('omni_asr_model')
        self.omni_voice = g('omni_voice')
        self.llm_model = g('llm_model')
        self.tts_model = g('tts_model')
        self.tts_voice = g('tts_voice')
        self.sample_rate = int(g('sample_rate'))
        self.tts_play_rate = int(g('tts_play_rate'))
        self.silence_thr = int(g('silence_threshold'))
        self.speech_thr = int(g('speech_threshold'))
        self.silence_end_s = float(g('silence_end_s'))
        self.pre_pad_s = float(g('pre_speech_pad_s'))
        self.min_utt_s = float(g('min_utterance_s'))
        self.max_utt_s = float(g('max_utterance_s'))
        self.max_tokens = int(g('max_tokens'))
        self.max_history_turns = int(g('max_history_turns'))
        self.enable_web_tools = bool(g('enable_web_tools'))
        self.default_location = g('default_location')
        workspace_param = g('workspace_root') or os.getcwd()
        self.workspace_root = _safe_root(str(workspace_param))
        memory_param = g('memory_file')
        self.memory_file = str(_safe_root(str(memory_param))) if memory_param else str(
            self.workspace_root / 'data' / 'memory.json'
        )
        self.assistant_name = g('assistant_name')
        self.speech_start_blocks = int(g('speech_start_blocks'))
        self.min_recorded_rms = int(g('min_recorded_rms'))

    def _handle_say(self, req, resp):
        resp.success = True
        resp.message = 'ok'
        return resp

    def _log(self, msg):
        self.get_logger().info(msg)

    def run(self):
        history: list = []
        while rclpy.ok():
            pcm = record_utterance(
                self.in_idx, self.sample_rate,
                self.silence_thr, self.speech_thr,
                self.silence_end_s, self.pre_pad_s,
                self.min_utt_s, self.max_utt_s,
                self.speech_start_blocks, self.min_recorded_rms,
                self._log,
            )
            if not pcm:
                self._log("(太短，忽略)")
                continue
            t0 = time.time()
            if self.asr_backend == "omni":
                text = omni_asr(
                    pcm, self.omni_asr_model, self.omni_voice, self.sample_rate,
                    self.max_tokens, self._log,
                )
            else:
                text = asr(pcm, self.asr_model, self.sample_rate, self._log)
            if not text:
                self._log("(未识别)")
                continue
            if _is_noise_transcript(text):
                self._log(f"(无效短语，忽略) {text}")
                continue
            self._log(f"[ASR {time.time()-t0:.2f}s] {text}")
            self.pub_asr.publish(String(data=text))

            remembered = self.memory.remember_from_text(text)
            identity_reply = self.memory.identity_reply(text)
            tool_context = self.tools.run(text)
            memory_context = self.memory.context()
            if remembered:
                memory_context = (memory_context + "\n" if memory_context else "") + (
                    f"[本轮新记忆]\n{remembered}"
                )
            if StrongMemoryStore.is_memory_query(text) and not memory_context:
                memory_context = "[长期记忆]\n目前还没有保存任何长期记忆。"
            if memory_context:
                tool_context = (tool_context + "\n\n" if tool_context else "") + memory_context
            if identity_reply:
                reply = identity_reply
            else:
                reply = chat(history, text, tool_context,
                             self.llm_model, self.max_tokens, self._log)
            self.pub_reply.publish(String(data=reply))

            speak(reply, self.out_idx, self.tts_model, self.tts_voice,
                  self.tts_play_rate, self._log)

            history.append({"role": "user", "content": text})
            history.append({"role": "assistant", "content": reply})
            self.memory.remember_interaction(text, reply)
            while len(history) > self.max_history_turns * 2:
                history.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceChatNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
