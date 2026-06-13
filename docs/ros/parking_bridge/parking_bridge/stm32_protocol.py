"""Protocol-shape inspection helpers for receive-only STM32 serial data."""

from __future__ import annotations

import collections
import hashlib
import math
from typing import Any


ASCII_WHITESPACE = {9, 10, 13}


def is_printable_ascii(byte: int) -> bool:
    return byte in ASCII_WHITESPACE or 32 <= byte <= 126


def ascii_preview(data: bytes, limit: int = 256) -> str:
    chars = []
    for byte in data[:limit]:
        if is_printable_ascii(byte):
            chars.append(chr(byte))
        else:
            chars.append(".")
    return "".join(chars).replace("\r", " ").replace("\n", " ")


def _entropy_bits_per_byte(counts: collections.Counter[int], total: int) -> float:
    if total <= 0:
        return 0.0
    entropy = 0.0
    for count in counts.values():
        probability = count / total
        entropy -= probability * math.log2(probability)
    return entropy


def _line_stats(data: bytes) -> dict[str, Any]:
    newline_count = data.count(10)
    cr_count = data.count(13)
    if not data or (newline_count == 0 and cr_count == 0):
        return {
            "newline_count": newline_count,
            "carriage_return_count": cr_count,
            "line_count": 0,
            "max_line_length": 0,
            "sample_lines": [],
        }
    normalized = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    lines = [
        line.decode("ascii", errors="replace")
        for line in normalized.split(b"\n")
        if line
    ]
    return {
        "newline_count": newline_count,
        "carriage_return_count": cr_count,
        "line_count": len(lines),
        "max_line_length": max((len(line) for line in lines), default=0),
        "sample_lines": lines[:6],
    }


def _classify(total: int, printable_ratio: float, nul_ratio: float, unique: int, entropy: float, lines: int) -> str:
    if total == 0:
        return "empty"
    if printable_ratio >= 0.9 and lines >= 1:
        return "ascii_line_protocol"
    if printable_ratio >= 0.85:
        return "mostly_ascii"
    if unique <= 4:
        return "low_entropy_repeating"
    if nul_ratio >= 0.05:
        return "binary_with_nul"
    if printable_ratio < 0.35 and entropy >= 3.0:
        return "binary_protocol_or_misdecoded"
    if entropy < 2.0:
        return "low_entropy_binary_or_status_pattern"
    return "mixed_binary_ascii"


def _protocol_family(classification: str) -> str:
    if classification in {"ascii_line_protocol", "mostly_ascii"}:
        return "ascii"
    if classification == "empty":
        return "empty"
    if classification == "mixed_binary_ascii":
        return "mixed"
    return "binary"


def analyze_bytes(data: bytes, sample_limit: int = 4096) -> dict[str, Any]:
    sample = data[:sample_limit]
    total = len(data)
    sample_total = len(sample)
    printable = sum(1 for byte in sample if is_printable_ascii(byte))
    nul = sample.count(0)
    counts = collections.Counter(sample)
    unique = len(counts)
    entropy = _entropy_bits_per_byte(counts, sample_total)
    lines = _line_stats(sample)
    printable_ratio = printable / sample_total if sample_total else 0.0
    nul_ratio = nul / sample_total if sample_total else 0.0
    classification = _classify(
        sample_total,
        printable_ratio,
        nul_ratio,
        unique,
        entropy,
        int(lines["line_count"]),
    )
    top_bytes = [
        {"byte": f"0x{byte:02x}", "count": count, "ratio": count / sample_total if sample_total else 0.0}
        for byte, count in counts.most_common(12)
    ]
    return {
        "bytes": total,
        "sample_bytes": sample_total,
        "sample_limit": sample_limit,
        "sha256": hashlib.sha256(data).hexdigest(),
        "printable_ascii_ratio": printable_ratio,
        "nul_ratio": nul_ratio,
        "unique_byte_count": unique,
        "entropy_bits_per_byte": entropy,
        "top_bytes": top_bytes,
        "ascii_preview": ascii_preview(sample),
        "line_stats": lines,
        "classification": classification,
        "protocol_family": _protocol_family(classification),
    }
