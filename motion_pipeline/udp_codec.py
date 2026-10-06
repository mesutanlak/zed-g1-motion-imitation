"""Small, backwards-compatible JSON datagrams for the dedicated ZED link."""

from __future__ import annotations

import json
from typing import Any
import zlib


COMPRESSED_JSON_MAGIC = b"G1Z1"
MAXIMUM_UNCOMPRESSED_BYTES = 2_000_000


def encode_json_datagram(value: Any, *, compress: bool = False) -> bytes:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8")
    if not compress:
        return payload
    return COMPRESSED_JSON_MAGIC + zlib.compress(payload, level=1)


def decode_json_datagram(
    payload: bytes,
    *,
    maximum_uncompressed_bytes: int = MAXIMUM_UNCOMPRESSED_BYTES,
) -> Any:
    """Decode plain legacy JSON or a bounded G1Z1 zlib JSON datagram."""
    raw = payload
    if payload.startswith(COMPRESSED_JSON_MAGIC):
        decompressor = zlib.decompressobj()
        raw = decompressor.decompress(
            payload[len(COMPRESSED_JSON_MAGIC):],
            int(maximum_uncompressed_bytes) + 1,
        )
        if (
            len(raw) > int(maximum_uncompressed_bytes)
            or decompressor.unconsumed_tail
            or not decompressor.eof
        ):
            raise ValueError("invalid_or_oversized_compressed_json_datagram")
    if len(raw) > int(maximum_uncompressed_bytes):
        raise ValueError("oversized_json_datagram")
    return json.loads(raw.decode("utf-8"))
