"""One common observation at the candidate; subsequent observations remain live."""

import hashlib
import json
from pathlib import Path

import numpy as np

from .replay_checkpoint import Decoder, Encoder
from .telemetry import write_json


def fingerprint(value):
    encoder = Encoder()
    document = encoder.encode(value)
    h = hashlib.sha256(json.dumps(document, sort_keys=True).encode())
    for key, array in sorted(encoder.arrays.items()):
        h.update(key.encode())
        h.update(array.tobytes())
    return h.hexdigest()


def save_packet(folder, tick, packet):
    folder = Path(folder)
    encoder = Encoder()
    payload = {k: v[1] for k, v in packet.items()}
    encoded = encoder.encode(payload)
    path = folder / "candidate_input.npz"
    np.savez_compressed(path, **encoder.arrays)
    write_json(
        folder / "candidate_input.json",
        {
            "tick": tick,
            "payload": encoded,
            "fingerprint": fingerprint(payload),
            "arrays_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        },
    )


def load_payload(folder, tick):
    folder = Path(folder)
    document = json.loads((folder / "candidate_input.json").read_text())
    path = folder / "candidate_input.npz"
    if (
        document["tick"] != tick
        or hashlib.sha256(path.read_bytes()).hexdigest() != document["arrays_sha256"]
    ):
        raise ValueError("Candidate packet boundary/checksum mismatch")
    with np.load(path, allow_pickle=False) as arrays:
        payload = Decoder(arrays).decode(document["payload"])
    if fingerprint(payload) != document["fingerprint"]:
        raise ValueError("Candidate packet fingerprint mismatch")
    return payload


def common_packet(reference, tick, live):
    payload = load_payload(reference, tick)
    if payload.keys() != live.keys():
        raise ValueError("Candidate sensor keys differ")
    # Preserve each current sensor frame ID; replay values, not stale absolute frames.
    return {k: (live[k][0], payload[k]) for k in live}


def fixed_backend():
    import torch

    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    torch.use_deterministic_algorithms(True)
    return {
        "cudnn_benchmark": torch.backends.cudnn.benchmark,
        "cudnn_deterministic": torch.backends.cudnn.deterministic,
        "deterministic_algorithms": torch.are_deterministic_algorithms_enabled(),
        "cudnn_allow_tf32": torch.backends.cudnn.allow_tf32,
        "matmul_allow_tf32": torch.backends.cuda.matmul.allow_tf32,
    }


def call_candidate(call, callback, agent, packet, timestamp, folder, tick, backend):
    """Hash actual forward arguments without replacing outputs or running extra inference."""
    if len(agent.nets) != 1 or getattr(agent.config, "compile", False):
        raise ValueError("Common-input audit requires one uncompiled TF++ model")
    net = agent.nets[0]
    original = net.forward
    had_local = "forward" in vars(net)
    local = vars(net).get("forward")
    hashes = []

    def forward(*args, **kwargs):
        hashes.append(fingerprint({"args": args, "kwargs": kwargs}))
        return original(*args, **kwargs)

    net.forward = forward
    try:
        result = call(callback, agent, packet, timestamp)
    finally:
        if had_local:
            net.forward = local
        else:
            del net.forward
    if len(hashes) != 1:
        raise ValueError("Candidate must perform exactly one actual model forward")
    write_json(
        Path(folder) / "candidate_application.json",
        {
            "tick": tick,
            "payload_fingerprint": fingerprint({k: v[1] for k, v in packet.items()}),
            "forward_input_fingerprint": hashes[0],
            "forward_calls": len(hashes),
            "backend": backend,
            "following_inputs": "live",
        },
    )
    return result
