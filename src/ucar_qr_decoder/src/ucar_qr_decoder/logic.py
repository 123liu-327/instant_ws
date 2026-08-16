"""Pure orchestration helpers used by the ROS QR coordinator and unit tests."""


VALID_BACKENDS = ("hybrid", "zbar_only", "opencv_only")


def rank_candidates(candidates, limit=3):
    """Return best whole frames first without modifying the input list."""
    return sorted(
        list(candidates or []),
        key=lambda candidate: float(candidate.get("score", 0.0)),
        reverse=True,
    )[:max(0, int(limit))]


class ReplaceableJobBuffer:
    """One pending slot; a newer job replaces the stale pending job."""

    def __init__(self):
        self.pending = None

    def put(self, job):
        replaced = self.pending is not None
        self.pending = job
        return replaced

    def take(self):
        job = self.pending
        self.pending = None
        return job


def legacy_url_payload(url, api_result=None, error=None):
    if api_result and api_result.get("code") == 200:
        return {"status": "success", "url": url, "json_data": api_result}
    message = error or (api_result or {}).get("error") or "invalid_response"
    return {
        "status": "error",
        "url": url,
        "error": str(message),
        "error_type": "request_error",
    }


def non_url_payload(text):
    return {
        "status": "not_url",
        "data": text,
        "message": "decoded content is not a URL",
    }


def run_decode_pipeline(candidates, backend, zbar_raw, zbar_enhanced, opencv):
    """Run the top-3/raw -> best/enhanced -> OpenCV policy.

    Callback contracts:
      zbar_raw(candidate) / zbar_enhanced(candidate)
        -> (texts, milliseconds, stage, timed_out)
      opencv(candidate) -> (texts, milliseconds, stage)
    """
    if backend not in VALID_BACKENDS:
        raise ValueError("unsupported backend: %s" % backend)
    ranked = rank_candidates(candidates, 3)
    if not ranked:
        return [], None, {
            "backend": backend,
            "hit_stage": "no_candidate",
            "zbar_ms": 0.0,
            "opencv_ms": 0.0,
            "zbar_timeouts": 0,
        }

    metadata = {
        "backend": backend,
        "hit_stage": "none",
        "zbar_ms": 0.0,
        "opencv_ms": 0.0,
        "zbar_timeouts": 0,
    }
    best = ranked[0]

    if backend != "opencv_only":
        for candidate in ranked:
            try:
                texts, elapsed, stage, timed_out = zbar_raw(candidate)
            except Exception:
                texts, elapsed, stage, timed_out = [], 0.0, "zbar_error", False
            metadata["zbar_ms"] += float(elapsed or 0.0)
            metadata["zbar_timeouts"] += int(bool(timed_out))
            if texts:
                metadata["backend"] = "zbar"
                metadata["hit_stage"] = stage or "zbar_raw"
                return list(texts), candidate, metadata

        try:
            texts, elapsed, stage, timed_out = zbar_enhanced(best)
        except Exception:
            texts, elapsed, stage, timed_out = [], 0.0, "zbar_error", False
        metadata["zbar_ms"] += float(elapsed or 0.0)
        metadata["zbar_timeouts"] += int(bool(timed_out))
        if texts:
            metadata["backend"] = "zbar"
            metadata["hit_stage"] = stage or "zbar_enhanced"
            return list(texts), best, metadata
        if backend == "zbar_only":
            metadata["backend"] = "zbar"
            return [], best, metadata

    try:
        texts, elapsed, stage = opencv(best)
    except Exception:
        texts, elapsed, stage = [], 0.0, "opencv_error"
    metadata["opencv_ms"] = float(elapsed or 0.0)
    metadata["backend"] = "opencv"
    if texts:
        metadata["hit_stage"] = stage or "opencv"
    return list(texts), best, metadata
