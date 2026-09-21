#!/usr/bin/env python3
"""YOLO model path resolution for NIDAR perception nodes.

Replaces the old hardcoded relative path::

    ../../../../Models/best.pt

with standard ``ament_index_python`` package-share resolution plus a
cache-directory fallback that auto-downloads a lightweight default
(``yolov8n.pt``) when weights are missing instead of raising
``[Errno 2] No such file or directory``.

Search order for ``resolve_model_path('best.pt')``:
  1. Absolute path, if it exists.
  2. Relative path as given, if it exists (cwd-relative).
  3. ``<share>/<package>/models/<basename>`` for the caller's package
     and for ``person_detection``.
  4. ``~/.cache/nidar/models/<basename>``.
  5. ``/tmp/nidar_models/<basename>`` (legacy cache).
  6. Auto-download ``yolov8n.pt`` to the cache dir and return it as a
     safe fallback (with a warning) when the requested file is missing.

PEP-8 compliant.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import urllib.request
from pathlib import Path
from typing import Optional

try:
    from ament_index_python.packages import (
        PackageNotFoundError,
        get_package_share_directory,
    )
except ImportError:  # pragma: no cover - non-ROS unit tests
    PackageNotFoundError = Exception  # type: ignore[misc,assignment]

    def get_package_share_directory(_package: str) -> str:
        """Fallback that always raises when ament is unavailable."""
        raise PackageNotFoundError('ament_index_python not available')


DEFAULT_MODEL = 'yolov8n.pt'
KNOWN_LIGHTWEIGHT_MODELS = (
    'yolov8n.pt',
    'yolov8s.pt',
    'yolov5nu.pt',
    'yolov5n.pt',
)
# Ultralytics GitHub release hosting the default nano weights.
DOWNLOAD_URL_TEMPLATE = (
    'https://github.com/ultralytics/assets/releases/download/v8.3.0/{model}'
)
CACHE_DIR = Path.home() / '.cache' / 'nidar' / 'models'
LEGACY_CACHE_DIR = Path('/tmp/nidar_models')


def _log(logger, level: str, message: str) -> None:
    """Log via ROS logger or print when no logger is available."""
    if logger is None:
        print(f'[model_utils] {message}')
        return
    log_fn = getattr(logger, level, None)
    if callable(log_fn):
        try:
            log_fn(message)
            return
        except Exception:  # noqa: BLE001 - never crash on logging
            pass
    print(f'[model_utils] {message}')


def ensure_cache_dir() -> Path:
    """Create and return the model cache directory."""
    try:
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        return CACHE_DIR
    except OSError:
        LEGACY_CACHE_DIR.mkdir(parents=True, exist_ok=True)
        return LEGACY_CACHE_DIR


def _download_with_urllib(url: str, dest: Path) -> bool:
    try:
        dest.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(url, str(dest))  # noqa: S310
        return dest.exists() and dest.stat().st_size > 0
    except Exception:  # noqa: BLE001 - try wget fallback
        return False


def _download_with_wget(url: str, dest: Path) -> bool:
    wget = shutil.which('wget')
    if wget is None:
        return False
    try:
        dest.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [wget, '-q', '-O', str(dest), url],
            check=True,
            timeout=120,
        )
        return dest.exists() and dest.stat().st_size > 0
    except Exception:  # noqa: BLE001 - caller handles failure
        return False


def download_model(model_name: str, dest: Optional[Path] = None,
                   logger=None) -> Optional[Path]:
    """Download a lightweight YOLO model to the cache directory.

    Returns the destination path on success, None on failure.
    """
    cache = ensure_cache_dir()
    target = Path(dest) if dest else (cache / Path(model_name).name)
    if target.exists() and target.stat().st_size > 0:
        return target
    url = DOWNLOAD_URL_TEMPLATE.format(model=Path(model_name).name)
    _log(logger, 'info', f'Downloading {model_name} from {url} ...')
    if _download_with_urllib(url, target):
        _log(logger, 'info', f'Model cached at {target}')
        return target
    if _download_with_wget(url, target):
        _log(logger, 'info', f'Model cached at {target} (via wget)')
        return target
    _log(logger, 'warn', f'Failed to download {model_name} from {url}')
    return None


def _share_candidates(basename: str) -> list[Path]:
    """Return package-share model candidates that exist on disk."""
    candidates: list[Path] = []
    for package in ('person_detection', 'air_mouse_mission'):
        try:
            share = Path(get_package_share_directory(package))
        except Exception:  # noqa: BLE001 - package may be missing
            continue
        for path in (
            share / 'models' / basename,
            share / basename,
        ):
            if path.exists():
                candidates.append(path)
    return candidates


def resolve_model_path(model_param: str,
                       package_name: str = 'air_mouse_mission',
                       logger=None) -> str:
    """Resolve a YOLO weights path dynamically.

    Args:
        model_param: ROS parameter value, e.g. ``best.pt``,
            ``yolov8n.pt`` or an absolute path.
        package_name: preferred package for share lookup.
        logger: optional ROS logger.

    Returns:
        A filesystem path string suitable for ``ultralytics.YOLO``.
        When the requested file is missing, a cached ``yolov8n.pt``
        fallback is returned (downloaded if needed) so the node keeps
        running instead of crashing with [Errno 2].
    """
    raw = (model_param or '').strip() or DEFAULT_MODEL
    basename = os.path.basename(raw) or DEFAULT_MODEL

    # 1. Absolute path.
    if os.path.isabs(raw) and os.path.exists(raw):
        return raw

    # 2. Relative path as given.
    if os.path.exists(raw):
        return os.path.abspath(raw)

    # 3a. Preferred package share.
    try:
        share = Path(get_package_share_directory(package_name))
        for candidate in (share / 'models' / basename, share / basename):
            if candidate.exists():
                _log(logger, 'info', f'Using model {candidate}')
                return str(candidate)
    except Exception:  # noqa: BLE001 - fall through to other candidates
        pass

    # 3b. Known packages.
    for candidate in _share_candidates(basename):
        _log(logger, 'info', f'Using model {candidate}')
        return str(candidate)

    # 4-5. Cache directories.
    for cache in (CACHE_DIR, LEGACY_CACHE_DIR):
        candidate = cache / basename
        if candidate.exists():
            _log(logger, 'info', f'Using cached model {candidate}')
            return str(candidate)

    # 6. Requested file missing: try to fetch it if it is a known
    # lightweight model, otherwise fall back to yolov8n.pt.
    if basename in KNOWN_LIGHTWEIGHT_MODELS:
        downloaded = download_model(basename, logger=logger)
        if downloaded is not None:
            return str(downloaded)
        # Let ultralytics attempt its own auto-download as last resort.
        _log(
            logger, 'warn',
            f'Model {basename} not found locally and download failed; '
            'passing the name to ultralytics for auto-download.',
        )
        return basename

    _log(
        logger, 'warn',
        f'Model {raw!r} not found; falling back to {DEFAULT_MODEL}. '
        f'Place custom weights at {ensure_cache_dir() / basename} '
        'or <share>/person_detection/models/.',
    )
    fallback = download_model(DEFAULT_MODEL, logger=logger)
    if fallback is not None:
        return str(fallback)
    cached_fallback = CACHE_DIR / DEFAULT_MODEL
    if cached_fallback.exists():
        return str(cached_fallback)
    return DEFAULT_MODEL
