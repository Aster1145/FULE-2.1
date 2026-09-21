#!/usr/bin/env python3
"""NumPy / OpenCV compatibility guard for ROS 2 Humble on ARM64.

Background:
  ROS 2 Humble ships ``python3-opencv`` built against NumPy 1.x. A plain
  ``pip install numpy`` on Ubuntu 22.04 ARM64 now pulls NumPy 2.x, which
  breaks ``import cv2`` with::

      AttributeError: _ARRAY_API not found

Fix: pin ``numpy<2.0.0`` (see ``requirements.txt`` / ``constraints.txt``).

This module provides a lightweight runtime check that perception nodes
call *before* importing cv2 so operators get an actionable error instead
of a cryptic AttributeError.

PEP-8 compliant.
"""

from __future__ import annotations

import sys
from typing import Optional


MAX_NUMPY_MAJOR = 1
PIN_HINT = 'pip install "numpy<2.0.0"  # then rebuild / restart nodes'


def parse_major(version: str) -> Optional[int]:
    """Return the major version number, or None if unparseable."""
    try:
        return int(str(version).split('.')[0])
    except (ValueError, AttributeError, IndexError):
        return None


def check_numpy_compat(logger=None) -> bool:
    """Check that the NumPy version is compatible with python3-opencv.

    Returns True when NumPy is missing (nothing to check) or its major
    version is 1.x. Returns False and logs an actionable message when
    NumPy 2.x is detected.
    """
    try:
        import numpy
    except ImportError:
        return True

    major = parse_major(getattr(numpy, '__version__', ''))
    if major is not None and major >= 2:
        message = (
            f'[numpy_compat] Detected NumPy {numpy.__version__}, which is '
            'incompatible with ROS 2 python3-opencv on ARM64 '
            '(AttributeError: _ARRAY_API not found). '
            f'Fix with: {PIN_HINT}'
        )
        if logger is not None:
            try:
                logger.error(message)
            except Exception:  # noqa: BLE001 - logging must not crash import
                print(message, file=sys.stderr)
        else:
            print(message, file=sys.stderr)
        return False
    return True


def import_cv2_guarded(logger=None):
    """Import cv2 with a friendly error when NumPy 2.x breaks it.

    Raises the original exception after logging the pin hint so callers
    can decide to abort or continue in a degraded mode.
    """
    check_numpy_compat(logger=logger)
    try:
        import cv2  # noqa: PLC0415 - intentional lazy import

        return cv2
    except AttributeError as exc:
        if '_ARRAY_API' in str(exc):
            message = (
                '[numpy_compat] import cv2 failed with '
                f'{exc!r}. This is the NumPy 2.x incompatibility. '
                f'Fix with: {PIN_HINT}'
            )
            if logger is not None:
                try:
                    logger.error(message)
                except Exception:  # noqa: BLE001
                    print(message, file=sys.stderr)
            else:
                print(message, file=sys.stderr)
        raise
