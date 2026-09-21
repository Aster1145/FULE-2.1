#!/usr/bin/env python3
"""Check NumPy / OpenCV compatibility for FULE-2.1.

Fails (exit 1) when NumPy 2.x is installed or `import cv2` raises the
`_ARRAY_API` AttributeError. Run in CI or on the VM before launching:

    python3 scripts/check_numpy_compat.py
    pip3 install "numpy<2.0.0"  # fix
"""

import sys


def main() -> int:
    """Run compatibility checks, returning 0 on success."""
    try:
        import numpy

        print(f'numpy: {numpy.__version__}')
        major = int(str(numpy.__version__).split('.')[0])
        if major >= 2:
            print(
                'ERROR: NumPy 2.x is incompatible with ROS 2 python3-opencv '
                'on ARM64 (AttributeError: _ARRAY_API not found).',
                file=sys.stderr,
            )
            print('Fix: pip3 install --upgrade "numpy<2.0.0"', file=sys.stderr)
            return 1
    except ImportError:
        print('numpy: not installed (skipping version check)')
    except (ValueError, IndexError):
        print('numpy: unparseable version (skipping)')

    try:
        import cv2  # noqa: F401

        print(f'cv2: OK ({cv2.__version__})')
    except AttributeError as exc:
        if '_ARRAY_API' in str(exc):
            print(f'ERROR: import cv2 failed: {exc!r}', file=sys.stderr)
            print(
                'This is the NumPy 2.x incompatibility. '
                'Fix: pip3 install --upgrade "numpy<2.0.0"',
                file=sys.stderr,
            )
            return 1
        raise
    except ImportError as exc:
        print(f'cv2: not installed ({exc})')
        return 1

    print('OK: NumPy / OpenCV compatible.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
