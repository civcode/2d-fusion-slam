"""Deterministic synthetic workshop FUSMAP2D fixture for an empty demo.

Never uses hardware, user files, or the graph optimizer. The resulting
binary obeys the exact C++ v1 map format and is validated before serving.
Generated only into a disposable private directory, never a map registry.
"""
from __future__ import annotations

from contextlib import contextmanager
from pathlib import Path
import struct
import tempfile

from .bundle import FNV_START, MapBundle, fnv_update, fnv_word

EXAMPLE_NAME = "sample-synthetic-workshop.slammap"
WIDTH, HEIGHT, RESOLUTION = 120, 90, .1
REVISION = 1


def _inside(x: float, y: float, bounds: tuple[float, float, float, float]) -> bool:
    a, b, c, d = bounds
    return a <= x <= b and c <= y <= d


def _cell(x: int, y: int) -> tuple[int, float]:
    """A schematic room with obstacles, free space and unknown areas.

    This is a synthetic *fixture*, not a reconstructed LiDAR observation.
    Unknown pockets illustrate why unknown and occupied must be distinct.
    """
    px, py = (x + .5) * RESOLUTION, (y + .5) * RESOLUTION
    if not _inside(px, py, (.5, 11.5, .5, 8.5)):
        return 0, 0.0
    if (px > 10.1 and py > 6.0) or (px < 1.8 and py > 6.1):
        return 0, 0.0
    walls = px < .7 or px > 11.3 or py < .7 or py > 8.3
    boxes = (
        (3.05, 4.25, 1.85, 4.4),
        (6.65, 8.15, 4., 5.35),
        (8.9, 10.15, 1.85, 3.7),
        (5.5, 6., 6.6, 8.1),
        (1.25, 2.15, 6.55, 7.8),
    )
    if walls or any(_inside(px, py, rectangle) for rectangle in boxes):
        return 1, 2.5
    # Reserved negative log-odds means observed free space. Do not use
    # 0.0 log-odds for unknown cells while their observed flag is false.
    return 1, -2.0


def create_example_bundle() -> bytes:
    """Construct independently checkable native-format FUSMAP2D v1 bytes."""
    cells = [_cell(x, y) for y in range(HEIGHT) for x in range(WIDTH)]
    fp = FNV_START
    for value in (WIDTH, HEIGHT):
        fp = fnv_word(fp, value)
    for value in (RESOLUTION, 0.0, 0.0, 0.0):
        fp = fnv_word(fp, struct.unpack('<Q', struct.pack('<d', value))[0])
    fp = fnv_word(fp, REVISION)
    for observed, odds in cells:
        fp = fnv_word(fp, struct.unpack('<Q', struct.pack('<d', odds))[0])
        fp = fnv_word(fp, observed)

    raw = bytearray(b'FUSMAP2D')
    raw.extend(struct.pack('<4I', 1, 0, WIDTH, HEIGHT))
    raw.extend(struct.pack('<8d', RESOLUTION, 0., 0., 0., .85, -.4, -5., 5.))
    raw.extend(struct.pack('<IQQ', 0, REVISION, fp))
    for label in (b'synthetic_demo_frame', b'schematic_virtual_workshop', b'synthetic_exact'):
        raw.extend(struct.pack('<I', len(label)))
        raw.extend(label)
    raw.extend(struct.pack('<Q', len(cells)))
    for observed, odds in cells:
        raw.extend(struct.pack('<Bd', observed, odds))
    raw.extend(struct.pack('<Q', fnv_update(FNV_START, raw)))

    # Do not serve a fixture that the same rigorous parser rejects.
    map_file = MapBundle(bytes(raw))
    if map_file.truth_class != 'synthetic_exact' or map_file.metric_origin_verified:
        raise RuntimeError('synthetic map provenance invalid')
    return bytes(raw)


@contextmanager
def data_dir_or_example(directory: Path, use_example: bool = False):
    """Use existing maps/registry unmodified, otherwise a disposable example.

    The user's directory is never written, even when it is mounted read-only.
    A malformed CURRENT or any .slammap file prevents a fallback that might
    hide a broken registry/map from the operator.
    """
    user_dir = directory.resolve(strict=True)
    if not user_dir.is_dir():
        raise ValueError('map directory is not a directory')
    has_data = any(p.name == 'CURRENT' or p.name.endswith('.slammap')
                   for p in user_dir.iterdir())
    if not use_example or has_data:
        yield user_dir
        return
    with tempfile.TemporaryDirectory(prefix='fusion-slam-synthetic-') as temp:
        example_dir = Path(temp)
        (example_dir / EXAMPLE_NAME).write_bytes(create_example_bundle())
        yield example_dir
