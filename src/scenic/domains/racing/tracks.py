"""Racing track representation — re-exported from segments for backward compatibility.

The implementation lives in :obj:`scenic.domains.racing.segments.tracks`.
Import from here or from segments.tracks.
"""

from scenic.domains.racing.segments.tracks import (
    RacingTrack,
    createRacingTrack,
    PitLane,
    RacingLine,
)

__all__ = [
    "RacingTrack",
    "createRacingTrack",
    "PitLane",
    "RacingLine",
]
