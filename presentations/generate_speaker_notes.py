"""SD-17: emit speaker_notes.md from the same slide data the .pptx uses.

Runs as a sibling of `scenic_racing_review.py` -- imports the SLIDES list,
walks it, and produces a human-readable Markdown document with each
slide's title, on-screen bullets, code anchor, video / missing-marker
spec, and the speaker note. Intended for the speaker to read during
practice and/or to glance at during the live talk if Google Slides'
notes pane is hard to see.

Run:
    python presentations/generate_speaker_notes.py

Output:
    presentations/speaker_notes.md
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure the deck script is importable
sys.path.insert(0, str(Path(__file__).parent))

from scenic_racing_review import SLIDES  # noqa: E402


def slide_to_md(sd) -> str:
    parts = []
    parts.append(f"## Slide {sd.n} — {sd.title}\n")
    parts.append(f"**Section:** {sd.section}  \n")
    if sd.code_anchor:
        parts.append(f"**Code anchor:** `{sd.code_anchor}`  \n")
    if sd.video_spec:
        parts.append(f"**Video placeholder:** {sd.video_spec}  \n")
    if sd.missing_marker:
        parts.append(f"**Missing-from-Scenic footer:** *{sd.missing_marker}*  \n")
    parts.append("\n**On screen:**\n\n")
    if sd.bullets:
        for b in sd.bullets:
            parts.append(f"- {b}\n")
    else:
        parts.append("- *(title slide — no bullets)*\n")
    parts.append(f"\n**Speaker note:**\n\n> {sd.speaker_note}\n\n")
    parts.append("---\n\n")
    return "".join(parts)


def build_md() -> str:
    header = (
        "# Scenic Racing Domain — Speaker Notes\n\n"
        "Companion to `scenic_racing_review.pptx`. 34 slides + 12 code-tour\n"
        "stops, 60-min budget (~30 min slides + ~25 min live code + ~5 min Q&A).\n\n"
        "Read this beforehand to lock in the narrative arc; glance during\n"
        "the talk if Google Slides' notes pane is small. The speaker notes\n"
        "anticipate likely audience pushback on the closing asks; rehearse\n"
        "those (slides 31 and 32) most.\n\n"
        "---\n\n"
        "## Quick navigation\n\n"
        "- **Open** (1-3) — frame and roadmap\n"
        "- **Act 1: Scenarios** (4-7) — F-bank + falsifiable bank\n"
        "- **Act 2a: Geometry** (8-13) — RacingCar, RacingTrack, TTL, regions\n"
        "- **Act 2b: Behaviors + MPC** (14-20) — the star behavior, MPC depth, planner\n"
        "- **Act 2c: Falsification** (21-24) — runner, monitors, CE result\n"
        "- **Act 3: dSPACE** (25-29) — bridge, MAPort, `_racing_st_offset`, current state\n"
        "- **Feedback** (30-32) — Ask 1 Frenet, Ask 2 state machines\n"
        "- **Close** (33-34) — recap, Q&A prompts\n\n"
        "**Code-tour stops** are interleaved (see plan file for the exact\n"
        "12-stop order tracked to slide numbers).\n\n"
        "**Mid-deck `Missing-from-Scenic:` footers** appear on slides 12, 18,\n"
        "and 28 — one-line callouts that pre-seed the closing asks so the\n"
        "audience hears them as familiar by the time we frame them formally.\n\n"
        "---\n\n"
    )
    body = "".join(slide_to_md(sd) for sd in SLIDES)
    return header + body


def main() -> int:
    out = Path(__file__).parent / "speaker_notes.md"
    md = build_md()
    out.write_text(md, encoding="utf-8")
    print(f"[notes] wrote {out}  ({len(md)} chars, {md.count(chr(10))} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
