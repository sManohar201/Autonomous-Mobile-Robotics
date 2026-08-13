#!/usr/bin/env python3
"""
generate_cards.py
Parse quiz/ and concepts/ topic files from all C++ course modules and output
cards.js for the spaced repetition web app.

Usage (from any directory):
    python course/review/generate_cards.py

Or from course/review/:
    python generate_cards.py
"""

import re
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent
MODULES_DIR = SCRIPT_DIR.parent / "cpp"
OUTPUT_FILE = SCRIPT_DIR / "cards.js"


def slugify(text: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")[:40]


def clean(text: str) -> str:
    lines = [line.rstrip() for line in text.split("\n")]
    while lines and not lines[0].strip():
        lines.pop(0)
    while lines and not lines[-1].strip():
        lines.pop()
    return "\n".join(lines)


def parse_quiz(content: str, mod_id: str, mod_name: str) -> list:
    """Extract Q&A flashcards from quiz.md.

    Handles both formats used in the course:
      Format 1: **Q1.** question text ... <details>..answer..</details>
      Format 2: ### Q1 — Title\n question text ... <details>..answer..</details>
    """
    cards = []
    # Split around each <details> answer block; odd-indexed parts are answers
    parts = re.split(r"<details>\s*<summary>Answer</summary>", content)

    for i in range(1, len(parts)):
        m = re.match(r"([\s\S]*?)</details>([\s\S]*)", parts[i])
        if not m:
            continue
        answer = clean(m.group(1))

        # Question is the last --- section of the preceding part
        q_sections = re.split(r"\n\s*---\s*\n", parts[i - 1])
        question = clean(q_sections[-1])

        # Strip leading section headers like "## Part A — Written Questions"
        question = re.sub(r"^##[^#][^\n]*\n+", "", question).strip()

        if not question or len(question) < 5:
            continue

        cards.append(
            {
                "id": f"{mod_id}_quiz_q{i:02d}",
                "module": mod_id,
                "moduleName": mod_name,
                "type": "quiz",
                "front": question,
                "back": answer,
            }
        )

    return cards


def parse_concepts(content: str, mod_id: str, mod_name: str) -> list:
    """Extract one flashcard per ## section from concepts.md."""
    cards = []
    # Split on ## headers (level 2 only, not ###)
    parts = re.split(r"\n(## [^#][^\n]*)\n", content)
    # parts layout: [preamble, "## header1", "body1", "## header2", "body2", ...]

    i = 1
    while i < len(parts) - 1:
        header = parts[i].lstrip("#").strip()
        body = clean(parts[i + 1]) if i + 1 < len(parts) else ""
        i += 2

        if len(body) < 20:
            continue

        # Trim long sections at a paragraph boundary for card readability
        if len(body) > 900:
            cut = body.rfind("\n\n", 200, 900)
            if cut > 200:
                body = body[:cut] + "\n\n*(see concepts.md for full section)*"
            else:
                body = body[:900] + "..."

        cards.append(
            {
                "id": f"{mod_id}_concept_{slugify(header)}",
                "module": mod_id,
                "moduleName": mod_name,
                "type": "concept",
                "front": f"Explain: {header}",
                "back": body,
            }
        )

    return cards


def parse_concept_file(content: str, mod_id: str, mod_name: str, file_stem: str) -> dict | None:
    """One flashcard from a split topic file (H1 + body)."""
    text = content.strip()
    if not text:
        return None
    lines = text.split("\n")
    header = re.sub(r"^#+\s*", "", lines[0]).strip()
    body = clean("\n".join(lines[1:]))
    if header.lower() == "overview" or len(body) < 20:
        return None
    if len(body) > 900:
        cut = body.rfind("\n\n", 200, 900)
        if cut > 200:
            body = body[:cut] + "\n\n*(see the topic file for the full section)*"
        else:
            body = body[:900] + "..."
    return {
        "id": f"{mod_id}_concept_{slugify(header) or file_stem}",
        "module": mod_id,
        "moduleName": mod_name,
        "type": "concept",
        "front": f"Explain: {header}",
        "back": body,
    }


def load_split_or_single(chapter: Path, name: str) -> tuple[str, list[Path]]:
    """Return ('folder'|'file', paths) for concepts or quiz sources."""
    folder = chapter / name
    if folder.is_dir():
        files = sorted(p for p in folder.glob("*.md") if p.name.lower() != "readme.md")
        if files:
            return "folder", files
    path = chapter / f"{name}.md"
    if path.exists():
        return "file", [path]
    return "missing", []


def module_display_name(d: Path) -> str:
    name = re.sub(r"^\d+_", "", d.name)
    return name.replace("_", " ").title()


def main():
    if not MODULES_DIR.exists():
        print(f"ERROR: modules directory not found: {MODULES_DIR}", file=sys.stderr)
        print("Run this script from the repo root or from course/review/", file=sys.stderr)
        sys.exit(1)

    all_cards = []
    for d in sorted(MODULES_DIR.iterdir()):
        if not d.is_dir():
            continue
        mod_id = d.name
        mod_name = module_display_name(d)

        kind, quiz_paths = load_split_or_single(d, "quiz")
        if kind != "missing":
            quiz_cards = []
            for path in quiz_paths:
                quiz_cards.extend(
                    parse_quiz(path.read_text(encoding="utf-8"), mod_id, mod_name)
                )
            all_cards.extend(quiz_cards)
            print(f"  {mod_id}/quiz → {len(quiz_cards)} cards")

        kind, concept_paths = load_split_or_single(d, "concepts")
        if kind == "folder":
            concept_cards = []
            for path in concept_paths:
                card = parse_concept_file(
                    path.read_text(encoding="utf-8"), mod_id, mod_name, path.stem
                )
                if card:
                    concept_cards.append(card)
            all_cards.extend(concept_cards)
            print(f"  {mod_id}/concepts → {len(concept_cards)} cards")
        elif kind == "file":
            concept_cards = parse_concepts(
                concept_paths[0].read_text(encoding="utf-8"), mod_id, mod_name
            )
            all_cards.extend(concept_cards)
            print(f"  {mod_id}/concepts.md → {len(concept_cards)} cards")

    output = (
        "// AUTO-GENERATED by generate_cards.py — do not edit manually.\n"
        "// Run: python generate_cards.py\n\n"
        f"window.CARDS = {json.dumps(all_cards, ensure_ascii=False, indent=2)};\n"
    )
    OUTPUT_FILE.write_text(output, encoding="utf-8")
    print(f"\nTotal: {len(all_cards)} cards → {OUTPUT_FILE.name}")


if __name__ == "__main__":
    main()
