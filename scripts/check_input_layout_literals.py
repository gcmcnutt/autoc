#!/usr/bin/env python3
"""
check_input_layout_literals.py — the index-free layout gate (044 FR-016 (a), research R14).

The NN input layout is defined in ONE place: the enums + metadata tables of include/autoc/nn/nn_inputs.h
(with the counts they imply in include/autoc/nn/topology.h). Every other hand-written consumer — C++,
Python, the browser decoder — must reach slots by NAME or by a count derived from that definition, so a
layout change is an edit there plus a format break, never a hunt for copies.

This check fails (exit 1, one line per hit) when a hand-written consumer contains:
  (a) a literal input COUNT — a known layout width (any count the M1 / M2 vectors have had) on a code line
      that talks about inputs/slots/columns, e.g. `NUM_INPUTS = 45`, `FL_NUM_IN = 45`, `INPUT_COLS = 45`;
  (b) a literal INDEX into an input vector or its flat view, e.g. `inputs[24]`, `nnInputs[3]`, `rawInputs[7]`;
  (c) a layout width as a slice end or range bound anywhere, e.g. `row[i0:i0 + 45]`, `range(45)` — the
      block-by-position idiom that needs no "input" word on the line to be wrong.
Comments, Python docstrings and lines annotated `layout-ok: <reason>` are ignored; generated code
(xiao/src/generated/) and the definition files themselves are excluded; historical feature scripts
(specs/0[0-3]*) are frozen by policy and not scanned.

Usage:
  check_input_layout_literals.py [--root REPO] [--list-files]
"""
import argparse
import os
import re
import sys

# Every width the M1 (pathgen) and M2 (tracker) input vectors have had, so a stale copy of an OLD width is
# caught too: M1 33/37/42/45 and 044's 53; M2 54/58/63/66 and 044's 74.
LAYOUT_WIDTHS = {33, 37, 42, 45, 53, 54, 58, 63, 66, 74}

SCAN_DIRS = ["src", "include", "tools", "xiao/src", "xiao/include", "xiao/web",
             "specs/043-acro-dual-loop", "specs/044-m1-craft-history"]
EXTS = {".py", ".cc", ".cpp", ".h", ".hpp", ".html", ".js"}
EXCLUDE_PREFIXES = ("xiao/src/generated/",)
DEFINITION_FILES = {"include/autoc/nn/nn_inputs.h", "include/autoc/nn/topology.h"}

INPUT_WORD = re.compile(r"input|slot|num_in|nn_in|_cols|inputcols", re.I)
WIDTH_LITERAL = re.compile(r"(?<![\w.])(%s)(?![\w.])" % "|".join(str(w) for w in sorted(LAYOUT_WIDTHS)))
INDEX_LITERAL = re.compile(r"\b(\w*(?:[Ii]nputs?|INPUTS?)\w*)\s*\[\s*(\d+)\s*\]")
_W = "|".join(str(w) for w in sorted(LAYOUT_WIDTHS))
SLICE_WIDTH = re.compile(r"(?:[+:]\s*(%s)\s*\]|\brange\(\s*(%s)\s*\))" % (_W, _W))
OK_MARK = re.compile(r"layout-ok:")


def strip_cpp_js(text):
    """Remove // and /* */ comments, keep line structure; string literals are kept (they can hold literals)."""
    out, i, n = [], 0, len(text)
    in_block = in_str = False
    quote = ""
    while i < n:
        c = text[i]
        if in_block:
            if text.startswith("*/", i):
                in_block = False; i += 2; continue
            out.append("\n" if c == "\n" else " "); i += 1; continue
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1]); i += 2; continue
            if c == quote:
                in_str = False
            i += 1; continue
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
            continue
        if text.startswith("/*", i):
            in_block = True; i += 2; continue
        if c in "\"'`":
            in_str = True; quote = c
        out.append(c); i += 1
    return "".join(out)


def strip_python(text):
    """Remove # comments and triple-quoted docstrings, keep line structure."""
    out_lines, in_doc, delim = [], False, ""
    for line in text.split("\n"):
        s = line
        if in_doc:
            k = s.find(delim)
            if k < 0:
                out_lines.append(""); continue
            s = s[k + 3:]; in_doc = False
        while True:
            m = re.search(r'("""|\'\'\')', s)
            if not m:
                break
            k = s.find(m.group(1), m.end())
            if k < 0:
                s = s[:m.start()]; in_doc = True; delim = m.group(1); break
            s = s[:m.start()] + s[k + 3:]
        h = s.find("#")
        if h >= 0:
            s = s[:h]
        out_lines.append(s)
    return "\n".join(out_lines)


def files(root):
    for d in SCAN_DIRS:
        base = os.path.join(root, d)
        if not os.path.isdir(base):
            continue
        for dp, _, fns in os.walk(base):
            for fn in fns:
                p = os.path.relpath(os.path.join(dp, fn), root)
                if os.path.splitext(fn)[1] not in EXTS:
                    continue
                if p.startswith(EXCLUDE_PREFIXES) or p in DEFINITION_FILES:
                    continue
                yield p


def scan(root, rel):
    raw = open(os.path.join(root, rel), encoding="utf-8", errors="replace").read()
    code = strip_python(raw) if rel.endswith(".py") else strip_cpp_js(raw)
    raw_lines = raw.split("\n")
    hits = []
    for no, line in enumerate(code.split("\n"), 1):
        if no <= len(raw_lines) and OK_MARK.search(raw_lines[no - 1]):
            continue
        if INPUT_WORD.search(line):
            for m in WIDTH_LITERAL.finditer(line):
                hits.append((no, f"literal input count {m.group(1)}", line.strip()))
        for m in INDEX_LITERAL.finditer(line):
            hits.append((no, f"literal index {m.group(1)}[{m.group(2)}]", line.strip()))
        if not INPUT_WORD.search(line):
            for m in SLICE_WIDTH.finditer(line):
                hits.append((no, f"layout width {m.group(1) or m.group(2)} as a slice/range bound", line.strip()))
    return hits


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ap.add_argument("--list-files", action="store_true")
    a = ap.parse_args()
    all_files = sorted(files(a.root))
    if a.list_files:
        print("\n".join(all_files)); return 0
    total = 0
    for rel in all_files:
        for no, what, line in scan(a.root, rel):
            total += 1
            print(f"{rel}:{no}: {what}: {line[:140]}")
    if total:
        print(f"\ninput-layout gate: {total} hit(s) in {len(all_files)} files — reach slots by name / derived count "
              f"(include/autoc/nn/nn_inputs.h), or annotate a deliberate line with `layout-ok: <reason>`.")
        return 1
    print(f"input-layout gate: clean ({len(all_files)} files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
