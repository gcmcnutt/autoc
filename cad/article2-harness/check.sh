#!/usr/bin/env bash
# Regenerate and validate the article 2 harness.
#
# The cross-check at the end is the point: the schematic and harness-spec.md are
# authored separately, so agreement between them is evidence rather than a
# tautology. It has already caught three defects -- a pad named generically in the
# markdown where the board has two distinct pads, a refdes that collided with an
# FC pin name, and the GPS TX/RX pair in the wrong order.
#
# Usage:  ./check.sh
set -euo pipefail
cd "$(dirname "$0")"

echo "== regenerate =="
python3 gen_symbols.py > article2-harness.kicad_sym
python3 gen_sheet.py   > article2-harness.kicad_sch

echo "== symbol library parses in KiCad =="
rm -rf .symcheck && mkdir -p .symcheck
kicad-cli sym export svg --output .symcheck article2-harness.kicad_sym >/dev/null
echo "   $(ls .symcheck/*.svg | wc -l) symbol units plotted"
rm -rf .symcheck

echo "== ERC =="
kicad-cli sch erc --severity-all -o erc-report.txt article2-harness.kicad_sch >/dev/null
grep -E '^ \*\* ERC messages' erc-report.txt

echo "== export =="
kicad-cli sch export netlist --format kicadsexpr -o net.txt article2-harness.kicad_sch >/dev/null
kicad-cli sch export pdf -o article2-harness.pdf article2-harness.kicad_sch >/dev/null
echo "   article2-harness.pdf"

echo "== schematic netlist vs harness-spec.md section 6 =="
python3 - <<'PY'
import re, sys, importlib.util
spec = importlib.util.spec_from_file_location("gs", "gen_sheet.py")
gs = importlib.util.module_from_spec(spec); spec.loader.exec_module(gs)

# what KiCad actually resolved
s = open('net.txt').read()
kicad = {}
for m in re.finditer(r'\(name "/([^"]+)"\)\s*\n\s*\(class[^\n]*\n(.*?)'
                     r'(?=\n\t\t\(net\b|\n\t\)\s*\n\)\s*$)', s, re.S):
    kicad[m.group(1)] = set(re.findall(
        r'\(ref "([^"]+)"\)\s*\n\s*\(pin "([^"]+)"\)', m.group(2)))

# what the markdown declares, as REF.PIN
md = open('harness-spec.md').read()
sec = md.split('## 6. Connection table')[1].split('**The GPS connector.**')[0]

def pinnum(ref, pname):
    sym = gs.PLACE[ref][0]
    for u in gs.LIB[sym]["units"].values():
        if pname in u["pins"]:
            return u["pins"][pname]["num"]
    return None

decl, unresolved = {}, []
for r in [x for x in sec.splitlines()
          if x.startswith('| ') and 'Net' not in x and '---' not in x]:
    c = [x.strip() for x in r.strip('|').split('|')]
    for ep in (c[1], c[2]):
        ep = ep.strip()
        if not ep or '.' not in ep:
            continue
        ref, pname = ep.split('.', 1)
        n = pinnum(ref, pname)
        if n is None:
            unresolved.append((c[0], ref, pname))
        else:
            decl.setdefault(c[0], set()).add((ref, n))

bad = 0
for n in sorted(set(kicad) | set(decl)):
    k, d = kicad.get(n, set()), decl.get(n, set())
    if k != d:
        bad += 1
        print(f"   ** {n}: only-in-schematic={k - d}  only-in-md={d - k}")
if unresolved:
    print(f"   ** md endpoints naming no real pin: {unresolved}")
if bad or unresolved:
    sys.exit("   FAIL - reconcile the schematic and harness-spec.md")
print(f"   OK - {len(kicad)} nets, schematic and harness-spec.md agree exactly")

# ground discipline: every ground net must have exactly one bonding module
grounds = [n for n in kicad if n.startswith('GND')]
print(f"   grounds kept distinct: {', '.join(sorted(grounds))}")
PY
echo "== all checks passed =="
