#!/usr/bin/env bash
# S4d-M shadow (OD12, M15): the RUN installation (the fresh disabled installation this run sourced)
# verified in a fresh environment after the restore verification: env -i; the documented toolchain,
# the verified r2 overlay local_setup, then ONLY that installation's setup.bash; imported from /.
# Writes the JSON that score_s4d.py --run-install-gates reads,
#   {"install_base": ..., "gates": {"spiderx_controller.<module>.<GATE>": false, ...}},
# and, separately, where each gate module was imported from. Read-only otherwise.
# Usage: run_install_gates.sh <install prefix> <out dir>   Exit 0 only if all three are False and
# every gate module was imported from that installation.
P=$1; O=$2; mkdir -p "$O"
env -i HOME="$HOME" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin P="$P" O="$O" \
  bash --noprofile --norc -c '
source /opt/mm/activate.sh >/dev/null 2>&1
source /root/spiderx_gz_overlay_ws_r2/install/local_setup.bash
source $P/setup.bash
cd / && PYTHONDONTWRITEBYTECODE=1 python3 - "$P" "$O" <<"PYEOF"
import importlib, json, os, sys
p, o = os.path.abspath(sys.argv[1]), sys.argv[2]
gates, mods = {}, {}
for mod, name in (("m61_live_contract", "M61_LIVE_DISPATCH_ENABLED"),
                  ("m6_live_contract", "LIVE_DISPATCH_ENABLED"),
                  ("m55_contract", "M55_LOCOMOTION_DISPATCH_ENABLED")):
    m = importlib.import_module("spiderx_controller." + mod)
    gates["spiderx_controller.%s.%s" % (mod, name)] = getattr(m, name)
    mods[mod] = {"imported_as": m.__file__, "resolves_to": os.path.realpath(m.__file__),
                 "imported_from_this_installation": os.path.abspath(m.__file__).startswith(p + os.sep)}
with open(os.path.join(o, "run_install_gates.json"), "w") as f:
    json.dump({"install_base": p, "gates": gates}, f, indent=1, sort_keys=True)
inside = all(x["imported_from_this_installation"] for x in mods.values())
with open(os.path.join(o, "run_install_modules.json"), "w") as f:
    json.dump({"install_base": p, "modules": mods, "all_from_this_installation": inside,
               "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH"),
               "PYTHONPATH": os.environ.get("PYTHONPATH")}, f, indent=1, sort_keys=True)
ok = inside and set(gates.values()) == {False} and all(v is False for v in gates.values())
for k, v in sorted(gates.items()):
    print(k, v)
for k, v in sorted(mods.items()):
    print(k, v["imported_as"], "->", v["resolves_to"])
print("RUN INSTALLATION GATES:", "PASS: all three False, imported from " + p if ok else "FAIL")
sys.exit(0 if ok else 1)
PYEOF'
