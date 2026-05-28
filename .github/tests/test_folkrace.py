"""Testid folkrace_driver.py jaoks."""
import glob
import re
import sys


def _find(name):
    matches = [f for f in glob.glob(f'**/{name}', recursive=True) if not f.startswith('.github')]
    if not matches:
        print(f"VIGA: '{name}' ei leitud repos (otsisin kõikidest kaustadest)")
        sys.exit(1)
    return matches[0]


FILE = _find("folkrace_driver.py")


def read_active_code(path):
    """Loe fail ja tagasta ainult aktiivsed koodiread (mitte kommentaarid)."""
    with open(path) as f:
        lines = f.readlines()
    active = []
    for line in lines:
        stripped = line.strip()
        # Jata vahele kommentaarid ja tyhja read
        if stripped.startswith("#") or not stripped:
            continue
        active.append(line)
    return "".join(active)


def read_full(path):
    with open(path) as f:
        return f.read()


def test_front_not_stub():
    """front ei tohi olla algne 8.0 TODO stub."""
    code = read_full(FILE)
    if re.search(r"^\s*front\s*=\s*8\.0\s*#\s*TODO", code, re.M):
        print("FAIL: front on endiselt 8.0 TODO stub")
        return False
    print("OK: front on implementeeritud")
    return True


def test_left_right_not_stub():
    """left ja right ei tohi olla 8.0 TODO stub."""
    code = read_full(FILE)
    errors = []
    if re.search(r"^\s*left\s*=\s*8\.0\s*#\s*TODO", code, re.M):
        errors.append("left")
    if re.search(r"^\s*right\s*=\s*8\.0\s*#\s*TODO", code, re.M):
        errors.append("right")
    if errors:
        print(f"FAIL: {', '.join(errors)} on endiselt stub")
        return False
    print("OK: left ja right implementeeritud")
    return True


def test_reactive_logic():
    """Aktiivses koodis peab olema linear.x ja angular.z määramine."""
    active = read_active_code(FILE)
    if "linear.x" not in active:
        print("FAIL: linear.x puudub aktiivses koodis")
        return False
    if "angular.z" not in active:
        print("FAIL: angular.z puudub aktiivses koodis")
        return False
    print("OK: reactive logic olemas")
    return True


def test_condition_logic():
    """Aktiivses koodis peab olema if-tingimus lidari andmetega."""
    active = read_active_code(FILE)
    has_if = "if " in active
    has_sensor = "front" in active or "left" in active or "right" in active
    if not (has_if and has_sensor):
        print("FAIL: tingimuse loogika puudub aktiivses koodis")
        return False
    print("OK: tingimuse loogika olemas")
    return True


def test_no_main_todos():
    """Pealoogikas ei tohi olla lahendamata TODO-sid."""
    code = read_full(FILE)
    main_todos = [
        l.strip()
        for l in code.split("\n")
        if l.strip().startswith("# TODO: kirjuta")
    ]
    if main_todos:
        print(f"FAIL: {len(main_todos)} lahendamata TODO-d")
        return False
    print("OK: TODO-d lahendatud")
    return True


if __name__ == "__main__":
    test_name = sys.argv[1] if len(sys.argv) > 1 else "all"

    tests = {
        "front": test_front_not_stub,
        "leftright": test_left_right_not_stub,
        "reactive": test_reactive_logic,
        "condition": test_condition_logic,
        "todos": test_no_main_todos,
    }

    if test_name == "all":
        results = {name: fn() for name, fn in tests.items()}
        failed = [n for n, r in results.items() if not r]
        sys.exit(1 if failed else 0)
    elif test_name in tests:
        sys.exit(0 if tests[test_name]() else 1)
    else:
        print(f"Tundmatu test: {test_name}")
        sys.exit(2)
