"""Compile selected production functions against host registers and device stubs."""
import re


def extract_functions(path, names):
    source = path.read_text(encoding="utf-8")
    functions = []
    for name in names:
        # Project functions and their closing braces start at column zero.
        pattern = r"^[\w *]+\b" + re.escape(name) + r"\([^;{}]*\)\s*\n\{.*?^\}"
        matches = re.findall(pattern, source, re.MULTILINE | re.DOTALL)
        if len(matches) != 1:
            raise ValueError(f"Expected one definition of {name} in {path}")
        functions.append(matches[0] + "\n")
    return "\n".join(functions)
