
from __future__ import annotations

from fnmatch import fnmatchcase


def shorthand(label: str) -> str:
    """
    Returns shorthand name of 'label'.
    """
    return label.rsplit("/", 1)[-1]

def match(names: list[str], pattern: str | list[str]) -> list[int]:
    """
    Returns indices of 'names' whose shorthand matches any glob in 'pattern'.
    """
    patterns = [pattern] if isinstance(pattern, str) else list(pattern)
    return [i for i, n in enumerate(names) if any(fnmatchcase(shorthand(n), p) for p in patterns)]
