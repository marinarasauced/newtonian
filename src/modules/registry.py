
from __future__ import annotations

from collections.abc import Callable
from typing import Generic, TypeVar

T = TypeVar("T")


class Registry(Generic[T]):
    """
    """
    def __init__(self, kind: str) -> None:
        """
        """
        self.kind = kind
        self._items: dict[str, T] = {}


    def register(self, name: str) -> Callable[[T], T]:
        """
        """
        def decorator(item: T) -> T:
            if name in self._items:
                raise KeyError(f"{self.kind} type '{name}' is already registered")
            self._items[name] = item
            return item

        return decorator


    def get(self, name: str) -> T:
        """
        """
        try:
            return self._items[name]
        except KeyError:
            raise KeyError(f"{self.kind} type '{name}' is unknown") from None


    def names(self) -> list[str]:
        """
        """
        return sorted(self._items)


    def __contains__(self, name: str) -> bool:
        """
        """
        return name in self._items
