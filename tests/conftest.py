"""
Pytest configuration for unit tests.

Mocks SQLAlchemy so that backend modules that declare ORM models can be
imported in unit tests without requiring a live database or sqlalchemy
to be installed in the test environment.

Only adds mocks for modules not already present in sys.modules, so this
is safe to use alongside integration tests that do use the real sqlalchemy.
"""

import sys
from unittest.mock import MagicMock


def pytest_configure(config):
    sqla_mock = MagicMock()
    for mod in [
        "sqlalchemy",
        "sqlalchemy.orm",
        "sqlalchemy.ext",
        "sqlalchemy.ext.declarative",
    ]:
        if mod not in sys.modules:
            sys.modules[mod] = sqla_mock
