import pytest
import requests


@pytest.fixture(autouse=True)
def no_real_network(monkeypatch):
    def blocked(*args, **kwargs):
        raise AssertionError("Tests must use a fake bridge, never physical hardware")
    monkeypatch.setattr(requests.sessions.Session, "request", blocked)
