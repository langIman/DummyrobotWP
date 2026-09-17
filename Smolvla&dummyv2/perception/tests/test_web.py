from app import create_app


class FakeService:
    def health(self):
        return {"status": "alive", "service": "dummyv2-perception", "version": "test"}


def test_health_contract_without_hardware():
    app = create_app(FakeService())
    app.testing = True
    response = app.test_client().get("/api/health")
    assert response.status_code == 200
    assert response.get_json()["service"] == "dummyv2-perception"


def test_unknown_route_stays_404():
    app = create_app(FakeService())
    app.testing = True
    response = app.test_client().get("/missing")
    assert response.status_code == 404


def test_javascript_uses_executable_mime_type():
    app = create_app(FakeService())
    app.testing = True
    response = app.test_client().get("/static/app.js")
    assert response.status_code == 200
    assert response.mimetype == "application/javascript"
