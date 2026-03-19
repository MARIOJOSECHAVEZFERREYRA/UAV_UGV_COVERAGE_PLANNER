"""Serve test field polygons from data/test_fields/."""
import json
from pathlib import Path

from fastapi import APIRouter, HTTPException

router = APIRouter(prefix="/fields", tags=["fields"])

FIELDS_DIR = Path(__file__).resolve().parents[2] / "tests" / "test_fields"


def _all_fields() -> list[dict]:
    fields = []
    for f in sorted(FIELDS_DIR.rglob("*.json")):
        category = f.parent.name  # basic / organic / stress_tests
        fields.append({"name": f.stem, "category": category, "path": str(f)})
    return fields


@router.get("/")
def list_fields():
    return [{"name": f["name"], "category": f["category"]} for f in _all_fields()]


@router.get("/{name}")
def get_field(name: str):
    for f in _all_fields():
        if f["name"] == name:
            return json.loads(Path(f["path"]).read_text())
    raise HTTPException(status_code=404, detail=f"Field '{name}' not found")
