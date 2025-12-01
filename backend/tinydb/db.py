from tinydb import TinyDB, Query
from datetime import datetime
from pathlib import Path
import uuid

#db = TinyDB(r"C:\Users\tutun\drawmemaybe\DrawMeMaybe\backend\tinydb\db.json")
BASE_DIR = Path(__file__).resolve().parent    # backend/
DB_DIR = BASE_DIR / "data"
DB_DIR.mkdir(exist_ok=True)                  # ensure the folder exists

DB_PATH = DB_DIR / "db.json"

db = TinyDB(DB_PATH)

sessions_table = db.table("sessions")
consents_table = db.table("consents")


def now_iso() -> str:
    return datetime.utcnow().isoformat() + "Z"


def create_session() -> str:
    session_id = str(uuid.uuid4())
    sessions_table.insert(
        {
            "session_id": session_id,
            "created_at": now_iso(),
            "status": "created",
        }
    )
    return session_id


def save_consent(session_id: str, consent_given: bool, name: str | None = None):
    entry = {
        "session_id": session_id,
        "consent_given": consent_given,
        "timestamp": now_iso(),
    }
    if name is not None:
        entry["name"] = name

    consents_table.insert(entry)


def get_session(session_id: str):
    Session = Query()
    return sessions_table.get(Session.session_id == session_id)

def list_sessions():
    """Return all sessions as a list of dicts."""
    return sessions_table.all()


def list_consents():
    """Return all consents as a list of dicts."""
    return consents_table.all()

def complete_session(session_id: str) -> bool:
    """Markiert eine Session als abgeschlossen."""
    Session = Query()
    updated = sessions_table.update(
        {
            "status": "completed",
            "completed_at": now_iso(),
        },
        Session.session_id == session_id,
    )
    # TinyDB update() gibt eine Liste der geänderten Dokument-IDs zurück
    return len(updated) > 0
