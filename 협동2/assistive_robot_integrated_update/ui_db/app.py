from __future__ import annotations

import hmac
import os
import threading
from functools import wraps
from typing import Any, Callable, Mapping

from dotenv import load_dotenv
from flask import Flask, jsonify, redirect, render_template, request, session, url_for
from redis.exceptions import RedisError

from redis_store import FIXED_CONFIG_VERSION, RedisStore
from runtime_log import clear_runtime_logs, read_runtime_logs

load_dotenv()

app = Flask(__name__)
app.secret_key = os.getenv("FLASK_SECRET_KEY", "development-only-change-me")
store = RedisStore()

USER_UI_STATE_KEY = "assistive_robot:user_ui_state"
_fixed_data_initialized = False
_fixed_data_lock = threading.Lock()


def admin_required(view: Callable[..., Any]) -> Callable[..., Any]:
    @wraps(view)
    def wrapped(*args: Any, **kwargs: Any) -> Any:
        if not session.get("admin_authenticated"):
            if request.path.startswith("/api/"):
                return jsonify({"ok": False, "message": "admin login required"}), 401
            return redirect(url_for("admin_login"))
        return view(*args, **kwargs)

    return wrapped


@app.errorhandler(RedisError)
def handle_redis_error(error: RedisError):
    app.logger.exception("Redis error")
    return jsonify({
        "ok": False,
        "message": "Redis 연결 또는 인증에 실패했습니다.",
        "detail": str(error),
    }), 503


@app.before_request
def ensure_fixed_data() -> None:
    global _fixed_data_initialized
    if _fixed_data_initialized or request.endpoint == "static":
        return
    with _fixed_data_lock:
        if not _fixed_data_initialized:
            result = store.initialize_fixed_data()
            app.logger.info("Fixed data initialization: %s", result)
            _fixed_data_initialized = True


@app.get("/")
def user_home():
    return render_template("user.html")


@app.route("/admin/login", methods=["GET", "POST"])
def admin_login():
    error = None
    if request.method == "POST":
        expected_user = os.getenv("ADMIN_USERNAME", "admin")
        expected_password = os.getenv("ADMIN_PASSWORD", "admin123")
        submitted_user = request.form.get("username", "")
        submitted_password = request.form.get("password", "")

        if hmac.compare_digest(submitted_user, expected_user) and hmac.compare_digest(
            submitted_password, expected_password
        ):
            session.clear()
            session["admin_authenticated"] = True
            return redirect(url_for("admin_database"))
        error = "아이디 또는 비밀번호를 확인하세요."
    return render_template("login.html", error=error)


@app.get("/admin/logout")
def admin_logout():
    session.clear()
    return redirect(url_for("admin_login"))


@app.get("/admin/database")
@admin_required
def admin_database():
    return render_template("database.html")


@app.get("/api/health")
def api_health():
    return jsonify({"ok": store.ping(), "redis": "connected"})


@app.get("/api/user/state")
def api_user_state_get():
    state = store.redis.hgetall(USER_UI_STATE_KEY)
    return jsonify({
        "state": state.get("state", "idle"),
        "message": state.get("message", "도움이 필요하시면 말씀해 주세요."),
        "user_text": state.get("user_text", ""),
        "assistant_text": state.get("assistant_text", ""),
    })


@app.post("/api/user/transcript")
def api_user_transcript():
    data = request.get_json(silent=True) or {}
    state = str(data.get("state", "idle"))
    message = str(data.get("message", ""))
    session_id = str(data.get("session_id", "default"))
    user_text = str(data.get("user_text", "")).strip()
    assistant_text = str(data.get("assistant_text", "")).strip()

    mapping = {"state": state, "message": message}
    if user_text:
        mapping["user_text"] = user_text
        store.append_conversation(
            role="user",
            text=user_text,
            session_id=session_id,
            source="http_transcript",
            state=state,
        )
    if assistant_text:
        mapping["assistant_text"] = assistant_text
        store.append_conversation(
            role="assistant",
            text=assistant_text,
            session_id=session_id,
            source="http_transcript",
            state=state,
        )
    store.redis.hset(USER_UI_STATE_KEY, mapping=mapping)
    return jsonify({"ok": True})


@app.get("/api/admin/objects")
@admin_required
def api_admin_objects():
    return jsonify({"ok": True, "objects": store.list_objects()})


def _extract_freeform_object(payload: Mapping[str, Any]) -> tuple[str, Mapping[str, Any], bool]:
    record_name = (
        payload.get("record_name")
        or payload.get("object_name")
        or payload.get("class_name")
    )
    if record_name is None:
        raise ValueError("record_name is required")

    if "data" in payload:
        object_data = payload.get("data")
        if not isinstance(object_data, Mapping):
            raise ValueError("data must be a JSON object")
    else:
        # 이전 API 형식도 자유 JSON 레코드로 저장되게 호환합니다.
        object_data = {
            key: value
            for key, value in payload.items()
            if key not in {"record_name", "object_name", "replace"}
        }

    return str(record_name), object_data, bool(payload.get("replace", True))


@app.post("/api/admin/objects")
@admin_required
def api_admin_objects_create():
    payload = request.get_json(silent=True) or {}
    try:
        record_name, object_data, replace = _extract_freeform_object(payload)
        item = store.save_object_record(
            record_name=record_name,
            data=object_data,
            replace=replace,
        )
    except (KeyError, TypeError, ValueError) as error:
        return jsonify({"ok": False, "message": str(error)}), 400
    return jsonify({"ok": True, "object": item})


@app.delete("/api/admin/objects/<record_name>")
@admin_required
def api_admin_objects_delete(record_name: str):
    return jsonify({"ok": True, "deleted": store.delete_object(record_name)})


@app.get("/api/admin/fixed-config")
@admin_required
def api_admin_fixed_config():
    return jsonify({
        "ok": True,
        "version": FIXED_CONFIG_VERSION,
        "fixed_points": store.list_fixed_points(),
        "scan_cases": store.list_scan_cases(),
    })


# 이전 프런트엔드/외부 코드 호환용 API
@app.get("/api/admin/fixed-points")
@admin_required
def api_admin_fixed_points():
    return jsonify({"ok": True, "fixed_points": store.list_fixed_points()})


@app.get("/api/admin/scan-cases")
@admin_required
def api_admin_scan_cases():
    return jsonify({"ok": True, "scan_cases": store.list_scan_cases()})


@app.get("/api/admin/conversations")
@admin_required
def api_admin_conversations():
    limit = request.args.get("limit", "500")
    try:
        parsed_limit = int(limit)
    except ValueError:
        parsed_limit = 500
    return jsonify({
        "ok": True,
        "conversations": store.list_conversations(parsed_limit),
    })


@app.delete("/api/admin/conversations")
@admin_required
def api_admin_conversations_delete():
    return jsonify({"ok": True, "deleted": store.clear_conversations()})


@app.get("/api/admin/runtime-logs")
@admin_required
def api_admin_runtime_logs():
    limit = request.args.get("limit", "500")
    try:
        parsed_limit = int(limit)
    except ValueError:
        parsed_limit = 500
    return jsonify({
        "ok": True,
        "logs": read_runtime_logs(parsed_limit),
    })


@app.delete("/api/admin/runtime-logs")
@admin_required
def api_admin_runtime_logs_delete():
    return jsonify({"ok": True, "deleted": clear_runtime_logs()})


def main() -> None:
    store.ping()
    fixed_result = store.initialize_fixed_data()
    app.logger.info("Fixed data initialization: %s", fixed_result)

    # 모든 네트워크 인터페이스에서 접속을 허용한다.
    # 같은 Wi-Fi의 다른 기기는 이 PC의 사설 IP와 5000번 포트로 접속한다.
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        use_reloader=False,
        threaded=True,
    )


if __name__ == "__main__":
    main()
