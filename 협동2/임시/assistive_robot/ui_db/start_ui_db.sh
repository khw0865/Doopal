#!/usr/bin/env bash
set -Eeuo pipefail

# 이 스크립트를 UI 프로젝트 루트(app.py, redis_store.py가 있는 폴더)에 둡니다.
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/humble/setup.bash}"
ROS_WS="${ROS_WS:-$HOME/ws_cobot_pjt/ws_dsr}"
PYTHON_BIN="$PROJECT_DIR/.venv/bin/python3"
PID_FILE="$PROJECT_DIR/.ui_db.pids"
LOG_DIR="$PROJECT_DIR/logs"

cd "$PROJECT_DIR"
mkdir -p "$LOG_DIR"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

[[ -f "$PROJECT_DIR/.env" ]] || fail ".env 파일이 없습니다: $PROJECT_DIR/.env"
[[ -f "$PROJECT_DIR/app.py" ]] || fail "app.py 파일이 없습니다: $PROJECT_DIR/app.py"
[[ -f "$PROJECT_DIR/redis_store.py" ]] || fail "redis_store.py 파일이 없습니다: $PROJECT_DIR/redis_store.py"
[[ -f "$PROJECT_DIR/docker-compose.yml" || -f "$PROJECT_DIR/compose.yml" || -f "$PROJECT_DIR/compose.yaml" ]] \
    || fail "docker-compose.yml 또는 compose.yml 파일이 없습니다."
[[ -f "$ROS_DISTRO_SETUP" ]] || fail "ROS 2 setup 파일이 없습니다: $ROS_DISTRO_SETUP"

BRIDGE_FILE=""
if [[ -f "$PROJECT_DIR/ros_object_bridge.py" ]]; then
    BRIDGE_FILE="$PROJECT_DIR/ros_object_bridge.py"
elif [[ -f "$PROJECT_DIR/ros_object_bridge_with_query.py" ]]; then
    BRIDGE_FILE="$PROJECT_DIR/ros_object_bridge_with_query.py"
else
    fail "ros_object_bridge.py 또는 ros_object_bridge_with_query.py가 없습니다."
fi

# 이미 실행 중인지 확인합니다.
if [[ -f "$PID_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$PID_FILE" || true
    BRIDGE_ALIVE=false
    FLASK_ALIVE=false
    [[ -n "${BRIDGE_PID:-}" ]] && kill -0 "$BRIDGE_PID" 2>/dev/null && BRIDGE_ALIVE=true
    [[ -n "${FLASK_PID:-}" ]] && kill -0 "$FLASK_PID" 2>/dev/null && FLASK_ALIVE=true

    if [[ "$BRIDGE_ALIVE" == true || "$FLASK_ALIVE" == true ]]; then
        echo "[INFO] UI/DB가 이미 실행 중입니다."
        echo "       Bridge PID: ${BRIDGE_PID:-없음}"
        echo "       Flask PID : ${FLASK_PID:-없음}"
        echo "       로그 확인 : ./logs_ui_db.sh"
        exit 0
    fi
    rm -f "$PID_FILE"
fi

# ROS 2 setup.bash 내부에서는 아직 정의되지 않은 환경변수를 참조할 수 있습니다.
# 이 스크립트는 set -u(nounset)를 사용하므로, ROS 환경을 불러오는 동안에만
# nounset을 잠시 해제한 뒤 다시 활성화합니다.
set +u
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"
if [[ -f "$ROS_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$ROS_WS/install/setup.bash"
else
    echo "[WARN] 워크스페이스 setup 파일이 없어 ROS 인터페이스 import가 실패할 수 있습니다:"
    echo "       $ROS_WS/install/setup.bash"
fi
set -u

command -v docker >/dev/null 2>&1 || fail "docker 명령을 찾을 수 없습니다."
docker compose version >/dev/null 2>&1 || fail "docker compose 플러그인을 사용할 수 없습니다."

# 가상환경이 없으면 ROS Python 패키지를 볼 수 있도록 생성합니다.
if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "[SETUP] Python 가상환경 생성"
    python3 -m venv --system-site-packages "$PROJECT_DIR/.venv"
fi

if [[ -f "$PROJECT_DIR/requirements.txt" ]]; then
    echo "[SETUP] Python 패키지 확인"
    "$PYTHON_BIN" -m pip install -q -r "$PROJECT_DIR/requirements.txt"
fi

# Redis를 먼저 실행합니다. Compose는 같은 폴더의 .env를 자동으로 읽습니다.
echo "[START] Redis"
docker compose up -d

# 이전 로그는 보존하고 실행 구분선을 추가합니다.
{
    echo
    echo "===== $(date '+%Y-%m-%d %H:%M:%S') Bridge started ====="
} >> "$LOG_DIR/bridge.log"
{
    echo
    echo "===== $(date '+%Y-%m-%d %H:%M:%S') Flask started ====="
} >> "$LOG_DIR/flask.log"

# 백그라운드 실행. 현재 셸이 종료되어도 프로세스를 유지합니다.
echo "[START] ROS Object Bridge: $(basename "$BRIDGE_FILE")"
nohup "$PYTHON_BIN" -u "$BRIDGE_FILE" >> "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

echo "[START] Flask UI"
nohup "$PYTHON_BIN" -u "$PROJECT_DIR/app.py" >> "$LOG_DIR/flask.log" 2>&1 &
FLASK_PID=$!

cat > "$PID_FILE" <<PIDS
BRIDGE_PID=$BRIDGE_PID
FLASK_PID=$FLASK_PID
PIDS

sleep 2

FAILED=false
if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "[ERROR] Bridge가 시작 직후 종료되었습니다."
    tail -n 30 "$LOG_DIR/bridge.log" || true
    FAILED=true
fi
if ! kill -0 "$FLASK_PID" 2>/dev/null; then
    echo "[ERROR] Flask가 시작 직후 종료되었습니다."
    tail -n 30 "$LOG_DIR/flask.log" || true
    FAILED=true
fi

if [[ "$FAILED" == true ]]; then
    "$PROJECT_DIR/stop_ui_db.sh" --keep-redis >/dev/null 2>&1 || true
    exit 1
fi

echo
echo "[OK] UI/DB 실행 완료"
echo "     Redis  : docker compose ps"
echo "     Bridge : PID $BRIDGE_PID"
echo "     Flask  : PID $FLASK_PID"
echo "     UI     : http://127.0.0.1:5000"
echo "     로그   : ./logs_ui_db.sh"
echo "     종료   : ./stop_ui_db.sh"
