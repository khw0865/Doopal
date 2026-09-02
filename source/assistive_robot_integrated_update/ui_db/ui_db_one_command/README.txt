사용법
======

1. start_ui_db.sh, stop_ui_db.sh, logs_ui_db.sh를 다음 파일들이 있는 UI 프로젝트 루트에 복사합니다.
   - app.py
   - redis_store.py
   - ros_object_bridge.py 또는 ros_object_bridge_with_query.py
   - docker-compose.yml
   - requirements.txt
   - .env

2. 실행 권한을 부여합니다.
   chmod +x start_ui_db.sh stop_ui_db.sh logs_ui_db.sh

3. 한 번에 실행합니다.
   ./start_ui_db.sh

4. 로그를 확인합니다.
   ./logs_ui_db.sh

5. 모두 종료합니다.
   ./stop_ui_db.sh

기본 ROS 워크스페이스는 ~/ws_cobot_pjt/ws_dsr 입니다.
다른 워크스페이스를 사용할 경우:
   ROS_WS=~/다른_워크스페이스 ./start_ui_db.sh
