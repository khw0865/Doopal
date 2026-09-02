# 사용자 음파 UI 적용

## 변경 파일

```text
templates/user.html
static/css/app.css
static/js/user.js
```

관리자 UI, Redis, ROS Bridge, Flask API는 수정하지 않는다.

## 상태별 화면

```text
idle      : 잔잔한 대기 음파
listening : 빠르고 크게 움직이는 청록색 음파
thinking  : 부드럽게 이동하는 파란색 음파
speaking  : 리듬감 있게 움직이는 보라색 음파
error     : 붉은색 오류 음파
```

사용자 문장 또는 로봇 응답 문장이 새로 들어오면 음파가 한 번 크게
반응한다. 실제 마이크 볼륨값을 사용하는 구조는 아니며,
`/api/user/state`에 저장된 상태에 맞춰 애니메이션이 변한다.

## 적용

```bash
cd ~/Desktop/assistive_robot_integrated_update/ui_db

cp templates/user.html templates/user.html.backup
cp static/css/app.css static/css/app.css.backup
cp static/js/user.js static/js/user.js.backup
```

다운로드한 파일을 해당 위치에 덮어쓴다.

```bash
cp ~/Downloads/user.html ./templates/user.html
cp ~/Downloads/app.css ./static/css/app.css
cp ~/Downloads/user.js ./static/js/user.js
```

Flask 재시작:

```bash
./stop_ui_db.sh
./start_ui_db.sh
```

브라우저에 기존 CSS와 JavaScript가 남아 있으면 강력 새로고침:

```text
Ctrl + Shift + R
```

## API 상태 예시

사용자 음성을 듣는 중:

```bash
curl -X POST http://127.0.0.1:5000/api/user/transcript   -H 'Content-Type: application/json'   -d '{"state":"listening","message":"편하게 말씀해 주세요"}'
```

로봇 응답 중:

```bash
curl -X POST http://127.0.0.1:5000/api/user/transcript   -H 'Content-Type: application/json'   -d '{"state":"speaking","message":"두팔이가 대답하고 있어요"}'
```

대기 상태:

```bash
curl -X POST http://127.0.0.1:5000/api/user/transcript   -H 'Content-Type: application/json'   -d '{"state":"idle","message":"말씀해 주세요"}'
```
