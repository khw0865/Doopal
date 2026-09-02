const stateLabels = {idle:'IDLE', listening:'LISTENING', thinking:'THINKING', speaking:'SPEAKING', error:'ERROR'};

async function updateUserState(){
  try{
    const response = await fetch('/api/user/state');
    const data = await response.json();
    const state = String(data.state || 'idle').toLowerCase();
    document.body.dataset.state = state;
    document.getElementById('stateText').textContent = stateLabels[state] || state.toUpperCase();
    document.getElementById('mainMessage').textContent = data.message || '도움이 필요하시면 말씀해 주세요.';
    document.getElementById('userText').textContent = data.user_text || '아직 인식된 문장이 없습니다.';
    document.getElementById('assistantText').textContent = data.assistant_text || '대기 중입니다.';
  }catch(error){
    document.body.dataset.state = 'error';
    document.getElementById('stateText').textContent = 'DISCONNECTED';
    document.getElementById('mainMessage').textContent = 'UI 서버 연결을 확인해 주세요.';
  }
}
updateUserState();
setInterval(updateUserState, 700);
