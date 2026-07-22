(() => {
  // Script do cliente: captura teclado/botões, envia eventos via Socket.IO
  // e exibe status de conexão e confirmação de entrega (ACK) do servidor.
  // Também gerencia o modo de controle (web vs gamepad).

  const connEl = document.getElementById('conn');
  const pressedEl = document.getElementById('pressed');
  const logEl = document.getElementById('log');
  const deliveryEl = document.getElementById('delivery');
  const modeDisplay = document.getElementById('mode-display');
  const pressedRow = document.getElementById('pressed-row');
  const gamepadStatusRow = document.getElementById('gamepad-status-row');
  const webControls = document.getElementById('web-controls');
  const gamepadControls = document.getElementById('gamepad-controls');

  // Rastreia teclas pressionadas para evitar flood por autorepeat
  const pressed = new Set();

  // Modo ativo: 'web' ou 'gamepad'
  let controlMode = 'web';

  // Controle de movimento pela web habilitado? Definido pelo servidor no
  // evento mode_info (web_teleop). Default true só até o mode_info chegar —
  // com a Fase 2 o servidor manda false por padrão (movimento via PS4/WASD).
  let webTeleop = true;

  // Prefere polling e faz upgrade para websocket (melhor compatibilidade)
  const socket = io({ transports: ['polling', 'websocket'] });
  // Exposto para outros scripts (map.js) reutilizarem a mesma conexão
  window.robotSocket = socket;

  // --- Controle de velocidade (compartilhado entre modos) ---
  const speedSlider = document.getElementById('speed-slider');
  const speedMultDisplay = document.getElementById('speed-mult-display');
  const speedLinearVal = document.getElementById('speed-linear-val');
  const speedAngularVal = document.getElementById('speed-angular-val');
  // Valores em "unidades internas" exibidos na UI (não SI). O servidor
  // converte a velocidade SI real (m/s, rad/s) via BASE_LINEAR_SPEED e
  // BASE_ANGULAR_SPEED em robot_controller.py; estes aqui só dão um número
  // estável pro slider mostrar enquanto o ack do servidor não chega.
  const BASE_LINEAR = 100;
  const BASE_ANGULAR = 65;
  let currentMultiplier = 1.0;

  function updateSpeedUI(mult, linearSpeed, angularSpeed) {
    currentMultiplier = mult;
    if (speedSlider) speedSlider.value = mult;
    if (speedMultDisplay) speedMultDisplay.textContent = mult.toFixed(1) + 'x';
    if (speedLinearVal) speedLinearVal.textContent = Math.round(linearSpeed || BASE_LINEAR * mult);
    if (speedAngularVal) speedAngularVal.textContent = Math.round(angularSpeed || BASE_ANGULAR * mult);
    // Destaca o preset ativo
    document.querySelectorAll('.speed-preset-btn').forEach(b => {
      const bm = parseFloat(b.getAttribute('data-mult'));
      b.classList.toggle('active', Math.abs(bm - mult) < 0.05);
    });
  }

  function sendSpeed(mult) {
    socket.emit('set_speed', { multiplier: mult });
  }

  if (speedSlider) {
    speedSlider.addEventListener('input', () => {
      const mult = parseFloat(speedSlider.value);
      updateSpeedUI(mult);
      sendSpeed(mult);
    });
  }

  document.querySelectorAll('.speed-preset-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const mult = parseFloat(btn.getAttribute('data-mult'));
      updateSpeedUI(mult);
      sendSpeed(mult);
    });
  });

  socket.on('speed_update', (data) => {
    if (data && data.ok) {
      updateSpeedUI(data.multiplier, data.linear_speed, data.angular_speed);
      appendLog('vel', `Velocidade: ${data.multiplier.toFixed(1)}x (L=${Math.round(data.linear_speed)} A=${Math.round(data.angular_speed)})`);
    }
  });

  // Expõe socket e helpers para o módulo gamepad
  window._robotSocket = socket;
  window._robotAppendLog = appendLog;
  window._robotDeliveryEl = deliveryEl;
  window._robotGetMode = () => controlMode;
  window._robotSendSpeed = sendSpeed;
  window._robotUpdateSpeedUI = updateSpeedUI;
  window._robotGetMultiplier = () => currentMultiplier;

  // Sequência incremental para correlacionar ACKs (confirmação)
  let seq = 0;
  const pending = new Map(); // seq -> {code, type, timer}

  // --- Seletor de modo ---
  document.querySelectorAll('.mode-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const mode = btn.getAttribute('data-mode');
      if (mode === controlMode) return;
      controlMode = mode;

      document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');

      if (mode === 'web') {
        modeDisplay.textContent = 'Teclado / Web';
        pressedRow.style.display = '';
        gamepadStatusRow.style.display = 'none';
        webControls.style.display = '';
        gamepadControls.style.display = 'none';
      } else {
        modeDisplay.textContent = 'Controle PS4';
        pressedRow.style.display = 'none';
        gamepadStatusRow.style.display = '';
        webControls.style.display = 'none';
        gamepadControls.style.display = '';
        // Limpa teclas pressionadas ao trocar para gamepad
        pressed.clear();
        renderPressed();
      }
      appendLog('modo', `Alterado para: ${modeDisplay.textContent}`);
    });
  });

  // --- Modo monitor (Fase 2): web sem controle de movimento ---
  // Esconde seletor de modo, barra de velocidade e os dois painéis de controle,
  // e mostra um aviso. NÃO toca no mapa/click-to-go/waypoints/infos.
  const modeSelectorEl = document.querySelector('.mode-selector');
  const speedControlEl = document.querySelector('.speed-control');
  const monitorNoticeEl = document.getElementById('monitor-notice');
  // Card que embrulha pressed-row/gamepad-status-row: sem teleop as duas
  // linhas somem e a casca ficava flutuando vazia — esconde ela junto.
  const statusCardEl = document.querySelector('.status');

  function applyTeleopVisibility(enabled) {
    if (statusCardEl) statusCardEl.style.display = enabled ? '' : 'none';
    if (enabled) {
      if (modeSelectorEl) modeSelectorEl.style.display = '';
      if (speedControlEl) speedControlEl.style.display = '';
      if (monitorNoticeEl) monitorNoticeEl.style.display = 'none';
      // Restaura os painéis conforme o modo ativo
      if (controlMode === 'web') {
        if (webControls) webControls.style.display = '';
        if (pressedRow) pressedRow.style.display = '';
      } else {
        if (gamepadControls) gamepadControls.style.display = '';
        if (gamepadStatusRow) gamepadStatusRow.style.display = '';
      }
    } else {
      if (modeSelectorEl) modeSelectorEl.style.display = 'none';
      if (speedControlEl) speedControlEl.style.display = 'none';
      if (webControls) webControls.style.display = 'none';
      if (gamepadControls) gamepadControls.style.display = 'none';
      if (pressedRow) pressedRow.style.display = 'none';
      if (gamepadStatusRow) gamepadStatusRow.style.display = 'none';
      if (monitorNoticeEl) monitorNoticeEl.style.display = '';
      // Solta qualquer tecla presa antes de cortar a captura
      pressed.clear();
      renderPressed();
    }
  }

  socket.on('mode_info', (data) => {
    if (!data) return;
    // Default true se o servidor for antigo e não mandar a chave
    webTeleop = data.web_teleop !== false;
    applyTeleopVisibility(webTeleop);
  });
  // Exposto pro gamepad.js parar de fazer poll/emit quando o web é só monitor
  window._robotIsTeleopEnabled = () => webTeleop;

  // ---- 🎥 Câmera POV — card visível em qualquer modo ----
  // src do <img> só é setado com o view ligado: desligado, o navegador fecha
  // a conexão e o servidor para de empurrar JPEG pelo WiFi.
  const povCard = document.getElementById('pov-card');
  const povWrap = document.getElementById('pov-wrap');
  const povImg  = document.getElementById('pov-img');
  const povRec  = document.getElementById('pov-rec');
  const btnPov  = document.getElementById('btn-pov');
  let povOn = false;
  function setPov(on) {
    povOn = on;
    if (btnPov) {
      btnPov.textContent = on ? 'Desligar' : 'Ligar';
      btnPov.classList.toggle('active', on);
    }
    if (povWrap) povWrap.style.display = on ? '' : 'none';
    if (povImg) povImg.src = on ? '/camera/stream?t=' + Date.now() : '';
  }
  if (btnPov) btnPov.addEventListener('click', () => setPov(!povOn));
  // ⏺/⏹ manual + toggle da gravação automática (ex.: teste de bateria com
  // live view ligado mas sem encher o SD da Pi). Estado vem do camera_status.
  const btnPovRec  = document.getElementById('btn-pov-rec');
  const btnPovAuto = document.getElementById('btn-pov-auto');
  let povRecording = false;
  let povAuto = true;
  if (btnPovRec) btnPovRec.addEventListener('click', () => {
    socket.emit('camera_record', { action: povRecording ? 'stop' : 'start' });
  });
  if (btnPovAuto) btnPovAuto.addEventListener('click', () => {
    socket.emit('camera_auto', { on: !povAuto });
  });

  // 🚶 Seguir pessoa: toggle START/STOP. O rótulo/estado vêm do follow_state
  // (person_follower -> map_bridge -> aqui). Ligar pausa a rota; ao parar (ou
  // perder a pessoa por tempo) a rota retoma sozinha no backend.
  const btnFollow = document.getElementById('btn-follow');
  let followState = 'idle';
  function followActive() {
    return followState === 'armed' || followState === 'following'
        || followState === 'lost';
  }
  function renderFollow() {
    if (!btnFollow) return;
    const active = followActive();
    if (followState === 'armed') btnFollow.textContent = '🚶 Aguardando…';
    else if (active) btnFollow.textContent = '🚶 Parar de seguir';
    else btnFollow.textContent = '🚶 Seguir';
    btnFollow.classList.toggle('active', active);
    btnFollow.title = followState === 'armed'
      ? 'seguir ARMADO — espera alguém se mexer na frente'
      : (active ? ('seguindo (' + followState + ')')
                : 'robô segue a pessoa à frente (arma e espera movimento)');
  }
  if (btnFollow) btnFollow.addEventListener('click', () => {
    socket.emit('follow', { action: followActive() ? 'stop' : 'start' });
  });
  socket.on('follow_state', (d) => {
    followState = (d && d.state) || 'idle';
    renderFollow();
  });
  renderFollow();

  socket.on('camera_status', (st) => {
    if (!st || !povCard) return;
    povCard.style.display = '';
    if (btnPov) {
      btnPov.disabled = !st.available;
      btnPov.title = st.available ? '' : 'câmera não detectada';
    }
    povRecording = !!st.recording;
    if ('auto_record' in st) povAuto = !!st.auto_record;
    if (btnPovRec) {
      btnPovRec.disabled = !st.available && !povRecording;
      btnPovRec.textContent = povRecording ? '⏹ Parar' : '⏺ Gravar';
      btnPovRec.classList.toggle('active', povRecording);
    }
    if (btnPovAuto) {
      btnPovAuto.textContent = povAuto ? 'Auto: ON' : 'Auto: OFF';
      btnPovAuto.classList.toggle('active', povAuto);
    }
    if (povRec) povRec.style.display = st.recording ? '' : 'none';
    // câmera caiu com o view aberto → desliga pra não ficar imagem morta
    if (!st.available && povOn) setPov(false);
  });

  socket.on('connect', () => {
    connEl.textContent = 'conectado';
    connEl.className = 'ok';
    const transport = socket.io.engine.transport && socket.io.engine.transport.name;
    appendLog('socket', `conectado sid=${socket.id} transport=${transport || 'n/a'}`);
    try {
      socket.emit('client_hello', {
        ts: Date.now(),
        href: location.href,
        ua: navigator.userAgent,
      });
    } catch (e) {
      console.error('hello emit failed', e);
    }
  });

  socket.on('disconnect', () => {
    connEl.textContent = 'desconectado';
    connEl.className = 'err';
    pressed.clear();
    renderPressed();
    appendLog('socket', 'desconectado');
    if (deliveryEl) deliveryEl.textContent = 'desconectado';
  });

  socket.on('connect_error', (err) => {
    appendLog('socket', `connect_error: ${err?.message || err}`);
    console.error('connect_error', err);
  });

  socket.on('error', (err) => {
    appendLog('socket', `error: ${err?.message || err}`);
    console.error('socket error', err);
  });

  socket.on('reconnect_error', (err) => {
    appendLog('socket', `reconnect_error: ${err?.message || err}`);
    console.error('reconnect_error', err);
  });

  socket.on('server_status', (data) => {
    connEl.textContent = 'conectado';
    connEl.className = 'ok';
    appendLog('socket', 'conectado');
  });

  // Telemetria de energia (power_monitor): tensão das 2 placas hoverboard.
  // Verde = ok; amarelo = sag/stall em curso; vermelho = placa caiu (BMS);
  // cinza = sem dados (MEGA/hoverboards desligados).
  socket.on('power_update', (data) => {
    const chip = document.getElementById('power-chip');
    if (!chip || !data) return;
    if (!data.fresh) {
      chip.textContent = '—';
      chip.className = 'power-chip power-na';
      return;
    }
    const fmt = (v) => (v == null ? '?' : `${v.toFixed(1)}V`);
    let txt = `F ${fmt(data.v_front)} · R ${fmt(data.v_rear)}`;
    let cls = 'power-chip power-ok';
    if (!data.front_ok || !data.rear_ok) {
      cls = 'power-chip power-trip';
      txt += !data.front_ok && !data.rear_ok ? ' ⚡ AMBAS' :
             (!data.front_ok ? ' ⚡ FRENTE' : ' ⚡ TRÁS');
    } else if (data.stall || (data.event && data.event.startsWith('sag'))) {
      cls = 'power-chip power-warn';
      if (data.stall) txt += ' STALL';
    }
    chip.textContent = txt;
    chip.className = cls;
    if (data.event) appendLog('power', `evento: ${data.event}`);
  });

  socket.on('server_hello', (data) => {
    appendLog('server', `hello sid=${data?.sid || '-'} ok`);
  });

  socket.on('ack', (res) => {
    const { ok, seq: rseq, type, code, action, command, error } = res || {};
    if (pending.has(rseq)) {
      const item = pending.get(rseq);
      clearTimeout(item.timer);
      pending.delete(rseq);
    }
    const human = humanize(type, code, action, command);
    if (ok) {
      if (deliveryEl) deliveryEl.textContent = `Recebido: ${human}`;
      appendLog('ok', `Recebido: ${human}`);
    } else {
      if (deliveryEl) deliveryEl.textContent = `Não recebido: ${human} (${error || 'erro'})`;
      appendLog('fail', `Não recebido: ${human} (${error || 'erro'})`);
    }
  });

  function send(type, code, repeat) {
    const id = ++seq;
    const payload = { type, code, repeat: !!repeat, seq: id };
    const timer = setTimeout(() => {
      if (pending.has(id)) {
        pending.delete(id);
        const human = humanize(type, code);
        if (deliveryEl) deliveryEl.textContent = `Não recebido: ${human} (timeout)`;
        appendLog('timeout', `Não recebido: ${human} (timeout)`);
      }
    }, 2000);
    pending.set(id, { code, type, timer });
    socket.emit('key_event', payload);
  }

  function renderPressed() {
    if (pressed.size === 0) {
      pressedEl.textContent = '(nenhuma)';
    } else {
      pressedEl.textContent = Array.from(pressed).join(', ');
    }
  }

  function appendLog(tag, message) {
    if (!logEl) return;
    const ts = new Date().toLocaleTimeString();
    const li = document.createElement('li');
    li.textContent = `[${ts}] ${tag}: ${message}`;
    logEl.prepend(li);
    while (logEl.children.length > 200) {
      logEl.removeChild(logEl.lastChild);
    }
  }

  // Listeners de teclado (só ativos no modo web).
  // Ignora eventos vindos de campos de entrada (prompt de nome de rota, etc.):
  // sem isso, digitar "rota1" no input acaba comandando o robô.
  const isTypingInField = (e) =>
    e.target && e.target.matches && e.target.matches('input,select,textarea,[contenteditable="true"]');

  window.addEventListener('keydown', (e) => {
    if (!webTeleop) return;
    if (controlMode !== 'web') return;
    if (isTypingInField(e)) return;
    const code = e.code || e.key;
    if (!pressed.has(code)) {
      pressed.add(code);
      send('down', code, e.repeat);
      renderPressed();
    }
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(code)) {
      e.preventDefault();
    }
  }, { passive: false });

  window.addEventListener('keyup', (e) => {
    if (!webTeleop) return;
    if (controlMode !== 'web') return;
    if (isTypingInField(e)) return;
    const code = e.code || e.key;
    if (pressed.has(code)) {
      pressed.delete(code);
      send('up', code, e.repeat);
      renderPressed();
    }
  });

  // Botões de toque/clique (suporte mobile) + diagonais via data-combo
  function setupPad(button) {
    const comboAttr = button.getAttribute('data-combo');
    const codes = comboAttr
      ? comboAttr.split(',').map((s) => s.trim()).filter(Boolean)
      : [button.getAttribute('data-code')].filter(Boolean);

    const down = () => {
      if (!webTeleop) return;
      if (controlMode !== 'web') return;
      let changed = false;
      for (const code of codes) {
        if (!pressed.has(code)) {
          pressed.add(code);
          send('down', code, false);
          changed = true;
        }
      }
      if (changed) renderPressed();
    };
    const up = () => {
      if (controlMode !== 'web') return;
      let changed = false;
      for (const code of codes) {
        if (pressed.has(code)) {
          pressed.delete(code);
          send('up', code, false);
          changed = true;
        }
      }
      if (changed) renderPressed();
    };
    button.addEventListener('mousedown', down);
    button.addEventListener('mouseup', up);
    button.addEventListener('mouseleave', up);
    button.addEventListener('touchstart', (e) => { e.preventDefault(); down(); }, { passive: false });
    button.addEventListener('touchend', (e) => { e.preventDefault(); up(); }, { passive: false });
    button.addEventListener('touchcancel', (e) => { e.preventDefault(); up(); }, { passive: false });
  }

  document.querySelectorAll('.pad').forEach(setupPad);

  // --- E-STOP (topbar) ---
  // Sempre cancela a navegação por waypoints. Com teleop web habilitado,
  // também solta qualquer tecla presa e manda Space (stop no controller).
  // Com teleop off (Fase 2) o key_event seria rejeitado pelo servidor,
  // então nem envia — o stop manual é o PS4/WASD no robô.
  const estopBtn = document.getElementById('btn-estop');
  if (estopBtn) {
    estopBtn.addEventListener('click', () => {
      socket.emit('stop_waypoints');
      if (webTeleop) {
        for (const code of Array.from(pressed)) {
          pressed.delete(code);
          send('up', code, false);
        }
        renderPressed();
        send('down', 'Space', false);
        send('up', 'Space', false);
      }
      appendLog('estop', 'STOP acionado (navegação cancelada' + (webTeleop ? ' + stop teleop)' : ')'));
      estopBtn.classList.remove('fired');
      void estopBtn.offsetWidth; // reinicia a animação de flash
      estopBtn.classList.add('fired');
    });
  }

  function humanize(type, code, action, command) {
    const cmdPt = { forward: 'frente', backward: 'ré', left: 'esquerda', right: 'direita', stop: 'parar' };
    const actPt = { start: 'Iniciar', stop: 'Parar' };
    if (action && command && cmdPt[command] && actPt[action]) {
      return `${actPt[action]} ${cmdPt[command]} (${code})`;
    }
    const typPt = { down: 'pressionar', up: 'soltar' };
    return `${typPt[type] || type} ${code}`;
  }

})();
