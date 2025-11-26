(() => {
  // Script do cliente: captura teclado/botões, envia eventos via Socket.IO
  // e exibe status de conexão e confirmação de entrega (ACK) do servidor.
  const connEl = document.getElementById('conn');
  const pressedEl = document.getElementById('pressed');
  const logEl = document.getElementById('log');
  const deliveryEl = document.getElementById('delivery');

  // Rastreia teclas pressionadas para evitar flood por autorepeat
  const pressed = new Set();

  // Prefere polling e faz upgrade para websocket (melhor compatibilidade)
  const socket = io({ transports: ['polling', 'websocket'] });

  // Sequência incremental para correlacionar ACKs (confirmação)
  let seq = 0;
  const pending = new Map(); // seq -> {code, type, timer}

  socket.on('connect', () => {
    connEl.textContent = 'conectado';
    const transport = socket.io.engine.transport && socket.io.engine.transport.name;
    appendLog('socket', `conectado sid=${socket.id} transport=${transport || 'n/a'}`);
    // Envia um "hello" de diagnóstico para o servidor
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
    appendLog('socket', 'conectado');
  });

  socket.on('server_hello', (data) => {
    appendLog('server', `hello sid=${data?.sid || '-'} ok`);
  });

  socket.on('ack', (res) => {
    // Recebe confirmação do servidor para um evento enviado
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

  // no-op: não exibimos mais o server_echo na UI

  function send(type, code, repeat) {
    // Envia um evento de tecla para o servidor com timeout de confirmação
    const id = ++seq;
    const payload = { type, code, repeat: !!repeat, seq: id };
    // Marca como pendente e define timeout para considerar "não recebido"
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
    // Atualiza a lista de teclas atualmente pressionadas
    if (pressed.size === 0) {
      pressedEl.textContent = '(nenhuma)';
    } else {
      pressedEl.textContent = Array.from(pressed).join(', ');
    }
  }

  function appendLog(tag, message) {
    // Adiciona uma linha no log visual (limite de 50 entradas)
    if (!logEl) return;
    const ts = new Date().toLocaleTimeString();
    const li = document.createElement('li');
    li.textContent = `[${ts}] ${tag}: ${message}`;
    logEl.prepend(li);
    while (logEl.children.length > 50) {
      logEl.removeChild(logEl.lastChild);
    }
  }

  // Listeners de teclado
  window.addEventListener('keydown', (e) => {
    const code = e.code || e.key;
    if (!pressed.has(code)) {
      pressed.add(code);
      send('down', code, e.repeat);
      renderPressed();
    }
    // Evita rolagem da página com setas/espaço
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'Space'].includes(code)) {
      e.preventDefault();
    }
  }, { passive: false });

  window.addEventListener('keyup', (e) => {
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

  function humanize(type, code, action, command) {
    // Converte dados técnicos em mensagem humana (pt-BR)
    const cmdPt = { forward: 'frente', backward: 'ré', left: 'esquerda', right: 'direita', stop: 'parar' };
    const actPt = { start: 'Iniciar', stop: 'Parar' };
    if (action && command && cmdPt[command] && actPt[action]) {
      return `${actPt[action]} ${cmdPt[command]} (${code})`;
    }
    const typPt = { down: 'pressionar', up: 'soltar' };
    return `${typPt[type] || type} ${code}`;
  }
})();
