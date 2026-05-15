(() => {
  // Painel TREKKING — render do canvas (sem mapa de fundo, frame `odom`),
  // toolbar de comandos (record/save/play/stop/reset) e gerenciamento de rotas.
  // Os eventos chegam pelo socket.io de client.js (window.robotSocket).

  const panel       = document.getElementById('trek-panel');
  const canvas      = document.getElementById('trek-canvas');
  if (!panel || !canvas) return;
  const ctx         = canvas.getContext('2d');

  const modePill    = document.getElementById('trek-mode-pill');
  const statusEl    = document.getElementById('trek-status');
  const poseEl      = document.getElementById('trek-pose');
  const wpCountEl   = document.getElementById('trek-wp-count');
  const coneCountEl = document.getElementById('trek-cone-count');
  const snapEl      = document.getElementById('trek-snap-info');
  const modeBadge   = document.getElementById('robot-mode-badge');

  const btnReset    = document.getElementById('trek-btn-reset');
  const btnRecord   = document.getElementById('trek-btn-record');
  const btnSavePt   = document.getElementById('trek-btn-save-pt');
  const btnPlay     = document.getElementById('trek-btn-play');
  const btnStop     = document.getElementById('trek-btn-stop');
  const btnSave     = document.getElementById('trek-btn-save');
  const btnLoad     = document.getElementById('trek-btn-load');
  const routeName   = document.getElementById('trek-route-name');
  const routeSelect = document.getElementById('trek-route-select');

  let state = null;                  // último /trekking/state recebido
  const trail = [];                  // últimas N poses do robô (decay)
  const MAX_TRAIL = 400;
  let trailLastX = null, trailLastY = null;

  function waitForSocket(cb) {
    if (window.robotSocket) return cb(window.robotSocket);
    setTimeout(() => waitForSocket(cb), 50);
  }

  // ----------------- transformação mundo → canvas -----------------
  // Auto-fit: enquadra robô + waypoints + cones + 1 m de margem.
  function computeView() {
    const w = canvas.width, h = canvas.height;
    let pts = [];
    if (state && state.have_pose) pts.push([state.x, state.y]);
    if (state) {
      (state.waypoints || []).forEach(wp => {
        pts.push([wp.x, wp.y]);
        if (wp.has_cone) pts.push([wp.cone_x, wp.cone_y]);
      });
      (state.cones || []).forEach(c => pts.push([c[0], c[1]]));
    }
    if (pts.length === 0) pts = [[0,0]];

    let xmin=Infinity, xmax=-Infinity, ymin=Infinity, ymax=-Infinity;
    for (const [x,y] of pts) {
      if (x<xmin) xmin=x; if (x>xmax) xmax=x;
      if (y<ymin) ymin=y; if (y>ymax) ymax=y;
    }
    // mantém aspect 1:1 com margem
    let dx = xmax-xmin, dy = ymax-ymin;
    const span = Math.max(dx, dy, 2.0) + 2.0;
    const cx = (xmax+xmin)/2, cy = (ymax+ymin)/2;
    const scale = Math.min(w, h) / span;   // px por metro
    return {
      scale,
      tx: (x) => w/2 + (x - cx) * scale,
      ty: (y) => h/2 - (y - cy) * scale,
      cx, cy, span,
    };
  }

  function drawGrid(view) {
    const w = canvas.width, h = canvas.height;
    ctx.strokeStyle = '#2a1505';
    ctx.lineWidth = 1;
    const stepM = view.span > 8 ? 2 : 1;
    const left   = view.cx - view.span/2;
    const right  = view.cx + view.span/2;
    const bottom = view.cy - view.span/2;
    const top    = view.cy + view.span/2;
    for (let x = Math.ceil(left/stepM)*stepM; x <= right; x += stepM) {
      const px = view.tx(x);
      ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke();
    }
    for (let y = Math.ceil(bottom/stepM)*stepM; y <= top; y += stepM) {
      const py = view.ty(y);
      ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke();
    }
    // Origem (0,0) — marca azul
    ctx.fillStyle = '#1e3a8a';
    const ox = view.tx(0), oy = view.ty(0);
    ctx.beginPath(); ctx.arc(ox, oy, 5, 0, 2*Math.PI); ctx.fill();
  }

  function drawTrail(view) {
    if (trail.length < 2) return;
    ctx.strokeStyle = '#65a30d';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(view.tx(trail[0][0]), view.ty(trail[0][1]));
    for (let i=1; i<trail.length; i++) ctx.lineTo(view.tx(trail[i][0]), view.ty(trail[i][1]));
    ctx.stroke();
  }

  function drawCones(view) {
    if (!state) return;
    // Cones ao vivo (vermelho hollow)
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 1.5;
    (state.cones || []).forEach(c => {
      const px = view.tx(c[0]), py = view.ty(c[1]);
      ctx.beginPath(); ctx.arc(px, py, 5, 0, 2*Math.PI); ctx.stroke();
    });
    // Cones gravados (laranja filled)
    ctx.fillStyle = '#fdba74';
    (state.waypoints || []).forEach(wp => {
      if (!wp.has_cone) return;
      const px = view.tx(wp.cone_x), py = view.ty(wp.cone_y);
      ctx.beginPath(); ctx.arc(px, py, 4, 0, 2*Math.PI); ctx.fill();
    });
    // Cone trancado (snap atual) — anel amarelo
    if (state.locked_cone) {
      const [cx, cy] = state.locked_cone;
      ctx.strokeStyle = '#fbbf24';
      ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(view.tx(cx), view.ty(cy), 9, 0, 2*Math.PI); ctx.stroke();
    }
  }

  function drawWaypoints(view) {
    if (!state) return;
    const wps = state.waypoints || [];
    // Linha entre waypoints
    if (wps.length >= 2) {
      ctx.strokeStyle = '#7c2d12';
      ctx.lineWidth = 1;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      ctx.moveTo(view.tx(wps[0].x), view.ty(wps[0].y));
      for (let i=1; i<wps.length; i++) ctx.lineTo(view.tx(wps[i].x), view.ty(wps[i].y));
      ctx.stroke();
      ctx.setLineDash([]);
    }
    // Cada waypoint
    wps.forEach((wp, idx) => {
      const px = view.tx(wp.x), py = view.ty(wp.y);
      const active = state.mode === 'play' && idx === state.current_idx;
      ctx.fillStyle = active ? '#fb923c' : '#fed7aa';
      ctx.strokeStyle = '#fb923c';
      ctx.lineWidth = active ? 2 : 1;
      ctx.beginPath(); ctx.arc(px, py, 8, 0, 2*Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#1a0e07';
      ctx.font = 'bold 10px monospace';
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(String(idx), px, py);
      // Setinha do bearing pro cone
      if (wp.has_cone) {
        const cpx = view.tx(wp.cone_x), cpy = view.ty(wp.cone_y);
        ctx.strokeStyle = '#fdba74';
        ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(cpx, cpy); ctx.stroke();
      }
    });
  }

  function drawRobot(view) {
    if (!state || !state.have_pose) return;
    const px = view.tx(state.x), py = view.ty(state.y);
    const yaw = state.yaw;
    // corpo
    ctx.fillStyle = '#f97316';
    ctx.beginPath(); ctx.arc(px, py, 6, 0, 2*Math.PI); ctx.fill();
    // seta de heading (15 px na direção do yaw; canvas y invertido)
    const tx = px + 18 * Math.cos(yaw);
    const ty = py - 18 * Math.sin(yaw);
    ctx.strokeStyle = '#fb923c';
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(tx, ty); ctx.stroke();
  }

  function render() {
    const w = canvas.width, h = canvas.height;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, w, h);
    const view = computeView();
    drawGrid(view);
    drawTrail(view);
    drawCones(view);
    drawWaypoints(view);
    drawRobot(view);
  }

  function refreshLabels() {
    if (!state) return;
    modePill.textContent = state.mode.toUpperCase();
    modePill.className = 'trek-pill mode-' + state.mode;
    statusEl.textContent = state.msg || '';
    if (state.have_pose) {
      poseEl.textContent = `${state.x.toFixed(2)}, ${state.y.toFixed(2)} | ${(state.yaw*180/Math.PI).toFixed(0)}°`;
    } else {
      poseEl.textContent = 'aguardando...';
    }
    wpCountEl.textContent = String(state.total || 0);
    if (state.mode === 'play') {
      wpCountEl.textContent = `${state.current_idx}/${state.total}`;
    }
    coneCountEl.textContent = String((state.cones || []).length);
    snapEl.textContent = state.locked_cone ? '✓ cone trancado' : '';
    // habilita/desabilita botões pelo modo
    btnPlay.disabled    = state.mode === 'play' || (state.total || 0) === 0;
    btnStop.disabled    = state.mode === 'idle';
    btnRecord.classList.toggle('active', state.mode === 'record');
    btnSavePt.disabled  = state.mode === 'play';
  }

  // ----------------- inputs -----------------
  function cmd(c, extra) {
    window.robotSocket.emit('trekking_cmd', Object.assign({cmd: c}, extra || {}));
  }
  btnReset .addEventListener('click', () => cmd('reset'));
  btnRecord.addEventListener('click', () => cmd('record'));
  btnSavePt.addEventListener('click', () => cmd('save_point'));
  btnPlay  .addEventListener('click', () => cmd('play'));
  btnStop  .addEventListener('click', () => cmd('stop'));

  btnSave.addEventListener('click', () => {
    const name = (routeName.value || 'rota').trim();
    window.robotSocket.emit('trekking_save_route', {name});
  });
  btnLoad.addEventListener('click', () => {
    // Toggle: se já tem select aberto e item selecionado, dispara load.
    if (routeSelect.style.display !== 'none' && routeSelect.value) {
      window.robotSocket.emit('trekking_load_route', {name: routeSelect.value});
      routeSelect.style.display = 'none';
    } else {
      window.robotSocket.emit('trekking_list_routes');
    }
  });

  // ----------------- socket -----------------
  waitForSocket((socket) => {
    socket.on('mode_info', (data) => {
      const mode = (data && data.mode) || 'teleop';
      if (modeBadge) {
        modeBadge.textContent = mode.toUpperCase();
        modeBadge.className = 'mode-badge mode-' + mode;
      }
      panel.style.display = (mode === 'trekking') ? '' : 'none';
    });

    socket.on('trekking_state', (data) => {
      state = data;
      // Limpa trail quando volta pra IDLE sem waypoints — sinaliza "novo
      // ensaio". Sem isso o trail acumula entre sessões e suja o canvas.
      if (state.mode === 'idle' && (state.total || 0) === 0) {
        trail.length = 0;
        trailLastX = null;
        trailLastY = null;
      } else if (state.have_pose) {
        if (trailLastX === null || Math.hypot(state.x - trailLastX, state.y - trailLastY) > 0.05) {
          trail.push([state.x, state.y]);
          trailLastX = state.x; trailLastY = state.y;
          if (trail.length > MAX_TRAIL) trail.shift();
        }
      }
      refreshLabels();
      render();
    });

    socket.on('trekking_ack', (data) => {
      if (data && !data.ok && data.error) statusEl.textContent = 'ERRO: ' + data.error;
    });
    socket.on('trekking_save_ack', (data) => {
      statusEl.textContent = data.ok ? `rota salva: ${data.name}` : ('ERRO: ' + (data.error||''));
    });
    socket.on('trekking_load_ack', (data) => {
      statusEl.textContent = data.ok ? `rota carregada: ${data.name} (${data.count} wp)` : ('ERRO: ' + (data.error||''));
    });
    socket.on('trekking_routes', (data) => {
      const routes = (data && data.routes) || [];
      routeSelect.innerHTML = '';
      if (routes.length === 0) {
        statusEl.textContent = 'nenhuma rota salva';
        return;
      }
      routes.forEach(r => {
        const opt = document.createElement('option');
        opt.value = r; opt.textContent = r;
        routeSelect.appendChild(opt);
      });
      routeSelect.style.display = '';
      statusEl.textContent = 'escolha uma rota e clique em Carregar de novo';
    });
  });

  // Render inicial em branco (panel oculto até receber mode_info='trekking')
  render();
})();
