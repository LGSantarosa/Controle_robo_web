(() => {
  // Cliente do painel de mapa — renderiza OccupancyGrid (PNG recebido via
  // socket.io), pose do robô (TF map→base_link) e trajetória planejada do Nav2.
  // Click no canvas envia /goal_pose (só no modo NAV2).
  //
  // O `socket` aqui é o mesmo criado por client.js — acessamos via window.
  const panel       = document.getElementById('map-panel');
  const canvas      = document.getElementById('map-canvas');
  const ctx         = canvas.getContext('2d');
  const statusEl    = document.getElementById('map-status');
  const sizeEl      = document.getElementById('map-info-size');
  const resEl       = document.getElementById('map-info-res');
  const poseEl      = document.getElementById('map-robot-pose');
  const clickHint   = document.getElementById('map-click-hint');
  const btnSave     = document.getElementById('btn-save-map');
  const modeBadge   = document.getElementById('robot-mode-badge');

  // Estado local — atualizado por eventos socket.io
  let currentMode = 'teleop';
  let mapInfo = null;      // { width, height, resolution, origin_x, origin_y, ... }
  let mapImage = null;     // Image carregada do PNG base64
  let robotPose = null;    // { x, y, yaw }
  let plan = [];           // [{ x, y }]
  let lastGoal = null;     // { x, y }

  // Waypoints
  let waypoints  = [];     // [{x, y, yaw}]
  let wpMode     = false;  // modo de adição de waypoints ativo
  let wpActive   = false;  // navegação rodando
  let wpActiveIdx = 0;     // índice do waypoint atual
  let wpDrag     = null;   // {worldX, worldY, canvasX, canvasY} durante drag de orientação
  let wpMouseDown = null;  // posição do mousedown para detectar drag vs click
  let setPoseMode = false; // armado: próximo click-arrasta define a pose real (relocaliza)
  let setPoseDrag = null;  // {canvasX, canvasY, curX, curY, world} durante o drag

  // Portas marcadas (travessia door_crossing)
  let doorMode = false;          // modo "marcar porta"
  let doorDrag = null;           // arraste atual: {ax,ay,cx,cy,curX,curY,curWorld,shift}
  let doors = [];                // [{id, a:[x,y], b:[x,y]}]
  let doorZone = null;           // {state, door_id} vindo do robô

  // Trava anti-torto: gruda a linha no múltiplo de 45° mais próximo (em coords do
  // mapa) quando o arraste está a menos de ~12° dele. Porta torta não existe — o
  // erro é de mão na hora de clicar. Shift segura -> ângulo livre (porta diagonal).
  function snapDoorEnd(ax, ay, bx, by, freeAngle) {
    const dx = bx - ax, dy = by - ay;
    const len = Math.hypot(dx, dy);
    if (freeAngle || len < 1e-6) return { x: bx, y: by };
    let ang = Math.atan2(dy, dx);
    const step = Math.PI / 4;                 // 45°
    const snapped = Math.round(ang / step) * step;
    let diff = ang - snapped;
    diff = Math.atan2(Math.sin(diff), Math.cos(diff));
    if (Math.abs(diff) < (12 * Math.PI / 180)) ang = snapped;
    return { x: ax + len * Math.cos(ang), y: ay + len * Math.sin(ang) };
  }
  const btnDoor = document.getElementById('map-btn-door');
  const doorChip = document.getElementById('map-door-chip');

  // Espera o socket de client.js existir. client.js cria `window.robotSocket`.
  function waitForSocket(cb) {
    if (window.robotSocket) return cb(window.robotSocket);
    setTimeout(() => waitForSocket(cb), 50);
  }

  // Elementos da toolbar de waypoints
  const wpToolbar   = document.getElementById('wp-toolbar');
  const btnWpMode   = document.getElementById('btn-wp-mode');
  const btnWpClear  = document.getElementById('btn-wp-clear');
  const btnWpStart  = document.getElementById('btn-wp-start');
  const btnWpStop   = document.getElementById('btn-wp-stop');
  const btnWpSave   = document.getElementById('btn-wp-save');
  const btnWpLoad   = document.getElementById('btn-wp-load');
  const btnSetPose  = document.getElementById('btn-set-pose');
  const wpRouteSelect = document.getElementById('wp-route-select');
  const wpLoopChk   = document.getElementById('wp-loop');
  const wpStatusEl  = document.getElementById('wp-status');

  function setWpMode(on) {
    wpMode = on;
    if (btnWpMode) {
      btnWpMode.textContent = on ? '✕ Cancelar' : '+ Waypoint';
      btnWpMode.classList.toggle('active', on);
    }
    canvas.style.cursor = on ? 'crosshair' : 'default';
    if (clickHint) clickHint.textContent = on
      ? 'clique = waypoint | clique+arraste = define direção'
      : (currentMode === 'nav2' ? '(clique no mapa para enviar o robô até o ponto)' : '');
  }

  function setSetPoseMode(on) {
    setPoseMode = on;
    if (on) setWpMode(false);   // exclusivos
    if (btnSetPose) btnSetPose.classList.toggle('active', on);
    canvas.style.cursor = on ? 'crosshair' : 'default';
    if (clickHint) clickHint.textContent = on
      ? 'Definir pose: clique onde o robô está e arraste pra direção'
      : (currentMode === 'nav2' ? '(clique no mapa para enviar o robô até o ponto)'
         : (currentMode === 'slam' ? '(mapeando em tempo real)' : ''));
  }

  function updateWpButtons() {
    if (!btnWpStart || !btnWpStop) return;
    btnWpStart.disabled = waypoints.length === 0 || wpActive;
    btnWpStop.disabled  = !wpActive;
    if (btnWpClear) btnWpClear.disabled = wpActive;
    if (btnWpMode)  btnWpMode.disabled  = wpActive;
  }

  waitForSocket((socket) => {
    socket.on('mode_info', (data) => {
      currentMode = (data && data.mode) || 'teleop';
      modeBadge.textContent = currentMode.toUpperCase();
      modeBadge.className = 'mode-badge mode-' + currentMode;

      if (currentMode === 'slam' || currentMode === 'nav2') {
        panel.style.display = '';
        btnSave.disabled = false;
        clickHint.textContent = currentMode === 'nav2'
          ? '(clique no mapa para enviar o robô até o ponto)'
          : '(mapeando em tempo real)';
      } else {
        panel.style.display = 'none';
      }
      if (wpToolbar) wpToolbar.style.display = currentMode === 'nav2' ? '' : 'none';
      if (btnSetPose) btnSetPose.style.display =
        (currentMode === 'slam' || currentMode === 'nav2') ? '' : 'none';
    });

    socket.on('map_update', (data) => {
      if (!data || !data.info || !data.png_b64) return;
      // Cria uma Image nova a cada update — reusar uma Image global (set
      // `.src` na mesma instância) é mais leve para o GC mas alguns
      // navegadores (Chromium/Safari) não disparam `onload` de forma
      // confiável em reatribuições rápidas de data URL, e o canvas fica
      // congelado no primeiro frame após o serviço subir. /map vem a 1 Hz,
      // então o custo de alocação é desprezível.
      mapInfo = data.info;
      const img = new Image();
      img.onload = () => {
        mapImage = img;
        sizeEl.textContent = `${mapInfo.width} × ${mapInfo.height} px`;
        resEl.textContent  = `${mapInfo.resolution.toFixed(3)} m/px`;
        statusEl.textContent = 'recebido';
        statusEl.className = 'map-status ok';
        render();
      };
      img.src = 'data:image/png;base64,' + data.png_b64;
    });

    socket.on('robot_pose', (data) => {
      robotPose = data;
      poseEl.textContent = `robô: x=${data.x.toFixed(2)} y=${data.y.toFixed(2)} yaw=${(data.yaw * 180 / Math.PI).toFixed(0)}°`;
      render();
    });

    socket.on('plan_update', (data) => {
      plan = (data && data.points) || [];
      render();
    });

    socket.on('nav_goal_ack', (data) => {
      if (!data.ok) {
        statusEl.textContent = 'erro: ' + (data.error || '?');
        statusEl.className = 'map-status err';
      } else {
        statusEl.textContent = `indo para (${data.x.toFixed(2)}, ${data.y.toFixed(2)})`;
        statusEl.className = 'map-status ok';
      }
    });

    socket.on('waypoint_status', (data) => {
      if (!data) return;
      wpActive    = !!data.active;
      wpActiveIdx = data.index || 0;
      if (wpStatusEl) {
        if (data.done) {
          wpStatusEl.textContent = 'concluído ✓';
        } else if (data.timeout) {
          wpStatusEl.textContent = `⚠ timeout — indo para ${data.index + 1}/${data.total}`;
        } else if (data.active) {
          wpStatusEl.textContent = `waypoint ${data.index + 1}/${data.total}`;
        } else {
          wpStatusEl.textContent = '';
        }
      }
      updateWpButtons();
      render();
    });

    socket.on('waypoints_ack', (data) => {
      if (!data.ok) {
        if (wpStatusEl) wpStatusEl.textContent = 'erro: ' + (data.error || '?');
        wpActive = false;
        updateWpButtons();
      }
    });

    // --- Botões de waypoints ---
    if (btnWpMode) btnWpMode.addEventListener('click', () => setWpMode(!wpMode));
    if (btnSetPose) btnSetPose.addEventListener('click', () => setSetPoseMode(!setPoseMode));
    socket.on('set_pose_ack', (data) => {
      if (statusEl) statusEl.textContent = data.ok
        ? `pose aplicada: (${data.x.toFixed(2)}, ${data.y.toFixed(2)})`
        : `falha ao definir pose: ${data.error}`;
    });

    if (btnWpClear) btnWpClear.addEventListener('click', () => {
      waypoints = [];
      lastGoal = null;
      setWpMode(false);
      updateWpButtons();
      render();
    });

    if (btnWpStart) btnWpStart.addEventListener('click', () => {
      if (waypoints.length === 0) return;
      setWpMode(false);
      socket.emit('start_waypoints', {
        waypoints,
        loop: wpLoopChk ? wpLoopChk.checked : false,
      });
    });

    if (btnWpStop) btnWpStop.addEventListener('click', () => {
      socket.emit('stop_waypoints');
    });

    // --- Salvar rota ---
    if (btnWpSave) btnWpSave.addEventListener('click', () => {
      if (waypoints.length === 0) { alert('Adicione waypoints antes de salvar.'); return; }
      const name = prompt('Nome da rota:', 'rota1');
      if (!name) return;
      socket.emit('save_route', { name, waypoints });
    });

    socket.on('save_route_ack', (data) => {
      if (data.ok) {
        if (wpStatusEl) wpStatusEl.textContent = `salvo: ${data.name}`;
      } else {
        alert('Erro ao salvar: ' + (data.error || '?'));
      }
    });

    // --- Carregar rota ---
    if (btnWpLoad) btnWpLoad.addEventListener('click', () => {
      socket.emit('list_routes');
    });

    socket.on('list_routes_ack', (data) => {
      if (!data.ok || !data.routes.length) {
        alert('Nenhuma rota salva encontrada.');
        return;
      }
      if (!wpRouteSelect) return;
      wpRouteSelect.innerHTML = '<option value="">— selecionar —</option>' +
        data.routes.map(r => `<option value="${r}">${r}</option>`).join('');
      wpRouteSelect.style.display = '';
      wpRouteSelect.focus();
    });

    if (wpRouteSelect) wpRouteSelect.addEventListener('change', () => {
      const name = wpRouteSelect.value;
      wpRouteSelect.style.display = 'none';
      if (!name) return;
      socket.emit('load_route', { name });
    });

    socket.on('load_route_ack', (data) => {
      if (!data.ok) {
        alert('Erro ao carregar: ' + (data.error || '?'));
        return;
      }
      waypoints = data.waypoints || [];
      if (wpLoopChk) wpLoopChk.checked = false;
      setWpMode(false);
      updateWpButtons();
      if (wpStatusEl) wpStatusEl.textContent = `carregado: ${data.name} (${waypoints.length} pts)`;
      render();
    });

    // --- Restaurar waypoints após F5 ---
    socket.on('waypoints_restored', (data) => {
      if (!data || !data.waypoints || data.waypoints.length === 0) return;
      waypoints    = data.waypoints;
      wpActive     = !!data.active;
      wpActiveIdx  = data.index || 0;
      if (wpLoopChk) wpLoopChk.checked = !!data.loop;
      updateWpButtons();
      if (wpStatusEl && data.active) {
        wpStatusEl.textContent = `waypoint ${data.index + 1}/${data.total}`;
      }
      render();
    });

    socket.on('save_map_ack', (data) => {
      if (data.ok) {
        alert(`Mapa salvo!\n${data.yaml}`);
        statusEl.textContent = `salvo: ${data.name}`;
        statusEl.className = 'map-status ok';
      } else {
        alert('Falha ao salvar mapa:\n' + (data.error || '?'));
      }
    });

    // --- Salvar mapa ---
    btnSave.addEventListener('click', () => {
      const name = prompt('Nome do mapa:', 'sala');
      if (!name) return;
      socket.emit('save_map', { name });
      statusEl.textContent = 'salvando...';
    });

    // --- Portas (travessia door_crossing) ---
    if (btnDoor) btnDoor.addEventListener('click', () => {
      doorMode = !doorMode;
      doorDrag = null;
      btnDoor.classList.toggle('active', doorMode);
      statusEl.textContent = doorMode
        ? 'modo porta: arraste de um batente até o outro (clique numa porta p/ apagar)'
        : '';
      render();
    });

    socket.on('doors_update', (payload) => {
      try { doors = JSON.parse(payload).doors || []; } catch (e) { doors = []; }
      render();
    });

    socket.on('door_ack', (r) => {
      if (!r.ok) statusEl.textContent = `porta: ${r.error}`;
    });

    socket.on('door_zone', (payload) => {
      try { doorZone = JSON.parse(payload); } catch (e) { doorZone = null; }
      const active = doorZone && doorZone.state !== 'idle';
      if (doorChip) {
        doorChip.style.display = active ? '' : 'none';
        if (active) {
          const nome = {staging: 'indo pro eixo', rotating: 'alinhando',
                        crossing: 'ATRAVESSANDO'}[doorZone.state] || doorZone.state;
          doorChip.textContent = `🚪 porta ${doorZone.door_id}: ${nome}`;
        }
      }
      render();
    });

    // --- Interação com o canvas (goal único + waypoints) ---
    const DRAG_THRESHOLD = 8; // pixels para considerar drag

    canvas.addEventListener('mousedown', (ev) => {
      if (!mapInfo || !mapImage) return;
      if (currentMode !== 'nav2' && !setPoseMode) return;
      const { cx, cy } = eventToCanvasPx(ev);
      const world = canvasToWorld(cx, cy);
      if (!world) return;
      if (setPoseMode) {
        setPoseDrag = { canvasX: cx, canvasY: cy, curX: cx, curY: cy, world };
        return;
      }
      wpMouseDown = { cx, cy, world };
      if (doorMode) {
        doorDrag = { ax: world.x, ay: world.y, cx, cy, curX: cx, curY: cy,
                     curWorld: world, shift: ev.shiftKey };
      }
      if (wpMode) {
        wpDrag = { worldX: world.x, worldY: world.y, canvasX: cx, canvasY: cy, curX: cx, curY: cy };
      }
    });

    canvas.addEventListener('mousemove', (ev) => {
      if (setPoseMode && setPoseDrag) {
        const p = eventToCanvasPx(ev);
        setPoseDrag.curX = p.cx;
        setPoseDrag.curY = p.cy;
        render();
        return;
      }
      if (doorMode && doorDrag) {
        const { cx, cy } = eventToCanvasPx(ev);
        const world = canvasToWorld(cx, cy);
        doorDrag.curX = cx; doorDrag.curY = cy;
        if (world) doorDrag.curWorld = world;
        doorDrag.shift = ev.shiftKey;
        render();
        return;
      }
      if (!wpDrag || !wpMode) return;
      const { cx, cy } = eventToCanvasPx(ev);
      wpDrag.curX = cx;
      wpDrag.curY = cy;
      render();
    });

    canvas.addEventListener('mouseup', (ev) => {
      if (!mapInfo || !mapImage) return;
      if (currentMode !== 'nav2' && !setPoseMode) return;
      const { cx, cy } = eventToCanvasPx(ev);

      if (setPoseMode && setPoseDrag) {
        const ddx = cx - setPoseDrag.canvasX;
        const ddy = cy - setPoseDrag.canvasY;
        const dragged = Math.sqrt(ddx * ddx + ddy * ddy) > DRAG_THRESHOLD;
        const yaw = dragged ? Math.atan2(-ddy, ddx) : 0.0;
        const w = setPoseDrag.world;
        socket.emit('set_pose', { x: w.x, y: w.y, yaw });
        if (statusEl) statusEl.textContent = `pose definida: (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`;
        setPoseDrag = null;
        setSetPoseMode(false);
        render();
        return;
      }

      // Modo porta: ARRASTA de um batente até o outro (linha reta com snap
      // anti-torto). Clique curtinho em cima de porta existente = apagar. Tem
      // precedência sobre waypoint/goal.
      if (doorMode && doorDrag) {
        const start = { x: doorDrag.ax, y: doorDrag.ay };
        const endRaw = canvasToWorld(cx, cy) || doorDrag.curWorld;
        const movedPx = Math.hypot(cx - doorDrag.cx, cy - doorDrag.cy);
        const NEAR = 0.35;
        const hit = doors.find(d => {
          const mx = (d.a[0] + d.b[0]) / 2, my = (d.a[1] + d.b[1]) / 2;
          return Math.hypot(start.x - mx, start.y - my) < NEAR;
        });
        if (movedPx <= DRAG_THRESHOLD && hit) {
          socket.emit('door_cmd', { del: hit.id });
          statusEl.textContent = `porta ${hit.id} apagada`;
        } else if (movedPx > DRAG_THRESHOLD) {
          const end = snapDoorEnd(start.x, start.y, endRaw.x, endRaw.y, ev.shiftKey);
          socket.emit('door_cmd', {
            add: { a: [start.x, start.y], b: [end.x, end.y] } });
          statusEl.textContent = 'porta marcada';
        } else {
          statusEl.textContent = 'arraste de um batente até o outro (Shift = ângulo livre)';
        }
        doorDrag = null; wpDrag = null; wpMouseDown = null;
        render();
        return;
      }

      if (wpMode && wpMouseDown) {
        const dx = cx - wpMouseDown.cx;
        const dy = cy - wpMouseDown.cy;
        const dragged = Math.sqrt(dx * dx + dy * dy) > DRAG_THRESHOLD;
        const world = wpMouseDown.world;
        // yaw: canvas y cresce pra baixo, ROS y cresce pra cima — inverte dy
        const yaw = dragged ? Math.atan2(-dy, dx) : 0.0;
        waypoints.push({ x: world.x, y: world.y, yaw });
        updateWpButtons();
        wpDrag = null;
        wpMouseDown = null;
        render();
        return;
      }

      // Click simples (sem modo waypoint) → goal único.
      // Click sem drag → yaw=0. Click+drag → yaw aponta na direção do drag
      // (mesma convenção dos waypoints; canvas y cresce pra baixo, ROS pra cima).
      if (!wpMode && wpMouseDown) {
        const ddx = cx - wpMouseDown.cx;
        const ddy = cy - wpMouseDown.cy;
        const dragged = Math.sqrt(ddx * ddx + ddy * ddy) > DRAG_THRESHOLD;
        const world = wpMouseDown.world;
        const yaw = dragged ? Math.atan2(-ddy, ddx) : 0.0;
        {
          lastGoal = world;
          socket.emit('nav_goal', { x: world.x, y: world.y, yaw });
          statusEl.textContent = `alvo: (${world.x.toFixed(2)}, ${world.y.toFixed(2)})`;
          render();
        }
      }

      wpDrag = null;
      wpMouseDown = null;
    });

    canvas.addEventListener('mouseleave', () => {
      wpDrag = null;
      wpMouseDown = null;
    });

    // Touch → encaminha pros mesmos handlers de mouse (preventDefault evita rolar a página)
    canvas.addEventListener('touchstart', (ev) => {
      if (ev.touches.length !== 1) return;
      ev.preventDefault();
      canvas.dispatchEvent(new MouseEvent('mousedown', {
        clientX: ev.touches[0].clientX, clientY: ev.touches[0].clientY }));
    }, { passive: false });
    canvas.addEventListener('touchmove', (ev) => {
      if (ev.touches.length !== 1) return;
      ev.preventDefault();
      canvas.dispatchEvent(new MouseEvent('mousemove', {
        clientX: ev.touches[0].clientX, clientY: ev.touches[0].clientY }));
    }, { passive: false });
    canvas.addEventListener('touchend', (ev) => {
      ev.preventDefault();
      const t = ev.changedTouches[0];
      if (t) canvas.dispatchEvent(new MouseEvent('mouseup', {
        clientX: t.clientX, clientY: t.clientY }));
    }, { passive: false });
  });

  // --- Helpers de transformação canvas ↔ mundo ---
  // Coordenada do evento → pixel INTERNO do canvas. Corrige o "torto" no mobile:
  // o canvas é exibido por CSS em tamanho != canvas.width/height, então é preciso
  // escalar. Funciona pra mouse E touch (MouseEvent sintético do touch também).
  function eventToCanvasPx(ev) {
    const rect = canvas.getBoundingClientRect();
    const src = (ev.touches && ev.touches[0]) ? ev.touches[0]
              : (ev.changedTouches && ev.changedTouches[0]) ? ev.changedTouches[0]
              : ev;
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    return {
      cx: (src.clientX - rect.left) * scaleX,
      cy: (src.clientY - rect.top) * scaleY,
    };
  }

  // O mapa é desenhado ajustado ao canvas (preserva aspect ratio).
  function getDrawRect() {
    if (!mapImage) return null;
    const cw = canvas.width, ch = canvas.height;
    const iw = mapImage.width, ih = mapImage.height;
    const scale = Math.min(cw / iw, ch / ih);
    const dw = iw * scale, dh = ih * scale;
    const dx = (cw - dw) / 2, dy = (ch - dh) / 2;
    return { dx, dy, dw, dh, scale };
  }

  // Converte pixel do canvas para coordenada do mundo (frame 'map').
  // Considera que o PNG já foi virado verticalmente pelo backend, então a
  // linha 0 do PNG corresponde ao topo do mapa (y_max no mundo).
  function canvasToWorld(cx, cy) {
    const r = getDrawRect();
    if (!r) return null;
    const px_in_img = (cx - r.dx) / r.scale;     // coluna do PNG
    const py_in_img = (cy - r.dy) / r.scale;     // linha do PNG (top = 0)
    if (px_in_img < 0 || px_in_img >= mapInfo.width) return null;
    if (py_in_img < 0 || py_in_img >= mapInfo.height) return null;
    // Linha do PNG → linha do grid original (origem no canto inferior)
    const grid_row = (mapInfo.height - 1) - py_in_img;
    const world_x = mapInfo.origin_x + px_in_img * mapInfo.resolution;
    const world_y = mapInfo.origin_y + grid_row  * mapInfo.resolution;
    return { x: world_x, y: world_y };
  }

  function worldToCanvas(wx, wy) {
    const r = getDrawRect();
    if (!r) return null;
    const px_in_img = (wx - mapInfo.origin_x) / mapInfo.resolution;
    const grid_row  = (wy - mapInfo.origin_y) / mapInfo.resolution;
    const py_in_img = (mapInfo.height - 1) - grid_row;
    return {
      x: r.dx + px_in_img * r.scale,
      y: r.dy + py_in_img * r.scale,
    };
  }

  // --- Render loop (chamado sob demanda) ---
  function render() {
    ctx.fillStyle = '#222';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    if (!mapImage) {
      ctx.fillStyle = '#888';
      ctx.font = '14px sans-serif';
      ctx.fillText('Aguardando /map...', 20, 30);
      return;
    }
    const r = getDrawRect();
    ctx.drawImage(mapImage, r.dx, r.dy, r.dw, r.dh);

    // Borda do mapa
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    ctx.strokeRect(r.dx, r.dy, r.dw, r.dh);

    // Trajetória planejada (Nav2)
    if (plan && plan.length > 1) {
      ctx.strokeStyle = '#4af';
      ctx.lineWidth = 2;
      ctx.beginPath();
      plan.forEach((p, i) => {
        const c = worldToCanvas(p.x, p.y);
        if (!c) return;
        if (i === 0) ctx.moveTo(c.x, c.y);
        else         ctx.lineTo(c.x, c.y);
      });
      ctx.stroke();
    }

    // Portas marcadas: segmento entre batentes + discos; ativa = destacada
    doors.forEach(d => {
      const a = worldToCanvas(d.a[0], d.a[1]);
      const b = worldToCanvas(d.b[0], d.b[1]);
      if (!a || !b) return;
      const active = doorZone && doorZone.door_id === d.id
                     && doorZone.state !== 'idle';
      ctx.strokeStyle = active ? '#0f0' : '#0aa';
      ctx.lineWidth = active ? 3 : 2;
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      const rPix = (0.30 / mapInfo.resolution) * getDrawRect().scale;
      [a, b].forEach(p => {
        ctx.beginPath(); ctx.arc(p.x, p.y, rPix, 0, 2 * Math.PI); ctx.stroke();
      });
      ctx.fillStyle = ctx.strokeStyle;
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'left';
      ctx.textBaseline = 'alphabetic';
      ctx.fillText(`🚪${d.id}`, (a.x + b.x) / 2 + 6, (a.y + b.y) / 2 - 6);
    });
    if (doorMode && doorDrag) {
      const aPx = worldToCanvas(doorDrag.ax, doorDrag.ay);
      const end = snapDoorEnd(doorDrag.ax, doorDrag.ay,
                              doorDrag.curWorld.x, doorDrag.curWorld.y,
                              doorDrag.shift);
      const bPx = worldToCanvas(end.x, end.y);
      if (aPx && bPx) {
        ctx.strokeStyle = '#0aa'; ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath(); ctx.moveTo(aPx.x, aPx.y); ctx.lineTo(bPx.x, bPx.y); ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#0aa';
        [aPx, bPx].forEach(p => {
          ctx.beginPath(); ctx.arc(p.x, p.y, 4, 0, 2 * Math.PI); ctx.fill();
        });
      }
    }

    // Último alvo goal único (bolinha vermelha) — esconde se waypoints ativos
    if (lastGoal && waypoints.length === 0) {
      const c = worldToCanvas(lastGoal.x, lastGoal.y);
      if (c) {
        ctx.fillStyle = '#e33';
        ctx.beginPath();
        ctx.arc(c.x, c.y, 6, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    // Linhas conectando waypoints
    if (waypoints.length > 1) {
      ctx.strokeStyle = 'rgba(96,165,250,0.5)';
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      waypoints.forEach((wp, i) => {
        const c = worldToCanvas(wp.x, wp.y);
        if (!c) return;
        if (i === 0) ctx.moveTo(c.x, c.y); else ctx.lineTo(c.x, c.y);
      });
      if (wpLoopChk && wpLoopChk.checked && waypoints.length > 1) {
        const c0 = worldToCanvas(waypoints[0].x, waypoints[0].y);
        if (c0) ctx.lineTo(c0.x, c0.y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Marcadores de waypoints
    waypoints.forEach((wp, i) => {
      const c = worldToCanvas(wp.x, wp.y);
      if (!c) return;
      const isActive = wpActive && i === wpActiveIdx;
      const isDone   = wpActive && i < wpActiveIdx;
      const r = 10;

      // Seta de orientação
      ctx.save();
      ctx.translate(c.x, c.y);
      ctx.rotate(-wp.yaw);
      ctx.strokeStyle = isActive ? '#facc15' : isDone ? '#4ade80' : '#60a5fa';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(r + 6, 0);
      ctx.moveTo(r + 6, 0);
      ctx.lineTo(r, -4);
      ctx.moveTo(r + 6, 0);
      ctx.lineTo(r, 4);
      ctx.stroke();
      ctx.restore();

      // Círculo com número
      ctx.beginPath();
      ctx.arc(c.x, c.y, r, 0, Math.PI * 2);
      ctx.fillStyle = isActive ? '#facc15' : isDone ? '#065f46' : '#1d4ed8';
      ctx.fill();
      ctx.strokeStyle = isActive ? '#fff' : '#93c5fd';
      ctx.lineWidth = isActive ? 2 : 1;
      ctx.stroke();

      ctx.fillStyle = isActive ? '#000' : '#fff';
      ctx.font = `bold ${r}px sans-serif`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(i + 1, c.x, c.y);
    });

    // Preview de orientação durante drag
    if (wpDrag && wpMode) {
      const c = { x: wpDrag.canvasX, y: wpDrag.canvasY };
      const dx = wpDrag.curX - c.x;
      const dy = wpDrag.curY - c.y;
      if (Math.sqrt(dx * dx + dy * dy) > 4) {
        ctx.save();
        ctx.translate(c.x, c.y);
        ctx.rotate(Math.atan2(dy, dx));
        ctx.strokeStyle = '#facc15';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 3]);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.sqrt(dx * dx + dy * dy), 0);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      }
      // Círculo preview
      ctx.beginPath();
      ctx.arc(c.x, c.y, 10, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(250,204,21,0.3)';
      ctx.fill();
      ctx.strokeStyle = '#facc15';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Preview de "Definir pose" durante o drag (magenta)
    if (setPoseDrag) {
      const c = { x: setPoseDrag.canvasX, y: setPoseDrag.canvasY };
      const dx = setPoseDrag.curX - c.x;
      const dy = setPoseDrag.curY - c.y;
      if (Math.sqrt(dx * dx + dy * dy) > 4) {
        ctx.save();
        ctx.translate(c.x, c.y);
        ctx.rotate(Math.atan2(dy, dx));
        ctx.strokeStyle = '#e879f9';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(Math.sqrt(dx * dx + dy * dy), 0);
        ctx.stroke();
        ctx.restore();
      }
      ctx.beginPath();
      ctx.arc(c.x, c.y, 8, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(232,121,249,0.35)';
      ctx.fill();
      ctx.strokeStyle = '#e879f9';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Robô — QUADRADO no tamanho real (footprint 0.5×0.5 m) na escala do mapa,
    // com risco de direção (yaw). Antes era uma seta fixa de 10 px, FORA de
    // escala, então parecia longe dos obstáculos que ele já encostava.
    if (robotPose) {
      const c = worldToCanvas(robotPose.x, robotPose.y);
      const dr = getDrawRect();
      if (c && dr) {
        const ROBOT_SIZE_M = 0.5;   // lado do footprint (base_link ±0.25 m)
        const half = (ROBOT_SIZE_M / mapInfo.resolution) * dr.scale / 2;
        ctx.save();
        ctx.translate(c.x, c.y);
        // No PNG y cresce pra baixo, então yaw (CCW positivo) é negativo visualmente.
        ctx.rotate(-robotPose.yaw);
        // Corpo: quadrado translúcido do tamanho real do robô
        ctx.fillStyle = 'rgba(255,153,0,0.35)';
        ctx.fillRect(-half, -half, half * 2, half * 2);
        ctx.strokeStyle = '#f90';
        ctx.lineWidth = 2;
        ctx.strokeRect(-half, -half, half * 2, half * 2);
        // Direção: risco do centro até a frente (+x do robô)
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(half, 0);
        ctx.stroke();
        ctx.restore();
      }
    }
  }

  // Cada handler que muda estado (map_update, robot_pose, plan_update,
  // mouse, waypoints, ...) já chama render() diretamente — manter um
  // setInterval(render, 66) redesenharia o canvas a 15 Hz mesmo parado,
  // queimando CPU sem motivo no Pi 4. Removido a favor do "render on demand".
})();
