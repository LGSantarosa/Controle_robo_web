/* face.js — cara do robô: olhos, sobrancelhas e boca (roda no iPad 2 / iOS 9.3.5!)
 *
 * ATENÇÃO: este arquivo é ES5 PURO de propósito. O iPad 2 parou no WebKit
 * de 2015: nada de sintaxe nem API pós-2015 — só var, function e XHR. Um
 * único token moderno mata o script INTEIRO em silêncio. O
 * test_face_app.py barra a lista de tokens proibidos — se ele reclamar
 * de uma linha sua, é isso.
 */
(function () {
  'use strict';

  var canvas = document.getElementById('face');
  var ctx = canvas.getContext('2d');
  var W = 0, H = 0;

  function resize() {
    W = window.innerWidth;
    H = window.innerHeight;
    canvas.width = W;
    canvas.height = H;
  }
  window.addEventListener('resize', resize, false);
  resize();

  var EYE_COLOR = '#39e6ff';

  function now() { return Date.now() / 1000; }
  function rand(a, b) { return a + Math.random() * (b - a); }

  // ---- estado ------------------------------------------------------------
  var gaze = { x: 0, y: 0 };          // olhar atual, -1..1
  var gazeTarget = { x: 0, y: 0 };
  var nextGazeAt = now() + 2;

  // Fase 2: enquanto now() < personHoldUntil, tem pessoa na mira — o
  // pollState manda no gazeTarget e o vagar/focused ficam de fora. O hold
  // de 3s segura o olhar quando a pessoa PARA (ela some dos clusters
  // móveis do lidar) — sem ping-pong pessoa/vagando.
  var personHoldUntil = 0;

  var blink = 0;                      // 0 = aberto, 1 = fechado
  var blinkPhase = 'idle';            // idle | closing | opening
  var nextBlinkAt = now() + 2;

  var mood = 'neutral';               // neutral | happy | squint | focused | yawn
  var moodUntil = 0;
  var nextMoodAt = now() + rand(8, 20);
  var yawnStart = 0;                  // época do bocejo em curso
  var yawnDur = 4;

  // ---- som (iOS só toca áudio DEPOIS de um toque na tela) ---------------
  // O 1º tap destrava: toca o "Olá" audível (confirma que o som funciona)
  // e destrava o "licença" com play+pause mascarado pelo olá.
  var sndOla = new Audio('/static/ola.mp3');
  var sndLicenca = new Audio('/static/licenca.mp3');
  sndOla.preload = 'auto';
  sndLicenca.preload = 'auto';
  var sndUnlocked = false;
  var nextOlaAt = 0;
  var nextLicencaAt = 0;

  function playSnd(a) {
    if (!sndUnlocked) return;
    try {
      a.currentTime = 0;
      var p = a.play();
      if (p && p['catch']) p['catch'](function () {});
    } catch (e) {}
  }

  function unlockAudio() {
    if (sndUnlocked) return;
    sndUnlocked = true;
    nextOlaAt = now() + 8;            // acabou de dar olá no tap
    try {
      var p = sndOla.play();
      if (p && p['catch']) p['catch'](function () {});
      var q = sndLicenca.play();
      var calaLicenca = function () {
        sndLicenca.pause();
        sndLicenca.currentTime = 0;
      };
      if (q && q['then']) q['then'](calaLicenca)['catch'](function () {});
      else calaLicenca();
    } catch (e) {}
  }

  // Cada humor é um alvo de "pose" da cara; o frame interpola até lá (lerp),
  // então trocar de humor nunca dá salto seco.
  //   browLift: sobrancelha sobe(+)/desce(-) — em unidades de m*0.04 px
  //   browTilt: rad; positivo = ponta INTERNA desce (cara de concentrado)
  //   eyeOpen : escala da abertura do olho (a piscada multiplica por cima)
  //   mouthCurve: positivo = canto da boca pra cima (sorriso)
  //   mouthOpen : 0 fechada .. 1 escancarada (bocejo)
  //   mouthW  : largura da boca em fração de m
  var MOODS = {
    neutral: { browLift: 0.0,  browTilt: 0.0,   eyeOpen: 1.0,  mouthCurve: 0.18,  mouthOpen: 0.04, mouthW: 0.26 },
    happy:   { browLift: 0.5,  browTilt: -0.12, eyeOpen: 1.0,  mouthCurve: 1.0,   mouthOpen: 0.25, mouthW: 0.40 },
    squint:  { browLift: -0.2, browTilt: 0.10,  eyeOpen: 0.45, mouthCurve: 0.10,  mouthOpen: 0.03, mouthW: 0.22 },
    focused: { browLift: -0.8, browTilt: 0.35,  eyeOpen: 0.55, mouthCurve: -0.08, mouthOpen: 0.02, mouthW: 0.20 },
    yawn:    { browLift: 0.9,  browTilt: -0.15, eyeOpen: 0.15, mouthCurve: 0.05,  mouthOpen: 1.0,  mouthW: 0.24 }
  };

  // Pose atual da cara (começa no neutro; o tick a puxa pro alvo do humor).
  var P = {
    browLift: 0, browTilt: 0, eyeOpen: 1,
    mouthCurve: 0.18, mouthOpen: 0.04, mouthW: 0.26
  };

  // Global de propósito: gancho da fase 2 (cara reativa ao estado do robô)
  // e útil pra brincar no console do navegador: setMood('happy', 5),
  // setMood('yawn'), setMood('focused', 8)
  window.setMood = function (m, secs) {
    mood = m;
    if (m === 'yawn') {
      // Bocejo é uma ANIMAÇÃO com começo/meio/fim, não um estado parado:
      // a duração é dele, ignora secs.
      yawnStart = now();
      yawnDur = rand(3.5, 4.5);
      moodUntil = yawnStart + yawnDur;
    } else {
      moodUntil = now() + (secs || 2.5);
    }
  };

  function lerp(a, b, k) { return a + (b - a) * k; }

  // Toque/clique = próxima expressão da fila — pra demonstrar pros outros
  // sem precisar do console. Fila fixa (não sorteio): quem demonstra sabe
  // o que vem. No iPad o touchstart também gera um click ~300ms depois;
  // o preventDefault + a janela de 0.5s seguram o disparo duplo.
  var DEMO = ['happy', 'yawn', 'focused', 'squint'];
  var demoIdx = 0;
  var lastTapAt = 0;

  // Android/desktop: o 1º toque também pede TELA CHEIA (a API exige gesto
  // do usuário, por isso mora aqui no tap). O iPad 2 não tem a API e sai
  // no guard — lá a tela cheia vem do Adicionar à Tela de Início.
  function goFullscreen() {
    if (document.fullscreenElement || document.webkitFullscreenElement) return;
    var el = document.documentElement;
    var fn = el.requestFullscreen || el.webkitRequestFullscreen;
    if (!fn) return;
    var p = fn.call(el);
    // Navegador moderno devolve uma promessa; engole a rejeição (usuário
    // pode negar) sem nomear o tipo — o léxico do teste barra o nome.
    if (p && p['catch']) p['catch'](function () {});
  }

  function onTap(ev) {
    var t = now();
    if (t - lastTapAt < 0.5) return;
    lastTapAt = t;
    goFullscreen();
    unlockAudio();
    window.setMood(DEMO[demoIdx], 3.5);
    demoIdx = (demoIdx + 1) % DEMO.length;
    if (ev.preventDefault) ev.preventDefault();
  }
  canvas.addEventListener('touchstart', onTap, false);
  canvas.addEventListener('mousedown', onTap, false);

  // ---- comportamento -------------------------------------------------------
  function tick(t) {
    // Olhar vagando: alvo novo a cada 4-10s, chega devagar (lerp).
    // Concentrado NÃO vaga: trava o olhar no centro (encarar = concentração).
    if (t < personHoldUntil) {
      // pessoa na mira: gazeTarget é do pollState, ninguém mexe
    } else if (mood === 'focused') {
      gazeTarget.x = 0;
      gazeTarget.y = 0;
    } else if (t >= nextGazeAt) {
      gazeTarget.x = rand(-1, 1);
      gazeTarget.y = rand(-0.5, 0.5);
      nextGazeAt = t + rand(4, 10);
    }
    // Pessoa na mira acompanha mais rápido que o vagar, mas sem perseguir
    // tremida (0.18 flicava na tela — dono, 07-17).
    var gazeK = (t < personHoldUntil) ? 0.10 : 0.04;
    gaze.x += (gazeTarget.x - gaze.x) * gazeK;
    gaze.y += (gazeTarget.y - gaze.y) * gazeK;

    // Piscada a cada 3-7s: fecha rápido, abre um pouco mais devagar.
    if (blinkPhase === 'idle' && t >= nextBlinkAt) blinkPhase = 'closing';
    if (blinkPhase === 'closing') {
      blink += 0.20;
      if (blink >= 1) { blink = 1; blinkPhase = 'opening'; }
    } else if (blinkPhase === 'opening') {
      blink -= 0.12;
      if (blink <= 0) {
        blink = 0;
        blinkPhase = 'idle';
        nextBlinkAt = t + rand(3, 7);
        if (Math.random() < 0.15) nextBlinkAt = t + 0.25;  // piscada dupla
      }
    }

    // Micro-expressão a cada 30-60s, volta pro neutro sozinha.
    if (mood === 'neutral' && t >= nextMoodAt) {
      var r = Math.random();
      var m2 = 'happy';
      if (r >= 0.35 && r < 0.55) m2 = 'squint';
      else if (r >= 0.55 && r < 0.80) m2 = 'focused';
      else if (r >= 0.80) m2 = 'yawn';
      window.setMood(m2, (m2 === 'focused') ? rand(3, 5) : rand(2, 3));
    }
    if (mood !== 'neutral' && t >= moodUntil) {
      mood = 'neutral';
      nextMoodAt = t + rand(30, 60);
    }

    // Alvo de pose do humor atual. Bocejo é dinâmico: um envelope de seno
    // (0 no início, pico no meio, 0 no fim) mistura neutro e bocejo — a
    // boca escancara e fecha, os olhos vão junto.
    var T = MOODS[mood] || MOODS.neutral;
    if (mood === 'yawn') {
      var p = (t - yawnStart) / yawnDur;
      if (p < 0) p = 0;
      if (p > 1) p = 1;
      var e = Math.sin(Math.PI * p);
      var N = MOODS.neutral, Y = MOODS.yawn;
      T = {
        browLift: lerp(N.browLift, Y.browLift, e),
        browTilt: lerp(N.browTilt, Y.browTilt, e),
        eyeOpen: lerp(N.eyeOpen, Y.eyeOpen, e),
        mouthCurve: lerp(N.mouthCurve, Y.mouthCurve, e),
        mouthOpen: lerp(N.mouthOpen, Y.mouthOpen, e),
        mouthW: lerp(N.mouthW, Y.mouthW, e)
      };
    }
    P.browLift += (T.browLift - P.browLift) * 0.14;
    P.browTilt += (T.browTilt - P.browTilt) * 0.14;
    P.eyeOpen += (T.eyeOpen - P.eyeOpen) * 0.14;
    P.mouthCurve += (T.mouthCurve - P.mouthCurve) * 0.14;
    P.mouthOpen += (T.mouthOpen - P.mouthOpen) * 0.14;
    P.mouthW += (T.mouthW - P.mouthW) * 0.14;
  }

  // ---- desenho -------------------------------------------------------------
  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  // Sobrancelha: barra arredondada acima do olho, girada pelo tilt.
  // side = -1 (esquerda) / +1 (direita): espelha o giro pra ponta interna
  // descer nos DOIS lados quando browTilt é positivo.
  function drawBrow(cx, cy, side, ew, eh, m) {
    var bw = ew * 0.92;
    var bh = m * 0.045;
    var topY = cy - eh / 2 - m * 0.075 - P.browLift * m * 0.04;
    ctx.save();
    ctx.translate(cx, topY);
    ctx.rotate(-side * P.browTilt);
    roundRect(-bw / 2, -bh / 2, bw, bh, bh / 2);
    ctx.fillStyle = EYE_COLOR;
    ctx.fill();
    ctx.restore();
  }

  // Boca: dois lábios em curva quadrática partindo dos cantos. Fechada, o
  // traço grosso vira uma linha curva (sorriso/neutra); aberta, o meio
  // desce e vira um "O" arredondado (bocejo).
  function drawMouth(mx, my, m) {
    var mw = P.mouthW * m;
    var lift = P.mouthCurve * mw * 0.30;   // canto sobe em relação ao centro
    var gape = P.mouthOpen * m * 0.20;     // abertura vertical
    var x0 = mx - mw / 2, x1 = mx + mw / 2;
    var yEdge = my - lift;
    ctx.strokeStyle = EYE_COLOR;
    ctx.fillStyle = EYE_COLOR;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.lineWidth = Math.max(m * 0.022, 4);
    ctx.beginPath();
    ctx.moveTo(x0, yEdge);
    ctx.quadraticCurveTo(mx, my + lift - gape * 0.6, x1, yEdge);
    ctx.quadraticCurveTo(mx, my + lift + gape * 1.6, x0, yEdge);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  function draw() {
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);

    var m = Math.min(W, H);
    var ew = m * 0.30;                       // largura do olho
    var eh = m * 0.38;                       // altura de olho aberto
    var half = ew * 0.55 + ew / 2;           // centro do olho até o meio

    var open = (1 - blink) * P.eyeOpen;
    var ehNow = Math.max(eh * open, eh * 0.06);   // nunca some de vez

    // Olhos um pouco acima do meio pra sobrar lugar pra boca embaixo.
    var cy = H * 0.44 + gaze.y * eh * 0.25;
    var offX = gaze.x * ew * 0.35;
    // Pessoa a 90° = olho COLADO na lateral (dono 07-17): o alvo pede
    // além da borda e o clamp encosta o olho de fora na moldura da tela.
    var maxOff = W / 2 - half - ew / 2 - m * 0.02;
    if (maxOff < 0) maxOff = 0;
    if (offX > maxOff) offX = maxOff;
    if (offX < -maxOff) offX = -maxOff;
    var r, i, cx;

    for (i = -1; i <= 1; i += 2) {
      cx = W / 2 + i * half + offX;
      r = Math.min(ew, ehNow) * 0.35;
      roundRect(cx - ew / 2, cy - ehNow / 2, ew, ehNow, r);
      ctx.fillStyle = EYE_COLOR;
      ctx.fill();
      if (mood === 'happy' && open > 0.5) {
        // Recorte preto por baixo: o olho vira meia-lua feliz (^ ^).
        ctx.fillStyle = '#000';
        ctx.beginPath();
        ctx.arc(cx, cy + ehNow * 0.55, ew * 0.75, 0, Math.PI * 2);
        ctx.fill();
      }
      drawBrow(cx, cy, i, ew, eh, m);
    }

    drawMouth(W / 2 + offX * 0.5, cy + eh * 0.72, m);
  }

  // ---- fase 2: olhos seguem a pessoa (poll no /state) --------------------
  // XHR puro (iPad 2!). Qualquer falha — rede, JSON, timeout — é tratada
  // como "sem pessoa": o hold expira sozinho e a cara volta a vagar.
  function pollState() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/state', true);
    xhr.timeout = 250;
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4 || xhr.status !== 200) return;
      var st = null;
      try { st = JSON.parse(xhr.responseText); } catch (e) { return; }
      if (!st) return;
      var tp = now();
      // Guard travado POR UMA PESSOA -> pede licença, e CONTINUA pedindo
      // enquanto está preso (a cada 8s). FORA do if(person) de propósito:
      // pessoa PARADA some do detector de movimento (person/x ficam falsos),
      // mas o guard segue 'blocked' por ela — era por isso que ele calava
      // depois do 1º pedido quando alguém parava na frente por muito tempo.
      if (st.blocked && tp >= nextLicencaAt) {
        playSnd(sndLicenca);
        nextLicencaAt = tp + 8;
      } else if (!st.blocked) {
        nextLicencaAt = 0;   // destravou -> próximo bloqueio pede na hora
      }
      if (st.person) {
        // 2.2x: pessoa desloca o olho BEM mais que o vagar (que fica em
        // ±1) — a 90° o alvo passa da borda e o clamp do draw() cola o
        // olho na lateral.
        // Banda morta: o rumo do lidar treme alguns graus parado; alvo só
        // mexe se mudou de verdade (~4°), senão o olho flicava (07-17).
        var nx = st.x * 2.2;
        if (Math.abs(nx - gazeTarget.x) > 0.11) gazeTarget.x = nx;
        gazeTarget.y = 0.1;
        // Pessoa NOVA (hold tinha expirado = ficou 3s+ sem ninguém): olá!
        if (tp >= personHoldUntil && tp >= nextOlaAt) {
          playSnd(sndOla);
          nextOlaAt = tp + 30;
        }
        personHoldUntil = tp + 3;
      }
    };
    xhr.send();
  }
  setInterval(pollState, 300);

  function frame() {
    tick(now());
    draw();
    window.requestAnimationFrame(frame);
  }
  window.requestAnimationFrame(frame);
})();
