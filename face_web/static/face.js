/* face.js — olhos do robô (roda no iPad 2 / iOS 9.3.5!)
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

  var blink = 0;                      // 0 = aberto, 1 = fechado
  var blinkPhase = 'idle';            // idle | closing | opening
  var nextBlinkAt = now() + 2;

  var mood = 'neutral';               // neutral | happy | squint
  var moodUntil = 0;
  var nextMoodAt = now() + rand(8, 20);

  // Global de propósito: gancho da fase 2 (cara reativa ao estado do robô)
  // e útil pra brincar no console do navegador: setMood('happy', 5)
  window.setMood = function (m, secs) {
    mood = m;
    moodUntil = now() + (secs || 2.5);
  };

  // ---- comportamento -------------------------------------------------------
  function tick(t) {
    // Olhar vagando: alvo novo a cada 4-10s, chega devagar (lerp).
    if (t >= nextGazeAt) {
      gazeTarget.x = rand(-1, 1);
      gazeTarget.y = rand(-0.5, 0.5);
      nextGazeAt = t + rand(4, 10);
    }
    gaze.x += (gazeTarget.x - gaze.x) * 0.04;
    gaze.y += (gazeTarget.y - gaze.y) * 0.04;

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

    // Micro-expressão a cada 30-60s, dura 2-3s, volta pro neutro.
    if (mood === 'neutral' && t >= nextMoodAt) {
      mood = (Math.random() < 0.6) ? 'happy' : 'squint';
      moodUntil = t + rand(2, 3);
    }
    if (mood !== 'neutral' && t >= moodUntil) {
      mood = 'neutral';
      nextMoodAt = t + rand(30, 60);
    }
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

  function draw() {
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);

    var m = Math.min(W, H);
    var ew = m * 0.30;                       // largura do olho
    var eh = m * 0.42;                       // altura de olho aberto
    var half = ew * 0.55 + ew / 2;           // centro do olho até o meio

    var open = 1 - blink;
    if (mood === 'squint') open *= 0.45;
    var ehNow = Math.max(eh * open, eh * 0.06);   // nunca some de vez

    var cy = H / 2 + gaze.y * eh * 0.25;
    var offX = gaze.x * ew * 0.35;
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
    }
  }

  function frame() {
    tick(now());
    draw();
    window.requestAnimationFrame(frame);
  }
  window.requestAnimationFrame(frame);
})();
