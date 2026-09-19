/* Bowling physics core. SI units internally. Works in browser (global BowlingSim) and Node (module.exports). */
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.BowlingSim = factory();
})(typeof self !== "undefined" ? self : this, function () {
  const IN = 0.0254;
  const FT = 0.3048;
  const G = 9.81;

  const LANE = {
    length: 60 * FT,          // foul line to headpin center
    width: 41.5 * IN,
    boards: 39,
    boardWidth: (41.5 * IN) / 39,
    arrowsY: 15 * FT,
    gutterWidth: 9.25 * IN,
    pitY: 60 * FT + 3 * 10.39 * IN + 36 * IN, // roughly where pins fall into the pit
  };
  const BALL = { radius: 8.5 * IN / 2, mass: 6.8 };  // 15 lb ball
  const PIN = { radius: 4.766 * IN / 2, mass: 1.58 }; // 3.5 lb pin
  const ROW = 10.39 * IN;                              // 12 in spacing * sin 60
  const COL = 6 * IN;

  // Pin numbering: x positive = right (from the bowler's view), boards count from the right for a right-hander.
  function pinLayout() {
    const y0 = LANE.length;
    return [
      { n: 1, x: 0, y: y0 },
      { n: 2, x: -COL, y: y0 + ROW }, { n: 3, x: COL, y: y0 + ROW },
      { n: 4, x: -2 * COL, y: y0 + 2 * ROW }, { n: 5, x: 0, y: y0 + 2 * ROW }, { n: 6, x: 2 * COL, y: y0 + 2 * ROW },
      { n: 7, x: -3 * COL, y: y0 + 3 * ROW }, { n: 8, x: -COL, y: y0 + 3 * ROW },
      { n: 9, x: COL, y: y0 + 3 * ROW }, { n: 10, x: 3 * COL, y: y0 + 3 * ROW },
    ];
  }

  function boardToX(board) { return (20 - board) * LANE.boardWidth; }
  function xToBoard(x) { return 20 - x / LANE.boardWidth; }
  function mphToMs(mph) { return mph * 0.44704; }
  function rpmToRad(rpm) { return rpm * 2 * Math.PI / 60; }

  const DEFAULTS = {
    speedMph: 17,
    revRpm: 350,
    axisRotationDeg: 45,   // 0 = pure forward roll (straight ball), 90 = full side roll (spinner-ish)
    axisTiltDeg: 15,
    standBoard: 20,        // ball lay-down board at the foul line
    targetBoard: 10.75,    // board the ball crosses at the arrows (15 ft)
    oilLengthFt: 40,
    oilFriction: 0.04,
    dryFriction: 0.22,
    hand: "R",
    ballLb: 15,            // ball weight; only matters at the pins (lane friction is mass-independent)
  };
  function ballMass(p) { return (p.ballLb || 15) * 0.4536; }

  // Coefficient of friction along the lane: oil in front, dry backend, ~1 m transition.
  function laneFriction(y, p) {
    const oilEnd = p.oilLengthFt * FT;
    const t = Math.min(1, Math.max(0, (y - oilEnd + 0.5) / 1.0));
    const s = t * t * (3 - 2 * t);
    return p.oilFriction + (p.dryFriction - p.oilFriction) * s;
  }

  function makeBall(p) {
    const hand = p.hand === "L" ? -1 : 1;
    const x0 = boardToX(p.standBoard);
    const xT = boardToX(p.targetBoard);
    const ang = Math.atan2(xT - x0, LANE.arrowsY);
    const v = mphToMs(p.speedMph);
    const w = rpmToRad(p.revRpm);
    const th = (p.axisRotationDeg * Math.PI / 180) * hand;
    const ph = p.axisTiltDeg * Math.PI / 180;
    // Rotation axis: pure forward roll is (-1,0,0); rotate toward the pins by axis rotation, then tilt up.
    const axis = [-Math.cos(th) * Math.cos(ph), -Math.sin(th) * Math.cos(ph), Math.sin(ph)];
    return {
      x: x0, y: 0, vx: v * Math.sin(ang), vy: v * Math.cos(ang),
      wx: w * axis[0], wy: w * axis[1], wz: w * axis[2],
      rolling: false, inGutter: false, phase: "skid",
    };
  }

  // One integration step of the ball on the lane. Coulomb friction at the contact patch drives skid -> hook -> roll.
  function stepBall(b, p, dt) {
    if (b.inGutter) {
      const gx = Math.sign(b.x) * (LANE.width / 2 + LANE.gutterWidth / 2);
      b.x += (gx - b.x) * Math.min(1, 8 * dt); b.vx = 0; b.y += b.vy * dt; return;
    }
    const R = BALL.radius, m = BALL.mass, I = 0.4 * m * R * R;
    // Contact point velocity relative to lane: v + w x r, r = (0,0,-R)
    const sx = b.vx + (-R * b.wy);
    const sy = b.vy + (R * b.wx);
    const slip = Math.hypot(sx, sy);
    const mu = laneFriction(b.y, p);
    if (slip > 0.02 && !b.rolling) {
      const ux = sx / slip, uy = sy / slip;
      const Fx = -mu * m * G * ux, Fy = -mu * m * G * uy;
      b.vx += Fx / m * dt; b.vy += Fy / m * dt;
      // torque = r x F = (R Fy, -R Fx, 0)
      b.wx += (R * Fy) / I * dt; b.wy += (-R * Fx) / I * dt;
      b.phase = Math.abs(b.vx) > 0.15 ? "hook" : "skid";
    } else {
      b.rolling = true; b.phase = "roll";
      const v = Math.hypot(b.vx, b.vy);
      const decel = 0.12 + mu * 0.6; // rolling resistance
      const nv = Math.max(0, v - decel * dt);
      if (v > 0) { b.vx *= nv / v; b.vy *= nv / v; }
      b.wy = -b.vx / R; b.wx = b.vy / R; // enforce rolling constraint
    }
    b.x += b.vx * dt; b.y += b.vy * dt;
    if (Math.abs(b.x) > LANE.width / 2 && b.y < LANE.length - 0.5) { b.inGutter = true; b.phase = "gutter"; }
  }

  function makePins() {
    return pinLayout().map((q) => ({ n: q.n, x: q.x, y: q.y, vx: 0, vy: 0, down: false, gone: false }));
  }

  function collide(a, b, ra, rb, ma, mb, e) {
    const dx = b.x - a.x, dy = b.y - a.y;
    const d = Math.hypot(dx, dy);
    const min = ra + rb;
    if (d === 0 || d >= min) return 0;
    const nx = dx / d, ny = dy / d;
    const rvx = b.vx - a.vx, rvy = b.vy - a.vy;
    const vn = rvx * nx + rvy * ny;
    // positional correction
    const pen = min - d;
    const tot = ma + mb;
    a.x -= nx * pen * (mb / tot); a.y -= ny * pen * (mb / tot);
    b.x += nx * pen * (ma / tot); b.y += ny * pen * (ma / tot);
    if (vn >= 0) return 0;
    const j = -(1 + e) * vn / (1 / ma + 1 / mb);
    a.vx -= j / ma * nx; a.vy -= j / ma * ny;
    b.vx += j / mb * nx; b.vy += j / mb * ny;
    return -vn;
  }

  const TOPPLE_SPEED = 0.35; // relative normal speed that knocks a standing pin over
  const DOWN_PIN_RADIUS = 0.08; // a toppled pin sweeps roughly its half-length, not just its belly
  function pinRadius(q) { return q.down ? DOWN_PIN_RADIUS : PIN.radius; }

  function stepPins(b, pins, p, dt) {
    const kick = LANE.width / 2 + LANE.gutterWidth;
    for (const q of pins) {
      if (q.gone || b.inGutter) continue;
      const hit = collide(b, q, BALL.radius, PIN.radius, ballMass(p), PIN.mass, 0.6);
      if (hit > TOPPLE_SPEED) q.down = true;
    }
    for (let i = 0; i < pins.length; i++) {
      for (let j = i + 1; j < pins.length; j++) {
        const a = pins[i], c = pins[j];
        if (a.gone || c.gone) continue;
        const hit = collide(a, c, pinRadius(a), pinRadius(c), PIN.mass, PIN.mass, 0.55);
        if (hit > TOPPLE_SPEED) { a.down = true; c.down = true; }
      }
    }
    for (const q of pins) {
      if (q.gone) continue;
      if (!q.down) { q.vx = 0; q.vy = 0; continue; }
      const v = Math.hypot(q.vx, q.vy);
      const nv = Math.max(0, v - 4.0 * dt); // fallen pins slide with heavy friction
      if (v > 0) { q.vx *= nv / v; q.vy *= nv / v; }
      q.x += q.vx * dt; q.y += q.vy * dt;
      if (Math.abs(q.x) > kick - PIN.radius) { q.x = Math.sign(q.x) * (kick - PIN.radius); q.vx *= -0.5; }
      if (q.y > LANE.pitY) q.gone = true;
      if (q.y < LANE.length - 0.6) { q.y = LANE.length - 0.6; q.vy = Math.abs(q.vy) * 0.3; }
    }
    // Ball off the back or into the pit
    if (b.y > LANE.pitY) { b.vx = 0; b.vy = 0; }
    if (Math.abs(b.x) > kick - BALL.radius) { b.x = Math.sign(b.x) * (kick - BALL.radius); b.vx *= -0.4; }
  }

  // Full shot. Returns trajectory samples, pin end states and derived stats.
  function simulate(params, opts) {
    const p = Object.assign({}, DEFAULTS, params || {});
    const dt = (opts && opts.dt) || 1 / 600;
    const record = !(opts && opts.noPath);
    const b = makeBall(p);
    const pins = makePins();
    const path = [];
    const stats = { entryBoard: null, entryAngleDeg: null, breakpointFt: null, breakpointBoard: null, gutter: false, speedAtPinsMph: null };
    let t = 0, sampleAcc = 0, reached = false, minVxSeen = 0, done = false, settle = 0;
    while (t < 12 && !done) {
      stepBall(b, p, dt);
      if (record) { sampleAcc += dt; if (sampleAcc >= 1 / 90) { sampleAcc = 0; path.push({ x: b.x, y: b.y, phase: b.phase }); } }
      // Breakpoint: farthest the ball travels away from the pocket side before it turns
      const hand = p.hand === "L" ? -1 : 1;
      if (!reached && hand * b.x > minVxSeen) { minVxSeen = hand * b.x; stats.breakpointFt = b.y / FT; stats.breakpointBoard = xToBoard(b.x); }
      if (!reached && b.y >= LANE.length - BALL.radius - PIN.radius) {
        reached = true;
        stats.entryBoard = xToBoard(b.x);
        stats.entryAngleDeg = Math.atan2(-hand * b.vx, b.vy) * 180 / Math.PI;
        stats.speedAtPinsMph = Math.hypot(b.vx, b.vy) / 0.44704;
        stats.gutter = b.inGutter;
      }
      if (b.y > LANE.length - 1) stepPins(b, pins, p, dt);
      if (reached) {
        const moving = pins.some((q) => !q.gone && q.down && Math.hypot(q.vx, q.vy) > 0.05) || Math.hypot(b.vx, b.vy) > 0.05 && b.y < LANE.pitY;
        if (!moving) settle += dt; else settle = 0;
        if (settle > 0.3 || b.y > LANE.pitY + 1) done = true;
      }
      if (b.inGutter && b.y > LANE.length + 1) done = true;
      t += dt;
    }
    const standing = pins.filter((q) => !q.down).map((q) => q.n);
    return { params: p, path, pins, standing, pinsDown: 10 - standing.length, strike: standing.length === 0, stats, ball: b };
  }

  // Human repeatability: run many shots with small random errors and report the strike rate.
  function strikeRate(params, n, noise) {
    const nz = Object.assign({ boardSd: 0.6, speedSd: 0.25, revSd: 15, targetSd: 0.5 }, noise || {});
    params = Object.assign({}, DEFAULTS, params || {});
    let strikes = 0; const leaves = {};
    for (let i = 0; i < n; i++) {
      const q = Object.assign({}, params, {
        standBoard: params.standBoard + gauss() * nz.boardSd,
        targetBoard: params.targetBoard + gauss() * nz.targetSd,
        speedMph: params.speedMph + gauss() * nz.speedSd,
        revRpm: params.revRpm + gauss() * nz.revSd,
      });
      const r = simulate(q, { noPath: true, dt: 1 / 400 });
      if (r.strike) strikes++;
      else { const k = r.standing.join("-"); leaves[k] = (leaves[k] || 0) + 1; }
    }
    const top = Object.entries(leaves).sort((a, b) => b[1] - a[1]).slice(0, 3);
    return { rate: strikes / n, n, topLeaves: top };
  }

  function gauss() {
    let u = 0, v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  // Sweep the target board (keeping stance fixed) and pick the line that puts the ball in the pocket
  // (board 17.5 for a righty) with the highest strike rate.
  function findLine(params, opts) {
    const o = Object.assign({ trials: 40, minBoard: 4, maxBoard: 25 }, opts || {});
    const hand = params.hand === "L" ? -1 : 1;
    const candidates = [];
    for (let tb = o.minBoard; tb <= o.maxBoard; tb += 0.25) {
      const q = Object.assign({}, params, { targetBoard: tb });
      const r = simulate(q, { noPath: true, dt: 1 / 400 });
      if (r.stats.entryBoard == null || r.stats.gutter) continue;
      const pocket = 20 - hand * 2.5;
      const err = Math.abs(r.stats.entryBoard - pocket);
      if (err < 1.5) candidates.push({ targetBoard: tb, entryBoard: r.stats.entryBoard, entryAngleDeg: r.stats.entryAngleDeg, strike: r.strike, err });
    }
    if (!candidates.length) return null;
    candidates.sort((a, b) => a.err - b.err);
    const best = candidates.slice(0, 6).map((c) => Object.assign(c, strikeRate(Object.assign({}, params, { targetBoard: c.targetBoard }), o.trials)));
    best.sort((a, b) => b.rate - a.rate || a.err - b.err);
    return best[0];
  }

  return { LANE, BALL, PIN, DEFAULTS, FT, IN, pinLayout, boardToX, xToBoard, laneFriction, simulate, strikeRate, findLine };
});
