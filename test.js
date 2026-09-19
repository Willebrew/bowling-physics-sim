const assert = require("node:assert/strict");
const S = require("./sim.js");

function run(name, fn) {
  try { fn(); console.log("ok   -", name); } catch (err) { console.log("FAIL -", name, "\n  ", err.message); process.exitCode = 1; }
}

run("straight ball through the pocket leaves pins (deflection)", () => {
  const r = S.simulate({ revRpm: 50, axisRotationDeg: 0, standBoard: 17.5, targetBoard: 17.5 }, { noPath: true });
  assert.ok(Math.abs(r.stats.entryBoard - 17.5) < 0.2);
  assert.ok(Math.abs(r.stats.entryAngleDeg) < 0.5);
});

run("right-hander hooks left, left-hander hooks right", () => {
  const rh = S.simulate({ hand: "R", standBoard: 20, targetBoard: 20 }, { noPath: true });
  const lh = S.simulate({ hand: "L", standBoard: 20, targetBoard: 20 }, { noPath: true });
  assert.ok(rh.stats.entryBoard > 20, "righty should finish left of center (higher board)");
  assert.ok(lh.stats.entryBoard < 20, "lefty should finish right of center (lower board)");
  assert.ok(Math.abs(rh.stats.entryBoard - (40 - lh.stats.entryBoard)) < 0.2, "mirror symmetry");
});

run("more revs = more hook and steeper entry angle", () => {
  const lo = S.simulate({ revRpm: 200 }, { noPath: true });
  const hi = S.simulate({ revRpm: 450 }, { noPath: true });
  assert.ok(hi.stats.entryAngleDeg > lo.stats.entryAngleDeg);
});

run("longer oil = later breakpoint and less hook", () => {
  const short = S.simulate({ oilLengthFt: 35 }, { noPath: true });
  const long = S.simulate({ oilLengthFt: 45 }, { noPath: true });
  assert.ok(long.stats.breakpointFt > short.stats.breakpointFt);
  assert.ok(long.stats.entryBoard < short.stats.entryBoard);
});

run("ball ends in the gutter when thrown wide", () => {
  const r = S.simulate({ standBoard: 20, targetBoard: 4 }, { noPath: true });
  assert.equal(r.stats.gutter, true);
  assert.equal(r.pinsDown, 0);
});

run("default line strikes and findLine returns a pocket line", () => {
  const r = S.simulate({}, { noPath: true });
  assert.equal(r.strike, true, "default should be a strike, left " + r.standing);
  const line = S.findLine(Object.assign({}, S.DEFAULTS), { trials: 10 });
  assert.ok(line && Math.abs(line.entryBoard - 17.5) < 1.5);
});

run("ball loses speed down the lane", () => {
  const r = S.simulate({ speedMph: 17 }, { noPath: true });
  assert.ok(r.stats.speedAtPinsMph < 16 && r.stats.speedAtPinsMph > 11);
});

run("heavier ball strikes at least as often as a light one on the same line", () => {
  const light = S.strikeRate({ ballLb: 10 }, 150);
  const heavy = S.strikeRate({ ballLb: 16 }, 150);
  assert.ok(heavy.rate >= light.rate, `heavy ${heavy.rate} vs light ${light.rate}`);
  assert.ok(S.simulate({ ballLb: 15 }).strike, "default weight still strikes");
});
