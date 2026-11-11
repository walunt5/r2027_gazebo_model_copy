// 台面着色：俯视"正下方最高实体顶面"推算逻辑回归测试。
// 直接从 index.html 里抽出 isFilm / topSurfaceAt 两个函数跑, 保证测的是真代码。
const fs = require('fs');
const path = require('path');
const ROOT = path.resolve(__dirname, '..', '..', '..');
const src = fs.readFileSync(path.join(ROOT, 'tool/scripts/field_editor/index.html'), 'utf8');

function grab(name) {
  const i = src.indexOf('function ' + name);
  if (i < 0) throw new Error('function not found: ' + name);
  let d = 0;
  const j = src.indexOf('{', i);
  for (let k = j; k < src.length; k++) {
    if (src[k] === '{') d++;
    else if (src[k] === '}') { d--; if (!d) return src.slice(i, k + 1); }
  }
}

const S = { items: [
  { kind: 'collision', name: 'Cube', pose: [0, 0, 0.3, 0, 0, 0], geometry: { type: 'box', size: [6, 6, 0.6] } },
  { kind: 'collision', name: 'Cube.017', pose: [0, 0, 0.75, 0, 0, 0], geometry: { type: 'box', size: [3, 3, 0.3] } },
  { kind: 'collision', name: 'Cylinder', pose: [4.15, 0, 0.25, 0, 0, 0], geometry: { type: 'cylinder', radius: 0.135, length: 0.5 } },
  { kind: 'visual', name: 'visual', pose: [0, 0, 0, 0, 0, 0], geometry: { type: 'mesh', uri: 'x.dae' } },
  { kind: 'visual', name: 'colorfilm_1', pose: [2, 2, 0.6025, 0, 0, 0], geometry: { type: 'box', size: [1, 1, 0.002] }, film: true, color: [1, 0, 0] },
] };

eval(grab('isFilm'));
eval(grab('topSurfaceAt'));

const top = (x, y) => { const r = topSurfaceAt(x, y); return r.top === null ? null : +r.top.toFixed(4); };
const cases = [
  ['(0,0) 落在 L2 上 -> 0.90', top(0, 0), 0.9],
  ['(2,2) 只落在 L1 上 -> 0.60（不会穿到 L2）', top(2, 2), 0.6],
  ['(4.15,0) 圆柱基座 -> 0.50', top(4.15, 0), 0.5],
  ['(5,5) 场外 -> null', top(5, 5), null],
];
let ok = true;
for (const [label, got, exp] of cases) {
  const pass = got === exp;
  ok = ok && pass;
  console.log((pass ? '  PASS  ' : '  FAIL  ') + label + '  got=' + got);
}
console.log(ok ? 'RESULT: ALL PASS' : 'RESULT: FAIL');
process.exit(ok ? 0 : 1);
