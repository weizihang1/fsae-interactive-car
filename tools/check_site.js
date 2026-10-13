// 校验生成的 site/index.html：
//   1. 所有占位符都被正确替换；
//   2. 内嵌的零件清单是合法 JSON；
//   3. 内嵌的模型数据确实是 GLB；
//   4. 打包后的应用脚本语法正确。
const fs = require('fs');
const path = require('path');

const target = path.join(__dirname, '..', 'site', 'index.html');
const html = fs.readFileSync(target, 'utf8');

function fail(msg) {
  console.error('失败：' + msg);
  process.exit(1);
}

['__APP_BUNDLE__', '__CAR_GLB_BASE64__', '__PARTS_JSON__'].forEach(function (ph) {
  if (html.indexOf(ph) >= 0) fail('占位符 ' + ph + ' 未被替换，网页将无法工作');
});

const cfg = html.match(
  /window\.__CAR_MANIFEST__\s*=\s*(\{[\s\S]*?\});\s*window\.__CAR_GLB__\s*=\s*"([A-Za-z0-9+/=]+)"/);
if (!cfg) fail('未找到内嵌的零件清单与模型数据');

let manifest = null;
try {
  manifest = JSON.parse(cfg[1]);
} catch (err) {
  fail('零件清单不是合法 JSON：' + err.message);
}
if (!manifest.totals || !manifest.totals.parts) fail('零件清单缺少 totals');

const glb = Buffer.from(cfg[2], 'base64');
if (glb.length < 12 || glb.readUInt32LE(0) !== 0x46546C67) {
  fail('内嵌数据不是合法 GLB（魔数不匹配）');
}
const glbVersion = glb.readUInt32LE(4);
const glbDeclared = glb.readUInt32LE(8);
if (glbDeclared !== glb.length) {
  fail('GLB 声明长度 ' + glbDeclared + ' 与实际 ' + glb.length + ' 不一致');
}

const blocks = Array.from(html.matchAll(/<script>([\s\S]*?)<\/script>/g));
if (blocks.length < 2) fail('未找到应用脚本');
const app = blocks[blocks.length - 1][1];
try {
  new Function(app);
} catch (err) {
  fail('应用脚本存在语法错误：' + err.message);
}

console.log('OK: 网页校验通过（' + manifest.totals.parts + ' 个零件，GLB v' + glbVersion
  + ' ' + (glb.length / 1048576).toFixed(2) + ' MB，HTML '
  + (Buffer.byteLength(html) / 1048576).toFixed(2) + ' MB）');
