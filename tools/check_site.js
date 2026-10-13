// 校验 site/index.html 内嵌的 ES 模块脚本语法（CI 与本地构建后都会跑）。
// 只做语法解析，不执行，因此不需要浏览器环境。
const fs = require('fs');
const path = require('path');

const target = path.join(__dirname, '..', 'site', 'index.html');
const html = fs.readFileSync(target, 'utf8');

const blocks = [...html.matchAll(/<script type="module">([\s\S]*?)<\/script>/g)];
if (!blocks.length) {
  console.error('失败：index.html 中未找到模块脚本');
  process.exit(1);
}

// 去掉 import 语句（Node 的 Function 构造器不接受 import 声明）
const body = blocks[blocks.length - 1][1]
  .replace(/^\s*import[\s\S]*?from\s+['"][^'"]*['"];?\s*$/gm, '');

const required = ['MANIFEST', 'partNodes', 'applyExplode', 'renderTree', 'select'];
for (const name of required) {
  if (!body.includes(name)) {
    console.error('失败：脚本中缺少必要标识符 ' + name);
    process.exit(1);
  }
}

try {
  new Function(body);
} catch (err) {
  console.error('失败：内嵌脚本存在语法错误 -> ' + err.message);
  process.exit(1);
}

const glb = html.match(/__CAR_GLB_BASE64__/);
if (glb) {
  console.error('失败：GLB 占位符未被替换，网页将无法显示模型');
  process.exit(1);
}

console.log('OK: 网页脚本语法通过（脚本 ' + body.length + ' 字符，文件 '
  + (Buffer.byteLength(html) / 1048576).toFixed(2) + ' MB）');
