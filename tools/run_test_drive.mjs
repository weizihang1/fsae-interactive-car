// 打包并运行驾驶模式物理测试（用 esbuild 解析 "three" 别名）。
import { execSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.join(here, '..');
const vendor = './.tooling/vendor';
const cmd = [
  'pnpm dlx esbuild@0.24.0 tools/test_drive.js',
  '--bundle --format=esm --platform=node --target=node20',
  '--alias:three=' + vendor + '/build/three.module.js',
  '--outfile=.tooling/test_drive.bundle.mjs',
].join(' ');
execSync(cmd, { cwd: root, stdio: 'inherit' });
execSync('node .tooling/test_drive.bundle.mjs', { cwd: root, stdio: 'inherit' });
