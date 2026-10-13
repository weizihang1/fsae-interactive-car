// 把 site/app.js（连同 three.js）打包成一个 IIFE，供 build.py 内嵌进单文件网页。
// 只有在升级 three.js 版本或修改前端代码后才需要重新运行；
// 打包产物 site/vendor/app.bundle.js 需要入库，这样 CI 与本地构建都不必联网。
const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const root = path.join(__dirname, '..');
// 注意：esbuild 的 alias 目标必须以 ./ 开头，否则会被当成 npm 包名解析
const vendor = './.tooling/vendor';
const alias = [
  '--alias:three=' + vendor + '/build/three.module.js',
  '--alias:three/addons/controls/OrbitControls.js=' + vendor + '/examples/jsm/controls/OrbitControls.js',
  '--alias:three/addons/loaders/GLTFLoader.js=' + vendor + '/examples/jsm/loaders/GLTFLoader.js',
].join(' ');

const cmd = [
  'pnpm dlx esbuild@0.24.0 site/app.js',
  '--bundle --format=iife --minify --target=es2020 --legal-comments=none',
  alias,
  '--outfile=site/vendor/app.bundle.js',
].join(' ');

execSync(cmd, { cwd: root, stdio: 'inherit' });

const out = path.join(root, 'site/vendor/app.bundle.js');
console.log('打包完成：site/vendor/app.bundle.js  '
  + (fs.statSync(out).size / 1024).toFixed(1) + ' KB');
