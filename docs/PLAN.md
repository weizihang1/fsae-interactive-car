# 大学生方程式（FSAE）底盘建模方案 v1

> **后续版本**：可交互整车 + 风洞 + 驾驶的综合方案见 [PLAN-interactive-car.md](./PLAN-interactive-car.md)。

> 目标：不依赖本机 CAD 软件，把"建模"变成 GitHub 上的代码 + 自动算力 + 浏览器查看。
> 状态：草案，等待确认建模对象与输入数据后细化。

---

## 0. 先说清楚环境现实（全部已实测）

| 项目 | 实测结果 | 影响 |
| --- | --- | --- |
| 工作区仓库 | 空仓库，无任何提交 | 从零开始 |
| GitHub CLI (`gh`) | 未安装 | 建仓/改仓需要先解决一条授权通道 |
| GitHub 凭据 / Token | 无 git 配置、无凭据、无 token | 推送前必须补授权 |
| 网络 | 沙盒内被拦；放行后可正常访问 GitHub / PyPI | 每次联网操作需你点一次允许 |
| CAD / CAE skill | **不存在**。与建模沾边的只有 visualize、spreadsheets、documents、pdf、typst、sympy、find-skills / skill-installer | 没有"一键出三维模型"的技能，需自建 |
| 本机 Python | 自带 3.12.14，已有 numpy / pandas；缺少 scipy / matplotlib / cadquery 等 | 缺失包均有 Windows+3.12 预编译轮子（已确认可安装） |

**结论**：不存在"用现成 skill 直接建模"这种捷径。能满足"走 GitHub、不在本机装软件、又能产出真实几何"的技术路线只有一条：

> **代码化建模（code-CAD）+ GitHub Actions 当算力 + GitHub Pages 当查看器**
>
> 模型 = 代码。改一个参数 = 一次提交；CI 自动重算并产出 STEP / STL / 图表 / 报告；你在浏览器里看结果、点链接下载文件。

---

## 1. 仓库骨架（一次搭好，长期复用）

```
fsae-chassis/
├─ .github/workflows/
│   ├─ build.yml        几何构建 + 渲染出图
│   ├─ fea.yml          刚度 / 强度计算（CalculiX）
│   └─ rules.yml        规则合规检查（管径壁厚、主环、侵入量…）
├─ geometry/            参数化几何定义（CadQuery / build123d）
│   ├─ frame/           车架管件（节点表 + 管径壁厚表）
│   └─ suspension/      双叉臂硬点、立柱、摇臂
├─ kinematics/          运动学求解器（camber / toe / 侧倾中心 / 轮距变化 vs 行程）
├─ fea/                 网格导出 + 算例（车架扭转刚度、立柱强度）
├─ rules/               规则约束脚本
├─ outputs/             STEP / STL / DXF / CSV / PNG（由 CI 生成，不入库大文件）
├─ site/                GitHub Pages：浏览器内交互查看 + 参数滑杆
└─ docs/                设计说明、参数表、变更记录
```

## 2. 技术选型（均验证过可获取）

| 环节 | 选型 | 理由 |
| --- | --- | --- |
| 实体建模 | CadQuery / build123d（纯 Python） | 模型可 diff、可评审、可参数扫描；依赖 cadquery-ocp 有 Win+3.12 轮子 |
| 悬架运动学 | numpy + 迭代求解 | 硬点包络、曲线族、灵敏度分析都天然适合代码 |
| 有限元 | CalculiX | 开源、可在 Actions 上跑车架扭转刚度与件强度 |
| 前端可视化 | three.js / vtk.js 单页 | 参考 MIT 许可的 `ZZzyy714227/Labsus`（124★，2026-09 仍在更新，浏览器单页 FSAE 悬架 K&C 工作台），可 fork 作底座 |
| 报告交付 | pdf / documents / spreadsheets skill | 出参数表、结果表、一页式 PDF 报告 |
| 自动化封装 | skill-creator 自建 `fsae-chassis` skill | 把这套流程固化成你能反复调用的技能 |

## 3. 阶段划分

- **S0 打通链路**：建仓 → CI → Pages → 用一个最小样例（几根管 + 一页悬架硬点图）跑通"提交即出图"。
  验收标准：推一次代码，几分钟后拿到可下载的 STEP 和网页截图。
- **S1 悬架几何 / K&C**：硬点参数化 → 运动学曲线（camber-toe vs 行程、侧倾中心高、瞬时中心、轮距变化）→ 参数扫描与优选。
- **S2 车架**：管径 / 壁厚 / 节点参数化 → 实体 + 质量表 → 扭转刚度 FEA → 规则校核（主环高度、前环、侵入量、管径壁厚下限）。
- **S3 交付**：参数表 + 结果报告 + 浏览器交互页 + 改前/改后叠加对比。

## 4. 需要你提供

1. **建模对象**：悬架几何/K&C、车架空间桁架、还是两块都做（或转向/制动/立柱等其它件）。
2. **GitHub 通道**（三选一）：装 `gh` 并由你登录 / 给我一个 PAT / 你先建空仓把地址给我。
3. **真实输入数据**：轮距、轴距、轮胎规格与胎压，现有硬点坐标或车架管件表，团队适用的规则约束版本。
   没有的话我可以用占位默认值先搭框架，但会在文档里明确标注"待替换"，结论仅用于验证链路，不能直接当设计依据。

## 5. 边界与风险（不打包票的部分）

- 我造不出比你们现役车更好的设计；我产出的是**可验证、可迭代、可评审的建模链路与计算结果**。
- FEA 结论的可信度取决于载荷假设与网格质量，我会附收敛性检查与假设清单。
- 我无法替你注册 GitHub 账号，也不会在无授权情况下推送代码。
- 若 GitHub 通道始终不通，退路是同一套代码在本机跑、Pages 换成离线 HTML——但那等于回到"在本机操作"，与你的要求相冲突。

---

## 附录 A：BlenderMCP 路线评估（2026-10-13 实测）

### A.1 事实核查

| 项目 | 实测结果 |
| --- | --- |
| 仓库 | `ahujasid/mcp-for-blender`（原名 `blender-mcp`，PyPI 包名已改为 `mcp-for-blender`，旧配置仍兼容） |
| 体量 | **30,562 star / 2,764 fork**，MIT 许可，最近提交 2026-10-06（活跃） |
| 架构 | 两端：Blender 插件 `addon.py`（在 Blender 内起 socket 服务）+ MCP 服务器 `src/blender_mcp/server.py`（客户端侧） |
| Codex 支持 | **官方一等公民**。仓库内有 `integrations/codex`（插件市场包），插件可在聊天侧栏内嵌 Blender 实时视口、点选物体带入对话 |
| 接入方式 | ① 插件：`codex plugin marketplace add ./blender-mcp/integrations/codex`；② 命令行：`codex mcp add blender -- uvx mcp-for-blender`；③ 桌面端：Settings → MCP servers → Add server → STDIO → `uvx mcp-for-blender` |
| 安全开关 | `BLENDER_MCP_SAFE_MODE=1` 可拦截脚本中的危险操作（默认关闭，即默认允许在 Blender 里跑任意 Python） |
| 本机现状 | **Blender 未安装**（三个标准安装路径均不存在）；**uv / uvx 未安装**；`~/.codex/config.toml` 已有 `[mcp_servers]` 段（当前仅 `cua_repl`），可按格式追加 |

### A.2 结论：能"直接建模"，但能不能建模**你要的东西**是另一回事

BlenderMCP 给我的是"在 Blender 里跑任意 Python / 操作网格 / 渲染"的能力，机制上确实是直接建模。但它下面这条边界必须先说清楚：

| 需求 | Blender 能否胜任 |
| --- | --- |
| 概念布局、三维沟通、设计报告配图、渲染 | 很好，这是它的主场 |
| 快速拼一个车架/悬架的视觉模型 | 可以（脚本生成管件 → 网格） |
| 参数化管件（外径/内径/节点表驱动的真实桁架） | 勉强，需要自己写生成脚本，改一处要重跑而非重算 |
| **STEP / IGES 工程格式交换** | **不行**（原生无此导出，仅有付费第三方插件） |
| **二维工程图纸、装配约束、尺寸链** | **不行**（这是 CAD 的活） |
| **有限元前处理与求解（扭转刚度/强度）** | **不行**，需另接 CalculiX/ANSYS |

一句话：**Blender 是造型与可视化工具，不是工程 CAD。** FSAE 车架/悬架的交付物（STEP、图纸、刚度报告、规则校核）它单独给不了。

### A.3 与"走 GitHub、不在本机操作"要求的冲突

BlenderMCP 天然是本机 GUI 路线：Blender 必须在你电脑上开着、插件服务必须点开。这与你的原始要求相反。

兼容两者的折中方案：**在 GitHub Actions 里跑 Blender 无头模式**（`blender --background --python script.py`），由 CI 产出渲染图 / GLB / STL 并发布到 Pages。这样既保留 Blender 的表现力，又不占用本机、不需要装 GUI 软件，还能版本化。代价是交互性不如实时视口。

### A.4 建议的组合

- **工程内核**：CadQuery / build123d → STEP + 质量表 + 传给 CalculiX 的网格。
- **可视化与汇报**：Blender（本地用 BlenderMCP 或 CI 无头）读 STEP/STL 出渲染图、装配动画、答辩素材。
- 两者共用同一份参数文件（节点表 / 硬点表），改数据两边同步更新。

### A.5 若要启用 BlenderMCP，需要的步骤与代价

1. 安装 uv（官方一行脚本，勿用 `pip install uv`）；
2. 安装 Blender（约 350 MB 下载）；
3. `uvx mcp-for-blender install-addon` 把插件装进 Blender；
4. Blender 侧栏（`N` 键）点 **Start MCP Server**；
5. 把 `blender` 注册进 Codex（设置界面或 `config.toml`），重启 Codex；
6. 之后工具才可用。

上述第 1–3 步涉及联网与写入用户目录，需要你批准后我才能执行。
