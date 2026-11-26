# 参数曲面离散化器 (surface-mesher)

自适应参数曲面离散化：Rivara 最长边二分、跨片共享边统一采样身份、硬顶点预算、
真实网格导出，以及可替换的误差度量与独立执行证据。

## 图形入口（主要界面）

在曲面编辑视窗中操作：启动本地服务后打开浏览器。

```bash
npm start            # 默认 http://localhost:8080
```

视窗内可：

- 选择场景、位置/法线容差、顶点硬上限与预算模式；
- 选择 **误差度量**（解析界+采样 / 仅采样估计）与 **共享边协调策略**
  （统一缓存边界剖分 / 邻片协商细化深度）；
- 执行自适应剖分，实时显示网格密度与几何误差、逐面片指标、焊缝连接性、
  细化原因，以及来自独立复算的 **执行证据**（网格误差 + 共享边逐顶点对应）；
- 拖动中途更换误差度量并审核：废弃请求数、被拒绝的过期结果数、最终提交代次；
- 导出 OBJ / STL（导出与显示使用同一边界身份）。

## 命令行（辅助）

命令行仅为脚本化与抽查辅助；所有数字都来自真实剖分与独立复算，绝不伪造。

```bash
npm test                                   # 完整测试套件（含前后差分回归）
node test/smoke.js <scene> [cap] [mode] [errorMetric] [boundaryStrategy]
node test/trace.js <scene>                 # 旧样例：单面片逐趟追溯到收敛
npm run golden:gen                         # 从 git HEAD 的旧实现重建基线
```

示例：

```bash
node test/smoke.js weldedPair 10000 coarsen-remote analytic-sampled unified-cached
node test/smoke.js weldedPair 10000 coarsen-remote sampled-only negotiated-depth
```

## 模块分层

`src/discretization/` 自底向上：

| 模块 | 职责 | 边界 |
|------|------|------|
| `sampling.js` | 曲面采样：探针布局、单元尺寸、稠密超采样 | 仅几何 |
| `estimation.js` | 局部单元误差：可替换度量 + 可替换审核后端 | 不涉及接缝/预算 |
| `boundary.js` | 共享边协调：两种边界剖分策略 | 仅裂缝约束 |
| `mesher.js` | 细化驱动：消费显式估计，委派边界给策略 | 唯一耦合点 |
| `scheduler.js` | 交互请求调度：废弃请求审核 + 过期结果提交闸门 | 与研究无关 |
| `evidence.js` | 执行证据：从导出网格独立复算网格误差与共享边对应 | 不信任网格器自报 |

要点：

- **局部误差估计**（`estimation.js`）与**跨片裂缝约束**（`boundary.js`）分离；
  改变误差度量只会改变细化位置，不会扰动共享边顶点身份。
- **两种边界策略**在同一接缝上对比：`unified-cached` 以单一样本身份
  逐个顶点相接；`negotiated-depth` 各片独立采样并协商深度，暴露出
  T 接/裂缝——`evidence.js` 会如实报告，各片无法靠独立封边隐藏裂口。
- **审核后端**可替换：误差模块只把事件交给后端，切换后端不改变估计结果。
- **请求调度**保证较慢的旧结果不会覆盖新接缝。

## 兼容层

旧的曲面离散入口保持可用（行为冻结）：

- `src/mesher.js` — `require('./mesher')` 仍导出 `Mesher/PatchMesh/Weld/edgeKey/uvKey/MAX_DEPTH`，
  其 `Mesher` 即重构后的驱动，默认策略与默认度量与旧实现逐字节一致。
- `src/error.js` — `evaluateTriangle` 及保证界辅助函数原样再导出。

## 前后差分回归

`test/golden/golden.test.js` 对相邻片（`weldedPair`/`frame`/`budgetWeld`/`lips`）
与高曲率（`spike`/`band`）场景，比较重构前（`test/golden/baseline.json`，
由 `test/golden/gen.js` 从 git HEAD 的旧实现重建）与重构后默认路径：

- 顶点/单元数、导出网格去重顶点数与面数 **完全相等**；
- 每片独立网格误差（`gridError`）**浮点容差内相等**；
- 每条共享边的最大裂口与顶点错配数 **完全相等**（逐个相接）。

基线不依赖手改数据：`npm run golden:gen` 直接从 HEAD 重建，可随时复核。

