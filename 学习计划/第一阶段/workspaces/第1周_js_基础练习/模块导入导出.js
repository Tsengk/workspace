#!/usr/bin/env node
/**
 * 练习5：模块导入导出 — 基于 MRCT / ICAOS_TOOL 前端模块化场景
 *
 * 目标：
 *   1. CommonJS: module.exports / require
 *   2. ESM: export / import（.mjs 文件）
 *   3. 命名导出 vs 默认导出
 *   4. 动态导入 import()
 *
 * 场景：将相机参数处理逻辑拆分为独立模块，主入口负责编排调用。
 */

// ============================================================
// 方式一：CommonJS（require / module.exports）
// 这是 Node.js 默认模块系统，直接运行即可
// ============================================================

const cameraModule = require("./cameraUtils");

console.log("=".repeat(50));
console.log("模块导入导出 — CommonJS 版");
console.log("=".repeat(50));

// 从模块获取数据
console.log(`\n支持的相机类型: ${cameraModule.CAMERA_TYPES.join(", ")}`);
console.log(`默认模型: ${cameraModule.DEFAULT_MODEL}`);

// 调用模块函数
const sampleK = [4238.609, 0, 1926.572, 0, 4224.351, 1088.389, 0, 0, 1];
const focal = cameraModule.computeFocalLength(sampleK);
console.log(`等效焦距: ${focal.toFixed(2)}`);

// 创建相机配置
const config = cameraModule.createCamera("front_120", {
  width: 3840,
  height: 2160,
  k1: -0.454,
  k2: 0.214,
});
console.log("\n生成的相机配置:");
console.log(JSON.stringify(config, null, 2));

// 批量校验
console.log("\n--- 批量校验 ---");
const testCameras = [
  { name: "front_120", k1: -0.454, k2: 0.214 },
  { name: "front_60", k1: 67.068, k2: -1302046.68 },
];
testCameras.forEach((cam) => {
  const result = cameraModule.validateCamera(cam);
  console.log(`  ${cam.name}: ${result.valid ? "有效" : "无效"} — ${result.warnings.join("; ") || "无警告"}`);
});

// 格式化相机列表
console.log("\n--- 格式化输出 ---");
const formatted = cameraModule.formatCameraList([
  { name: "front_120", epsilon: 1.228, enabled: true },
  { name: "left_fish_eye", epsilon: 1.285, enabled: false },
]);
formatted.forEach((s) => console.log(`  ${s}`));


// ============================================================
// 方式二：动态导入（ESM import() — 返回 Promise）
// 下面演示动态导入 .mjs 文件
// ============================================================

console.log("\n--- 动态导入 ESM 模块 ---");

// 动态 import() 是异步的，返回 Promise
async function demoDynamicImport() {
  try {
    // 尝试导入 ESM 模块（如果文件存在）
    const esmModule = await import("./cameraUtils.mjs");
    console.log("  ESM 模块加载成功");
    console.log(`  常量: ${esmModule.MODEL_TYPE}`);
  } catch (err) {
    console.log("  (ESM 模块演示: cameraUtils.mjs 需要以 .mjs 扩展名单独创建)");
    console.log(`  提示: ${err.message.slice(0, 50)}...`);
  }
}

// 执行异步演示
demoDynamicImport().then(() => {
  console.log("\n" + "=".repeat(50));
  console.log("练习完成 — 模块化小结:");
  console.log("  CommonJS: module.exports + require()   — Node.js 默认");
  console.log("  ESM:      export + import              — 现代标准 (.mjs / type:module)");
  console.log("  动态导入: import()                     — 按需加载，返回 Promise");
  console.log("=".repeat(50));
});
