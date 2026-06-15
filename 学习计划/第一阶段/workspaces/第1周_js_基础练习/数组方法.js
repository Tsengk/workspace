#!/usr/bin/env node
/**
 * 练习4：数组方法 — 基于 MRCT / ICAOS_TOOL 前端多相机批量处理场景
 *
 * 目标：
 *   1. 数组基本操作：push/pop/shift/unshift/splice/slice
 *   2. 迭代方法：forEach / map / filter / reduce / find / some / every
 *   3. sort（自定义比较函数）
 *   4. flat / flatMap
 *   5. 数组解构
 *   6. Array.from / Array.of
 */

"use strict";

console.log("=".repeat(50));
console.log("多相机批量处理 — 数组方法");
console.log("=".repeat(50));

// ============================================================
// 初始数据：模拟 ICAOS_TOOL 前端的相机列表
// ============================================================
const cameraList = [
  { name: "front_120",       angle: 120, type: "standard",  epsilon: 1.228, enabled: true },
  { name: "left_front_100",  angle: 100, type: "standard",  epsilon: 3.985, enabled: true },
  { name: "right_front_100", angle: 100, type: "standard",  epsilon: 0.573, enabled: true },
  { name: "left_back_100",   angle: 100, type: "standard",  epsilon: 0.947, enabled: false },
  { name: "right_back_100",  angle: 100, type: "standard",  epsilon: 15.855, enabled: false },
  { name: "back_120",        angle: 120, type: "standard",  epsilon: 7.010, enabled: true },
  { name: "front_60",        angle: 60,  type: "standard",  epsilon: 55.261, enabled: true },
  { name: "left_back_60",    angle: 60,  type: "standard",  epsilon: 0.383, enabled: true },
  { name: "right_back_60",   angle: 60,  type: "standard",  epsilon: 0.060, enabled: true },
  { name: "front_fish_eye",  angle: 0,   type: "fisheye",   epsilon: 1.233, enabled: true },
  { name: "left_fish_eye",   angle: 0,   type: "fisheye",   epsilon: 1.285, enabled: false },
  { name: "right_fish_eye",  angle: 0,   type: "fisheye",   epsilon: 1.295, enabled: false },
  { name: "back_fish_eye",   angle: 0,   type: "fisheye",   epsilon: 1.293, enabled: true },
];

// ============================================================
// 一、基本操作：push / pop / shift / unshift / splice / slice
// ============================================================

console.log(`\n相机总数: ${cameraList.length}`);

// slice — 截取（不修改原数组）
const mainCameras = cameraList.slice(0, 6);
console.log(`  前 6 个主相机: ${mainCameras.map((c) => c.name).join(", ")}`);

// splice — 删除/插入（修改原数组）
const copy = [...cameraList]; // 拷贝一份演示
const removed = copy.splice(6, 3); // 从索引 6 开始删除 3 个
console.log(`  splice 删除的: ${removed.map((c) => c.name).join(", ")}`);

// push — 追加
const newList = [...cameraList];
newList.push({ name: "top_view", angle: 360, type: "special", epsilon: 0.5, enabled: true });
console.log(`  push 后最后一项: ${newList[newList.length - 1].name}`);

// ============================================================
// 二、forEach — 遍历
// ============================================================

console.log("\n--- 已启用相机 ---");
cameraList.forEach((cam) => {
  if (cam.enabled) {
    console.log(`  ✓ ${cam.name.padEnd(18)} epsilon=${cam.epsilon.toFixed(3)}`);
  }
});


// ============================================================
// 三、map — 映射转换
// ============================================================

console.log("\n--- map: 提取名称列表 ---");

const allNames = cameraList.map((cam) => cam.name);
console.log(`  ${allNames.join(" | ")}`);

// map 生成摘要对象
const summaries = cameraList.map((cam) => ({
  label: cam.name,
  info: `${cam.angle}° ${cam.type}`,
  status: cam.enabled ? "启用" : "停用",
}));
console.log("  摘要:");
summaries.forEach((s) => console.log(`    ${s.label.padEnd(18)} ${s.info.padEnd(14)} ${s.status}`));


// ============================================================
// 四、filter — 筛选
// ============================================================

console.log("\n--- filter: 筛选 ---");

// 鱼眼相机
const fishEyeCams = cameraList.filter((cam) => cam.type === "fisheye");
console.log(`  鱼眼相机: ${fishEyeCams.map((c) => c.name).join(", ")}`);

// epsilon > 5 的异常相机
const highEpsilon = cameraList.filter((cam) => cam.epsilon > 5);
console.log(`  高 epsilon (>5): ${highEpsilon.map((c) => `${c.name}(${c.epsilon.toFixed(1)})`).join(", ")}`);

// 已启用 且 标准相机
const enabledStandard = cameraList.filter((cam) => cam.enabled && cam.type === "standard");
console.log(`  已启用标准相机: ${enabledStandard.length} 个`);


// ============================================================
// 五、reduce — 聚合
// ============================================================

console.log("\n--- reduce: 聚合统计 ---");

// 总 epsilon 和
const totalEpsilon = cameraList.reduce((sum, cam) => sum + cam.epsilon, 0);
console.log(`  总 epsilon: ${totalEpsilon.toFixed(3)}, 平均: ${(totalEpsilon / cameraList.length).toFixed(3)}`);

// 按类型分组
const groupByType = cameraList.reduce((acc, cam) => {
  if (!acc[cam.type]) acc[cam.type] = [];
  acc[cam.type].push(cam.name);
  return acc;
}, {});
console.log("  按类型分组:", groupByType);

// 按角度统计数量
const angleCounts = cameraList.reduce((acc, cam) => {
  const key = cam.type === "fisheye" ? "鱼眼" : `${cam.angle}°`;
  acc[key] = (acc[key] || 0) + 1;
  return acc;
}, {});
console.log("  按角度统计:", angleCounts);


// ============================================================
// 六、find / findIndex / some / every
// ============================================================

console.log("\n--- find / some / every ---");

// find — 找第一个鱼眼
const firstFish = cameraList.find((cam) => cam.type === "fisheye");
console.log(`  第一个鱼眼: ${firstFish.name}`);

// findIndex — 找索引
const idx = cameraList.findIndex((cam) => cam.name === "back_120");
console.log(`  back_120 索引: ${idx}`);

// some — 是否存在满足条件的
const hasDisabled = cameraList.some((cam) => !cam.enabled);
console.log(`  有停用相机: ${hasDisabled}`);

// every — 是否全部满足
const allStandard = cameraList.every((cam) => cam.type === "standard");
console.log(`  全是标准相机: ${allStandard}`);


// ============================================================
// 七、sort — 排序
// ============================================================

console.log("\n--- sort: 按 epsilon 降序 ---");

const sorted = [...cameraList].sort((a, b) => b.epsilon - a.epsilon);
sorted.slice(0, 5).forEach((cam) => {
  console.log(`  ${cam.name.padEnd(18)} epsilon=${cam.epsilon.toFixed(3)}`);
});

// 多级排序：先类型（标准在前），再 epsilon 降序
console.log("\n--- 多级排序：类型 + epsilon ---");
const multiSorted = [...cameraList].sort((a, b) => {
  if (a.type !== b.type) return a.type === "standard" ? -1 : 1;
  return b.epsilon - a.epsilon;
});
multiSorted.forEach((cam) => {
  console.log(`  ${cam.name.padEnd(18)} ${cam.type.padEnd(8)} epsilon=${cam.epsilon.toFixed(3)}`);
});


// ============================================================
// 八、flat / flatMap
// ============================================================

console.log("\n--- flat / flatMap ---");

// 模拟每个相机对应的 ROI 列表
const camerasWithROIs = [
  { name: "front_120", rois: [[0, 0, 640, 360], [640, 0, 1280, 360]] },
  { name: "left_front_100", rois: [[0, 360, 640, 720]] },
  { name: "front_fish_eye", rois: [[460, 0, 900, 550]] },
];

// flatMap: 展开嵌套数组
const allROIs = camerasWithROIs.flatMap((cam) => cam.rois);
console.log(`  所有 ROI (flatMap):`);
allROIs.forEach((roi, i) => console.log(`    ROI ${i + 1}: [${roi.join(", ")}]`));


// ============================================================
// 九、数组解构
// ============================================================

console.log("\n--- 数组解构 ---");

const [first, second, third, ...rest] = cameraList;
console.log(`  前三个: ${first.name}, ${second.name}, ${third.name}`);
console.log(`  剩余: ${rest.length} 个`);

// 交换值
let a = "front_120", b = "back_120";
[a, b] = [b, a];
console.log(`  交换后: a=${a}, b=${b}`);


// ============================================================
// 十、Array.from / Array.of
// ============================================================

console.log("\n--- Array.from / Array.of ---");

// Array.from — 从 Set 创建数组并映射
const uniqueAngles = new Set(cameraList.map((c) => c.angle));
const angleLabels = Array.from(uniqueAngles, (a) => (a === 0 ? "鱼眼" : `${a}°`));
console.log(`  唯一角度: ${angleLabels.join(", ")}`);

// Array.of — 保证创建数组
console.log(`  Array.of(1, 2, 3): ${JSON.stringify(Array.of(1, 2, 3))}`);
// 对比 Array 构造函数
console.log(`  Array(3): ${JSON.stringify(Array(3))} (稀疏数组)`);
console.log(`  Array.of(3): ${JSON.stringify(Array.of(3))} (单元素)`);
