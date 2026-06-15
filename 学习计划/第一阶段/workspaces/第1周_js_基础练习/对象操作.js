#!/usr/bin/env node
/**
 * 练习3：对象操作 — 基于 MRCT / ICAOS_TOOL 前端配置对象场景
 *
 * 目标：
 *   1. 对象字面量、属性访问（. 和 []）
 *   2. 属性增删改查
 *   3. Object.keys / values / entries
 *   4. 解构赋值
 *   5. 展开运算符 ...
 *   6. Object.assign / 深拷贝
 *   7. 可选链 ?. 与空值合并 ??
 */

"use strict";

console.log("=".repeat(50));
console.log("相机配置对象 — 对象操作");
console.log("=".repeat(50));

// ============================================================
// 一、对象字面量与属性访问
// ============================================================

const cameraConfig = {
  camera_name: "front_120",
  model_type: "MEI",
  image_width: 3840,
  image_height: 2160,
  mirror_parameters: {
    xi: 1.227754354466145,
  },
  distortion_parameters: {
    k1: -0.4540382069750924,
    k2: 0.21425673994294858,
    p1: -0.00023703759012878812,
    p2: 0.00005439639596613181,
  },
  projection_parameters: {
    gamma1: 4238.609331970091,
    gamma2: 4224.3512966406915,
    u0: 1926.5715847651911,
    v0: 1088.3891957199476,
  },
  // 计算属性名
  ["bev_" + "roi"]: null,
};

// 点号访问
console.log(`相机: ${cameraConfig.camera_name}`);
console.log(`模型: ${cameraConfig.model_type}`);

// 方括号访问（动态键名）
const paramSet = "distortion_parameters";
console.log(`k1: ${cameraConfig[paramSet].k1}`);
console.log(`xi: ${cameraConfig["mirror_parameters"].xi}`);


// ============================================================
// 二、属性增删改查
// ============================================================

console.log("\n--- CRUD ---");

// 增
cameraConfig.location = "front";
cameraConfig.calibration_date = "2024-01-15";

// 查（in 运算符 和 hasOwnProperty）
console.log(`  有 location 属性: ${"location" in cameraConfig}`);
console.log(`  有 calibration_date: ${cameraConfig.hasOwnProperty("calibration_date")}`);

// 改
cameraConfig.image_width = 1920; // 缩略图模式

// 删
delete cameraConfig.bev_roi;
console.log(`  bev_roi 已删除: ${!("bev_roi" in cameraConfig)}`);


// ============================================================
// 三、Object.keys / values / entries
// ============================================================

console.log("\n--- 遍历 ---");

console.log("  顶层键:", Object.keys(cameraConfig));

// 只取非嵌套的标量值
const scalars = Object.entries(cameraConfig)
  .filter(([, v]) => typeof v !== "object" || v === null)
  .map(([k, v]) => `${k}=${v}`);
console.log("  顶层标量:", scalars);


// ============================================================
// 四、解构赋值
// ============================================================

console.log("\n--- 解构 ---");

// 对象解构
const { camera_name, model_type, image_width, image_height } = cameraConfig;
console.log(`  解构: ${camera_name} | ${model_type} | ${image_width}x${image_height}`);

// 嵌套解构 + 重命名
const {
  distortion_parameters: { k1, k2, p1, p2 },
  projection_parameters: { gamma1: fx, gamma2: fy, u0: cx, v0: cy },
} = cameraConfig;

console.log(`  畸变: k1=${k1.toFixed(4)} k2=${k2.toFixed(4)} p1=${p1.toFixed(6)} p2=${p2.toFixed(6)}`);
console.log(`  投影: fx=${fx.toFixed(2)} fy=${fy.toFixed(2)} cx=${cx.toFixed(2)} cy=${cy.toFixed(2)}`);

// 带默认值的解构
const { calibration_date = "unknown" } = cameraConfig;
console.log(`  标定日期: ${calibration_date}`);


// ============================================================
// 五、展开运算符
// ============================================================

console.log("\n--- 展开运算符 ---");

// 浅拷贝并覆盖属性
const thumbnailConfig = {
  ...cameraConfig,
  image_width: 640,
  image_height: 360,
  is_thumbnail: true,
};
console.log(`  缩略图: ${thumbnailConfig.image_width}x${thumbnailConfig.image_height}, thumbnail=${thumbnailConfig.is_thumbnail}`);

// 合并对象
const runtimeParams = { exposure: 800, gain: 1.2 };
const merged = { ...cameraConfig, ...runtimeParams };
console.log(`  合并后: exposure=${merged.exposure}, gain=${merged.gain}`);


// ============================================================
// 六、Object.assign vs 深拷贝
// ============================================================

console.log("\n--- 拷贝 ---");

// 浅拷贝（Object.assign）
const shallow = Object.assign({}, cameraConfig);
shallow.distortion_parameters.k1 = 999; // 会影响原对象！

console.log(`  原对象 k1: ${cameraConfig.distortion_parameters.k1} (被浅拷贝修改了)`);

// 深拷贝（structuredClone — Node 17+, 或用 JSON 方法）
const deep = JSON.parse(JSON.stringify(cameraConfig));
deep.distortion_parameters.k1 = -999;
console.log(`  原对象 k1: ${cameraConfig.distortion_parameters.k1} (深拷贝不影响)`);


// ============================================================
// 七、可选链 ?. — 安全访问深层属性
// ============================================================

console.log("\n--- 可选链 ---");

// 安全访问不存在的属性
const missingKey = cameraConfig.calibration_params?.some_field;
console.log(`  不存在的路径: ${missingKey}`); // undefined，不报错

// 安全访问存在路径
const xiSafe = cameraConfig.mirror_parameters?.xi ?? "N/A";
console.log(`  xi: ${xiSafe}`);

// 嵌套可选链
const nonexistentParam = cameraConfig.extra?.sub_field?.value ?? "无此参数";
console.log(`  extra.sub_field.value: ${nonexistentParam}`);


// ============================================================
// 八、实用：批量生成配置对象
// ============================================================

console.log("\n--- 工厂函数 ---");

function createCameraConfig(name, width = 1920, height = 1080) {
  return {
    camera_name: name,
    image_width: width,
    image_height: height,
    model_type: "MEI",
    created_at: new Date().toISOString(),
  };
}

const cams = ["front_120", "left_front_100", "front_fish_eye"].map((name) =>
  createCameraConfig(name, name.includes("fish") ? 1920 : 3840, name.includes("fish") ? 1280 : 2160)
);

cams.forEach((c) => {
  console.log(`  ${c.camera_name}: ${c.image_width}x${c.image_height} @ ${c.created_at.slice(0, 10)}`);
});
