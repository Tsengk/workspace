#!/usr/bin/env node
/**
 * 练习1：变量声明 — 基于 MRCT / ICAOS_TOOL 前端相机配置场景
 *
 * 目标：
 *   1. 掌握 var / let / const 的区别
 *   2. 理解块级作用域与变量提升
 *   3. 基本类型：number, string, boolean, undefined, null
 */

"use strict";

// ============================================================
// 一、const — 不可重新赋值（对象属性仍可变）
// ============================================================

// 相机名称常量
const CAMERA_NAME = "front_120";

// 图像尺寸（const 声明的对象，属性可以修改）
const IMAGE_SIZE = {
  width: 3840,
  height: 2160,
};

// 畸变参数（数组也可以用 const — 元素可以修改）
const DISTORTION = [-0.4540382069750924, 0.21425673994294858, -0.00023703759012878812, 0.00005439639596613181];

console.log("=".repeat(50));
console.log("相机标定参数 — 变量声明");
console.log("=".repeat(50));

console.log(`\n相机名称: ${CAMERA_NAME}`);
console.log(`图像尺寸: ${IMAGE_SIZE.width} x ${IMAGE_SIZE.height}`);
console.log(`畸变参数 [k1, k2, p1, p2]: ${DISTORTION}`);


// ============================================================
// 二、let — 块级作用域，可重新赋值
// ============================================================

console.log("\n--- let 块级作用域 ---");

let epsilon = 1.227754354466145; // MEI 模型的 xi 参数
console.log(`初始 epsilon: ${epsilon}`);

// 根据阈值调整
if (epsilon > 2.0) {
  let message = "高畸变相机"; // 此变量只在 if 块内有效
  console.log(`  判定: ${message}`);
}
// console.log(message); // ReferenceError — message 在块外不可访问

epsilon = 1.5; // let 允许重新赋值
console.log(`调整后 epsilon: ${epsilon}`);


// ============================================================
// 三、var — 函数级作用域，存在变量提升
// ============================================================

console.log("\n--- var 变量提升 ---");

function demoVarHoisting() {
  // var 声明会被"提升"到函数顶部，但赋值为 undefined
  console.log(`  提升前 status: ${status}`); // undefined，不报错

  var status = "calibrated"; // 声明提升，赋值留在原地

  console.log(`  赋值后 status: ${status}`);
}
demoVarHoisting();


// ============================================================
// 四、类型对比
// ============================================================

console.log("\n--- 基本类型 ---");

const strVal = "MEI";                    // string
const numVal = 4238.609;                 // number
const boolVal = true;                    // boolean
const undefinedVal = undefined;          // undefined
const nullVal = null;                    // null
const bigintVal = 3840n;                 // bigint (ES2020)

console.log(`  ${strVal}       -> ${typeof strVal}`);
console.log(`  ${numVal}    -> ${typeof numVal}`);
console.log(`  ${boolVal}        -> ${typeof boolVal}`);
console.log(`  ${undefinedVal} -> ${typeof undefinedVal}`);
console.log(`  ${nullVal}        -> ${typeof nullVal}`);       // "object" — 历史遗留 bug
console.log(`  ${bigintVal}       -> ${typeof bigintVal}`);


// ============================================================
// 五、模板字符串 — 汇总输出
// ============================================================

console.log("\n--- 汇总 ---");

const summary = `
  相机: ${CAMERA_NAME}
  模型: ${strVal}
  分辨率: ${IMAGE_SIZE.width} x ${IMAGE_SIZE.height}
  epsilon: ${epsilon}
  k1: ${DISTORTION[0]}
  状态: 已标定
`;
console.log(summary.trim());
