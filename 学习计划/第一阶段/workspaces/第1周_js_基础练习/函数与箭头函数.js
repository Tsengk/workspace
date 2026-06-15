#!/usr/bin/env node
/**
 * 练习2：函数与箭头函数 — 基于 MRCT / ICAOS_TOOL 前端参数计算场景
 *
 * 目标：
 *   1. 函数声明 vs 函数表达式
 *   2. 箭头函数与 this 绑定
 *   3. 默认参数、剩余参数
 *   4. 回调函数
 *   5. 闭包
 */

"use strict";

console.log("=".repeat(50));
console.log("相机参数计算 — 函数与箭头函数");
console.log("=".repeat(50));

// ============================================================
// 一、函数声明 — 标准函数
// ============================================================

/**
 * 根据内参矩阵 K 计算等效焦距。
 * K 格式: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
 * @param {number[]} K - 相机内参矩阵
 * @returns {number} 等效焦距
 */
function computeFocalLength(K) {
  const fx = K[0];
  const fy = K[4]; // 索引 4 是 fy
  return (fx + fy) / 2;
}

const K = [4238.609, 0, 1926.572, 0, 4224.351, 1088.389, 0, 0, 1];
console.log(`\n等效焦距: ${computeFocalLength(K).toFixed(2)}`);


// ============================================================
// 二、函数表达式 — 匿名函数赋给变量
// ============================================================

const computePrincipalPoint = function (K) {
  return { cx: K[2], cy: K[5] };
};

const pp = computePrincipalPoint(K);
console.log(`主点: cx=${pp.cx.toFixed(2)}, cy=${pp.cy.toFixed(2)}`);


// ============================================================
// 三、箭头函数
// ============================================================

// 短小箭头函数
const square = (x) => x * x;

// 计算畸变幅度
const distortionMagnitude = (k1, k2, p1, p2) =>
  Math.sqrt(square(k1) + square(k2) + square(p1) + square(p2));

const mag = distortionMagnitude(-0.454, 0.214, -0.000237, 0.0000544);
console.log(`\n畸变幅度: ${mag.toFixed(6)}`);


// ============================================================
// 四、默认参数
// ============================================================

/**
 * 校验畸变参数是否在阈值内。
 * @param {number} k1
 * @param {number} k2
 * @param {number} [k1Max=50] - k1 阈值
 * @param {number} [k2Max=1000] - k2 阈值
 * @returns {string[]} 警告列表
 */
function checkDistortion(k1, k2, k1Max = 50, k2Max = 1000) {
  const warnings = [];
  if (Math.abs(k1) > k1Max) warnings.push(`k1=${k1} 超出阈值 ${k1Max}`);
  if (Math.abs(k2) > k2Max) warnings.push(`k2=${k2} 超出阈值 ${k2Max}`);
  return warnings;
}

// 使用默认阈值
console.log("\n--- 畸变校验 ---");
console.log(`  front_120: ${checkDistortion(-0.454, 0.214)}`);
console.log(`  front_60:  ${checkDistortion(67.068, -1302046.68)}`);
// 自定义阈值
console.log(`  front_60(宽松): ${checkDistortion(67.068, -1302046.68, 100, 2000000)}`);


// ============================================================
// 五、剩余参数 (rest parameters)
// ============================================================

function averageEpsilon(...epsilons) {
  if (epsilons.length === 0) return 0;
  const sum = epsilons.reduce((acc, val) => acc + val, 0);
  return sum / epsilons.length;
}

const avgEps = averageEpsilon(1.228, 3.985, 0.573, 0.947, 15.855, 7.010);
console.log(`\n平均 epsilon: ${avgEps.toFixed(4)}`);


// ============================================================
// 六、回调函数 — 对相机列表执行统一操作
// ============================================================

console.log("\n--- 回调：格式化相机列表 ---");

const cameras = [
  { name: "front_120", epsilon: 1.228 },
  { name: "left_front_100", epsilon: 3.985 },
  { name: "front_fish_eye", epsilon: 1.233 },
];

/**
 * 对每个相机执行回调
 * @param {object[]} list
 * @param {function} callback
 */
function forEachCamera(list, callback) {
  for (const cam of list) {
    callback(cam);
  }
}

// 传入箭头函数作为回调
forEachCamera(cameras, (cam) => {
  const type = cam.epsilon > 3 ? "高畸变" : "正常";
  console.log(`  ${cam.name.padEnd(20)} epsilon=${cam.epsilon.toFixed(3)} -> ${type}`);
});


// ============================================================
// 七、闭包 — 创建带上下文的参数校验器
// ============================================================

console.log("\n--- 闭包：参数校验器工厂 ---");

function createDistortionValidator(k1Max, k2Max) {
  // 闭包捕获 k1Max 和 k2Max
  return function (k1, k2) {
    return Math.abs(k1) <= k1Max && Math.abs(k2) <= k2Max;
  };
}

const strictValidator = createDistortionValidator(10, 100);
const looseValidator = createDistortionValidator(100, 2000000);

console.log(`  front_120 严格校验: ${strictValidator(-0.454, 0.214)}`);
console.log(`  front_60  严格校验: ${strictValidator(67.068, -1302046.68)}`);
console.log(`  front_60  宽松校验: ${looseValidator(67.068, -1302046.68)}`);


// ============================================================
// 八、箭头函数的 this 绑定 — 对象方法场景
// ============================================================

console.log("\n--- 箭头函数 this ---");

const cameraConfig = {
  name: "front_120",
  epsilons: [1.228, 1.5, 1.3],

  // 普通函数：this 指向调用者
  averageNormal: function () {
    const self = this;
    return this.epsilons.reduce(function (acc, v) {
      return acc + v;
    }, 0) / this.epsilons.length;
  },

  // 箭头函数：this 继承自外层作用域
  averageArrow: function () {
    return this.epsilons.reduce((acc, v) => acc + v, 0) / this.epsilons.length;
  },
};

console.log(`  普通函数: ${cameraConfig.averageNormal().toFixed(4)}`);
console.log(`  箭头函数: ${cameraConfig.averageArrow().toFixed(4)}`);
