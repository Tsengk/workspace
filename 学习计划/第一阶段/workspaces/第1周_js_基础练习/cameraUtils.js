/**
 * cameraUtils — CommonJS 模块
 *
 * 导出相机参数处理工具函数和常量。
 * 被 模块导入导出.js 通过 require() 引用。
 */

"use strict";

// ============================================================
// 命名导出：常量
// ============================================================

const DEFAULT_MODEL = "MEI";

const CAMERA_TYPES = ["standard", "fisheye"];

const DISTORTION_THRESHOLDS = {
  k1_max: 50,
  k2_max: 1000,
  p1_max: 0.5,
  p2_max: 0.5,
};

// ============================================================
// 命名导出：工具函数
// ============================================================

/**
 * 根据内参矩阵 K 计算等效焦距。
 * @param {number[]} K
 * @returns {number}
 */
function computeFocalLength(K) {
  return (K[0] + K[4]) / 2;
}

/**
 * 创建相机配置对象。
 * @param {string} name - 相机名称
 * @param {object} [opts] - 可选参数
 * @returns {object}
 */
function createCamera(name, opts = {}) {
  const {
    width = 1920,
    height = 1080,
    model = DEFAULT_MODEL,
    k1 = 0,
    k2 = 0,
    p1 = 0,
    p2 = 0,
    epsilon = 0,
  } = opts;

  return {
    camera_name: name,
    model_type: model,
    image_width: width,
    image_height: height,
    mirror_parameters: { xi: epsilon },
    distortion_parameters: { k1, k2, p1, p2 },
    created_at: new Date().toISOString(),
  };
}

/**
 * 校验相机畸变参数。
 * @param {object} camera - { name, k1, k2, p1?, p2? }
 * @returns {{ valid: boolean, warnings: string[] }}
 */
function validateCamera(camera) {
  const { k1, k2, p1 = 0, p2 = 0 } = camera;
  const { k1_max, k2_max, p1_max, p2_max } = DISTORTION_THRESHOLDS;
  const warnings = [];

  if (Math.abs(k1) > k1_max)
    warnings.push(`k1=${k1.toFixed(3)} 超出阈值 ${k1_max}`);
  if (Math.abs(k2) > k2_max)
    warnings.push(`k2=${k2.toFixed(3)} 超出阈值 ${k2_max}`);
  if (Math.abs(p1) > p1_max)
    warnings.push(`p1=${p1.toFixed(6)} 超出阈值 ${p1_max}`);
  if (Math.abs(p2) > p2_max)
    warnings.push(`p2=${p2.toFixed(6)} 超出阈值 ${p2_max}`);

  return { valid: warnings.length === 0, warnings };
}

/**
 * 格式化相机列表为可读字符串。
 * @param {object[]} cameras - [{ name, epsilon, enabled }]
 * @returns {string[]}
 */
function formatCameraList(cameras) {
  return cameras.map((cam) => {
    const mark = cam.enabled ? "✓" : "✗";
    return `[${mark}] ${cam.name.padEnd(20)} epsilon=${cam.epsilon.toFixed(3)}`;
  });
}

// ============================================================
// CommonJS 导出
// ============================================================
module.exports = {
  DEFAULT_MODEL,
  CAMERA_TYPES,
  DISTORTION_THRESHOLDS,
  computeFocalLength,
  createCamera,
  validateCamera,
  formatCameraList,
};
