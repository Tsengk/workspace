/**
 * cameraUtils — ESM 模块 (.mjs)
 *
 * 与 cameraUtils.js 功能相同，但使用 ES Module 语法。
 * 被 模块导入导出.js 通过动态 import() 引用。
 */

// ============================================================
// 命名导出：常量
// ============================================================

export const MODEL_TYPE = "MEI";

export const SUPPORTED_ANGLES = ["60", "100", "120", "fisheye"];

// ============================================================
// 命名导出：函数
// ============================================================

/**
 * 从相机名称中提取角度信息。
 * @param {string} name
 * @returns {string}
 */
export function extractAngle(name) {
  if (name.includes("fish_eye")) return "鱼眼";
  const match = name.match(/(\d+)/);
  return match ? `${match[1]}°` : "未知";
}

// ============================================================
// 默认导出
// ============================================================

const defaultConfig = {
  model_type: "MEI",
  image_width: 1920,
  image_height: 1080,
};

export default defaultConfig;
