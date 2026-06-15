#!/usr/bin/env node
"use strict";
const CAMERA_NAME = "front_120";
const IMAGE_SIZE = {
  width: 3840,
  height: 2160,
};
const DISTORTION = [-0.4540382069750924, 0.21425673994294858, -0.00023703759012878812, 0.00005439639596613181];

console.log("=".repeat(50));
console.log("相机标定参数 — 变量声明");
console.log("=".repeat(50));


console.log(`\n相机名称: ${CAMERA_NAME}`);
console.log(`图像尺寸: ${IMAGE_SIZE.width} x ${IMAGE_SIZE.height}`);
console.log(`畸变参数 [k1,k2,p1,p2]: ${DISTORTION}`);


console.log(`\n----let块级作用域----`);

let epsilon = 1.2222222222;
console.log(`初始epsilon: ${epsilon}`);

if (epsilon > 1) {
  let message = "epsilon is greater than 1";//该变量只在if块内有效
  console.log(message);
};

epsilon = 0.5;
console.log(`更新后的epsilon: ${epsilon}`);


console.log(`\n----var变量提升----`);

function demoVarHoistiong() {
  console.log (`提升前status: ${status}`);
  var status = "var变量提升";
  console.log (`提升后status: ${status}`);
}
demoVarHoistiong();

console.log(`\n----基础类型----`);

const strVar = "MEI";
const numVar = 123;
const boolVar = true;
const nullVar = null;
const undefinedVar = undefined;
const bigintVar = 123n;

console.log(`\n基础类型变量值:`);
console.log(`字符串: ${strVar}`);
console.log(`数字: ${numVar}`);
console.log(`布尔值: ${boolVar}`);
console.log(`null: ${nullVar}`);
console.log(`undefined: ${undefinedVar}`);
console.log(`BigInt: ${bigintVar}`);

