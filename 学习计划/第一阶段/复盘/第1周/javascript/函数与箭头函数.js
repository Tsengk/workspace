#!/usr/bin/env node
"use strict"

console.log("=".repeat(50));
console.log("函数与箭头函数");
console.log("=".repeat(50));

console.log("\n---标准函数---");
function computerFocalLength(K) {
    const fx = K[0];
    const fy = K[4];
    return (fx + fy) / 2;
}

const K = [4238.609, 0, 1926.572, 0, 4224.351, 1088.389, 0, 0, 1]
console.log(`\n焦距:${computerFocalLength(K).toFixed(2)}`);

console.log("\n---函数表达式---");
const computePrincioalPoint = function (K) {
    return { cx: K[2], cy: K[5]};
};

const pp = computePrincioalPoint(K);
console.log(`\n主点: cx=${pp.cx.toFixed(2)}, cy=${pp.cy.toFixed(2)}`);

console.log(`\n---箭头函数---`);
const square = (x) => x * x;

const distortionMagnitude = (k1, k2, p1, p2) => 
    Math.sqrt(square(k1) + square(k2) + square(p1) + square(p2));
const mag = distortionMagnitude(-0.454, 0.214, 0, 0);
console.log(`\n畸变幅度: ${mag.toFixed(2)}`);

console.log(`\n---默认参数---`);

function checkDistortion(k1, k2, k1Max = 50,k2Max = 1000) {
    const results = [];
    if (Math.abs(k1) > k1Max) results.push(`k1=${k1} 超出阈值${k1Max}`);
    if (Math.abs(k2) > k2Max) results.push(`k2=${k2} 超出阈值${k2Max}`);
    if (Math.abs(k1) <= k1Max) results.push(`k1=${k1} 在阈值内`);
    if (Math.abs(k2) <= k2Max) results.push(`k2=${k2} 在阈值内`);
    return results;
}

console.log("\n---畸变检查---");
console.log(`front_120:${checkDistortion(-0.454, 0.214)}`);
console.log(`front_60:${checkDistortion(67.068, -1302046.68)}`)


console.log(`\n---剩余参数---`);
function averageEpsilon(...epsilons) {
    if (epsilons.length === 0) return 0;
    const sum = epsilons.reduce((acc, val) => acc + val, 0);
    return sum / epsilons.length;
}
console.log(`\n平均误差: ${averageEpsilon(1, 2, 3, 4, 5).toFixed(2)}`);

console.log(`\n---回调函数---`);

const cameras = [
    { name: "front_120" , epsilon: 1.228},
    { name: "front_60" , epsilon: 2.345},
    { name: "front_fish_eye" , epsilon: 1.567},
];
//对每个相机执行回调
function forEachCamera(list, callback) {
    for (const cam of list) {
        callback(cam);
    }
}
//传入箭头函数作为回调
forEachCamera(cameras, (cam) => {
    const type = cam.epsilon > 3 ? "高畸变" : "低畸变";
    console.log(`${cam.name.padEnd(20)} epsilon= ${cam.epsilon.toFixed(2)} -> ${type}`);
});

console.log("\n---闭包---");
function createDistortionValidator(k1Max, k2Max) {
    return function(k1, k2) {
        return Math.abs(k1) <= k1Max && Math.abs(k2) <= k2Max;
    };
}

const strictValidator = createDistortionValidator(10, 100);
const looseValidator = createDistortionValidator(100, 2000);

console.log(`front_120 严格验证: ${strictValidator(-0.454, 0.214)}`);
console.log(`front_120 宽松验证: ${looseValidator(67.068, 1000.123)}`);

console.log("\n---箭头函数 this---");

const cameraConfig = {
    name: "front_120",
    epsilons: [1.228, 0.454, 0.214],

    //普通函数：this指向调用者
    averageNormal: function() {
        const self = this;
        return this.epsilons.reduce(function(acc, val) {
            return acc + val;
        },0) / this.epsilons.length;
    },
    //箭头函数：this继承自外层作用域
    averageArrow: function () {
        return this.epsilons.reduce((acc, val) => acc + val, 0) / this.epsilons.length;
    },
};

console.log(`普通函数平均误差: ${cameraConfig.averageNormal().toFixed(2)}`);
console.log(`箭头函数平均误差: ${cameraConfig.averageArrow().toFixed(2)}`);