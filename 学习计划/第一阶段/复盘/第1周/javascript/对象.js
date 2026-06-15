#!/usr/bin/env node
"use strict";

console.log("=".repeat(50));
console.log("对象相关操作");
console.log("=".repeat(50));

console.log(`\n---对象字面量与属性访问---`);

const cameraConfig = {
    camera_name: "front_120",
    model_type: "MEI",
    image_width: 3840,
    image_height: 2160,
    mirror_parameters: {
        xi: 1.22222222,
    },
    distortion_parameters: {
        k1: -0.4540382069750924,
        k2: 0.21425673994294858,
        p1: -0.00023703759012878812,
        p2: 0.00005439639596613181,
    },
    projection_parameters: {
        gamma1: 4238.66666666,
        gamma2: 4238.66666666,
        u0: 1926.57777,
        v0: 1080.57777,
    },
    ["bev_" + "roi"]: null,
};

console.log(`相机: ${cameraConfig.camera_name}`);
console.log(`模型：${cameraConfig.model_type}`);

console.log(`k1: ${cameraConfig["distortion_parameters"].k1}`);
console.log(`k2: ${cameraConfig["distortion_parameters"].k2}`);
console.log(`p1: ${cameraConfig["distortion_parameters"].p1}`);
console.log(`p2: ${cameraConfig["distortion_parameters"].p2}`);

const parameters = "distortion_parameters";
console.log(`k1: ${cameraConfig[parameters].k1}`);
console.log(`k2: ${cameraConfig[parameters].k2}`);
console.log(`p1: ${cameraConfig[parameters].p1}`);
console.log(`p2: ${cameraConfig[parameters].p2}`);

console.log(`\n---属性增删改查---`);

//增
cameraConfig.location = "front";
cameraConfig.calibration_date = "2023-10-01";

//查
console.log(`location: ${"location" in cameraConfig}`);
console.log(`calibration_date: ${cameraConfig.hasOwnProperty("calibration_date")}`);

//改
cameraConfig.image_width = 1920;

//删
delete cameraConfig.bev_roi;
console.log(`bev_roi: ${"bev_roi" in cameraConfig}`);

console.log(`\n---对象keys/values/entries---`);

console.log(`keys: ${Object.keys(cameraConfig)}`);

//只取非嵌套的标量值
const scalars = Object.entries(cameraConfig)
    .filter(([,v]) => typeof v !== "object" || v === null)
    .map(([k, v]) => `${k}=${v}`);
console.log("顶层标量:",scalars);


console.log(`\n---解构---`);
const { camera_name, model_type, image_width, image_height } = cameraConfig;
console.log(`  解构: ${camera_name} | ${model_type} | ${image_width}x${image_height}`);

const {
    distortion_parameters: {k1, k2, p1, p2},
    projection_parameters: {gamma1, gamma2, u0, v0},
} = cameraConfig;

console.log(`畸变：k1=${k1.toFixed(4)}, k2=${k2.toFixed(4)}, p1=${p1.toFixed(4)}, p2=${p2.toFixed(4)}`);
console.log(`投影：gamma1=${gamma1.toFixed(2)}, gamma2=${gamma2.toFixed(2)}, u0=${u0.toFixed(2)}, v0=${v0.toFixed(2)}`);

const { calibration_date = "unknown" } = cameraConfig;
console.log(`校准日期: ${calibration_date}`);

console.log(`\n---展开运算符---`);

const thumbnailConfig = {
    ...cameraConfig,
    image_width: 640,
    image_height: 360,
    is_thumbnail: true,
};
console.log(`缩略图配置: ${thumbnailConfig.image_width}x${thumbnailConfig.image_height}, is_thumbnail: ${thumbnailConfig.is_thumbnail}`);

const runtimeParams = { exposure:800, gain: 1.2};
const merged = { ...cameraConfig, ...runtimeParams};
console.log(`合并后的配置: exposure: ${merged.exposure}, gain: ${merged.gain}`);

console.log(`\n---Object.assign vs 深拷贝---`);

const shallow = Object.assign({}, cameraConfig);
shallow.distortion_parameters.k1 = 999;
console.log(`浅拷贝修改后: k1=${shallow.distortion_parameters.k1}`);
console.log(`原对象修改后: k1=${cameraConfig.distortion_parameters.k1}`);

const deep = JSON.parse(JSON.stringify(cameraConfig));
deep.distortion_parameters.k1 = -999;
console.log(`深拷贝修改后: k1=${deep.distortion_parameters.k1}`);
console.log(`原对象修改后: k1=${cameraConfig.distortion_parameters.k1}`);


console.log(`\n---可选链---`);