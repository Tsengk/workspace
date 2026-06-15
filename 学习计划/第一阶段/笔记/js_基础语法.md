# 学习笔记：JavaScript 基础语法

> 关联输出物模板：[输出物模板.md#1.2](./输出物模板.md)
> 覆盖知识点：变量声明、函数定义与箭头函数、对象操作、数组方法、模块导入导出

---

## 笔记

**关键字**
| 类别 | 关键字 | 说明 |
|-----|--------|-------|
| 变量声明 | var / let / const | 变量声明方式 |
| 常量 | const | 声明常量，不可重新赋值 |
| 逻辑值 | true / false / null / undefined | 布尔真值/假值/空值/未定义 |
| 条件控制 | if / else / switch / case / break / default | 条件判断与分支控制 |
| 循环控制 | for / while / do / continue / break | 循环语句 |
| 函数定义 | function / return | 定义函数/返回值 |
| 异常处理 | try / catch / finally / throw | 异常捕获与抛出 |
| 模块 | export / import / from / as / default | ES6 模块导入导出 |
| 类与对象 | class / extends / super / new / this | 类定义与继承 |
| 操作相关 | typeof / instanceof / delete / void | 类型检查/删除/运算 |
| 异步 | async / await / yield | 异步编程 |
| 其他 | in / of | `for...in` 遍历键，`for...of` 遍历值 |

---

**变量与声明**

| 特性 | var | let | const |
|------|-----|-----|-------|
| 作用域 | 函数作用域 | 块级作用域 `{}` | 块级作用域 `{}` |
| 变量提升 | 声明提升，初始值 `undefined` | 声明提升但不会初始化（暂时性死区） | 同 let |
| 重复声明 | ✅ 允许 | ❌ 不允许 | ❌ 不允许 |
| 重新赋值 | ✅ 允许 | ✅ 允许 | ❌ 不允许 |
| 全局声明 | 会成为 `window` 的属性 | 不会成为 `window` 的属性 | 不会成为 `window` 的属性 |

```javascript
// 变量提升现象
console.log(a);  // undefined（不会报错）
var a = 1;

console.log(b);  // ReferenceError: Cannot access 'b' before initialization
let b = 2;

// const 必须初始化且不可重新赋值
const PI = 3.14;
// PI = 3;  // ❌ TypeError

// const 对象属性可变（引用不变）
const user = { name: "Alice" };
user.name = "Bob";  // ✅ 允许修改属性
// user = {};       // ❌ 不允许重新赋值
```

**变量命名规则**
- 只能包含字母、数字、`$` 和 `_`
- 不能以数字开头
- 区分大小写
- 不能使用保留关键字
- 推荐驼峰命名法：`userName`, `getData`

---

**数据类型**
| 类别 | 类型 | typeof 结果 | 示例 |
|------|------|-------------|------|
| 基本类型 | number | `"number"` | `42`, `3.14`, `NaN`, `Infinity` |
| | string | `"string"` | `"hello"`, `'world'`, `\`模板字符串\`` |
| | boolean | `"boolean"` | `true`, `false` |
| | undefined | `"undefined"` | `let x;` → x 为 undefined |
| | null | `"object"`（语言遗留 bug） | `let x = null` |
| | symbol | `"symbol"` | `Symbol("id")` |
| | bigint | `"bigint"` | `42n` |
| 引用类型 | object | `"object"` | `{}`, `[]`, `new Date()` |
| | function | `"function"` | `function() {}` |

> **类型转换**
> - 显式：`Number("42")` `String(42)` `Boolean(1)` `parseInt("42px")`
> - 隐式：`"1" + 2` → `"12"`（+ 是字符串拼接）；`"5" - 2` → `3`（- 转为数字运算）
> - 假值（falsy）：`false`, `0`, `""`, `null`, `undefined`, `NaN` — 其余都是真值

---

**分支**
| 语句 | 说明 | 示例 |
|------|------|------|
| if / else if / else | 条件判断 | `if (score >= 90) { ... } else { ... }` |
| switch / case | 多分支匹配 | `switch(day) { case 1: ...; break; default: ... }` |
| 三元表达式 | 简洁条件 | `const status = age >= 18 ? "成年" : "未成年"` |
| 短路逻辑 | 利用 && 和 \|\| 做条件赋值 | `const name = user.name \|\| "默认名"` |

| 运算符 | 含义 | 注意 |
|--------|------|------|
| `===` / `!==` | 严格等于/不等于（类型+值） | **推荐**始终使用 |
| `==` / `!=` | 宽松等于（会做类型转换） | 避免使用 |
| `>` / `<` / `>=` / `<=` | 比较运算符 | |
| `&&` | 逻辑与 | 短路：左为假则返回左，否则返回右 |
| `\|\|` | 逻辑或 | 短路：左为真则返回左，否则返回右 |
| `??` | 空值合并 | 仅在左为 `null/undefined` 时返回右 |
| `!.` | 可选链 | `obj?.prop?.subProp` — 中间为 null 则不继续访问 |

```javascript
// 短路用法
const url = baseURL || "http://localhost:3000";  // 默认值
const isAdmin = user.role === "admin" && showAdminPanel();  // 条件执行

// 空值合并（更精确的默认值）
const count = data.count ?? 0;     // count=0 仍是有效值，不会被替换
const count2 = data.count || 0;    // count=0 会被替换为 0（因为 0 是假值）

// 可选链（安全访问深层属性）
const city = user?.address?.city ?? "未知";
```

---

**字符串（String）**
| 操作 | 语法 / 方法 | 示例 | 结果 |
|------|------------|------|------|
| 拼接 | `+` / 模板字符串 | `\`Hello ${name}\`` | `` `Hello Alice` `` |
| 长度 | `.length` | `"Hello".length` | `5` |
| 索引 | `[index]` | `"Hello"[0]` | `"H"` |
| 查找 | `.indexOf()` / `.includes()` | `"Hello".includes("ll")` | `true` |
| 截取 | `.slice(start, end)` | `"Hello".slice(1, 4)` | `"ell"` |
| 替换 | `.replace()` / `.replaceAll()` | `"a-b-c".replaceAll("-", "/")` | `"a/b/c"` |
| 分割 | `.split()` | `"a,b,c".split(",")` | `["a", "b", "c"]` |
| 连接 | `.join()`（数组方法） | `["a", "b"].join("-")` | `"a-b"` |
| 去除空白 | `.trim()` | `"  Hi  ".trim()` | `"Hi"` |
| 大小写 | `.toUpperCase()` / `.toLowerCase()` | `"Hi".toUpperCase()` | `"HI"` |
| 模板字符串 | 反引号 + `${}` | `` `结果: ${a + b}` `` | 支持换行和表达式嵌入 |

---

**循环**
| 循环类型 | 语法 | 适用场景 |
|---------|------|----------|
| for | `for (let i=0; i<5; i++)` | 按索引遍历，精确控制次数 |
| for...in | `for (let key in obj)` | 遍历对象**可枚举属性键** |
| for...of | `for (let val of iterable)` | 遍历**可迭代对象的值**（数组/字符串/Map/Set） |
| while | `while (条件) { ... }` | 条件驱动循环 |
| do...while | `do { ... } while (条件)` | 至少执行一次 |
| forEach | `arr.forEach((v, i) => {...})` | 数组遍历（不能 break/continue） |

```javascript
// for...in vs for...of 的区别
const arr = ["a", "b", "c"];
for (let key in arr)  console.log(key);    // "0", "1", "2"（索引，字符串）
for (let val of arr)  console.log(val);    // "a", "b", "c"（值）

// for...in 遍历对象
const user = { name: "Alice", age: 25 };
for (let key in user) {
  console.log(key, user[key]);
}

// while 循环
let i = 0;
while (i < arr.length) {
  console.log(arr[i]);
  i++;
}
```

---

**数组（Array）**
| 方法 | 说明 | 是否修改原数组 | 示例 |
|------|------|---------------|------|
| `.push(x)` | 末尾追加 | ✅ 是 | `arr.push(4)` |
| `.pop()` | 移除并返回末尾元素 | ✅ 是 | `let last = arr.pop()` |
| `.unshift(x)` | 开头插入 | ✅ 是 | `arr.unshift(0)` |
| `.shift()` | 移除并返回开头元素 | ✅ 是 | `let first = arr.shift()` |
| `.splice(i, n, ...)` | 从 i 开始删 n 个，可选插入 | ✅ 是 | `arr.splice(1, 1, "x")` |
| `.slice(s, e)` | 返回子数组副本 | ❌ 否 | `arr.slice(1, 3)` |
| `.concat(arr2)` | 合并数组 | ❌ 否 | `arr.concat([4,5])` |
| `.indexOf(x)` | 查找元素索引 | ❌ 否 | `arr.indexOf(2)` |
| `.includes(x)` | 是否包含某元素 | ❌ 否 | `arr.includes(2)` |
| `.join(sep)` | 转换为字符串 | ❌ 否 | `arr.join(",")` |
| `.reverse()` | 反转数组 | ✅ 是 | `arr.reverse()` |
| `.sort(fn)` | 排序 | ✅ 是 | `arr.sort((a,b) => a-b)` |
| **展开** | `[...arr]` | ❌ 否 | `const copy = [...arr]` |

**高阶函数（遍历/转换/筛选）**
| 方法 | 说明 | 返回 | 示例 |
|------|------|------|------|
| `.forEach(fn)` | 遍历每个元素 | `undefined` | `arr.forEach(v => console.log(v))` |
| `.map(fn)` | 变换每个元素 | **新数组** | `arr.map(x => x * 2)` |
| `.filter(fn)` | 筛选符合条件的元素 | **新数组** | `arr.filter(x => x > 0)` |
| `.find(fn)` | 查找第一个符合条件的元素 | 元素或 `undefined` | `arr.find(x => x > 3)` |
| `.some(fn)` | 是否有元素符合条件 | `boolean` | `arr.some(x => x > 5)` |
| `.every(fn)` | 是否所有元素都符合条件 | `boolean` | `arr.every(x => x > 0)` |
| `.reduce(fn, init)` | 累积归约为单个值 | 归约结果 | `arr.reduce((sum, x) => sum + x, 0)` |
| `.flat(n)` | 数组扁平化 | 新数组 | `[[1,2],[3]].flat()` → `[1,2,3]` |

```javascript
const nums = [1, 2, 3, 4, 5];

// map + filter 链式调用
const result = nums
  .filter(x => x % 2 === 0)        // [2, 4]
  .map(x => x * 10);                // [20, 40]

// reduce 统计数据
const sum = nums.reduce((acc, x) => acc + x, 0);      // 15
const max = nums.reduce((a, b) => a > b ? a : b);     // 5

// 展开运算符合并数组
const a = [1, 2], b = [3, 4];
const merged = [...a, ...b];       // [1, 2, 3, 4]
const copy = [...nums];            // 浅拷贝

// 解构赋值
const [first, second, ...rest] = nums;  // first=1, second=2, rest=[3,4,5]
```

---

**对象（Object）**
| 操作 | 语法 / 方法 | 示例 | 说明 |
|------|------------|------|------|
| 创建对象 | `{}` / `new Object()` | `const obj = { name: "Alice" }` | 对象字面量 |
| 访问属性 | `.prop` / `["prop"]` | `user.name` / `user["name"]` | 点语法或方括号 |
| 添加/修改 | 直接赋值 | `user.age = 25` | 属性不存在则创建 |
| 删除属性 | `delete` | `delete user.age` | 删除属性（慎用） |
| 检查属性 | `in` 运算符 | `"name" in user` | 检查自身或继承属性 |
| 自身属性 | `.hasOwnProperty()` | `user.hasOwnProperty("name")` | 仅检查自身属性 |
| 全部键 | `Object.keys(obj)` | `Object.keys(user)` | 返回键数组 |
| 全部值 | `Object.values(obj)` | `Object.values(user)` | 返回值数组 |
| 全部键值对 | `Object.entries(obj)` | `Object.entries(user)` | 返回 `[[k,v],...]` |
| 合并对象 | `Object.assign(t, ...src)` | `Object.assign(target, src)` | ✅ 修改 target |
| 合并对象 | 展开运算符 `{...obj1, ...obj2}` | `const c = {...a, ...b}` | ❌ 不修改原对象 |
| 冻结对象 | `Object.freeze(obj)` | | 禁止修改属性 |

```javascript
const user = { name: "Alice", age: 25 };

// 对象解构
const { name, age } = user;
const { name: userName, city = "未知" } = user;  // 别名 + 默认值

// 展开运算符合并/拷贝对象（浅拷贝）
const clone = { ...user };
const extended = { ...user, role: "admin", age: 26 };  // 覆盖已有属性

// 属性简写
const name = "Alice";
const user2 = { name };   // 等价于 { name: name }

// 计算属性名
const key = "dynamicKey";
const obj = { [key]: "value" };  // { dynamicKey: "value" }

// 遍历对象
for (const [key, value] of Object.entries(user)) {
  console.log(`${key}: ${value}`);
}
```

---

**函数定义与箭头函数**
| 定义方式 | 语法 | 特点 |
|---------|------|------|
| 函数声明 | `function fn() {}` | 有提升，可提前调用 |
| 函数表达式 | `const fn = function() {}` | 无提升，赋值后才能调用 |
| 箭头函数 | `const fn = () => {}` | 无自己的 `this`/`arguments`，不能做构造函数 |
| 方法简写 | `const obj = { fn() {} }` | ES6 对象方法简写形式 |
| 立即执行函数 (IIFE) | `(function() {})()` | 创建独立作用域 |

```javascript
// 函数声明（有提升）
sayHi("Alice");  // ✅ 可以提前调用
function sayHi(name) {
  return `Hi, ${name}!`;
}

// 箭头函数
const add = (a, b) => a + b;                    // 单行隐式 return
const square = x => x * x;                       // 单参数可省略括号
const greet = (name = "Guest") => `Hello ${name}`;  // 默认参数

// 剩余参数（rest params）
const sumAll = (...nums) => nums.reduce((s, n) => s + n, 0);
sumAll(1, 2, 3, 4);  // 10

// this 绑定差异
const obj = {
  name: "Alice",
  greet1: function() { console.log(this.name); },  // this → obj
  greet2: () => console.log(this.name),            // this → 外层作用域
};
obj.greet1();  // "Alice"
obj.greet2();  // undefined（或全局对象）

// 默认参数 + 解构
const createUser = ({ name = "匿名", age = 0 } = {}) => ({ name, age });
createUser({ name: "Alice" });  // { name: "Alice", age: 0 }
```

---

**模块导入导出（ES Module）**
| 导出方式 | 语法 | 导入方式 |
|---------|------|----------|
| 默认导出 | `export default function() {}` | `import fn from "./module"` |
| 命名导出 | `export const name = "..."` | `import { name } from "./module"` |
| 混合导出 | `export default class {}` + 多个命名导出 | `import X, { a, b } from "./module"` |
| 全部导入 | — | `import * as mod from "./module"` |
| 重命名导出 | `export { fn as myFn }` | `import { myFn } from "./module"` |
| 重命名导入 | — | `import { fn as myFn } from "./module"` |

```javascript
// ---------- math.js ----------
// 命名导出
export const PI = 3.14159;
export const add = (a, b) => a + b;

// 默认导出
export default function multiply(a, b) {
  return a * b;
}

// 重命名导出
export { add as sum };

// ---------- app.js ----------
// 导入默认 + 命名
import multiply, { PI, add } from "./math.js";

// 全部导入
import * as math from "./math.js";
math.PI;        // 3.14159
math.default(2, 3);  // 6（默认导出通过 .default 访问）

// 动态导入（按需加载）
const module = await import("./math.js");
console.log(module.add(2, 3));
```

> **CommonJS (Node.js 传统方式)**：`module.exports = {}` / `require("./module")`，ES Module 是 JavaScript 的标准模块系统。

---

**实用内置函数与对象**
| 函数/对象 | 说明 | 示例 |
|-----------|------|------|
| `console.log()` | 控制台输出 | `console.log("值:", x)` |
| `console.table()` | 以表格形式输出数组/对象 | `console.table(users)` |
| `typeof x` | 返回类型字符串 | `typeof 42` → `"number"` |
| `parseInt(s)` / `parseFloat(s)` | 字符串转整数/浮点数 | `parseInt("42px")` → `42` |
| `Number(s)` | 字符串转数字 | `Number("42.5")` → `42.5` |
| `JSON.stringify(obj)` | 对象 → JSON 字符串 | `JSON.stringify({a:1})` → `'{"a":1}'` |
| `JSON.parse(str)` | JSON 字符串 → 对象 | `JSON.parse('{"a":1}')` → `{a:1}` |
| `setTimeout(fn, ms)` | 延迟执行（一次） | `setTimeout(() => alert("时间到"), 1000)` |
| `setInterval(fn, ms)` | 间隔执行 | `const id = setInterval(() => tick(), 1000)` |
| `clearTimeout(id)` / `clearInterval(id)` | 取消定时器 | |
| `Array.isArray(x)` | 判断是否为数组 | `Array.isArray([])` → `true` |
| `Math.max/min/round/floor/ceil/random` | 数学工具 | `Math.random()` → `0~1` 随机数 |
| `Date.now()` | 当前毫秒时间戳 | `Date.now()` → `1717488000000` |
| `.toString()` | 转字符串 | `(42).toString(16)` → `"2a"`（转16进制） |

**异步编程基础**
```javascript
// Promise
const fetchData = () => {
  return new Promise((resolve, reject) => {
    setTimeout(() => {
      const data = { id: 1 };
      resolve(data);    // 成功
      // reject(new Error("失败"));  // 失败
    }, 1000);
  });
};

// async/await（推荐）
async function load() {
  try {
    const data = await fetchData();
    console.log(data);
  } catch (err) {
    console.error("请求失败:", err);
  }
}
```

---

## 学习要点速查

| 知识点 | 关键语法 | 示例 |
|--------|----------|------|
| 变量声明 | `var`(函数级) / `let`(块级) / `const`(常量) | `const PI = 3.14; let count = 0` |
| 函数定义与箭头函数 | `function` 声明/表达式、`=>` 箭头函数、`...args` 剩余参数 | `const add = (a, b) => a + b` |
| 对象操作 | 字面量 `{}`、`.prop`/`[key]`、解构、展开 `...`、`Object.keys/values/entries` | `const { name, ...rest } = user` |
| 数组方法 | `push/pop/map/filter/reduce/find/forEach`、展开、解构 | `nums.filter(x => x > 0).map(x => x * 2)` |
| 模块导入导出 | `export default` / `export { named }` / `import` / 动态 `import()` | `import { readFile } from "fs"` |
| 模板字符串 | 反引号 + `${}`、支持多行和表达式 | `` `Hello ${name}, 总数: ${items.length}` `` |
| 解构赋值 | 数组 `[a, b] = arr` / 对象 `{name, age} = obj` | `const [first, ...rest] = arr` |
| 展开运算符 | `[...arr]` / `{...obj}` | `const merged = { ...a, ...b }` |
| 异步 | Promise / async/await / try-catch | `const data = await fetch(url)` |
| 类型判断 | `typeof` / `Array.isArray()` / `instanceof` / `=== null` | `typeof x === "string"` |

## 遇到的问题

| 日期 | 问题 | 解决方式 |
|------|------|----------|
| | | |
| | | |
