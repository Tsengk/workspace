# 学习笔记：Python3 基础语法

> 关联输出物模板：[输出物模板.md#1.1](./输出物模板.md)
> 覆盖知识点：变量、分支、循环、函数、列表、字典、字符串

---

## 笔记

**关键字**
| 类别 | 关键字 | 说明 |
|-----|--------|-------|
| 逻辑值 | True | 布尔真值 |
|       | False | 布尔假值 |
|       | None | 表示空值或无值 |
| 逻辑运算 | and | 逻辑与运算 |
|       | or | 逻辑或运算 |
|       | not | 逻辑非运算 |
| 条件控制 | if | 条件判断语句 |
|       | elif | 否则如果 |
|       | else | 否则分支 |
| 循环控制 | for | 迭代循环 |
|       | while | 条件循环 |
|       | break | 跳出循环 |
|       | continue | 跳出当前循环的剩余部分，进入下一次迭代 |
| 异常处理 | try | 尝试执行代码块 |
|       | except | 捕获异常 |
|       | finally | 无论是否发生异常都会执行的代码块 |
|       | raise | 抛出异常 |
| 函数定义 | def | 定义函数 |
|       | return | 从函数返回值 |
|       | lambda | 创建匿名函数 |
| 类与对象 | class | 定义类 |
|       | del | 删除对象引用 |
| 模块导入 | import | 导入模块 |
|       | from | 从模块导入特定部分 |
|       | as | 为导入的模块或对象创建别名 |
| 作用域 | global | 声明全局变量 |
|       | nonlocal | 声明非局部变量 |
| 异步编程 | async | 声明异步函数 |
|       | await | 等待异步操作完成 |
| 其他 | assert | 断言，用于测试条件是否为真 |
|       | in | 检查成员关系 |
|       | is | 检查对象身份 |
|       | pass | 空语句，用于占位 |
|       | with | 上下文管理器，用于资源管理 |
|       | yield | 从生成器函数返回值 |


---

**变量**
| 变量 | 变量不需要提前声明类型，第一次给一个变量赋值时，这个变量就被创建了 |
|-----|---------------------------------------------------------|
| 变量命名规则 | 只能包含字母、数字和下划线 |
| | 不能以数字开头 |
| | 区分大小写 |
| | 不能使用Python的保留关键字 |
| 多变量赋值 | `a, b = 1, 2` → a=1, b=2（元组解包） |
| | `a = b = c = 0` → 链式赋值，三个变量均赋值为 0 |
| 动态类型 | 变量可以随时重新赋值为不同类型的值 |
| 引用计数 | 变量是对象的引用，多个变量可引用同一对象 |

---

**类型**
| 类别 | 类型名称 | 关键字 | 特点 | 示例 |
|-----|---------|-------|-----|------|
| 数字 | 整数 | int | 支持任意大小的整数 | 42, -5 |
| | 浮点数 | float | 带小数点的数字 | 3.14, -0.001 |
| | 布尔值 | bool | 用于逻辑判断（本质上是 1 和 0） | True, False |
| 序列 | 字符串 | str | 文本数据，不可变 | "Hello", 'Python' |
| | 列表 | list | 有序集合，可变（可随时增删改） | [1, "apple", True] |
| | 元组 | tuple | 有序集合，不可变（创建后不能修改） | (1, "apple", True) |
| 映射 | 字典 | dict | 键值对 (Key-Value) 集合，无序且可变 | {"name": "Alice", "age": 25} |
| 集合 | 集合 | set | 无序且元素唯一，常用于去重和数学交并集 | {1, 2, 3} |
| **类型查看** | type() | `type(42)` → `<class 'int'>` | 查看变量类型 | |
| **类型转换** | int() / float() / str() / list() / bool() | `int("42")` → 42 | 显式类型转换 | |
| **可变 vs 不可变** | 不可变：int, float, str, tuple, bool | 可变：list, dict, set | | |

---

**分支**
| 分支 | 含义 |
|-----|------|
| if | 如果条件为 True（真），则执行其下方的代码块 |
| elif | 前面的 if 或 elif 条件为假，程序会检查这个条件 |
| else | 前面的所有条件都为假，程序就会执行 else 下方的代码块 |

| 运算符 | 含义 | 示例 |
|--------|------|------|
| == | 等于 | `a == b` |
| != | 不等于 | `a != b` |
| > | 大于 | `a > b` |
| < | 小于 | `a < b` |
| >= | 大于或等于 | `a >= b` |
| <= | 小于或等于 | `a <= b` |
| and | 与 (两个条件同时为 True) | `a > 0 and b > 0` |
| or | 或 (至少一个条件为 True) | `a > 0 or b > 0` |
| not | 非 (取反) | `not a > 0` |
| in | 成员检查 | `"key" in dict` |
| is | 身份检查（是否为同一对象） | `a is None` |

```python
# 分支示例
score = 85
if score >= 90:
    grade = "A"
elif score >= 80:
    grade = "B"
elif score >= 70:
    grade = "C"
else:
    grade = "D"
```

**三元表达式**：`value_if_true if condition else value_if_false`
```python
result = "成年" if age >= 18 else "未成年"
```

---

**字符串（str）**
| 操作 | 语法 / 方法 | 示例 | 结果 |
|------|------------|------|------|
| 拼接 | `+` | `"Hello" + " " + "World"` | `"Hello World"` |
| 重复 | `*` | `"Ha" * 3` | `"HaHaHa"` |
| 索引 | `[index]` | `"Python"[0]` | `'P'` |
| 切片 | `[start:end:step]` | `"Python"[0:3]` | `"Pyt"` |
| 长度 | `len()` | `len("Python")` | `6` |
| 查找 | `.find()` / `.index()` | `"Hello".find("l")` | `2` |
| 替换 | `.replace()` | `"Hello".replace("l", "x")` | `"Hexxo"` |
| 分割 | `.split()` | `"a,b,c".split(",")` | `["a", "b", "c"]` |
| 连接 | `.join()` | `",".join(["a", "b", "c"])` | `"a,b,c"` |
| 去除空白 | `.strip()` | `"  Hi  ".strip()` | `"Hi"` |
| 格式化 | f-string | `f"值={x}"` | `"值=42"` |
| 格式化 | `.format()` | `"{} is {}".format(n, a)` | `"Alice is 25"` |
| 大小写 | `.upper()` / `.lower()` | `"Hi".upper()` | `"HI"` |

> **f-string (f"{变量}")** 是 Python 3.6+ 推荐的格式化方式，清晰且性能好。
> 字符串是不可变类型：每次修改都会创建新字符串对象。

---

**循环**
| 循环类型 | 语法 | 说明 |
|---------|------|------|
| for 循环 | `for 变量 in 可迭代对象:` | 遍历序列、字典、文件等可迭代对象中每个元素 |
| while 循环 | `while 条件:` | 当条件为 True 时重复执行代码块 |

**range() 函数**
| 用法 | 说明 | 示例 | 生成的序列 |
|------|------|------|-----------|
| range(stop) | 0 → stop-1 | `range(5)` | `[0,1,2,3,4]` |
| range(start, stop) | start → stop-1 | `range(2, 6)` | `[2,3,4,5]` |
| range(start, stop, step) | 按步长递增 | `range(1, 10, 2)` | `[1,3,5,7,9]` |
| range(start, stop, -step) | 递减 | `range(5, 0, -1)` | `[5,4,3,2,1]` |

```python
# for 循环遍历列表
fruits = ["苹果", "香蕉", "橙子"]
for fruit in fruits:
    print(fruit)

# 使用 range 控制循环次数
for i in range(5):
    print(f"第 {i} 次迭代")

# 使用 enumerate 同时获取索引和值
for idx, fruit in enumerate(fruits):
    print(f"{idx}: {fruit}")

# while 循环（带计数器）
count = 0
while count < 3:
    print(count)
    count += 1
```

**循环控制**
| 语句 | 作用 | 示例场景 |
|------|------|----------|
| break | 立即跳出整个循环 | 找到目标后提前终止 |
| continue | 跳过当前迭代，进入下一次 | 过滤掉不符合条件的元素 |
| else（循环配套） | 循环**正常结束**（未被 break 中断）时执行 | 检查循环是否完整遍历 |

```python
# break + else 配合使用
for n in range(2, 10):
    for x in range(2, n):
        if n % x == 0:
            print(f"{n} = {x} * {n//x}")
            break
    else:
        print(f"{n} 是质数")  # 仅当内层循环未被 break 中断时执行
```

---

**函数**
| 概念 | 说明 | 示例 |
|------|------|------|
| 定义函数 | 使用 `def` 关键字 | `def greet(name):` |
| 返回值 | 使用 `return` 返回结果；无 return 返回 None | `return a + b` |
| 位置参数 | 按传入顺序匹配 | `add(1, 2)` |
| 默认参数 | 参数有默认值，调用时可省略 | `def power(x, n=2):` |
| 关键字参数 | 通过参数名传参，不依赖顺序 | `greet(name="Alice")` |
| 可变位置参数 | `*args` → 接收任意数量位置参数，打包成元组 | `def sum_all(*nums):` |
| 可变关键字参数 | `**kwargs` → 接收任意数量关键字参数，打包成字典 | `def info(**data):` |
| 文档字符串 | 函数内的说明字符串，用三个引号 | `"""说明"""` |

```python
# 函数定义示例
def greet(name, greeting="Hello"):
    """向指定名称打招呼"""
    return f"{greeting}, {name}!"

print(greet("Alice"))        # Hello, Alice!
print(greet("Bob", "Hi"))    # Hi, Bob!

# *args 和 **kwargs
def log(msg, *tags, **details):
    print(f"消息: {msg}")
    print(f"标签: {tags}")       # ('error', 'critical')
    print(f"详情: {details}")    # {'code': 500, 'user': 'admin'}

log("系统错误", "error", "critical", code=500, user="admin")
```

**匿名函数（lambda）**
| 表达式 | 说明 |
|--------|------|
| `lambda 参数: 返回值表达式` | 创建小型匿名函数，通常作为参数传递 |

```python
# lambda 示例
square = lambda x: x ** 2
print(square(5))  # 25

# 与 sorted 配合使用
students = [("Alice", 85), ("Bob", 92), ("Charlie", 78)]
sorted_students = sorted(students, key=lambda s: s[1], reverse=True)
```

**作用域规则（LEGB）**
| 作用域 | 英文 | 说明 |
|--------|------|------|
| 局部 | Local (L) | 函数内部定义的变量 |
| 闭包 | Enclosing (E) | 嵌套函数中外层函数的变量 |
| 全局 | Global (G) | 模块级别的变量 |
| 内置 | Built-in (B) | Python 内置的命名空间 |

```python
count = 0         # 全局变量

def test():
    global count  # 声明使用全局变量
    count += 1

def outer():
    x = "outer"
    def inner():
        nonlocal x  # 声明使用外层变量
        x = "inner"
    inner()
    return x        # "inner"
```

---

**列表（list）**
| 操作 | 语法 / 方法 | 示例 | 结果 |
|------|------------|------|------|
| 创建 | `[]` 或 `list()` | `nums = [1, 2, 3]` | |
| 索引 | `[index]` | `nums[0]` | `1` |
| 反索引 | `[-index]` | `nums[-1]` | `3` |

**切片操作** `list[start:end:step]`
```python
nums = [0, 1, 2, 3, 4, 5]
nums[1:4]      # [1, 2, 3]
nums[:3]       # [0, 1, 2]    从头开始
nums[3:]       # [3, 4, 5]    到末尾结束
nums[::2]      # [0, 2, 4]    步长为2
nums[::-1]     # [5, 4, 3, 2, 1, 0]  反转
```

**常用方法**
| 方法 | 说明 | 示例 |
|------|------|------|
| `.append(x)` | 末尾追加元素 | `nums.append(4)` |
| `.extend(iter)` | 用另一个可迭代对象扩展 | `nums.extend([5,6])` |
| `.insert(i, x)` | 在指定位置插入 | `nums.insert(0, -1)` |
| `.remove(x)` | 删除第一个匹配的元素 | `nums.remove(2)` |
| `.pop(i)` | 弹出（删除并返回）指定位置元素 | `last = nums.pop()` |
| `.index(x)` | 返回第一个匹配的索引 | `idx = nums.index(3)` |
| `.count(x)` | 统计元素出现次数 | `c = nums.count(1)` |
| `.sort()` | 原地排序 | `nums.sort(reverse=True)` |
| `.reverse()` | 原地反转 | `nums.reverse()` |
| `sorted()` | 返回排序后的新列表（不修改原列表） | `sorted_nums = sorted(nums)` |
| `len()` | 获取列表长度 | `n = len(nums)` |
| `del list[i]` | 删除指定索引元素 | `del nums[2]` |

**列表推导式（List Comprehension）**
```python
# 基本: [表达式 for 变量 in 可迭代对象]
squares = [x**2 for x in range(10)]           # [0, 1, 4, 9, ..., 81]

# 带条件: [表达式 for 变量 in 可迭代对象 if 条件]
evens = [x for x in range(20) if x % 2 == 0]  # [0, 2, 4, ..., 18]

# 嵌套循环
pairs = [(x, y) for x in [1,2,3] for y in [4,5]]  # [(1,4),(1,5),(2,4),(2,5),(3,4),(3,5)]

# 实际场景：批量处理文件名
files = ["data.csv", "report.pdf", "notes.txt"]
csv_files = [f for f in files if f.endswith(".csv")]
```

---

**字典（dict）**
| 操作 | 语法 / 方法 | 示例 | 结果 |
|------|------------|------|------|
| 创建 | `{}` 或 `dict()` | `d = {"a": 1, "b": 2}` | |
| 取键值 | `[key]` | `d["a"]` | `1` |
| 安全取值 | `.get(key, default)` | `d.get("c", 0)` | `0`（键不存在时返回默认值） |
| 设置默认值 | `.setdefault(k, v)` | `d.setdefault("c", 3)` | 键不存在则插入默认值并返回 |
| 合并更新 | `.update(d2)` | `d.update({"b": 22, "d": 44})` | 更新已有键，插入新键 |
| 删除键 | `.pop(key)` | `val = d.pop("a")` | 返回被删值，键不存在报错 |
| 删除最后 | `.popitem()` | `k, v = d.popitem()` | 删除并返回最后一个键值对 |
| 全部键 | `.keys()` | `d.keys()` | `dict_keys(['a', 'b'])` |
| 全部值 | `.values()` | `d.values()` | `dict_values([1, 2])` |
| 全部项 | `.items()` | `d.items()` | `dict_items([('a', 1), ('b', 2)])` |
| 清空 | `.clear()` | `d.clear()` | 清空字典 |
| 复制 | `.copy()` | `d2 = d.copy()` | 浅拷贝 |

**遍历字典**
```python
config = {"host": "localhost", "port": 8080, "debug": True}

# 遍历键
for key in config:            # 等同于 config.keys()
    print(key)

# 遍历值
for value in config.values():
    print(value)

# 同时遍历键和值（推荐）
for key, value in config.items():
    print(f"{key} = {value}")

# 检查键是否存在
if "host" in config:          # 比 .keys() 更高效
    print("host 已配置")
```

**字典推导式**
```python
# {键表达式: 值表达式 for 变量 in 可迭代对象}
squares = {x: x**2 for x in range(5)}  # {0: 0, 1: 1, 2: 4, 3: 9, 4: 16}

# 带条件筛选
even_squares = {x: x**2 for x in range(10) if x % 2 == 0}

# 实际场景：配置文件解析、数据映射
# 将两个列表合并为字典
keys = ["name", "age", "city"]
values = ["Alice", 25, "Beijing"]
user_info = dict(zip(keys, values))  # {"name": "Alice", "age": 25, "city": "Beijing"}
```

> **注意**：Python 3.7+ 字典保持插入顺序；Python 3.6 及以下字典是无序的。

---

**实用内置函数**
| 函数 | 说明 | 示例 |
|------|------|------|
| `print()` | 输出到控制台 | `print("Hello", end="")` |
| `input()` | 从控制台读取输入 | `name = input("输入名称:")` |
| `len()` | 返回长度 | `len([1,2,3])` → 3 |
| `type()` | 返回类型 | `type(42)` → `<class 'int'>` |
| `range()` | 生成整数序列 | `range(5)` |
| `enumerate()` | 返回索引-值对 | `list(enumerate(["a","b"]))` → `[(0,"a"),(1,"b")]` |
| `zip()` | 并行打包多个可迭代对象 | `list(zip([1,2],["a","b"]))` → `[(1,"a"),(2,"b")]` |
| `sum()` | 求和 | `sum([1,2,3])` → 6 |
| `max()` / `min()` | 最大值 / 最小值 | `max([3,1,2])` → 3 |
| `sorted()` | 返回排序后的列表 | `sorted([3,1,2])` → `[1,2,3]` |
| `map()` | 对每个元素应用函数 | `list(map(str, [1,2]))` → `["1","2"]` |
| `filter()` | 按条件过滤 | `list(filter(lambda x: x>0, [-1,0,2]))` → `[2]` |
| `all()` | 所有元素为 True | `all([True, 1, "a"])` → True |
| `any()` | 至少一个元素为 True | `any([False, 0, "a"])` → True |

---

## 学习要点速查

| 知识点 | 关键语法 | 示例 |
|--------|----------|------|
| 变量与类型 | 动态类型、多变量赋值、类型转换 | `name = "Alice"` → str; `age = 25` → int; `int("42")` → 类型转换 |
| 条件分支 | `if/elif/else`、三元表达式 | `"成人" if age >= 18 else "未成年"` |
| 循环控制 | `for/while`、`range()`、`break/continue/else`、`enumerate()` | `for i, v in enumerate(items):` |
| 函数定义 | `def`、参数（位置/默认/关键字/`*args`/`**kwargs`）、`return`、`lambda` | `def func(a, b=10, *args, **kwargs):` |
| 列表操作 | 索引/切片、`.append()/.pop()/.sort()`、列表推导式 | `[x**2 for x in range(10) if x % 2 == 0]` |
| 字典操作 | `.get()/.items()/.update()`、字典推导式、遍历 | `for k, v in config.items():` |
| 字符串操作 | `.split()/.join()/.replace()/.strip()`、f-string、切片 | `f"目录: {path}"`、`",".join(tags)` |
| 内置函数 | `len/type/range/enumerate/zip/map/filter/sorted` | `list(zip(keys, values))` → 合并为字典 |

## 遇到的问题

| 日期 | 问题 | 解决方式 |
|------|------|----------|
| | | |
| | | |
