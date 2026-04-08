# 纸牌程序设计文档

## 1. 项目说明

本项目基于 `cocos2d-x` 搭建，实现了一个简化的纸牌匹配演示程序。  
当前程序围绕以下核心规则运行：

- 手牌区的牌可以直接点击，并替换顶部牌。
- 桌牌区的牌只有在与当前顶部牌点数相差 `1` 时才允许移动。
- 每一次有效移动都会进入撤销栈，点击“回退”后按后进先出顺序恢复。
- 初始牌局由 `Resources/configs/levels/level_1.json` 配置并加载。

这份 README 同时作为程序设计文档交付，重点说明当前代码结构、模块职责、数据流，以及后续如何在现有架构下扩展“新卡牌”和“新类型的回退功能”。

## 2. 当前实现范围

### 2.1 已实现功能

- 关卡 JSON 加载
- 静态关卡配置转运行时模型
- 桌牌区与手牌区的界面创建
- 顶部牌替换与匹配规则判断
- 顶部牌历史堆叠显示
- 单步/多步撤销
- 中文化界面提示与区域标题

### 2.2 当前规则特点

- 花色只参与显示，不参与匹配判断。
- 点数匹配规则由 `MatchRuleService` 统一控制。
- 撤销记录目前按“一次移动一条记录”的方式实现。
- 界面层不直接决定规则，界面只负责显示与动画。

## 3. 代码结构设计

项目核心代码位于 `Classes/` 目录，采用接近 MVC 的分层组织方式。

game
Classes
|- AppDelegate.*                      应用启动与生命周期入口
|- GameScene.*                        场景入口，创建总控制器
|- configs
|  |- loaders
|  |  `- LevelConfigLoader.*          关卡 JSON 读取与校验
|  `- models
|     |- CardResConfig.h              卡牌资源与布局参数
|     `- LevelConfig.h                静态关卡配置结构
|- controllers
|  |- GameController.*                总流程控制
|  `- CardController.*                卡牌点击、移动、回退逻辑
|- managers
|  `- UndoManager.*                   撤销栈管理
|- models
|  |- CardTypes.h                     花色、点数、区域等枚举
|  |- CardModel.h                     单张卡牌运行时数据
|  |- GameModel.*                     整局游戏运行时数据
|  `- UndoModel.h                     单条撤销记录
|- services
|  |- GameModelFromLevelGenerator.*   静态配置转运行时模型
|  `- MatchRuleService.*              匹配规则服务
|- utils
|  `- CardUtils.*                     资源路径与卡牌辅助转换
`- views
   |- CardView.*                      单张卡牌显示与点击转发
   `- GameView.*                      主界面、布局、动画与提示
```

## 4. 模块职责说明

### 4.1 启动层

- `AppDelegate`
  负责初始化引擎、OpenGL 上下文与首个场景。
- `GameScene`
  负责创建 `GameController`，并启动默认关卡。

### 4.2 配置层

- `LevelConfigLoader`
  从 `Resources/configs/levels/*.json` 中读取牌局配置，并检查字段合法性。
- `LevelConfig`
  表示 JSON 对应的静态关卡数据。
- `CardResConfig`
  描述一张牌的资源路径与各元素布局参数。

### 4.3 模型层

- `CardTypes`
  定义花色、点数、卡牌区域等基础枚举。
- `CardModel`
  表示单张牌在运行时的状态，例如区域、位置、是否可点击。
- `GameModel`
  表示整局游戏的运行时状态，包括：
  - 所有卡牌
  - 桌牌区卡牌 id 集合
  - 手牌区卡牌 id 集合
  - 顶部历史牌 id 集合
  - 当前顶部牌 id
- `UndoModel`
  表示一次可撤销操作在回退时所需的最小状态。

### 4.4 控制层

- `GameController`
  负责组织“加载配置 -> 生成模型 -> 创建视图 -> 绑定回调 -> 动画同步”的总流程。
- `CardController`
  负责具体交互逻辑：
  - 处理卡牌点击
  - 判断是手牌移动还是桌牌匹配
  - 修改 `GameModel`
  - 生成动画参数 `CardMoveResult`
  - 压入撤销记录
  - 执行回退恢复

### 4.5 服务与管理层

- `MatchRuleService`
  抽离匹配规则，避免把规则硬编码进视图层。
- `GameModelFromLevelGenerator`
  将静态关卡配置转换成运行时模型。
- `UndoManager`
  使用栈结构保存撤销历史。

### 4.6 视图层

- `GameView`
  负责主界面布局、卡牌容器、按钮、提示文字、动画播放。
- `CardView`
  负责单张牌的牌面渲染、资源替换和点击事件转发。

## 5. 核心数据流

### 5.1 游戏启动数据流

1. `GameScene` 创建 `GameController`
2. `GameController::startGame()` 调用 `LevelConfigLoader`
3. `LevelConfigLoader` 读取 `level_1.json`
4. `GameModelFromLevelGenerator` 把 `LevelConfig` 转换为 `GameModel`
5. `GameView` 根据 `GameModel` 构建整套界面
6. `CardView` 根据每张 `CardModel` 显示具体牌面

### 5.2 点击移动数据流

1. 用户点击 `CardView`
2. `CardView` 仅把 `cardId` 回调给 `GameView`
3. `GameView` 再转发给 `GameController`
4. `GameController` 调用 `CardController::handleCardClick()`
5. `CardController` 判断规则并修改 `GameModel`
6. 同时把本次操作写入 `UndoManager`
7. `GameController` 根据返回的动画参数通知 `GameView` 播放动画
8. 动画结束后，`GameView` 再次按最新 `GameModel` 同步界面

### 5.3 回退数据流

1. 用户点击“回退”
2. `GameController` 调用 `CardController::handleUndo()`
3. `CardController` 从 `UndoManager` 弹出最近记录
4. 根据 `UndoModel` 恢复被移动牌和上一个顶部牌
5. 重新刷新桌牌区可点击状态
6. `GameView` 播放回退动画并同步界面

## 6. 当前关键信息结构

### 6.1 关卡配置

当前关卡入口文件：

- `Resources/configs/levels/level_1.json`

关卡里每张牌至少包含：

- `CardFace`
- `CardSuit`
- `Position`

### 6.2 运行时卡牌结构

`CardModel` 中对扩展最关键的字段包括：

- `cardId`：运行时唯一标识
- `face`：点数
- `suit`：花色
- `area`：当前所在区域
- `homePosition`：原始位置
- `currentPosition`：当前逻辑位置
- `isFaceUp`：是否正面显示
- `isInteractive`：是否允许点击

### 6.3 撤销结构

当前 `UndoModel` 记录的内容包括：

- `movedCardId`
- `previousTopCardId`
- `previousArea`
- `previousPosition`

这意味着当前撤销模型主要面向“单张牌移动到顶部牌位”这一类操作。

## 7. 未来扩展说明

下面是后续开发最常见的两个扩展方向。

### 7.1 如何新增一张卡牌

这里分成两种情况：  
第一种是“在现有规则下给牌局再加一张牌”；第二种是“新增一种以前没有出现过的卡牌表现或资源”。

#### 7.1.1 只是在关卡中新增一张现有类型的牌

这是最常见、改动最小的情况，通常只需要改配置文件。

操作步骤：

1. 打开 `Resources/configs/levels/level_1.json`
2. 选择把新牌加入 `Playfield` 或 `Stack`
3. 按现有格式追加一个对象，填写：
   - `CardFace`
   - `CardSuit`
   - `Position.x`
   - `Position.y`
4. 重新运行程序

数据会按下面的链路自动生效：

`level_1.json`
-> `LevelConfigLoader`
-> `LevelConfig`
-> `GameModelFromLevelGenerator`
-> `GameModel`
-> `GameView / CardView`

这类新增通常不需要改以下代码：

- `MatchRuleService`
- `CardController`
- `UndoManager`

前提是：

- 使用的是当前已经支持的点数枚举
- 使用的是当前已经支持的花色枚举
- 对应资源图片已经存在

#### 7.1.2 如果新增的卡牌需要新的点数、花色或资源表现

如果未来要扩展“新的卡牌定义”，需要按下面顺序修改。

1. 修改 `Classes/models/CardTypes.h`
   - 增加新的点数枚举或花色枚举
   - 保证枚举范围与校验逻辑一致

2. 修改 `Classes/configs/loaders/LevelConfigLoader.cpp`
   - 检查 `isValidFaceValue()` / `isValidSuitValue()` 是否仍然覆盖新枚举范围

3. 修改 `Classes/utils/CardUtils.cpp`
   - 为新的点数或花色补充资源路径映射
   - 确保 `getBigNumberRes()`、`getSmallNumberRes()`、`getSuitRes()` 能返回正确资源

4. 补充 `Resources/res/...` 下的实际图片资源
   - 大号数字
   - 小号数字
   - 花色图标
   - 如需新背景，也要补对应资源

5. 如果新卡牌的布局和普通牌不同，修改：
   - `Classes/configs/models/CardResConfig.h`
   - 或 `Classes/views/CardView.cpp`

6. 如果新卡牌还需要新规则，再修改：
   - `Classes/services/MatchRuleService.*`
   - `Classes/controllers/CardController.*`

#### 7.1.3 推荐原则

- 能通过关卡配置解决的，尽量不要先改控制器逻辑。
- 资源映射统一收口到 `CardUtils`，不要把路径散落在各处。
- 牌面显示差异优先放在 `CardView` 和 `CardResConfig` 层处理。
- 交互规则变化优先放在 `MatchRuleService` 和 `CardController` 层处理。

### 7.2 如何新增一种新的回退功能

当前回退机制只覆盖“卡牌移动到顶部牌位”这一类动作。  
如果未来新增新的操作类型，例如：

- 撤销翻开一张牌
- 撤销一次发牌
- 撤销一次洗牌
- 撤销一次批量移动

建议不要继续把所有逻辑硬塞在现有的单一 `UndoModel` 字段里，而是按“操作类型”扩展。

#### 7.2.1 推荐扩展方案

建议把当前撤销结构升级为“带类型的撤销记录”。

推荐新增一个枚举，例如：

```cpp
enum UndoActionType
{
    UAT_MOVE_TO_TOP = 0,
    UAT_FLIP_CARD,
    UAT_DEAL_CARD
};
```

然后在 `UndoModel` 中增加动作类型和额外状态字段，例如：

```cpp
struct UndoModel
{
    UndoActionType actionType;

    int movedCardId;
    int previousTopCardId;
    CardAreaType previousArea;
    cocos2d::Vec2 previousPosition;

    bool previousFaceUp;
    bool previousInteractive;
};
```

这样做的核心目的，是让每种新操作都能明确知道“回退时需要恢复哪些状态”。

#### 7.2.2 实现步骤

1. 扩展 `Classes/models/UndoModel.h`
   - 增加 `UndoActionType`
   - 为新操作补充需要恢复的状态字段

2. 保持 `Classes/managers/UndoManager.*` 尽量不变
   - `UndoManager` 本质只是栈容器
   - 只要 `UndoModel` 结构可表达新动作，一般不需要改管理器接口

3. 修改 `Classes/controllers/CardController.cpp`
   - 在新动作发生时压入对应类型的 `UndoModel`
   - 在 `handleUndo()` 中根据 `actionType` 分发到不同恢复逻辑

4. 推荐把 `handleUndo()` 拆成多个私有函数
   - `undoMoveToTop(...)`
   - `undoFlipCard(...)`
   - `undoDealCard(...)`

5. 如果新回退会带来不同动画，扩展 `CardMoveResult`
   - 当前 `CardMoveResult` 只适合单张牌从 A 点飞到 B 点
   - 如果以后是多张牌回退，可能需要：
     - 多张牌动画参数
     - 动画类型字段
     - 是否需要刷新整屏的标记

6. 如果新回退涉及新的显示状态，还要同步修改：
   - `Classes/views/GameView.*`
   - `Classes/views/CardView.*`

#### 7.2.3 一个清晰的落地例子：新增“翻牌回退”

假设未来增加一个新功能：  
某张牌被点击后不是立刻移动，而是先从背面翻到正面；这个动作也需要支持回退。

建议改法如下：

1. 在 `UndoModel` 里增加：
   - `actionType`
   - `targetCardId`
   - `previousFaceUp`
   - `previousInteractive`

2. 在执行“翻牌”动作的控制逻辑里：
   - 修改 `CardModel.isFaceUp`
   - 修改 `CardModel.isInteractive`
   - 同时压入一条 `UAT_FLIP_CARD` 类型记录

3. 在 `CardController::handleUndo()` 中：
   - 识别到 `UAT_FLIP_CARD`
   - 找到目标牌
   - 恢复 `isFaceUp`
   - 恢复 `isInteractive`

4. 如果翻牌需要动画：
   - 扩展 `GameView` 的动画接口
   - 或增加单独的翻牌动画方法

#### 7.2.4 设计建议

- 撤销记录要记录“恢复所需的最小完整状态”，不要只记录结果。
- 控制器负责写入撤销记录，也负责消费撤销记录。
- 管理器只负责保存顺序，不负责理解业务。
- 如果未来操作类型越来越多，建议把回退逻辑从 `CardController` 中再拆一层 `UndoService`。

## 8. 后续扩展建议

如果项目继续演进，建议按下面方向逐步增强：

- 增加 `UndoActionType`，把撤销从单一移动扩展为多类型动作系统
- 将匹配规则从“点数相差 1”扩展为策略模式
- 把关卡配置支持到多关卡切换
- 为 `CardView` 增加翻牌、发牌、批量回退等动画接口
- 给 `README` 再补一份“关卡配置格式说明”

## 9. 交付结论

本项目当前已经具备清晰的分层结构：

- 配置层负责输入
- 模型层负责状态
- 控制层负责规则与流程
- 视图层负责显示与动画
- 管理层负责撤销历史
- 服务层负责规则和生成逻辑

在这套结构下：

- 新增一张普通卡牌，优先只改关卡 JSON
- 新增一类卡牌表现，优先改 `CardTypes + CardUtils + 资源文件 + CardView`
- 新增一种回退功能，优先改 `UndoModel + CardController`，必要时扩展 `GameView`

因此，当前代码结构对后续需求扩展是友好的，尤其适合继续追加：

- 新关卡
- 新牌面资源
- 新匹配规则
- 新的回退动作类型
