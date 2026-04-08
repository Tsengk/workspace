#ifndef __GAME_CONTROLLER_H__
#define __GAME_CONTROLLER_H__

#include <string>

#include "controllers/CardController.h"
#include "managers/UndoManager.h"
#include "models/GameModel.h"

namespace cocos2d
{
class Node;
}

class GameView;

/**
 * @brief 游戏主控制器。
 *        负责组织关卡加载、模型生成、视图创建、点击回调和撤销流程。
 */
class GameController
{
public:
    GameController();

    /**
     * @brief 使用场景根节点初始化控制器。
     * @param parent 用于挂载 GameView 的父节点。
     * @return 是否初始化成功。
     */
    bool init(cocos2d::Node* parent);

    /**
     * @brief 加载关卡、生成运行时模型并创建整套界面。
     * @param levelConfigPath Resources/ 下的关卡路径。
     * @return 是否启动成功。
     */
    bool startGame(const std::string& levelConfigPath);

private:
    /**
     * @brief 处理来自视图层的卡牌点击。
     * @param cardId 被点击的牌 id。
     */
    void handleCardClick(int cardId);

    /**
     * @brief 处理来自视图层的回退请求。
     */
    void handleUndoClick();

private:
    /** 挂载视图的父节点，一般就是当前场景。 */
    cocos2d::Node* _parent;
    /** 当前关卡的运行时模型。 */
    GameModel _gameModel;
    /** 负责单次卡牌点击/回退处理的子控制器。 */
    CardController _cardController;
    /** 当前关卡使用的撤销管理器。 */
    UndoManager _undoManager;
    /** 根视图对象，负责把模型显示到界面上。 */
    GameView* _gameView;
    /** 动画播放锁，避免用户在移动过程中重复触发操作。 */
    bool _isAnimating;
};

#endif // __GAME_CONTROLLER_H__
