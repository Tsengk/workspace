#ifndef __GAME_VIEW_H__
#define __GAME_VIEW_H__

#include <functional>
#include <unordered_map>

#include "cocos2d.h"

#include "configs/models/CardResConfig.h"
#include "models/GameModel.h"

class CardView;

/**
 * @brief 游戏主视图。
 *        负责渲染区域背景、卡牌节点、提示文字和回退按钮。
 */
class GameView : public cocos2d::Layer
{
public:
    /**
     * @brief 创建全屏游戏视图。
     * @param cardResConfig 传递给每张卡牌视图的资源配置。
     * @return 自动释放的 GameView 实例。
     */
    static GameView* create(const CardResConfig& cardResConfig);

    /**
     * @brief 初始化主视图。
     * @param cardResConfig 传递给每张卡牌视图的资源配置。
     * @return 是否初始化成功。
     */
    bool init(const CardResConfig& cardResConfig);

    /**
     * @brief 根据当前模型创建所有卡牌子视图。
     * @param gameModel 当前关卡的运行时模型。
     */
    void buildView(const GameModel& gameModel);

    /**
     * @brief 用运行时模型全量刷新当前界面。
     * @param gameModel 当前关卡的运行时模型。
     */
    void syncView(const GameModel& gameModel);

    /**
     * @brief 刷新界面，但保持某张正在移动的牌暂时停留在旧位置。
     * @param gameModel 当前关卡的运行时模型。
     * @param movingCardId 需要保持在动画起点的牌 id。
     * @param startPosition 动画开始前的显示坐标。
     */
    void syncViewKeepingCard(const GameModel& gameModel, int movingCardId, const cocos2d::Vec2& startPosition);

    /**
     * @brief 为指定卡牌播放简单移动动画。
     * @param cardId 需要播放动画的牌 id。
     * @param targetPosition 动画终点。
     * @param onComplete 动画完成回调。
     */
    void playCardMoveAnimation(int cardId, const cocos2d::Vec2& targetPosition, const std::function<void()>& onComplete);

    /**
     * @brief 注册卡牌点击回调。
     * @param onCardClick 点击后回传 cardId 的回调。
     */
    void setCardClickCallback(const std::function<void(int)>& onCardClick);

    /**
     * @brief 注册回退按钮回调。
     * @param onUndoClick 点击回退按钮时触发的回调。
     */
    void setUndoClickCallback(const std::function<void()>& onUndoClick);

    /**
     * @brief 更新底部提示文本。
     * @param message 需要显示的提示内容。
     */
    void showMessage(const std::string& message);

private:
    /**
     * @brief 创建静态背景、区域标题和回退按钮等固定界面元素。
     */
    void setupLayout();

    /**
     * @brief 把一张牌的模型状态同步到对应视图上。
     * @param gameModel 用于计算层级和叠放偏移的运行时模型。
     * @param cardModel 当前要刷新的卡牌数据。
     * @param cardView 对应的卡牌视图对象。
     */
    void syncSingleCard(const GameModel& gameModel, const CardModel& cardModel, CardView* cardView);

    /**
     * @brief 计算卡牌最终显示位置，包含手牌区和顶部历史区的叠放偏移。
     * @param gameModel 当前运行时模型。
     * @param cardModel 当前卡牌数据。
     * @return 这张牌在界面上的最终显示坐标。
     */
    cocos2d::Vec2 getDisplayPosition(const GameModel& gameModel, const CardModel& cardModel) const;

    /**
     * @brief 计算卡牌显示层级。
     * @param gameModel 当前运行时模型。
     * @param cardModel 当前卡牌数据。
     * @return 这张牌应该使用的 z-order。
     */
    int getZOrderForCard(const GameModel& gameModel, const CardModel& cardModel) const;

private:
    /** 卡牌资源静态配置。 */
    CardResConfig _cardResConfig;
    /** 承载所有卡牌节点的父容器。 */
    cocos2d::Node* _cardContainer;
    /** 界面底部的提示文字。 */
    cocos2d::Label* _hintLabel;
    /** 转发给 GameController 的卡牌点击回调。 */
    std::function<void(int)> _onCardClick;
    /** 转发给 GameController 的回退回调。 */
    std::function<void()> _onUndoClick;
    /** 运行时卡牌视图索引表，便于按 id 快速找到对应视图。 */
    std::unordered_map<int, CardView*> _cardViews;
};

#endif // __GAME_VIEW_H__
