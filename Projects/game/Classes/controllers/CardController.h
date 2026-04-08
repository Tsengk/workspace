#ifndef __CARD_CONTROLLER_H__
#define __CARD_CONTROLLER_H__

#include "managers/UndoManager.h"
#include "models/GameModel.h"

/**
 * @brief 控制器修改模型后，返回给视图层的动画参数。
 */
struct CardMoveResult
{
    /** 本次点击是否形成了有效移动。 */
    bool success;
    /** 需要播放动画的牌 id。 */
    int movedCardId;
    /** 动画起点。 */
    cocos2d::Vec2 fromPosition;
    /** 动画终点。 */
    cocos2d::Vec2 toPosition;

    CardMoveResult()
        : success(false)
        , movedCardId(kInvalidCardId)
        , fromPosition(cocos2d::Vec2::ZERO)
        , toPosition(cocos2d::Vec2::ZERO)
    {
    }
};

/**
 * @brief 卡牌交互控制器。
 *        负责处理点击规则、修改模型，并生成视图层需要的动画数据。
 */
class CardController
{
public:
    CardController();

    /**
     * @brief 注入撤销管理器。
     * @param undoManager 由 GameController 持有的撤销管理器。
     */
    void init(UndoManager* undoManager);

    /**
     * @brief 处理卡牌点击。
     * @param gameModel 需要被修改的运行时模型。
     * @param cardId 被点击的牌 id。
     * @param outResult 返回给视图层的动画参数。
     * @return 本次点击是否产生了有效移动。
     */
    bool handleCardClick(GameModel& gameModel, int cardId, CardMoveResult& outResult);

    /**
     * @brief 处理回退。
     * @param gameModel 需要被恢复的运行时模型。
     * @param outResult 返回给视图层的回退动画参数。
     * @return 是否成功生成回退动画。
     */
    bool handleUndo(GameModel& gameModel, CardMoveResult& outResult);

private:
    /**
     * @brief 把手牌区的一张牌移动到顶部牌位置。
     */
    bool moveReserveCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult);

    /**
     * @brief 把满足规则的桌牌移动到顶部牌位置。
     */
    bool movePlayfieldCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult);

    /**
     * @brief 有效移动的公共实现，手牌替换和桌牌匹配都会走这里。
     */
    bool moveCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult);

private:
    /** 由 GameController 注入的撤销管理器。 */
    UndoManager* _undoManager;
};

#endif // __CARD_CONTROLLER_H__
