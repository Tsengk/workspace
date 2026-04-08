#ifndef __GAME_MODEL_H__
#define __GAME_MODEL_H__

#include <vector>

#include "models/CardModel.h"

/**
 * @brief 游戏运行时总模型。
 *        用来保存桌牌区、手牌区、顶部牌和顶部历史牌的完整状态。
 */
class GameModel
{
public:
    /** 当前场景里所有运行时卡牌。 */
    std::vector<CardModel> cards;
    /** 属于桌牌区的卡牌 id。 */
    std::vector<int> playfieldCardIds;
    /** 属于手牌区的卡牌 id。 */
    std::vector<int> reserveCardIds;
    /** 已经被替换下来的顶部牌 id，用于形成顶部历史堆叠效果。 */
    std::vector<int> topHistoryCardIds;
    /** 当前顶部牌 id。 */
    int currentTopCardId;
    /** 顶部牌槽位坐标，也是匹配动画的目标点。 */
    cocos2d::Vec2 topCardPosition;

    GameModel();

    /**
     * @brief 清空整个运行时模型。
     */
    void reset();

    /**
     * @brief 按 id 查找可修改的卡牌对象。
     * @param cardId 运行时卡牌 id。
     * @return 找到则返回指针，否则返回 nullptr。
     */
    CardModel* findCard(int cardId);

    /**
     * @brief 按 id 查找只读卡牌对象。
     * @param cardId 运行时卡牌 id。
     * @return 找到则返回指针，否则返回 nullptr。
     */
    const CardModel* findCard(int cardId) const;
};

/**
 * @brief 与布局派生状态相关的模型工具函数。
 */
namespace GameModelUtils
{
    /**
     * @brief 刷新桌牌区可点击状态。
     *        每组桌牌只允许当前最外层那张牌可点击。
     * @param gameModel 需要原地更新的运行时模型。
     */
    void refreshPlayfieldInteractivity(GameModel& gameModel);
}

#endif // __GAME_MODEL_H__
