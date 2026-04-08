#ifndef __UNDO_MODEL_H__
#define __UNDO_MODEL_H__

#include "cocos2d.h"

#include "models/CardTypes.h"

/**
 * @brief 单条撤销记录。
 *        记录一次卡牌移动前的关键状态，供回退时恢复顶部牌关系和原位置。
 */
struct UndoModel
{
    /** 本次被移动到顶部牌位的卡牌 id。 */
    int movedCardId;
    /** 移动发生前原本显示在顶部牌位的卡牌 id。 */
    int previousTopCardId;
    /** 被移动卡牌移动前所在的区域。 */
    CardAreaType previousArea;
    /** 被移动卡牌移动前的位置。 */
    cocos2d::Vec2 previousPosition;

    UndoModel()
        : movedCardId(kInvalidCardId)
        , previousTopCardId(kInvalidCardId)
        , previousArea(CAT_NONE)
        , previousPosition(cocos2d::Vec2::ZERO)
    {
    }
};

#endif // __UNDO_MODEL_H__
