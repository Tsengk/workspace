#include "models/GameModel.h"
#include <map>

GameModel::GameModel()
    : currentTopCardId(kInvalidCardId)
    , topCardPosition(cocos2d::Vec2::ZERO)
{
}

void GameModel::reset()
{
    cards.clear();
    playfieldCardIds.clear();
    reserveCardIds.clear();
    topHistoryCardIds.clear();
    currentTopCardId = kInvalidCardId;
    topCardPosition = cocos2d::Vec2::ZERO;
}

CardModel* GameModel::findCard(int cardId)
{
    // 运行时通过 cardId 在扁平数组里查卡牌，避免多个层都维护复杂索引。
    for (std::size_t index = 0; index < cards.size(); ++index)
    {
        if (cards[index].cardId == cardId)
        {
            return &cards[index];
        }
    }

    return nullptr;
}

const CardModel* GameModel::findCard(int cardId) const
{
    // const 版本供只读逻辑使用，例如规则判断。
    for (std::size_t index = 0; index < cards.size(); ++index)
    {
        if (cards[index].cardId == cardId)
        {
            return &cards[index];
        }
    }

    return nullptr;
}

void GameModelUtils::refreshPlayfieldInteractivity(GameModel& gameModel)
{
    std::map<int, std::pair<float, int> > topCardByColumn;
    const float midLineX = 540.0f;

    // 先把所有桌牌重置为不可点击，随后只恢复每组最外层那张。
    for (std::size_t index = 0; index < gameModel.cards.size(); ++index)
    {
        CardModel& cardModel = gameModel.cards[index];
        if (cardModel.area == CAT_PLAYFIELD)
        {
            cardModel.isInteractive = false;

            // 当前演示固定只有左右两组桌牌，因此按左右半区分组即可。
            const int columnKey = cardModel.currentPosition.x < midLineX ? 0 : 1;
            std::map<int, std::pair<float, int> >::iterator it = topCardByColumn.find(columnKey);
            // y 越小越靠下，也就是当前视觉上最外层、应该能点到的牌。
            if (it == topCardByColumn.end() || cardModel.currentPosition.y < it->second.first)
            {
                topCardByColumn[columnKey] = std::make_pair(cardModel.currentPosition.y, cardModel.cardId);
            }
        }
    }

    for (std::map<int, std::pair<float, int> >::const_iterator it = topCardByColumn.begin();
         it != topCardByColumn.end();
         ++it)
    {
        CardModel* cardModel = gameModel.findCard(it->second.second);
        if (cardModel != nullptr)
        {
            // 每组只放开一张牌的交互，这样被压住的牌不会提前被点到。
            cardModel->isInteractive = true;
        }
    }
}
