#include "services/GameModelFromLevelGenerator.h"

namespace
{
// 把静态关卡中的单张牌转换成运行时模型。
CardModel buildCardModel(int cardId,
                         const LevelCardConfig& cardConfig,
                         CardAreaType area,
                         bool isInteractive)
{
    CardModel model;
    model.cardId = cardId;
    model.face = cardConfig.face;
    model.suit = cardConfig.suit;
    model.area = area;
    model.homePosition = cardConfig.position;
    model.currentPosition = cardConfig.position;
    model.isFaceUp = true;
    model.isInteractive = isInteractive;
    return model;
}
} // namespace

GameModel GameModelFromLevelGenerator::generateGameModel(const LevelConfig& levelConfig) const
{
    GameModel gameModel;

    int nextCardId = 0;
    // 1. 先生成桌牌区，初始都属于 Playfield。
    for (std::size_t index = 0; index < levelConfig.playfieldCards.size(); ++index)
    {
        CardModel cardModel = buildCardModel(nextCardId, levelConfig.playfieldCards[index], CAT_PLAYFIELD, true);
        gameModel.playfieldCardIds.push_back(cardModel.cardId);
        gameModel.cards.push_back(cardModel);
        ++nextCardId;
    }
    // 2. 再生成底部牌区：
    //    第一张作为顶部牌，其余作为可点击的手牌区牌。
    for (std::size_t index = 0; index < levelConfig.stackCards.size(); ++index)
    {
        const bool isTopCard = index == 0;
        CardModel cardModel = buildCardModel(nextCardId,
                                             levelConfig.stackCards[index],
                                             isTopCard ? CAT_TOP : CAT_STACK_RESERVE,
                                             !isTopCard);
        if (isTopCard)
        {
            // 顶部牌位置会作为所有匹配/替换动画的目标点。
            gameModel.currentTopCardId = cardModel.cardId;
            gameModel.topCardPosition = cardModel.currentPosition;
        }
        else
        {
            gameModel.reserveCardIds.push_back(cardModel.cardId);
        }
        gameModel.cards.push_back(cardModel);
        ++nextCardId;
    }
    // 3. 根据当前桌牌布局，刷新每组真正露在最外层、允许点击的牌。
    GameModelUtils::refreshPlayfieldInteractivity(gameModel);
    return gameModel;
}
