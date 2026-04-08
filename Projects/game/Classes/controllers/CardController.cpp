#include "controllers/CardController.h"

#include "services/MatchRuleService.h"

CardController::CardController()
    : _undoManager(nullptr)
{
}

void CardController::init(UndoManager* undoManager)
{
    _undoManager = undoManager;
}

bool CardController::handleCardClick(GameModel& gameModel, int cardId, CardMoveResult& outResult)
{
    // 点击入口统一先校验：牌必须存在、正面朝上，并且当前允许交互。
    CardModel* targetCard = gameModel.findCard(cardId);
    if (targetCard == nullptr || !targetCard->isFaceUp || !targetCard->isInteractive)
    {
        return false;
    }

    // 根据牌所在区域决定走“手牌替换”还是“桌牌匹配”逻辑。
    switch (targetCard->area)
    {
    case CAT_STACK_RESERVE:
        return moveReserveCardToTop(gameModel, *targetCard, outResult);
    case CAT_PLAYFIELD:
        return movePlayfieldCardToTop(gameModel, *targetCard, outResult);
    default:
        return false;
    }
}

bool CardController::handleUndo(GameModel& gameModel, CardMoveResult& outResult)
{
    // 没有撤销管理器或栈已空时，说明当前无法继续回退。
    if (_undoManager == nullptr || !_undoManager->canUndo())
    {
        return false;
    }

    // 取出最近一次移动记录，恢复“移动牌”和“上一个顶部牌”的状态。
    const UndoModel undoModel = _undoManager->popUndo();
    CardModel* movedCard = gameModel.findCard(undoModel.movedCardId);
    CardModel* previousTopCard = gameModel.findCard(undoModel.previousTopCardId);
    if (movedCard == nullptr || previousTopCard == nullptr)
    {
        return false;
    }

    // 告诉视图层：本次回退动画应该从顶部飞回原来的位置。
    outResult.success = true;
    outResult.movedCardId = movedCard->cardId;
    outResult.fromPosition = movedCard->currentPosition;
    outResult.toPosition = undoModel.previousPosition;

    // 被移动到顶部的牌回到原区域和原坐标。
    movedCard->area = undoModel.previousArea;
    movedCard->currentPosition = undoModel.previousPosition;
    movedCard->isInteractive = true;

    // 回退后，原顶部牌重新成为当前顶部牌。
    previousTopCard->area = CAT_TOP;
    previousTopCard->isInteractive = false;
    gameModel.currentTopCardId = previousTopCard->cardId;

    if (!gameModel.topHistoryCardIds.empty())
    {
        // 顶部历史和撤销栈一一对应，回退一步就弹出一层历史显示。
        gameModel.topHistoryCardIds.pop_back();
    }

    // 回退后需要重新计算桌牌最外层可点击状态。
    GameModelUtils::refreshPlayfieldInteractivity(gameModel);
    return true;
}

bool CardController::moveReserveCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult)
{
    return moveCardToTop(gameModel, targetCard, outResult);
}

bool CardController::movePlayfieldCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult)
{
    const CardModel* currentTopCard = gameModel.findCard(gameModel.currentTopCardId);
    if (currentTopCard == nullptr)
    {
        return false;
    }

    // 桌牌必须先满足“与顶部牌点数相差 1”的规则，才允许替换顶部牌。
    if (!MatchRuleService::canMatch(currentTopCard->face, targetCard.face))
    {
        return false;
    }

    return moveCardToTop(gameModel, targetCard, outResult);
}

bool CardController::moveCardToTop(GameModel& gameModel, CardModel& targetCard, CardMoveResult& outResult)
{
    if (_undoManager == nullptr)
    {
        return false;
    }

    // 当前顶部牌不存在，或点击的就是顶部牌本身，都不需要处理。
    CardModel* currentTopCard = gameModel.findCard(gameModel.currentTopCardId);
    if (currentTopCard == nullptr || currentTopCard->cardId == targetCard.cardId)
    {
        return false;
    }

    // 1. 先记录撤销数据，保证本次移动可以完整回滚。
    UndoModel undoModel;
    undoModel.movedCardId = targetCard.cardId;
    undoModel.previousTopCardId = currentTopCard->cardId;
    undoModel.previousArea = targetCard.area;
    undoModel.previousPosition = targetCard.currentPosition;
    _undoManager->pushUndo(undoModel);

    // 2. 产出给视图层的动画参数：从原位置飞向顶部牌位置。
    outResult.success = true;
    outResult.movedCardId = targetCard.cardId;
    outResult.fromPosition = targetCard.currentPosition;
    outResult.toPosition = gameModel.topCardPosition;

    // 3. 旧顶部牌转入历史层，用于形成“顶部牌堆叠”的视觉效果。
    currentTopCard->area = CAT_TOP_HISTORY;
    currentTopCard->isInteractive = false;
    gameModel.topHistoryCardIds.push_back(currentTopCard->cardId);

    // 4. 被点击的牌成为新的顶部牌。
    targetCard.area = CAT_TOP;
    targetCard.currentPosition = gameModel.topCardPosition;
    targetCard.isInteractive = false;

    // 5. 替换完成后重新计算桌牌区哪些牌可以继续点击。
    gameModel.currentTopCardId = targetCard.cardId;
    GameModelUtils::refreshPlayfieldInteractivity(gameModel);
    return true;
}
