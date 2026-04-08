#include "views/GameView.h"

#include <new>

#include "views/CardView.h"

namespace
{
const float kAnimationDuration = 0.22f;
const float kStackDividerY = 580.0f;
const char* kUiFontName = "Microsoft YaHei";

// 统一创建中文 UI 文本，优先系统字体，失败时再回退到资源字体。
cocos2d::Label* createUiLabel(const std::string& text, float fontSize)
{
    cocos2d::Label* label = cocos2d::Label::createWithSystemFont(text, kUiFontName, fontSize);
    if (label == nullptr)
    {
        label = cocos2d::Label::createWithTTF(text, "fonts/arial.ttf", fontSize);
    }

    return label;
}

int findHistoryIndex(const GameModel& gameModel, int cardId)
{
    // 顶部历史牌用数组维护顺序，这里返回其在历史堆里的层号。
    for (std::size_t index = 0; index < gameModel.topHistoryCardIds.size(); ++index)
    {
        if (gameModel.topHistoryCardIds[index] == cardId)
        {
            return static_cast<int>(index);
        }
    }

    return -1;
}

} // namespace

GameView* GameView::create(const CardResConfig& cardResConfig)
{
    GameView* gameView = new (std::nothrow) GameView();
    if (gameView != nullptr && gameView->init(cardResConfig))
    {
        gameView->autorelease();
        return gameView;
    }

    CC_SAFE_DELETE(gameView);
    return nullptr;
}

bool GameView::init(const CardResConfig& cardResConfig)
{
    if (!Layer::init())
    {
        return false;
    }

    _cardResConfig = cardResConfig;
    _cardContainer = nullptr;
    _hintLabel = nullptr;

    setContentSize(cocos2d::Size(1080.0f, 2080.0f));
    setupLayout();
    return true;
}

void GameView::buildView(const GameModel& gameModel)
{
    // 重新建视图时先清空旧节点，避免重复创建。
    _cardContainer->removeAllChildren();
    _cardViews.clear();

    // 为模型中的每张牌创建一个 CardView，并保存在映射表里。
    for (std::size_t index = 0; index < gameModel.cards.size(); ++index)
    {
        CardView* cardView = CardView::create(_cardResConfig);
        if (cardView == nullptr)
        {
            continue;
        }

        cardView->setCardClickCallback(_onCardClick);
        _cardContainer->addChild(cardView);
        _cardViews[gameModel.cards[index].cardId] = cardView;
    }

    syncView(gameModel);
}

void GameView::syncView(const GameModel& gameModel)
{
    // 全量同步：遍历运行时模型，把位置、层级和牌面都刷新到视图。
    for (std::size_t index = 0; index < gameModel.cards.size(); ++index)
    {
        std::unordered_map<int, CardView*>::const_iterator it = _cardViews.find(gameModel.cards[index].cardId);
        if (it != _cardViews.end() && it->second != nullptr)
        {
            syncSingleCard(gameModel, gameModel.cards[index], it->second);
        }
    }
}

void GameView::syncViewKeepingCard(const GameModel& gameModel, int movingCardId, const cocos2d::Vec2& startPosition)
{
    // 先让所有静态状态更新到最新，再把正在动的那张牌强行摆回起点。
    syncView(gameModel);

    std::unordered_map<int, CardView*>::iterator it = _cardViews.find(movingCardId);
    if (it != _cardViews.end() && it->second != nullptr)
    {
        it->second->setPosition(startPosition);
    }
}

void GameView::playCardMoveAnimation(int cardId, const cocos2d::Vec2& targetPosition, const std::function<void()>& onComplete)
{
    std::unordered_map<int, CardView*>::iterator it = _cardViews.find(cardId);
    if (it == _cardViews.end() || it->second == nullptr)
    {
        if (onComplete)
        {
            onComplete();
        }

        return;
    }

    // 每次只保留当前一段移动动画，避免重复点击造成动作叠加。
    CardView* cardView = it->second;
    cardView->stopAllActions();
    cardView->runAction(cocos2d::Sequence::create(
        cocos2d::EaseSineInOut::create(cocos2d::MoveTo::create(kAnimationDuration, targetPosition)),
        cocos2d::CallFunc::create([onComplete]()
        {
            if (onComplete)
            {
                onComplete();
            }
        }),
        nullptr));
}

void GameView::setCardClickCallback(const std::function<void(int)>& onCardClick)
{
    _onCardClick = onCardClick;
}

void GameView::setUndoClickCallback(const std::function<void()>& onUndoClick)
{
    _onUndoClick = onUndoClick;
}

void GameView::showMessage(const std::string& message)
{
    if (_hintLabel != nullptr)
    {
        _hintLabel->setString(message);
    }
}

void GameView::setupLayout()
{
    // 上半部分是桌牌区，下半部分是手牌/顶部牌区。
    addChild(cocos2d::LayerColor::create(cocos2d::Color4B(15, 42, 58, 255)), 0);

    cocos2d::LayerColor* playfieldBackground = cocos2d::LayerColor::create(cocos2d::Color4B(25, 91, 109, 255), 1080, 1500);
    playfieldBackground->setPosition(cocos2d::Vec2(0.0f, kStackDividerY));
    addChild(playfieldBackground, 0);

    cocos2d::LayerColor* stackBackground = cocos2d::LayerColor::create(cocos2d::Color4B(11, 53, 68, 255), 1080, 580);
    stackBackground->setPosition(cocos2d::Vec2::ZERO);
    addChild(stackBackground, 0);

    cocos2d::LayerColor* divider = cocos2d::LayerColor::create(cocos2d::Color4B(255, 255, 255, 38), 1080, 4);
    divider->setPosition(cocos2d::Vec2(0.0f, kStackDividerY));
    addChild(divider, 1);

    cocos2d::Label* titleLabel = createUiLabel(u8"\u7eb8\u724c\u5339\u914d\u6f14\u793a", 52.0f);
    if (titleLabel != nullptr)
    {
        titleLabel->setPosition(cocos2d::Vec2(540.0f, 1990.0f));
        titleLabel->setColor(cocos2d::Color3B(247, 243, 231));
        addChild(titleLabel, 2);
    }

    cocos2d::Label* playfieldLabel = createUiLabel(u8"\u684c\u724c\u533a", 38.0f);
    if (playfieldLabel != nullptr)
    {
        playfieldLabel->setAnchorPoint(cocos2d::Vec2::ANCHOR_MIDDLE_LEFT);
        playfieldLabel->setPosition(cocos2d::Vec2(80.0f, 1870.0f));
        playfieldLabel->setColor(cocos2d::Color3B(235, 244, 247));
        addChild(playfieldLabel, 2);
    }

    cocos2d::Label* reserveLabel = createUiLabel(u8"\u624b\u724c\u533a", 34.0f);
    if (reserveLabel != nullptr)
    {
        reserveLabel->setAnchorPoint(cocos2d::Vec2::ANCHOR_MIDDLE_LEFT);
        reserveLabel->setPosition(cocos2d::Vec2(120.0f, 510.0f));
        reserveLabel->setColor(cocos2d::Color3B(235, 244, 247));
        addChild(reserveLabel, 2);
    }

    cocos2d::Label* topLabel = createUiLabel(u8"\u9876\u90e8\u724c", 34.0f);
    if (topLabel != nullptr)
    {
        topLabel->setAnchorPoint(cocos2d::Vec2::ANCHOR_MIDDLE_LEFT);
        topLabel->setPosition(cocos2d::Vec2(640.0f, 510.0f));
        topLabel->setColor(cocos2d::Color3B(235, 244, 247));
        addChild(topLabel, 2);
    }

    _hintLabel = createUiLabel(u8"\u684c\u724c\u70b9\u6570\u4e0e\u9876\u90e8\u724c\u76f8\u5dee 1 \u5373\u53ef\u5339\u914d\uff0c\u70b9\u51fb\u201c\u56de\u9000\u201d\u53ef\u6309\u76f8\u53cd\u987a\u5e8f\u8fd8\u539f\u3002", 28.0f);
    if (_hintLabel != nullptr)
    {
        _hintLabel->setDimensions(900.0f, 96.0f);
        _hintLabel->setAnchorPoint(cocos2d::Vec2::ANCHOR_MIDDLE_LEFT);
        _hintLabel->setPosition(cocos2d::Vec2(80.0f, 42.0f));
        _hintLabel->setColor(cocos2d::Color3B(220, 231, 235));
        addChild(_hintLabel, 2);
    }

    cocos2d::Label* undoLabel = createUiLabel(u8"\u56de\u9000", 44.0f);
    if (undoLabel != nullptr)
    {
        undoLabel->setColor(cocos2d::Color3B(252, 219, 89));
        cocos2d::MenuItemLabel* undoItem = cocos2d::MenuItemLabel::create(undoLabel, [this](cocos2d::Ref*)
        {
            if (_onUndoClick)
            {
                _onUndoClick();
            }
        });
        undoItem->setPosition(cocos2d::Vec2(920.0f, 500.0f));

        cocos2d::Menu* undoMenu = cocos2d::Menu::create(undoItem, nullptr);
        undoMenu->setPosition(cocos2d::Vec2::ZERO);
        addChild(undoMenu, 3);
    }

    // 所有卡牌统一挂在一个容器节点下，方便整体管理和刷新。
    _cardContainer = cocos2d::Node::create();
    addChild(_cardContainer, 10);
}

void GameView::syncSingleCard(const GameModel& gameModel, const CardModel& cardModel, CardView* cardView)
{
    const bool isCurrentTop = cardModel.cardId == gameModel.currentTopCardId;
    cardView->applyCard(cardModel, isCurrentTop);
    cardView->setPosition(getDisplayPosition(gameModel, cardModel));
    cardView->setLocalZOrder(getZOrderForCard(gameModel, cardModel));
}

cocos2d::Vec2 GameView::getDisplayPosition(const GameModel& gameModel, const CardModel& cardModel) const
{
    if (cardModel.area != CAT_TOP_HISTORY)
    {
        return cardModel.currentPosition;
    }

    // 顶部历史牌在同一落点做微小偏移，形成“上一张在下面”的堆叠感。
    const int historyIndex = findHistoryIndex(gameModel, cardModel.cardId);
    if (historyIndex < 0)
    {
        return cardModel.currentPosition;
    }

    return cardModel.currentPosition + cocos2d::Vec2(8.0f * (historyIndex + 1), -6.0f * (historyIndex + 1));
}

int GameView::getZOrderForCard(const GameModel& gameModel, const CardModel& cardModel) const
{
    switch (cardModel.area)
    {
    case CAT_TOP:
        // 当前顶部牌永远处于最上层。
        return 4000;
    case CAT_TOP_HISTORY:
        // 历史顶部牌在顶部牌下方，但仍高于其他区域。
        return 3900 + findHistoryIndex(gameModel, cardModel.cardId);
    case CAT_STACK_RESERVE:
        // 手牌区整体高于桌牌区，避免底部区域被桌牌盖住。
        return 3700 + cardModel.cardId;
    case CAT_PLAYFIELD:
        // 桌牌区按 y 值反向排序：越靠下越显示在更上层。
        return 1000 + static_cast<int>(2000.0f - cardModel.currentPosition.y);
    default:
        return 0;
    }
}
