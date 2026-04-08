#include "views/CardView.h"

#include <new>

#include "utils/CardUtils.h"

namespace
{
// 切换数字或花色资源时，统一走这个函数刷新纹理和显示区域。
void updateSpriteTexture(cocos2d::Sprite* sprite, const std::string& resourcePath)
{
    if (sprite == nullptr)
    {
        return;
    }

    sprite->setTexture(resourcePath);
    if (sprite->getTexture() != nullptr)
    {
        const cocos2d::Size textureSize = sprite->getTexture()->getContentSize();
        sprite->setTextureRect(cocos2d::Rect(0.0f, 0.0f, textureSize.width, textureSize.height));
    }
}
} // namespace

CardView* CardView::create(const CardResConfig& cardResConfig)
{
    CardView* cardView = new (std::nothrow) CardView();
    if (cardView != nullptr && cardView->init(cardResConfig))
    {
        cardView->autorelease();
        return cardView;
    }

    CC_SAFE_DELETE(cardView);
    return nullptr;
}

bool CardView::init(const CardResConfig& cardResConfig)
{
    if (!Node::init())
    {
        return false;
    }

    // 这些成员在运行时会随着 applyCard 不断被覆盖，因此先初始化成安全默认值。
    _cardResConfig = cardResConfig;
    _cardId = kInvalidCardId;
    _isInteractive = false;
    _backgroundSprite = nullptr;
    _bigNumberSprite = nullptr;
    _smallNumberSprite = nullptr;
    _smallSuitSprite = nullptr;
    _centerSuitSprite = nullptr;
    _focusMask = nullptr;

    setAnchorPoint(cocos2d::Vec2::ANCHOR_MIDDLE);
    setCascadeOpacityEnabled(true);
    // 视图初始化分两步：先搭静态节点，再挂触摸回调。
    setupView();
    setupTouchHandling();
    return true;
}

void CardView::applyCard(const CardModel& cardModel, bool isCurrentTop)
{
    // 同步当前卡牌 id 和交互开关，触摸回调会用到这两个状态。
    _cardId = cardModel.cardId;
    _isInteractive = cardModel.isInteractive;

    // 顶部牌微微放大，历史顶部牌稍微半透明，方便用户区分层级。
    setVisible(cardModel.isFaceUp);
    setScale(isCurrentTop ? 1.03f : 1.0f);
    setOpacity(cardModel.area == CAT_TOP_HISTORY ? 220 : 255);

    if (_focusMask != nullptr)
    {
        _focusMask->setVisible(isCurrentTop);
    }

    // 根据运行时模型重新组合牌面资源。
    updateSpriteTexture(_bigNumberSprite, CardUtils::getBigNumberRes(cardModel.face, cardModel.suit));
    updateSpriteTexture(_smallNumberSprite, CardUtils::getSmallNumberRes(cardModel.face, cardModel.suit));
    updateSpriteTexture(_smallSuitSprite, CardUtils::getSuitRes(cardModel.suit));
    updateSpriteTexture(_centerSuitSprite, CardUtils::getSuitRes(cardModel.suit));

    if (_centerSuitSprite != nullptr)
    {
        _centerSuitSprite->setScale(_cardResConfig.centerSuitScale);
    }
}

void CardView::setCardClickCallback(const std::function<void(int)>& onCardClick)
{
    _onCardClick = onCardClick;
}

void CardView::setupView()
{
    // 1. 创建统一背景。
    _backgroundSprite = cocos2d::Sprite::create(_cardResConfig.cardBackgroundPath);
    if (_backgroundSprite == nullptr)
    {
        return;
    }

    setContentSize(_backgroundSprite->getContentSize());
    _backgroundSprite->setPosition(getContentSize() * 0.5f);
    addChild(_backgroundSprite);

    // 2. 创建顶部牌高亮遮罩，只在当前顶部牌上显示。
    _focusMask = cocos2d::LayerColor::create(cocos2d::Color4B(247, 211, 94, 36),
                                             static_cast<int>(getContentSize().width - 16.0f),
                                             static_cast<int>(getContentSize().height - 16.0f));
    _focusMask->setPosition(cocos2d::Vec2(8.0f, 8.0f));
    _focusMask->setVisible(false);
    addChild(_focusMask, 1);

    // 3. 预创建牌面各个子部件，后续通过 applyCard 换纹理即可。
    _bigNumberSprite = cocos2d::Sprite::create("res/number/big_black_A.png");
    _smallNumberSprite = cocos2d::Sprite::create("res/number/small_black_A.png");
    _smallSuitSprite = cocos2d::Sprite::create("res/suits/club.png");
    _centerSuitSprite = cocos2d::Sprite::create("res/suits/club.png");

    if (_bigNumberSprite != nullptr)
    {
        _bigNumberSprite->setPosition(_cardResConfig.bigNumberPosition);
        addChild(_bigNumberSprite, 2);
    }

    if (_smallNumberSprite != nullptr)
    {
        _smallNumberSprite->setPosition(_cardResConfig.smallNumberPosition);
        addChild(_smallNumberSprite, 2);
    }

    if (_smallSuitSprite != nullptr)
    {
        _smallSuitSprite->setPosition(_cardResConfig.smallSuitPosition);
        addChild(_smallSuitSprite, 2);
    }

    if (_centerSuitSprite != nullptr)
    {
        _centerSuitSprite->setPosition(_cardResConfig.centerSuitPosition);
        _centerSuitSprite->setScale(_cardResConfig.centerSuitScale);
        addChild(_centerSuitSprite, 2);
    }
}

void CardView::setupTouchHandling()
{
    cocos2d::EventListenerTouchOneByOne* touchListener = cocos2d::EventListenerTouchOneByOne::create();
    touchListener->setSwallowTouches(true);

    touchListener->onTouchBegan = [this](cocos2d::Touch* touch, cocos2d::Event*) -> bool
    {
        // 不可点击、没有回调或当前不可见时，直接忽略触摸。
        if (!_isInteractive || _onCardClick == nullptr || !isVisible())
        {
            return false;
        }

        // 命中检测统一使用卡牌矩形区域。
        const cocos2d::Vec2 localPoint = convertToNodeSpaceAR(touch->getLocation());
        const cocos2d::Rect hitRect(-getContentSize().width * 0.5f,
                                    -getContentSize().height * 0.5f,
                                    getContentSize().width,
                                    getContentSize().height);
        return hitRect.containsPoint(localPoint);
    };

    touchListener->onTouchEnded = [this](cocos2d::Touch* touch, cocos2d::Event*) -> void
    {
        const cocos2d::Vec2 localPoint = convertToNodeSpaceAR(touch->getLocation());
        const cocos2d::Rect hitRect(-getContentSize().width * 0.5f,
                                    -getContentSize().height * 0.5f,
                                    getContentSize().width,
                                    getContentSize().height);
        if (hitRect.containsPoint(localPoint) && _onCardClick != nullptr)
        {
            // 只把 cardId 往上抛给控制层，具体规则由控制器负责判断。
            _onCardClick(_cardId);
        }
    };

    _eventDispatcher->addEventListenerWithSceneGraphPriority(touchListener, this);
}
