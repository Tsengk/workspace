#ifndef __CARD_VIEW_H__
#define __CARD_VIEW_H__

#include <functional>

#include "cocos2d.h"

#include "configs/models/CardResConfig.h"
#include "models/CardModel.h"

/**
 * @brief 单张卡牌视图。
 *        负责渲染牌面内容，并把点击事件转发给控制层。
 */
class CardView : public cocos2d::Node
{
public:
    /**
     * @brief 创建卡牌视图。
     * @param cardResConfig 卡牌资源布局配置。
     * @return 自动释放的 CardView 实例。
     */
    static CardView* create(const CardResConfig& cardResConfig);

    /**
     * @brief 初始化卡牌视图。
     * @param cardResConfig 卡牌资源布局配置。
     * @return 是否初始化成功。
     */
    bool init(const CardResConfig& cardResConfig);

    /**
     * @brief 用运行时模型刷新牌面显示。
     * @param cardModel 当前卡牌的运行时数据。
     * @param isCurrentTop 当前是否是顶部牌。
     */
    void applyCard(const CardModel& cardModel, bool isCurrentTop);

    /**
     * @brief 注册点击回调。
     * @param onCardClick 点击后回传 cardId 的回调。
     */
    void setCardClickCallback(const std::function<void(int)>& onCardClick);

private:
    /**
     * @brief 创建牌面子节点。
     */
    void setupView();

    /**
     * @brief 安装触摸监听，把点击转发给上层控制器。
     */
    void setupTouchHandling();

private:
    /** 所有刷新过程共用的卡牌资源配置。 */
    CardResConfig _cardResConfig;
    /** 当前卡牌的运行时 id，会在点击时回传给控制器。 */
    int _cardId;
    /** 当前这张牌是否允许响应点击。 */
    bool _isInteractive;
    /** 牌面背景精灵。 */
    cocos2d::Sprite* _backgroundSprite;
    /** 中央大号点数字精灵。 */
    cocos2d::Sprite* _bigNumberSprite;
    /** 左上角小号点数字精灵。 */
    cocos2d::Sprite* _smallNumberSprite;
    /** 左上角小号花色图标。 */
    cocos2d::Sprite* _smallSuitSprite;
    /** 中央大号花色图标。 */
    cocos2d::Sprite* _centerSuitSprite;
    /** 顶部牌高亮遮罩。 */
    cocos2d::LayerColor* _focusMask;
    /** 父视图注册进来的点击回调。 */
    std::function<void(int)> _onCardClick;
};

#endif // __CARD_VIEW_H__
