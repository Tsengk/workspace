#include "controllers/GameController.h"

#include "cocos2d.h"

#include "configs/loaders/LevelConfigLoader.h"
#include "configs/models/CardResConfig.h"
#include "services/GameModelFromLevelGenerator.h"
#include "views/GameView.h"

GameController::GameController()
    : _parent(nullptr)
    , _gameView(nullptr)
    , _isAnimating(false)
{
}

bool GameController::init(cocos2d::Node* parent)
{
    _parent = parent;
    _undoManager.init();
    _cardController.init(&_undoManager);
    return _parent != nullptr;
}

bool GameController::startGame(const std::string& levelConfigPath)
{
    LevelConfig levelConfig;
    std::string errorMessage;
    // 1. 先加载静态关卡配置。
    if (!LevelConfigLoader::loadLevelConfig(levelConfigPath, levelConfig, errorMessage))
    {
        cocos2d::log("Failed to load level config: %s", errorMessage.c_str());
        return false;
    }

    // 2. 用服务层把静态配置转换成运行时模型。
    _undoManager.init();
    GameModelFromLevelGenerator generator;
    _gameModel = generator.generateGameModel(levelConfig);

    if (_gameView != nullptr)
    {
        // 重开关卡时，先移除旧视图，避免叠加。
        _gameView->removeFromParent();
        _gameView = nullptr;
    }

    // 3. 创建视图并把点击事件绑定回控制器。
    _gameView = GameView::create(CardResConfig::createDefault());
    if (_gameView == nullptr)
    {
        return false;
    }

    _gameView->setCardClickCallback([this](int cardId)
    {
        handleCardClick(cardId);
    });
    _gameView->setUndoClickCallback([this]()
    {
        handleUndoClick();
    });
    // 4. 根据当前模型生成整套 UI。
    _gameView->buildView(_gameModel);
    _parent->addChild(_gameView);
    return true;
}

void GameController::handleCardClick(int cardId)
{
    // 动画播放期间不允许重复点击，避免模型和视图状态错位。
    if (_isAnimating || _gameView == nullptr)
    {
        return;
    }

    CardMoveResult moveResult;
    // 先由 CardController 改模型并产出本次移动的动画参数。
    if (!_cardController.handleCardClick(_gameModel, cardId, moveResult))
    {
        _gameView->showMessage(u8"\u53ea\u6709\u624b\u724c\u533a\u5361\u724c\uff0c\u6216\u4e0e\u9876\u90e8\u724c\u70b9\u6570\u76f8\u5dee 1 \u7684\u684c\u724c\uff0c\u624d\u53ef\u4ee5\u79fb\u52a8\u3002");
        return;
    }

    _isAnimating = true;
    // 先把界面刷新到“移动后的逻辑状态”，但把当前动的牌暂时钉在旧位置。
    _gameView->syncViewKeepingCard(_gameModel, moveResult.movedCardId, moveResult.fromPosition);
    // 再播放实际移动动画，动画完成后做一次最终同步。
    _gameView->playCardMoveAnimation(moveResult.movedCardId, moveResult.toPosition, [this]()
    {
        if (_gameView != nullptr)
        {
            _gameView->syncView(_gameModel);
            _gameView->showMessage(u8"\u5df2\u8bb0\u5f55\u672c\u6b21\u79fb\u52a8\uff0c\u70b9\u51fb\u201c\u56de\u9000\u201d\u53ef\u9006\u5e8f\u8fd8\u539f\u3002");
        }
        _isAnimating = false;
    });
}

void GameController::handleUndoClick()
{
    // 动画期间禁止再次回退，避免多次并发修改模型。
    if (_isAnimating || _gameView == nullptr)
    {
        return;
    }

    CardMoveResult moveResult;
    // 先让 CardController 恢复模型，再把回退路径交给视图层动画。
    if (!_cardController.handleUndo(_gameModel, moveResult))
    {
        _gameView->showMessage(u8"\u5f53\u524d\u6ca1\u6709\u53ef\u56de\u9000\u7684\u8bb0\u5f55\u3002");
        return;
    }

    _isAnimating = true;
    _gameView->syncViewKeepingCard(_gameModel, moveResult.movedCardId, moveResult.fromPosition);
    _gameView->playCardMoveAnimation(moveResult.movedCardId, moveResult.toPosition, [this]()
    {
        if (_gameView != nullptr)
        {
            _gameView->syncView(_gameModel);
            _gameView->showMessage(u8"\u56de\u9000\u5b8c\u6210\u3002");
        }
        _isAnimating = false;
    });
}
