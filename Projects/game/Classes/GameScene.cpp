#include "GameScene.h"

#include <new>

namespace
{
// 中文界面优先使用系统字体，避免资源字体缺字。
const char* kUiFontName = "Microsoft YaHei";
}

cocos2d::Scene* GameScene::createScene()
{
    return GameScene::create();
}

bool GameScene::init()
{
    if (!Scene::init())
    {
        return false;
    }

    // 场景本身不处理具体业务，只负责创建顶层控制器。
    _gameController.reset(new (std::nothrow) GameController());
    if (_gameController == nullptr || !_gameController->init(this))
    {
        return false;
    }

    // 进入场景后直接加载默认关卡，便于启动后立即进入演示。
    if (!_gameController->startGame("configs/levels/level_1.json"))
    {
        // 配置加载失败时，直接在场景中央给出错误提示，方便排查。
        cocos2d::Label* errorLabel =
            cocos2d::Label::createWithSystemFont(u8"\u5173\u5361 level_1.json \u52a0\u8f7d\u5931\u8d25", kUiFontName, 42);
        if (errorLabel == nullptr)
        {
            errorLabel = cocos2d::Label::createWithTTF("level_1.json load failed", "fonts/arial.ttf", 42);
        }

        if (errorLabel != nullptr)
        {
            errorLabel->setPosition(cocos2d::Vec2(540.0f, 1040.0f));
            addChild(errorLabel);
        }
    }

    return true;
}
