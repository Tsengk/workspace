#include "managers/UndoManager.h"

void UndoManager::init()
{
    // 新开局或重新开始时，直接清空撤销栈。
    _undoStack.clear();
}

void UndoManager::pushUndo(const UndoModel& undoModel)
{
    // 每次成功移动都按时间顺序压栈，后续按 LIFO 逆序回退。
    _undoStack.push_back(undoModel);
}

bool UndoManager::canUndo() const
{
    // 栈里只要还有记录，就允许继续回退。
    return !_undoStack.empty();
}

UndoModel UndoManager::popUndo()
{
    if (_undoStack.empty())
    {
        // 返回默认值，调用方会将其视为“没有可回退数据”。
        return UndoModel();
    }

    // 取出最近一次动作，实现“最后一步先回退”。
    const UndoModel undoModel = _undoStack.back();
    _undoStack.pop_back();
    return undoModel;
}
