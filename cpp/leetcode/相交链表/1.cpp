#include <iostream>

struct ListNode {
    int val;
    ListNode *next;
    ListNode(int x) : val(x), next(NULL) {}
};

class Solution {
public:
    ListNode* getIntersectionNode(ListNode *headA, ListNode *headB) {
        if (!headA || !headB) return nullptr;

        ListNode *pA = headA;
        ListNode *pB = headB;

        while (pA != pB) {
            pA = (pA == nullptr) ? headB : pA -> next;
            pB = (pB == nullptr) ? headA : pB -> next;
        }
        return pA;
    }
};

int main() {
    Solution solution;

    //公共：8 -> 4 -> 5
    ListNode *c1 = new ListNode(8);
    ListNode *c2 = new ListNode(4);
    ListNode *c3 = new ListNode(5);
    c1->next = c2;
    c2->next = c3;

    //链表A：4 -> 1 -> 8 -> 4 -> 5
    ListNode *a1 = new ListNode(4);
    ListNode *a2 = new ListNode(1);
    a1->next = a2;
    a2->next = c1;

    //链表B：5 -> 0 -> 1 -> 8 -> 4 -> 5
    ListNode *b1 = new ListNode(5);
    ListNode *b2 = new ListNode(0);
    ListNode *b3 = new ListNode(1);
    b1->next = b2;
    b2->next = b3;
    b3->next = c1;

    ListNode *intersection = solution.getIntersectionNode(a1, b1);
    if (intersection != nullptr) {
        std::cout << "相交节点的值：" << intersection->val << std::endl;
        std::cout << "相交节点实际内存地址：" << intersection << std::endl;
    } else {
        std::cout << "链表不相交" << std::endl;
    }
    delete c1;
    delete c2;
    delete c3;
    delete a1;
    delete a2;
    delete b1;
    delete b2;
    delete b3;

    return 0;
}