class ListNode:
    def __init__(self, x):
        self.val = x
        self.next = None

class Solution:
    def getIntersectionNode(self, headA: ListNode, headB: ListNode) -> ListNode:
        if not headA or not headB:
            return None

        pA = headA
        pB = headB

        # 两指针在交点处相遇，或同时走至末尾的 None
        while pA != pB:
            pA = headB if pA is None else pA.next
            pB = headA if pB is None else pB.next

        return pA

if __name__ == "__main__":
    # 1. 构建公共部分节点: 8 -> 4 -> 5
    c1 = ListNode(8)
    c2 = ListNode(4)
    c3 = ListNode(5)
    c1.next = c2
    c2.next = c3

    # 2. 构建链表 A: 4 -> 1 -> 8(c1)
    a1 = ListNode(4)
    a2 = ListNode(1)
    a1.next = a2
    a2.next = c1

    # 3. 构建链表 B: 5 -> 6 -> 1 -> 8(c1)
    b1 = ListNode(5)
    b2 = ListNode(6)
    b3 = ListNode(1)
    b1.next = b2
    b2.next = b3
    b3.next = c1

    # 4. 调用接口求解
    solution = Solution()
    intersection = solution.getIntersectionNode(a1, b1)

    # 5. 打印验证
    if intersection:
        print(f"相交节点的值: {intersection.val}")
        print(f"相交节点的对象内存地址: {hex(id(intersection))}")
        print(f"验证 c1 的对象内存地址: {hex(id(c1))}")
    else:
        print("链表不相交")