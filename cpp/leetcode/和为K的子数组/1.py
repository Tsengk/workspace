from typing import List

class Solution:
    def subarraySum(self, nums: List[int], k: int) -> int:
        count = 0
        pre_sum = 0
        pre_map = {0: 1}

        for num in nums:
            pre_sum += num
            target = pre_sum - k
            if target in pre_map:
                count += pre_map[target]
            # Python 的标准字典不会像 C++ 那样自动给不存在的键赋初始值 0
            # 所以使用 .get(pre_sum, 0) 获取之前的值（如果不存在则返回 0），然后加 1
            pre_map[pre_sum] = pre_map.get(pre_sum, 0) + 1
        return count

if __name__ == "__main__":
    solution = Solution()
    nums = [1, 1, 1]
    k = 2
    result = solution.subarraySum(nums, k)
    print(result)  # 输出: 2