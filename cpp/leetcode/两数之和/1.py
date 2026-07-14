from typing import List
class Solution:
    def twoSum(self, nums: List[int], target: int) -> List[int]:
        if not nums:
            return []

        map = {}
        for i, num in enumerate(nums):
            if target - num in map:
                return [map[target - num], i]
            map[num] = i
        return []

if __name__ == "__main__":
    nums = [2, 11, 7, 15]
    target = 9

    solution = Solution()
    result = solution.twoSum(nums, target)

    if result:
        print(f"找到匹配的下标: {result}")
        print(f"对应的数字是: {nums[result[0]]} 和 {nums[result[1]]}")
    else:
        print("没有找到匹配的下标")