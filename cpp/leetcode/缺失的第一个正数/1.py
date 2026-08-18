from typing import List

class Solution:
    def firstMissiongPositive(self, nums: List[int]) -> int:
        n = len(nums)

        for i in range(n):
            while nums[i] >= 1 and nums[i] <= n and nums[nums[i] - 1] != nums[i]:
                nums[i], nums[nums[i] - 1] = nums[nums[i] - 1], nums[i]
        
        for i in range(n):
            if nums[i] != i + 1:
                return i + 1

        return n + 1


if __name__ == "__main__":
    solution = Solution()
    nums = [1, 2, -1]
    result = solution.firstMissiongPositive(nums)
    print(result)