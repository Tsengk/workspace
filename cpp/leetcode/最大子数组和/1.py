from typing import List

class Solution:
    def maxSubArray(self, nums: List[int]) -> int:
        current_sum, max_sum = nums[0], nums[0]
        for num in nums[1:]:
            current_sum = max(num, current_sum + num)
            max_sum = max(max_sum, current_sum)
        return max_sum

if __name__ == "__main__":
    solution = Solution()
    nums = [-2,1,-3,4,-1,2,1,-5,4]
    print(solution.maxSubArray(nums))  # Output: 6