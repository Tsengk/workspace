from typing import List

class Solution:
    def longestConsecutive(self, nums: List[int]) -> int:
        nums_set = set(nums)
        longest_streak = 0

        for num in nums_set:
            if num - 1 not in nums_set:
                current_num = num
                current_streak = 1

                while current_num + 1 in nums_set:
                    current_num += 1
                    current_streak += 1

                longest_streak = max(longest_streak, current_streak)
        return longest_streak

if __name__ == "__main__":
    solution = Solution()
    nums = [100, 4, 200, 1, 3, 2]
    print(f"输出：{solution.longestConsecutive(nums)}")