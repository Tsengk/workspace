from typing import List

class Solution:
    def rotate(self, nums: List[int], k: int) -> List[int]:
        n = len(nums)
        if n==0:
            return []

        k %= n
        if k==0:
            return []

        def reverse(left: int, right: int) -> None:
            while left < right:
                nums[left], nums[right] = nums[right], nums[left]
                left += 1
                right -= 1

        reverse(0, n-1)
        reverse(0, k-1)
        reverse(k, n-1)

        return nums

if __name__ == "__main__":
    solution = Solution()
    nums = [1, 2, 3, 4, 5, 6, 7]
    result = solution.rotate(nums, 3)
    print(result)