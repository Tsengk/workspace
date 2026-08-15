from typing import List

class Solution:
    def merge(self, intervals: List[int]) -> List[int]:
        intervals.sort()
        result = []

        for interval in intervals:
            if not result or interval[0] > result[-1][1]:
                result.append(interval)
            else:
                result[-1][1] = max(result[-1][1], interval[1])
        return result


if __name__ =="__main__":
    solution = Solution()
    intervals = [[1, 3], [2, 6], [8, 10], [15, 18]]
    result = solution.merge(intervals)
    print(result)