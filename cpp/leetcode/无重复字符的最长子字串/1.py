class Solution:
    def lengthOfLongestSubstring(self, s: str) -> int:
        charSet = set()
        max_length = 0
        left = 0

        for right in range(len(s)):
            while s[right] in charSet:
                charSet.remove(s[left])
                left += 1
            charSet.add(s[right])
            max_length = max(max_length, right - left + 1)
        return max_length

if __name__ == "__main__":
    solution = Solution()
    s = "pwwkew"
    print(f"{solution.lengthOfLongestSubstring(s)}")