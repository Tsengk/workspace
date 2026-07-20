from typing import List

class Solution:
    def findAnagrams(self, s: str, p: str) -> List[int]:
        s_len , p_len = len(s), len(p)
        if s_len < p_len:
            return []

        p_count = [0]*26
        s_count = [0]*26
        ans = []

        for i in range(p_len):
            p_count[ord(p[i]) - ord('a')] += 1
            s_count[ord(s[i]) - ord('a')] += 1
        if p_count == s_count:
            ans.append(0)

        for i in range(s_len - p_len):
            s_count[ord(s[i]) - ord('a')] -= 1
            s_count[ord(s[i + p_len]) - ord('a')] += 1

            if p_count == s_count:
                ans.append(i + 1)
        return ans

if __name__ == "__main__":
    solution = Solution()
    s = "cbaebabacd"
    p = "abc"

    print(f"{solution.findAnagrams(s, p)}")