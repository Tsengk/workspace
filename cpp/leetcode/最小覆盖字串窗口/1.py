class Solution:
    def minWindow(self, s: str, t: str) -> str:
        m, n = len(s), len(t)
        if m < n:
            return ""
        
        need = {}
        window = {}
        for c in t:
            need[c] = need.get(c, 0) + 1
        need_type_count = len(need)

        left, right, valid, start = 0, 0, 0, 0
        min_len = m + 1

        while right < m:
            c = s[right]
            right += 1

            if c in need:
                window[c] = window.get(c, 0) + 1
                if window[c] == need[c]:
                    valid += 1
            while valid == need_type_count:
                if right - left < min_len:
                    start = left
                    min_len = right - left
                d = s[left]
                left += 1
                if d in need:
                    if window[d] == need[d]:
                        valid -= 1
                    window[d] -= 1

        return "" if min_len == m + 1 else s[start:start + min_len]#s[起始索引 : 结束索引]

if __name__ == "__main__":
    s = "ADOBECODEBANC"
    t = "ABC"
    solution = Solution()
    result = solution.minWindow(s, t)
    print(result)  # 输出: "BANC"