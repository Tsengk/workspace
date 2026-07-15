import collections
from typing import List
class Solution:
    def groupAnagrams(self, strs: List[str]) -> List[List[str]]:
        anagram_map = collections.defaultdict(list)

        for s in strs:
            key = "".join(sorted(s))

            anagram_map[key].append(s)

        return list(anagram_map.values())

if __name__ == "__main__":
    solution = Solution()
    strs = ["eat", "tea", "tan", "ate", "nat", "bat"]
    result = solution.groupAnagrams(strs)

    print(result)

    print("[")
    # 既需要用到元素本身，又需要用到它的索引，直接用 enumerate() 就可以了
    # i, group -> 必须是 (索引, 元素) 的顺序
    for i, group in enumerate(result):
        # for word in group -> 'tan''nat'
        # f'"{word}"' -> '"tan"''"nat"'
        # ",".join(f'"{word}"' for word in group) -> '"tan","nat"'
        formatted_group = ",".join(f'"{word}"' for word in group)
        # 判断是否是最后一个元素，如果是最后一个元素就不加逗号，否则加逗号
        end_comma = "," if i < len(result) - 1 else ""

        print(f"  [{formatted_group}]{end_comma}")
    print("]")

