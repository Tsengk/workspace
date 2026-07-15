#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

using namespace std;

class Solution {
public:
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        unordered_map<string, vector<string>> anagram_map;
        // const string& s 不修改原字符串
        for (const string& s : strs) {
            string key = s;
            // 对字符串进行排序，key="eat"->key="aet",key="tae"->key="aet",key="ate"->key="aet"
            sort(key.begin(), key.end());
            // unordered_map(哈希表)查找与自动创建
            // 如果key存在在哈希表中，则返回与之对应的"value"的引用
            // 如果key不存在在哈希表中，则创建一个新的key-value对，键为当前的key,value为vector<string>类型的空向量，并返回该空向量的引用
            anagram_map[key].push_back(s);
            //{"abt": ["bat"], "ant": ["tan", "nat"], "aet": ["eat", "tea", "ate"]}
        }
        vector<vector<string>> result;
        for (auto& pair : anagram_map) {
            result.push_back(pair.second);
            // result = [["bat"], ["tan", "nat"], ["eat", "tea", "ate"]]
        }
        return result;
    }
};

int main() {
    Solution solution;
    vector<string> strs = {"eat", "tea", "tan", "ate", "nat", "bat"};
    vector<vector<string>> result = solution.groupAnagrams(strs);

    cout << "[";
    for (size_t i = 0; i < result.size(); ++i) {
        cout << " [";
        for (size_t j = 0; j < result[i].size(); ++j) {
            cout << "\"" << result[i][j] << "\"";
            if (j < result[i].size() -1) cout << ", ";
        }
        cout << "]";
        if (i < result.size() - 1) cout << ",";
    }
    cout << "]" << endl;

    return 0;
}