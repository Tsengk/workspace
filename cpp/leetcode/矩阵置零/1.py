from typing import List

class Solution:
    def setZeroes(self, matrix: List[List[int]]) -> List[List[int]]:
        m, n = len(matrix), len(matrix[0])
        row0, col0 = False, False

        for i in range(m):
            if matrix[i][0] == 0:
                col0 = True
                break
        for j in range(n):
            if matrix[0][j] == 0:
                row0 = True
                break

        for i in range(1,m):
            for j in range(1,n):
                if matrix[i][j] == 0:
                    matrix[i][0] = 0
                    matrix[0][j] = 0
        for i in range(1,m):
            for j in range(1,n):
                if matrix[i][0] == 0 or matrix[0][j] == 0:
                    matrix[i][j] = 0

        if row0:
            for j in range(n):
                matrix[0][j] = 0

        if col0:
            for i in range(m):
                matrix[i][0] = 0

        return matrix

if __name__ == "__main__":
    solution = Solution()
    matrix = [[0,1,2,0],[3,4,5,2],[1,3,1,5]]
    result = solution.setZeroes(matrix)
    print(result)