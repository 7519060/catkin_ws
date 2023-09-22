#!/usr/bin/env python
#coding: UTF-8

import random

# 1つ目の要素
element1 = random.uniform(1.545, 1.748)
while abs(element1 - 1.678) < 0.01:
    element1 = random.uniform(1.545, 1.748)

# 2つ目の要素
element2 = random.uniform(-1.705, -1.511)
while abs(element2 - (-1.607)) < 0.01:
    element2 = random.uniform(-1.705, -1.511)

# 3つ目の要素
element3 = random.uniform(1.769, 1.955)
while abs(element3 - 1.867) < 0.01:
    element3 = random.uniform(1.769, 1.955)

# 4つ目の要素
element4 = random.uniform(-0.295, -0.197)
while abs(element4 - (-0.245)) < 0.005:
    element4 = random.uniform(-0.295, -0.197)

# 5つ目の要素
element5 = random.uniform(1.251, 1.503)
while abs(element5 - 1.383) < 0.01:
    element5 = random.uniform(1.251, 1.503)

# 6つ目の要素
element6 = random.uniform(3.157, 3.162)
while abs(element6 - 3.159) < 0.00025:
    element6 = random.uniform(3.157, 3.162)

# リストの生成
result_list = [element1, element2, element3, element4, element5, element6]

# 結果の出力
print(result_list)
