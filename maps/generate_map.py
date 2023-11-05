import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# 创建一个20x20的全白地图
map = np.ones((20, 20))
resolution = 10

# 在地图上创建一个7x7的房间（我们将房间的墙的像素值设为0，表示黑色）
up = 4
down = 15
left = 9
right = 18
map[up:down, left:right] = 1  # 房间内部

# map[6, 9:16] = map[12, 9:16] = map[6:13, 9] = map[6:13, 16] = 0  # 房间的墙
map[up, left:right + 1] = map[down, left:right + 1] = map[up:down + 1, left] = map[up:down + 1, right] = 0  # 房间的墙

## 走廊右边
map[1:18,9] = 0

# # 在房间上创建一个1像素的门（我们将门的像素值设为1，表示白色）
map[12:14, 9] = 1

# # 创建一个4像素宽度，15像素长的走廊
# map[2:18, 8:12] = 1  # 走廊内部
# map[2, 8:12] = map[17, 8:12] = map[2:18, 8] = map[2:18, 11] = 0  # 走廊的墙

map[1:18,5] = 0

upsampled_map = np.repeat(np.repeat(map, resolution, axis=0), resolution, axis=1)

# 使用matplotlib显示地图
plt.imshow(upsampled_map, cmap='gray')
plt.show()

# 使用PIL保存地图为图片
img = Image.fromarray((upsampled_map * 255).astype(np.uint8))
img.save('./map_1m_to_'+str(resolution)+'pixel.png')
