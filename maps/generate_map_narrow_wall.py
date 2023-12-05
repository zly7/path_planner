import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

def thin_obstacles(map):
    height, width = map.shape
    new_map = np.copy(map)
    to_white = []  # 用于记录需要变为白色的黑色像素位置

    # 遍历地图的每个像素
    for y in range(height):
        for x in range(width):
            if map[y, x] == 0:
                # 检查四周是否有白色
                if (y > 0 and map[y - 1, x] == 1) or \
                   (y < height - 1 and map[y + 1, x] == 1) or \
                   (x > 0 and map[y, x - 1] == 1) or \
                   (x < width - 1 and map[y, x + 1] == 1):
                    to_white.append((y, x))

    # 将记录的黑色像素变为白色
    for y, x in to_white:
        new_map[y, x] = 1

    return new_map

def main():
    # 创建一个20x20的全白地图
    map = np.ones((20, 20))
    resolution =6

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

    map[1:18,2] = 0

    upsampled_map = np.repeat(np.repeat(map, resolution, axis=0), resolution, axis=1)

    upsampled_map1 = thin_obstacles(upsampled_map)
    plt.imshow(upsampled_map1, cmap='gray')
    plt.show()
    upsampled_map2 = thin_obstacles(upsampled_map1)
    plt.imshow(upsampled_map2, cmap='gray')
    plt.show()

    # 使用PIL保存地图为图片
    img = Image.fromarray((upsampled_map2 * 255).astype(np.uint8))
    img.save('./map_thin_1m_to_'+str(resolution)+'pixel.png')
if __name__ == "__main__":
    main()
