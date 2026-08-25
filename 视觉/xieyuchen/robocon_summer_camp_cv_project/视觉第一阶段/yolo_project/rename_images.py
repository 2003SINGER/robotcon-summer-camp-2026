import os
from pathlib import Path

img_dir = Path('raw_imgs')

# 支持多种图片格式
extensions = ('.jpg', '.jpeg', '.png', '.bmp')

# 获取所有图片文件（按原文件名排序，保证顺序一致）
img_files = sorted([f for f in img_dir.iterdir() if f.suffix.lower() in extensions])

if not img_files:
    print("没有找到图片文件！")
    exit()

# 可选：生成一个旧名->新名的映射表，便于追溯
mapping_file = Path('rename_mapping.txt')
with open(mapping_file, 'w') as f:
    f.write("旧文件名\t新文件名\n")
    for idx, img_path in enumerate(img_files, start=1):
        new_name = f"img_{idx:04d}{img_path.suffix}"  # 例如 img_0001.jpg
        new_path = img_dir / new_name

        # 如果新名已存在，自动跳过（避免覆盖）
        #if new_path.exists():
         #   print(f"警告：{new_name} 已存在，跳过 {img_path.name}")
         #   continue

        # 重命名
        img_path.rename(new_path)
        f.write(f"{img_path.name}\t{new_name}\n")
        print(f"重命名：{img_path.name} -> {new_name}")

print(f"重命名完成！共处理 {len(img_files)} 张图片。")
print(f"新旧文件名映射已保存到 {mapping_file}")
