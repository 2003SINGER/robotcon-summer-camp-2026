import os
import shutil
import random

img_dir = 'raw_imgs'          # 原始图片文件夹
label_dir = 'labels'          # 标注文件夹（.txt）
target_base = 'dataset'       # 目标数据集根目录

# 获取所有图片文件
all_imgs = [f for f in os.listdir(img_dir) if f.endswith('.jpg')]

# 过滤：只保留那些在 labels 中有对应 .txt 的图片
valid_imgs = []
for img in all_imgs:
    label_name = img.replace('.jpg', '.txt')
    label_path = os.path.join(label_dir, label_name)
    if os.path.exists(label_path):
        valid_imgs.append(img)
    else:
        print(f"⚠️ 跳过 {img}，因为找不到标注文件 {label_name}")

print(f"总图片: {len(all_imgs)}，有效图片（有标注）: {len(valid_imgs)}")

if not valid_imgs:
    print("没有有效图片，退出。")
    exit()

# 随机打乱
random.shuffle(valid_imgs)
split_idx = int(len(valid_imgs) * 0.8)

# 确保目标文件夹存在（自动创建）
for sub in ['images/train', 'images/val', 'labels/train', 'labels/val']:
    os.makedirs(os.path.join(target_base, sub), exist_ok=True)

# 复制文件
for i, img in enumerate(valid_imgs):
    label = img.replace('.jpg', '.txt')
    img_src = os.path.join(img_dir, img)
    label_src = os.path.join(label_dir, label)
    
    if i < split_idx:
        shutil.copy(img_src, os.path.join(target_base, 'images/train/'))
        shutil.copy(label_src, os.path.join(target_base, 'labels/train/'))
    else:
        shutil.copy(img_src, os.path.join(target_base, 'images/val/'))
        shutil.copy(label_src, os.path.join(target_base, 'labels/val/'))

print(f"划分完成！训练集: {split_idx} 张，验证集: {len(valid_imgs) - split_idx} 张")
print("打乱前的前10张:", valid_imgs[:10])
random.shuffle(valid_imgs)
print("打乱后的前10张:", valid_imgs[:10])
