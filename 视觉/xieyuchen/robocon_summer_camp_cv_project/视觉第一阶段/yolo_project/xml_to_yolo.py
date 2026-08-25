import os
import glob
import xml.etree.ElementTree as ET

# ===== 请根据你的实际情况修改这三行 =====
XML_DIR = "labels"          # 存放 .xml 标注文件的文件夹
IMG_DIR = "raw_imgs"        # 存放原始图片的文件夹
OUTPUT_DIR = "labels"       # 转换后的 .txt 文件输出文件夹（可以覆盖原xml，也可另建）
CLASS_NAMES = ['red', 'blue']  # 类别列表，顺序必须与标注时一致！
# =====================================

os.makedirs(OUTPUT_DIR, exist_ok=True)

def convert_xml_to_yolo(xml_path):
    """将单个 XML 文件转换为 YOLO 格式的 .txt 文件"""
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # 获取图片尺寸
    size = root.find('size')
    img_w = int(size.find('width').text)
    img_h = int(size.find('height').text)
    
    # 获取图片文件名（不含后缀）
    img_name = os.path.splitext(root.find('filename').text)[0]
    
    yolo_lines = []
    for obj in root.findall('object'):
        # 获取类别名称
        class_name = obj.find('name').text
        if class_name not in CLASS_NAMES:
            print(f"⚠️ 警告: 未知类别 '{class_name}' 出现在 {xml_path}，已跳过")
            continue
        class_id = CLASS_NAMES.index(class_name)
        
        # 获取边界框坐标
        bndbox = obj.find('bndbox')
        xmin = float(bndbox.find('xmin').text)
        ymin = float(bndbox.find('ymin').text)
        xmax = float(bndbox.find('xmax').text)
        ymax = float(bndbox.find('ymax').text)
        
        # 坐标转换（核心公式）
        x_center = (xmin + xmax) / 2 / img_w
        y_center = (ymin + ymax) / 2 / img_h
        width = (xmax - xmin) / img_w
        height = (ymax - ymin) / img_h
        
        # 格式：class_id x_center y_center width height（保留6位小数）
        yolo_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}")
    
    # 写入 .txt 文件（与图片同名）
    txt_path = os.path.join(OUTPUT_DIR, img_name + '.txt')
    with open(txt_path, 'w') as f:
        f.write('\n'.join(yolo_lines))
    print(f"✅ 已转换: {os.path.basename(xml_path)} -> {img_name}.txt")

# 批量处理所有 XML 文件
xml_files = glob.glob(os.path.join(XML_DIR, '*.xml'))
if not xml_files:
    print("❌ 在 labels 文件夹中没有找到 .xml 文件！")
else:
    for xml_file in xml_files:
        convert_xml_to_yolo(xml_file)
    print(f"\n🎉 全部完成！共处理 {len(xml_files)} 个标注文件。")
