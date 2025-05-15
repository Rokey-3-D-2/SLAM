import os

# 클래스 매핑 정의
prefix_to_class = {
    'img1': 0,
    'img2': 1,
    'img3': 2,
    'img4': 3,
    'img5': 4,
    'img1_err': 5,
    'img2_err': 6,
    'img3_err': 7,
    'img4_err': 8,
    'img5_err': 9
}

# 기준 디렉토리
base_dir = 'images_split'
splits = ['train', 'val', 'test']

def extract_prefix(filename):
    """
    파일 이름에서 prefix (img1, img2_err 등)를 추출
    """
    parts = filename.split('_')
    if len(parts) >= 2:
        candidate = parts[0] + '_' + parts[1]
        if candidate in prefix_to_class:
            return candidate
    if parts[0] in prefix_to_class:
        return parts[0]
    return None

for split in splits:
    images_dir = os.path.join(base_dir, split, 'images')
    labels_dir = os.path.join(base_dir, split, 'labels')

    if not os.path.exists(images_dir):
        print(f"[경고] {images_dir} 없음")
        continue

    for image_filename in os.listdir(images_dir):
        if not image_filename.lower().endswith(('.jpg', '.jpeg', '.png')):
            continue

        prefix = extract_prefix(image_filename)
        if prefix is None:
            print(f"[무시] 접두어 없음 또는 매칭 실패: {image_filename}")
            continue

        label_filename = os.path.splitext(image_filename)[0] + '.txt'
        label_path = os.path.join(labels_dir, label_filename)

        if not os.path.isfile(label_path):
            print(f"[누락] 라벨 파일 없음: {label_path}")
            continue

        with open(label_path, 'r') as f:
            lines = f.readlines()

        new_lines = []
        for line in lines:
            parts = line.strip().split()
            if not parts:
                continue
            parts[0] = str(prefix_to_class[prefix])  # 클래스 번호 변경
            new_lines.append(' '.join(parts))

        with open(label_path, 'w') as f:
            f.write('\n'.join(new_lines) + '\n')

        print(f"[완료] {label_filename} → 클래스 {prefix_to_class[prefix]}")
