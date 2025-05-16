import os
import shutil

source_dirs = [f'img{i}_split' for i in range(1, 6)]
target_base = 'images_split'
subdirs = ['train/images', 'train/labels', 'val/images', 'val/labels', 'test/images', 'test/labels']

# 대상 디렉토리 생성
for subdir in subdirs:
    os.makedirs(os.path.join(target_base, subdir), exist_ok=True)

# 복사 및 디버깅 출력
for src in source_dirs:
    for subdir in subdirs:
        src_path = os.path.join(src, subdir)
        dst_path = os.path.join(target_base, subdir)

        if not os.path.exists(src_path):
            print(f"경고: {src_path} 없음")
            continue

        files = os.listdir(src_path)
        if not files:
            print(f"정보: {src_path} 에 복사할 파일 없음")
            continue

        for filename in files:
            src_file = os.path.join(src_path, filename)
            dst_file = os.path.join(dst_path, filename)

            if os.path.exists(dst_file):
                prefix = src.split('_')[0]
                dst_file = os.path.join(dst_path, f"{prefix}_{filename}")

            shutil.copy2(src_file, dst_file)
        print(f"{src_path} → {dst_path} 로 {len(files)}개 파일 복사됨")

print("폴더 병합 완료")
