import numpy as np

# 맵 데이터를 파일에서 읽는 함수
def read_map_from_txt(file_path):
    with open(file_path, 'r') as f:
        # 각 줄을 읽어 2차원 리스트로 변환
        lines = f.readlines()
        grid = [list(map(int, line.strip().split())) for line in lines]
    return np.array(grid)

# 좌우 반전하는 함수
def flip_map_left_right(grid):
    # 각 행(row)을 좌우 반전 (numpy의 flip 함수 사용)
    flipped_grid = np.fliplr(grid)
    return flipped_grid

# 맵 데이터를 파일에 쓰는 함수
def write_map_to_txt(grid, file_path):
    with open(file_path, 'w') as f:
        for row in grid:
            # 각 행을 문자열로 변환하고 파일에 쓰기
            row_str = ' '.join(map(str, row))
            f.write(row_str + '\n')

# 사용 예시
input_file = 'C:\\Users\\SSAFY\\Desktop\\first_map.txt'  # 입력 파일 경로
output_file = 'C:\\Users\\SSAFY\\Desktop\\map.txt'  # 출력 파일 경로

# 1. 파일에서 맵 데이터 읽기
grid = read_map_from_txt(input_file)

# 2. 좌우 반전
flipped_grid = flip_map_left_right(grid)

# 3. 좌우 반전된 맵 데이터를 새로운 파일에 쓰기
write_map_to_txt(flipped_grid, output_file)

print("맵 좌우 반전 완료, 결과는 'flipped_map.txt'에 저장되었습니다.")
