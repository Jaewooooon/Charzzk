#!/ C:\Python37\python.exe
import os
import easyocr
import numpy as np
import cv2
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time  # 추가된 부분

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩



class OCR(Node):

    def __init__(self):
        super().__init__(node_name='picture-ocr')
        print("Picture node is running", flush = True)

        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가 
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.
        
        # 저장할 디렉토리의 절대 경로 설정
        # self.save_directory = "C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\image"  # 원하는 절대 경로로 변경
        
        # 한 장의 사진만 저장할 플래그
        self.image_saved = False

        # 이미지 데이터 구독
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):
        if self.image_saved:  # 이미 사진을 저장했으면 더 이상 동작하지 않음
            print("Shutdown img callback", flush = True)
            return
        
        print("Succdss img callback", flush = True)

        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.        
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 이미지 색상 공간을 BGR에서 HSV로 변환
        # 색상 인식, 추출 정확도를 높이기 위해
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 하늘색 범위 설정 (HSV 색상 범위)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([120, 255, 255])

        # 파란색 영역 마스크 생성
        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # 마스크를 사용하여 원본 이미지에서 파란색 영역을 추출
        blue_regions = cv2.bitwise_and(image, image, mask=mask)

        # 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 중앙 좌표 찾기
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2

        # 정중앙을 기준으로 특정 영역만 검색
        search_radius = 150  # 검색 반경
        central_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.circle(central_mask, (center_x, center_y), search_radius, 255, -1)  # 정중앙을 기준으로 원 생성
        masked_blue_regions = cv2.bitwise_and(mask, central_mask)

        # 정중앙 영역에서 윤곽선 찾기
        central_contours, _ = cv2.findContours(masked_blue_regions, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 가장 큰 윤곽선 찾기
        if central_contours:
            largest_contour = max(central_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)  # 사각형 영역 좌표

            # 여유 공간 설정 (픽셀 단위)
            padding = 30  # 여유 공간

            # 파란색 박스의 좌표 설정 (가로에만 여유 추가)
            x1, y1 = max(0, x - padding), y  # 좌상단 좌표
            x2, y2 = min(width, x + w + padding), y + h  # 우하단 좌표 (세로는 그대로)
            # 이미지 자르기
            cropped_image = image[y1:y2, x1:x2]

            # EasyOCR Reader 객체 생성 (언어 설정 가능)
            reader = easyocr.Reader(['ko', 'en'])  # 한국어 및 영어

            # OCR 적용하여 텍스트 추출
            result = reader.readtext(cropped_image)

            # 잘린 이미지 출력
            plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))  # OpenCV는 BGR 형식으로 이미지를 읽으므로 RGB로 변환
            plt.axis('off')  # 축 숨기기
            plt.show()  # 이미지 표시

            print("Detected text:")
            # 추출된 텍스트 출력
            for (bbox, text, prob) in result:
                print(f"Detected text: {text}, Probability: {prob:.2f}")
                print("length: ", len(text))

                if(len(text) == 8):
                    # 노드를 종료
                    self.image_saved = True
                    print("Shutdown img callback", flush = True)
                    rclpy.shutdown()

        else:
            print("No blue regions found in the central area of the image.")

def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = OCR()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()