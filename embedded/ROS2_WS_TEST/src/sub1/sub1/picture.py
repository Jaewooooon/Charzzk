#!/ C:\Python37\python.exe
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time  # 추가된 부분

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')
        print("Picture node is running", flush = True)
        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가 
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.
        
        # 저장할 디렉토리의 절대 경로 설정
        self.save_directory = "C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\image"  # 원하는 절대 경로로 변경
        
        # 한 장의 사진만 저장할 플래그
        self.image_saved = False

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):
        if self.image_saved:  # 이미 사진을 저장했으면 더 이상 동작하지 않음
            return
        
        print("Succdss img callback", flush = True)
        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.        

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # 로직 3: 이미지 저장
        timestamp = time.strftime("%Y%m%d-%H%M%S")  # 현재 시간을 기반으로 파일명 생성
        file_name = f"image_{timestamp}.jpg"
        
        # 절대 경로를 사용하여 이미지 저장
        absolute_path = os.path.join(self.save_directory, file_name)

        # 로직 3: 이미지 저장
        if cv2.imwrite(absolute_path, img_bgr):  # 이미지를 .jpg 파일로 저장하고 성공 여부 확인
            print(f"Image saved as {absolute_path}")
            self.image_saved = True  # 사진을 한 장만 저장했다는 플래그 설정
        else:
            print("Failed to save the image.")
            return

         # 잠시 대기하여 저장이 완료되도록 함
        time.sleep(1)

        # 로직 5: 이미지 출력 (cv2.imshow)
        #cv2.imshow("img_bgr", img_bgr)
        #cv2.waitKey(1)

        # 노드를 종료
        rclpy.shutdown()
        print("Shutdown img callback", flush = True)

def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()