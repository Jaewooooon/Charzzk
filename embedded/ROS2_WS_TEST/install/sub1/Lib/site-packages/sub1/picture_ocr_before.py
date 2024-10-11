import os
import easyocr
import numpy as np
import cv2
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time
from fastapi import FastAPI
import json
from .message_producer import MessageProducer
import datetime

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩


class OCR(Node):

    def __init__(self):
        super().__init__(node_name='picture_ocr_before')
        print("Picture node is running", flush = True)

        # RabbitMQ 프로듀서 인스턴스 생성
        self.message_producer = MessageProducer(host='localhost')  # 필요에 따라 RabbitMQ 서버 정보를 수정

        # 한 장의 사진만 저장할 플래그
        self.image_saved = False

        # 이미지 데이터 구독
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
    

    # JSON 파일에서 데이터를 불러오는 함수
    def load_json_data(self):
        filename_txt="charge_command.json"
        filename = os.path.join("C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\sub1", filename_txt)  # 절대 경로 설정
        try:
            with open(filename, "r",encoding="utf-8") as f:
                data = json.load(f)
                return data
            
        except FileNotFoundError:
            #print("Error: JSON file not found.")
            return None

    def ocr(self, cropped_image, target_length = 7, max_attemps = 5):
        # EasyOCR Reader 객체 생성 (언어 설정 가능)
        reader = easyocr.Reader(['ko', 'en'])  # 한국어 및 영어
        # OCR 적용하여 텍스트 추출
        
        # 잘린 이미지 출력
        plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))  # OpenCV는 BGR 형식으로 이미지를 읽으므로 RGB로 변환
        plt.axis('off')  # 축 숨기기
        plt.show()  # 이미지 표시

        picNumber = ""
        attempts = 0

        while(len(picNumber) != target_length and attempts <max_attemps) :
            print(f"Attempt {attempts + 1}: Performing OCR ...")
            result = reader.readtext(cropped_image)

            # OCR 결과 처리
            for (bbox, text, prob) in result:
                text = text.replace('[', '').replace(']', '').replace(' ', '').strip() 
                print(f"Detected text: {text}, Probability: {prob:.2f}")
                picNumber+= text
        
            # 최종 합쳐진 문자열 출력
            print(f"Final detected text: {picNumber}, length: {len(picNumber)}")
            attempts += 1

        return picNumber

    def img_callback(self, msg):
        print("picture img callback", flush = True)

        # if self.image_saved:  # 이미 사진을 저장했으면 더 이상 동작하지 않음
        #     return
               
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 이미지 색상 공간을 BGR에서 HSV로 변환
        # 색상 인식, 추출 정확도를 높이기 위해
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

         # OCR 실행
        picNumber = self.ocr(image)

        # BE에서 데이터 받아오기
        jsondata = self.load_json_data()
        if jsondata is None:
            self.get_logger().error('No data found, cannot publish goal')
            return
        storedNumber = jsondata['carNumber']
        print(f"jsondata: {storedNumber}")

        # 비교
        if(picNumber == storedNumber):
            # 맞으면 1 보내기
            message = {
                "type": "recognition_status",
                "status": "success",
                "reservationId": jsondata['reservationId'],
                "chargerId": jsondata['chargerId'],
                "carNumber": jsondata['carNumber'],
                "timestamp": datetime.datetime.now().isoformat()
                }
            self.message_producer.send_message_to_queue(message)
            print("Send success!", flush=True)
            
        else:
            # 틀리면 0 보내기
            message = {
                "type": "recognition_status",
                "status": "failure",
                "reservationId": jsondata['reservationId'],
                "chargerId": jsondata['chargerId'],
                "carNumber": picNumber,
                "timestamp": datetime.datetime.now().isoformat()
                }
            self.message_producer.send_message_to_queue(message)
            print("Send fail!", flush=True)

        # 노드를 종료
        self.image_saved = True
        if self.message_producer:
            self.message_producer.close_connection()
            print("RabbitMQ connection closed", flush=True)
        self.destroy_node()  # 노드를 종료
        print("OCR shutdown success", flush = True)

       


def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = OCR()
    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin_once(image_parser)

if __name__ == '__main__':

    main()