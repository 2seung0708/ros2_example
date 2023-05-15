import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile # QoS 설정을 위해 
from std_msgs.msg import String # String 메시지 인터페이스를 사용하기 위해 import

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import os
import torch

## For detection
from torchvision.models.detection import ssdlite320_mobilenet_v3_large, SSDLite320_MobileNet_V3_Large_Weights
from torchvision.utils import draw_bounding_boxes
from torchvision.transforms.functional import to_pil_image, pil_to_tensor

from ssd_detection.misc import Timer ## 작성한 타이머 관련 코드 import

class DetectionNode(Node):

    def __init__(self):
        super().__init__('detection_node')

        qos_profile = QoSProfile(depth=10)

        # Create a subscriber to the Image topic
        self.subscription = self.create_subscription(
               Image, # 토픽 메시지 타입
               'image', # 토픽 이름
               self.detect_callback, # 수신 받은 메세지를 처리할 콜백함수
               qos_profile ) # QoS 설정
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        
        
        ###======================= SSD 설정 부분 =======================
        # Step 1: Initialize model with the best available weights
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu") ## Device 사용 가능한지 확인
        self.get_logger().info("Device: %s "%(self.device)) ## device 정보 확인
        self.weights = SSDLite320_MobileNet_V3_Large_Weights.COCO_V1 ## mobilenet을 backbone으로 하는 사전학습 모델 불러오기
        #=====
        # ## ver1. torchvision의 checkpoint 이용
        # self.net = ssdlite320_mobilenet_v3_large(weights=self.weights, score_thresh=0.5).cuda() ## SSD_mobilenet 모델 선언후 GPU로 올리기
        ## ver2. custom checkpoint 이용
        self.ckpoint = torch.load("./src/ssd_detection/COCO_CheckPoint.pth") ## 체크포인트 경로 지정
        self.net = self.ckpoint['model'].cuda() ## SSD_mobilenet 모델 선언후 GPU로 올리기
        #=====        
        self.net.eval() ## 평가모드로 설정

        # Step 2: Initialize the inference transforms
        self.preprocess = self.weights.transforms() ## transform 정보 받아오기
        #=====
        # ## ver1. torchvision의 checkpoint 이용
        # self.class_names = self.weights.meta["categories"] ## 카테고리 정보 받아오기(COCO데이터셋)
        ## ver2. custom checkpoint 이용
        self.class_names = checkpoints_dict['category_list']
        #=====
        ###=============================================================
        
        self.timer = Timer()


    def detect_callback(self, data):
        self.get_logger().info("Received an image! ") ## 출력 문구
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")## 받은 이미지 데이터를 cv2 타입으로 변환
        except CvBridgeError as e: ## 에러 발생시
          print(e)

        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) ## BGR을 RGB로 변경
       
        
        ###======================= 모델 추론 부분 =======================
        # Step 3: Apply inference preprocessing transforms
        image = to_pil_image(image) ## 이미지를 pil 형식으로 변경
        image = self.preprocess(image) ## transform 적용

        # Step 4: Use the model and visualize the prediction
        self.timer.start() ## inference 속도 측정 시작
        
        images = image.unsqueeze(0)
        images = images.to(self.device) ## GPU 혹은 CPU로

        with torch.no_grad():
            pred = self.net(images)[0] ## SSD 모델의 성능 확인
        
        boxes, probs, labels = pred["boxes"], pred["scores"], pred["labels"]
        interval = self.timer.end()
        print('Time: {:.2f}s, Detect Objects: {:d}.'.format(interval, labels.size(0))) ## inference에 소요된 시간과 box 개수 출력
        ###=============================================================


        ###======================= 결과 시각화 코드 =======================
        for i in range(boxes.size(0)):
            box = boxes[i, :]
            label = f"{self.class_names[labels[i]]}: {probs[i]:.2f}"
            print("Object: " + str(i) + " " + label)

            ## bounding box 그리기
            cv2.rectangle(cv_image, 
                          (int(box[0]), int(box[1])), # 좌표
                          (int(box[2]), int(box[3])), # 좌표
                          (255, 255, 0), # 박스 색
                          4)# 선 두께

            ## 라벨 정보와 score 정보 달기
            cv2.putText(cv_image, 
                          label, ## 예측한 라벨 정보
                       (int(box[0])+20, int(box[1])+40), ## 정보 작성 위치 (바운딩박스 기준으로)
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,  # font scale
                       (255, 0, 255), 2)  # line type
        ###=============================================================

        # Displaying the predictions
        cv2.imshow('object_detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args) # 초기화
    node = DetectionNode()# DetectionNode를 node라는 이름으로 생성
    try:
        rclpy.spin(node) # 생성한 노드를 spin하여 지정된 콜백 함수 실행 
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:# 종료시 (ex `Ctrl + c`)
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown() # rclpy.shutdown 함수로 노드 종료


if __name__ == '__main__':
    main()
