import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        self.roi = 'Bottom'
        self.bridge = CvBridge()
        self.idx = 0

        # Subscribe to raw image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for processed image
        self.publisher = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        self.get_logger().info('Image Processor Node with imshow started.')

    #YUV 색공간을 사용하여 **명도(Y)**만 히스토그램 평활화(equalizeHist)를 적용
    def auto_exposure(self,frame):
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
    
    #LAB 색공간에서 색상 보정을 수행합니다. 특히 **a 채널(녹-적)과 b 채널(청-황)**의 평균값을 기준으로 자동 보정을 수행
    def white_balance(self,img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)

        avg_a = np.average(lab[:, :, 1])
        avg_b = np.average(lab[:, :, 2])

        lab[:, :, 1] -= ((avg_a - 128) * (lab[:, :, 0] / 255.0) * 1.1)
        lab[:, :, 2] -= ((avg_b - 128) * (lab[:, :, 0] / 255.0) * 1.1)

        lab = np.clip(lab, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    def huffman_lane(self,frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Grayscale
        edges = cv2.Canny(gray, 50, 150)               # ✅ Canny Edge Detection

        # ROI 설정 (예시: 하단 절반)
        height = frame.shape[0]
        width = frame.shape[1]
        mask = np.zeros_like(edges)

        ratio = 0.5

        self.roi = 'Left'         

        if self.roi == 'Bottom':
            roi_vertices = np.array([[(0, height), (width, height), (width, int(height*ratio)), (0, int(height*ratio))]], dtype=np.int32)               
            #                           좌상단     , 우상단          , 우하단                     ,좌하단
            cv2.fillPoly(mask, roi_vertices, 255)
            roi_edges = cv2.bitwise_and(edges, mask)

            top_left = (0, int(height * ratio))
            bottom_right = (width, height)            
            color = (255,0,0)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선

        elif self.roi == 'Top':
            roi_vertices = np.array([[(0, height), (width, height), (width, int(height*ratio)), (0, int(height*ratio))]], dtype=np.int32)               
            
            cv2.fillPoly(mask, roi_vertices, 255)
            inverted_mask = cv2.bitwise_not(mask) ######
            roi_edges = cv2.bitwise_and(edges, inverted_mask)

            top_left = (0, 0)
            bottom_right = (int(width-1), int(height/2))
            color = (255,255,0)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선


        if self.roi == 'Right':
            roi_vertices = np.array([[(width*ratio, 0),(width,0), (width,height), (int(width*ratio), int(height))]], dtype=np.int32)               
            #                           좌상단     , 우상단          , 우하단                     ,좌하단
            
            cv2.fillPoly(mask, roi_vertices, 255)
            roi_edges = cv2.bitwise_and(edges, mask)

            top_left = (int(width*ratio), 0)
            bottom_right = (width,height)         
            color = (255,0,0)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선            
            

        elif self.roi == 'Left':
            roi_vertices = np.array([[(0, 0), (width*ratio, 0), (width*ratio, height), (0, int(height))]], dtype=np.int32)               
            #                           좌상단     , 우상단          , 우하단                     ,좌하단
            
            cv2.fillPoly(mask, roi_vertices, 255)
            roi_edges = cv2.bitwise_and(edges, mask)

            top_left = (0, 0)
            bottom_right = (int(width*ratio), int(height))         
            color = (255,0,0)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선

        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=50,
            maxLineGap=150
        )

        # 선 그리기
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), color, 2)
                
        return frame
    
    def process_image(self,frame):


        # adding Gaussian blur to the image of original
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # 자동 노출 보정 + 화이트 밸런싱
        frame = self.auto_exposure(frame) #not good
        frame = self.white_balance(frame)


        frame = self.huffman_lane(frame)
        cv2.imshow("huffman_lane Image", frame)
        cv2.waitKey(1)  # necessary for OpenCV GUI to update
        



        # HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self.idx = 1

        # 흰색 마스킹
        # 0:level.yaml ,1: GPT 2: Nellow 3:Wide
        lower_white = np.array([[0, 105, 0],[0, 0, 200],[0,0,200],[0,0,220]])
        upper_white = np.array([[179, 255, 70],[180, 60, 255],[179,30,255],[179,20,255]])
        mask_white = cv2.inRange(hsv, lower_white[self.idx], upper_white[self.idx])

        # 노란색 마스킹
        # 0:level.yaml ,1: GPT 2: Nellow 3:Wide
        lower_yellow = np.array([[10, 95, 700],[15, 80, 80],[20,100,100],[15,80,80]])
        upper_yellow = np.array([[127, 255, 255],[35, 255, 255],[30,255,255],[35,255,255]])
        mask_yellow = cv2.inRange(hsv, lower_yellow[self.idx], upper_yellow[self.idx])

        # 합치기
        mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

        return mask_white, mask_yellow, mask_combined

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            mask_white, mask_yellow, mask_combined = self.process_image(frame)
            # Example: convert to grayscale and back to BGR

            #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            mask_white = cv2.cvtColor(mask_white, cv2.COLOR_GRAY2BGR)
            mask_yellow = cv2.cvtColor(mask_yellow, cv2.COLOR_GRAY2BGR)
            mask_combined = cv2.cvtColor(mask_combined, cv2.COLOR_GRAY2BGR)

            # Show the processed image using OpenCV
            cv2.imshow("mask_white Image", mask_white)
            cv2.waitKey(1)  # necessary for OpenCV GUI to update

            cv2.imshow("mask_yellow Image", mask_yellow)
            cv2.waitKey(1)  # necessary for OpenCV GUI to update

            cv2.imshow("mask_combined Image", mask_combined)
            cv2.waitKey(1)  # necessary for OpenCV GUI to update


            # Convert back to ROS Image and publish
            out_msg = self.bridge.cv2_to_imgmsg(mask_combined, encoding='bgr8')
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
