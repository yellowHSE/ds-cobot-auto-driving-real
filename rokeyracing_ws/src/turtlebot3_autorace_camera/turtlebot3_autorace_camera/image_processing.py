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

        # Declare parameter with default integer value
        self.video_port = self.declare_parameter('video_port', 0).get_parameter_value().integer_value
        self.roi = self.declare_parameter('roi', 'Left').get_parameter_value().string_value
        

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

        self.get_logger().info(f'Image Processor Node with imshow started. roi={self.roi}')

    def auto_exposure(self,frame):
        yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV)
        yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

    def white_balance_simple(self,img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)

        avg_a = np.average(lab[:, :, 1])
        avg_b = np.average(lab[:, :, 2])

        lab[:, :, 1] -= ((avg_a - 128) * (lab[:, :, 0] / 255.0) * 1.1)
        lab[:, :, 2] -= ((avg_b - 128) * (lab[:, :, 0] / 255.0) * 1.1)

        lab = np.clip(lab, 0, 255).astype(np.uint8)
        return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)



    def region_of_interest(self,img, vertices):
        """Apply a mask to focus on the region of interest."""
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, [vertices], 255)
        masked_img = cv2.bitwise_and(img, mask)
        return masked_img

    def average_slope_intercept(self,image, lines):
        """Average and extrapolate lines to draw a single line for left and right lanes."""
        left_lines = []
        right_lines = []
        
        if lines is None or len(lines) == 0:
            return None
        
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            slope = (y2 - y1) / (x2 - x1 + 1e-5)  # Avoid division by zero
            intercept = y1 - slope * x1
            if slope < -0.4:  # Negative slope for left lane
                left_lines.append((slope, intercept))
            elif slope > 0.4:  # Positive slope for right lane
                right_lines.append((slope, intercept))

        def compute_line(self,lines, y1, y2):
            if not lines:
                return None
            slopes, intercepts = zip(*lines)
            avg_slope = np.mean(slopes)
            avg_intercept = np.mean(intercepts)
            x1 = int((y1 - avg_intercept) / avg_slope)
            x2 = int((y2 - avg_intercept) / avg_slope)
            return np.array([x1, int(y1), x2, int(y2)])

        y1 = image.shape[0]  # Bottom of image
        y2 = int(image.shape[0] * 0.6)  # Middle of ROI
        left_line = compute_line(left_lines, y1, y2)
        right_line = compute_line(right_lines, y1, y2)
        return np.array([left_line, right_line]) if left_line is not None and right_line is not None else None

    def draw_lines(self,image, lines, color=(255, 0, 0), thickness=10):
        """Draw lines on the image."""
        line_image = np.zeros_like(image)
        if lines is not None:
            for x1, y1, x2, y2 in lines:
                if x1 is not None and x2 is not None:
                    cv2.line(line_image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
        return cv2.addWeighted(image, 0.8, line_image, 1, 0)

    def process_frame(self,image):
        """Process a single frame to detect and draw lane lines."""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        
        # Define region of interest (trapezoid)
        height, width = image.shape[:2]
        vertices = np.array([[
            (0.15 * width, height),  # Bottom-left
            (0.85 * width, height),  # Bottom-right
            (0.55 * width, 0.6 * height),  # Top-right
            (0.45 * width, 0.6 * height)   # Top-left
        ]], dtype=np.int32)
        
        # Apply ROI mask
        masked_edges = self.region_of_interest(edges, vertices)
        
        # Hough Transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=2,
            theta=np.pi/180,
            threshold=100,
            minLineLength=40,
            maxLineGap=100
        )
        
        # Average and extrapolate lines
        averaged_lines = self.average_slope_intercept(image, lines)
        
        # Draw lines on the original image
        result = self.draw_lines(image, averaged_lines)
        return result
    
    def fraction_frame(self,frame,fraction):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Grayscale
        edges = cv2.Canny(gray, 50, 150)               # ✅ Canny Edge Detection

        height = frame.shape[0]
        width = frame.shape[1]
        
        ratio = 0.5
        color = (255,0,255)
        
        if fraction == 'Bottom':
            top_left = np.array([0, height//2])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
        elif fraction == 'Top':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width, height//2])  # (x, y)
        elif fraction == 'Left':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width//2, height])  # (x, y)
        elif fraction == 'Right':
            # Define coordinates to keep (left half)
            top_left = np.array([width//2, 0])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)

        if True:
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            fraction_frame = frame[y1:y2, x1:x2]

            cv2.imshow("fraction_frame Image", fraction_frame)
            cv2.waitKey(1)  # necessary for OpenCV GUI to update
       
            #color = (255,0,255)
            #cv2.rectangle(frame, top_left, bottom_right, color, 2)  # 초록색 선

        return fraction_frame
        """
        if self.roi == 'Bottom':
            top_left = np.array([0, height//2])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (255,0,255)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선

        elif self.roi == 'Top':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width, height//2])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (0,255,0)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선


        elif self.roi == 'Left':
            top_left = np.array([0, 0])           # (x, y)
            bottom_right = np.array([width//2, height])  # (x, y)
            
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

       
            color = (255,255,0)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선

        elif self.roi == 'Right':
            # Define coordinates to keep (left half)
            top_left = np.array([width//2, 0])           # (x, y)
            bottom_right = np.array([width, height])  # (x, y)
       
            x1, y1 = top_left
            x2, y2 = bottom_right
            
            # Crop using coordinates (remember numpy indexing is [y, x])
            roi_edges = edges[y1:y2, x1:x2]

            color = (255,255,255)
            cv2.rectangle(roi_edges, top_left, bottom_right, color, 2)  # 초록색 선
        """



        return frame
    
    def process_image(self,frame):


        # adding Gaussian blur to the image of original
        frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # 자동 노출 보정 + 화이트 밸런싱
        frame = self.auto_exposure(frame)
        frame = self.white_balance_simple(frame)
        
        frame = self.process_frame(frame)
        cv2.imshow('Lane Detection', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


        # HSV 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 흰색 마스킹
        # 기존 level.yaml 파일 값
        # lower_white = np.array([0, 105, 0])
        # upper_white = np.array([179, 255, 70])
        
        # 흰색 마스킹 새로운 값
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 60, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 노란색 마스킹
        # 기존 level.yaml 파일 값
        # lower_yellow = np.array([10, 95, 700])
        # upper_yellow = np.array([127, 255, 255])
        
        # 노란색 마스킹 새로운 값
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 흰색 노란색 합치기
        mask_combined = cv2.bitwise_or(mask_white, mask_yellow)

        cv2.imshow("Left, Right Frame Image", mask_yellow)
        cv2.waitKey(1)  # necessary for OpenCV GUI to update

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
            #cv2.imshow("mask_white Image", mask_white)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update

            #cv2.imshow("mask_yellow Image", mask_yellow)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update

            #cv2.imshow("mask_combined Image", mask_combined)
            #cv2.waitKey(1)  # necessary for OpenCV GUI to update


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
