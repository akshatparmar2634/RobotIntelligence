import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
from PIL import Image as PILImage
import numpy as np
from transformers import CLIPProcessor, CLIPModel

YOLO_MODEL_PATH = "yolov8s.pt"
RUN_CLIP = True
CANDIDATE_LABELS = ["chair", "table", "sofa", "robot", "person", "monitor", "cup", "book", "door", "window"]

class YoloClipCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_clip_camera_node')
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        self.get_logger().info("Loading YOLOv8 model...")
        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.yolo_model.to(self.device)
        self.get_logger().info("YOLOv8 loaded.")

        self.run_clip = RUN_CLIP
        if self.run_clip:
            self.get_logger().info("Loading CLIP...")
            try:
                self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
                self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
                inputs = self.clip_processor(text=CANDIDATE_LABELS, return_tensors="pt", padding=True).to(self.device)
                with torch.no_grad():
                    text_emb = self.clip_model.get_text_features(**inputs)
                    text_emb = text_emb / text_emb.norm(p=2, dim=-1, keepdim=True)
                self.text_embeddings = text_emb
                self.get_logger().info("CLIP loaded.")
            except Exception as e:
                self.get_logger().error(f"CLIP load failed: {e}")
                self.run_clip = False

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.get_logger().info("Yolo+CLIP camera node started. Waiting for images...")

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.yolo_model(frame)
        annotated = frame.copy()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].int().cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.yolo_model.names[cls]

                cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                cv2.putText(annotated, f"{label} {conf:.2f}", (b[0], b[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                if self.run_clip:
                    crop = frame[b[1]:b[3], b[0]:b[2]]
                    if crop.size != 0:
                        pil_crop = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                        inputs = self.clip_processor(text=None, images=pil_crop, return_tensors="pt").to(self.device)
                        with torch.no_grad():
                            img_emb = self.clip_model.get_image_features(**inputs)
                            img_emb = img_emb / img_emb.norm(p=2, dim=-1, keepdim=True)
                            sim = (img_emb @ self.text_embeddings.T).squeeze(0)
                            best_idx = sim.argmax().item()
                            open_label = CANDIDATE_LABELS[best_idx]
                            score = sim[best_idx].item()
                            cv2.putText(annotated, f"CLIP: {open_label} ({score:.2f})", (b[0], b[3]+15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

        cv2.imshow("YOLOv8 + CLIP Detection", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloClipCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
