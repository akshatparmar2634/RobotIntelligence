#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from transformers import AutoProcessor, AutoModelForImageTextToText
from PIL import Image as PILImage
import time
import re

class YoloSmolVLM2Node(Node):
    def __init__(self):
        super().__init__("yolo_smolvlm2_node")
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # --- Frame rate control (2 FPS) ---
        self.last_frame_time = 0.0
        self.frame_interval = 0.5  # seconds → 2 FPS

        # --- YOLO ---
        self.get_logger().info("Loading YOLO model...")
        self.yolo = YOLO("yolov8s.pt")  
        self.yolo.to(self.device)
        self.get_logger().info("YOLO loaded successfully.")

        # --- SmolVLM2 ---
        self.get_logger().info("Loading SmolVLM2 model...")
        try:
            model_path = "HuggingFaceTB/SmolVLM2-256M-Video-Instruct"
            self.processor = AutoProcessor.from_pretrained(model_path)
            self.vlm_model = AutoModelForImageTextToText.from_pretrained(
                model_path,
                torch_dtype=torch.bfloat16 if self.device == "cuda" else torch.float32,
                _attn_implementation="eager"
            ).to(self.device)
            self.vlm_enabled = True
            self.get_logger().info("SmolVLM2 loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load SmolVLM2: {e}")
            self.vlm_enabled = False

        # --- Subscription to camera topic ---
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.get_logger().info("Node started. Waiting for camera frames...")

    def image_callback(self, msg: Image):
        # --- Frame rate limiter ---
        current_time = time.time()
        if current_time - self.last_frame_time < self.frame_interval:
            return  # Skip frame to maintain 2 FPS
        self.last_frame_time = current_time

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.yolo(frame)
        annotated = frame.copy()

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.yolo.model.names[cls]

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                if not self.vlm_enabled:
                    continue

                crop = frame[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                pil_img = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                messages = [
                    {
                        "role": "user",
                        "content": [
                            {"type": "image", "image": pil_img},
                            {"type": "text", "text": "Describe this object briefly."},
                        ]
                    }
                ]

                try:
                    inputs = self.processor.apply_chat_template(
                        messages,
                        add_generation_prompt=True,
                        tokenize=True,
                        return_dict=True,
                        return_tensors="pt"
                    ).to(self.device, dtype=torch.bfloat16 if self.device == "cuda" else torch.float32)

                    with torch.no_grad():
                        generated_ids = self.vlm_model.generate(**inputs, do_sample=False, max_new_tokens=50)
                        text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

                    # --- Robust parsing of assistant response ---
                    try:
                        if "<assistant>" in text.lower():
                            match = re.search(r"<\s*assistant\s*>\s*(.*)", text,
                                              flags=re.IGNORECASE | re.DOTALL)
                            text = match.group(1).strip() if match else text
                        elif "assistant:" in text.lower():
                            text = re.split(r"assistant:\s*", text,
                                            flags=re.IGNORECASE)[-1].strip()
                        elif "Assistant" in text:
                            text = text.split("Assistant")[-1].strip()
                        text = text.strip()
                    except Exception as e:
                        self.get_logger().warn(f"Assistant parsing failed: {e}")
                        text = text.strip()

                    # --- Optional: trim very long responses for overlay ---
                    if len(text) > 80:
                        text = text[:77] + "..."

                    cv2.putText(annotated, f"VLM: {text}", (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                except torch.cuda.OutOfMemoryError:
                    self.get_logger().warn("GPU OOM during VLM inference — skipping this box.")
                    torch.cuda.empty_cache()
                except Exception as e:
                    self.get_logger().warn(f"SmolVLM2 inference failed: {e}")

        cv2.imshow("YOLO + SmolVLM2", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloSmolVLM2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
 