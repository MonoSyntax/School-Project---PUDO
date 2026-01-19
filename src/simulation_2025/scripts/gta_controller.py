#!/usr/bin/env python3
"""
OTAGG Car Controller
====================
Pygame-based controller for Ackermann steering vehicle.

Controls:
  W/S or ↑/↓: Throttle/Brake
  A/D or ←/→: Steer
  SPACE: Handbrake
  P: Take Photo
  T: Toggle Camera Topic (Raw/Compressed)
  R: Reset
  Q/ESC: Quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
import math
import os
import time
import cv2
import numpy as np
from cv_bridge import CvBridge

try:
    import pygame
except ImportError:
    print("Error: Pygame not installed. Run: pip install pygame")
    exit(1)


# Colors
BLACK = (15, 15, 20)
DARK = (30, 35, 45)
MID = (50, 55, 70)
LIGHT = (140, 150, 165)
WHITE = (230, 235, 245)
CYAN = (0, 200, 255)
ORANGE = (255, 150, 50)
RED = (255, 70, 80)
GREEN = (60, 255, 130)
YELLOW = (255, 230, 60)


class SmoothValue:
    """Low-pass filter for smooth value changes."""
    def __init__(self, initial=0.0, smoothing=0.15):
        self.value = initial
        self.target = initial
        self.smoothing = smoothing
    
    def set_target(self, target):
        self.target = target
    
    def update(self):
        self.value += (self.target - self.value) * self.smoothing
        return self.value
    
    def snap_to(self, value):
        self.value = value
        self.target = value


class OtaggController(Node):
    def __init__(self):
        super().__init__('otagg_controller')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Camera Subscriptions
        self.bridge = CvBridge()
        self.latest_image = None
        self.camera_topics = ['/camera', '/camera_compressed']
        self.current_topic_idx = 0
        self.img_sub = None
        self._setup_subscriber()
        
        self.save_path = "Training Folder/data/images"
        os.makedirs(self.save_path, exist_ok=True)
        
        self.max_speed = 8.0
        self.max_reverse = 4.0
        self.max_yaw_rate = 1.2
        
        self.speed = SmoothValue(0.0, smoothing=0.15)
        self.steering = SmoothValue(0.0, smoothing=0.12)
        
        self.accel_rate = 5.0
        self.brake_rate = 10.0
        self.steer_rate = 1.5
        
        self.gear = 'N'
        self.status_msg = ""
        self.status_color = WHITE
        self.status_time = 0
        
        pygame.init()
        pygame.display.set_caption("OTAGG Control")
        
        self.width = 700
        self.height = 400
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        
        self.font_sm = pygame.font.Font(None, 22)
        self.font_md = pygame.font.Font(None, 32)
        self.font_lg = pygame.font.Font(None, 52)
        self.font_xl = pygame.font.Font(None, 80)
        
        self.running = True
        self.get_logger().info(f"OTAGG Controller started. Current topic: {self.camera_topics[self.current_topic_idx]}")

    def _setup_subscriber(self):
        if self.img_sub:
            self.destroy_subscription(self.img_sub)
        
        topic = self.camera_topics[self.current_topic_idx]
        if 'compressed' in topic:
            self.img_sub = self.create_subscription(
                CompressedImage, topic, self.compressed_image_callback, 10)
        else:
            self.img_sub = self.create_subscription(
                Image, topic, self.image_callback, 10)
        
        self.get_logger().info(f"Subscribed to {topic}")

    def toggle_camera(self):
        self.current_topic_idx = (self.current_topic_idx + 1) % len(self.camera_topics)
        self._setup_subscriber()
        self.set_status(f"Topic: {self.camera_topics[self.current_topic_idx]}", CYAN)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.latest_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Compressed image conversion error: {e}")

    def take_photo(self):
        if self.latest_image is None:
            self.set_status("No camera data!", RED)
            return
        
        timestamp = int(time.time() * 1000)
        filename = f"capture_{timestamp}.jpg"
        filepath = os.path.join(self.save_path, filename)
        
        try:
            cv2.imwrite(filepath, self.latest_image)
            self.set_status(f"Saved: {filename}", GREEN)
            self.get_logger().info(f"Photo saved to {filepath}")
        except Exception as e:
            self.set_status("Save failed!", RED)
            self.get_logger().error(f"Failed to save photo: {e}")

    def set_status(self, msg, color):
        self.status_msg = msg
        self.status_color = color
        self.status_time = time.time()

    def run(self):
        while self.running and rclpy.ok():
            dt = self.clock.tick(60) / 1000.0
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_p:
                        self.take_photo()
                    elif event.key == pygame.K_t:
                        self.toggle_camera()
            
            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
                self.running = False
            
            self.process_input(keys, dt)
            self.update_smooth()
            self.publish_cmd()
            self.draw()
            
            rclpy.spin_once(self, timeout_sec=0)
        
        self.cleanup()

    def process_input(self, keys, dt):
        if keys[pygame.K_r]:
            self.speed.snap_to(0.0)
            self.steering.snap_to(0.0)
            self.gear = 'N'
            return
        
        throttle = keys[pygame.K_w] or keys[pygame.K_UP]
        brake = keys[pygame.K_s] or keys[pygame.K_DOWN]
        left = keys[pygame.K_a] or keys[pygame.K_LEFT]
        right = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        handbrake = keys[pygame.K_SPACE]
        
        current_target = self.speed.target
        
        if handbrake:
            self.speed.set_target(0.0)
            self.speed.smoothing = 0.25
            self.gear = 'N'
        elif throttle and not brake:
            self.speed.smoothing = 0.15
            if current_target < 0:
                self.speed.set_target(min(current_target + self.brake_rate * dt, 0.0))
                if current_target >= -0.1:
                    self.gear = 'D'
            else:
                self.speed.set_target(min(current_target + self.accel_rate * dt, self.max_speed))
                self.gear = 'D'
        elif brake and not throttle:
            self.speed.smoothing = 0.15
            if current_target > 0.1:
                self.speed.set_target(max(current_target - self.brake_rate * dt, 0.0))
            else:
                self.speed.set_target(max(current_target - self.accel_rate * 0.5 * dt, -self.max_reverse))
                self.gear = 'R'
        else:
            self.speed.smoothing = 0.12
            if abs(current_target) > 0.05:
                friction = 3.0 * dt
                if current_target > 0:
                    self.speed.set_target(max(0.0, current_target - friction))
                else:
                    self.speed.set_target(min(0.0, current_target + friction))
            else:
                self.speed.set_target(0.0)
                self.gear = 'N'
        
        if left and not right:
            direction = -1 if self.speed.value < 0 else 1
            new_steer = self.steering.target + direction * self.steer_rate * dt
            self.steering.set_target(max(-self.max_yaw_rate, min(self.max_yaw_rate, new_steer)))
        elif right and not left:
            direction = 1 if self.speed.value < 0 else -1
            new_steer = self.steering.target + direction * self.steer_rate * dt
            self.steering.set_target(max(-self.max_yaw_rate, min(self.max_yaw_rate, new_steer)))
        else:
            if abs(self.steering.target) > 0.02:
                center_rate = 1.5 * dt
                if self.steering.target > 0:
                    self.steering.set_target(max(0, self.steering.target - center_rate))
                else:
                    self.steering.set_target(min(0, self.steering.target + center_rate))
            else:
                self.steering.set_target(0.0)

    def update_smooth(self):
        self.speed.update()
        self.steering.update()

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.speed.value)
        msg.angular.z = float(self.steering.value)
        self.cmd_pub.publish(msg)

    def draw(self):
        self.screen.fill(BLACK)
        
        pygame.draw.rect(self.screen, DARK, (0, 0, self.width, 45))
        pygame.draw.line(self.screen, CYAN, (0, 45), (self.width, 45), 2)
        title = self.font_md.render("◈ OTAGG CONTROL ◈", True, CYAN)
        self.screen.blit(title, (self.width//2 - title.get_width()//2, 10))
        
        self._draw_speed_panel(25, 65, 280, 165)
        self._draw_steering_panel(320, 65, 170, 165)
        self._draw_inputs_panel(505, 65, 170, 165)
        self._draw_status_bar()
        
        # Display Topic Info
        topic_text = f"Topic: {self.camera_topics[self.current_topic_idx]}"
        topic_surf = self.font_sm.render(topic_text, True, LIGHT)
        self.screen.blit(topic_surf, (25, 240))
        
        if time.time() - self.status_time < 3.0:
            status = self.font_md.render(self.status_msg, True, self.status_color)
            self.screen.blit(status, (self.width//2 - status.get_width()//2, 280))
        
        pygame.display.flip()

    def _draw_speed_panel(self, x, y, w, h):
        pygame.draw.rect(self.screen, DARK, (x, y, w, h), border_radius=8)
        pygame.draw.rect(self.screen, CYAN, (x, y, w, h), 2, border_radius=8)
        
        self.screen.blit(self.font_sm.render("SPEED", True, LIGHT), (x+10, y+8))
        
        speed_kmh = abs(self.speed.value) * 3.6
        color = GREEN if self.gear == 'D' else RED if self.gear == 'R' else YELLOW
        
        self.screen.blit(self.font_xl.render(f"{speed_kmh:.0f}", True, color), (x+20, y+30))
        self.screen.blit(self.font_md.render("km/h", True, LIGHT), (x+140, y+60))
        
        gear_box = pygame.Rect(x+195, y+105, 55, 45)
        pygame.draw.rect(self.screen, MID, gear_box, border_radius=5)
        self.screen.blit(self.font_lg.render(self.gear, True, color), (gear_box.x+15, gear_box.y+5))
        
        bar_rect = pygame.Rect(x+15, y+130, 170, 14)
        pygame.draw.rect(self.screen, MID, bar_rect)
        pct = min(1.0, abs(self.speed.value) / self.max_speed)
        pygame.draw.rect(self.screen, color, (bar_rect.x, bar_rect.y, int(pct * bar_rect.width), bar_rect.height))

    def _draw_steering_panel(self, x, y, w, h):
        pygame.draw.rect(self.screen, DARK, (x, y, w, h), border_radius=8)
        pygame.draw.rect(self.screen, ORANGE, (x, y, w, h), 2, border_radius=8)
        
        self.screen.blit(self.font_sm.render("STEERING", True, LIGHT), (x+10, y+8))
        
        cx, cy = x + w//2, y + 85
        radius = 48
        pygame.draw.circle(self.screen, MID, (cx, cy), radius, 6)
        
        angle = (self.steering.value / self.max_yaw_rate) * 0.5
        ex = cx + int(math.sin(angle) * radius * 0.75)
        ey = cy - int(math.cos(angle) * radius * 0.75)
        pygame.draw.line(self.screen, ORANGE, (cx, cy), (ex, ey), 4)
        pygame.draw.circle(self.screen, ORANGE, (cx, cy), 8)
        
        angle_deg = (self.steering.value / self.max_yaw_rate) * 30
        ang_txt = self.font_md.render(f"{angle_deg:+.0f}°", True, WHITE)
        self.screen.blit(ang_txt, (cx - ang_txt.get_width()//2, y + 140))

    def _draw_inputs_panel(self, x, y, w, h):
        pygame.draw.rect(self.screen, DARK, (x, y, w, h), border_radius=8)
        pygame.draw.rect(self.screen, (150, 100, 255), (x, y, w, h), 2, border_radius=8)
        
        self.screen.blit(self.font_sm.render("INPUTS", True, LIGHT), (x+10, y+8))
        
        keys = pygame.key.get_pressed()
        sz, gap = 28, 4
        sx, sy = x + 38, y + 38
        
        for col, row, letter, pressed in [
            (1, 0, 'W', keys[pygame.K_w] or keys[pygame.K_UP]),
            (0, 1, 'A', keys[pygame.K_a] or keys[pygame.K_LEFT]),
            (1, 1, 'S', keys[pygame.K_s] or keys[pygame.K_DOWN]),
            (2, 1, 'D', keys[pygame.K_d] or keys[pygame.K_RIGHT]),
        ]:
            kx, ky = sx + col * (sz + gap), sy + row * (sz + gap)
            color = CYAN if pressed else MID
            pygame.draw.rect(self.screen, color, (kx, ky, sz, sz), border_radius=4)
            txt = self.font_md.render(letter, True, BLACK if pressed else LIGHT)
            self.screen.blit(txt, (kx + 6, ky + 2))
        
        space_pressed = keys[pygame.K_SPACE]
        sc = RED if space_pressed else MID
        pygame.draw.rect(self.screen, sc, (sx-5, sy+70, 100, 22), border_radius=4)
        self.screen.blit(self.font_sm.render("BRAKE", True, BLACK if space_pressed else LIGHT), (sx+22, sy+73))

    def _draw_status_bar(self):
        pygame.draw.rect(self.screen, DARK, (0, self.height - 35, self.width, 35))
        pygame.draw.line(self.screen, CYAN, (0, self.height - 35), (self.width, self.height - 35), 1)
        
        txt = "W/S/A/D: Move  •  SPACE: Brake  •  P: Photo  •  T: Toggle Cam  •  Q: Quit"
        self.screen.blit(self.font_sm.render(txt, True, LIGHT), (self.width//2 - 300, self.height - 25))
        self.screen.blit(self.font_sm.render("● ROS2", True, GREEN), (15, self.height - 25))

    def cleanup(self):
        self.speed.snap_to(0.0)
        self.steering.snap_to(0.0)
        self.publish_cmd()
        self.destroy_node()
        pygame.quit()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    ctrl = OtaggController()
    ctrl.run()


if __name__ == "__main__":
    main()