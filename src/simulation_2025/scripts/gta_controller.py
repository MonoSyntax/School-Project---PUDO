#!/usr/bin/env python3
"""
OTAGG Car Controller with Navigation Commands
==============================================
Pygame-based controller for Ackermann steering vehicle.

Controls:
  W/S or ↑/↓: Throttle/Brake
  A/D or ←/→: Steer
  SPACE: Handbrake
  1-5: Navigation Commands
  R: Reset
  Q/ESC: Quit

NOTE: Command definitions MUST be kept in sync with:
  otagg_vision/scripts/nav_commands.py (source of truth)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math
import time

try:
    import pygame
except ImportError:
    print("Error: Pygame not installed. Run: pip install pygame")
    exit(1)


# ═══════════════════════════════════════════════════════════════════════════════
# NAVIGATION COMMANDS - Keep in sync with otagg_vision/scripts/nav_commands.py
# ═══════════════════════════════════════════════════════════════════════════════
NAV_COMMANDS = {
    0: {'name': 'FOLLOW_LANE', 'short': 'FLW', 'color': (60, 255, 130)},
    1: {'name': 'TURN_LEFT',   'short': 'TL',  'color': (255, 230, 60)},
    2: {'name': 'TURN_RIGHT',  'short': 'TR',  'color': (255, 180, 60)},
    3: {'name': 'LANE_LEFT',   'short': 'LL',  'color': (0, 200, 255)},
    4: {'name': 'LANE_RIGHT',  'short': 'LR',  'color': (180, 100, 255)},
    5: {'name': 'DOCK_STOP',   'short': 'DST', 'color': (255, 100, 100)},
    6: {'name': 'MERGE_ROAD',  'short': 'MRG', 'color': (100, 255, 200)},
    7: {'name': 'GO_MIDDLE',   'short': 'MID', 'color': (255, 200, 100)},
}
NUM_COMMANDS = len(NAV_COMMANDS)
COMMAND_KEYS = [pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5, 
                pygame.K_6, pygame.K_7, pygame.K_8]
# ═══════════════════════════════════════════════════════════════════════════════


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
        self.nav_cmd_pub = self.create_publisher(Int32, '/nav_command', 10)
        
        self.wheelbase = 1.75
        self.max_speed = 8.0
        self.max_reverse = 4.0
        self.max_yaw_rate = 1.2
        
        self.speed = SmoothValue(0.0, smoothing=0.15)
        self.steering = SmoothValue(0.0, smoothing=0.12)
        
        self.accel_rate = 5.0
        self.brake_rate = 10.0
        self.steer_rate = 1.5
        
        self.gear = 'N'
        self.current_command = 0
        self.start_time = time.time()
        self.max_speed_reached = 0.0
        self.distance = 0.0
        
        pygame.init()
        pygame.display.set_caption("OTAGG Control")
        
        self.width = 700
        self.height = 480
        self.screen = pygame.display.set_mode((self.width, self.height))
        self.clock = pygame.time.Clock()
        
        self.font_sm = pygame.font.Font(None, 22)
        self.font_md = pygame.font.Font(None, 32)
        self.font_lg = pygame.font.Font(None, 52)
        self.font_xl = pygame.font.Font(None, 80)
        
        self.running = True
        self.get_logger().info("OTAGG Controller started. Keys 1-5 for nav commands.")

    def run(self):
        while self.running and rclpy.ok():
            dt = self.clock.tick(60) / 1000.0
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    self._handle_command_key(event.key)
            
            keys = pygame.key.get_pressed()
            if keys[pygame.K_ESCAPE] or keys[pygame.K_q]:
                self.running = False
            
            self.process_input(keys, dt)
            self.update_smooth()
            self.publish_cmd()
            self.update_stats(dt)
            self.draw()
            
            rclpy.spin_once(self, timeout_sec=0)
        
        self.cleanup()

    def _handle_command_key(self, key):
        for i, cmd_key in enumerate(COMMAND_KEYS):
            if key == cmd_key and i < NUM_COMMANDS:
                if self.current_command != i:
                    self.current_command = i
                    self.get_logger().info(f"Command: [{i+1}] {NAV_COMMANDS[i]['name']}")
                    msg = Int32()
                    msg.data = i
                    self.nav_cmd_pub.publish(msg)
                break

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

    def update_stats(self, dt):
        spd = abs(self.speed.value)
        if spd > self.max_speed_reached:
            self.max_speed_reached = spd
        self.distance += spd * dt

    def draw(self):
        self.screen.fill(BLACK)
        
        pygame.draw.rect(self.screen, DARK, (0, 0, self.width, 45))
        pygame.draw.line(self.screen, CYAN, (0, 45), (self.width, 45), 2)
        title = self.font_md.render("◈ OTAGG CONTROL ◈", True, CYAN)
        self.screen.blit(title, (self.width//2 - title.get_width()//2, 10))
        
        self._draw_speed_panel(25, 55, 280, 165)
        self._draw_steering_panel(320, 55, 170, 165)
        self._draw_inputs_panel(505, 55, 170, 165)
        self._draw_command_panel(25, 230, 650, 70)
        self._draw_telemetry(25, 310, 650, 65)
        self._draw_status_bar()
        
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

    def _draw_command_panel(self, x, y, w, h):
        cmd_color = NAV_COMMANDS[self.current_command]['color']
        pygame.draw.rect(self.screen, DARK, (x, y, w, h), border_radius=8)
        pygame.draw.rect(self.screen, cmd_color, (x, y, w, h), 2, border_radius=8)
        
        self.screen.blit(self.font_sm.render("NAV COMMAND", True, LIGHT), (x+10, y+8))
        
        btn_w, btn_h = 75, 36
        btn_y = y + 27
        btn_x_start = x + 10
        
        for i in range(NUM_COMMANDS):
            btn_x = btn_x_start + i * (btn_w + 4)
            is_active = (i == self.current_command)
            color = NAV_COMMANDS[i]['color'] if is_active else MID
            pygame.draw.rect(self.screen, color, (btn_x, btn_y, btn_w, btn_h), border_radius=5)
            
            key_txt = self.font_sm.render(f"{i+1}", True, BLACK if is_active else LIGHT)
            self.screen.blit(key_txt, (btn_x + 3, btn_y + 2))
            
            txt_color = BLACK if is_active else LIGHT
            cmd_txt = self.font_sm.render(NAV_COMMANDS[i]['short'], True, txt_color)
            self.screen.blit(cmd_txt, (btn_x + btn_w//2 - cmd_txt.get_width()//2 + 5, btn_y + 10))

    def _draw_telemetry(self, x, y, w, h):
        pygame.draw.rect(self.screen, DARK, (x, y, w, h), border_radius=8)
        pygame.draw.rect(self.screen, GREEN, (x, y, w, h), 2, border_radius=8)
        
        self.screen.blit(self.font_sm.render("TELEMETRY", True, LIGHT), (x+10, y+8))
        
        elapsed = time.time() - self.start_time
        stats = [
            ("TIME", f"{int(elapsed//60):02d}:{int(elapsed%60):02d}"),
            ("MAX", f"{self.max_speed_reached*3.6:.0f} km/h"),
            ("DIST", f"{self.distance:.0f} m"),
            ("CMD", NAV_COMMANDS[self.current_command]['name']),
        ]
        
        ox = 15
        for name, val in stats:
            self.screen.blit(self.font_sm.render(name, True, LIGHT), (x + ox, y + 25))
            self.screen.blit(self.font_md.render(val, True, WHITE), (x + ox, y + 38))
            ox += 165

    def _draw_status_bar(self):
        pygame.draw.rect(self.screen, DARK, (0, self.height - 35, self.width, 35))
        pygame.draw.line(self.screen, CYAN, (0, self.height - 35), (self.width, self.height - 35), 1)
        
        txt = "W/S: Speed  •  A/D: Steer  •  SPACE: Brake  •  1-8: Commands  •  Q: Quit"
        self.screen.blit(self.font_sm.render(txt, True, LIGHT), (self.width//2 - 270, self.height - 25))
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
