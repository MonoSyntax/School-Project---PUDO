#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import MaterialColor
from std_msgs.msg import ColorRGBA

class EntityColorChanger(Node):
    def __init__(self):
        super().__init__('entity_color_changer')

        self.publisher_ = self.create_publisher(MaterialColor, '/world/default/material_color', 10)

        self.timer = self.create_timer(5.0, self.change_entity_color)

        # Başlangıçta kırmızıyla başlasın
        self.current_light = 'red'
        self.light_order = ['red', 'yellow', 'green', 'yellow']
        self.light_index = 0

    def change_entity_color(self):
        msg = MaterialColor()

        # Mevcut ışık sırasındaki rengi ayarla
        current_light_name = self.light_order[self.light_index]

        # Işığı sırasıyla değiştir
        msg.entity_match = 1
        msg.entity.name = current_light_name

        ambient_color = ColorRGBA()
        specular_color = ColorRGBA()
        emissive_color = ColorRGBA()

        # Emissive rengi sıfırlamak için önce tüm ışıkları söndür
        off_emissive = ColorRGBA()
        off_emissive.r = 0.0
        off_emissive.g = 0.0
        off_emissive.b = 0.0
        off_emissive.a = 0.0 
        
        # Renk seçimine göre ışıkları ayarla
        if current_light_name == 'red':
            ambient_color.r = 1.0
            ambient_color.g = 0.0
            ambient_color.b = 0.0

            specular_color.r = 1.0
            specular_color.g = 0.0
            specular_color.b = 0.0

            emissive_color.r = 1.0
            emissive_color.g = 0.0
            emissive_color.b = 0.0  

            # Sarı ve yeşil ışıkları söndür
            self.turn_off_light('yellow')
            self.turn_off_light('green')
        elif current_light_name == 'yellow':
            ambient_color.r = 1.0
            ambient_color.g = 1.0
            ambient_color.b = 0.0

            specular_color.r = 1.0
            specular_color.g = 1.0
            specular_color.b = 0.0

            emissive_color.r = 1.0
            emissive_color.g = 1.0
            emissive_color.b = 0.0  

            # Kırmızı ve yeşil ışıkları söndür
            self.turn_off_light('red')
            self.turn_off_light('green')
        elif current_light_name == 'green':
            ambient_color.r = 0.0
            ambient_color.g = 1.0
            ambient_color.b = 0.0

            specular_color.r = 0.0
            specular_color.g = 1.0
            specular_color.b = 0.0

            emissive_color.r = 0.0
            emissive_color.g = 1.0
            emissive_color.b = 0.0  

            # Kırmızı ve sarı ışıkları söndür
            self.turn_off_light('red')
            self.turn_off_light('yellow')

        ambient_color.a = 1.0
        specular_color.a = 0.0
        emissive_color.a = 1.0
        msg.ambient = ambient_color
        msg.specular = specular_color
        msg.emissive = emissive_color

        # Mesajı yayınla
        self.publisher_.publish(msg)

        # Bir sonraki ışığa geç
        self.light_index = (self.light_index + 1) % len(self.light_order)

    def turn_off_light(self, light_name):
        """Belirtilen ışığın söndürülmesini sağlar."""
        msg = MaterialColor()
        msg.entity_match = 1
        msg.entity.name = light_name

        off_emissive = ColorRGBA()
        off_emissive.r = 0.0
        off_emissive.g = 0.0
        off_emissive.b = 0.0
        off_emissive.a = 0.0  

        msg.emissive = off_emissive

        # Işığı söndür
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    entity_color_changer = EntityColorChanger()

    try:
        rclpy.spin(entity_color_changer)
    except KeyboardInterrupt:
        pass

    entity_color_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
