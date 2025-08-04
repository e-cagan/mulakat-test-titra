import rclpy
from rclpy.node import Node
import time

class FlightController(Node):
    def __init__(self, mavros_interface=None):
        super().__init__('flight_controller')
        self.mavros = mavros_interface
        self.get_logger().info("Test Flight Controller başlatıldı.")
    
    def task_1_arm_and_disarm(self):
        self.get_logger().info("Görev 1 başlatılıyor: Arm ve Disarm...")
        # Test amaçlı - bu çalışırsa testler kesin çalışıyor demektir
        raise NotImplementedError("BAŞARILI! Görev 1 testi çalışıyor - Kod yazılmadı.")
    
    def task_2_takeoff_and_land(self):
        self.get_logger().info("Görev 2 başlatılıyor: Kalkış ve İniş...")
        # Test amaçlı - bu çalışırsa testler kesin çalışıyor demektir
        raise NotImplementedError("BAŞARILI! Görev 2 testi çalışıyor - Kod yazılmadı.")

def main():
    rclpy.init()
    controller = FlightController()
    try:
        # Hangi task çağrılırsa çağrılsın, hata verecek
        import sys
        if len(sys.argv) > 1:
            if sys.argv[1] == "task1":
                controller.task_1_arm_and_disarm()
            elif sys.argv[1] == "task2":
                controller.task_2_takeoff_and_land()
    except NotImplementedError as e:
        controller.get_logger().error(f"BEKLENEN TEST HATASI: {e}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()