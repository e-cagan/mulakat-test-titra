import rclpy
import sys
from mavros_interface.node import MavrosInterface
from candidate_package.flight_logic import FlightController

def main(args=None):
    rclpy.init(args=args)
    
    # Komut satırından hangi görevin çalıştırılacağını al
    if len(sys.argv) < 2:
        print("Lütfen bir görev belirtin: task1, task2")
        return

    task_name = sys.argv[1]

    # Gerekli düğümleri başlat
    try:
        mavros_interface = MavrosInterface()
        flight_controller = FlightController(mavros_interface)
        
        # Belirtilen görevi çalıştır
        if task_name == 'task1':
            flight_controller.task_1_arm_and_disarm()
        elif task_name == 'task2':
            flight_controller.task_2_takeoff_and_land()
        else:
            flight_controller.get_logger().error(f"Bilinmeyen görev: {task_name}")
    finally:
        # Her durumda düğümleri kapat ve rclpy'ı sonlandır
        if 'flight_controller' in locals() and flight_controller.executor:
            flight_controller.destroy_node()
        if 'mavros_interface' in locals() and mavros_interface.executor:
            mavros_interface.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
