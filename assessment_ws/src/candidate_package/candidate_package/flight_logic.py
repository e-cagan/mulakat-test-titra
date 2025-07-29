import rclpy
from rclpy.node import Node
from mavros_interface.mavros_interface.node import MavrosInterface # Bir önceki adımda yazdığımız arayüzü import ediyoruz
import time

class FlightController(Node):
    def __init__(self, mavros_interface: MavrosInterface):
        super().__init__('flight_controller')
        self.mavros = mavros_interface
        self.get_logger().info("Adayın Uçuş Kontrolcüsü başlatıldı.")

    def task_1_arm_and_disarm(self):
        """
        GÖREV 1: Bu fonksiyon çağrıldığında, İHA'yı arm (çalıştırma) durumuna getirmeli,
        5 saniye beklemeli ve ardından disarm (durdurma) etmelidir.
        İPUCU: self.mavros.arm(), self.mavros.disarm() ve time.sleep() kullanın.
        """
        self.get_logger().info("Görev 1 başlatılıyor: Arm ve Disarm...")
        
        # ############### ADAYIN KODU BURAYA GELECEK ###############
        
        
        
        # ###########################################################
        
        self.get_logger().info("Görev 1 tamamlandı.")

    def task_2_takeoff_and_land(self):
        """
        GÖREV 2: Bu fonksiyon, İHA'yı 10 metre yüksekliğe otonom olarak kaldırmalı,
        ardından otonom olarak iniş yapmalıdır (LAND modunu kullanarak).
        İPUCU: self.mavros.takeoff(10.0) ve self.mavros.set_mode('LAND') kullanın.
        """
        self.get_logger().info("Görev 2 başlatılıyor: Kalkış ve İniş...")
        
        # ############### ADAYIN KODU BURAYA GELECEK ###############
        
        
        
        # ###########################################################

        self.get_logger().info("Görev 2 tamamlandı.")
