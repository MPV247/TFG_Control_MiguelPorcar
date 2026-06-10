import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial

class SerialJointPublisher(Node):
    def __init__(self):
        super().__init__('serial_joint_publisher')

        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ser = None

        # Publicador para el estado de las articulaciones
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer a 100Hz (10ms) para emparejar la tasa del C2000
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'Nodo iniciado. Intentando conectar a {self.port}...')

        self.connect_serial()

    def connect_serial(self):
        try:
            if self.ser: 
                self.ser.close()
            # Añadimos un timeout corto para no bloquear el flujo de ROS
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            self.get_logger().info(f'Conectado exitosamente a {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo conectar al puerto: {e}. Reintentando...')

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            if self.ser.in_waiting > 0:
                # Lectura telemetría en crudo (formato CSV)
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                # Trama esperada: x, ref_x, theta, ref_theta, x_dot, theta_dot, u (7 elementos)
                parts = line.split(',')

                if len(parts) >= 7:
                    try:
                        # Extraemos estrictamente lo necesario para el Gemelo Digital
                        pos_carro = float(parts[0])     # x (m) -> Primera columna
                        ang_pendulo = float(parts[2])   # theta (rad) -> Tercera columna

                        # Crear el mensaje de ROS 2
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        
                        msg.name = ['cart_slider', 'pole_joint']
                        msg.position = [pos_carro, ang_pendulo]

                        self.publisher_.publish(msg)
                        
                    except ValueError:
                        # Ignora líneas corruptas o cadenas incompletas del buffer serie
                        pass
        except Exception as e:
            self.get_logger().warn(f'Error leyendo serial: {e}')
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()