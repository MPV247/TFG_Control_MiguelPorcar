import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class SerialJointPublisher(Node):
    def __init__(self):
        super().__init__('serial_joint_publisher')

        # Parámetros configurables (para no tocar código si cambias de puerto)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Configuración del Serial
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ser = None

        # Publicador
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer: intentará leer y publicar cada 0.01s (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f'Nodo iniciado. Intentando conectar a {self.port}...')

        self.connect_serial()

    def connect_serial(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Conectado exitosamente a {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'No se pudo conectar al puerto: {e}. Reintentando...')

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            self.connect_serial()
            return

        try:
            if self.ser.in_waiting > 0:
                # Leemos la línea del Arduino
                line = self.ser.readline().decode('utf-8').strip()

                # Formato esperado: "angulo_motor,angulo_pendulo" (en radianes o grados)
                # Ejemplo: "1.57,0.1"
                parts = line.split(',')

                if len(parts) == 2:
                    try:
                        q1 = float(parts[0]) # Brazo
                        q2 = float(parts[1]) # Péndulo

                        # Crear el mensaje
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.name = ['joint_motor', 'joint_pendulum']
                        msg.position = [q1, q2]

                        self.publisher_.publish(msg)
                    except ValueError:
                        pass # Si llega basura, la ignoramos
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
