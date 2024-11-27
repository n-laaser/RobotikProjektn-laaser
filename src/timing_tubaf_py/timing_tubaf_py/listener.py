import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Duration


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        # Subscriber für den String-Nachrichtentyp auf dem 'number'-Kanal
        self.subscription = self.create_subscription(
            String,
            'number',
            self.topic_callback,
            10
        )
        self.publisher = self.create_publisher(
            Duration,  # Nachrichtentyp für die Zeitdifferenz
            'diff',
            10
        )
        self.last_message_time = None  # Letzter Zeitpunkt (nur für die Differenzberechnung)
        self.message_count = 0  # Zähler der Nachrichten

    def topic_callback(self, msg):
        # Holen des aktuellen ROS-Clock-Stempels relativ zur Node-Startzeit
        current_message_time = self.get_clock().now().nanoseconds

        # Nachrichtenanzahl inkrementieren
        self.message_count += 1

        # Wenn bereits eine vorherige Nachricht empfangen wurde, Differenz berechnen
        if self.last_message_time is not None:
            # Zeitdifferenz berechnen (in Nanosekunden)
            time_diff_ns = current_message_time - self.last_message_time

            # Log-Ausgabe der Differenz
            time_diff_s = time_diff_ns / 1e9
            self.get_logger().info(f"Time difference: {time_diff_s:.6f} seconds")

            # Nachricht mit der Differenz erstellen und veröffentlichen
            diff_msg = Duration()
            diff_msg.sec = int(time_diff_ns // 1e9)  # Sekunden aus der Differenz
            diff_msg.nanosec = int(time_diff_ns % 1e9)  # Restliche Nanosekunden
            self.publisher.publish(diff_msg)

        # Aktuellen Zeitstempel für die nächste Nachricht speichern
        self.last_message_time = current_message_time

        # Die empfangene Nachricht ausgeben
        self.get_logger().info(f"I heard message {self.message_count}: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    # Instanz des Subscribers erstellen
    minimal_subscriber = MinimalSubscriber()
    # Node ausführen
    rclpy.spin(minimal_subscriber)
    # Shutdown von rclpy nach dem Stoppen des Nodes
    rclpy.shutdown()


if __name__ == '__main__':
    main()

