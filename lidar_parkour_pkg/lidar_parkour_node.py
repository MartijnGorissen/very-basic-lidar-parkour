import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from my_robot_interfaces.msg import BesturingsData

class LidarParkourNode(Node):
    def __init__(self):
        super().__init__('lidar_parkour_node')
        self.LaserScan_subscription = self.create_subscription(LaserScan, '/scan', self.parkour_callback, 10)
        self.parkour_subscription = self.create_subscription(Bool, 'lidar_parkour_mode', self.controller_callback, 10)
        
        self.besturingsData_publisher = self.create_publisher(BesturingsData, 'besturings_data', 10)
        self.left_scan_publisher = self.create_publisher(LaserScan, '/scan/left', 10)
        self.right_scan_publisher = self.create_publisher(LaserScan, '/scan/right', 10)
        self.front_scan_publisher = self.create_publisher(LaserScan, '/scan/front', 10)

        self.lidar_parkour_mode = False

    def controller_callback(self, msg):
        # Ontvangen van de xbox controller, (de)activeren parkour mode
        self.lidar_parkour_mode = msg.data
        
    def parkour_callback(self, msg):
        if not self.lidar_parkour_mode:
            print(f'Waiting for activation.')
            return

        # bereken aantal metingen
        num_measurements = len(msg.ranges)
        angle_increment = (msg.angle_max - msg.angle_min) / num_measurements

        # voor radialen:
        # 210/180 * 3.14 (pi) = 3.66519
        # 240/180 * 3.14 (pi) = 4.18879
        # 120/180 * 3.14 (pi) = 2.094395
        # 150/180 * 3.14 (pi) = 2.61799
        # 160/180 * 3.14 (pi) = 2.7925
        # 200/180 * 3.14 (pi) = 3.49066

        left_angle_range = (3.66519, 4.18879)  # ~210 tot ~240 graden in radialen
        right_angle_range = (2.094395, 2.61799)  # ~120 tot ~150 graden in radialen
        front_angle_range = (2.7925, 3.49066) # ~160 tot ~200 graden in radialen

        # Bereken de index voor de linker-, rechterhoeken en voorkant
        left_start_index = int(left_angle_range[0] / angle_increment)
        left_end_index = int(left_angle_range[1] / angle_increment)
        right_start_index = int(right_angle_range[0] / angle_increment)
        right_end_index = int(right_angle_range[1] / angle_increment)
        front_start_index = int(front_angle_range[0]/ angle_increment)
        front_end_index = int(front_angle_range[1]/ angle_increment)

        # Metingen van de linker- en rechterhoek ophalen
        left_ranges = msg.ranges[left_start_index:left_end_index]
        right_ranges = msg.ranges[right_start_index:right_end_index]
        front_ranges = msg.ranges[front_start_index:front_end_index]

        # 'inf' waardes uit de lijst halen.
        left_ranges_filtered = []
        right_ranges_filtered = []

        for distance in left_ranges:
            if distance != float('inf'):
                left_ranges_filtered.append(distance)

        for distance in right_ranges:
            if distance != float('inf'):
                right_ranges_filtered.append(distance)


        # controleren of de lijsten niet leeg zijn, anders crasht het programma. 
        # Minimale en maximale afstanden berekenen
        # Als iets te dichtbij de lidar is (<0.15), komen er geen waardes binnen. Zonder de else crasht het programma omdat de max() functie geen waarde krijgt.
        
        if left_ranges_filtered:
            min_left_distance = min(left_ranges_filtered)
            max_left_distance = max(left_ranges_filtered)
        else:
            min_left_distance = 1.0
            max_left_distance = 5.0

        if right_ranges_filtered:
            min_right_distance = min(right_ranges_filtered)
            max_right_distance = max(right_ranges_filtered)
        else:
            min_right_distance = 1.0
            max_right_distance = 5.0

        min_front_distance = min(front_ranges)

        # Stuurgegevens initialiseren
        besturings_data = BesturingsData()
        besturings_data.throttle = 0.0
        besturings_data.brake = 0.0
        besturings_data.steering = 0.0
        besturings_data.direction = 1

        # grenswaarde, pas aan indien nodig
        min_threshold = 0.5
        max_threshold = 1.5

        min_differnce = abs(min_left_distance - min_right_distance)
        max_difference = abs(max_left_distance - max_right_distance)

        #throttle laag voor testen
        speed = 0.3
        stuur_factor = 0.5 
        
        # naar links sturen = -steering (negatief)
        # steering en throttle van -1 tot 1

        # obstakel op de weg, stuur naar de kant waar meer ruimte is.
        # dit is ver van perfect, misschien simpeler houden
        if min_front_distance < 1.25:
            print(f'Obstacle at {min_front_distance:0.3} meters.')
            if max_left_distance > max_right_distance and min_left_distance > min_right_distance:
                print(f'Obstakel op de weg. Naar links sturen.')
                besturings_data.steering = -0.5
                besturings_data.throttle = speed
            elif max_right_distance > max_left_distance and min_right_distance > min_left_distance:
                print(f'Obstakel op de weg. Naar rechts sturen')
                besturings_data.steering = 0.5
                besturings_data.throttle = speed
            elif min_left_distance > min_right_distance:
                print(f'Obstakel op de weg. Naar links sturen.')
                besturings_data.steering = -0.5
                besturings_data.throttle = speed
            else:
                print(f'Obstakel op de weg. Naar rechts sturen')
                besturings_data.steering = 0.5
                besturings_data.throttle = speed

        #minimum aan een kant is groter dan het maximum aan de andere kant, max stuurhoek (op dit moment in een bocht)
        elif min_left_distance > max_right_distance:
            besturings_data.steering = -1.0
            besturings_data.throttle = speed * 0.5
            print(f'Maximaal naar links sturen.')
        elif min_right_distance > max_left_distance:
            besturings_data.steering = 1.0
            besturings_data.throttle = speed * 0.5
            print(f'Maximaal naar rechts sturen.')

        # maximum verschil is groot, er komt een bocht aan
        elif max_difference > max_threshold:
            #dynamisch stuurhoek bepalen
            steering_adjustment = (max_difference - max_threshold) * stuur_factor
            steering_adjustment = max(-1.0, min(1.0, steering_adjustment))
            if max_left_distance > max_right_distance:
                #maximum is verder links, bocht naar links
                besturings_data.steering = float(-steering_adjustment)
                besturings_data.throttle = speed * 0.75
                print(f'Naar links sturen.')
            else:
                #maximum is verder rechts, bocht naar rechts
                besturings_data.steering = float(steering_adjustment)
                besturings_data.throttle = speed * 0.75
                print(f'Naar rechts sturen')

        elif min_left_distance > min_right_distance:
            steering_adjustment = (min_left_distance - min_right_distance) * stuur_factor
            steering_adjustment = max(-1.0, min(1.0, steering_adjustment))
            besturings_data.steering = -steering_adjustment
            besturings_data.throttle = speed
            print(f'Bijsturen naar links')
        else:
            steering_adjustment = (min_right_distance - min_left_distance) * stuur_factor
            steering_adjustment = max(-1.0, min(1.0, steering_adjustment))
            besturings_data.steering = steering_adjustment
            besturings_data.throttle = speed
            print(f'Bijsturen naar rechts')    

        # Verstuur de besturingsdata
        self.besturingsData_publisher.publish(besturings_data)
        print(f'Min Left: {min_left_distance:.3f}, Max Left: {max_left_distance:.3}, Min Right: {min_right_distance:.3}, Max Right: {max_right_distance:.3}, Steering: {besturings_data.steering}, Throttle: {besturings_data.throttle:.3f}')

        self.create_publish_laserscan(msg, left_start_index, left_end_index, left_angle_range, self.left_scan_publisher)
        self.create_publish_laserscan(msg, right_start_index, right_end_index, right_angle_range, self.right_scan_publisher)
        self.create_publish_laserscan(msg, front_start_index, front_end_index, front_angle_range, self.front_scan_publisher)

    def create_publish_laserscan(self, original_scan, start_index, end_index, angle_range, publisher):
        # selfmade laserscan voor de 3 area's infront
        scan = LaserScan()

        # Kopieer de header en andere metadata
        scan.header = original_scan.header
        scan.angle_min = angle_range[0]
        scan.angle_max = angle_range[1]
        scan.angle_increment = original_scan.angle_increment
        scan.time_increment = original_scan.time_increment
        scan.scan_time = original_scan.scan_time
        scan.range_min = original_scan.range_min
        scan.range_max = original_scan.range_max

        # Vul de `ranges`-waarden voor de opgegeven hoek
        scan.ranges = original_scan.ranges[start_index:end_index]

        # Publiceer het aangepaste LaserScan-bericht
        publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarParkourNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
