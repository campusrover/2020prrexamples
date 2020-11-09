
#!/usr/bin/env python
import csv

def kalman_update(elapsed, distance, bearing):
    return (distance, bearing)

def kalman_control(elapsed, distance, bearing, g_forward_cmd, g_turn_cmd):
    return (distance, bearing)

# Main program starts here
if __name__ == '__main__':
# fields are: elapsed,g_forward_cmd,g_turn_cmd,g_shortest_bearing,g_shortest
    with open('kalman_test_data.csv', "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            elapsed = row['elapsed']
            forward_speed = row['g_forward_cmd']
            rotation_speed = row['g_turn_cmd']
            bearing = row['g_shortest_bearing']
            distance = row['g_shortest']

