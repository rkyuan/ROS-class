This node expands the range of the lidar alarm by considering the 100 scans to the left and right of the center point.
the alarm message is modified from a bool to an int.
normal, non-alarm operation is indicated by a 0 on the laser alarm message, a 1 represents an obstacle on the right and a 2 represents an obstacle on the left.
The commander is also modified to recieve these commands so that it rotates to avoid obstacles in the most sensible manner and it randomly bounces around the map.
