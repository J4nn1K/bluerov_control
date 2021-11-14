import numpy as np
import rospy

class Path(object):
    def __init__(self, waypoints):
        self.target_index = 0
        self.waypoints = waypoints
        self.target_point = np.zeros([2])
    
    def update_target(self, position, look_ahead_distance):
        n_points = self.waypoints.shape[1]
        index = self.target_index
        success = False

        for _ in range(n_points):
            if index >= n_points:
                index -= 1
                break
            vector = self.waypoints[:,index] - position
            #point_square = np.inner(vector, vector)
            point_distance = np.linalg.norm(vector)
            if point_distance > look_ahead_distance:
                success = True
                break
            index += 1

        # if not success:
        #     if not loop:
        #         index = n_points - 1

        self.target_index = index % n_points
        self.target_point = self.waypoints[:,self.target_index]
        return success

    def get_target_point(self):
        return np.copy(self.target_point)


