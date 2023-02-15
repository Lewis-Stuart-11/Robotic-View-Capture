from math import pi, sin, cos, sqrt, acos
import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np 
import geometry_msgs.msg
import os

class TrajectoryHandler(object):
    def __init__(self, X_OFFSET, Y_OFFSET, MAX_DIST, MIN_Z, save_directory="", experiment_name=""):
        self.all_positions = []

        self.predicted_positions = {"valid_positions": {}, "invalid_positions": {}}

        self.traversed_positions = {"valid_positions": {}, "invalid_positions": {}}

        self.pos_dist = lambda pos: sqrt(pos.x**2 + pos.y**2 + pos.z**2)
        self.is_pos_outside_centre = lambda pos: (pos.x > X_OFFSET or pos.x < -X_OFFSET) or (pos.y > Y_OFFSET or pos.y < -Y_OFFSET)
        self.is_pos_outside_boundaries = lambda pos: (self.pos_dist(pos) < MAX_DIST) and (pos.z > MIN_Z)
        
        self.is_pos_valid = lambda pos: self.is_pos_outside_centre(pos) and self.is_pos_outside_boundaries(pos)

        self.current_point_idx = 0
        self.current_failed_point_idx = 0

        self.save_directory = save_directory
        self.experiment_name = experiment_name

    def calculate_sphere_points(self, sphere_origin, radius, rings=10, sectors=20, cut_off=0.3):
        ring_d = 1.0/(rings)
        sect_d = 1.0/(sectors)

        ring_d *= (1-cut_off)

        for sect in range(sectors):
            for ring in range(1, rings+1):

                x = cos(2 * pi * sect * sect_d) 
                y = sin(2 * pi * sect * sect_d) 
                
                x *= sin(pi * ring * ring_d)
                y *= sin(pi * ring * ring_d)

                z = -sin(-(pi/2) + (pi * ring * ring_d))

                self.add_and_validate_point(x, y, z, sphere_origin, radius)

        self.add_and_validate_point(0, 0, 1, sphere_origin, radius)
        

    def add_and_validate_point(self, x, y, z, sphere_origin, radius):
        # type: (float, float, float, list, float, bool)

        new_point = geometry_msgs.msg.Point()

        new_point.x = x * radius
        new_point.y = y * radius
        new_point.z = z * radius
        
        new_point.x += sphere_origin.x
        new_point.y += sphere_origin.y
        new_point.z += sphere_origin.z
    
        self.all_positions.append(new_point)

        if self.is_pos_valid(new_point):
            self.predicted_positions["valid_positions"][len(self.all_positions)-1] = new_point
        else:
            self.predicted_positions["invalid_positions"][len(self.all_positions)-1] = new_point

    def visualise_predicted_valid_points(self, save=False):
        self.visualise_points(self.predicted_positions, save=save, predicted_points=True)

    def visualise_traversed_points(self, save=False):
        self.visualise_points(self.traversed_positions, save=save)

    def visualise_points(self, positions, show_order=True, save=False, predicted_points=False):
        
        visualisation = {"valid_positions": {"colour": "b", "marker": "^"},
                         "invalid_positions": {"colour": "r", "marker": "o"}}

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        for point_type, points in positions.items():
            for i, point in points.items():
                ax.scatter(point.x, point.y, point.z, c=visualisation[point_type]["colour"], marker=visualisation[point_type]["marker"])
                if show_order:
                    ax.text(point.x, point.y, point.z, (str(i+1)), size=10, zorder=1, color=visualisation[point_type]["colour"])

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        plt.title(self.experiment_name.replace("_", " ").capitalize())

        if save:
            save_fig_file = self.experiment_name 
            save_fig_file += "_predicted_points" if predicted_points else "_traversed_points"

            save_fig_path = os.path.join(self.save_directory, save_fig_file + ".png")

            plt.savefig(save_fig_path)

        plt.show()

    def get_next_pos(self):
        if self.current_point_idx >= len(self.all_positions):
            return None, None

        current_index = self.current_point_idx
        current_pos = self.all_positions[current_index]  

        self.current_point_idx += 1

        return current_index, current_pos

    def get_failed_pos(self):
        if self.current_failed_point_idx >= len(self.traversed_positions["invalid_positions"]):
            return None, None

        current_pos = self.traversed_positions["invalid_positions"].values()[self.current_failed_point_idx]  
        current_index = self.traversed_positions["invalid_positions"].keys()[self.current_failed_point_idx]

        self.current_failed_point_idx += 1

        return current_index, current_pos


    def pos_verdict(self, current_pos_idx, success):
        current_pos = self.all_positions[current_pos_idx] 
        
        if success:
            self.traversed_positions["valid_positions"][current_pos_idx] = current_pos
            if current_pos_idx in self.traversed_positions["invalid_positions"].keys():
                del self.traversed_positions["invalid_positions"][current_pos_idx]
        else:
            self.traversed_positions["invalid_positions"][current_pos_idx] = current_pos

    def save_positions(self):
        with open(self.save_directory+"_points.txt", "w") as positions_file:
            positions_file.write("All Positions:\n")
            for idx, position in enumerate(self.all_positions):
                positions_file.write(str(idx) + ": " + str(position.x) + " " + str(position.y) + " " + str(position.z) + "\n")
            
            positions_file.write("\n")

            for pos_type, positions in self.traversed_positions.items():
                positions_file.write(pos_type.replace("_", " ").capitalize() + "\n")
                for idx, position in positions.items():
                    positions_file.write(str(idx) + ": " + str(position.x) + " " + str(position.y) + " " + str(position.z) + "\n")
            
                positions_file.write("\n")