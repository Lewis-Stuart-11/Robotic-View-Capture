#!/usr/bin/env python

import time
import geometry_msgs
import json
import os

from trajectory_handler import TrajectoryHandler
from robot_control import RobotControl
from quartonian_handler import QuartonianHandler
from db_handler import ImgCaptureDatabaseHandler

dc_to_wc = None
wc_to_dc = None

up = geometry_msgs.msg.Point()
up.x = 0
up.y = 0
up.z = 1

def convert_list_to_point(point_list, convert_coords=False):
    # type: (list, bool) -> geometry_msgs.msg.Point

    if convert_coords:
        point_list = [wc_to_dc(point) for point in point_list]

    new_point = geometry_msgs.msg.Point()

    new_point.x = point_list[0]
    new_point.y = point_list[1]
    new_point.z = point_list[2]

    return new_point

def take_snapshot(current_pos_idx):
    time.sleep(1.0)

"""
def move_robot_base(new_base_pos, current_base_pos):
    success = True

    return success

def calculate_new_base_position(pos_to_move_base, current_base_pos, radius_from_origin, quartonian_handler):
    flat_pos = pos_to_move_base
    flat_pos.z = 0

    normalised_flat_pos = quartonian_handler.Normalize(flat_pos)
    
    normalised_flat_pos.x *= radius_from_origin 
    normalised_flat_pos.y *= radius_from_origin

    if normalised_flat_pos.x != current_base_pos.x or normalised_flat_pos.y != current_base_pos.y or normalised_flat_pos.z != current_base_pos.z:

        move_success = move_robot_base(normalised_flat_pos, current_base_pos)

        if move_success:
            current_base_pos = normalised_flat_pos

            print "Successfully moved base to position: " + current_base_pos.x + ", " + current_base_pos.y + ", " + current_base_pos.z
        else:
            print "Failed to move base to position: " + current_base_pos.x + ", " + current_base_pos.y + ", " + current_base_pos.z

    return current_base_pos
"""

def generate_objects(robot, args, scene_obj_positions):

    robot.add_sphere_to_scene("plant", scene_obj_positions["main_obj"], args.main_obj_radius, attach=False)

    robot.add_box_to_scene("camera", scene_obj_positions["start"], args.camera_size, attach=True)

    if args.auto_add_obj_stand:
        stand_height = scene_obj_positions["main_obj"].z - args.main_obj_radius

        stand_position = scene_obj_positions["main_obj"]
        stand_position.z = stand_height/2

        stand_size = [0.07, 0.07, stand_height]

        robot.add_box_to_scene("stand", stand_position, stand_size, attach=False)

    if args.include_extra_obj:
        with open(args.scene_objs, "r") as scene_file:
            json_file = json.load(scene_file)

            objects = json_file["objects"]

            for object in objects:
                obj_pos = convert_list_to_point(object["position"], convert_coords=True)
                obj_size = [wc_to_dc(size) for size in object["size"]]

                if object["type"] == "mesh":
                    robot.add_mesh_to_scene(object["name"], obj_pos, obj_size,
                                            object["mesh_file_name"], attach=bool(object["attach"]),)
                elif object["type"] == "box":
                    robot.add_box_to_scene(object["name"], obj_pos, obj_size, 
                                            attach=True if "attach" in object.keys() else False)
                elif object["type"] == "sphere":
                    robot.add_sphere_to_scene(object["name"], obj_pos, obj_size, 
                                            attach=True if "attach" in object.keys() else False)
                else:
                    raise Exception("Object type " + object["type"] + " is not currently supported")


def test(robot, quartonian_handler):
    origin = convert_list_to_point([0.705882352941, 0.0, 1.0])

    test_poses = [
        convert_list_to_point([0.5,0.5,1]),
        convert_list_to_point([-0.5,0.5,1]),
        convert_list_to_point([-0.5,-0.5,1]),
        convert_list_to_point([0.5,-0.5,1])
    ]

    for pose in test_poses:
        quartonian = quartonian_handler.QuaternionLookRotation(quartonian_handler.SubtractVectors(origin, pose), up)

        success = robot.move_and_orientate_arm(pose.x, pose.y, pose.z,
                                                quartonian.x, quartonian.y, quartonian.z, quartonian.w)
        
        time.sleep(4)


def config_parser():

    import configargparse
    parser = configargparse.ArgumentParser()

    # Config Handling
    parser.add_argument("--config", is_config_file=True, help="Config file path")
    parser.add_argument("--log_dir", type=str, default="", help="Log file path")
    parser.add_argument("--experiment_name", type=str, required=True, help="Name of this experiment iteration")
    parser.add_argument("--print", action="store_true", default=False, help="Print useful info to stdout")
    parser.add_argument("--visualise", action="store_true", default=False, help="Generate scatter diagrams of positions")
    parser.add_argument("--save_fig", action="store_true", default=False, help="Whether to save the generated visualisation")
    
    # Database Handling
    parser.add_argument("--save_to_db", action="store_true", default=True, help="Whether to save experiment to database")
    parser.add_argument("--continue_experiment", action="store_true", default=False, 
                        help="Whether to continue training an experiment if the name already exists in the database")
    parser.add_argument("--replace_stored_experiment", action="store_true", default=False, 
                        help="Whether to replace the stored experiment or increment the current experiment name")

    # Scene Objects
    parser.add_argument("--main_obj_position", type=float, action="append", default=[], required=True,
                        help="The object position in relation to the base position (cm)")
    parser.add_argument("--main_obj_radius", type=float, required=True,
                        help="The known radius of the object (cm)")
    parser.add_argument("--starting_position", type=float, action="append", default=[], required=True,
                        help="The object starting position in relation to the base position (cm)")
    parser.add_argument("--camera_size", type=float, action="append", default=[], required=True,
                        help="Size of the camera in: width, depth, height (cm)")
    
    parser.add_argument("--auto_add_obj_stand", action="store_true", default=False,
                        help="Automatically adds the object stand based on its starting position")
    parser.add_argument("--include_extra_obj", action="store_true", default=False,
                        help="Set to allow extra objects to be added from json object file")
    parser.add_argument("--scene_objs", type=str, default="", help="Object JSON file path")

    # View Generation
    parser.add_argument("--rings", type=int, default=7, 
                         help="The number of rings in the generated sphere of capture views")
    parser.add_argument("--sectors", type=int, default=14, 
                         help="The number of sectors in the generated sphere of capture views")
    parser.add_argument("--camera_dist_from_obj_origin", type=float, required=True,
                        help="The distance of the camera to the origin when calculating views (cm)")

    # Movement Handling
    parser.add_argument("--planning_time", type=float, default=3.0, 
                         help="Number of seconds per planning attempt")
    parser.add_argument("--num_move_attempts", type=int, default=2, 
                         help="Number of move attempts before moving to next position in queue")
    parser.add_argument("--retry_failed_pos", action="store_true", default=False,
                         help="Set to attempt to move to previously failed positions")
    
    # Robot DC Settings (default for UR5 robot)
    parser.add_argument("--restricted_x", type=float, default=0.05, help="Minumum X value for a self-collision")
    parser.add_argument("--restricted_y", type=float, default=0.05, help="Minumum Y value for a self-collision")
    parser.add_argument("--restricted_z", type=float, default=-0.1, help="Minumum Z value for a self-collision")
    parser.add_argument("--max_dist", type=float, default=1, 
                        help="Maximum distance from base for robotic reach (normally always 1)")

    # Robot WC Settings (default for UR5 robot)
    parser.add_argument("--max_reach", type=float, default=85, 
                        help="Maximum distance from base for robotic reach (cm)")
    parser.add_argument("--dynamic_base", action="store_true", default=False, 
                        help="Set to allow the base to move for the best chance to reach a certain position")


    args = parser.parse_args()

    # Converst real world coordinates into device coordinates
    global dc_to_wc, wc_to_dc
    dc_to_wc = lambda x: float(x) * float(args.max_reach) 
    wc_to_dc = lambda x: float(x) / float(args.max_reach) 

    # Checks that plant radius is not larger than the camera capture radius 
    if args.camera_dist_from_obj_origin <= args.main_obj_radius:
        raise Exception("Distance of the camera to the plant origin must be bigger than the max plant radius (avoids collision)")

    # Number of move attempts must be larger than 0
    if args.num_move_attempts <= 0:
        raise Exception("Number of move attempts must be larger than 0")
    
    # Stores the positions of important objects/points in the scene in the correct object format
    scene_obj_positions = {
        "main_obj": convert_list_to_point(args.main_obj_position, convert_coords=True),
        "start": convert_list_to_point(args.starting_position, convert_coords=True),
        "current_base": convert_list_to_point([0,0,0]),
        "scene_origin": convert_list_to_point([0,0,0])
    }

    # Values are converted into the correct coordinate system
    args.camera_size = [wc_to_dc(camera_dim) for camera_dim in args.camera_size]
    args.main_obj_radius = wc_to_dc(args.main_obj_radius)
    args.camera_dist_from_obj_origin = wc_to_dc(args.camera_dist_from_obj_origin)

    return args, scene_obj_positions


def save_experiment_in_db(args, db_handler, experiment_name):
    if db_handler.get_experiment_with_name(experiment_name):
        if not args.continue_experiment:
            if not args.replace_stored_experiment:
                db_name_i = 1
                experiment_name += "_" + str(db_name_i)

                while db_handler.get_experiment_with_name(experiment_name):
                    experiment_name = experiment_name[:-1]
                    experiment_name += str(db_name_i)
                    db_name_i+= 1
            else:
                db_handler.remove_experiment_with_name(experiment_name)

            db_handler.create_new_experiment(experiment_name, args.main_obj_position, args.main_obj_radius, 
                                            args.camera_dist_from_obj_origin, args.rings, args.sectors)
    else:
        db_handler.create_new_experiment(experiment_name, args.main_obj_position, args.main_obj_radius, 
                                            args.camera_dist_from_obj_origin, args.rings, args.sectors)

    return db_handler


def main():

    args, scene_obj_positions = config_parser()

    experiment_name_condensed = args.experiment_name.replace(" ", "_").lower()

    experiment_file_name = os.path.join(args.log_dir, experiment_name_condensed)

    db_handler = None

    if args.save_to_db:

        db_handler = ImgCaptureDatabaseHandler(args.log_dir)

        db_handler = save_experiment_in_db(args, db_handler, experiment_name_condensed)

    # Handles all control of the UR5 and Scene
    robot = RobotControl(num_move_attempts=args.num_move_attempts)

    # Handles all vectors and quaronian logic for the robot
    quartonian_handler = QuartonianHandler()
    
    # Handles all positions that the robot needs to traverse to  
    trajectory_handler = TrajectoryHandler(args.restricted_x, args.restricted_y, 
                                           args.max_dist, args.restricted_z, 
                                           save_directory=experiment_file_name)

    # Calculates all camera view positions in the scene based on where the object is and the number
    # of required views to be captured
    trajectory_handler.calculate_sphere_points(scene_obj_positions["main_obj"], 
                                               args.camera_dist_from_obj_origin, 
                                               rings=args.rings, sectors=args.sectors, swap_y=True)

    print str(len(trajectory_handler.predicted_positions["invalid_positions"])) + " different positions have been evaluated as invalid! "

    # Shows a scatter diagram of the calculated valid and invalid positions in the scene
    if args.visualise:
        trajectory_handler.visualise_predicted_valid_points(save=args.save_fig)

    print "Enter anything to continue: "

    continue_process = raw_input()

    if continue_process == "exit":
        exit()

    robot.set_planning_time(args.planning_time)

    # Attempts to moves arm to the set starting position
    success = robot.move_arm(scene_obj_positions["start"].x, 
                             scene_obj_positions["start"].y, 
                             scene_obj_positions["start"].z)

    #robot.move_group.set_planner_id("RRTConnectkConfigDefault")

    if success:
        print "Moved robot to starting position, adding scene objects"
    else:
        raise Exception("Failed to move robot to starting position")

    # Generates all the required objects in the scene to ensure no collisions occur
    generate_objects(robot, args, scene_obj_positions)

    print "Beginning capturing..."

    # Gets the current position for the robot to traverse to and continues to loop until no more 
    # positions in the queue
    current_pos_idx, current_pos = trajectory_handler.get_next_pos()
    while current_pos != None:
        print str(current_pos_idx)

        if args.save_to_db and args.continue_experiment:
            current_point = db_handler.get_point_with_num(current_pos_idx)

            if current_point is not None and current_point[2] and current_point[3]:
                print "Point has already been traveresed previously, skipping..."

                current_pos_idx, current_pos = trajectory_handler.get_next_pos()

                continue

        """if args.dynamic_base:
            
            scene_obj_positions["current_base"] = calculate_new_base_position(current_pos, 
                                                           scene_obj_positions["current_base"], 
                                                           args.camera_dist_from_obj_origin, 
                                                           quartonian_handler)"""

        print "Attempting to move to position: " + str(current_pos.x) + ", " + str(current_pos.y) + ", " + str(current_pos.z)

        quartonian = quartonian_handler.QuaternionLookRotation(quartonian_handler.SubtractVectors(scene_obj_positions["main_obj"], current_pos), up)

        success = robot.move_and_orientate_arm(current_pos.x, current_pos.y, current_pos.z,
                                                quartonian.x, quartonian.y, quartonian.z, quartonian.w)

        if success:
            print "Successfully moved arm to new position"
            take_snapshot(current_pos_idx)
            print "Snapshot taken"
                
        else:
            print "Unable to move arm to position after " + str(args.num_move_attempts) + " attempts"
        
        if args.save_to_db:
            db_handler.update_point_status(current_pos_idx, success)

        print ""

        trajectory_handler.pos_verdict(current_pos_idx, success)

        current_pos_idx, current_pos = trajectory_handler.get_next_pos()

    print trajectory_handler.traversed_positions["invalid_positions"]

    if args.retry_failed_pos:
        print "Retrying previously failed positions"
        print ""

        current_pos_idx, current_pos = trajectory_handler.get_failed_pos()
        while current_pos != None:
            print str(current_pos_idx)
            print "Reattempting to move to position: " + str(current_pos.x) + ", " + str(current_pos.y) + ", " + str(current_pos.z)

            quartonian = quartonian_handler.QuaternionLookRotation(quartonian_handler.SubtractVectors(scene_obj_positions["main_obj"], current_pos), up)

            success = robot.move_and_orientate_arm(current_pos.x, current_pos.y, current_pos.z,
                                                   quartonian.x, quartonian.y, quartonian.z, quartonian.w)

            if success:
                print "Successfully moved arm to new position"
                take_snapshot(current_pos_idx)
                print "Snapshot taken"

                trajectory_handler.pos_verdict(current_pos_idx, success)

                if args.save_to_db:
                    db_handler.update_point_status(current_pos_idx, success)
            else:
                print "Unable to move arm to position after " + str(args.num_move_attempts) + " attempts"

            print ""

            current_pos_idx, current_pos = trajectory_handler.get_failed_pos()

    db_handler.close_database()

    trajectory_handler.save_positions()

    if args.visualise:
        trajectory_handler.visualise_traversed_points(save=args.save_fig)

if __name__ == '__main__':
  main()