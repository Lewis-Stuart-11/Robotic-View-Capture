import sqlite3 as sq
import os


class ImgCaptureDatabaseHandler():
    def __init__(self, database_path, database_name="experiments.db"):
        self.database_file = os.path.join(database_path, database_name)

        create_database = not os.path.isfile(self.database_file)

        self.conn = sq.connect(self.database_file)
        self.cur = self.conn.cursor()

        self.current_experiment = None

        if create_database:
            self.cur.execute("""
                            CREATE TABLE experiments
                            (experiment_id INTEGER PRIMARY KEY AUTOINCREMENT,
                            experiment_name VARCHAR(50) NOT NULL UNIQUE,
                            obj_pos VARCHAR(20) NOT NULL,
                            obj_radius DOUBLE NOT NULL,
                            camera_dist_from_obj_origin DOUBLE NOT NULL,
                            rings DOUBLE NOT NULL,
                            sectors DOUBLE NOT NULL);
                            """)

            self.cur.execute("""
                            CREATE TABLE points(
                            point_id INTEGER PRIMARY KEY AUTOINCREMENT,
                            point_num INTEGER NOT NULL,
                            was_valid BOOLEAN,
                            attempt_made BOOLEAN NOT NULL,
                            experiment_id INTEGER NOT NULL,
                            FOREIGN KEY(experiment_id) REFERENCES experiments(experiment_id));
                            """)

    def get_experiment_with_name(self, experiment_name):
        experiments = self.cur.execute("SELECT * FROM experiments WHERE experiment_name='" + experiment_name + "';")

        return experiments.fetchone()

    def get_all_experiments(self):
        return self.cur.execute("SELECT * FROM experiments;").fetchall()

    def set_current_experiment(self, experiment_name):
        self.current_experiment = self.get_experiment_with_name(experiment_name)

    def create_new_experiment(self, experiment_name, obj_pos, obj_radius, camera_dist, rings, sectors):

        obj_pos_str = " ".join(str(coord) for coord in obj_pos)

        experiment = (experiment_name, obj_pos_str, str(obj_radius),
                      str(camera_dist), str(rings), str(sectors))

        self.cur.executemany("""INSERT INTO experiments (experiment_name, obj_pos, obj_radius, 
                                camera_dist_from_obj_origin, rings, sectors) VALUES (?,?,?,?,?,?)""", [experiment])

        self.conn.commit()

        self.set_current_experiment(experiment_name)

        self.create_points_and_add_to_experiment(rings * sectors)

        return True

    def remove_experiment_with_name(self, experiment_name):
        experiment = self.get_experiment_with_name(experiment_name)

        if experiment is None:
            return False

        self.cur.execute("DELETE FROM experiments WHERE experiment_name='" + experiment_name + "';")
        self.cur.execute("DELETE FROM points WHERE experiment_id='" + str(experiment[0]) + "';")

        self.conn.commit()

        return True

    def create_points_and_add_to_experiment(self, num_points):
        points = [(i, False, False, self.current_experiment[0]) for i in range(num_points)]

        self.cur.executemany("""INSERT INTO points (point_num, was_valid, attempt_made, 
                                experiment_id) VALUES (?,?,?,?)""", points)

        self.conn.commit()

    def get_point_with_num(self, point_num):
        point = self.cur.execute("SELECT * FROM points WHERE experiment_id='" + str(self.current_experiment[0]) +
                                 "' AND point_num='" + str(point_num) + "';")

        return point.fetchone()

    def get_points_for_experiment(self):
        experiment = self.cur.execute(
            "SELECT * FROM points WHERE experiment_id='" + str(self.current_experiment[0]) + "';")

        return experiment.fetchall()

    def update_point_status(self, point_num, is_valid):

        is_valid = "1" if is_valid else "0"

        self.cur.execute("UPDATE points SET was_valid ='" + is_valid +
                         "', attempt_made='1' WHERE experiment_id='" + str(self.current_experiment[0]) +
                         "' AND point_num='" + str(point_num) + "';")

        self.conn.commit()

    def get_experiment_statistics(self):

        statistics = {"experiments": {}, "points": {}}

        experiments = self.get_all_experiments()

        for experiment in experiments:
            self.set_current_experiment(experiment[1])

            points = self.get_points_for_experiment()

            statistics["experiments"][experiment[1]] = {"experiment_data": experiment,
                                                        "points": points,
                                                        "num_attempted": len([i for i in points if i[3] == 1]),
                                                        "num_successful": len([i for i in points if i[2] == 1])}

            for point in points:
                if point[3] == 1:

                    if str(point[1]) not in statistics["points"].keys():
                        statistics["points"][str(point[1])] = {"total": 0, "successful": 0, "unsuccessful": 0}

                    statistics["points"][str(point[1])]["total"] += 1

                    if point[2] == 1:
                        statistics["points"][str(point[1])]["successful"] += 1
                    else:
                        statistics["points"][str(point[1])]["unsuccessful"] += 1

        return statistics

    def close_database(self):
        self.conn.close()