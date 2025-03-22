#!/usr/bin/env python3
import subprocess
import sys
import time

def run_command(cmd):
    """Launch a command via bash -c and return the Popen handle."""
    return subprocess.Popen(["bash", "-c", cmd])

def main():
    if len(sys.argv) < 2:
        print("Usage: {} <simulation_number>".format(sys.argv[0]))
        sys.exit(1)

    sim_num = sys.argv[1]

    # Build a command string to source the three setup files.
    env_source = (
        "source /home/kota/ego_swarm_ws/devel/setup.bash && "
        # "source /home/kota/livox_ros_ws/devel/setup.bash && "
        "source /home/kota/mid360_ws/devel/setup.bash"
    )

    # Define each command.
    roscore_cmd = f"{env_source} && roscore"
    start_world_cmd = f"{env_source} && roslaunch --wait acl_sim start_world.launch"
    perfect_tracker_cmd = f"{env_source} && roslaunch --wait acl_sim perfect_tracker_and_sim.launch x:=0.0 y:=0.0 z:=3.0 yaw:=0.0"
    rviz_cmd = f"{env_source} && roslaunch --wait ego_planner rviz.launch"
    single_run_cmd = f"{env_source} && roslaunch --wait ego_planner single_run_in_sim.launch simulation_number:={sim_num}"
    goal_pub_cmd = f"""{env_source} && sleep 10 && rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{{header: {{frame_id: "world"}}, pose: {{position: {{x: 105.0, y: 0.0, z: 3.0}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}}' -1"""
    bag_file = f"/home/kota/data/ego_swarm_num_{sim_num}.bag"
    rosbag_cmd = f"{env_source} && rosbag record -a -O {bag_file}"

    processes = []

    print("Launching roscore...")
    processes.append(run_command(roscore_cmd))
    time.sleep(5)  # Wait for roscore to initialize

    print("Launching start_world.launch...")
    processes.append(run_command(start_world_cmd))
    time.sleep(2)

    print("Launching perfect_tracker_and_sim.launch...")
    processes.append(run_command(perfect_tracker_cmd))
    time.sleep(2)

    print("Launching ego_planner rviz.launch...")
    processes.append(run_command(rviz_cmd))
    time.sleep(2)

    print("Launching single_run_in_sim.launch with simulation_number: {}".format(sim_num))
    processes.append(run_command(single_run_cmd))
    time.sleep(2)

    print("Launching goal publisher...")
    processes.append(run_command(goal_pub_cmd))
    time.sleep(2)

    print(f"Launching rosbag recording to {bag_file} ...")
    processes.append(run_command(rosbag_cmd))

    print("All processes launched. Press Ctrl+C to terminate.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Terminating all processes...")
        for p in processes:
            p.terminate()
        for p in processes:
            p.wait()
        print("Done.")

if __name__ == "__main__":
    main()
