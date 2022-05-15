import shlex
import time
import os
import subprocess
import io
import sys

#name = sys.argv[1]


def run(args):
    p = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            universal_newlines=True, bufsize=-1)
    return p

def process_output(process):
    list = []
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            # print(output.strip())
            list.append(output.strip())
    rc = process.poll()
    return rc, list

def main():
    root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/decrease_step_length/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/decrease/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/final_results/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/widowX_radii_runs/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/aep_shift/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/step_length/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/original_with_pull_at_back/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/walknet_curve-walking_final_results/original/"
    # root = "/home/jsimmering/masterthesis/catkin_ws/logs/velocity_test/"
    root_dirs = os.listdir(root)
    print("root dirs = " + str(root_dirs))
    dirs = []
    prefix = []
    for d in root_dirs:
        sub_d = os.listdir(root + d)
        for s in sub_d:
            dirs.append(root + d + "/" + s)
            prefix.append(d.replace('s', '') + " & " + s.replace('dir', '') + " & ")
    dirs = sorted(dirs)
    prefix = sorted(prefix)
    print("dirs = " + str(dirs))
    print("")

    print("Get durations: ")
    durations = []
    for d in dirs:
        p = run(["python3", "stance_duration_printer.py", "-dir", d])
        rc, duration = process_output(p)
        durations += duration
    print(durations)
    print("")

    print("Get length: ")
    lengths = []
    for d in dirs:
        length_dir = d + "/stability"
        p = run(["python3", "stance_length_printer.py", "-dir", length_dir])
        rc, length = process_output(p)
        lengths += length
    print(lengths)
    print("")

    print("Get velocities: ")
    for i in range(0, len(lengths)):
        duration_args = durations[i].split()
        d1 = duration_args[0]
        d2 = duration_args[1]
        d3 = duration_args[2]
        d4 = duration_args[3]
        d5 = duration_args[4]
        d6 = duration_args[5]
        length_args = lengths[i].split()
        l1 = length_args[0]
        l2 = length_args[1]
        l3 = length_args[2]
        l4 = length_args[3]
        l5 = length_args[4]
        l6 = length_args[5]
        p = run(["python3", "velocity.py", "-d", d1, d2, d3, d4, d5, d6, "-l", l1, l2, l3, l4, l5, l6])
        stdout = p.communicate()[0]
        print(prefix[i] + '{} \\\\'.format(stdout))
    print("")
    # print("#############################")
    # print("musik spielen")
    # p = run(["bin/" + name, "mp3"])
    # time.sleep(2)
    # print("song")
    # p.stdin.write("song\n")
    # p.stdin.flush()
    # time.sleep(2)
    # print("---start-----playlist----")
    # p.stdin.write("playlist\n")
    # p.stdin.flush()
    # time.sleep(2)
    # print("---end-----playlist----")


if __name__ == "__main__":
    main()
