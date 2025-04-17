#!/usr/bin/env python3
import sys

num_drones = 4

def print_formatted_line(state):
    # Print payload info
    # print("payload SE(3):", " ".join(f"{v:.3f}" for v in state[0:7]))
    # print("payload velocities:", " ".join(f"{v:.3f}" for v in state[7:13]))
    # print()
    
    # For 4 drones (each drone block is 11 numbers)
    for i in range(num_drones):
        base = 13 + i * 11
        drone_so3 = state[base:base+4]
        drone_vel = state[base+4:base+7]
        cable = state[base+7:base+11]  # cable: 2 angles + 2 derivatives
        # print(f"drone {i+1}: SO(3): {' '.join(f'{v:.3f}' for v in drone_so3)} | "
        #       f"velocities: {' '.join(f'{v:.3f}' for v in drone_vel)} | "
        #       f"cable: {' '.join(f'{v:.3f}' for v in cable)}")
    # print()
    
    # Compute start index for extra numbers:
    inputs_start = 13 + num_drones * 11  # payload (13 numbers) + each drone-cable (11 numbers)
    # Then, for each drone's input (4 numbers per drone):
    inputs = []
    for i in range(num_drones):
        base = inputs_start + i * 4
        inp = state[base:base+4]
        inputs.append(f"drone {i+1}: {' '.join(f'{v:.3f}' for v in inp)}")
    print("Inputs by drones:", " | ".join(inputs))

    # Control duration is the last extra number:
    control_duration = state[inputs_start + 4 * num_drones]
    # print("Control duration:", f"{control_duration:.3f}")
    # print("=" * 60)


def main():
    if len(sys.argv) != 2:
        print("Usage: python print_solution_formatted.py <solution_path.txt>")
        sys.exit(1)
    filename = sys.argv[1]
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            state = [float(x) for x in line.split()]
            print_formatted_line(state)

if __name__ == "__main__":
    main()
