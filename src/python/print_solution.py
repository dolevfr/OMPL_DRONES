#!/usr/bin/env python3
import sys

def print_formatted_line(state):
    # Print payload info
    print("payload SE(3):", " ".join(f"{v:.3f}" for v in state[0:7]))
    print("payload velocities:", " ".join(f"{v:.3f}" for v in state[7:13]))
    print()
    
    # For 4 drones (each drone block is 11 numbers)
    for i in range(4):
        base = 13 + i * 11
        drone_so3 = state[base:base+4]
        drone_vel = state[base+4:base+7]
        cable = state[base+7:base+11]  # cable: 2 angles + 2 derivatives
        print(f"drone {i+1}: SO(3): {' '.join(f'{v:.3f}' for v in drone_so3)} | "
              f"velocities: {' '.join(f'{v:.3f}' for v in drone_vel)} | "
              f"cable: {' '.join(f'{v:.3f}' for v in cable)}")
    print()
    
    # Extra numbers: first 16 are inputs (4 for each drone), last one is control duration.
    inputs = []
    for i in range(4):
        base = 57 + i * 4  # 13 + 4*11 = 57
        inp = state[base:base+4]
        inputs.append(f"drone{i+1}: {' '.join(f'{v:.3f}' for v in inp)}")
    print("Inputs by drones:", " | ".join(inputs))
    print("Control duration:", f"{state[73]:.3f}")
    print("=" * 60)

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
