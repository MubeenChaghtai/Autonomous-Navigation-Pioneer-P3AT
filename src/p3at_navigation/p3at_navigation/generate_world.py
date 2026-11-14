import argparse
from pathlib import Path

from p3at_navigation.world import GridMap, gazebo_world_sdf, office_layout


def main():
    parser = argparse.ArgumentParser(description="Write the Gazebo world and the matching ROS occupancy map for the office layout.")
    parser.add_argument("--worlds", default="worlds")
    parser.add_argument("--maps", default="maps")
    parser.add_argument("--resolution", type=float, default=0.05)
    args = parser.parse_args()
    layout = office_layout()
    Path(args.worlds).mkdir(parents=True, exist_ok=True)
    (Path(args.worlds) / "office.sdf").write_text(gazebo_world_sdf(layout), encoding="utf-8")
    GridMap.from_layout(layout, args.resolution).save_ros_map(Path(args.maps) / "office")
    print(f"wrote {Path(args.worlds) / 'office.sdf'} and {Path(args.maps) / 'office.yaml'}")


if __name__ == "__main__":
    main()
