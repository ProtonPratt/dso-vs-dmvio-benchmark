#!/usr/bin/env python3
"""3D plot of a TUM-format trajectory with camera orientation frames.

Draws the camera path and, every N poses, the camera's x/y/z axes in
red/green/blue so rotation is visible as well as translation.

    python scripts/plot_trajectory_3d.py data/tumvi/magistrale5/dmvio_result.txt \
        --every 50 --axis-length 0.5 --save magistrale5_dmvio.png
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np

import trajectory_utils as tu


class CameraVisualizer:

    def __init__(self):
        self.fig = plt.figure(figsize=(15, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')

    def plot_camera_frame(self, position, R, scale):
        """Draw the three body axes of one pose."""
        axes = np.array([[scale, 0, 0], [0, scale, 0], [0, 0, scale]])
        transformed_axes = np.dot(R, axes.T).T + position

        colors = ['r', 'g', 'b']
        for i in range(3):
            self.ax.plot([position[0], transformed_axes[i, 0]],
                         [position[1], transformed_axes[i, 1]],
                         [position[2], transformed_axes[i, 2]],
                         color=colors[i], linewidth=2)

    def create_static_plot(self, filename, save_path=None, show_frames_interval=500, scale=0.05):
        _, positions, quaternions = tu.load_tum(filename)
        rotations = tu.quat_to_rot(quaternions)

        self.ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'k-', label='Camera Path')

        for i in range(0, len(positions), show_frames_interval):
            self.plot_camera_frame(positions[i], rotations[i], scale)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')

        self.set_equal_aspects()

        if save_path:
            plt.savefig(save_path, bbox_inches='tight')
            print(f"Plot saved to {save_path}")
        else:
            plt.show()

    def set_equal_aspects(self):
        """Give all three axes the same span so the path is not distorted."""
        limits = np.array([
            self.ax.get_xlim3d(),
            self.ax.get_ylim3d(),
            self.ax.get_zlim3d(),
        ])

        center = np.mean(limits, axis=1)
        radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

        self.ax.set_xlim3d([center[0] - radius, center[0] + radius])
        self.ax.set_ylim3d([center[1] - radius, center[1] + radius])
        self.ax.set_zlim3d([center[2] - radius, center[2] + radius])


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("trajectory", help="TUM-format file: t x y z qx qy qz qw")
    parser.add_argument("--every", type=int, default=50, help="draw an orientation frame every N poses")
    parser.add_argument("--axis-length", type=float, default=0.05,
                        help="length of the drawn axes, in trajectory units")
    parser.add_argument("--save", help="output image path (shows a window if omitted)")
    args = parser.parse_args()

    visualizer = CameraVisualizer()
    visualizer.create_static_plot(args.trajectory, save_path=args.save,
                                  show_frames_interval=args.every, scale=args.axis_length)


if __name__ == "__main__":
    main()
