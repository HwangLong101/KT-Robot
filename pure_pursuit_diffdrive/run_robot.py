"""Run the differential-drive adaptation. Example: python run_robot.py --animate"""
import argparse
import csv
import json
import platform
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('straight', 's_curve', 'corners', 'all'), default='s_curve')
    parser.add_argument('--animate', action='store_true', help='Open an interactive animation window')
    parser.add_argument('--gif', action='store_true', help='Save an offline GIF animation')
    parser.add_argument('--lookahead', type=float, default=0.55, help='Look-ahead circle radius [m]')
    parser.add_argument('--speed', type=float, default=0.5, help='Nominal forward speed [m/s]')
    args = parser.parse_args()
    if args.animate and args.scenario == 'all':
        parser.error('Choose one scenario for --animate; use --scenario all --gif for batch output')
    import matplotlib
    if not args.animate:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle, Polygon
    import numpy as np
    import PIL
    from model import Robot
    from tracking import Settings, COLUMNS, simulate
    from scenarios import NAMES, scenario

    try:
        settings = Settings(lookahead=args.lookahead, speed=args.speed)
    except ValueError as exc:
        parser.error(str(exc))
    robot = Robot()
    names = NAMES if args.scenario == 'all' else (args.scenario,)
    summaries = []
    animations = []
    out = ROOT / 'outputs'
    out.mkdir(exist_ok=True)
    for name in names:
        points, initial_pose = scenario(name)
        data, metrics = simulate(points, initial_pose, robot, settings)
        metrics['scenario'] = name
        metrics['initial_pose'] = list(initial_pose)
        metrics['environment'] = {'python': platform.python_version(), 'numpy': np.__version__,
                                  'matplotlib': matplotlib.__version__, 'pillow': PIL.__version__}
        summaries.append(metrics)
        folder = out / name
        folder.mkdir(exist_ok=True)
        np.savetxt(folder / 'trajectory.csv', data, delimiter=',', header=','.join(COLUMNS), comments='')
        np.savetxt(folder / 'reference.csv', points, delimiter=',', header='x_m,y_m', comments='')
        (folder / 'summary.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
        ax = axes[0, 0]
        ax.plot(*points.T, 'r--', lw=1.7, label='Reference')
        ax.plot(data[:, 1], data[:, 2], color='#2563a6', label='Axle midpoint')
        ax.scatter(*initial_pose[:2], color='black', s=25, label='Start')
        ax.scatter(*points[-1], marker='*', s=100, color='red', label='Goal')
        ax.add_patch(Circle(points[-1], settings.goal_tolerance, fill=False, ec='green'))
        ax.set(xlabel='x [m]', ylabel='y [m]', title=f'Differential-drive robot: {name}')
        ax.axis('equal')
        ax.legend(fontsize=8)
        axes[0, 1].plot(data[:, 0], data[:, 8], label='Distance to reference')
        axes[0, 1].set(xlabel='Time [s]', ylabel='Distance [m]', title=f'Path error | RMSE={metrics["path_rmse_m"]:.3f} m')
        axes[0, 1].legend(fontsize=8)
        axes[1, 0].plot(data[:, 0], data[:, 4], label='v [m/s]')
        axes[1, 0].plot(data[:, 0], data[:, 5], label='omega [rad/s]')
        axes[1, 0].set(xlabel='Time [s]', ylabel='Body velocity (units in legend)', title='Applied commands')
        axes[1, 0].legend(fontsize=8)
        axes[1, 1].plot(data[:, 0], data[:, 6], label='Left wheel')
        axes[1, 1].plot(data[:, 0], data[:, 7], label='Right wheel')
        for bound in (-robot.wheel_limit, robot.wheel_limit):
            axes[1, 1].axhline(bound, color='gray', ls='--', lw=.8)
        axes[1, 1].set(xlabel='Time [s]', ylabel='Wheel speed [rad/s]', title='Wheel commands and limits')
        axes[1, 1].legend(fontsize=8)
        for ax in axes.flat:
            ax.grid(alpha=.25)
        fig.savefig(folder / 'results.png', dpi=160)
        plt.close(fig)

        if args.gif or args.animate:
            fig, ax = plt.subplots(figsize=(8, 5), layout='constrained')
            lo = np.minimum(points.min(axis=0), data[:, 1:3].min(axis=0)) - .8
            hi = np.maximum(points.max(axis=0), data[:, 1:3].max(axis=0)) + .8
            ax.plot(*points.T, 'r--', label='Reference')
            line, = ax.plot([], [], color='#2563a6', label='Axle midpoint')
            target, = ax.plot([], [], 'x', color='green', label='Look-ahead target')
            ax.scatter(*points[-1], marker='*', s=100, color='red')
            ax.add_patch(Circle(points[-1], settings.goal_tolerance, fill=False, ec='green'))
            circle = Circle(data[0, 1:3], settings.lookahead, fill=False, ec='green', alpha=.3)
            ax.add_patch(circle)
            body = Circle(data[0, 1:3], robot.track/2, color='#93c5fd', alpha=.7)
            ax.add_patch(body)
            heading, = ax.plot([], [], color='#173d72', lw=2)
            wheels = []
            for _ in range(2):
                patch = Polygon(np.zeros((4, 2)), closed=True, color='#111827')
                ax.add_patch(patch)
                wheels.append(patch)
            status = ax.text(.02, .98, '', transform=ax.transAxes, va='top', fontsize=9,
                             bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': 'none'})
            ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='x [m]', ylabel='y [m]',
                   title=f'Pure Pursuit | differential drive | {name}')
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.25)
            ax.legend(loc='lower right', fontsize=8)

            def draw(i, d=data, path_line=line, marker=target, disk=body,
                     look_circle=circle, heading_line=heading, patches=wheels, text=status):
                p = d[i, 1:3]
                theta = d[i, 3]
                c, s = np.cos(theta), np.sin(theta)
                rot = np.array([[c, -s], [s, c]])
                path_line.set_data(d[:i+1, 1], d[:i+1, 2])
                marker.set_data([d[i, 11]], [d[i, 12]])
                disk.center = p
                look_circle.center = p
                heading_line.set_data([p[0], p[0]+robot.track*.65*c], [p[1], p[1]+robot.track*.65*s])
                for patch, side in zip(patches, (1, -1)):
                    poly = np.array([[-.07, -.025], [.07, -.025], [.07, .025], [-.07, .025]])
                    poly[:, 1] += side*robot.track/2
                    patch.set_xy(poly @ rot.T + p)
                modes = ('Pure Pursuit', 'Align heading', 'Goal reached', 'Time limit')
                text.set_text(f't={d[i,0]:.2f} s | v={d[i,4]:.2f} m/s\n'
                              f'left={d[i,6]:.2f}, right={d[i,7]:.2f} rad/s\n'
                              f'goal error={d[i,9]:.3f} m | {modes[int(d[i,13])]}')
                return path_line, marker, disk, look_circle, heading_line, *patches, text

            if args.gif:
                frames = np.unique(np.linspace(0, len(data)-1, min(90, len(data))).astype(int))
                anim = FuncAnimation(fig, draw, frames=frames, interval=80, repeat=False)
                anim.save(folder / 'animation.gif', writer=PillowWriter(fps=12))
            if args.animate:
                anim = FuncAnimation(fig, draw, frames=range(len(data)), interval=settings.dt*1000, repeat=False)
                animations.append(anim)  # keep a live reference until plt.show returns
            else:
                plt.close(fig)
        print(f'{name}: reached={metrics["reached_goal"]}; time={metrics["time_s"]:.2f} s; '
              f'goal error={metrics["final_goal_error_m"]:.4f} m; RMSE={metrics["path_rmse_m"]:.4f} m')
    if args.scenario == 'all':
        with (out / 'verification_summary.csv').open('w', newline='', encoding='utf-8') as f:
            fields = ['scenario', 'reached_goal', 'time_s', 'final_goal_error_m', 'path_rmse_m',
                      'path_max_error_m', 'max_abs_wheel_speed_rad_s', 'max_abs_yaw_rate_rad_s']
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(summaries)
    print(f'Outputs: {out}')
    if args.animate:
        plt.show()
    if not all(m['reached_goal'] for m in summaries):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
