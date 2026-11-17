"""Run upstream main() unchanged, then export its recorded states.

Only interactive plotting is switched off for batch export. A profiler reads
main's local variables at return; it does not replace the simulation loop.
"""
import argparse
import csv
import hashlib
import importlib.util
import json
import platform
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gif', action='store_true', help='Also export an animated GIF')
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    import numpy as np
    import scipy
    import PIL

    manifest = json.loads((ROOT / 'source_manifest.json').read_text())
    vendor = ROOT / 'vendor' / 'PythonRobotics'
    for name, expected in manifest['sha256'].items():
        actual = hashlib.sha256((vendor / name).read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f'Upstream file changed: {name}')
    source = vendor / 'PathTracking' / 'pure_pursuit' / 'pure_pursuit.py'
    spec = importlib.util.spec_from_file_location('upstream_pure_pursuit', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.show_animation = False
    captured = {}

    def capture(frame, event, arg):
        if event == 'return' and frame.f_code is module.main.__code__:
            captured.update(frame.f_locals)

    previous = sys.getprofile()
    try:
        sys.setprofile(capture)
        module.main()
    finally:
        sys.setprofile(previous)
    states = captured['states']
    course = np.column_stack((captured['cx'], captured['cy']))
    center = np.column_stack((states.x, states.y))
    yaw = np.asarray(states.yaw)
    rear = center - module.WB / 2 * np.column_stack((np.cos(yaw), np.sin(yaw)))

    def distance_to_polyline(points):
        a = course[:-1]
        d = np.diff(course, axis=0)
        w = points[:, None, :] - a[None, :, :]
        u = np.clip(np.sum(w * d, axis=2) / np.sum(d * d, axis=1), 0, 1)
        projection = a[None, :, :] + u[:, :, None] * d[None, :, :]
        return np.min(np.linalg.norm(points[:, None, :] - projection, axis=2), axis=1)

    center_error = distance_to_polyline(center)
    rear_error = distance_to_polyline(rear)
    out = ROOT / 'outputs'
    out.mkdir(exist_ok=True)
    with (out / 'trajectory.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s', 'center_x_m', 'center_y_m', 'yaw_rad', 'speed_m_s',
                         'rear_x_m', 'rear_y_m', 'center_path_error_m', 'rear_path_error_m'])
        writer.writerows(zip(states.t, states.x, states.y, states.yaw, states.v,
                             rear[:, 0], rear[:, 1], center_error, rear_error))
    np.savetxt(out / 'reference_path.csv', course, delimiter=',',
               header='x_m,y_m', comments='')
    summary = {
        'source_commit': manifest['commit'],
        'model': 'Upstream kinematic bicycle model (steering vehicle)',
        'run_mode': 'Unmodified upstream main; show_animation=False; data captured at return',
        'simulation_time_s': float(states.t[-1]),
        'samples_including_initial': len(states.t),
        'target_index_final': int(captured['target_ind']),
        'last_path_index': int(captured['lastIndex']),
        'termination': 'lookahead_target_reached_final_index' if captured['target_ind'] == captured['lastIndex'] else 'time_limit',
        'center_distance_to_goal_m': float(np.linalg.norm(center[-1] - course[-1])),
        'rear_distance_to_goal_m': float(np.linalg.norm(rear[-1] - course[-1])),
        'center_path_rmse_m': float(np.sqrt(np.mean(center_error ** 2))),
        'center_path_max_error_m': float(center_error.max()),
        'rear_path_rmse_m': float(np.sqrt(np.mean(rear_error ** 2))),
        'rear_path_max_error_m': float(rear_error.max()),
        'error_definition': 'Unsigned minimum Euclidean distance to reference polyline; all samples, including initial offset',
        'parameters': {'dt_s': module.dt, 'wheelbase_m': module.WB,
                       'lookahead_base_m': module.Lfc, 'lookahead_gain_s': module.k,
                       'speed_gain_per_s': module.Kp, 'target_speed_m_s': captured['target_speed'],
                       'max_steer_rad': module.MAX_STEER, 'time_limit_s': captured['T']},
        'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                        'numpy': np.__version__, 'scipy': scipy.__version__,
                        'matplotlib': matplotlib.__version__, 'pillow': PIL.__version__},
        'limitations': ['Not yet a differential drive robot',
                        'Original assertion is not proof of goal arrival',
                        'No physical vehicle or VS Code GUI was tested here']}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), layout='constrained')
    ax = axes[0]
    ax.plot(*course.T, 'r--', label='Reference path')
    ax.plot(*center.T, color='#2456a6', label='Vehicle center')
    ax.plot(*rear.T, color='#2d956a', alpha=.8, label='Rear reference point')
    ax.scatter(*center[0], marker='o', color='black', label='Initial center')
    ax.scatter(*course[-1], marker='*', s=90, color='red', label='Path endpoint')
    ax.set(xlabel='x [m]', ylabel='y [m]', title='Original Pure Pursuit baseline')
    ax.axis('equal')
    ax.legend(fontsize=8)
    axes[1].plot(states.t, np.asarray(states.v) * 3.6)
    axes[1].axhline(captured['target_speed'] * 3.6, color='gray', ls='--')
    axes[1].set(xlabel='Simulation time [s]', ylabel='Speed [km/h]', title='Upstream speed response')
    axes[2].plot(states.t, center_error, label='Center')
    axes[2].plot(states.t, rear_error, label='Rear reference point')
    axes[2].set(xlabel='Simulation time [s]', ylabel='Distance [m]', title='Distance to reference polyline')
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=.25)
    fig.savefig(out / 'baseline_results.png', dpi=180)
    plt.close(fig)
    if args.gif:
        fig, ax = plt.subplots(figsize=(7.2, 5.2), layout='constrained')
        lo = np.minimum(course.min(axis=0), center.min(axis=0)) - 4
        hi = np.maximum(course.max(axis=0), center.max(axis=0)) + 4
        indices = np.unique(np.linspace(0, len(states.t)-1, min(90, len(states.t))).astype(int))

        def draw(i):
            ax.clear()
            ax.plot(*course.T, 'r--', label='Reference')
            ax.plot(*center[:i+1].T, color='#2456a6', label='Vehicle center')
            # Infer the last applied steering from the recorded heading increment.
            steer = 0.0
            if i > 0 and abs(states.v[i-1]) > 1e-12:
                dyaw = (yaw[i] - yaw[i-1] + np.pi) % (2*np.pi) - np.pi
                steer = np.arctan(dyaw * module.WB / (states.v[i-1] * module.dt))
            # Offline rendering only; does not affect the original simulation.
            c, s = np.cos(yaw[i]), np.sin(yaw[i])
            rotation = np.array([[c, -s], [s, c]])
            body = np.array([[-module.LENGTH/2, -module.WIDTH/2],
                             [module.LENGTH/2, -module.WIDTH/2],
                             [module.LENGTH/2, module.WIDTH/2],
                             [-module.LENGTH/2, module.WIDTH/2],
                             [-module.LENGTH/2, -module.WIDTH/2]])
            body = body @ rotation.T + center[i]
            ax.plot(*body.T, color='#2456a6')
            for bx in (-module.WB/2, module.WB/2):
                for by in (-module.WIDTH/2, module.WIDTH/2):
                    wheel_center = center[i] + np.array([bx, by]) @ rotation.T
                    wheel_yaw = yaw[i] + (steer if bx > 0 else 0)
                    wheel_delta = module.WHEEL_LEN/2 * np.array([np.cos(wheel_yaw), np.sin(wheel_yaw)])
                    ax.plot([wheel_center[0]-wheel_delta[0], wheel_center[0]+wheel_delta[0]],
                            [wheel_center[1]-wheel_delta[1], wheel_center[1]+wheel_delta[1]], color='black', lw=3)
            ax.arrow(*center[i], c*module.LENGTH/3, s*module.LENGTH/3, head_width=.5, color='red')
            ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='x [m]', ylabel='y [m]',
                   title=f'Original steering vehicle | t={states.t[i]:.1f} s')
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.25)
            ax.legend(loc='lower left')

        animation = FuncAnimation(fig, draw, frames=indices, interval=80)
        animation.save(out / 'baseline_animation.gif', writer=PillowWriter(fps=12))
        plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(f'Outputs saved to: {out}')


if __name__ == '__main__':
    main()
