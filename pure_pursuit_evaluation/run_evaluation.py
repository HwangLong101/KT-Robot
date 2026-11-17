"""Reproduce the fixed experiment matrix and create comparison artifacts."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform

ROOT = Path(__file__).resolve().parent
LOOKAHEADS = (0.30, 0.40, 0.55, 0.80, 1.00)
METHODS = ('baseline', 'curvature')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gif', action='store_true', help='Also export corners comparison GIF')
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle
    import numpy as np
    import PIL
    from scenarios import scenario, NAMES
    from tracking import Settings, COLUMNS, simulate
    from model import Robot

    out = ROOT / 'outputs'
    out.mkdir(exist_ok=True)
    cache = {}
    rows = []

    def run(name, method, lookahead=.55, dt=.05):
        key = (name, method, lookahead, dt)
        if key in cache:
            return cache[key]
        points, initial = scenario(name)
        data, metrics = simulate(points, initial, settings=Settings(speed_policy=method, lookahead=lookahead, dt=dt))
        run_id = f'{name}_{method}_L{lookahead:.2f}_dt{dt:.3f}'
        folder = out / 'runs' / run_id
        folder.mkdir(parents=True, exist_ok=True)
        metrics.update(scenario=name, method=method, lookahead_m=lookahead, dt_s=dt,
                       run_id=run_id, initial_pose=list(initial))
        (folder / 'summary.json').write_text(json.dumps(metrics, indent=2), encoding='utf-8')
        np.savetxt(folder / 'trajectory.csv', data, delimiter=',', header=','.join(COLUMNS), comments='')
        np.savetxt(folder / 'reference.csv', points, delimiter=',', header='x_m,y_m', comments='')
        rows.append(metrics)
        cache[key] = data, metrics
        return data, metrics

    # Predeclared parameters: no hidden best-run selection or stochastic trials.
    for name in NAMES:
        for lookahead in LOOKAHEADS:
            for method in METHODS:
                run(name, method, lookahead)
    for name in NAMES:
        for method in METHODS:
            for dt in (.025, .1):
                run(name, method, .55, dt)

    fields = ['scenario', 'method', 'lookahead_m', 'dt_s', 'reached_goal', 'time_s',
              'final_goal_error_m', 'path_rmse_m', 'distance_weighted_rmse_m', 'path_max_error_m',
              'max_lateral_demand_m_s2', 'max_lateral_demand_after_1m_m_s2',
              'max_abs_wheel_speed_rad_s', 'max_abs_yaw_rate_rad_s', 'run_id']

    def write_csv(filename, selected):
        with (out / filename).open('w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(selected)

    write_csv('all_trials.csv', rows)
    comparison_rows = [run(n, m)[1] for n in NAMES for m in METHODS]
    write_csv('comparison.csv', comparison_rows)
    write_csv('lookahead_sweep.csv', [r for r in rows if r['dt_s'] == .05])
    write_csv('timestep_check.csv', [r for r in rows if r['lookahead_m'] == .55])
    manifest = {
        'date': '2026-11-17',
        'matrix': {'paths': list(NAMES), 'methods': list(METHODS), 'lookahead_m': list(LOOKAHEADS),
                   'main_dt_s': .05, 'additional_dt_at_L055_s': [.025, .1],
                   'unique_trials': len(rows), 'lateral_limit_m_s2': .12},
        'environment': {'python': platform.python_version(), 'platform': platform.platform(),
                        'numpy': np.__version__, 'matplotlib': matplotlib.__version__, 'pillow': PIL.__version__},
        'sha256_code': {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in
                        ['model.py','tracking.py','scenarios.py','run_robot.py','run_evaluation.py']},
        'sources': ['https://www.roboticsbook.org/S52_diffdrive_actions.html',
                    'https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm',
                    'https://arxiv.org/abs/2305.20026'],
        'all_trials_reached_goal': all(r['reached_goal'] for r in rows),
        'notes': ['Deterministic synthetic kinematics; no random seeds or statistical confidence intervals',
                  'v*omega is a kinematic lateral acceleration demand; no friction or motor dynamics model',
                  'The speed law is an educational extension, not a full implementation of Nav2 RPP']}
    (out / 'experiment_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    colors = {'baseline':'#2563a6','curvature':'#e18423'}

    for name in NAMES:
        points, _ = scenario(name)
        fig, axes = plt.subplots(2, 3, figsize=(15, 8), layout='constrained')
        axes[0,0].plot(*points.T, 'k--', label='Reference')
        for method in METHODS:
            d, m = run(name, method)
            color = colors[method]
            axes[0,0].plot(d[:,1],d[:,2],color=color,label=method)
            axes[0,1].plot(d[:,0],d[:,8],color=color,label=method)
            axes[0,2].plot(d[:,10],d[:,8],color=color,label=method)
            axes[1,0].plot(d[:,0],d[:,4],color=color,label=method)
            axes[1,1].plot(d[:,0],np.abs(d[:,4]*d[:,5]),color=color,label=method)
            axes[1,2].plot(d[:,0],np.max(np.abs(d[:,6:8]),axis=1),color=color,label=method)
        axes[0,0].set(xlabel='x [m]',ylabel='y [m]',title=f'Path comparison: {name}')
        axes[0,0].axis('equal')
        axes[0,1].set(xlabel='Time [s]',ylabel='Error [m]',title='Error vs time')
        axes[0,2].set(xlabel='Forward path progress [m]',ylabel='Error [m]',title='Error vs progress')
        axes[1,0].set(xlabel='Time [s]',ylabel='v [m/s]',title='Applied forward speed')
        axes[1,1].set(xlabel='Time [s]',ylabel='|v * omega| [m/s^2]',title='Kinematic lateral demand')
        axes[1,1].axhline(.12,color='gray',ls='--',label='Design bound 0.12')
        axes[1,2].set(xlabel='Time [s]',ylabel='Max wheel magnitude [rad/s]',title='Applied wheel speed')
        axes[1,2].axhline(Robot().wheel_limit,color='gray',ls='--')
        for ax in axes.flat:
            ax.grid(alpha=.25)
            ax.legend(fontsize=8)
        fig.savefig(out / f'comparison_{name}.png',dpi=160)
        plt.close(fig)

    fig, axes = plt.subplots(3,3,figsize=(13,11),layout='constrained')
    for i,name in enumerate(NAMES):
        for method in METHODS:
            selected = [run(name,method,L)[1] for L in LOOKAHEADS]
            for j,metric in enumerate(('distance_weighted_rmse_m','time_s','max_lateral_demand_m_s2')):
                axes[i,j].plot(LOOKAHEADS,[r[metric] for r in selected],'-o',label=method,color=colors[method],ms=4)
        for j,label in enumerate(('Distance-weighted RMSE [m]','Time [s]','Max lateral demand [m/s^2]')):
            axes[i,j].set(xlabel='Look-ahead [m]',ylabel=label,title=name)
            axes[i,j].grid(alpha=.25)
            axes[i,j].legend(fontsize=8)
    fig.savefig(out/'lookahead_sweep.png',dpi=160)
    plt.close(fig)

    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for i,name in enumerate(NAMES):
        for method in METHODS:
            ds=(.025,.05,.1)
            axes[i].plot(ds,[run(name,method,.55,dt)[1]['distance_weighted_rmse_m'] for dt in ds],
                         '-o',color=colors[method],label=method)
        axes[i].set(xlabel='Simulation step [s]',ylabel='Distance-weighted RMSE [m]',title=name)
        axes[i].grid(alpha=.25)
        axes[i].legend(fontsize=8)
    fig.savefig(out/'timestep_check.png',dpi=160)
    plt.close(fig)

    if args.gif:
        points,_=scenario('corners')
        trajectories={method:run('corners',method)[0] for method in METHODS}
        t_end=max(d[-1,0] for d in trajectories.values())
        times=np.linspace(0,t_end,100)
        fig,axes=plt.subplots(1,2,figsize=(10,5),layout='constrained')
        artists={}
        for ax,method in zip(axes,METHODS):
            d=trajectories[method]
            ax.plot(*points.T,'k--')
            line,=ax.plot([],[],color=colors[method])
            heading,=ax.plot([],[],color='black')
            body=Circle(d[0,1:3],.15,color=colors[method],alpha=.7)
            ax.add_patch(body)
            wheels=[ax.plot([],[],'k-',lw=4)[0] for _ in range(2)]
            status=ax.text(.02,.98,'',transform=ax.transAxes,va='top',fontsize=8)
            ax.set(xlim=(-.8,8.8),ylim=(-1,6.8),xlabel='x [m]',ylabel='y [m]',title=method)
            ax.set_aspect('equal',adjustable='box')
            ax.grid(alpha=.25)
            artists[method]=(line,heading,body,wheels,status)

        def draw(t):
            for method,d in trajectories.items():
                i=min(int(np.searchsorted(d[:,0],t,side='right')-1),len(d)-1)
                line,heading,body,wheels,status=artists[method]
                p=d[i,1:3];theta=d[i,3]
                h=np.array([np.cos(theta),np.sin(theta)])
                n=np.array([-h[1],h[0]])
                line.set_data(d[:i+1,1],d[:i+1,2])
                body.center=p
                heading.set_data([p[0],p[0]+.22*h[0]],[p[1],p[1]+.22*h[1]])
                for wheel,side in zip(wheels,(-1,1)):
                    q=p+side*.15*n
                    ends=np.stack((q-.07*h,q+.07*h))
                    wheel.set_data(ends[:,0],ends[:,1])
                status.set_text(f't={t:.1f} s | '+('GOAL' if i==len(d)-1 else f'v={d[i,4]:.2f} m/s')
                                +f'\ngoal error={d[i,9]:.3f} m')
        animation=FuncAnimation(fig,draw,frames=times,interval=80,repeat=False)
        animation.save(out/'comparison_corners.gif',writer=PillowWriter(fps=12))
        plt.close(fig)

    text = [
        '# Kết quả mô phỏng, đánh giá — bước 3', '',
        'Đây là bản ghi kết quả chạy chương trình, chưa phải báo cáo cuối kỳ hoàn chỉnh.', '',
        '## Thiết kế thử nghiệm', '',
        '42 cấu hình xác định: 3 tuyến × 2 phương pháp × 5 look-ahead tại dt=0,05 s (30 cấu hình), '
        'cộng 12 cấu hình kiểm tra bước thời gian dt=0,025 và 0,1 s tại look-ahead=0,55 m. '
        'Không có nhiễu, không lấy trung bình từ nhiều lần chạy ngẫu nhiên. Không có tập dữ liệu thực.', '',
        'So sánh chính giữ look-ahead=0,55 m, v yêu cầu=0,5 m/s, trạng thái đầu=(0; −0,6; 0), '
        'cùng mô hình robot, giới hạn bánh, căn hướng và điều kiện đích. '
        'Chỉ thay luật chọn vận tốc: baseline hoặc v=min(v_cơ_sở, sqrt(0,12/|kappa|)). '
        'Ngưỡng 0,12 m/s² là lựa chọn thiết kế cho mô phỏng, không phải thông số đo từ robot thật.', '',
        '## So sánh chính', '',
        '| Tuyến | Phương pháp | Thời gian [s] | Sai số cuối [m] | RMSE theo mẫu [m] | RMSE theo quãng đường [m] | Max |vω| [m/s²] |',
        '|---|---|---:|---:|---:|---:|---:|']
    # Escape the absolute-value bars so Markdown table columns stay intact.
    text[-2] = '| Tuyến | Phương pháp | Thời gian [s] | Sai số cuối [m] | RMSE theo mẫu [m] | RMSE theo quãng đường [m] | Max v·omega tuyệt đối [m/s²] |'
    for r in comparison_rows:
        text.append(f'| {r["scenario"]} | {r["method"]} | {r["time_s"]:.2f} | {r["final_goal_error_m"]:.4f} | '
                    f'{r["path_rmse_m"]:.4f} | {r["distance_weighted_rmse_m"]:.4f} | {r["max_lateral_demand_m_s2"]:.4f} |')
    text += ['', '## Diễn giải', '',
             'Cả hai phương pháp đạt đích trên ba tuyến. Bản giới hạn theo độ cong giảm mức yêu cầu gia tốc ngang '
             'động học, nhưng hoàn thành chậm hơn. RMSE lấy mẫu theo thời gian tăng vì robot đi chậm ở vùng sai lệch '
             'ban đầu và đoạn cua; RMSE theo quãng đường gần như giữ nguyên. Kết quả không chứng minh bám đường '
             'chính xác hơn khi chỉ giảm vận tốc trong mô hình động học lý tưởng.', '',
             'Sai lệch ban đầu 0,6 m khiến robot phải quay để hội tụ ngay cả trên đường thẳng. '
             'Do đó chỉ tiêu cực đại toàn tuyến bị ảnh hưởng mạnh bởi quá trình đầu. '
             'Cột max_lateral_demand_after_1m_m_s2 trong CSV tách thêm phần sau tiến độ 1 m; '
             'đây chỉ là mô tả bổ sung, không loại bỏ mẫu đầu khỏi RMSE chính.', '',
             'RMSE theo quãng đường dùng trọng số ds=|v|dt và trung bình e² hai đầu mỗi khoảng. '
             'Nó giúp tách ảnh hưởng robot đi chậm, nhưng không tính thời gian đứng quay tại chỗ. '
             'RMSE theo mẫu vẫn tính tất cả mẫu đều thời gian, kể cả sai lệch ban đầu và mẫu kết thúc.', '',
             '## Khảo sát look-ahead', '',
             'Các giá trị 0,30; 0,40; 0,55; 0,80; 1,00 m được công bố đầy đủ trong lookahead_sweep.csv và đồ thị. '
             'Không chọn một cấu hình khác nhau cho từng tuyến để tuyên bố cải tiến. '
             'Khảo sát cho thấy đánh đổi giữa mức cắt góc, sai số hội tụ, thời gian và yêu cầu quay. '
             'Kết quả phụ thuộc tuyến, chưa xác định một tham số tối ưu cho mọi môi trường.', '',
             '## Kiểm tra độ nhạy bước thời gian', '',
             '| Tuyến | Phương pháp | RMSE quãng đường dt=0,025 s [m] | dt=0,05 s [m] | dt=0,1 s [m] |',
             '|---|---|---:|---:|---:|']
    for name in NAMES:
        for method in METHODS:
            vals=[run(name,method,.55,dt)[1]['distance_weighted_rmse_m'] for dt in (.025,.05,.1)]
            text.append(f'| {name} | {method} | '+' | '.join(f'{v:.5f}' for v in vals)+' |')
    text += ['', 'Việc đổi dt đồng thời đổi chu kỳ cập nhật điều khiển; đây là kiểm tra độ nhạy rời rạc hóa, '
             'không phải chứng minh hội tụ toán học. Bảng đầy đủ ghi cả thời gian và tình trạng đạt đích.', '',
             '## Giới hạn kết luận', '',
             'Đại lượng |v·omega| là yêu cầu gia tốc ngang suy ra từ động học; không mô phỏng lực bám, trượt bánh, '
             'động lực học động cơ hay tải trọng. Không dùng kết quả để kết luận đã giảm trượt bánh hoặc tăng độ an toàn '
             'trên robot thực. Luật vận tốc là mở rộng đơn giản để học tập, không phải triển khai đầy đủ Nav2 Regulated Pure Pursuit.', '',
             '## Nguồn và tái lập', '',
             'Chạy `python run_evaluation.py --gif` từ thư mục dự án. Cấu hình, phiên bản thư viện và SHA-256 mã '
             'ở outputs/experiment_manifest.json; toàn bộ 42 lần chạy có reference.csv, trajectory.csv và summary.json '
             'trong outputs/runs. Hình do chương trình tạo từ dữ liệu mô phỏng thực sự đã chạy.', '',
             '[1] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,” CMU-RI-TR-92-01, Jan. 1992. '
             'https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm.', '',
             '[2] F. Dellaert and S. Hutchinson, “5.2. Motion Model for the Differential Drive Robot,” '
             'Introduction to Robotics and Perception. https://www.roboticsbook.org/S52_diffdrive_actions.html.', '',
             '[3] S. Macenski, S. Singh, F. Martin, and J. Gines, “Regulated Pure Pursuit for Robot Path Tracking,” '
             '2023, arXiv:2305.20026. https://arxiv.org/abs/2305.20026. '
             'Tham khảo ý tưởng điều chỉnh vận tốc; không khẳng định công thức của bản này trùng với thuật toán trong bài.', '',
             'Ngày truy cập nguồn: 17/11/2026. Mã và tài liệu được phát triển với hỗ trợ ChatGPT; cần tự kiểm tra và '
             'ghi nhận vai trò AI khi đưa vào báo cáo học phần.', '']
    (ROOT/'Ket_qua_danh_gia.md').write_text('\n'.join(text),encoding='utf-8')
    print(f'Completed {len(rows)} distinct trials; reached goal: {sum(r["reached_goal"] for r in rows)}/{len(rows)}')
    for r in comparison_rows:
        print(f'{r["scenario"]:9s} {r["method"]:9s}: time={r["time_s"]:.2f}s; '
              f'RMSE_distance={r["distance_weighted_rmse_m"]:.5f}m; demand_max={r["max_lateral_demand_m_s2"]:.4f}m/s^2')
    print(f'Outputs: {out}')
    return 0 if manifest['all_trials_reached_goal'] else 1


if __name__=='__main__':
    raise SystemExit(main())
