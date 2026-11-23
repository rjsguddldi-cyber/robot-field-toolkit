"""
PHM 정비 리포트: health_runner CSV -> Markdown. 순수 파이썬, 통계만 (추세선 = 최소제곱 직선).

- 조건별 요약 (예: 바퀴 마모 단계)
- 언제부터: PHM 첫 주의·경고 vs 기존 알람(E105, E401) 첫 발생, 몇 사이클 앞섰나
- 어떤 지표가, 얼마나: 기준선 대비 이동평균 이탈
- 남은 사이클 추정: 최근 추세선이 기준(PHM 경고선, 해당하면 기존 알람 기준)에 닿을 때까지
- 권고 조치와 우선순위
- 채점(ground truth): 실제 도착 오차, 믿는 위치와 실제 위치 차이. 감지에는 쓰지 않은 값
"""

import statistics

from robot_field_toolkit.core.phm import HEALTH_INDICATORS
from robot_field_toolkit.core.phm_response import response_for

LEVEL_RANK = {'learning': 0, 'normal': 1, 'caution': 2, 'warning': 3}
STATUS_KO = {'learning': '학습 중', 'normal': '정상', 'caution': '주의', 'warning': '경고'}
# 지표 -> (기존 알람 코드, 기준값). 해당 지표가 이 값에 닿으면 기존 알람이 뜬다
EXISTING_LIMITS = {'arrive_dist_cm': ('E401', 10.0), 'amcl_xy_mean': ('E402', 0.35)}
EXISTING_CODES = ('E105', 'E401', 'E402', 'E403')
NAMES = {
    'rot_ratio': '① 회전 비율 IMU/odom', 'rot_resid_rms': '① 회전 잔차 RMS',
    'trans_resid_pct': '② 이동 잔차 %', 'track_ratio': '③ 구동 추종률',
    'amcl_xy_mean': '④ AMCL 위치 표준편차', 'arrive_dist_cm': '⑤ 도착 거리 오차 cm',
    'arrive_yaw_deg': '⑤ 도착 각도 오차 도', 'arm_time_total': '⑥ 팔 도달 시간 s',
    'lidar_valid_mean': '⑦ LiDAR 유효 %', 'scan_jitter_ms': '⑧ /scan 지터 ms',
}


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def trend(points):
    """[(x, y)] -> (기울기, 절편). 점이 2개 미만이거나 x가 모두 같으면 None."""
    if len(points) < 2:
        return None
    xs, ys = [p[0] for p in points], [p[1] for p in points]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    slope = sum((x - mx) * (y - my) for x, y in points) / sxx
    return slope, my - slope * mx


def cycles_to_limit(points, limit, upward):
    """
    추세선이 limit에 닿을 때까지 남은 사이클 수 (마지막 사이클 기준).

    upward=True면 위로 넘는 기준, False면 아래로. 이미 넘었으면 0,
    추세가 기준 반대쪽으로 가거나 없으면 None.
    """
    fit = trend(points)
    if fit is None:
        return None
    slope, icpt = fit
    now = slope * points[-1][0] + icpt
    if (upward and now >= limit) or (not upward and now <= limit):
        return 0
    if (upward and slope <= 0) or (not upward and slope >= 0):
        return None
    return (limit - now) / slope


def warn_limit(key, baseline, warning_d=3.0):
    """PHM 경고선 값 (나쁜 방향). 기준선이 없으면 None. both는 현재 쪽을 고르기 위해 둘 다."""
    side, floor = HEALTH_INDICATORS[key]
    m, sd = baseline['mean'].get(key), baseline['std'].get(key)
    if m is None:
        return None
    s = max(sd or 0.0, floor)
    if side == 'high':
        return (m + warning_d * s,)
    if side == 'low':
        return (m - warning_d * s,)
    return (m - warning_d * s, m + warning_d * s)


def first_index(rows, pred):
    for i, r in enumerate(rows):
        if pred(r):
            return i
    return None


def build_report(rows, baseline, title='PHM 정비 리포트', trend_window=8, warning_d=3.0,
                 load='empty'):
    """rows: CSV 줄(dict) 시간순, baseline: 그 하중 상태의 기준선 dict(mean, std)."""
    L = [f'# {title}', '',
         '> **Gazebo 시뮬레이션 기준** 결과입니다. 실제 로봇 수치가 아닙니다. '
         'ground truth(Gazebo 실제 위치)는 채점 표에만 쓰고 감지에는 쓰지 않았습니다.', '']
    judged = [r for r in rows if r.get('phm_status') not in (None, '', 'learning')]
    seq = {id(r): i + 1 for i, r in enumerate(rows)}   # 전체 사이클 번호 (1부터)

    # ---- 조건별 요약 ----
    conds = []
    for r in rows:
        c = r.get('condition') or '-'
        if c not in conds:
            conds.append(c)
    L += ['## 1. 조건별 요약', '',
          '| 조건 | 사이클 | 회전 비율 평균 | 이동 잔차 평균 % | 건강도 최저~최고 | PHM 최고 수준 '
          '| E105 | E401 | E402 | E403 |',
          '|---|---|---|---|---|---|---|---|---|---|']
    for c in conds:
        rs = [r for r in rows if (r.get('condition') or '-') == c]

        def mean(k):
            v = [num(r.get(k)) for r in rs if num(r.get(k)) is not None]
            return f'{statistics.fmean(v):.4g}' if v else '-'
        hs = [num(r.get('health')) for r in rs if num(r.get('health')) is not None]
        top = max((r.get('phm_status') or 'learning' for r in rs),
                  key=lambda s: LEVEL_RANK.get(s, 0))
        cnt = {code: sum(code in (r.get('alarms') or '') for r in rs) for code in EXISTING_CODES}
        h_txt = f'{min(hs):.0f}~{max(hs):.0f}' if hs else '-'
        L.append(f'| {c} | {len(rs)} | {mean("rot_ratio")} | {mean("trans_resid_pct")} '
                 f'| {h_txt} | {STATUS_KO.get(top, top)} | '
                 + ' | '.join(str(cnt[code]) for code in EXISTING_CODES) + ' |')

    # ---- 언제부터: PHM vs 기존 알람 ----
    L += ['', '## 2. 언제 잡았나: PHM 예지 경보 vs 기존 알람', '']
    events = {
        'PHM 주의': first_index(rows, lambda r: LEVEL_RANK.get(r.get('phm_status'), 0) >= 2),
        'PHM 경고': first_index(rows, lambda r: r.get('phm_status') == 'warning'),
        'E105 바퀴 헛돎': first_index(rows, lambda r: 'E105' in (r.get('alarms') or '')),
        'E401 도착 오차': first_index(rows, lambda r: 'E401' in (r.get('alarms') or '')),
    }
    L += ['| 이벤트 | 처음 발생 사이클 | 그때 조건 |', '|---|---|---|']
    for name, i in events.items():
        L.append(f'| {name} | {i + 1 if i is not None else "발생 안 함"} '
                 f'| {rows[i].get("condition", "-") if i is not None else "-"} |')
    phm_i = events['PHM 주의']
    old = [i for k, i in events.items() if k.startswith('E') and i is not None]
    L.append('')
    if phm_i is None:
        L.append('- PHM 예지 경보가 뜨지 않았습니다.')
    elif old and min(old) > phm_i:
        L.append(f'- **PHM이 기존 알람보다 {min(old) - phm_i}사이클 먼저** 잡았습니다.')
    elif old:
        L.append(f'- 기존 알람이 {min(old) + 1}번째 사이클에서 PHM({phm_i + 1}번째)보다 먼저 떴습니다. '
                 '갑작스러운 사고(충돌·헛돎)는 기존 알람이 맡는 영역입니다.')
    else:
        L.append(f'- **PHM은 {phm_i + 1}번째 사이클({rows[phm_i].get("condition")})에서 잡았고, '
                 f'기존 알람(E105·E401)은 끝({len(rows)}사이클, {rows[-1].get("condition")})까지 '
                 '한 번도 뜨지 않았습니다.** 기존 알람만 있었다면 이 열화는 보이지 않았습니다.')

    # ---- 어떤 지표가, 얼마나, 남은 사이클 ----
    L += ['', '## 3. 어떤 지표가, 얼마나 (마지막 사이클 기준)', '',
          f'추세선 = 최근 {trend_window}사이클 최소제곱 직선. 남은 사이클 = 추세선이 기준에 닿을 때까지.',
          '', '| 지표 | 기준선 평균 | 최근 값 | 이탈 d (이동평균) | 추세/사이클 '
          '| PHM 경고선까지 | 기존 알람 기준까지 |', '|---|---|---|---|---|---|---|']
    last = rows[-1] if rows else {}
    flagged = set((last.get('alarm_indicators') or '').split())
    rank = []
    for key in HEALTH_INDICATORS:
        pts = [(seq[id(r)], num(r.get(key))) for r in rows if num(r.get(key)) is not None]
        pts = pts[-trend_window:]
        m, sd = baseline['mean'].get(key), baseline['std'].get(key)
        if not pts or m is None:
            continue
        side, floor = HEALTH_INDICATORS[key]
        s_eff = max(sd or 0.0, floor)
        recent = [p[1] for p in pts[-5:]]
        z = (statistics.fmean(recent) - m) / s_eff
        d = abs(z) if side == 'both' else max(0.0, z if side == 'high' else -z)
        fit = trend(pts)
        lims = warn_limit(key, baseline, warning_d)
        up = side == 'high' or (side == 'both' and z >= 0)
        to_warn = None
        if lims:
            lim = lims[0] if len(lims) == 1 else (lims[1] if up else lims[0])
            to_warn = cycles_to_limit(pts, lim, up)
        ex = EXISTING_LIMITS.get(key)
        to_ex = cycles_to_limit(pts, ex[1], True) if ex else None
        rank.append((d, key))
        if d < 1.0 and key not in flagged:
            continue
        L.append(f'| {NAMES[key]}{" ⚠" if key in flagged else ""} | {m:.4g} | {recent[-1]:.4g} '
                 f'| {d:.1f} | {f"{fit[0]:+.3g}" if fit else "-"} '
                 f'| {fmt_cycles(to_warn)} '
                 f'| {(ex[0] + " " + fmt_cycles(to_ex)) if ex else "해당 알람 없음"} |')

    if not any(line.startswith('| ①') or line.startswith('| ②') or line.startswith('| ③')
               or line.startswith('| ④') or line.startswith('| ⑤') or line.startswith('| ⑥')
               or line.startswith('| ⑦') or line.startswith('| ⑧') for line in L):
        L.append('| (기준선에서 평소 흔들림 이상 벗어난 지표 없음) | | | | | | |')

    # ---- 권고 ----
    status = last.get('phm_status', 'normal')
    resp = response_for(status, load)
    worst = [k for _, k in sorted(rank, reverse=True)[:3]]
    L += ['', '## 4. 권고 조치와 우선순위', '',
          f'- 현재 수준: **{STATUS_KO.get(status, status)}** → 자동 대응: {resp.label() or "없음"}']
    prio = 1
    if status == 'warning':
        L.append(f'- {prio}순위: odom 재캘리브레이션 (바퀴 반지름·간격). ①②가 함께 벗어나면 바퀴 마모·공기압·'
                 '이물질 의심 → 바퀴 점검')
        prio += 1
        L.append(f'- {prio}순위: 재캘리브레이션 후 기준선 재설정 '
                 '(`ros2 service call /health_runner/reset_baseline std_srvs/srv/Trigger`)')
        prio += 1
    elif status == 'caution':
        L.append(f'- {prio}순위: 다음 정기 점검에서 {", ".join(NAMES[k] for k in worst)} 확인, 추세 감시')
        prio += 1
    L.append(f'- 이탈이 큰 지표 순: {", ".join(NAMES[k] for k in worst)}')

    # ---- 채점 ----
    L += ['', '## 5. 채점 (ground truth, 감지에는 안 씀)', '',
          '| 조건 | 실제 도착 오차 평균 cm | 실제 도착 오차 최대 cm | 믿는 위치↔실제 위치 평균 cm |',
          '|---|---|---|---|']
    for c in conds:
        rs = [r for r in rows if (r.get('condition') or '-') == c]
        a = [num(r.get('gt_arrive_dist_cm')) for r in rs
             if num(r.get('gt_arrive_dist_cm')) is not None]
        e = [num(r.get('gt_loc_err_cm')) for r in rs if num(r.get('gt_loc_err_cm')) is not None]
        L.append(f'| {c} | {statistics.fmean(a):.1f} | {max(a):.1f} | {statistics.fmean(e):.1f} |'
                 if a and e else f'| {c} | - | - | - |')
    L.append(f'\n판정한 사이클 {len(judged)}개 / 전체 {len(rows)}개.')
    return '\n'.join(L) + '\n'


def fmt_cycles(n):
    if n is None:
        return '추세 없음'
    if n == 0:
        return '이미 넘음'
    return f'약 {n:.0f}사이클'
