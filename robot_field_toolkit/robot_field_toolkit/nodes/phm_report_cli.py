"""
정비 리포트 만들기 (ROS 없이 실행 가능). health_runner CSV + 기준선 -> Markdown.

    ros2 run robot_field_toolkit phm_report ~/field_toolkit_reports/wear_A.csv
"""

import argparse
import csv
import json
import os
import sys

from robot_field_toolkit.core.phm_report import build_report


def main(args=None):
    ap = argparse.ArgumentParser(description='PHM 정비 리포트 (Markdown)')
    ap.add_argument('csv', help='health_runner CSV')
    ap.add_argument('--baseline', default='~/field_toolkit_reports/phm_baseline.json')
    ap.add_argument('--load', default='empty', help='하중 상태 (기준선 고르기)')
    ap.add_argument('--out', default='', help='저장 경로 (기본: CSV 이름 + _report.md)')
    ap.add_argument('--title', default='PHM 정비 리포트')
    opt = ap.parse_args(args if args is not None else sys.argv[1:])
    path = os.path.expanduser(opt.csv)
    with open(path, newline='', encoding='utf-8') as f:
        rows = [r for r in csv.DictReader(f) if r.get('load_state', opt.load) == opt.load]
    with open(os.path.expanduser(opt.baseline), encoding='utf-8') as f:
        base = json.load(f)[opt.load]
    out = opt.out or os.path.splitext(path)[0] + '_report.md'
    with open(out, 'w', encoding='utf-8') as f:
        f.write(build_report(rows, base, title=opt.title))
    print(f'리포트 저장: {out}')


if __name__ == '__main__':
    main()
