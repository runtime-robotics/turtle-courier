import argparse
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import time

from field import POSES
from world import make_world


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument('--start', type=int, choices=POSES)
    cli.add_argument('--goal', type=int, choices=POSES)
    cli.add_argument('--seed', type=int)
    cli.add_argument('--yaw-offset', type=float, help='override the random starting heading, in degrees')
    cli.add_argument('--clear', action='store_true', help='leave the main road open')
    cli.add_argument('--gui', action='store_true')
    cli.add_argument('--result', type=Path, default=Path('/results/run.json'))
    args = cli.parse_args()
    rng = random.Random(args.seed)
    start = args.start if args.start is not None else rng.choice(
        [i for i in POSES if i != args.goal])
    goal = args.goal if args.goal is not None else rng.choice([i for i in POSES if i != start])
    if args.yaw_offset is not None and not math.isfinite(args.yaw_offset):
        cli.error('yaw offset must be finite')
    if start == goal:
        cli.error('start and goal must be different')
    if args.gui and not os.environ.get('DISPLAY'):
        cli.error('--gui requires DISPLAY and a mounted X11 socket')
    print(f'Scenario: start={start}, goal={goal}, main road blocked={not args.clear}', flush=True)
    path = args.result.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='turtle-courier-') as temp:
        root = Path(temp)
        worlds = root / 'worlds'
        worlds.mkdir()
        shutil.copyfile('worlds/CourierTurtle.proto', worlds / 'CourierTurtle.proto')
        (root / 'controllers').symlink_to('/app/controllers', target_is_directory=True)
        offset = rng.uniform(-180., 180.) if args.yaw_offset is None else args.yaw_offset
        world = make_world(worlds, start, math.radians(offset), not args.clear)
        result = root / 'result.json'
        env = dict(os.environ, COURSE_START=str(start), COURSE_GOAL=str(goal),
                   COURSE_GUI=str(int(args.gui)), COURSE_BLOCKED=str(int(not args.clear)),
                   COURSE_RESULT=str(result))
        nav = subprocess.Popen([sys.executable, '/app/navigate.py', '--goal', str(goal)])
        command = ['webots', '--batch', '--stdout', '--stderr', str(world)]
        if args.gui:
            command.append('--mode=realtime')
        else:
            command = ['xvfb-run', '-a', *command, '--no-rendering', '--minimize', '--mode=fast']
        try:
            with subprocess.Popen(command, env=env) as sim:
                recorded = False
                deadline = time.monotonic() + 150
                try:
                    while sim.poll() is None:
                        if not args.gui and time.monotonic() > deadline:
                            raise RuntimeError('Webots or ROS stopped progressing')
                        if nav.poll() is not None:
                            raise RuntimeError('ROS navigator exited early')
                        if result.is_file() and not recorded:
                            shutil.copyfile(result, path)
                            recorded = True
                        try:
                            sim.wait(timeout=.2)
                        except subprocess.TimeoutExpired:
                            continue
                finally:
                    if sim.poll() is None:
                        sim.terminate()
            if result.is_file():
                shutil.copyfile(result, path)
            return 0 if result.is_file() and json.loads(result.read_text())['passed'] else 2
        finally:
            nav.terminate()
            try:
                nav.wait(timeout=5)
            except subprocess.TimeoutExpired:
                nav.kill()
                nav.wait()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
