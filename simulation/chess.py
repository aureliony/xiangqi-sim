import argparse
import asyncio
import time

import pybullet

from engine.pikafish import Pikafish
from simulation.robotiq import SimulationEnv
from simulation.robotiq_rrt import SimulationEnvRRT


def get_user_move_input(is_red_turn: bool):
    valid_x = 'abcdefghi'
    valid_y = '0123456789'
    while True:
        if is_red_turn:
            print("(Red) It's your move!")
            example_move = 'b2e2'
        else:
            print("(Black) It's your move!")
            example_move = 'b7e7'

        print(f"Enter your move (e.g. {example_move}), or exit to stop the game.")
        print(">>> ", end='', flush=True)
        user_input = input().strip()
        if user_input == 'exit':
            exit()

        if (
            len(user_input) == 4
            and user_input[:2] != user_input[2:]
            and user_input[0] in valid_x and user_input[2] in valid_x
            and user_input[1] in valid_y and user_input[3] in valid_y
        ):
            return user_input
        print("Invalid input. It must be a 4-character string with valid board coordinates.")


async def main_loop(num_humans, depth):
    process = await asyncio.create_subprocess_exec(
        "engine/pikafish.exe",
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )

    engine = Pikafish(process)
    env = SimulationEnv(engine)
    # env = SimulationEnvRRT(engine)
    env.reset()

    is_red_turn = True
    is_game_over = False
    while True:
        if is_game_over:
            time.sleep(1)
            continue

        move = None
        if num_humans == 2 or (num_humans == 1 and is_red_turn):
            move = get_user_move_input(is_red_turn)
        else:
            print("Pikafish is thinking...")
        move_outcome = await env.make_move(
            is_red_turn=is_red_turn,
            move=move,
            print_evals=(num_humans == 0),
            depth=depth
        )

        if move_outcome is True:
            is_red_turn = not is_red_turn
        elif move_outcome is False:
            print("Invalid move.")
        else:
            is_game_over = True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--humans",
        type=int,
        default=0,
        choices=[0, 1, 2],
        help="Number of human players in the game."
    )
    parser.add_argument(
        "--level",
        type=int,
        default=8,
        choices=list(range(1, 20+1)),
        help="Difficulty level of Pikafish."
    )

    # Parse the arguments
    args = parser.parse_args()
    num_humans = args.humans
    depth = args.level

    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 1)
    pybullet.setGravity(0, 0, -9.81)
    pybullet.setRealTimeSimulation(1)
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=0,
        cameraPitch=-80,
        cameraTargetPosition=[0.08, -0.05, 0.6]
    )

    asyncio.run(main_loop(num_humans, depth))
