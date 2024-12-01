import asyncio
import time


class Pikafish:
    def __init__(self, engine_path="engine/pikafish.exe"):
        self.engine_path = engine_path
        self.process = None

    async def send_command(self, command):
        """Send a command to the chess engine asynchronously."""
        # print(f">> {command}")
        self.process.stdin.write((command + "\n").encode())
        await self.process.stdin.drain()  # Ensure the command is flushed to the engine

    async def read_output(self, timeout=2.0):
        """Read all lines of output from the chess engine."""
        output = []

        while True:
            try:
                # Wait for a line of output with a timeout
                line = await asyncio.wait_for(self.process.stdout.readline(), timeout=timeout)
                line = line.decode()
                line = line.strip()
                if line:
                    # print(line, flush=True)
                    output.append(line)

            except asyncio.TimeoutError:
                # If no line is received within the timeout, stop the engine and get the current best move
                # TODO: Fix this
                # await self.send_command("stop")
                # task = asyncio.create_task(self.process.stdout.readline())
                # time.sleep(0.5)
                # print(task.result())
                # assert 0
                break

        return output

    async def get_best_move(self, fen: str = None, depth: int = 10, print_evals=False) -> str:
        """
        go depth 1 -> "bestmove m"
        position startpos moves m1 m2...
        """
        # Start the engine asynchronously
        self.process = await asyncio.create_subprocess_exec(
            self.engine_path,
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
            text=False
        )
        command = f"position fen {fen}" if fen is not None else "position startpos"
        await self.send_command(command)

        command = "eval"
        await self.send_command(command)
        output = await self.read_output()
        if print_evals:
            for line in output[3:-23]:
                print(line, flush=True)

        command = f"go depth {depth}"
        await self.send_command(command)
        output = await self.read_output()
        bestmove = output[-1].split(' ')[1]

        command = "quit"
        await self.send_command(command)
        await self.process.wait()
        
        return bestmove


if __name__ == '__main__':
    engine = Pikafish()
    bestmove = asyncio.run(engine.get_best_move())
    print("bestmove:", bestmove, flush=True)
