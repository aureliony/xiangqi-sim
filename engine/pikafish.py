import asyncio


class Pikafish:
    def __init__(self, process: asyncio.subprocess.Process):
        assert isinstance(process, asyncio.subprocess.Process)
        self.process = process

    async def send_command(self, command: str, timeout: float = 5.0):
        """Send a command to the chess engine asynchronously."""
        # print(f">> {command}")
        self.process.stdin.write((command + '\n').encode())
        await self.process.stdin.drain()

        if command.startswith('position'):
            return []

        lst = []
        while True:
            try:
                line = await asyncio.wait_for(self.process.stdout.readline(), timeout=timeout)
                line = line.decode().strip()
                if line:
                    lst.append(line)
                if line.startswith('bestmove') or line.startswith('Final evaluation'):
                    break
            except:
                break
        return lst

    async def get_best_move(self, fen: str = None, depth: int = 20, print_evals=False) -> str:
        """
        go depth 1 -> "bestmove m"
        position startpos moves m1 m2...
        """
        # Start the engine asynchronously
        command = f"position fen {fen}" if fen is not None else "position startpos"
        output = await self.send_command(command)

        if print_evals:
            command = "eval"
            output = await self.send_command(command)
            for line in output[2:-23]:
                print(line.removeprefix('NNUE network contributions (').removesuffix(')'))
            print(output[-1].removesuffix(' [with scaled NNUE, ...]'), flush=True)

        command = f"go depth {depth}"
        output = await self.send_command(command)
        bestmove = output[-1].split(' ')[1]

        return bestmove

    async def close(self):
        if self.process.stdin:
            self.process.stdin.close()
        await self.process.wait()


async def main():
    process = await asyncio.create_subprocess_exec(
        "engine/pikafish.exe",
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE
    )
    engine = Pikafish(process)
    bestmove = await engine.get_best_move()
    await engine.close()
    return bestmove

if __name__ == '__main__':
    bestmove = asyncio.run(main())
    print("bestmove:", bestmove, flush=True)
