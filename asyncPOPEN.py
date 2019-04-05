import asyncio


async def run_command(*args):
    # Create subprocess
    process = await asyncio.create_subprocess_exec(
        *args,
        # stdout must a pipe to be accessible as process.stdout
        stdout=asyncio.subprocess.PIPE)
    # Wait for the subprocess to finish
    stdout, stderr = await process.communicate()
    # Return stdout
    return stdout.decode().strip()


loop = asyncio.get_event_loop()
# Gather uname and date commands
commands = asyncio.gather(run_command('ls'))
# Run the commands
ls = loop.run_until_complete(commands)
# Print a report
loop.close()

print(ls)
