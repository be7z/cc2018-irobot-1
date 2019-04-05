import asyncio
import time

i = 0
    
#@asyncio.coroutine
async def test1():
    global i
    while True:
        await asyncio.sleep(1)
        i += 1
        #print()

#@asyncio.coroutine
async def main():
    while True:
        global i
        print(i)
        await asyncio.sleep(4)
    
    
#---------- ASYNC ----------#
loop = asyncio.get_event_loop()
tasks = [
    asyncio.async(test1()),
    asyncio.async(main())
]

try:
    loop.run_until_complete(asyncio.gather(*tasks))
except KeyboardInterrupt:
    loop.close()
