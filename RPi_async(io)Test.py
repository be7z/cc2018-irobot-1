import asyncio
import time

# global var
i = 0
    
#@asyncio.coroutine
async def every1s():
    global i
    while True:
        await asyncio.sleep(1)
        i += 1
        #print()

#@asyncio.coroutine
async def every4s():
    while True:
        global i
        print(i)
        await asyncio.sleep(4)
    
    
#---------- ASYNC ----------#
loop = asyncio.get_event_loop()
tasks = [
    asyncio.async(every1s()),
    asyncio.async(every4s())
]

try:
    loop.run_until_complete(asyncio.gather(*tasks))
except KeyboardInterrupt:
    loop.close()
