import os
import time
hostname = "10.0.0.22"
#print(time.time())
#response = os.system("ping -c 1 " + hostname)
print(time.time())
response = os.system("ping -c 1 -W 2000 10.0.0.22")
print(time.time())
print(response)
