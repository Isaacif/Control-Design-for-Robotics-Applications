import requests
import time 

esp32_ip = "192.168.100.30"  
url = f"http://{esp32_ip}/"

message_value = 4000
while(True):
    for elem in range(0, 4500, 500):
        data = {"Joint": str(1), "Angle": str(elem)}
        response = requests.post(url, data=data)

        print(response.text)
        time.sleep(1)


"""data = {"Joint": str(message_value), "Angle": str(message_value)}
response = requests.post(url, data=data)

print(response.text)
"""