import requests
import time 

esp32_ip = "192.168.100.30"  
url = f"http://{esp32_ip}/"

data = {"Joint": str(1001), "Angle": str(0)}
response = requests.post(url, data=data)

print(response.text)

"""data = {"Joint": str(message_value), "Angle": str(message_value)}
response = requests.post(url, data=data)

print(response.text)
"""