import requests

esp32_ip = "192.168.100.29"  
url = f"http://{esp32_ip}/"

message_value = 5

data = {"Joint": str(message_value), "Angle": str(message_value)}
response = requests.post(url, data=data)

print(response.text)