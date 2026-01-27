import numpy as np
import requests


API_ORB = "http://192.168.1.183:8005/color"
task_id = 'whatever'

data_request = {
    "task_id": task_id,
}
response = requests.post(API_ORB, json=data_request)
print(response.json())


API_ORB = "http://192.168.1.183:8005/depth"
task_id = 'whatever'

data_request = {
    "task_id": task_id,
}
response = requests.post(API_ORB, json=data_request)
print(response.json())


API_PERCEP =  "http://192.168.1.183:2077"
data_request = {
        "task_id": task_id,
        "input": {
            'image_dir' : "http://192.168.1.206:9000/storage/whatever/original/color.png",
            'option': "general", 
            'prompts':'metal',
        },
        'setting': {
            'mask_url': True,
        }
        }
response = requests.post(API_PERCEP, json=data_request)
print(response.json())