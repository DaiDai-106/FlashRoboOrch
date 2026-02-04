from flask import Flask, request, Response
import jsonpickle
import numpy as np
import json
import os
from io import BytesIO


app = Flask(__name__)



@app.route("/", methods=["GET", "POST"])
def index():

    json_data = request.get_json()
    
    # print(path_info)
    
    # 保存 path_info 到 tianji_path.json
    save_path = '/home/daidai/FlashRoboOrch/src/robots_orchestra/controller/camera_cache/whatever/tianji_path.json'
    with open(save_path, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, ensure_ascii=False, indent=2)
    
    result_json = {'message':'finish grab get service'}
    response_pickled = jsonpickle.encode(result_json)
    return Response(response=response_pickled, status=200, mimetype="application/json")


if __name__ == "__main__":
    app.debug = True
    app.run(host='0.0.0.0', port=8888)