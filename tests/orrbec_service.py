from dotenv import load_dotenv
import uvicorn
from pyorbbecsdk import *
load_dotenv()

from robots_orchestra.driver.orrbec.orbbec_server_bolt import OrbbecGrabber

"""
Orbbec Gemini 336L, serial number: CP828410007K
Orbbec Femto Bolt, serial number: CL838420167
"""

def main():
    ctx = Context()
    ctx.set_logger_level(OBLogLevel.ERROR)
    device_list = ctx.query_devices()
    count = device_list.get_count()
    if count == 0:
        print("No device found")
        return
    device_0 = device_list.get_device_by_serial_number("CP828410007K")
    print(device_0.get_device_info())
    
    device_1 = device_list.get_device_by_serial_number("CL838420167")
    print(device_1.get_device_info())

    
    # orbbec_server = OrbbecGrabber()
    # app = orbbec_server.run()
    # uvicorn.run(app, host="0.0.0.0", port=8005)

if __name__ == "__main__":
    main()