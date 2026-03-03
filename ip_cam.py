import requests
import time
from requests.auth import HTTPDigestAuth

'''

rtsp://192.168.8.65:554/Streaming/channels/101

rtsp://admin:Technored@192.168.8.65:554/Streaming/channels/101




http://192.168.8.65:80/ISAPI/System/deviceinfo
'''

base = "http://192.168.8.65/"

url = "ISAPI/PTZCtrl/channels/1/continuous"


pan = 60
while True:
    xml = """
    <?xml version="1.0" encoding="UTF-8"?>
    <PTZData>""" +\
    f"<pan>{pan}</pan>" +\
    """<tilt>0</tilt>
    </PTZData>
    """
    pan = -pan
    r = requests.put(base+url, auth=HTTPDigestAuth("admin", "Technored"), data=xml)
    print(r.status_code)
    print(r.text)
    time.sleep(2)
