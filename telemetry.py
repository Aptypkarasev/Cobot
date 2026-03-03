import urx

rob = urx.Robot("192.168.8.3")

print(rob.get_realtime_monitor().get_all_data())
