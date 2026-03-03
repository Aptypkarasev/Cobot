import socket
import time

ip = '192.168.8.3'

ports = [80, 443, 502, 8000, 8080, 9093, 5000, 6000, 1883, 5001]

print(f"Сканирование портов на {ip}...\n")
open_ports = []

for port in ports:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1)

    try:
        result = sock.connect_ex((ip, port))
        status = "ОТКРЫТ" if result == 0 else " закрыт"
        if result == 0:
            open_ports.append(port)
        print(f"Порт {port:5d}: {status}")
    except:
        print(f"Порт {port:5d}:️ ошибка")
    finally:
        sock.close()
    time.sleep(0.1)

print("\nОткрытые порты:", open_ports if open_ports else "не найдено")