import serial.tools.list_ports as Serial

ports = Serial.comports()

for port in ports:
    print("port name " + port.name + " manufactor " + port.manufacturer + " location " + port.location + " device " + port.device + " prodoct " + port.product + " desction " + port.hwid)