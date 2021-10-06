import struct
import serial
import numpy
import socket
import threading
import time
import zlib

server_id = 0x42fd9abc

reportThreads = {}
sock = None


def packMessage(body):
    out = bytearray()
    out += "DSUS".encode('utf-8')
    out += struct.pack("<HHII", 1001, len(body), 0, server_id)
    out += body

    crc = zlib.crc32(out)
    out[8:12] = struct.pack("<I", crc)
    return out

def commonSubHeader(i):
    macAddr = [0,0,0,0,0,0]
    return struct.pack("<bbbb6bb", i, 2, 2, 1, *macAddr, 1)

def controllerInfo(i):
    return struct.pack("<I", 0x100001) + commonSubHeader(i) + bytes(1)

def controllerData(i, A, G, pack_number, timestamp):
    return struct.pack("<I", 0x100002) + commonSubHeader(i) + struct.pack("<BI20BBBHHBBHHQ6f",1,pack_number, 0, 0, 0, 0, 128, 128, 128, 128, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0,0,0,0,0,0,0,0,0,timestamp, A[0,0],A[0,1],A[0,2],G[0,0],G[0,1],G[0,2])

def touchController(addr, slot=-1):
    if (slot == -1 and mac == bytes(6)):
        raise ValueError("Gotta register something")
    if (addr, slot) in reportThreads:
        reportThreads[(addr, slot)] = (1500, reportThreads[(addr, slot)][1])
    else:
        reportThreads[(addr, slot)] = (1500, 0)

def serverThread():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 26760))
    while(True):
        data, addr = sock.recvfrom(1500)

        print("pacote recebido")
        if (data[0:4] != "DSUC".encode('utf-8')):
            print("Magic word errada")
            continue
        version = struct.unpack("<H", data[4:6])[0]
        if (version > 0x1001):
            print("Versão acima da suportada(recebi {:X}, aceito so 0x1001)".format(version))
        lenght = struct.unpack("<H", data[6:8])[0]
        # dont give a shit about CRC, kernel would have dropped it
        clientId = struct.unpack("<I", data[12:16])[0]
        eventType = struct.unpack("<I", data[16:20])[0]
        print("Tipo de evento : {:X}".format(eventType))
        if (eventType == 0x100000):
            pass
        elif (eventType == 0x100001):
            numPorts = struct.unpack("<I", data[20:24])[0]
            for i in range(0, numPorts):
                if (i == 0):
                    sock.sendto(packMessage(controllerInfo(i)), addr)
        elif (eventType == 0x100002):
            actionBitmask = data[20]
            if (actionBitmask & 0x1):
                touchController(addr, slot=data[21])
            if (actionBitmask & 0x2):
                touchController(mac=data[22:28])

calibMat = numpy.zeros((1000,3))
calibrateGbias = True
calibIndex = 0

Gbias = numpy.array([0,0,0])

def serialRoutine():
    global calibrateGbias
    global Gbias
    global calibIndex
    ser = serial.Serial('/dev/ttyUSB0', 115200)

    sync_machine_state = 0
    while(sync_machine_state != 100):
        c = ser.read(1)
        print(c)
        print(sync_machine_state)
        if (sync_machine_state == 0):
            if (c == "b".encode('utf-8')):
                sync_machine_state = 1
        elif (sync_machine_state == 1):
            if (c == "o".encode('utf-8')):
                sync_machine_state = 2
            elif (c == "b".encode('utf-8')):
                sync_machine_state = 1
            else:
                sync_machine_state = 0
        elif (sync_machine_state == 2):
            if (c == "t".encode('utf-8')):
                sync_machine_state = 3
            elif (c == "b".encode('utf-8')):
                sync_machine_state = 1
            else:
                sync_machine_state = 0
        elif (sync_machine_state == 3):
            if (c == "w".encode('utf-8')):
                sync_machine_state = 4
            elif (c == "b".encode('utf-8')):
                sync_machine_state = 1
            else:
                sync_machine_state = 0
        elif (sync_machine_state == 4):
            if (c == "1".encode('utf-8')):
                sync_machine_state = 100
                ser.read(9*2)
            elif (c == "b".encode('utf-8')):
                sync_machine_state = 1
            else:
                sync_machine_state = 0

    testeSomatorio = numpy.zeros((1,3))
    while(True):
        if (ser.read(5) != "botw1".encode('utf-8')):
            raise ValueError("out of sync")
        data = struct.unpack("<9h", ser.read(9*2))
        Araw = numpy.array([data[0], data[1], data[2]], dtype="double")
        Graw = numpy.array([data[3], data[4], data[5]], dtype="double")
        Mraw = numpy.array([data[6], data[7], data[8]], dtype="double")
        print(data)
        print(hex(data[0] & 0xFFFFFF), hex(data[1] & 0xFFFFFF), hex(data[2] & 0xFFFFFF))

        tMatrix = numpy.matrix([[0.0,  1.0,  0.0],
                                [1.0,  0.0,  0.0],
                                [0.0,  0.0, -1.0]])

        #print("\033[4A'\r######")
        print(Araw)
        print(Graw)
        print(Mraw)
        G = 9.807
        accelScale = G * 8.0/32767.5
        Abias = numpy.array([0.1, 0.13,  -0.34])
        Ascale = numpy.array([1.0, 1.0,  0.99])


        gyroScale = 1000.0/32767.5

        Mscale = numpy.array([0.18, 0.18, 0.17])
        Mbias = numpy.array([-66.4, 3.82, -24.07])
        Muscale = numpy.array([0.6, 1.5, 1.49])

        A = numpy.multiply((Araw * tMatrix * accelScale - Abias), Ascale);
        A[0,0] = A[0,0] * -1
        G = Graw * tMatrix * gyroScale - Gbias;
        # tmp = G[0,1]
        # G[0,1] = G[0,0]
        # G[0,0] = tmp
        M = (Mraw * Mscale - Mbias) * Muscale
        testeSomatorio += G

        if (calibrateGbias == True):
            calibMat[calibIndex] = G[0]
            calibIndex += 1
            if calibIndex == 1000:
                Gbias = numpy.mean(calibMat, 0)
                calibrateGbias = False

        # print(A)
        # print(G)
        # print(testeSomatorio)
        # print(M)
        # print(numpy.linalg.norm(M))
        # print("-----")

        for key in list(reportThreads):
            sock.sendto(packMessage(controllerData(key[1], A, G, reportThreads[key][1], reportThreads[key][1] * 4000)), key[0])
            reportThreads[key] = (reportThreads[key][0] - 1, reportThreads[key][1] + 1)
            if reportThreads[key][0] < 0:
                del reportThreads[key]


serverThread = threading.Thread(target=serverThread)
serverThread.start()
while(True):
    serialRoutine()
    time.sleep(0.004)

  # _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
  # _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
  # _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
  # _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
  # _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
  # _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
  # _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
  # _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
  # _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
  # _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;


