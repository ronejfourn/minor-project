from tkinter import *
from tkinter.filedialog import askopenfilename
import serial, os, queue, time, threading
from sys import argv

if len(argv) == 1:
    print(f"USAGE: {argv[0]} <serial port>")
    exit(1)

port = argv[1]
ser = serial.Serial(port=port, baudrate=115200, timeout=0.1)
run = True
txqueue = queue.Queue()

smp = threading.Semaphore(0)

recving = False
sending = False
txfptr = None
txfn = ''
txfs = 0
txsn = 0
txtm = 0

rxfptr = None
rxfn = ''
rxfs = 0
rxsn = 0
rxtm = 0

lastsent = None
resendcount = 0
ackrecv = True

mtx = threading.Lock()

def make_crc_table():
    poly = 0x8408
    table = []
    for byte in range(256):
        crc = 0
        for bit in range(8):
            if (byte ^ crc) & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            byte >>= 1
        table.append(crc)
    return table

table = make_crc_table()

def crc_16_fast(msg):
    crc = 0xffff
    for byte in msg:
        crc = table[(byte ^ crc) & 0xff] ^ (crc >> 8)
    return crc ^ 0xffff

def txput(b):
    txqueue.put(b)
    smp.release()

def sendfile():
    fp = askopenfilename()
    if not fp:
        return
    b['state'] = 'disabled'
    fn = os.path.basename(fp)
    fs = os.path.getsize(fp)
    if fs > 0xffff:
        print("file size too large")
        b['state'] = 'normal'
        return
    if len(fn) > 0xff - 6:
        print("file name too long")
        b['state'] = 'normal'
        return

    global txfptr, resendcount
    txfptr = open(fp, 'rb')
    byts = bytes([0xfb, (fs >> 8) & 0xff, (fs & 0xff), len(fn)])
    byts += str.encode(fn)
    crc = crc_16_fast(byts[1:])
    crcbytes = bytes([(crc >> 8) & 0xff, crc & 0xff])
    txput(byts + crcbytes)
    resendcount = 0
    print(f"Sending file '{fp}' of size {fs}")
    global txfn, txfs, txsn
    txfn = fn
    txfs = fs
    txsn = 0

def sendack():
    txput(b'\xfa')

def sendarq():
    txput(b'\xff')

def sendterm():
    txput(b'\xf0')

iserror = False
rxlast = 0

def rxhandle(a, cs):
    global recving, iserror, ackrecv
    global rxfn, rxfs, rxsn, rxfptr, rxlast, rxtm

    if a == b"SENT\n":
        cs.set()
    elif a == b"RECV\n":
        pln = ser.read(1)[0]
        hdr = ser.read(1)[0]
        if hdr == 0xfb: # file transfer begin request
            msbb = ser.read(1)
            lsbb = ser.read(1)
            flnb = ser.read(1)

            msb = msbb[0]
            lsb = lsbb[0]
            fln = flnb[0]
            size = (msb << 8) | lsb
            
            nameb = ser.read(fln)
            name = nameb.decode()
            crcbytes = ser.read(2)
            crc = (crcbytes[0] << 8) | crcbytes[1]

            if not recving:
                comp = crc_16_fast(msbb + lsbb + flnb + nameb)
                if comp == crc:
                    print(f"Receiving file '{name}' of size {size} bytes")
                    recving = True
                    rxfn = name
                    rxfs = size
                    rxsn = 0
                    rxfptr = open(rxfn + ".recv", "wb")
                    rxlast = time.time() * 1000
                    rxtm = time.time()
                    sendack()
                else:
                    sendarq()
            elif name != rxfn or rxsn > 0:
                print(f"Unexpected 0xfb terminating receiving file '{name}'")
                recving = False
                rxfptr.close()
                sendterm()
            else:
                rxlast = time.time() * 1000
                rxtm = time.time()
                sendack()

        elif hdr == 0xfd: # file data
            if recving:
                b = ser.read(252)
                crcbytes = ser.read(2)
                crc = (crcbytes[0] << 8) | crcbytes[1]
                comp = crc_16_fast(b)

                if comp == crc:
                    rxfptr.write(b)
                    rxsn += 252
                    print(f"Receiving file '{rxfn}' ({round(rxsn/rxfs*100)}%)")
                    rxlast = time.time() * 1000
                    sendack()
                else:
                    sendarq()
            else:
                print(f"Unexpected 0xfd")
                sendterm()

        elif hdr == 0xfe: # file end
            if recving:
                lb = ser.read(1)
                l = lb[0]
                b = ser.read(l)
                crcbytes = ser.read(2)
                crc = (crcbytes[0] << 8) | crcbytes[1]
                comp = crc_16_fast(lb + b)

                if comp == crc:
                    recving = False
                    rxfptr.write(b)
                    rxfptr.close()
                    t = time.time()-rxtm
                    print(f"Finished receiving file '{rxfn}' took {round(t, 2)}s ({round(rxfs*8/t)}bps)")
                    sendack()
                else:
                    sendarq()
            else:
                print(f"Unexpected 0xfe")
                sendterm()

        elif hdr == 0xfa: # acknowledge
            iserror = False
            ackrecv = True

        elif hdr == 0xff: # failure
            iserror = True
            ackrecv = True

        elif hdr == 0xf0:
            global sending, shouldcheckack
            with mtx:
                shouldcheckack = False
                sending = False
                if txfptr:
                    txfptr.close()
                    txfptr = None
                cs.set()
            print("Terminating sending file '{txfn}'")

        else:
            ser.read(pln) # discard unknown

    elif a == b"BRKN\n":
        if recving:
            sendarq()
        else:
            sendterm()
    else:
        print(a)
        cs.set()

waitackbegin = 0
shouldcheckack = False

def beginwaitack():
    global resendcount, ackrecv, iserror, waitackbegin, shouldcheckack
    shouldcheckack = True
    waitackbegin = time.time() * 1000
    ackrecv = False

def checkack():
    global resendcount, ackrecv, iserror, waitackbegin, shouldcheckack

    if not shouldcheckack:
        return "ACK"

    if ackrecv:
        shouldcheckack = False
        return "ERR" if iserror else "ACK"
    
    now = time.time() * 1000
    if now - waitackbegin > 1000:
        shouldcheckack = False
        return "ERR"
    else:
        return "WAIT"

def txhandle(cs):
    global sending, lastsent, resendcount, shouldcheckack
    global txfn, txfs, txsn, txfptr, txtm

    s = checkack()

    if s == "ACK":
        resendcount = 0
        if lastsent:
            if lastsent[0] == 0xfe:
                t = time.time()-txtm
                print(f"Finished sending file '{txfn}' took {round(t, 2)}s ({round(txfs*8/t)}bps)")
                b['state'] = 'normal'
            if lastsent[0] == 0xfd:
                txsn += 252
                print(f"Sending file '{txfn}' ({round(txsn/txfs*100)}%)")
            lastsent = None

    if s == "ACK" or s == "WAIT":
        if sending and s == "ACK":
            with mtx:
                d = txfptr.read(252)
                l = len(d)
                if l == 252:
                    byts = b"\xfd" + d
                else:
                    byts = bytes([0xfe, l]) + d
                    txfptr.close()
                    txfptr = None
                    sending = False
                crc = crc_16_fast(byts[1:])
                crcbytes = bytes([(crc >> 8) & 0xff, crc & 0xff])
                txput(byts + crcbytes)

        if not smp.acquire(timeout=0.001):
            return
        byts = txqueue.get()
    else:
        resendcount += 1
        if resendcount > 5:
            print("Too many resend attempts. Quitting...")
            with mtx:
                txfptr.close()
                sending = False
                resendcount = 0
                txfptr = None
                lastsent = None
                shouldcheckack = False
                b['state'] = 'normal'
            return
        print("Resend attempt", resendcount)
        byts = lastsent

    cs.wait(0.5)
    cs.clear()

    if byts[0] == 0xff:
        ser.write(byts)

    if byts[0] == 0xfb:
        sending = True
        beginwaitack()
        ser.write(byts)
        lastsent = byts
        txtm = time.time()

    if byts[0] == 0xfd or byts[0] == 0xfe:
        beginwaitack()
        ser.write(byts)
        lastsent = byts

    if byts[0] == 0xfa:
        ser.write(byts)

def rxfunc(cs):
    global recving, rxlast, rxfptr
    while 1:
        a = ser.readline()
        if a:
            rxhandle(a, cs)

        if recving:
            d = time.time() * 1000 - rxlast
            if d > 5000:
                print(f"No reply in {round(d)}ms terminating receiving file '{rxfn}'")
                recving = False
                rxfptr.close()
                sendterm()

def txfunc(cs):
    while 1:
        txhandle(cs)


if __name__ == '__main__':
    print(f"Waiting for Arduino ({port})")
    while 1:
        a = ser.readline()
        if a == b"READY\n":
            break
    print("Arduino is ready")
    
    cansend = threading.Event()
    cansend.set()
    rxthread = threading.Thread(target=rxfunc, args=[cansend], daemon=True)
    txthread = threading.Thread(target=txfunc, args=[cansend], daemon=True)
    rxthread.start()
    txthread.start()
    
    w = Tk()
    w.title(f"Send File[{port}]")
    w.minsize(300, 0)
    w.resizable(False, False)
    
    b = Button(w, text="Choose File", command=sendfile)
    b.pack(side=TOP)
    w.mainloop()
