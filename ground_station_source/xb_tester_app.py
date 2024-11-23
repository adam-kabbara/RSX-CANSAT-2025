import PySimpleGUI as sg
import time
from serial import Serial
import serial.tools.list_ports
import time
import threading
from queue import Queue

THREAD_EVENT_1 = '-NEW_DATA_THREAD-'
BAUD_RATE = 57600

back_color = '#404252'
big_text_color = '#d2d4da'
data_back = '#b3b5bd'
data_text = '#282a3a'

msg = "NOTHING YET..."
status = "STATUS: OFFLINE"

# All the stuff inside your window.
layout = [  
            [sg.Text(status, key='-STATUS-', font=('Arial', 20), text_color = big_text_color, background_color=back_color, expand_x=True)],
            [sg.Text('', background_color=back_color, size=(0,4), expand_x=True)],
            [sg.Text('RECEIVED MESSAGE', font=('Arial Bold', 40), justification='center', expand_x=True, text_color=big_text_color, background_color=back_color)],
            [sg.Text(msg, key='-RECV-', font=('Arial', 20), justification='center', expand_x=True, text_color='#159947', background_color=data_back)],
            [sg.Text('', background_color=back_color, size=(0, 5), expand_x=True)],
            [sg.Text('ENTER COMMAND', font=('Arial Bold', 40), justification='center', expand_x=True, text_color=big_text_color, background_color=back_color)],
            [sg.InputText(key='-INPUT-', font=('Arial', 20), expand_x=True, justification='center', background_color=data_back, text_color=data_text, border_width='0'),
                sg.Button('SEND', mouseover_colors='#159947', font=('Arial Bold', 20), button_color=(big_text_color, '#2d2e40'), border_width='0', size=(20,0))],
            [sg.Text('', background_color=back_color, size=(0,4), expand_x=True)],
            [sg.Push(background_color=back_color), 
            sg.Column([
                [sg.Button('CLOSE CONNECTION', font=('Arial Bold', 20), button_color=(big_text_color, '#2d2e40'), border_width='0', size=(20, 0)),
                sg.Text('', background_color=back_color, size=(2,0)),
                sg.Button('OPEN CONNECTION', font=('Arial Bold', 20), button_color=(big_text_color, '#2d2e40'), border_width='0', size=(20, 0))]
            ], justification='center', element_justification='center', expand_x=True, background_color=back_color), 
            sg.Push(background_color=back_color)]
]

# Create the Window
window = sg.Window('Xbee Test App', layout, background_color=back_color, finalize=True)
window.maximize()

ports = list(serial.tools.list_ports.comports())

matching_ports = [port for port in ports if "usb serial port" in port.description.lower()]

device_found = False
device = None
queue = Queue()

if len(matching_ports) > 1:
    window['-STATUS-'].update("ERROR: TOO MANY PORTS. CONNECT ONE AND RESTART APP")
elif len(matching_ports) == 1:
    device = Serial(matching_ports[0].device, BAUD_RATE, timeout=0)
    window['-STATUS-'].update(f"STATUS: CONNECTED TO PORT {matching_ports[0].device}")
    device_found = True
else:
    window['-STATUS-'].update("ERROR: DID NOT FIND ANY PORTS. CONNECT AND RESTART APP")

def transmit_thread(t_window, t_device, stop_event):
    while not stop_event.is_set():
        if t_device.inWaiting() > 0:
            line = t_device.readline()
            # Make sure the full packet was received
            while not '\\n' in str(line):
                time.sleep(0.001)
                temp = t_device.readline()
                if not not temp.decode():
                    line = (line.decode() + temp.decode()).encode()
            line = line.decode().strip()
            t_window.write_event_value('-NEW_DATA_THREAD-', line)
        else:
            time.sleep(0.01)

def send_thread(t_queue, t_device, stop_event):
    while not stop_event.is_set():
        if not t_queue.empty():
            data_send = t_queue.get()
            try:
                t_device.write(data_send.encode())
            except Exception as e:
                print(f'Exception: {e}')
                continue
        else:
            time.sleep(0.01)

st_event = threading.Event()
t1 = threading.Thread(target=transmit_thread, args=(window,device, st_event))
t2 = threading.Thread(target=send_thread, args=(queue,device, st_event))

try:
    if device_found == False:
        while True:
            event, values = window.read(timeout=100)
            if event == sg.WIN_CLOSED:
                break
    else:
        t1.start()
        t2.start()

        while True:
            event, values = window.read(timeout=100)
            if event == sg.WIN_CLOSED:
                break
            elif event == THREAD_EVENT_1:
                update_recv_msg = values['-NEW_DATA_THREAD-']
                print(update_recv_msg)
                window['-RECV-'].update(update_recv_msg)
            elif event == 'SEND':
                data_to_send = values['-INPUT-']
                if data_to_send:
                    queue.put(data_to_send)
    
finally:
    if device_found == True:
        st_event.set()
        t1.join()
        t2.join()
    window.close()
    if device is not None:
        device.close()