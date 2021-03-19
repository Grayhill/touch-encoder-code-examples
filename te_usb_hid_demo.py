"""te_usb_hid_demo.py
Sample code demonstrating some of the Touch Encoders USB HID functionality.
"""
import sys
import usb.core
import usb.util
import time
import numpy as np
import threading
from dataclasses import dataclass

""" TE-specific defines """
TE_VID = 0x1658
TE_PID = 0x0060
TE_REPORT_ID_EVENTS = 0x01
TE_REPORT_ID_COMMAND = 0x02
TE_REPORT_ID_WIDGET_STATUS = 0x03
TE_REPORT_ID_FORCE_WIDGET = 0x04
TE_EVENT_ENDPOINT_OUT = 0x01
TE_EVENT_ENDPOINT_IN = 0x81
TE_WIDGET_ENDPOINT_OUT = 0x02
TE_WIDGET_ENDPOINT_IN = 0x82
TE_USB_WRITE_TIMEOUT = 100
TE_USB_READ_TIMEOUT = 100
TE_COMMAND_DELAY = .15

""" events report payload structure """
@dataclass
class events_payload:
    screen_id: int
    reserved1: int
    event_id: int
    encoder_val: int
    tap_mask: int
    swipe_mask: int

""" command report payload structure """
@dataclass
class command_payload:
    command_id: int
    command_val1: int
    command_val2: int

""" widget report payload structure """
@dataclass
class widget_payload:
    screen_id: int
    active_val_mask: int
    val1_id: int
    val1_val: int
    val1_disp: int
    val2_id: int
    val2_val: int
    val2_disp: int
    val3_id: int
    val3_val: int
    val3_disp: int

""" force widget report payload structure """
@dataclass
class force_widget_payload:
    screen_id: int
    active_val_mask: int
    mode_code: int
    val1_id: int
    val1_val: int
    val1_disp: int
    val2_id: int
    val2_val: int
    val2_disp: int
    val3_id: int
    val3_val: int
    val3_disp: int

class HidInterface:
    def __init__(self):
        self.stop_threads = False
        self.saved_wi_rep = []
        self.saved_ev_rep = []
        self.event_report_thread = None
        self.widget_report_thread = None

    """ read_events_thread(self, usb_device)
        Parameters:
            usb_device - USB device in usb.core format

        read thread that is created during init and keeps receiving USB
        packets on events endpoint
    """
    def read_events_thread(self, usb_device):
        while True:
            try:
                ev_rep = usb_device.read(TE_EVENT_ENDPOINT_IN, 64, TE_USB_READ_TIMEOUT)
            except:
                if self.stop_threads:
                    break
            else:
                self.events_read = True
                self.saved_ev_rep = ev_rep
                print("Event report recieved: %s" % ev_rep)

    """ read_widget_thread(self, usb_device)
        Parameters:
            usb_device - USB device in usb.core format

        read thread that is created during init and keeps receiving USB
        packets on widget endpoint
    """
    def read_widget_thread(self, usb_device):
        while True:
            try:
                wi_rep = usb_device.read(TE_WIDGET_ENDPOINT_IN, 64, TE_USB_READ_TIMEOUT)
            except:
                if self.stop_threads:
                    break
            else:
                self.saved_wi_rep = wi_rep
                print("Widget report recieved: %s" % wi_rep)

    """ check_event_report(self, ev)
        Parameters:
            ev - contains events data struct to be checked against a received packet

        checks a recently received events packet to ensure it contains the expected data
    """
    def check_event_report(self, ev):
        if self.events_read:
            return self.saved_ev_rep[0] == TE_REPORT_ID_EVENTS and \
                   self.saved_ev_rep[1] == ev.screen_id and \
                   self.saved_ev_rep[2] == 0 and \
                   self.saved_ev_rep[3] == ev.event_id and \
                   self.saved_ev_rep[4] == ev.encoder_val and \
                   self.saved_ev_rep[5] == ev.tap_mask and \
                   self.saved_ev_rep[6] == ev.swipe_mask
        return False

    """ check_widget_report(self, wi)
        Parameters:
            wi - contains widget data struct to be checked against a received packet

        checks a recently received widget packet to ensure it contains the expected data
    """
    def check_widget_report(self, wi):
        if wi.mode_code & 0x01 == 0x01:
            return True
        if self.widget_read:
            if self.saved_wi_rep[0] == TE_REPORT_ID_WIDGET_STATUS and \
                self.saved_wi_rep[1] == wi.screen_id:
                if wi.active_val_mask & 0x01 == 0x01:
                    return self.saved_wi_rep[3] == wi.val1_id and \
                            self.saved_wi_rep[4] == (np.uint16(wi.val1_val) >> 0) & 0xFF and \
                            self.saved_wi_rep[5] == (np.uint16(wi.val1_val) >> 8) & 0xFF and \
                            self.saved_wi_rep[6] == wi.val1_disp
                elif wi.active_val_mask & 0x02 == 0x02:
                    return self.saved_wi_rep[7] == wi.val2_id and \
                            self.saved_wi_rep[8] == (np.uint16(wi.val2_val) >> 0) & 0xFF and \
                            self.saved_wi_rep[9] == (np.uint16(wi.val2_val) >> 8) & 0xFF and \
                            self.saved_wi_rep[10] == wi.val2_disp
                elif wi.active_val_mask & 0x04 == 0x04:
                    return self.saved_wi_rep[11] == wi.val3_id and \
                            self.saved_wi_rep[12] == (np.uint16(wi.val3_val) >> 0) & 0xFF and \
                            self.saved_wi_rep[13] == (np.uint16(wi.val3_val) >> 8) & 0xFF and \
                            self.saved_wi_rep[14] == wi.val2_disp
        return False

""" send_command(self, usb_device, cmd)
    Parameters:
        usb_device - USB device in usb.core format
        cmd - contains command struct to be made into OUT report payload

    sends a command report to TE device
"""
def send_command(usb_device, cmd):
    command = [TE_REPORT_ID_COMMAND,
              cmd.command_id,
              (np.uint32(cmd.command_val1) >> 0) & 0xFF,
              (np.uint32(cmd.command_val1) >> 8) & 0xFF,
              (np.uint32(cmd.command_val1) >> 16) & 0xFF,
              (np.uint32(cmd.command_val1) >> 24) & 0xFF,
              (np.uint32(cmd.command_val2) >> 0) & 0xFF,
              (np.uint32(cmd.command_val2) >> 8) & 0xFF,
              (np.uint32(cmd.command_val2) >> 16) & 0xFF]
    print("Sending command: %s" % command)
    return bool(usb_device.write(TE_EVENT_ENDPOINT_OUT,command,TE_USB_WRITE_TIMEOUT) == len(command))


""" update_widget(usb_device, fw, event_flag, widget_flag, timeout)
    Parameters:
        usb_device - USB device in usb.core format
        fw - contains force widget struct to be made into OUT report payload
        delay - time waitied for command to take place (seconds)

    sends a force widget report to TE device
"""
def update_widget(usb_device, fw, delay):
    report = [TE_REPORT_ID_FORCE_WIDGET,
                fw.screen_id,
                fw.active_val_mask,
                fw.mode_code,
                fw.val1_id,
                (np.uint16(fw.val1_val) >> 0) & 0xFF,
                (np.uint16(fw.val1_val) >> 8) & 0xFF,
                fw.val1_disp,
                fw.val2_id,
                (np.uint16(fw.val2_val) >> 0) & 0xFF,
                (np.uint16(fw.val2_val) >> 8) & 0xFF,
                fw.val2_disp,
                fw.val3_id,
                (np.uint16(fw.val3_val) >> 0) & 0xFF,
                (np.uint16(fw.val3_val) >> 8) & 0xFF,
                fw.val3_disp] + \
                [0] * 20
    print("Sending change widget: %s" % report)
    usb_device.write(TE_WIDGET_ENDPOINT_OUT,report,TE_USB_WRITE_TIMEOUT)
    time.sleep(delay)


""" setup(dev_serial_number=None, hid_interface = HidInterface())
    Parameters:
        dev_serial_number - serial number of touch encoder to initialize
        hid_interface - hid_interface obj contianing report threads to spawn

   Initialize to begin USB HID communication
"""
def setup(dev_serial_number=None, hid_interface = HidInterface()):
    # find our device
    usb_device = None

    if dev_serial_number == None:
        usb_device = usb.core.find(idVendor=TE_VID,
                            idProduct=TE_PID)
    else:
        usb_device = usb.core.find(idVendor=TE_VID,
                            idProduct=TE_PID,
                            serial_number=dev_serial_number)
    # was it found?
    if usb_device is None:
        raise ValueError('Device not found')

    """ if necessary, detach device from kernel driver """
    for cfg in usb_device:
        for intf in cfg:
            if usb_device.is_kernel_driver_active(intf.bInterfaceNumber):
                try:
                    usb_device.detach_kernel_driver(intf.bInterfaceNumber)
                    print("Detach: %s" % intf.bInterfaceNumber)
                except usb.core.USBError as e:
                    sys.exit("Could not detach kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))

    print("Manufacturer: \t%s" % usb_device.manufacturer)
    print("Product: \t%s" % usb_device.product)
    print("Serial Num: \t%s" % usb_device.serial_number)

    """ initiate events & widgets endpoint read threads """
    hid_interface.stop_threads = False
    hid_interface.event_report_thread = threading.Thread(target=hid_interface.read_events_thread, args=(usb_device,))
    hid_interface.widget_report_thread = threading.Thread(target=hid_interface.read_widget_thread, args=(usb_device,))
    hid_interface.widget_report_thread.start()
    hid_interface.event_report_thread.start()

    return usb_device

""" teardown(usb_device, hid_interface)
    Parameters:
        usb_device - pyusb device obj to teardown
        hid_interface - hid_interface obj for device

   teardown usb device resources and connection
"""
def teardown(usb_device, hid_interface):
    """ shut down events & widget endpoint read threads """
    hid_interface.stop_threads = True
    time.sleep(1)
    hid_interface.event_report_thread.join()
    hid_interface.widget_report_thread.join()
    """ release USB resources """
    usb.util.dispose_resources(usb_device)

""" change_widget_values(hid_interface, usb_device, scrn_vals)
    Parameters:
        hid_interface - hid_interface obj for device
        usb_device - pyusb device obj to teardown
        scrn_vals - list of widget values

    sends a force widget report to the device
"""
def change_widget_values(usb_device, hid_interface, scrn_vals):
    for value in range(np.int16(scrn_vals[3]), np.int16(scrn_vals[4]) + 1):
        if scrn_vals[1] == 0x01:
            fw_check = force_widget_payload(scrn_vals[0],
                                            scrn_vals[1],
                                            scrn_vals[2],
                                            scrn_vals[1],
                                            value,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0)
        elif scrn_vals[1] == 0x04:
            fw_check = force_widget_payload(scrn_vals[0],
                                            scrn_vals[1],
                                            scrn_vals[2],
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            0,
                                            scrn_vals[1],
                                            value,
                                            0)
        update_widget(usb_device,
                      fw_check,
                      TE_COMMAND_DELAY)

""" test_command(usb_device, cmd_vals)
    Parameters:
        cmd_vals - hid_interface obj for device
        usb_device - pyusb device obj to teardown

    sends specified command to device
"""
def test_command(usb_device, cmd_val):
    cmd = command_payload(cmd_val[0],
                          cmd_val[1],
                          0)
    send_command(usb_device, cmd)
    time.sleep(TE_COMMAND_DELAY)

def main():
    print("Starting Demo...")
    interface = HidInterface()
    dev = setup(dev_serial_number=None, hid_interface=interface)

    """ scrn_vals - contains the different test iterations for change_widget_values """
    scrn_vals = [[3, 1, 0, 0x8000, 0x800F],
                 [2, 1, 0, -5, 5],
                 [4, 1, 0, 0, 50],
                 [1, 1, 0, 0, 50],
                 [4, 4, 0, 0x8000, 0x801F]]

    for vals in scrn_vals:
        change_widget_values(dev, interface, vals)

    """ cmd_vals - contains the different test iterations for test_command """
    cmd_vals = [[0x80, 10 << 8],
                [0x80, 25 << 8],
                [0x80, 50 << 8],
                [0x80, 75 << 8],
                [0x80, 100 << 8]]

    for cmd in cmd_vals:
        test_command(dev, cmd)

    teardown(dev, interface)
    del interface
    print("Demo Finished!")

if __name__ == "__main__":
    main()