#run with parameter True to put in verbose mode
import subprocess, os, sys, time, fcntl
from subprocess import Popen, PIPE

SUPPORTED_PIXEL_FORMATS = ['JPEG','RGB3','BGR3','RGB4','BGR4','BGR','YUYV','GRAY8','NV12','YV12','I420']

VERBOSE = False
if len(sys.argv) > 1:
	VERBOSE = sys.argv[1]

#============================= RESET USB FUNCTIONS ===========================================
def create_usb_list():
    device_list = []
    try:
        lsusb_out = Popen('lsusb -v', shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read().strip().decode('utf-8')
        usb_devices = lsusb_out.split('%s%s' % (os.linesep, os.linesep))
        for device_categories in usb_devices:
            if not device_categories:
                continue
            categories = device_categories.split(os.linesep)
            device_stuff = categories[0].strip().split()
            bus = device_stuff[1]
            device = device_stuff[3][:-1]
            path = '/dev/bus/usb/%s/%s' % (bus, device)

            device_list.append(path)
    except Exception as ex:
        p('Failed to list usb devices! Error: %s' % ex)
    return device_list

def reset_usb_device(dev_path):
    USBDEVFS_RESET = 21780
    try:
        p('resetting... %s' % dev_path)
        f = open(dev_path, 'w', os.O_WRONLY)
        fcntl.ioctl(f, USBDEVFS_RESET, 0)
        p('Successfully reset %s' % dev_path)
    except Exception as ex:
        p('Failed to reset device! Error: %s' % ex)


#============================= FIND ALL CAMERAS ===========================================

def cmd(c):
	return subprocess.check_output(c, shell=True).decode().splitlines()
def p(c):
	if VERBOSE:
		print(c)


#reset all usb devices (to clean up any cameras that were left open or are in use etc.)
print('reset usb devices (Resource temporarily unavailable is OK)')
list = create_usb_list();
for i in range(len(list)):
	reset_usb_device(list[i])
#give the kernel time to rebuild the /dev/video directory
time.sleep(1)
print('...done');

#get list of cameras with their /dev/video devices
result = cmd('v4l2-ctl --list-devices || true')
nextIsName,namedDevices = True,{}
for r in result:
	if r == '':
		nextIsName = True
	elif nextIsName:
		namedDevices[r] = []
		lastName = r
		nextIsName = False;
	else:
		namedDevices[lastName].append(r.replace('\t',''))

#check and possibly switch formats for each video device so there is at least 1 device per cameras
#that has an uncompressed pixel format
print('===============')
for dname in sorted(namedDevices.keys()):
	p(dname)
	if 'bcm2835-codec' not in dname:	#make sure fake camera on the PI is not included
		for dpath in namedDevices[dname]:
			fmtLines = cmd('v4l2-ctl -d '+dpath+' --get-fmt-video || true');
			if 'Invalid argument' not in fmtLines[0]:
				p(dpath + ' has format ' + (fmtLines[2][22:])[0:4])
				print(dpath)
				break;
