import glob

def find_tty_devices():
  """Finds all ttyUSB and ttyACM devices connected."""
  tty_devices = []
  for device in glob.glob("/dev/tty[AU]SB*"):
    tty_devices.append(device)
  return tty_devices

if __name__ == "__main__":
  devices = find_tty_devices()
  if devices:
    print("Found tty devices:")
    for device in devices:
      print(device)
  else:
    print("No ttyUSB or ttyACM devices found.")
