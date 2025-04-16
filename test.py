import time

ACT_LED_PATH = "/sys/class/leds/ACT"

# Function to write values to a file
def write(path, value):
    with open(path, 'w') as f:
        f.write(value)

# Disable the default trigger (SD card activity)
write(f"{ACT_LED_PATH}/trigger", "none")

# Blink loop
try:
    while True:
        write(f"{ACT_LED_PATH}/brightness", "1")  # Turn on LED
        time.sleep(1)
        write(f"{ACT_LED_PATH}/brightness", "0")  # Turn off LED
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopped by user.")
