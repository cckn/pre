import serial
import threading
import colorama
import os

# init
colorama.init(autoreset=True)
# lora_serial = serial.Serial('/dev/ttyAMA0',115200)
lora_serial = serial.Serial('/dev/serial0', 921600)
# lora_serial = serial.Serial('/dev/ttyUSB0',9600)
lora_serial.close()
lora_serial.open()


class cmd_input(threading.Thread):  # user input cmd recv

    def run(self):

        print "\n\n\n\t" +                                      \
              colorama.Back.WHITE + "   " +                      \
              colorama.Back.RESET + " :: WHITE is your cmd\n"
        while True:
            cmd = raw_input()
            lora_serial.write(cmd + '\n')


class print_rx_msg(threading.Thread):  # LoRa msg recv

    def run(self):
        print colorama.Fore.GREEN + "\n\t" +                     \
            colorama.Back.GREEN + "   " +                      \
            colorama.Back.RESET + " :: GREEN is LoRa\'s Response\n\n"
        while True:
            rx_msg = lora_serial.readline()
            print colorama.Fore.GREEN + rx_msg


def main():
    os.system('clear')
    cmd_in = cmd_input()
    print_rx = print_rx_msg()
    cmd_in.start()
    print_rx.start()

if __name__ == "__main__":
    main()
