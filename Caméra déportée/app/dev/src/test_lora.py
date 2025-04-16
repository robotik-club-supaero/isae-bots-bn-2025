from lora import ModemConfig, LoRa
import struct
import time

print("Setting up  LoRa...")
lora = LoRa(channel=0, interrupt=17, this_address=2, modem_config=ModemConfig.Bw500Cr45Sf128, tx_power=14, acks=False)
header_to = 255 # Send to all
lora.set_mode_tx()

list = [10*i for i in range(11)]

start = time.time()

for item in list:
    lora.send(item, header_to)

print(time.time() - start)
print("\n")

start = time.time()

format = f'{len(list)}i'
payload = struct.pack(format, *list)
lora.send(payload, header_to)

print(time.time() - start)
print("\n")

lora.close()