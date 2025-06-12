import depthai as dai

# this returns both USB and PoE devices in BOOTLOADER or UNBOOTED state
infos = dai.DeviceBootloader.getAllAvailableDevices()
for info in infos:
    print(f"Device found: {info.name} ({info.state})")
    print(f"  IP: {info.ipAddress}")
    print(f"  USB: {info.usbId}")
    print(f"  PoE: {info.poeId}")
    print(f"  Serial: {info.serialNumber}")
    print(f"  Firmware version: {info.firmwareVersion}")
    print(f"  Bootloader version: {info.bootloaderVersion}")
    print(f"  Bootloader state: {info.bootloaderState}")