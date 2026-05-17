#!/usr/bin/env python3

import optparse
import sys
import time

import can

# ------------------------------------------------------------------------------
BL_CMD_UNLOCK = 0xA0
BL_CMD_DATA = 0xA1
BL_CMD_VERIFY = 0xA2
BL_CMD_RESET = 0xA3
BL_CMD_BKSWAP_RESET = 0xA4
BL_CMD_DEVCFG_DATA = 0xA5
BL_CMD_READ_VERSION = 0xA6

BL_RESP_OK = 0x50
BL_RESP_ERROR = 0x51
BL_RESP_INVALID = 0x52
BL_RESP_CRC_OK = 0x53
BL_RESP_CRC_FAIL = 0x54

BL_GUARD = 0x5048434D

ERASE_SIZE = 256
BOOTLOADER_SIZE = 2048
DEV_CFG_SUPPORT = False

# Supported Devices [ERASE_SIZE, BOOTLOADER_SIZE, DEV_CFG_SUPPORT]
devices = {
    "SAME7X": [8192, 8192, False],
    "SAME5X": [8192, 8192, True],
    "SAMD5X": [8192, 8192, True],
    "SAMG5X": [8192, 8192, False],
    "SAMC2X": [256, 2048, True],
    "SAMD1X": [256, 2048, True],
    "SAMD2X": [256, 2048, True],
    "SAMDA1": [256, 2048, True],
    "SAML1X": [256, 2048, True],
    "SAML2X": [256, 2048, True],
    "SAMHA1": [256, 2048, True],
    "SAMRH71": [256, 8192, False],
    "SAMRH71EK_PROM": [4096, 8192, False],
    "SAMRH71TFBGA_PROM": [8192, 8192, False],
    "SAMA5": [512, 131072, False],
    "SAMA7": [512, 131072, False],
    "SAM9X6": [512, 131072, False],
    "SAM9X7": [512, 131072, False],
    "PIC32MK": [4096, 8192, False],
    "PIC32MZ": [16384, 16384, False],
    "PIC32MZW": [4096, 8192, False],
    "PIC32MX": [1024, 4096, False],
    "PIC32MM": [2048, 4096, False],
    "PIC32CM": [256, 2048, True],
    "PIC32CZ": [4096, 8192, False],
    "WBZ451": [4096, 4096, False],
    "PIC32CXBZ2": [4096, 4096, False],
    "WBZ45X": [4096, 4096, False],
    "WBZ351": [4096, 4096, False],
    "PIC32CZCA70": [8192, 8192, False],
    "PIC32CM_GC00_SG00": [
        4096,
        8192,
        True,
        [{"start": 0xA000000, "size": 0x1000}, {"start": 0xA002000, "size": 0x1000}],
    ],
    "PIC32CK_GC01_SG01": [
        4096,
        4096,
        True,
        [
            {"start": 0xA000000, "size": 0x1000},
            {"start": 0xA002000, "size": 0x1000},
            {"start": 0xA008000, "size": 0x1000},
            {"start": 0xA00A000, "size": 0x1000},
        ],
    ],
    "PIC32CX_MT": [8192, 8192, False],
    "PIC32WM_BZ6": [4096, 4096, False],
    "PIC32CX_SG41": [8192, 8192, True],
}


# ------------------------------------------------------------------------------
def error(text):
    sys.stderr.write('\nError: %s\n' % text)
    sys.exit(-1)


# ------------------------------------------------------------------------------
def warning(text):
    sys.stderr.write('\nWarning: %s\n' % text)


# ------------------------------------------------------------------------------
def verbose(verb, text):
    if verb:
        print("\n" + text)


# ------------------------------------------------------------------------------
def crc32_tab_gen():
    res = []

    for i in range(256):
        value = i

        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ 0xEDB88320
            else:
                value = value >> 1

        res += [value]

    return res


# ------------------------------------------------------------------------------
def crc32(tab, data):
    crc = 0xFFFFFFFF

    for d in data:
        crc = tab[(crc ^ d) & 0xFF] ^ (crc >> 8)
    return crc


# ------------------------------------------------------------------------------
def uint32(v):
    return [(v >> 0) & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]


# ------------------------------------------------------------------------------
class SocketCanBootloaderTransport:
    """Simple raw CAN/CAN FD transport.

    Frame format used here:
      byte 0: sequence number
      byte 1: flags (bit0 = last frame)
      byte 2..: payload bytes

    This is only a host-side chunking scheme. It must match the target bootloader.
    If your target expects ISO-TP or a different CAN framing, adjust this class.
    """

    def __init__(
        self,
        interface: str,
        req_id: int,
        resp_id: int,
        timeout: float = 2.0,
        use_fd: bool = False,
        extended_id: bool = False,
    ):
        self.interface = interface
        self.req_id = req_id
        self.resp_id = resp_id
        self.timeout = timeout
        self.use_fd = use_fd
        self.extended_id = extended_id
        self.chunk_payload = 61 if use_fd else 5  # 64-3 for FD, 8-3 for classic CAN

        try:
            self.bus = can.Bus(interface="socketcan", channel=interface)
        except Exception as exc:
            error(f"failed to open SocketCAN interface {interface}: {exc}")

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _send_payload(self, payload: bytes):
        seq = 0
        offset = 0

        while offset < len(payload):
            chunk = payload[offset : offset + self.chunk_payload]
            offset += len(chunk)
            last = 1 if offset >= len(payload) else 0
            frame_data = bytes([seq & 0xFF, last & 0x01]) + chunk

            msg = can.Message(
                arbitration_id=self.req_id,
                is_extended_id=self.extended_id,
                is_fd=self.use_fd,
                bitrate_switch=self.use_fd,
                data=frame_data,
            )
            self.bus.send(msg)
            seq = (seq + 1) & 0xFF

    def _recv_matching(self, timeout=None):
        timeout = self.timeout if timeout is None else timeout
        deadline = time.time() + timeout

        while time.time() < deadline:
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue
            if msg.arbitration_id != self.resp_id:
                continue
            return msg

        return None

    def get_response(self):
        msg = self._recv_matching()
        if msg is None or len(msg.data) == 0:
            return None
        return msg.data[0]

    def get_version(self):
        msg = self._recv_matching()
        if msg is None or len(msg.data) < 2:
            return None
        major_version = msg.data[0]
        minor_version = msg.data[1]
        return f"v{major_version}.{minor_version}"

    def send_request(self, cmd, size, data):
        packet = uint32(BL_GUARD) + size + [cmd] + data
        payload = bytes(packet)

        for i in range(3):
            self._send_payload(payload)
            resp = self.get_response()

            if resp is None:
                warning('no response received, retrying %d' % (i + 1))
                time.sleep(0.2)
            else:
                return resp

        error('no response received, giving up')


# ------------------------------------------------------------------------------
def printProgressBar(
    iteration, total, prefix='', suffix='', decimals=1, length=100, fill='|'
):
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print('\r%s |%s| %s%% %s \r' % (prefix, bar, percent, suffix), end="")

    if iteration == total:
        print()


# ------------------------------------------------------------------------------
def send_device_configurations(devCfgFile, transport, erase_size):
    data = []
    address = 0

    input_file = open(devCfgFile)

    for line in input_file:
        line = line.strip()

        if not line:
            continue

        if "ROW_START" in line:
            try:
                address = int(line.split(' ')[1], 0)
                rowStart = address & (~(erase_size - 1))
                prefixBytes = address - rowStart

                for _ in range(0, prefixBytes):
                    data += [0xFF]

            except Exception:
                error(
                    'Provide valid address for the Row in the deviceconfiguration file (Example: ROW_START 0x12345678'
                )

        elif line == "ROW_END":
            while len(data) % erase_size > 0:
                data += [0xFF]

            blocks = [data[i : i + erase_size] for i in range(0, len(data), erase_size)]

            for blk in blocks:
                resp = transport.send_request(
                    BL_CMD_DEVCFG_DATA, uint32(erase_size + 4), uint32(rowStart) + blk
                )

                rowStart += erase_size

                if resp != BL_RESP_OK:
                    if resp == BL_RESP_INVALID:
                        warning(
                            'Device configuration programming is not supported, Enable Fuse Programming in MHC for Bootloader (status = 0x%02x)'
                            % resp
                        )
                        return
                    else:
                        error(
                            'Device configuration programming failed (status = 0x%02x)'
                            % resp
                        )

            data = []

        else:
            value = int(line, 0)
            data += [value & 0xFF]
            data += [(value >> 8) & 0xFF]
            data += [(value >> 16) & 0xFF]
            data += [(value >> 24) & 0xFF]

    input_file.close()


# ------------------------------------------------------------------------------
def main():
    parser = optparse.OptionParser(usage='usage: %prog [options]')
    parser.add_option(
        '-v',
        '--verbose',
        dest='verbose',
        help='enable verbose output',
        default=False,
        action='store_true',
    )
    parser.add_option(
        '-i',
        '--interface',
        dest='port',
        help='SocketCAN interface (example: can0)',
        metavar='IFACE',
    )
    parser.add_option(
        '-f', '--file', dest='file', help='binary file to program', metavar='FILE'
    )
    parser.add_option(
        '-g',
        '--devCfgBinfile',
        dest='devCfgBinfile',
        help='binary file to program device configuration',
        metavar='DEVCFGBINFILE',
    )
    parser.add_option(
        '-c',
        '--devcfgfile',
        dest='devcfgfile',
        help='device configuration text file',
        metavar='DEVCFGFILE',
    )
    parser.add_option(
        '-a', '--address', dest='address', help='destination address', metavar='ADDR'
    )
    parser.add_option(
        '-e',
        '--devCfgAddress',
        dest='devCfgAddress',
        help='device configuration address',
        metavar='ADDR',
    )
    parser.add_option(
        '-p',
        '--sectorSize',
        dest='sectSize',
        help='Device Sector Size in Bytes',
        metavar='SectSize',
    )
    parser.add_option(
        '-b',
        '--boot',
        dest='boot',
        help='enable write to the bootloader area',
        default=False,
        action='store_true',
    )
    parser.add_option(
        '-s',
        '--swap',
        dest='swap',
        help='swap banks after programming',
        default=False,
        action='store_true',
    )
    parser.add_option(
        '-d', '--device', dest='device', help='target device', metavar='DEV'
    )

    parser.add_option(
        '--req-id',
        dest='req_id',
        default='0x600',
        help='request CAN ID',
        metavar='CANID',
    )
    parser.add_option(
        '--resp-id',
        dest='resp_id',
        default='0x650',
        help='response CAN ID',
        metavar='CANID',
    )
    parser.add_option(
        '--fd', dest='fd', default=False, action='store_true', help='use CAN FD frames'
    )
    parser.add_option(
        '--extid',
        dest='extid',
        default=False,
        action='store_true',
        help='use extended CAN IDs',
    )

    (options, args) = parser.parse_args()

    if options.port is None:
        error('SocketCAN interface is required (try -h option)')

    if options.file is None and options.devCfgBinfile is None:
        error('File name is required (use -f or -g option)')

    if options.devCfgBinfile is not None and options.devCfgAddress is None:
        error('device configuration address is required (use -e option)')

    if options.device is None:
        error('target device is required (use -d option)')

    if options.address is None:
        if options.device.upper() not in ("SAMA5", "SAMA7", "SAM9X6", "SAM9X7"):
            error('destination address is required (use -a option)')

    device = options.device.upper()

    global ERASE_SIZE
    global BOOTLOADER_SIZE
    global DEV_CFG_SUPPORT

    if device in devices:
        if device == "PIC32MX":
            if options.sectSize is None:
                error('device sector size is required (use -p option)')
            ERASE_SIZE = int(options.sectSize)
        else:
            if device in ("SAMA5", "SAMA7", "SAM9X6", "SAM9X7"):
                if options.sectSize is None:
                    ERASE_SIZE = devices[device][0]
                else:
                    ERASE_SIZE = int(options.sectSize)
            else:
                ERASE_SIZE = devices[device][0]

        BOOTLOADER_SIZE = devices[device][1]
        DEV_CFG_SUPPORT = devices[device][2]
    else:
        error('invalid device')

    if options.swap:
        if device not in (
            "SAME5X",
            "SAMD5X",
            "PIC32MZ",
            "PIC32CX_SG41",
            "PIC32MK",
            "PIC32CX_MT",
            "PIC32CZ",
        ):
            error('Bank Swapping not supported on this device')

    try:
        if device in ("SAMA5", "SAMA7", "SAM9X6", "SAM9X7"):
            address = 0
        else:
            address = int(options.address, 0)
    except ValueError:
        error('invalid address value: %s' % options.address)

    if device not in ("SAMA5", "SAMA7", "SAM9X6", "SAM9X7"):
        if ("SAM" in device) or ("PIC32C" in device):
            if address < BOOTLOADER_SIZE and options.boot is False:
                error(
                    'address is within the bootloader area, use --boot options to unlock writes'
                )
        else:
            if options.boot is True:
                error('--boot option is not supported on this device')

    transport = SocketCanBootloaderTransport(
        interface=options.port,
        req_id=int(options.req_id, 0),
        resp_id=int(options.resp_id, 0),
        timeout=2.0,
        use_fd=options.fd,
        extended_id=options.extid,
    )

    data = []
    data1 = []
    data2 = []

    verbose(options.verbose, 'Reading Bootloader Version')

    resp = transport.send_request(BL_CMD_READ_VERSION, uint32(0), uint32(0))

    if resp != BL_RESP_OK:
        error('invalid response code (0x%02x). Read Bootloader version failed.' % resp)

    version = transport.get_version()
    if version is not None:
        verbose(options.verbose, 'Bootloader version : %s' % version)
    else:
        warning('no version bytes received')

    if ("PIC32MK" in device) or ("PIC32MZ" in device):
        address = address & (~(ERASE_SIZE - 1))

    appAddr = address
    count = 0
    if options.devCfgBinfile is not None:
        count += 1
    if options.file is not None:
        count += 1

    for cnt in range(0, count):
        if (options.devCfgBinfile is not None) and count == 2 and cnt == 0:
            data1 += [(x) for x in open(options.devCfgBinfile, 'rb').read()]
            address = int(options.devCfgAddress, 0)
            data = data1

        if (options.file is not None) and (
            (count == 2 and cnt == 1) or (count == 1 and cnt == 0)
        ):
            data2 += [(x) for x in open(options.file, 'rb').read()]
            address = appAddr
            data = data2

        if device not in ["SAMA5", "SAMA7", "SAM9X6", "SAM9X7"]:
            while len(data) % ERASE_SIZE > 0:
                data += [0xFF]

        crc32_tab = crc32_tab_gen()
        crc = crc32(crc32_tab, data)

        size = len(data)

        if options.file and ((count == 2 and cnt == 1) or (count == 1 and cnt == 0)):
            verbose(options.verbose, 'Unlocking\n')
            resp = transport.send_request(
                BL_CMD_UNLOCK, uint32(8), uint32(address) + uint32(size)
            )

        if resp != BL_RESP_OK:
            error(
                'invalid response code (0x%02x). Check that your file size and address are correct.'
                % resp
            )

        blocks = [data[i : i + ERASE_SIZE] for i in range(0, len(data), ERASE_SIZE)]

        addr = address

        for idx, blk in enumerate(blocks):
            if ((idx + 1) == len(blocks)) and ((size % ERASE_SIZE) != 0):
                data_length = size % ERASE_SIZE
            else:
                data_length = ERASE_SIZE

            if (options.devCfgBinfile is not None) and count == 2 and cnt == 0:
                verbose(
                    options.verbose,
                    'Unlocking address range: 0x%08X - 0x%08X'
                    % (addr, addr + data_length),
                )
                resp = transport.send_request(
                    BL_CMD_UNLOCK, uint32(8), uint32(addr) + uint32(data_length)
                )

            if resp != BL_RESP_OK:
                error(
                    'Unlock failed for address range: 0x%08X - 0x%08X'
                    % (addr, addr + data_length)
                )
                break

            printProgressBar(
                idx + 1,
                len(blocks),
                prefix='Programming:',
                suffix='Complete',
                length=50,
            )

            if (options.devCfgBinfile is not None) and count == 2 and cnt == 0:
                addr_range_list = devices[device][3]
                resp = BL_RESP_OK
                for x in addr_range_list:
                    devCfgStartAddr = x["start"]
                    devCfgEndAddr = x["start"] + x["size"]
                    if addr >= devCfgStartAddr and addr < devCfgEndAddr:
                        resp = transport.send_request(
                            BL_CMD_DEVCFG_DATA,
                            uint32(data_length + 4),
                            uint32(addr) + blk,
                        )
                        break

            if (options.devCfgBinfile is not None) and count == 2 and cnt == 0:
                verbose(
                    options.verbose,
                    'Verifying CRC for address range: 0x%08X - 0x%08X'
                    % (addr, addr + data_length),
                )
                crc_chunk = crc32(crc32_tab, blk)

                resp = transport.send_request(
                    BL_CMD_VERIFY, uint32(4), uint32(crc_chunk)
                )

                if resp == BL_RESP_CRC_OK:
                    verbose(
                        options.verbose,
                        '... CRC success for address range 0x%08X - 0x%08X'
                        % (addr, addr + data_length),
                    )
                else:
                    error(
                        '... CRC failed for address range 0x%08X - 0x%08X'
                        % (addr, addr + data_length)
                    )

                verbose(
                    options.verbose,
                    'Rebooting MCU after processing address range: 0x%08X - 0x%08X'
                    % (addr, addr + data_length),
                )
                resp = transport.send_request(BL_CMD_RESET, uint32(0), uint32(0))
                time.sleep(1)
                if resp != BL_RESP_OK:
                    error(
                        'Reboot failed after processing address range: 0x%08X - 0x%08X'
                        % (addr, addr + data_length)
                    )
                else:
                    verbose(options.verbose, 'Reboot successful.')

            if (options.file is not None) and (
                (count == 2 and cnt == 1) or (count == 1 and cnt == 0)
            ):
                resp = transport.send_request(
                    BL_CMD_DATA, uint32(data_length + 4), uint32(addr) + blk
                )

            addr += data_length

            if resp != BL_RESP_OK:
                error('invalid response code (0x%02x)' % resp)

            if device == "PIC32CZ":
                time.sleep(1)

        if options.file and ((count == 2 and cnt == 1) or (count == 1 and cnt == 0)):
            if device == "PIC32CX_MT":
                time.sleep(1)

            verbose(options.verbose, 'Verification')
            resp = transport.send_request(BL_CMD_VERIFY, uint32(4), uint32(crc))
            if resp == BL_RESP_CRC_OK:
                verbose(options.verbose, '... success')
            else:
                error('... fail (status = 0x%02x)' % resp)

        if options.devcfgfile is not None:
            if DEV_CFG_SUPPORT is False:
                warning(
                    'Device configuration programming is not supported for this device'
                )
            else:
                verbose(options.verbose, 'Sending Device Configuration Bits')
                send_device_configurations(options.devcfgfile, transport, ERASE_SIZE)

        if options.swap is True:
            verbose(options.verbose, 'Swapping Bank And Rebooting')
            resp = transport.send_request(
                BL_CMD_BKSWAP_RESET, uint32(16), uint32(0) * 4
            )
        else:
            verbose(options.verbose, 'Rebooting')
            resp = transport.send_request(BL_CMD_RESET, uint32(16), uint32(0) * 4)

        if resp == BL_RESP_OK:
            verbose(options.verbose, 'Reboot Done')
        else:
            error('... Reset fail (status = 0x%02x)' % resp)

    transport.close()


# ------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
