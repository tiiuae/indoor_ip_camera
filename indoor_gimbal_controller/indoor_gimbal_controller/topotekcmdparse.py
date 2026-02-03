'''
Topotek commands examples.
Please refer to Topotek-SIP-series-Protocol.docx to see details of commands.
''' 
import re
import struct
 

def validate_hex_input(data: str) -> None: 
    if ' ' in data:
        pattern = r'^(?:[0-9A-Fa-f]{1,2}\s*)+$'
        if not re.fullmatch(pattern, data):
            raise ValueError("Invalid hex string. Bytes must be 1-2 hex digits separated by spaces.")
    else: 
        if len(data) % 2 != 0 or not re.fullmatch(r'^[0-9A-Fa-f]+$', data):
            raise ValueError("Invalid hex string. Continuous hex must be even length and only 0-9a-f.")


def calculate_crc(command: bytes) -> int: 
    return sum(command) & 0xFF


def build_command(
    frame_header: str,
    address_bit1: str,
    address_bit2: str,
    control_bit: str,
    identifier_bit: str,
    data: str,
    data_mode: str = 'ASCII',
    input_space_separate: bool = False,
    output_format: str = 'ASCII',
    output_space_separate: bool = False
) -> str:
    """
    Assemble a command string.

    Parameters:
      frame_header: one of '#TP', '#tp', '#tP', '#Tp'
      address_bit1: single letter source code, e.g. 'P'
      address_bit2: single letter dest code, e.g. 'G'
      control_bit: 'w' or 'r' (empty when frame_header='#Tp')
      identifier_bit: three-character command identifier
      data: payload string, ASCII or hex
      data_mode: 'ASCII' or 'Hex'
      input_space_separate: whether hex input is space-separated
      output_format: 'ASCII' or 'Hex'
      output_space_separate: whether to insert spaces in hex output

    Returns:
      The assembled command as a string in ASCII or hex form.

    Raises:
      ValueError on invalid parameters or data.
    """
 
    if frame_header not in ('#TP', '#tp', '#tP', '#Tp'):
        raise ValueError(f"Unsupported frame header: {frame_header}")
    if len(address_bit1) != 1 or len(address_bit2) != 1:
        raise ValueError("Address bits must be single characters.")
    if len(identifier_bit) != 3:
        raise ValueError("Identifier bit must be exactly 3 characters.")
 
    if data_mode == 'ASCII':
        data_bytes = data.encode('ascii')
    elif data_mode == 'Hex': 
        if input_space_separate:
            validate_hex_input(data)
            data_bytes = bytes(int(b, 16) for b in data.split())
        else:
            validate_hex_input(data)
            data_bytes = bytes.fromhex(data)
    else:
        raise ValueError(f"Unsupported data_mode: {data_mode}")
    data_length = len(data_bytes)
     
    lh = frame_header
    if lh == '#TP':
        length_bytes = b'2'
        cb = control_bit
    elif lh == '#tp':
        if data_length > 0x0F:
            raise ValueError("Data length exceeds maximum for #tp (15 bytes).")
        length_bytes = f"{data_length:X}".encode('ascii')
        cb = control_bit
    elif lh == '#tP':
        if data_length > 0xFF:
            raise ValueError("Data length exceeds maximum for #tP (255 bytes).")
        length_bytes = data_length.to_bytes(1, 'big')
        cb = control_bit
    elif lh == '#Tp':
        if data_length > 0xFFFF:
            raise ValueError("Data length exceeds maximum for #Tp (65535 bytes).")
        length_bytes = data_length.to_bytes(2, 'big')
        cb = ''  # no control bit
    else: 
        raise ValueError("Invalid frame header")
   
    cmd = bytearray()
    cmd.extend(frame_header.encode('ascii'))
    cmd.extend(address_bit1.encode('ascii'))
    cmd.extend(address_bit2.encode('ascii'))
    cmd.extend(length_bytes)
    if cb:
        cmd.extend(cb.encode('ascii'))
    cmd.extend(identifier_bit.encode('ascii'))
    cmd.extend(data_bytes)
    #  CRC
    crc_val = calculate_crc(cmd)
    cmd.extend(f"{crc_val:02X}".encode('ascii'))
 
    if output_format == 'ASCII':
        try:
            return cmd.decode('ascii')
        except UnicodeDecodeError:
            raise ValueError("Command contains non-ASCII bytes; choose hex output.")
    elif output_format == 'Hex':
        hexstr = cmd.hex().upper()
        if output_space_separate:
            return ' '.join(hexstr[i:i+2] for i in range(0, len(hexstr), 2))
        return hexstr
    else:
        raise ValueError(f"Unsupported output_format: {output_format}")
 
 

 












 