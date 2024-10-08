def calculate_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum

def verify_checksum(data, checksum):
    calculated_checksum = calculate_checksum(data)
    return calculated_checksum == checksum

data_to_send = b'\x11\x22\x03\x24' 
checksum_to_send = calculate_checksum(data_to_send) 
print(hex(checksum_to_send))


received_data = data_to_send
received_checksum = checksum_to_send


checksum_valid = verify_checksum(received_data, received_checksum)

if checksum_valid:
    print("pass")
else:
    print("failed")
