# convert.py
def convert_to_c_array(filename):
    with open(filename, 'rb') as f:
        byte_array = f.read()
        c_array = ', '.join(f'0x{byte:02x}' for byte in byte_array)
        return f"const uint8_t {filename.split('.')[0]}[] = {{{c_array}}};\nconst size_t {filename.split('.')[0]}_len = sizeof({filename.split('.')[0]});"

if __name__ == "__main__":
    print(convert_to_c_array("server.crt"))
    print(convert_to_c_array("server.key"))
