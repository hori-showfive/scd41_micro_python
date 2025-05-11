import machine
import time
import struct

class SCD41:
    """
    MicroPython driver for the Sensirion SCD41 CO2, temperature, and humidity sensor.
    """
    DEFAULT_I2C_ADDRESS = 0x62

    # SCD41 Commands
    CMD_START_PERIODIC_MEASUREMENT = b'\x21\xb1'
    CMD_READ_MEASUREMENT = b'\xec\x05'  # Max execution time: 1 ms
    CMD_STOP_PERIODIC_MEASUREMENT = b'\x3f\x86' # Max execution time: 500 ms
    CMD_GET_DATA_READY_STATUS = b'\xe4\xb8' # Max execution time: 1 ms
    CMD_GET_SERIAL_NUMBER = b'\x36\x82'     # Max execution time: 1 ms

    # CRC8 parameters
    CRC8_POLYNOMIAL = 0x31
    CRC8_INIT = 0xFF

    def __init__(self, i2c, address=DEFAULT_I2C_ADDRESS):
        self.i2c = i2c
        self.address = address
        
        # Buffers for I2C communication, initialized to a non-zero pattern
        self._read_buffer = bytearray([0xFF] * 9) # Initialize with 0xFF
        self._cmd_buffer = bytearray(2)

        print("DEBUG: SCD41.__init__ started.")
        # Wait for sensor startup (SCD41 datasheet: 1000 ms after power-up/reset)
        print("DEBUG: Waiting for SCD41 initial startup (1000 ms)...")
        time.sleep_ms(1000) 
        print("DEBUG: SCD41 initial startup wait complete.")

        # Check if sensor is present by trying to communicate
        # This scan is after the initial delay, main() also scans before this.
        devices = self.i2c.scan()
        if self.address not in devices:
            print(f"DEBUG: SCD41 not found at 0x{self.address:02x} after initial delay. Found: {[hex(d) for d in devices]}")
            raise OSError(f"SCD41 not found at I2C address 0x{self.address:02x}.")
        print(f"DEBUG: SCD41 confirmed at 0x{self.address:02x} by scan after initial delay.")

        # Sensor setup sequence based on C code structure and datasheet timings
        print("DEBUG: Sending stop_periodic_measurement...")
        self.stop_periodic_measurement() # Command itself is quick to send
        print("DEBUG: Waiting 500ms for stop_periodic_measurement to complete (datasheet max time).")
        time.sleep_ms(500)  # Max execution time for stop_periodic_measurement
        print("DEBUG: 500ms wait after stop_periodic_measurement complete.")

        print("DEBUG: Attempting to get serial number...")
        serial_info = self.get_serial_number() # Contains 1ms internal delay for command processing before read
        if serial_info:
            # serial_info could be the words or None if CRC failed.
            print(f"DEBUG: Serial number info (processed): {serial_info}")
        else:
            print("DEBUG: Failed to get serial number or CRC error during init.")
        
        # C code has a 500ms delay after get_serial_code()
        print("DEBUG: Waiting 500ms after get_serial_number (mimicking C code structure for sensor settling)...")
        time.sleep_ms(500) 
        print("DEBUG: 500ms wait after get_serial_number complete.")

        print("DEBUG: Sending start_periodic_measurement...")
        self.start_periodic_measurement() # Command itself is quick to send
        # C code has a 500ms delay after start_periodic_measurements()
        # This might allow the sensor to stabilize after starting measurements.
        # First data is available after 5s, but command itself is fast.
        print("DEBUG: Waiting 500ms after start_periodic_measurement (mimicking C code structure for sensor settling)...")
        time.sleep_ms(500)
        print("DEBUG: 500ms wait after start_periodic_measurement complete.")
        
        print("DEBUG: SCD41.__init__ finished.\n\n")

    def _sensirion_crc8(self, data_bytes):
        crc = self.CRC8_INIT
        for byte in data_bytes:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ self.CRC8_POLYNOMIAL
                else:
                    crc = crc << 1
                crc &= 0xFF # Ensure crc remains a byte
        return crc

    def _send_command(self, command_bytes, delay_ms_after_cmd):
        self.i2c.writeto(self.address, command_bytes)
        if delay_ms_after_cmd > 0:
            time.sleep_ms(delay_ms_after_cmd)

    def _read_data(self, num_bytes):
        # Ensure we are reading into the correct portion of the buffer
        read_slice = self._read_buffer[:num_bytes]
        self.i2c.readfrom_into(self.address, read_slice)
        return read_slice

    def _execute_command_read_response(self, command_bytes, response_length, cmd_processing_time_ms):
        # Send the command, ensuring no STOP condition is sent to allow for a REPEATED START.
        # Changed stop=False to positional False
        self.i2c.writeto(self.address, command_bytes, False) 
        
        # Wait for the sensor to process the command and prepare data.
        if cmd_processing_time_ms > 0:
            time.sleep_ms(cmd_processing_time_ms)
        
        # Read the response data. This will issue a REPEATED START.
        # Use a slice of the buffer for reading.
        read_buf_slice = self._read_buffer[:response_length]
        # The readfrom_into call uses the default stop=True, which is appropriate here.
        self.i2c.readfrom_into(self.address, read_buf_slice) 
        return read_buf_slice

    def start_periodic_measurement(self):
        self._send_command(self.CMD_START_PERIODIC_MEASUREMENT, 0)

    def stop_periodic_measurement(self):
        self._send_command(self.CMD_STOP_PERIODIC_MEASUREMENT, 0)

    def get_data_ready_status(self):
        """Checks if new measurement data is available."""
        response_bytes = self._execute_command_read_response(self.CMD_GET_DATA_READY_STATUS, 3, 1)
        
        data_for_crc = response_bytes[0:2]
        calculated_crc = self._sensirion_crc8(data_for_crc)
        received_crc = response_bytes[2]
        
        # print(f"DEBUG: get_data_ready_status:")
        # print(f"  Raw response: {[hex(b) for b in response_bytes]}")
        # print(f"  Data for CRC: {[hex(b) for b in data_for_crc]}") # Explicitly show data used for CRC
        # print(f"  Calculated CRC: {hex(calculated_crc)}, Received CRC: {hex(received_crc)}")

        if calculated_crc != received_crc:
            # print("  DEBUG: Data ready status: CRC error") # Indented for clarity
            return False
        
        status_word = (response_bytes[0] << 8) | response_bytes[1]
        is_ready = (status_word & 0x07FF) != 0
        # print(f"  DEBUG: Status word: {hex(status_word)}, Data ready: {is_ready}") # Indented
        return is_ready

    def read_measurement(self):
        """Reads CO2, temperature, and humidity data."""
        response = self._execute_command_read_response(self.CMD_READ_MEASUREMENT, 9, 1)

        # Verify CRC for CO2, temperature, and humidity data
        if self._sensirion_crc8(response[0:2]) != response[2]:
            # print("CO2 data CRC error")
            return None
        if self._sensirion_crc8(response[3:5]) != response[5]:
            # print("Temperature data CRC error")
            return None
        if self._sensirion_crc8(response[6:8]) != response[8]:
            # print("Humidity data CRC error")
            return None

        raw_co2 = (response[0] << 8) | response[1]
        raw_temp = (response[3] << 8) | response[4]
        raw_humidity = (response[6] << 8) | response[7]

        co2 = raw_co2
        temperature = -45.0 + 175.0 * (raw_temp / 65535.0)
        humidity = 100.0 * (raw_humidity / 65535.0)
        
        return co2, temperature, humidity

    def get_serial_number(self):
        """Reads the sensor's serial number."""
        response_bytes = self._execute_command_read_response(self.CMD_GET_SERIAL_NUMBER, 9, 1)
        
        # print(f"DEBUG: get_serial_number:")
        # print(f"  Raw response: {[hex(b) for b in response_bytes]}")

        serial_words = []
        valid_crc_overall = True
        for i in range(3):
            word_bytes = response_bytes[i*3 : i*3+2]
            crc_byte = response_bytes[i*3+2]
            
            calculated_crc = self._sensirion_crc8(word_bytes)
            print(f"  Word {i+1}: Data={[hex(b) for b in word_bytes]}, Calc_CRC={hex(calculated_crc)}, Recv_CRC={hex(crc_byte)}")

            if calculated_crc != crc_byte:
                # print(f"  DEBUG: Serial number word {i+1} CRC error")
                valid_crc_overall = False
                # We can choose to break here or collect all words and report overall failure
        
        if not valid_crc_overall:
            return None

        # Re-extract words if all CRCs were okay
        for i in range(3):
            word_bytes = response_bytes[i*3 : i*3+2]
            serial_words.append((word_bytes[0] << 8) | word_bytes[1])
        
        return serial_words

# --- Main program ---
def main():
    # Configure I2C pins according to your ESP32 board's wiring.
    # Common ESP32 I2C0 pins: SCL=GPIO22, SDA=GPIO21
    # The C code used SCL=21, SDA=19. This might be for I2C1 or custom pins.
    # Ensure you use the correct I2C bus ID (0 or 1 for ESP32) and pin numbers.
    SCL_PIN_NUM = 21  # GPIO number for SCL
    SDA_PIN_NUM = 19  # GPIO number for SDA
    I2C_BUS_ID = 0    # Or 1, depending on ESP32 I2C peripheral used

    # print(f"Initializing I2C on bus {I2C_BUS_ID} with SCL={SCL_PIN_NUM}, SDA={SDA_PIN_NUM}")
    
    try:
        i2c = machine.I2C(I2C_BUS_ID, scl=machine.Pin(SCL_PIN_NUM), sda=machine.Pin(SDA_PIN_NUM), freq=100000)
    except ValueError as e:
        # print(f"Error initializing I2C. Check pins or I2C bus ID. {e}")
        # print("If using default ESP32 I2C0 pins, try SCL=22, SDA=21.")
        return
    except Exception as e:
        # print(f"An unexpected error occurred during I2C initialization: {e}")
        return

    # print("I2C initialized. Scanning for devices...")
    devices = i2c.scan()
    if not devices:
        # print("No I2C devices found. Check wiring.")
        return
    # print("Found I2C devices:", [hex(device) for device in devices])

    if SCD41.DEFAULT_I2C_ADDRESS not in devices:
        # print(f"SCD41 sensor not found at address {hex(SCD41.DEFAULT_I2C_ADDRESS)}.")
        return

    try:
        scd = SCD41(i2c)
        # print("SCD41 sensor initialized successfully.")
        # Optional: Display serial number
        # serial_num = scd.get_serial_number()
        # if serial_num:
        #     print(f"SCD41 Serial Number: {serial_num[0]:04X}{serial_num[1]:04X}{serial_num[2]:04X}")

    except OSError as e:
        # print(f"Error initializing SCD41 sensor: {e}")
        return
    except Exception as e:
        # print(f"An unexpected error occurred during SCD41 initialization: {e}")
        return

    # print("Starting periodic data acquisition from SCD41...")
    # SCD41 typically has a 5-second measurement interval.
    # We will poll every 5 seconds.
    POLL_INTERVAL_SECONDS = 5 

    while True:
        try:
            # print(f"Waiting {POLL_INTERVAL_SECONDS} seconds before next read attempt...")
            time.sleep(POLL_INTERVAL_SECONDS) # Wait first, then check status

            # print("Checking data ready status...")
            if scd.get_data_ready_status():
                # print("Data is ready. Reading measurement...")
                measurement = scd.read_measurement()
                if measurement:
                    co2, temp, hum = measurement
                    print(f"CO2: {co2} ppm, Temperature: {temp:.2f} C, Humidity: {hum:.2f} %RH")
                else:
                    # print("Failed to read measurement or CRC error in measurement data.")
                    pass
            else:
                # This case might be less frequent if POLL_INTERVAL_SECONDS is >= 5
                # print("Data not ready yet. Sensor might still be processing or an issue occurred.")
                pass
            
        except OSError as e:
            # print(f"I2C communication error: {e}. Retrying after a delay...")
            time.sleep(5) # Wait before retrying

        except Exception as e:
            # print(f"An unexpected error occurred in the main loop: {e}")
            # print("Attempting to continue after a delay...")
            time.sleep(5)
            

if __name__ == "__main__":
    main()