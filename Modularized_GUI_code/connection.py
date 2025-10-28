import socket
import threading

class ESP32Connection:
    def __init__(self, host="esp32.local", port=12345): #Can be changed to the actual IP address or hostname of the ESP32
        """
        Initializes the connection to the ESP32 over TCP socket.
        Args:
            host (str): Hostname or IP address of the ESP32.
            port (int): Port number to connect on to the ESP32.
        """
        self.host = host
        self.port = port
        self.sock = None  # Socket object for TCP connection
        self.lock = threading.Lock()  # Thread lock to ensure thread-safe access of the below functions
        # self.connect()  # Try to connect upon initialization

    def connect(self):
        """
        Establish a TCP connection to the ESP32.
        Closes any existing connection and creates a new one.
        Returns:
            bool: True if connection was successful, False otherwise.
        """
        with self.lock: #ensures mutual exclusion(only one thread can execute the critical section of code at a time.)
            try:
                # Close existing socket if already open
                if self.sock:
                    self.sock.close()

                # Create a new TCP socket
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5)  # Timeout for connection attempt
                self.sock.connect((self.host, self.port))  # Attempt to connect
                return True
            except Exception as e:
                print(f"Connection failed: {e}")
                self.sock = None
                return False

    def send_command(self, command, timeout=1.0):
        """
        Send a command string to the ESP32 and wait for a response.
        Args:
            command (str): The command to send (e.g., "START", "STOP").
            timeout (float): Timeout for waiting for the response.
        Returns:
            str: Response from the ESP32, or empty string on failure.
        """
        with self.lock:
            try:
                # Attempt to reconnect if not already connected
                if not self.sock:
                    if not self.connect():
                        return ""

                # Clear receive buffer before sending new command
                self.sock.settimeout(0.1)
                try:
                    while True:
                        data = self.sock.recv(1024)
                        if not data:
                            break
                except (socket.timeout, socket.error):
                    pass  # Ignore timeout errors during buffer clearing

                # Send the command and wait for response
                self.sock.settimeout(timeout)
                self.sock.sendall((command + "\n").encode())  # Send command with newline
                response = self.sock.recv(1024).decode().strip()  # Read response
                print(f"Sent: {command}, Received: {response}")
                return response
            except Exception as e:
                print(f"Command {command} failed: {e}")
                self.sock = None  # Invalidate socket on error
                return ""

    def recv_data(self, bufsize=1024, timeout=0.05):
        """
        Receive arbitrary data from the ESP32 (non-command streaming).
        Args:
            bufsize (int): Maximum number of bytes to receive.
            timeout (float): How long to wait for data before timing out.
        Returns:
            str: Received data as a decoded string, or empty string on error/timeout.
        """
        with self.lock:
            try:
                if not self.sock and not self.connect():
                    return ""

                self.sock.settimeout(timeout)
                data = self.sock.recv(bufsize)
                return data.decode(errors='ignore')  # Ignore decode errors (e.g., corrupted bytes)
            except socket.timeout:
                return ""  # Return empty string on timeout
            except Exception as e:
                print(f"[recv_data ERROR] {e}")
                return ""

    def close(self):
        """
        Cleanly close the connection to the ESP32.
        """
        with self.lock:
            if self.sock:
                self.sock.close()
                self.sock = None  # Release the socket object
