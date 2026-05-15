# connection.py — TCP connection to ESP32 + auto-discovery
import socket
import threading
import concurrent.futures


# ── Auto-discovery ────────────────────────────────────────────────────────────

def _try_connect_ip(ip: str, port: int, timeout: float):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((ip, port))
        s.close()
        return ip
    except Exception:
        return None


def _resolve_mdns(hostname: str, port: int, timeout: float):
    try:
        infos = socket.getaddrinfo(
            hostname, port,
            socket.AF_INET, socket.SOCK_STREAM,
            0, socket.AI_ADDRCONFIG)
        ip = infos[0][4][0]
        return _try_connect_ip(ip, port, timeout)
    except Exception:
        return None


def _scan_subnet(port: int, connect_timeout: float = 0.4,
                 max_workers: int = 150):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception:
        return None

    prefix = ".".join(local_ip.split(".")[:3])
    ips    = [f"{prefix}.{i}" for i in range(1, 255)]

    found = None
    lock  = threading.Lock()
    stop  = threading.Event()

    def try_ip(ip):
        nonlocal found
        if stop.is_set():
            return
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(connect_timeout)
            s.connect((ip, port))
            s.close()
            with lock:
                if found is None:
                    found = ip
            stop.set()
        except Exception:
            pass

    with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as ex:
        ex.map(try_ip, ips)

    return found


def discover_esp32(port: int = 12345,
                   hostname: str = "esp32.local",
                   timeout: float = 5.0,
                   progress_cb=None):
    """
    Find the ESP32 on the local network using two parallel strategies:
      1. OS mDNS resolution of esp32.local
      2. /24 subnet scan for port 12345
    Returns IP string or None.
    """
    def cb(msg):
        if progress_cb:
            progress_cb(msg)

    result   = [None]
    found_ev = threading.Event()

    def run_mdns():
        cb("Trying mDNS (esp32.local)…")
        ip = _resolve_mdns(hostname, port, timeout=2.0)
        if ip and not found_ev.is_set():
            result[0] = ip
            found_ev.set()
            cb(f"Found via mDNS: {ip}")

    def run_scan():
        cb("Scanning network for ESP32…")
        ip = _scan_subnet(port, connect_timeout=0.4)
        if ip and not found_ev.is_set():
            result[0] = ip
            found_ev.set()
            cb(f"Found via scan: {ip}")

    for t in [threading.Thread(target=run_mdns, daemon=True),
              threading.Thread(target=run_scan,  daemon=True)]:
        t.start()

    found_ev.wait(timeout=timeout)
    return result[0]


# ── ESP32Connection ───────────────────────────────────────────────────────────

class ESP32Connection:
    """Thread-safe TCP connection to the ESP32."""

    def __init__(self, host: str = "esp32.local", port: int = 12345):
        self.host  = host
        self.port  = port
        self.sock  = None
        self._lock = threading.Lock()

    def connect(self) -> bool:
        with self._lock:
            try:
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
                print(f"[Connection] connecting to {self.host}:{self.port} …")
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(5)
                try:
                    s.connect((self.host, self.port))
                except socket.gaierror:
                    print(f"[Connection] hostname '{self.host}' not resolved. "
                          f"Use Auto Connect.")
                    s.close()
                    self.sock = None
                    return False
                s.settimeout(None)
                self.sock = s
                print(f"[Connection] connected to {self.host}:{self.port}")
                return True
            except Exception as e:
                print(f"[Connection] connect failed: {e}")
                self.sock = None
                return False

    def connect_to_ip(self, ip: str, port: int = None) -> bool:
        self.host = ip
        if port is not None:
            self.port = port
        return self.connect()

    def send_command(self, command: str, timeout: float = 2.0) -> str:
        with self._lock:
            try:
                if not self.sock:
                    if not self._connect_unlocked():
                        return ""
                self.sock.settimeout(0.05)
                try:
                    while self.sock.recv(4096):
                        pass
                except (socket.timeout, OSError):
                    pass
                self.sock.settimeout(timeout)
                self.sock.sendall((command + "\n").encode())
                response = self._readline_unlocked(timeout)
                print(f"[CMD] {command!r} → {response!r}")
                return response
            except Exception as e:
                print(f"[Connection] send_command '{command}' failed: {e}")
                self._close_unlocked()
                return ""

    def recv_data(self, bufsize: int = 4096, timeout: float = 0.05) -> str:
        with self._lock:
            try:
                if not self.sock:
                    return ""
                self.sock.settimeout(timeout)
                data = self.sock.recv(bufsize)
                return data.decode(errors="ignore") if data else ""
            except socket.timeout:
                return ""
            except Exception as e:
                print(f"[Connection] recv_data error: {e}")
                self._close_unlocked()
                return ""

    def is_connected(self) -> bool:
        return self.sock is not None

    def close(self):
        with self._lock:
            self._close_unlocked()

    def _connect_unlocked(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self.sock = s
            return True
        except Exception as e:
            print(f"[Connection] auto-reconnect failed: {e}")
            self.sock = None
            return False

    def _close_unlocked(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _readline_unlocked(self, timeout: float) -> str:
        buf = b""
        self.sock.settimeout(timeout)
        while True:
            ch = self.sock.recv(1)
            if not ch:
                break
            buf += ch
            if ch == b"\n":
                break
        return buf.decode(errors="ignore").strip()
