"""Full bridge/PTY regression. Run only in a disposable --network=none container.

It creates a simulated WFB service on the production loopback port. Never run
this on a live controller. TOTA_ISOLATED_TEST=1 is an explicit execution guard.
"""
import json
import os
import pathlib
import pty
import select
import socket
import subprocess
import tempfile
import threading
import time
import unittest


@unittest.skipUnless(os.environ.get('TOTA_ISOLATED_TEST') == '1', 'requires isolated network namespace')
class SerialCutoffTest(unittest.TestCase):
    def test_real_process_starts_silent_and_stops_every_serial_byte(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        os.set_blocking(master, False)
        with tempfile.TemporaryDirectory() as root:
            pathlib.Path(root, 'config.yaml').write_text(
                'serial:\n  tx_port_name: ' + os.ttyname(slave) + '\n  tx_baud_rate: 400000\n')
            listener = socket.socket()
            self.addCleanup(listener.close)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(('127.0.0.1', 8103))
            listener.listen()
            listener.settimeout(1)
            stop = threading.Event()
            healthy = threading.Event()

            def stats():
                total = 0
                while not stop.is_set():
                    try:
                        conn, _ = listener.accept()
                    except socket.timeout:
                        continue
                    with conn:
                        while not stop.is_set():
                            if healthy.is_set():
                                total += 10
                                message = dict(type='rx', id='video rx', timestamp=time.time(),
                                               session={'epoch': 0}, packets={'dec_ok': [10, total]})
                                try:
                                    conn.sendall((json.dumps(message) + '\n').encode())
                                except (BrokenPipeError, ConnectionResetError):
                                    break
                            stop.wait(.05)

            thread = threading.Thread(target=stats, daemon=True)
            thread.start()

            def capture(seconds):
                deadline = time.monotonic() + seconds
                data = bytearray()
                while time.monotonic() < deadline:
                    ready, _, _ = select.select([master], [], [], min(.05, max(0, deadline-time.monotonic())))
                    if ready:
                        data.extend(os.read(master, 65536))
                return data

            with open(pathlib.Path(root, 'bridge.log'), 'w+') as log:
                process = subprocess.Popen([os.environ['TOTA_TEST_BRIDGE']], cwd=root, stdout=log, stderr=log)
                try:
                    self.assertEqual(capture(.5), b'', 'serial bytes at startup without VTX')
                    healthy.set()
                    self.assertGreater(len(capture(2)), 0, 'no serial output with healthy video')
                    healthy.clear()
                    capture(3.2)  # consume the grace interval and already-written bytes
                    self.assertEqual(capture(.6), b'', 'serial traffic continued after cutoff')
                    process.wait(timeout=2)
                    self.assertEqual(process.returncode, 0)
                    # Simulate the supervisor restart: still silent, then fresh
                    # video permits initialization again (no old channel cache).
                    process = subprocess.Popen([os.environ['TOTA_TEST_BRIDGE']], cwd=root, stdout=log, stderr=log)
                    self.assertEqual(capture(.5), b'', 'restart bypassed missing-video gate')
                    healthy.set()
                    self.assertGreater(len(capture(1.5)), 0, 'recovery did not initialize serial')
                finally:
                    process.terminate()
                    process.wait(timeout=3)
                    stop.set()
                    thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
