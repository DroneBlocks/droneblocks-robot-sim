import msvcrt
import time

def non_blocking_key_reader():
    arrow_key_map = {
        b'\xe0H': '\x1b[A',  # Up arrow
        b'\xe0P': '\x1b[B',  # Down arrow
        b'\xe0K': '\x1b[D',  # Left arrow
        b'\xe0M': '\x1b[C',  # Right arrow
    }

    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in [b'\x00', b'\xe0']:
                ch += msvcrt.getch()
                mapped_key = arrow_key_map.get(ch)
                if mapped_key:
                    yield mapped_key
                else:
                    # Yield the raw sequence if it's not an arrow key
                    yield ch.decode('utf-8', errors='ignore')
            else:
                try:
                    yield ch.decode('utf-8')
                except UnicodeDecodeError:
                    yield ch.decode('utf-8', errors='ignore')
        else:
            yield None

        time.sleep(0.001)