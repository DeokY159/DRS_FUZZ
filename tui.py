# core/tui.py
import os
import glob
import time
import datetime

from rich.live import Live
from rich.layout import Layout
from rich.panel import Panel
from rich.columns import Columns
from rich.console import Console
from rich.table import Table
from rich.text import Text

BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, 'output')
LOGS_DIR   = os.path.join(OUTPUT_DIR, 'logs')
STATE_LOG  = os.path.join(LOGS_DIR, 'current_state.log')


def tail(path: str, n: int = 10) -> str:
    """Return last n lines of a text file."""
    try:
        with open(path, 'r', errors='ignore') as f:
            lines = f.readlines()
        return ''.join(lines[-n:])
    except Exception:
        return ''


def hexdump(path: str, width: int = 16, max_lines: int = 4) -> str:
    """Return up to max_lines of hex+ASCII dump."""
    try:
        data = open(path, 'rb').read()
    except Exception:
        return ''
    lines = []
    for i in range(0, min(len(data), width * max_lines), width):
        chunk = data[i:i+width]
        hex_bytes = ' '.join(f"{b:02x}" for b in chunk)
        ascii_bytes = ''.join(chr(b) if 32 <= b < 127 else '·' for b in chunk)
        lines.append(f"{i:08x}  {hex_bytes:<{width*3}}  |{ascii_bytes}|")
    if len(data) > width * max_lines:
        lines.append("... (truncated) ...")
    return "\n".join(lines)


def parse_state_log():
    """Parse current_state.log into structured state info."""
    state = {
        'version': None,
        'robot': None,
        'topic': None,
        'strategy': None,
        'weights': {},
        'stage': 0,
        'round': 0,
        'crashes': 0
    }
    try:
        with open(STATE_LOG, 'r') as f:
            for line in f:
                _, _, msg = line.partition(' - ')
                msg = msg.strip()
                if msg.startswith('Initial state:'):
                    parts = msg[len('Initial state:'):].split(',')
                    for p in parts:
                        if '=' in p:
                            k, v = p.strip().split('=', 1)
                            state[k] = v
                elif msg.startswith('Mutation strategy:'):
                    strat_part, _, w_part = msg.partition(', weights:')
                    state['strategy'] = strat_part[len('Mutation strategy:'):].strip()
                    try:
                        state['weights'] = eval(w_part.strip())
                    except:
                        state['weights'] = {}
                elif msg.startswith('Stage '):
                    num = msg.split()[1]
                    if num.isdigit():
                        # 마지막으로 읽은 Stage 값을 그대로 사용
                        state['stage'] = int(num)
                elif msg.startswith('Round '):
                    num = msg.split()[1]
                    if num.isdigit():
                        # 마지막으로 읽은 Round 값을 그대로 사용
                        state['round'] = int(num)
                elif msg.startswith('Crash #'):
                    # Extract crash number correctly
                    # Format: "Crash #N detected; ..."
                    parts = msg.split()
                    if len(parts) >= 2 and parts[1].startswith('#'):
                        num = parts[1][1:]  # Remove '#' prefix
                        if num.isdigit():
                            state['crashes'] = int(num)
    except Exception:
        pass
    return state


def create_mutated_panels_layout(bins):
    """Create 2-column 5-row fixed-size layout for mutated bins."""
    if not bins:
        return Panel('— no mutated packets —', title='Mutated Packets', border_style='green')
    
    bins = bins[-10:]
    left_panels = []
    right_panels = []
    for i, path in enumerate(bins):
        title = os.path.basename(path)
        d = hexdump(path, max_lines=10)
        panel = Panel(Text(d or '— empty —'), title=title, border_style='green')
        if i % 2 == 0:
            left_panels.append(panel)
        else:
            right_panels.append(panel)
    while len(left_panels) < 5:
        left_panels.append(Panel('', title='', border_style='green'))
    while len(right_panels) < 5:
        right_panels.append(Panel('', title='', border_style='green'))

    mutated_layout = Layout()
    mutated_layout.split_row(
        Layout(name='left_col', minimum_size=25, size=83),
        Layout(name='right_col', minimum_size=25, size=83)
    )
    left_layout = Layout()
    left_layout.split_column(
        *(Layout(name=f'left_{i}', minimum_size=5, size=6) for i in range(5))
    )
    right_layout = Layout()
    right_layout.split_column(
        *(Layout(name=f'right_{i}', minimum_size=5, size=6) for i in range(5))
    )
    for i in range(5):
        left_layout[f'left_{i}'].update(left_panels[i])
        right_layout[f'right_{i}'].update(right_panels[i])
    mutated_layout['left_col'].update(left_layout)
    mutated_layout['right_col'].update(right_layout)
    return mutated_layout


def create_layout() -> Layout:
    layout = Layout()
    layout.split_row(
        Layout(name='left', minimum_size=130, size=130),
        Layout(name='right', minimum_size=170, size=170)
    )
    layout['left'].split_column(
        Layout(name='state', minimum_size=15, size=15),
        Layout(name='main', minimum_size=60, size=60)
    )
    layout['right'].split_column(
        Layout(name='mutated', minimum_size=30, size=30),
        Layout(name='containers', minimum_size=30, size=45)
    )

    info = parse_state_log()
    st = Table.grid(padding=(0,1))
    st.add_column(justify='right', style='cyan', no_wrap=True, width=12)
    st.add_column(style='white', no_wrap=True, max_width=30)

    def truncate(text, max_len=50):
        return text[:max_len] + '...' if len(text) > max_len else text

    st.add_row('Version:', truncate(info.get('version') or 'N/A'))
    st.add_row('Robot:', truncate(info.get('robot') or 'N/A'))
    st.add_row('Topic:', truncate(info.get('topic') or 'N/A'))
    st.add_row('Strategy:', truncate(info.get('strategy') or 'N/A'))
    for fn, w in info.get('weights', {}).items():
        st.add_row(f'  {fn}:', str(w))
    st.add_row('Stage:', str(info.get('stage', 0)))
    st.add_row('Round:', str(info.get('round', 0)))
    st.add_row('Crashes:', str(info.get('crashes', 0)))
    layout['left']['state'].update(Panel(st, title='Fuzzing State', border_style='cyan'))

    main_txt = tail(os.path.join(LOGS_DIR, 'main.log'), 60) or '— no main log —'
    layout['left']['main'].update(
        Panel(Text.from_ansi(main_txt), title='Main Log', border_style='cyan')
    )

    bins = sorted(glob.glob(os.path.join(LOGS_DIR, 'mutated_*.bin')))
    mutated_layout = create_mutated_panels_layout(bins)
    layout['right']['mutated'].update(mutated_layout)

    log_panels = []
    log_files = sorted(glob.glob(os.path.join(LOGS_DIR, '*.log')))
    for path in log_files[:3]:
        name = os.path.basename(path)
        if name in ('main.log', 'current_state.log'):
            continue
        raw = tail(path, 42) or '— empty —'
        # interpret ANSI color codes
        panel_text = Text.from_ansi(raw)
        log_panels.append(
            Panel(panel_text, title=name, border_style='magenta')
        )
    if log_panels:
        layout['right']['containers'].update(
            Columns(log_panels, equal=True, expand=False, width=81)
        )
    else:
        layout['right']['containers'].update(
            Panel('— no containers —', title='Containers', border_style='magenta')
        )
    return layout


def main():
    console = Console()
    console.print('[bold]Starting TUI (fixed size)... Ctrl+C to exit[/]')
    time.sleep(1)
    try:
        with Live(create_layout(), refresh_per_second=1, screen=True, console=console,
                  vertical_overflow="crop") as live:
            while True:
                live.update(create_layout())
                time.sleep(1)
    except KeyboardInterrupt:
        console.print('\n[red]Exiting TUI[/red]')


if __name__ == '__main__':
    main()