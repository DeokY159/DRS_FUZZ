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
    state = {'version': None, 'robot': None, 'topic': None,
             'strategy': None, 'weights': {}, 'runs': 0, 'crashes': 0}
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
                elif msg.startswith('Run #'):
                    num = msg.split(':')[0].lstrip('Run #')
                    state['runs'] = max(state['runs'], int(num)) if num.isdigit() else state['runs']
                elif msg.startswith('Crash #'):
                    num = msg.split()[0].lstrip('Crash #')
                    state['crashes'] = max(state['crashes'], int(num)) if num.isdigit() else state['crashes']
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
        Layout(name='left_0', minimum_size=5, size=6),
        Layout(name='left_1', minimum_size=5, size=6),
        Layout(name='left_2', minimum_size=5, size=6),
        Layout(name='left_3', minimum_size=5, size=6),
        Layout(name='left_4', minimum_size=5, size=6)
    )

    right_layout = Layout()
    right_layout.split_column(
        Layout(name='right_0', minimum_size=5, size=6),
        Layout(name='right_1', minimum_size=5, size=6),
        Layout(name='right_2', minimum_size=5, size=6),
        Layout(name='right_3', minimum_size=5, size=6),
        Layout(name='right_4', minimum_size=5, size=6)
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
    for fn, w in list(info.get('weights', {}).items()):
        st.add_row(f'  {fn}:', str(w))
    st.add_row('Runs:', str(info.get('runs', 0)))
    st.add_row('Crashes:', str(info.get('crashes', 0)))
    layout['left']['state'].update(Panel(st, title='Fuzzing State', border_style='cyan'))

    main_txt = tail(os.path.join(LOGS_DIR, 'main.log'), 60) or '— no main log —'
    main_lines = []
    for line in main_txt.split('\n')[:60]:
        if len(line) > 120:
            line = line[:117] + '...'
        main_lines.append(line)
    main_txt = '\n'.join(main_lines)
    layout['left']['main'].update(Panel(Text(main_txt), title='Main Log', border_style='cyan'))

    bins = sorted(glob.glob(os.path.join(LOGS_DIR, 'mutated_*.bin')))
    mutated_layout = create_mutated_panels_layout(bins)
    layout['right']['mutated'].update(mutated_layout)

    log_panels = []
    log_files = sorted(glob.glob(os.path.join(LOGS_DIR, '*.log')))
    for path in log_files[:3]:
        name = os.path.basename(path)
        if name in ('main.log', 'current_state.log'): 
            continue
        c = tail(path, 42) or '— empty —'
        c_lines = []
        for line in c.split('\n')[:42]:
            if len(line) > 78:
                line = line[:74] + '...'
            c_lines.append(line)
        c = '\n'.join(c_lines)
        
        short_name = name
        log_panels.append(Panel(Text(c), title=short_name, border_style='magenta'))
    
    if log_panels:
        layout['right']['containers'].update(Columns(log_panels, equal=True, expand=False, width=81))
    else:
        layout['right']['containers'].update(Panel('— no containers —', title='Containers', border_style='magenta'))

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