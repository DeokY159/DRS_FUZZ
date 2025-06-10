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

# ----- Helpers -----
def tail(path: str, n: int = 10) -> str:
    try:
        with open(path, 'r', errors='ignore') as f:
            lines = f.readlines()
        return ''.join(lines[-n:])
    except Exception:
        return ''

def hexdump(path: str, width: int = 16, max_lines: int = 4) -> str:
    try:
        data = open(path, 'rb').read()
    except Exception:
        return ''
    lines = []
    for i in range(0, min(len(data), width * max_lines), width):
        chunk = data[i:i+width]
        hex_bytes = ' '.join(f"{b:02x}" for b in chunk)
        lines.append(f"{i:08x}  {hex_bytes:<{width*3}}  ")
    if len(data) > width * max_lines:
        lines.append("... (truncated) ...")
    return "\n".join(lines)

def get_start_time():
    try:
        with open(STATE_LOG, 'r', encoding='utf-8', errors='ignore') as f:
            first = f.readline()
            if not first:
                return None
            timestamp = first.split(' - ')[0]
            return datetime.datetime.fromisoformat(timestamp)
    except Exception:
        return None

def format_elapsed_time(start_time, now=None):
    if not start_time:
        return 'N/A'
    now = now or datetime.datetime.now()
    delta = now - start_time
    hours, remainder = divmod(delta.seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    days = delta.days
    if days > 0:
        return f"{days}d {hours:02}:{minutes:02}:{seconds:02}"
    return f"{hours:02}:{minutes:02}:{seconds:02}"

# ----- Log Parsing ----
def parse_state_log():
    state = {
        'version': None,
        'robot': None,
        'topic': None,
        'headless': False,
        'asan': False,
        'stage': 0,
        'round': 0,
        'crashes': 0,
        'bug': 0,
        'error': 0,
        'strategy': None,
        'weights': {},
        'qos_strategy': None,
        'qos_weights': {},
        'config': {},
        'qos_profile': {},
    }
    try:
        with open(STATE_LOG, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                _, _, msg = line.partition(' - ')
                msg = msg.strip()
                if msg.startswith('Initial state:'):
                    parts = msg[len('Initial state:'):].split(',')
                    for p in parts:
                        if '=' in p:
                            k, v = p.strip().split('=', 1)
                            if k in state:
                                state[k] = v
                            elif k == "headless":
                                state['headless'] = (v.lower() == 'true')
                            elif k == "asan":
                                state['asan'] = (v.lower() == 'true')
                elif msg.startswith('Initial Config:'):
                    config_part = msg[len('Initial Config:'):].strip()
                    for pair in config_part.split(','):
                        if '=' in pair:
                            k, v = pair.strip().split('=', 1)
                            state['config'][k.strip()] = v.strip()
                elif msg.startswith('QoS Setting'):
                    profile_part = msg[len('QoS Setting'):].strip()
                    for pair in profile_part.split(','):
                        if '=' in pair:
                            k, v = pair.strip().split('=', 1)
                            state['qos_profile'][k.strip()] = v.strip()
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
                        state['stage'] = int(num)
                elif msg.startswith('Round '):
                    num = msg.split()[1]
                    if num.isdigit():
                        state['round'] = int(num)
                elif msg.startswith('Crash #'):
                    parts = msg.split()
                    if len(parts) >= 2 and parts[1].startswith('#'):
                        num = parts[1][1:]
                        if num.isdigit():
                            state['crashes'] = int(num)
                elif msg.startswith('Semantic Bug #'):
                    parts = msg.split()
                    if len(parts) >= 2 and parts[2].startswith('#'):
                        num = parts[2][1:]
                        if num.isdigit():
                            state['bug'] = int(num)
                elif msg.startswith('Error #'):
                    parts = msg.split()
                    if len(parts) >= 2 and parts[1].startswith('#'):
                        num = parts[1][1:]
                        if num.isdigit():
                            state['error'] = int(num)
                elif 'QoS settings updated' in msg:
                    state['qos_strategy'] = f"Stage {state['stage']} updated"
    except Exception:
        pass
    return state

def get_mutated_packet_paths():
    bins = sorted(glob.glob(os.path.join(LOGS_DIR, 'mutated_*.bin')))
    return bins if len(bins) >= 10 else bins


def get_latest_seed_selected():
    try:
        with open(STATE_LOG, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        for line in reversed(lines):
            if 'Seed Selected:' in line:
                part = line.split('Seed Selected:')[-1].strip()
                return part
    except Exception:
        pass
    return None

# ----- Panels -----
def build_settings_panel(state):
    t = Table.grid(padding=(0,1))
    t.add_column(justify="right", style="cyan", width=20)
    t.add_column()
    config = state.get('config', {})
    t.add_row("")
    for k in ["RETRY_MAX_ATTEMPTS", "RETRY_DELAY", "RUN_DELAY", "PACKETS_PER_QOS", "MESSAGES_PER_RUN", "MESSAGE_PERIOD", "UDP_SPORT"]:
        v = config.get(k, None)
        
        if v is not None:
            t.add_row(k + ":", str(v))
    return Panel(t, title="Settings", border_style="cyan")

def build_process_time_panel(state, start_time):
    t = Table.grid(padding=(0,1))
    t.add_column(justify="right", style="cyan", width=20)
    t.add_column()
    t.add_row("Elapsed Time", format_elapsed_time(start_time))
    t.add_row("Total Stage", str(state.get('stage', 0)))
    t.add_row("Total Round", str(state.get('round', 0)))
    return Panel(t, title="Process", border_style="cyan")

def build_header_panel(state):
    txt = f"DRS FUZZ ver.1.0 - {state.get('version','N/A')} {state.get('robot','N/A')}"
    if state.get('headless'): txt += " :headless"
    if state.get('asan'):     txt += " :asan"
    return Panel(Text(txt, style="bold yellow", justify="center"), style="on black", padding=(0,1))

def build_overall_results_panel(state):
    t = Table.grid(padding=(0,1))
    t.add_column(justify="right", style="magenta", width=20)
    t.add_column()
    t.add_row("Crash found", str(state.get('crashes', 0)))
    t.add_row("Bug found", str(state.get('bug', 0)))
    t.add_row("Error occured", str(state.get('error', 0)))
    return Panel(t, title="Overall Results", border_style="magenta")

def build_strategy_panel(state):
    t = Table.grid(padding=(0,1))
    t.add_column(justify="right", style="green", width=20)
    t.add_column()
    seed = get_latest_seed_selected()
    if seed:
        t.add_row("Seed Selected", seed)
    strat = state.get('strategy', None)
    if strat:
        t.add_row("Mutation Strategy", strat)
    profile = state.get('qos_profile', {})
    if profile:
        for k in ["durability", "history", "depth", "liveliness"]:
            v = profile.get(k, None)
            if v is not None:
                t.add_row("  " + k.capitalize(), str(v))
    return Panel(t, title="Current Packet Mutation / QoS Info", border_style="green")

def build_strategy_weights_panel(state):
    t = Table.grid(padding=(0,1))
    t.add_column(justify="right", style="yellow", width=20)
    t.add_column()
    weights = state.get('weights', {})
    if isinstance(weights, dict):
        for k, v in weights.items():
            t.add_row(f" {k}:", str(v))
    return Panel(t, title="Mutate Strategy Weights", border_style="yellow")


def build_main_log_panel():
    main_log_path = os.path.join(LOGS_DIR, 'main.log')
    main_txt = tail(main_log_path, 40) or '— no main log —'
    return Panel(Text.from_ansi(main_txt), title='Main Log', border_style='blue')

def build_mutated_packets_panel(paths):
    if not paths:
        return Panel("— No mutated packets —", title="Mutated Packets", border_style="blue")
    panels = []
    for i, path in enumerate(paths):
        title = os.path.basename(path)
        d = hexdump(path, max_lines=8)
        panels.append(Panel(Text(d or '— empty —', style="white"), title=title, border_style="blue"))
    while len(panels) < 10:
        panels.append(Panel('', title='', border_style="blue"))
    grid = Columns([Columns(panels[:2], equal=True, expand=True),
                    Columns(panels[2:], equal=True, expand=True)], 
                   equal=True, expand=True)
    return Panel(grid, title="Mutated Packets", border_style="blue")

# ----- Layout -----
def create_layout():
    state = parse_state_log()
    mutated_packet_paths = get_mutated_packet_paths()
    start_time = get_start_time()
    layout = Layout()
    layout.split_column(
        Layout(name="header", size=3),
        Layout(name="body", ratio=1),
    )
    layout["body"].split_row(
        Layout(name="left", ratio=1),
        Layout(name="right", ratio=1),
    )
    # left: process, results, strategy, mutated
    layout["left"].split_column(
        Layout(name="proc", size=5),           # Process
        Layout(name="results", size=5),        # Overall Results
        Layout(name="strategy", size=8),       # Current Strategy
        Layout(name="mainlog", ratio=1)        # Main Log
    )
    # right: settings, weights, mutated
    layout["right"].split_column(
        Layout(name="settings", size=10),      # Settings 
        Layout(name="weights", size=8),        # Strategy Weights
        Layout(name="mutated", ratio=1)        # Mutated Packets
    )
    layout["header"].update(build_header_panel(state))
    layout["left"]["proc"].update(build_process_time_panel(state, start_time))
    layout["left"]["results"].update(build_overall_results_panel(state))
    layout["left"]["strategy"].update(build_strategy_panel(state))
    mainlog_panel = build_main_log_panel()
    layout["left"]["mainlog"].update(mainlog_panel)
    # right-up: settings
    layout["right"]["settings"].update(build_settings_panel(state))
    layout["right"]["weights"].update(build_strategy_weights_panel(state))
    mutated_panel = build_mutated_packets_panel(mutated_packet_paths)
    layout["right"]["mutated"].update(mutated_panel)
    return layout

def main():
    console = Console()
    console.print('[bold]Starting DRS FUZZ TUI... Ctrl+C to exit[/]')
    time.sleep(1)
    try:
        with Live(create_layout(), refresh_per_second=1, screen=True, console=console,
                  vertical_overflow="crop") as live:
            while True:
                live.update(create_layout())
                time.sleep(1)
    except KeyboardInterrupt:
        console.print('\n[red]Exiting DRS FUZZ TUI[/red]')

if __name__ == '__main__':
    main()
