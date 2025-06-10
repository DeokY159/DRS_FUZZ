# core/ui.py
import os
import logging

logs_dir = os.path.join(os.getcwd(), 'output', 'logs')
os.makedirs(logs_dir, exist_ok=True)

# --- 1) Define ColorFormatter so both console and file handlers can use it ---
class ColorFormatter(logging.Formatter):
    LEVEL_COLOR = {
        'INFO':    '\033[94m',
        'WARNING': '\033[93m',
        'ERROR':   '\033[91m',
        'DEBUG':   '\033[95m',
        'DONE':    '\033[92m',
    }
    RESET = '\033[0m'

    def format(self, record):
        color = self.LEVEL_COLOR.get(record.levelname, '')
        prefix = f"{color}[FUZZER {record.levelname}]{self.RESET}"
        return f"{prefix} {record.getMessage()}"

# --- 2) Configure main logger ---
main_log = os.path.join(logs_dir, 'main.log')
logger   = logging.getLogger('fuzzer')
logger.setLevel(logging.DEBUG)

fh = logging.FileHandler(main_log, mode='w')
fh.setLevel(logging.DEBUG)
fh.setFormatter(ColorFormatter())
logger.addHandler(fh)

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(ColorFormatter())
logger.addHandler(ch)

# --- 3) Logging utility functions ---
def banner():
    logo = '''

██████╗ ██████╗ ███████╗    ███████╗██╗   ██╗███████╗███████╗
██╔══██╗██╔══██╗██╔════╝    ██╔════╝██║   ██║╚══███╔╝╚══███╔╝
██║  ██║██████╔╝███████╗    █████╗  ██║   ██║  ███╔╝   ███╔╝ 
██║  ██║██╔══██╗╚════██║    ██╔══╝  ██║   ██║ ███╔╝   ███╔╝  
██████╔╝██║  ██║███████║    ██║     ╚██████╔╝███████╗███████╗
╚═════╝ ╚═╝  ╚═╝╚══════╝    ╚═╝      ╚═════╝ ╚══════╝╚══════╝
                                                                by Oogway                                                         
'''
    print(logo)

def info(message):
    logger.info(message)

def warn(message):
    logger.warning(message)

def error(message):
    logger.error(message)

def done(message):
    record = logger.makeRecord(logger.name, logging.INFO, None, None, message, None, None)
    record.levelname = 'DONE'
    logger.handle(record)

def debug(message):
    logger.debug(message)
