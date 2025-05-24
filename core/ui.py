import os
import logging

# ensure output/logs dir
logs_dir = os.path.join(os.getcwd(), 'output', 'logs')
os.makedirs(logs_dir, exist_ok=True)

# configure main logger
main_log = os.path.join(logs_dir, 'main.log')
logger = logging.getLogger('fuzzer')
logger.setLevel(logging.DEBUG)

# file handler for main.log
fh = logging.FileHandler(main_log)
fh.setLevel(logging.DEBUG)
fh.setFormatter(logging.Formatter('[FUZZER %(levelname)s] %(message)s'))
logger.addHandler(fh)

# console handler with colors
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

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(ColorFormatter())
logger.addHandler(ch)

def info(message):
    logger.info(message)

def warn(message):
    logger.warning(message)

def error(message):
    logger.error(message)

def done(message):
    # log as INFO but display as DONE
    record = logger.makeRecord(logger.name, logging.INFO, None, None, message, None, None)
    record.levelname = 'DONE'
    logger.handle(record)

def debug(message):
    logger.debug(message)
