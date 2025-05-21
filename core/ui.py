def info(message):
    print(f"\033[94m[FUZZER INFO]\033[0m {message}")

def warn(message):
    print(f"\033[93m[FUZZER WARN]\033[0m {message}")

def error(message):
    print(f"\033[91m[FUZZER ERROR]\033[0m {message}")

def done(message):
    print(f"\033[92m[FUZZER DONE]\033[0m{message}")