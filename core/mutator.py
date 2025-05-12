from typing import Tuple, Dict
import random

WEIGHTS = {
    'bit_flip': 3,
    'byte_flip': 3,
    'interesting_byte': 2,
    'arithmetic': 2,
    'swap_bytes': 1
}

INTERESTING_BYTES = [0x00, 0xFF, 0x7F, 0x80]

def choose_strategy(weights: Dict[str, int]) -> str:
    strategies, w = zip(*weights.items())
    return random.choices(strategies, weights=w, k=1)[0]

def bit_flip(arr: bytearray, bound: float) -> None:
    n = len(arr)
    k = max(1, int(bound * n))
    for i in random.sample(range(n), k):
        b = 1 << random.randrange(8)
        arr[i] ^= b

def byte_flip(arr: bytearray, bound: float) -> None:
    n = len(arr)
    k = max(1, int(bound * n))
    for i in random.sample(range(n), k):
        arr[i] ^= 0xFF

def interesting_byte(arr: bytearray, bound: float) -> None:
    n = len(arr)
    k = max(1, int(bound * n))
    for i in random.sample(range(n), k):
        arr[i] = random.choice(INTERESTING_BYTES)

def arithmetic(arr: bytearray, bound: float) -> None:
    n = len(arr)
    k = max(1, int(bound * n))
    deltas = [1, -1, 0x7F, -0x7F]
    for i in random.sample(range(n), k):
        arr[i] = (arr[i] + random.choice(deltas)) & 0xFF

def swap_bytes(arr: bytearray, bound: float) -> None:
    n = len(arr)
    k = max(1, int(bound * n))
    for _ in range(k):
        i, j = random.sample(range(n), 2)
        arr[i], arr[j] = arr[j], arr[i]

def mutate(data: bytes, bound: float, weights: Dict[str, int] = WEIGHTS) -> Tuple[bytes, str]:
    arr = bytearray(data)
    strat = choose_strategy(weights)
    globals()[strat](arr, bound)
    return bytes(arr), strat

if __name__ == '__main__':
    sample = b'abcdef0123456789'
    for b in [0.0, 0.25, 0.5, 1.0]:
        mutated, strategy = mutate(sample, b)
        print(f'bound={b}, strategy={strategy}, before={sample.hex()}, after={mutated.hex()}')