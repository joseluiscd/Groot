import bitcoin
import webbrowser
import json
from urllib.request import urlopen
from mnemonic import Mnemonic

def check_addr(seed: str):
    print("Checking", seed)
    a = bitcoin.sha256(seed)
    k = bitcoin.privtoaddr(a)
    print("Addr", k)

    htmlfile = urlopen("https://blockchain.info/address/%s?format=json" % k, timeout = 10) 
    text = htmlfile.read().decode("utf-8")
    text = json.loads(text)
    print("RECV:", text["total_received"])
    balance = text["final_balance"]
    if balance == 0:
        print("Balance", balance)
    else:
        print("##########\nPREMIO!!\n############", balance)

    return balance

def compute_addr(seed: str):
    a = bitcoin.sha256(seed)
    k = bitcoin.privtoaddr(a)
    return k

def compute_addrs(seeds: [str]):
    return map(compute_addr, seeds)

def get_balances(seeds: [str]):
    addrs = compute_addrs(seeds)

    #print(list(addrs))
    addr_string = "|".join(addrs)
    htmlfile = urlopen("https://blockchain.info/balance?active=%s" % addr_string, timeout = 10) 
    text = htmlfile.read().decode("utf-8")
    text = json.loads(text)

    return map(lambda x: x["final_balance"], text.values())

def check_addrs(seeds: [str]):
    balances = get_balances(seeds)
    if any(seeds):
        print("PREMIOOOOOO!!!!!!!!")
        print(seeds)
        return True
    else:
        b = list(balances)
        print(b)
        print(len(b))
        return False

if __name__ == "__main__":
    """while True:
        seed = input("Seed: ")
        check_addr(seed)"""

    """x = 25890
    balance = False
    while not balance:
        seeds = map(str, range(x, x+100))
        balance = check_addrs(seeds)
    
        print("last x:", x)
        x += 100
"""

    mn = Mnemonic("english")