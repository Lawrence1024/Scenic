DEBUG_ENABLED = False

def log(tag, message):
    if DEBUG_ENABLED:
        print(f"[{tag}] {message}")


