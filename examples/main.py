ush=True)
    # Try again if no card is available.
    if uid is None:
        continue
    print('Found card with UID:', [hex(i) for i in uid])
