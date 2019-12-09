import numpy as np
w = 5
for norm in np.linspace(-1, 1, 21):
    mid = (360 + norm * 180) % 360
    p1 = (mid - w + 360) % 360
    p2 = (mid + w + 360) % 360
    p1i = int(p1)
    p2i = int(p2-1)
    midi = int(mid)
    print (p1i, midi-1, midi, p2i)
