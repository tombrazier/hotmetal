#!/usr/bin/python3

values = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2, 10.0, 12.0, 15.0, 18.0, 22.0, 27.0, 33.0, 39.0, 47.0, 56.0, 68.0, 82.0, 100.0, 120.0, 150.0, 180.0, 220.0, 270.0, 330.0, 390.0, 470.0, 560.0, 680.0, 820.0, 1000.0, 1200.0, 1500.0, 1800.0, 2200.0, 2700.0, 3300.0, 3900.0, 4700.0, 5600.0, 6800.0, 8200.0]

valset = set(values)

best = []

for r1 in [100.0]:
    for r3 in [100.0]:
        for r2 in values:
            # we want a gain of at least about 10 in the op amp for stability
            # so r2 / (r1 + r2) should be less than around 0.1
            if r2 / (r1 + r2) > 0.1: continue
            for r4 in values:
                for r5 in values:
                    alpha = r2/(r1+r2)

                    a = r2 / (r1 + r2) * (r3*r4 + r4*r5 + r5*r3) / (r3*r4)
                    b = r5 / r3

                    bot = a * 2.5 - 5 * b
                    if bot < 0.0: continue
                    top = a * 4.5 - 5 * b
                    if top > 1.8: continue
                    
                    if top - bot < 0.7: continue
                    
                    Vupper = (1.8 + 5 * b) / a
                    Vlower = (0.0 + 5 * b) / a

                    best.append((top-bot, bot, top, Vupper, Vlower, r1, r2, r3, r4, r5))

for row in sorted(best, reverse=True)[0:20]:
    print(row)
