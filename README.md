# Strike Lab -- bowling physics simulator

A single-page, dependency-free simulator that shows what it takes to throw a strike and tells you where to stand and where to aim.

Open `index.html` in any browser (works on a phone), or serve the folder with `python3 -m http.server`.

## What it does

- Simulates a 15 lb ball on a USBC-spec lane (60 ft, 39 boards, 4.766 in pins on 12 in centers).
- Real skid / hook / roll mechanics: Coulomb friction at the contact patch acts against the slip velocity `v + w x r`, low friction on the oil pattern, high friction on the dry backend. Side rotation is converted into lateral force (the hook) until slip reaches zero and the ball rolls straight.
- Ball-pin and pin-pin collisions with restitution, kickbacks and a pit. Pins topple when struck above a threshold speed, then slide and sweep neighbours.
- Readouts: entry board, entry angle, breakpoint, speed at the pins, pins down / leave.
- **Find my line**: sweeps the target board for your speed, rev rate and axis rotation, and reports the stance/target combination with the highest strike rate.
- **Strike %**: rolls 100 shots with realistic human error (about half a board at the arrows, a quarter mph, 15 rpm) so you can see how forgiving a line really is.
- Coach panel translates the result into a move ("missed high, move your feet left 2 boards, same target").

## Files

- `sim.js` -- physics core (UMD; also runs under Node for tests)
- `index.html` -- UI, canvas rendering, coach text
- `test.js` -- sanity tests (`node test.js`)

## Physics summary

| Quantity | Value |
| --- | --- |
| Ball | 8.5 in diameter, 6.8 kg, solid-sphere inertia 2/5 m r^2 |
| Pin | 4.766 in belly, 1.58 kg |
| Oil friction | 0.04 |
| Dry backend friction | 0.22 |
| Pocket | board 17.5 (right hand), 22.5 (left hand) |
| Ideal entry angle | 4-6 degrees |

The model is 2D top-down (pins are treated as sliding disks once toppled), so pin carry is approximate. Ball motion is the real rigid-body friction model.
