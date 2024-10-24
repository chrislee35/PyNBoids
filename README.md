# PyNBoids

`python3 ./pynboids_sp.py --clock --fish --number 40 --wrap --color 0 20 200 --follow --waves --bgcolor 10 10 50 --bubbles --bgimg Underwater-Transparent-Background.png --size 600 400 --top pynboids --bottom demo`

[![Demo Video](https://github.com/chrislee35/PyNBoids/raw/refs/heads/main/pynboids.gif)](https://github.com/chrislee35/PyNBoids/raw/refs/heads/main/pynboids.mp4 "Demo Video")

I took [Niko's Boids simulation](https://github.com/Nikorasu/PyNBoids) and added a few options:
```
usage: PyNBoids Screensaver [options]

options:
  -h, --help            show this help message and exit
  -c, --clock           display the time
  -s SIZE SIZE, --size SIZE SIZE
                        size of window, fullscreen otherwise
  -f, --fish            set if you want fish
  -w, --wrap            wrap
  -n NUMBER, --number NUMBER
                        number of swimmers
  -b BGCOLOR BGCOLOR BGCOLOR, --bgcolor BGCOLOR BGCOLOR BGCOLOR
                        background color
  --color COLOR COLOR COLOR
                        text color
  -t TIMER, --timer TIMER
                        number of second for a timer, at the end of which, this will exit
  -u UNTIL, --until UNTIL
                        like timer, but it calculates the countdown until a given time
  --speed SPEED         base speed of fish
  -x, --small           sets the size to a small window, no windowing
  --top TOP             sets static text at the top
  --bottom BOTTOM       sets static text at the bottom
  --follow              follow mouse direction from center (click to toggle)
  --waves               make background grow lighter and darker
  --bubbles             draw bubbles
  --bgimg BGIMG         specify an image for the background
```

Also, each fish has a slightly different speed and the simulation speeds up and slows down over time.


### A Python Boids Simulation
This is a [Boids simulation](https://en.wikipedia.org/wiki/Boids "Wikipedia"),
written in Python3, with Pygame2 and NumPy.

**To use:** Save the `pynboids_sp.py` file (and `nboids.png` if you want the
icon, not required) and run via python. (Example: `python3 pynboids.py`)

`Esc` key to quit.

I've included several customizable settings near the top of the code.  
You can adjust window size, fullscreen, fps, and how many boids to spawn,
as well as whether they avoid the screen edges or wrap to the other side,
change the background color, or turn the boids into fish! ;)

##### Update (11/14/22):
Minor updates to several files. New `nboids_ss.py` version is a linux-only
pseudo-screensaver that **_requires_** _xprintidle_ be installed. BUT, due
to a possible memory leak, the screensaver must be started using `run_ss.py`.

##### Update (5/20/21):
New `pynboids_sp.py` version, implements a spatial partitioning grid to
improve efficiency of detecting other boids. Most efficient version so far!

##### Update (5/16/21):
Added `pynboids2.py` version, an update to the original pynboids, with numpy
array methods from pixelboids.py to improve efficiency. 2x more boids then b4.

##### Update (5/14/21):
Added `pixelboids.py` version, draws boids as pixels in surfarray that fades
as they move. Distance sorting & for-loop math replaced with numpy array math.
Uses a fading surfArray to create tails, pixelation makes them look animated.

#### Special Thanks:  (Let me know if I forgot anyone.)
I couldn't have gotten this far without the Pygame Discord channel:  
CozyFractal, for help with the spatial partition grid & improving efficiency.  
Mega_JC, Ghast, and bydariogamer, for answering various questions I had.

For more information, and future updates,
[see github page](https://github.com/Nikorasu/PyNBoids "PyNBoids").

---

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.
        If not, see: https://www.gnu.org/licenses/gpl-3.0.html

###### Copyright (c) 2021  Nikolaus Stromberg - nikorasu85@gmail.com
