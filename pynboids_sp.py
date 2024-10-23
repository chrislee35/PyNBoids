#!/usr/bin/env python3
from math import pi, sin, cos, atan2, radians, degrees
from random import randint, random
import time
import pygame as pg
from datetime import datetime, timedelta

'''
PyNBoids - a Boids simulation - github.com/Nikorasu/PyNBoids
This version uses a spatial partitioning grid to improve performance.
Copyright (c) 2021  Nikolaus Stromberg  nikorasu85@gmail.com
'''
class Bubble(pg.sprite.Sprite):
    def __init__(self, drawSurf):
        super().__init__()
        self.drawSurf = drawSurf
        self.image = pg.Surface((22, 22)).convert()
        self.image.set_colorkey(0)
        v = randint(100, 180)
        self.color = pg.Color(v,v,v+75)  # preps color so we can use hsva
        #self.color.hsva = (randint(0,360), 90, 90) 
        self.bSize = 22
        r = randint(6, 10)
        self.speed = random()*8+1
        pg.draw.circle(self.image, self.color, (11,11), r, 1)
        maxW, maxH = self.drawSurf.get_size()
        self.rect = self.image.get_rect(center=(randint(50, maxW - 50), randint(maxH - 50, maxH)))
        self.pos = pg.Vector2(self.rect.center)
        self.ang = 90.0

    def update(self, dt, speed, ejWrap=False, bias_dir=None):
        maxW, maxH = self.drawSurf.get_size()
        selfCenter = pg.Vector2(self.rect.center)

        if self.rect.top < 0:
            self.pos.y = maxH
            self.pos.x = randint(50, maxW - 50)
        else:
            self.pos.y -= self.speed
            self.ang += (random()-0.5)
            if self.ang < 85.0: self.ang = 85.0
            if self.ang > 95.0: self.ang = 95.0
            self.pos.x += cos(radians(self.ang))*5

        # Actually update position of boid
        self.rect.center = self.pos

class Boid(pg.sprite.Sprite):

    def __init__(self, grid, drawSurf, isFish=False):  #, cHSV=None
        super().__init__()
        self.grid = grid
        self.drawSurf = drawSurf
        self.image = pg.Surface((15, 15)).convert()
        self.image.set_colorkey(0)
        self.color = pg.Color(0)  # preps color so we can use hsva
        self.color.hsva = (randint(0,360), 90, 90) #if cHSV is None else cHSV # randint(5,55) #4goldfish
        if isFish:  # (randint(120,300) + 180) % 360  #4noblues
            pg.draw.polygon(self.image, self.color, ((7,0),(12,5),(3,14),(11,14),(2,5),(7,0)), width=3)
            self.image = pg.transform.scale(self.image, (16, 24))
        else:
            pg.draw.polygon(self.image, self.color, ((7,0), (13,14), (7,11), (1,14), (7,0)))
        self.bSize = 22 if isFish else 17
        self.orig_image = pg.transform.rotate(self.image.copy(), -90)
        self.dir = pg.Vector2(1, 0)  # sets up forward direction
        maxW, maxH = self.drawSurf.get_size()
        self.rect = self.image.get_rect(center=(randint(50, maxW - 50), randint(50, maxH - 50)))
        self.ang = randint(0, 360)  # random start angle, & position ^
        self.pos = pg.Vector2(self.rect.center)
        self.grid_lastpos = self.grid.getcell(self.pos)
        self.grid.add(self, self.grid_lastpos)
        self.speed_factor = ((random() - 0.5)*0.5) + 1.0

    def update(self, dt, speed, ejWrap=False, bias_dir=None):
        maxW, maxH = self.drawSurf.get_size()
        selfCenter = pg.Vector2(self.rect.center)
        turnDir = xvt = yvt = yat = xat = 0
        turnRate = 120 * dt  # about 120 seems ok
        margin = 42
        if bias_dir:
            if bias_dir > self.ang:
                self.ang = self.ang + randint(0, 4)
            else:
                self.ang = self.ang + randint(-4, 0)
            self.ang = (self.ang + 360) % 360
        else:
            self.ang = self.ang + randint(-4, 4)
        # Grid update stuff
        self.grid_pos = self.grid.getcell(self.pos)
        if self.grid_pos != self.grid_lastpos:
            self.grid.add(self, self.grid_pos)
            self.grid.remove(self, self.grid_lastpos)
            self.grid_lastpos = self.grid_pos
        # get nearby boids and sort by distance
        near_boids = self.grid.getnear(self, self.grid_pos)
        neiboids = sorted(near_boids, key=lambda i: pg.Vector2(i.rect.center).distance_to(selfCenter))
        del neiboids[7:]  # keep 7 closest, dump the rest
        # check when boid has neighborS (also sets ncount with walrus :=)
        if (ncount := len(neiboids)) > 1:
            nearestBoid = pg.Vector2(neiboids[0].rect.center)
            for nBoid in neiboids:  # adds up neighbor vectors & angles for averaging
                xvt += nBoid.rect.centerx
                yvt += nBoid.rect.centery
                yat += sin(radians(nBoid.ang))
                xat += cos(radians(nBoid.ang))
            tAvejAng = degrees(atan2(yat, xat))
            targetV = (xvt / ncount, yvt / ncount)
            # if too close, move away from closest neighbor
            if selfCenter.distance_to(nearestBoid) < self.bSize : targetV = nearestBoid
            tDiff = targetV - selfCenter  # get angle differences for steering
            tDistance, tAngle = pg.math.Vector2.as_polar(tDiff)
            # if boid is close enough to neighbors, match their average angle
            if tDistance < self.bSize*5 : tAngle = tAvejAng
            # computes the difference to reach target angle, for smooth steering
            angleDiff = (tAngle - self.ang) + 180
            if abs(tAngle - self.ang) > .5: turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            # if boid gets too close to target, steer away
            if tDistance < self.bSize and targetV == nearestBoid : turnDir = -turnDir
        # Avoid edges of screen by turning toward the edge normal-angle
        sc_x, sc_y = self.rect.centerx, self.rect.centery
        if not ejWrap and min(sc_x, sc_y, maxW - sc_x, maxH - sc_y) < margin:
            if sc_x < margin : tAngle = 0
            elif sc_x > maxW - margin : tAngle = 180
            if sc_y < margin : tAngle = 90
            elif sc_y > maxH - margin : tAngle = 270
            angleDiff = (tAngle - self.ang) + 180  # increase turnRate to keep boids on screen
            turnDir = (angleDiff / 360 - (angleDiff // 360)) * 360 - 180
            edgeDist = min(sc_x, sc_y, maxW - sc_x, maxH - sc_y)
            turnRate = turnRate + (1 - edgeDist / margin) * (20 - turnRate) #turnRate=minRate, 20=maxRate
        if turnDir != 0:  # steers based on turnDir, handles left or right
            self.ang += turnRate * abs(turnDir) / turnDir
        self.ang %= 360  # ensures that the angle stays within 0-360
        # Adjusts angle of boid image to match heading
        self.image = pg.transform.rotate(self.orig_image, -self.ang)
        self.rect = self.image.get_rect(center=self.rect.center)  # recentering fix
        self.dir = pg.Vector2(1, 0).rotate(self.ang).normalize()
        self.pos += self.dir * dt * ((self.speed_factor*speed) + (7 - ncount) * 5)  # movement speed
        # Optional screen wrap
        if ejWrap and not self.drawSurf.get_rect().contains(self.rect):
            if self.rect.bottom < 0 : self.pos.y = maxH
            elif self.rect.top > maxH : self.pos.y = 0
            if self.rect.right < 0 : self.pos.x = maxW
            elif self.rect.left > maxW : self.pos.x = 0
        # Actually update position of boid
        self.rect.center = self.pos


class BoidGrid():  # tracks boids in spatial partition grid

    def __init__(self):
        self.grid_size = 100
        self.dict = {}
    # finds the grid cell corresponding to given pos
    def getcell(self, pos):
        return (pos[0]//self.grid_size, pos[1]//self.grid_size)
    # boids add themselves to cells when crossing into new cell
    def add(self, boid, key):
        if key in self.dict:
            self.dict[key].append(boid)
        else:
            self.dict[key] = [boid]
    # they also remove themselves from the previous cell
    def remove(self, boid, key):
        if key in self.dict and boid in self.dict[key]:
            self.dict[key].remove(boid)
    # Returns a list of nearby boids within all surrounding 9 cells
    def getnear(self, boid, key):
        if key in self.dict:
            nearby = []
            for x in (-1, 0, 1):
                for y in (-1, 0, 1):
                    nearby += self.dict.get((key[0] + x, key[1] + y), [])
            nearby.remove(boid)
        return nearby

class BoidScreensaver:
    def __init__(self, fullscreen=True, show=None, size_wh: tuple[int]=(1200,800), timer: int=None):
        self.fullscreen = fullscreen
        self.show = show # None, or "clock" or "timer"
        self.size = size_wh
        self.boidz = 200
        self.wrap = False
        self.fish = False
        self.speed = 150
        self.bgcolor = (0,0,0)
        self.color = (0,200,0)
        self.fps = 60
        self.timer = timer
        self.windowed = True
        self.top_text = None
        self.bottom_text = None
        self.follow_mouse = False
        self.waves = False
        self.bubbles = False
        
    def start(self):
        self.start_time = time.time()
        if self.timer:
            self.end_time = self.start_time + self.timer
        pg.init()  # prepare window
        pg.display.set_caption("PyNBoids")
        try:
            pg.display.set_icon(pg.image.load("nboids.png"))
        except:
            print("Note: nboids.png icon not found, skipping..")
        # setup fullscreen or window mode
        if self.fullscreen:
            self.size = (pg.display.Info().current_w, pg.display.Info().current_h)
            screen = pg.display.set_mode(self.size, pg.SCALED | pg.NOFRAME | pg.FULLSCREEN, vsync=1)
            pg.mouse.set_visible(False)
        elif self.windowed:
            screen = pg.display.set_mode(self.size, pg.RESIZABLE | pg.SCALED, vsync=1)
        else:
            screen = pg.display.set_mode(self.size, pg.RESIZABLE | pg.NOFRAME, vsync=1)

        boidTracker = BoidGrid()
        nBoids = pg.sprite.Group()
        # spawns desired # of boidz
        for n in range(self.boidz):
            nBoids.add(Boid(boidTracker, screen, self.fish))

        if self.bubbles:
            for n in range(randint(6, 20)):
                nBoids.add(Bubble(screen))
        font_size = 260
        if font_size > self.size[1] // 3:
            font_size = self.size[1] // 3
        font = pg.font.Font(None, font_size)

        top_offset = None
        bottom_offset = None
        offset = None
        
        clock = pg.time.Clock()
        # main loop
        while True:
            for e in pg.event.get():
                if e.type == pg.QUIT or e.type == pg.KEYDOWN and (e.key == pg.K_ESCAPE or e.key == pg.K_q or e.key==pg.K_SPACE):
                    return

            dt = clock.tick(self.fps) / 1000
            # update boid logic, then draw them
            delta_t = time.time() - self.start_time
            speed = int(cos(delta_t)*(self.speed/3) + self.speed)
            bgcolor = self.bgcolor
            if self.waves:
                v = ((1-cos(delta_t)))*16
                bgcolor = (bgcolor[0]+v,bgcolor[1]+v,bgcolor[2]+v)

            screen.fill(bgcolor)
            
            ang = None
            if self.follow_mouse:
                m_x, m_y = pg.mouse.get_pos()
                if m_x != 0 or m_y != 0:
                    dx = m_x - (self.size[0]//2)
                    dy = (m_y - (self.size[1]//2))
                    ang = degrees(atan2(dy, dx))
                    if ang < 0:
                        ang = ang + 360
                    ang = int(ang % 360)
            if pg.mouse.get_pressed(num_buttons=3)[0]:
                self.follow_mouse = not self.follow_mouse

            nBoids.update(dt, speed, self.wrap, ang)
            nBoids.draw(screen)
            # if true, displays the fps in the upper left corner, for debugging
            if self.show == "clock":
                rendered = font.render(time.strftime("%H:%M:%S"), True, self.color)
            elif self.show == "timer":
                remainder = self.end_time - time.time()
                if remainder < 0.0:
                    return
                remainder_str = time.strftime("%H:%M:%S", time.gmtime(int(remainder)))
                rendered = font.render(remainder_str, True, self.color)
            elif self.show == "both":
                remainder = self.end_time - time.time()
                if remainder < 0.0:
                    return
                remainder_str = time.strftime("%M:%S", time.gmtime(int(remainder)))
                both_str = time.strftime("%H:%M:%S")+" "+remainder_str
                rendered = font.render(both_str, True, self.color)
            
            if self.show:
                if not offset:
                    x = (self.size[0] - rendered.get_width())//2
                    y = (self.size[1] - rendered.get_height())//2
                    offset = (x,y)
                screen.blit(rendered, offset)

            if self.top_text:
                rendered = font.render(self.top_text, True, self.color)
                if not top_offset:
                    x = (self.size[0] - rendered.get_width())//2
                    y = 5
                    top_offset = (x,y)
                screen.blit(rendered, top_offset)

            if self.bottom_text:
                rendered = font.render(self.bottom_text, True, self.color)
                if not bottom_offset:
                    x = (self.size[0] - rendered.get_width())//2
                    y = self.size[1] - rendered.get_height() - 5
                    bottom_offset = (x,y)
                screen.blit(rendered, bottom_offset)

            pg.display.update()

if __name__ == '__main__':

    def seconds_until(strtime: str) -> int:
        now = datetime.now()
        hour, minute = [int(x) for x in strtime.split(":")]
        return int((timedelta(hours=24) - (now - now.replace(hour=hour, minute=minute, second=0, microsecond=0))).total_seconds() % (24 * 3600))

    import argparse
    parser = argparse.ArgumentParser(prog='PyNBoids Screensaver', usage='%(prog)s [options]')
    parser.add_argument('-c', '--clock', action='store_true', default=False, help='display the time')
    parser.add_argument('-s', '--size', nargs=2, default=None, help='size of window, fullscreen otherwise')
    parser.add_argument('-f', '--fish', action='store_true', default=False, help='set if you want fish')
    parser.add_argument('-w', '--wrap', action='store_true', default=False, help='wrap')
    parser.add_argument('-n', '--number', type=int, default=200, help='number of swimmers')
    parser.add_argument('-b', '--bgcolor', nargs=3, type=int, default=(0,0,0), help='background color')
    parser.add_argument('--color', nargs=3, type=int, default=(0,200,0), help='text color')
    parser.add_argument('-t', '--timer', type=int, default=None, help='number of second for a timer, at the end of which, this will exit')
    parser.add_argument('-u', '--until', default=None, help='like timer, but it calculates the countdown until a given time')
    parser.add_argument('--speed', type=int, default=150, help='base speed of fish')
    parser.add_argument('-x', '--small', action='store_true', default=False, help='sets the size to a small window, no windowing')
    parser.add_argument('--top', default=None, help='sets static text at the top')
    parser.add_argument('--bottom', default=None, help='sets static text at the bottom')
    parser.add_argument('--follow', action='store_true', default=False, help='follow mouse direction from center')
    parser.add_argument('--waves', action='store_true', default=False, help='make background grow lighter and darker')
    parser.add_argument('--bubbles', action='store_true', default=False, help='draw bubbles')
    args = parser.parse_args()

    bs = BoidScreensaver()

    show = None
    if args.clock: show = "clock"
    if args.timer: show = "timer"
    if args.until: show = "timer"
    if args.clock and (args.timer or args.until): show = "both"
    
    bs.show = show
    bs.timer = args.timer
    if args.until:
        bs.timer = seconds_until(args.until)

    bs.fish = args.fish
    bs.boidz = args.number
    bs.bgcolor = args.bgcolor
    bs.color = args.color
    if args.small:
        bs.fullscreen = False
        bs.size = (322, 200)
        bs.windowed = False
    elif not args.size:
        bs.fullscreen = True
    else:
        bs.size = args.size
    bs.speed = args.speed
    bs.wrap = args.wrap
    bs.top_text = args.top
    bs.bottom_text = args.bottom
    bs.follow_mouse = args.follow
    bs.waves = args.waves
    bs.bubbles = args.bubbles
    bs.start()
    pg.quit()
