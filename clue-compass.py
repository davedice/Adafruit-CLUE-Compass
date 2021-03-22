## Copyright 2021 Dave Dice
##
## -------------------------------------------------------------------------------------
## Permission is hereby granted, free of charge, to any person obtaining a 
## copy of this software and associated documentation files (the "Software"), to 
## deal in the Software without restriction, including without limitation the rights 
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of 
## the Software, and to permit persons to whom the Software is furnished to do so, subject 
## to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all 
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
## INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
## PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
## LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
## TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
## OR OTHER DEALINGS IN THE SOFTWARE.
## -------------------------------------------------------------------------------------
##
## Tilt-compensated compass for adafruit CLUE
## Released under standard MIT license

import time
import random
import board
import displayio
import terminalio
import struct
import math
from math import cos, sin, sqrt, radians, degrees,  atan2, log, exp, atan2, atan, asin, fabs

from adafruit_display_shapes.rect import Rect
from adafruit_display_shapes.circle import Circle
from adafruit_display_shapes.roundrect import RoundRect
from adafruit_display_shapes.triangle import Triangle
from adafruit_display_shapes.polygon import Polygon
from adafruit_display_shapes.line import Line
from adafruit_display_shapes.polygon import Polygon
from adafruit_display_text import label
from adafruit_bitmap_font import bitmap_font
from adafruit_clue import clue
from vectorio import Polygon, VectorShape
from microcontroller import nvm

## Vector operators 
## 
## References
## *  A New Solution Algorithm of Magnetic Azimuth 
##    Y Sun et al 2006 J. Phys.: Conf. Ser. 48 020
##    https://iopscience.iop.org/article/10.1088/1742-6596/48/1/020/meta
## *  https://openstax.org/books/calculus-volume-3/pages/2-4-the-cross-product
## *  https://github.com/kolosy/ArduSailor/blob/master/firmware/ahrs.cpp

class Vector :
  x = 0.0 
  y = 0.0 
  z = 0.0
  def __init__ (self, x, y, z) :
    self.x = x
    self.y = y
    self.z = z

def VCross (a, b, out) :
  out.x = a.y * b.z - a.z * b.y
  out.y = a.z * b.x - a.x * b.z
  out.z = a.x * b.y - a.y * b.x 

def VCross (a,b) :
  outx = a.y * b.z - a.z * b.y
  outy = a.z * b.x - a.x * b.z
  outz = a.x * b.y - a.y * b.x 
  return Vector(outx, outy, outz) 

def VDot (a, b) -> float : 
  return a.x * b.x + (a.y * b.y) + (a.z * b.z) 

def VNormalize (a) :
  mag = sqrt(VDot(a, a))
  if mag == 0 : return 
  a.x /= mag;
  a.y /= mag;
  a.z /= mag;
  return a 

## IMU-MPU orientation 
From = Vector (1.0, 0.0, 0.0) 

## Vheading()
## "a" is the gravity vector obtained from the accelerometer
## "m" is the magnetic vector obtained from the magnetometer
## The cross-product of m and a yields East
## The cross-product of a and East yields North, which is what we want

def VHeading (a, m) -> float :
  ## compute E and N
  E = VCross (m, a);
  VNormalize (E) 
  N = VCross (a, E)
  VNormalize (N) 

  ## compute heading
  ## return value is in radians and not yet converted from "math"
  ## convention, where 0 degrees is points Eastward and increasing values angle values
  ## turn counter-clockwise, to the navigation frame where 0 is North and increasing values
  ## turn clockwise
  return atan2(VDot(E, From), VDot(N, From)) 
  
## -----------------------------------------------------

def NormalizeAngle (A) :
  if A >= 360.0 : A -= 360.0 
  if A < 0.0    : A += 360.0
  ## Claim A in [0, 360) 
  return A

def SignOf (x) :
  return 1.0 if x >= 0.0 else -1.0 

mx,my,mz = clue.magnetic
Epsilon = 0.0001      ## avoid initial divide-by-0
minx = mx
maxx = mx + Epsilon
miny = my
maxy = my + Epsilon
minz = mz
maxz = mz + Epsilon

## Recent magnetic calibration/correction is stored in NVM
## NVM magnetic calibration state : (0xDD, minx, maxx, miny, maxy, minz, maxz) 
## Ideally we'd keep a full 3x3 rotation matrix, but the current implementation suffices

NVMTAG = 0xDD
magcal = struct.unpack ("iffffff", nvm[0:28]) 
if magcal[0] != NVMTAG : 
  print ('Magnet calibration not found in NVM') 
  time.sleep (5) 
else :
  minx = magcal[1]
  maxx = magcal[2]
  miny = magcal[3] 
  maxy = magcal[4]
  minz = magcal[5]
  maxz = magcal[6] 
  print ('Restored magnetic calibration from NVM') 
  print ("X {:+5.1f} {:+5.1f} ".format (minx, maxx))
  print ("Y {:+5.1f} {:+5.1f} ".format (miny, maxy))
  print ("Z {:+5.1f} {:+5.1f} ".format (minz, maxz)) 
  time.sleep (1.0) 

# Make the display context
splash = displayio.Group(max_size=100)
board.DISPLAY.show(splash)
board.DISPLAY.auto_refresh = False

PolyPalette = displayio.Palette(2)
PolyPalette.make_transparent(0)
PolyPalette[1] = 0x00FF00

def DrawLine (x1,y1,x2,y2,thickness) : 
  global PolyPalette 
  thickness /= 2
  angle = atan2(y2-y1,x2-x1)
  rect = Polygon (points = [ 
    (int(x1 + thickness*cos(angle+math.pi/2)) , int(y1 + thickness*sin(angle+math.pi/2))), 
    (int(x1 + thickness*cos(angle-math.pi/2)) , int(y1 + thickness*sin(angle-math.pi/2))),
    (int(x2 + thickness*cos(angle-math.pi/2)) , int(y2 + thickness*sin(angle-math.pi/2))), 
    (int(x2 + thickness*cos(angle+math.pi/2)) , int(y2 + thickness*sin(angle+math.pi/2))) 
  ]) ; 
  return VectorShape (shape=rect, x=0, y=0, pixel_shader=PolyPalette) ; 


## 8 points with 2-letter max and 45 degree slice 
## 16 points with 3-letter max and 22.5 degrees slice
## Recall that N is mid-point of N region

CardinalPoints = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"] 

def ToCardinal (A) :
  ## normalize
  if A >= 360 : A -= 360 
  if A < 0    : A += 360
  arc = 360/len(CardinalPoints)
  A += arc/2
  if A >= 360 : A -= 360 
  return CardinalPoints[int(A/arc)] 


## Draw compass rose with tics on circumference
## Use heavier/longer tic line for cardinal points
## cx and cy are abs screen coordinates center of circle
## R is radius, serves as H hypotenuse in trig calculations
## A is heading angle with 0=north 
## LL is tic line length

def PlaceTick (cx, cy, R, A, LL, LW) :
   global PolyPalette 
   LL /= 2.0
   LW /= 2.0

   A = 90.0 - A          
   cosa = cos(radians(A)) 
   sina = sin(radians(A)) 
   ox = cx + cosa * (R + LL) 
   oy = cy - sina * (R + LL) 
   ix = cx + cosa * (R - LL)  
   iy = cy - sina * (R - LL) 

   ## optimize for degenerate case where LW <= 1.0 
   ## just return Line (ix, iy, ox, oy, 0xFF0000)
   ## use vectorio Polygon only for non-trivial lines with > 1 thickness
   if LW <= 1.0 : 
     return Line (int(ix), int(iy), int(ox), int(oy), 0xFF0000)

   ## (ix,iy) and (ox,oy) define center points of the tic line
   ## the tic line crosses the circle 
   ## We calculate (ky,ky) offsets to get from the center line
   ## to the edges of the polygon
   kx = sina*LW
   ky = cosa*LW 

   rect = Polygon (points = [ 
     (int(ix-kx), int(iy-ky)), 
     (int(ix+kx), int(iy+ky)),
     (int(ox+kx), int(oy+ky)), 
     (int(ox-kx), int(oy-ky))
   ]) ;
   return VectorShape (shape=rect, x=0, y=0, pixel_shader=PolyPalette) ; 

## Tags are the small N/S/E/W letters inscirbed with the circumference of the ring

def MoveTag (Tag, cx, cy, Radius, A) :
  ## Note that we anchored text at its center instead of the default center-of-left-edge  
  ## Use anchor_position (0.5, 0.5) 
  ## Shrink radius so we draw on a virtual circle inside the main circle
  Radius -= 25.0
  cosa = cos(radians(NormalizeAngle(90.0 - A))) 
  sina = sin(radians(NormalizeAngle(90.0 - A))) 
  Tag.x = int(cx + cosa * Radius) 
  Tag.y = int(cy - sina * Radius) 

def Delay(D) :
  if D <= 0 : return
  time.sleep(D) 

def AngularDistance (A,B) :
  D = abs(A-B)
  if D > 180.0 : D = 360.0 - D
  return D 

EmptyLine    = Line(0, 0, 0, 0, 0xFF0000) 

RefreshDelay = 0
edge         = 20       ## buffer around edge : margin
D            = 240      ## assume square display

cx           = D/2      ## Center of rose at (D/2),(D/2) 
cy           = D/2 

## The main circle and the north mark are fixed and immutable 
## they don't vary with heading
## so we draw them just once during initialization
## First, draw the main circle -- azimuth circle

Radius = (D-(edge*2.0))/2.0
C = Circle (int(D/2), int(D/2), int(Radius), fill=0, outline=0x00FF00, stroke=3)  
splash.append (C) 

## Place Top boxing mark
## Reference circle for orientation 
## A triangle with point down might be more aesthetically pleasing than a circle
Top = Circle (int(D/2), 0, 22, fill=0x0F0FFF, outline=0x0F0F0F)
splash.append (Top) 

## Initialize north "needle" at top 
## TODO: this should be a triangle with tip of long side touching the ring
North = Circle (int(D/2), edge+25, 15, fill=0xFF0000) 
splash.append(North) 
nx = cos(radians(90.0 - 0)) * (Radius - 20) 
ny = sin(radians(90.0 - 0)) * (Radius - 20) 
North.x = int(cx + nx - 15) 
North.y = int(cy - ny - 15) 



## Text window in center : 
## contains an abbreviated direction "NW" and numeric value "270" 
font = bitmap_font.load_font("/fonts/CascadiaCode-Bold-42.bdf")

CentralTextArea = label.Label(font, text="--", color=0x0000FF)
CentralTextArea.x = int(cx - 25) 
CentralTextArea.y = int(cy - 30) 
CentralTextArea.anchor_point = (0.5, 1.0)           ## Center
splash.append(CentralTextArea) 

CentralNumeric  = label.Label(font, text="000", color=0x0000FF) 
CentralNumeric.x = int(cx - 30)
CentralNumeric.y = int(cy + 10) 
CentralNumeric.anchor_point = (0.5, 1.0)            ## Center
splash.append(CentralNumeric) 

## Precompute heading strings
HeadingStrings = [""] * 360 
for k in range(360) : 
  HeadingStrings[k] = f'{k}'

FixedElements = len(splash) 
TicElements   = len(splash) 

TicLines = [None] * 16 
for k in range(16) :
  L = Line (0, k*5, 200, k*5, 0xFF0000) 
  splash.append(L) 
  TicLines[k] = L 


## Tags are signle letters N-E-S-W  that float just inside
## the compass ring and abut major cardinal marks 
## They are non-fixed and must be moved when adjusting the heading
## Transiently place at 0-0 until we move them 
TagFont = terminalio.FONT
TagFont = bitmap_font.load_font("/fonts/Helvetica-Bold-16.bdf")
TagN = label.Label(TagFont, text="N", color=0x00FF00) 
TagN.x = 0 
TagN.y = 0 
TagN.anchor_position = (0.5, 0.5) 
splash.append(TagN)
TagE = label.Label(TagFont, text="E", color=0x00FF00) 
TagE.x = 0 
TagE.y = 0 
TagN.anchor_position = (0.5, 0.5) 
splash.append(TagE)
TagS = label.Label(TagFont, text="S", color=0x00FF00) 
TagS.x = 0 
TagS.y = 0 
TagS.anchor_position = (0.5, 0.5) 
splash.append(TagS)
TagW = label.Label(TagFont, text="W", color=0x00FF00) 
TagW.x = 0 
TagW.y = 0 
TagW.anchor_position = (0.5, 0.5) 
splash.append(TagW)

## for debug mode it can be useful to refresh() and pause at this point

Heading      = 0.0 
rh           = 0.0
HLast        = Heading + 90 

while True:
  ## Read raw magnetometer values in uT units
  mx,my,mz = clue.magnetic 

  ## Capture raw
  rx = mx 
  ry = my
  rz = mz

  maxx = max (maxx, mx)
  minx = min (minx, mx)
  maxy = max (maxy, my)
  miny = min (miny, my)
  maxz = max (maxz, mz)
  minz = min (minz, mz) 

  ## hard and soft iron error/distortion 
  ## calibration : normalizes into [-1, 1] 
  ## Apply calibration correction via semi-min and semi-max
  mx = (((mx - minx) * 2) / (maxx - minx)) - 1.0 
  my = (((my - miny) * 2) / (maxy - miny)) - 1.0
  mz = (((mz - minz) * 2) / (maxz - minz)) - 1.0 

  ## Compute simple calibrated compass heading but without tilt compensation
  Heading = degrees(atan2 (my, mx))  
  Heading = NormalizeAngle (90.0 - Heading) 

  ## OPTIONAL : metric of quality -- figure of merit; QoS
  ## Distance from mn norm to 1 indicates confidence/reliability/error/quality of sensor
  ## SensorError = abs(mn - 1.0) 
  ## Expect the value to converge to 1 via incremental calibration
  ## Also for detecting introduced nearby magnets
  ## Expect raw field to be about 50K uT
  MagField = sqrt (rx*rx + ry*ry + rz*rz) / 2.0
  mn = sqrt (mx*mx + my*my + mz*mz)
  SensorError = fabs (mn - 1.0) 

  ## Read tilt from accelerometer : pitch and roll
  ax,ay,az = clue.acceleration

  ## produce tilt-compensated heading
  Heading = VHeading (Vector(ax, ay, az), Vector(mx, my, mz)) 
  Heading = NormalizeAngle (90.0 + degrees(Heading)) 

  ## We have the Heading in hand, now update the GUI
  ## CONSIDER: smooth and damp changes with filter

  ## Update central window : NE | 45 
  CentralTextArea.text = ToCardinal(Heading) 
  CentralNumeric.text  = "  " 
  CentralNumeric.text  = HeadingStrings[int(Heading)] 

  ## Move NORTH to proper position -- reposition
  ## Position is function of heading angle
  ## Note that tileGrid.x/y are the upper-left-hand corner of the grid rectangle
  nh = NormalizeAngle (90 + Heading) 
  nx = cos(radians(nh)) * (Radius - 0) 
  ny = sin(radians(nh)) * (Radius - 0) 
  North.x = int((cx - 15) + nx) 
  North.y = int((cy - 15) - ny) 
  
  ## Draw tick marks on main circle
  ## Use relative bearing : offset
  nh = NormalizeAngle (360.0 - Heading) 
  Angle = nh 
  for k in range(16) : 
    ## Short lines for minor and long lines for major
    TLL = 25 
    TLW = 3
    if (k % 2) == 1 : 
      TLL = 10 
      TLW = 1 
    splash[FixedElements + k] = PlaceTick (cx, cy, Radius, Angle, TLL, TLW)  
    Angle += 22.5 
    if Angle >= 360.0 : Angle -= 360.0

  ## Move the tags 
  MoveTag (TagN, cx, cy, Radius, nh + 0.0) 
  MoveTag (TagE, cx, cy, Radius, nh + 90.0) 
  MoveTag (TagS, cx, cy, Radius, nh + 180.0) 
  MoveTag (TagW, cx, cy, Radius, nh + 270.0) 
    
  board.DISPLAY.refresh()

  Delay (RefreshDelay) 
  
  if clue.touch_1 : 
    clue.white_leds = True
    clue.play_tone (1200, 1.0) 
    time.sleep (1.0) 
    print ('Reset magnet calibration') 
    mx,my,mz = clue.magnetic 
    minx = mx
    maxx = mx + Epsilon
    miny = my
    maxy = my + Epsilon
    minz = mz
    maxz = mz + Epsilon
    time.sleep (0.3) 
    clue.white_leds = False
  
  if clue.button_b : 
    CurrentColor = clue.pixel 
    clue.pixel.fill((255,0,255)) 
    clue.play_tone (660, 0.5) 
    print ('Saving magnetic calibration to NVM') 
    print ("X {:+5.1f} {:+5.1f} ".format (minx, maxx))
    print ("Y {:+5.1f} {:+5.1f} ".format (miny, maxy))
    print ("Z {:+5.1f} {:+5.1f} ".format (minz, maxz)) 
    nvm[0:28] = struct.pack ("iffffff", NVMTAG, minx, maxx, miny, maxy, minz, maxz)  
    clue.pixel.fill(0) 
    time.sleep (1) 


