"""RoboQuest: a 2D robot navigation and obstacle-avoidance simulation.

The simulation supports manual driving and autonomous A* navigation. It also
includes fixed and temporary obstacles, rotating laser hazards, simulated
LiDAR rays, collision handling, and mission status information.
"""
import heapq
import math
import sys
import pygame

# Display and arena settings
WIDTH, HEIGHT, FPS = 1280, 720, 60
ARENA = pygame.Rect(28, 112, 770, 525)
CELL = 35
COLS, ROWS = ARENA.width // CELL, ARENA.height // CELL
ROBOT_SPEED, ROBOT_RADIUS = 220.0, 13
START, GOAL = (1, 13), (20, 1)

# Colors
BG=(5,10,22); PANEL=(10,22,40); BORDER=(35,107,151)
GRID_A=(12,29,49); GRID_B=(14,34,55); GRID_LINE=(28,59,83)
CYAN=(48,222,255); ICE=(205,247,255); BLUE=(55,124,255)
GREEN=(55,235,155); GOLD=(255,192,72); RED=(255,66,91)
PURPLE=(182,100,255); ORANGE=(255,139,52); MUTED=(123,157,187)
WHITE=(240,248,255); PATH=(62,220,255); EXPLORED=(38,116,171)

pygame.init()
screen=pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("ROBOQUEST  //  BUILD 04: DYNAMIC HAZARDS")
clock=pygame.time.Clock()
F_TINY=pygame.font.SysFont("segoeui",10)
F_SMALL=pygame.font.SysFont("segoeui",12)
F_BODY=pygame.font.SysFont("segoeui",14)
F_MED=pygame.font.SysFont("segoeui",19,bold=True)
F_BIG=pygame.font.SysFont("segoeui",30,bold=True)
F_LOGO=pygame.font.SysFont("segoeui",32,bold=True)

# Drawing helpers
def txt(surface,value,pos,color=WHITE,font=F_BODY):
    surface.blit(font.render(str(value),True,color),pos)

def rounded(surface,rect,color,radius=9,border=None,width=1):
    pygame.draw.rect(surface,color,rect,border_radius=radius)
    if border:
        pygame.draw.rect(surface,border,rect,width,border_radius=radius)

def glow(surface,pos,radius,color,alpha=55):
    x,y=int(pos[0]),int(pos[1])
    layer=pygame.Surface((radius*6,radius*6),pygame.SRCALPHA)
    center=(radius*3,radius*3)
    for n in range(5,0,-1):
        pygame.draw.circle(layer,(*color,max(3,alpha//n)),center,max(1,radius*n//2))
    surface.blit(layer,(x-radius*3,y-radius*3))

def cell_center(col,row):
    return (ARENA.left+col*CELL+CELL//2,ARENA.top+row*CELL+CELL//2)

def pixel_cell(x,y):
    col=int((x-ARENA.left)//CELL)
    row=int((y-ARENA.top)//CELL)
    if 0<=col<COLS and 0<=row<ROWS:
        return col,row
    return None

def draw_background(t):
    screen.fill(BG)
    for i in range(12):
        x=(i*137+int(t*8))%WIDTH
        pygame.draw.line(screen,(8,24,42),(x,0),(x,HEIGHT),1)
    rounded(screen,pygame.Rect(20,16,WIDTH-40,78),(8,19,35),12,(25,75,110))
    txt(screen,"ROBOQUEST",(38,23),CYAN,F_LOGO)
    txt(screen,"AUTONOMOUS NAVIGATION LAB",(42,61),MUTED,F_TINY)
    txt(screen,"MISSION 02  /  INDUSTRIAL YARD",(420,38),GOLD,F_SMALL)
    pygame.draw.circle(screen,GREEN,(1194,43),5)
    txt(screen,"SYSTEM ONLINE",(1207,36),GREEN,F_TINY)

# Obstacles and hazards
class Crate:
    def __init__(self,c,r,warning=False):
        self.cell=(c,r)
        self.rect=pygame.Rect(ARENA.left+c*CELL+3,ARENA.top+r*CELL+3,CELL-6,CELL-6)
        self.warning=warning
    def draw(self,surface,t):
        r=self.rect
        pygame.draw.rect(surface,(2,6,13),r.move(2,3),border_radius=5)
        rounded(surface,r,(35,55,76),5,(115,151,177))
        inner=r.inflate(-8,-8)
        rounded(surface,inner,(17,34,53),3,(65,100,127))
        pygame.draw.line(surface,(87,126,153),inner.topleft,inner.bottomright,2)
        pygame.draw.line(surface,(87,126,153),inner.topright,inner.bottomleft,2)
        for p in ((r.left+5,r.top+5),(r.right-5,r.top+5),(r.left+5,r.bottom-5),(r.right-5,r.bottom-5)):
            pygame.draw.circle(surface,GOLD if self.warning else (143,174,195),p,2)
        if self.warning:
            pygame.draw.line(surface,ORANGE,(r.left+7,r.centery),(r.right-7,r.centery),2)

class LaserTurret:
    """Rotating laser with timed ON/OFF cycle and geometric beam collision."""
    def __init__(self,c,r,phase):
        self.x,self.y=cell_center(c,r)
        self.phase=phase
        self.range=102
        self.period=5.0
        self.active_duration=3.5

    def is_active(self,t):
        return ((t+self.phase*0.7) % self.period) < self.active_duration

    def beam(self,t):
        angle=self.phase+math.sin(t*.65+self.phase)*0.72
        end=(self.x+math.cos(angle)*self.range,
             self.y+math.sin(angle)*self.range)
        return (self.x,self.y),end

    def draw(self,surface,t):
        start,end=self.beam(t)
        active=self.is_active(t)
        beam=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
        if active:
            pygame.draw.line(beam,(255,35,72,28),start,end,18)
            pygame.draw.line(beam,(255,55,90,155),start,end,5)
            pygame.draw.line(beam,(255,220,225,235),start,end,1)
        else:
            pygame.draw.line(beam,(120,140,165,35),start,end,2)
        surface.blit(beam,(0,0))
        glow(surface,(self.x,self.y),13,RED if active else MUTED,65 if active else 25)
        pygame.draw.circle(surface,(28,34,53),(self.x,self.y),15)
        pygame.draw.circle(surface,(151,45,68) if active else (75,91,110),(self.x,self.y),12,3)
        pygame.draw.circle(surface,RED if active else MUTED,(self.x,self.y),5)
        pygame.draw.circle(surface,ICE if active else MUTED,(int(end[0]),int(end[1])),2)
        # The indicator changes color when the laser is active.
        pygame.draw.circle(surface,GREEN if not active else RED,(self.x,self.y-18),3)

class EnergyBarrier:
    def __init__(self,c,r,vertical=True):
        self.cell=(c,r)
        w,h=(9,CELL-8) if vertical else (CELL-8,9)
        self.rect=pygame.Rect(ARENA.left+c*CELL+(CELL-w)//2,
                              ARENA.top+r*CELL+(CELL-h)//2,w,h)
        self.vertical=vertical
    def draw(self,surface,t):
        pulse=.5+.5*math.sin(t*4)
        color=(int(110+70*pulse),int(65+45*pulse),255)
        fx=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
        pygame.draw.rect(fx,(*color,35),self.rect.inflate(15,15),border_radius=8)
        surface.blit(fx,(0,0))
        rounded(surface,self.rect,(48,29,99),5,color,2)
        if self.vertical:
            for y in range(self.rect.top+5,self.rect.bottom-2,8):
                pygame.draw.line(surface,ICE,(self.rect.left+2,y),(self.rect.right-2,y+2),1)
        else:
            for x in range(self.rect.left+5,self.rect.right-2,8):
                pygame.draw.line(surface,ICE,(x,self.rect.top+2),(x+2,self.rect.bottom-2),1)

class AppearingObstacle:
    """Obstacle with a visible warning phase, active phase, and timed removal."""
    WARNING_SECONDS = 1.8
    ACTIVE_SECONDS = 8.0

    def __init__(self, cell, now):
        self.cell = cell
        self.created_at = now
        self.active_at = now + self.WARNING_SECONDS
        self.expires_at = self.active_at + self.ACTIVE_SECONDS

    def phase(self, now):
        if now < self.active_at:
            return "WARNING"
        if now < self.expires_at:
            return "ACTIVE"
        return "EXPIRED"

    def draw(self, surface, now):
        col, row = self.cell
        rect = pygame.Rect(ARENA.left+col*CELL+3,
                           ARENA.top+row*CELL+3, CELL-6, CELL-6)
        state = self.phase(now)
        if state == "WARNING":
            remaining = max(0.0, self.active_at-now)
            pulse = 0.5 + 0.5*math.sin(now*13)
            color = (255, int(145+75*pulse), 55)
            glow(surface, rect.center, 13, color, 70)
            rounded(surface, rect, (65,43,17), 6, color, 2)
            pygame.draw.circle(surface, color, rect.center, 7, 2)
            label = F_TINY.render(f"{remaining:.1f}", True, ICE)
            surface.blit(label, label.get_rect(center=rect.center))
        else:
            glow(surface, rect.center, 12, RED, 65)
            rounded(surface, rect, (82,22,42), 5, RED, 2)
            inner = rect.inflate(-9,-9)
            rounded(surface, inner, (42,14,28), 3, (255,145,160), 1)
            pygame.draw.line(surface, (255,170,180), inner.topleft, inner.bottomright, 2)
            pygame.draw.line(surface, (255,170,180), inner.topright, inner.bottomleft, 2)
            pygame.draw.circle(surface, GOLD, (rect.centerx, rect.centery), 3)

# A* pathfinding
def heuristic(a,b):
    """Manhattan distance: exact lower-bound distance for 4-way grid motion."""
    return abs(a[0]-b[0])+abs(a[1]-b[1])

def astar(start,goal,blocked):
    """
    Find a shortest 4-direction route.

    Returns:
      path: list of grid cells from the next cell after start through goal.
      explored: cells removed from the priority queue and expanded.
      None path means no route exists.
    """
    if start in blocked or goal in blocked:
        return None, [], 0

    frontier=[]
    sequence=0
    heapq.heappush(frontier,(heuristic(start,goal),sequence,start))
    came_from={}
    g_score={start:0}
    explored=[]
    closed=set()

    directions=((1,0),(-1,0),(0,1),(0,-1))

    while frontier:
        _,_,current=heapq.heappop(frontier)
        if current in closed:
            continue
        if current==goal:
            path=[]
            while current!=start:
                path.append(current)
                current=came_from[current]
            path.reverse()
            return path,explored,len(explored)

        closed.add(current)
        explored.append(current)

        for dc,dr in directions:
            nxt=(current[0]+dc,current[1]+dr)
            if not (0<=nxt[0]<COLS and 0<=nxt[1]<ROWS):
                continue
            if nxt in blocked or nxt in closed:
                continue
            tentative=g_score[current]+1
            if tentative<g_score.get(nxt,float("inf")):
                came_from[nxt]=current
                g_score[nxt]=tentative
                sequence+=1
                f_score=tentative+heuristic(nxt,goal)
                heapq.heappush(frontier,(f_score,sequence,nxt))

    return None,explored,len(explored)

# Robot movement and drawing
class Robot:
    def __init__(self,start):
        self.start=start
        self.reset()
    def reset(self):
        self.x,self.y=cell_center(*self.start)
        self.heading=0.0
        self.distance=0.0
        self.collisions=0
        self.wheel_phase=0.0
        self.health=100
        self.invulnerable_until=0.0
        self.damage_flash_until=0.0
        self.hits=0
    def _try_axis(self,dx,dy,solid_rects):
        nx,ny=self.x+dx,self.y+dy
        body=pygame.Rect(int(nx-ROBOT_RADIUS),int(ny-ROBOT_RADIUS),
                         ROBOT_RADIUS*2,ROBOT_RADIUS*2)
        if not ARENA.contains(body) or any(body.colliderect(r) for r in solid_rects):
            self.collisions+=1
            return False
        self.x,self.y=nx,ny
        self.distance+=math.hypot(dx,dy)
        return True
    def move_vector(self,dx,dy,dt,solid_rects):
        if not(dx or dy): return
        mag=math.hypot(dx,dy); dx/=mag; dy/=mag
        self.heading=math.atan2(dy,dx)
        self._try_axis(dx*ROBOT_SPEED*dt,0,solid_rects)
        self._try_axis(0,dy*ROBOT_SPEED*dt,solid_rects)
        self.wheel_phase+=math.hypot(dx,dy)*ROBOT_SPEED*dt*.12
    def draw(self,surface,t):
        x,y=int(self.x),int(self.y)
        glow(surface,(x,y+3),18,CYAN,65)
        pygame.draw.ellipse(surface,(2,5,12),(x-23,y+12,46,13))
        for wx in (-15,15):
            wr=pygame.Rect(x+wx-5,y-13,10,26)
            rounded(surface,wr,(7,15,27),4,(75,119,147))
            for off in (-7,0,7):
                pygame.draw.line(surface,(75,110,134),(wr.left+2,y+off),(wr.right-2,y+off),1)
        body=pygame.Rect(x-16,y-20,32,40)
        rounded(surface,body,(30,73,103),9,(154,225,244),2)
        rounded(surface,body.inflate(-7,-8),(13,31,52),6,(61,137,171))
        pygame.draw.arc(surface,(190,230,242),(x-12,y-17,24,20),math.pi,2*math.pi,2)
        glow(surface,(x+7,y-7),7,CYAN,75)
        pygame.draw.circle(surface,(20,95,129),(x+7,y-7),5)
        pygame.draw.circle(surface,ICE,(x+8,y-8),2)
        pygame.draw.circle(surface,GREEN,(x-7,y+9),2)
        hx=x+math.cos(self.heading)*23; hy=y+math.sin(self.heading)*23
        pygame.draw.line(surface,GOLD,(x,y),(hx,hy),2)
        pygame.draw.circle(surface,GOLD,(int(hx),int(hy)),2)

# Sensor and collision calculations
def point_segment_distance(point,a,b):
    """Shortest distance from a point to a finite line segment."""
    px,py=point; ax,ay=a; bx,by=b
    dx,dy=bx-ax,by-ay
    denom=dx*dx+dy*dy
    if denom==0:
        return math.hypot(px-ax,py-ay)
    u=max(0.0,min(1.0,((px-ax)*dx+(py-ay)*dy)/denom))
    qx,qy=ax+u*dx,ay+u*dy
    return math.hypot(px-qx,py-qy)

def laser_contact(t):
    """Return the first active laser touching the rover, or None."""
    if elapsed_time < robot.invulnerable_until:
        return None
    for turret in turrets:
        if turret.is_active(t):
            a,b=turret.beam(t)
            if point_segment_distance((robot.x,robot.y),a,b) <= ROBOT_RADIUS+4:
                return turret
    return None

def apply_laser_hit():
    """Damage once, flash, and return to the last safe checkpoint (start)."""
    global replan_requested, search_status, game_over
    if elapsed_time < robot.invulnerable_until:
        return
    robot.health=max(0,robot.health-35)
    robot.hits+=1
    robot.damage_flash_until=elapsed_time+0.75
    robot.invulnerable_until=elapsed_time+2.0
    # Return the robot to the start so it is clear of the laser beam.
    robot.x,robot.y=cell_center(*START)
    robot.heading=0.0
    robot.collisions+=1
    replan_requested=True
    if robot.health<=0:
        game_over=True
        search_status="ROBOT DISABLED"

def laser_blocked_cells(t):
    """Rasterize active beam segments onto the grid for A* hazard avoidance."""
    blocked=set()
    for turret in turrets:
        if not turret.is_active(t):
            continue
        a,b=turret.beam(t)
        length=math.hypot(b[0]-a[0],b[1]-a[1])
        steps=max(1,int(length/5))
        for i in range(steps+1):
            u=i/steps
            x=a[0]+(b[0]-a[0])*u
            y=a[1]+(b[1]-a[1])*u
            cell=pixel_cell(x,y)
            if cell:
                # Block nearby cells as well, leaving a small safety margin.
                for dc in (-1,0,1):
                    for dr in (-1,0,1):
                        c,r=cell[0]+dc,cell[1]+dr
                        if 0<=c<COLS and 0<=r<ROWS:
                            blocked.add((c,r))
    return blocked

def lidar_scan(max_range=145,ray_count=24):
    """Simple 360-degree ray marching against solid rectangles and arena edge."""
    solids=solid_rectangles()
    hits=0
    rays=[]
    for i in range(ray_count):
        angle=(math.tau*i/ray_count)+elapsed_time*0.12
        end=(robot.x+math.cos(angle)*max_range,robot.y+math.sin(angle)*max_range)
        hit_point=end
        for distance in range(4,int(max_range)+1,4):
            px=robot.x+math.cos(angle)*distance
            py=robot.y+math.sin(angle)*distance
            probe=pygame.Rect(int(px)-2,int(py)-2,4,4)
            if not ARENA.contains(probe) or any(probe.colliderect(rect) for rect in solids):
                hit_point=(px,py)
                hits+=1
                break
        rays.append((end,hit_point))
    return rays,hits

def draw_lidar(surface):
    rays,hits=lidar_scan()
    layer=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
    for end,hit in rays:
        pygame.draw.line(layer,(40,210,255,24),(robot.x,robot.y),end,1)
        pygame.draw.line(layer,(45,220,255,105),(robot.x,robot.y),hit,1)
        if hit!=end:
            pygame.draw.circle(layer,(100,240,255,180),(int(hit[0]),int(hit[1])),2)
    surface.blit(layer,(0,0))
    return hits

# World setup
crates=[
    Crate(4,2),Crate(4,3,True),Crate(8,2),Crate(9,2),
    Crate(14,3,True),Crate(15,3),Crate(18,9),Crate(18,10,True),
    Crate(7,11),Crate(8,11),Crate(12,9),Crate(13,9,True)
]
turrets=[LaserTurret(10,5,.4),LaserTurret(17,6,2.2),LaserTurret(6,9,1.4)]
barriers=[EnergyBarrier(12,2,True),EnergyBarrier(16,11,False)]
robot=Robot(START)

# Manual obstacles are user-placed; appearing_obstacles are spawned by the game.
dynamic_obstacles=set()
appearing_obstacles={}
next_spawn_at=7.0
spawn_count=0
MAX_APPEARING_OBSTACLES=3
manual_mode=True
paused=False
won=False
game_over=False
show_help=True
start_ticks=pygame.time.get_ticks()

# Planner state.
planned_path=[]
explored_cells=[]
search_steps=0
search_status="NOT PLANNED"
path_index=0
replan_requested=True
auto_message="Press M for AUTO mode"

def static_blocked_cells():
    """Convert the fixed crate and energy gate locations into grid cells."""
    blocked={obj.cell for obj in crates}
    blocked.update(obj.cell for obj in barriers)
    return blocked

def appearing_blocked_cells():
    # Reserve warning cells so the robot can plan around them before activation.
    return {cell for cell, hazard in appearing_obstacles.items()
            if hazard.phase(elapsed_time) in ("WARNING", "ACTIVE")}

def all_blocked_cells():
    # Include every obstacle type in the pathfinding map.
    return (static_blocked_cells() | dynamic_obstacles |
            appearing_blocked_cells() | laser_blocked_cells(elapsed_time))

def solid_rectangles():
    rects=[obj.rect for obj in crates]+[obj.rect for obj in barriers]
    for c,r in dynamic_obstacles | appearing_blocked_cells():
        rects.append(pygame.Rect(ARENA.left+c*CELL+2,ARENA.top+r*CELL+2,CELL-4,CELL-4))
    return rects

def request_plan():
    global planned_path,explored_cells,search_steps,path_index,replan_requested,search_status
    start=pixel_cell(robot.x,robot.y)
    if start is None:
        planned_path=[]; explored_cells=[]; search_steps=0
        search_status="ROBOT OUTSIDE GRID"
        replan_requested=False
        return
    planned_path,explored_cells,search_steps=astar(start,GOAL,all_blocked_cells())
    path_index=0
    if planned_path is None:
        search_status="NO ROUTE AVAILABLE"
        planned_path=[]
    else:
        search_status="ROUTE FOUND"
    replan_requested=False

def current_waypoint():
    global path_index
    while path_index<len(planned_path):
        target=cell_center(*planned_path[path_index])
        if math.hypot(target[0]-robot.x,target[1]-robot.y)<5:
            path_index+=1
        else:
            return target
    return None

def update_autonomous(dt):
    global won,replan_requested,search_status
    if replan_requested:
        request_plan()
    if search_status!="ROUTE FOUND":
        return
    target=current_waypoint()
    if target is None:
        if pixel_cell(robot.x,robot.y)==GOAL:
            won=True
        return
    dx,dy=target[0]-robot.x,target[1]-robot.y
    distance=math.hypot(dx,dy)
    if distance<1:
        return
    step=min(ROBOT_SPEED*dt,distance)
    before=robot.collisions
    robot.move_vector(dx,dy,step/ROBOT_SPEED,solid_rectangles())
    if robot.collisions>before:
        # The map may have changed, so calculate a fresh route.
        replan_requested=True
    if pixel_cell(robot.x,robot.y)==GOAL:
        won=True

def add_dynamic_obstacle(cell):
    global replan_requested
    if cell in (START,GOAL):
        return
    if cell in dynamic_obstacles:
        dynamic_obstacles.remove(cell)
    elif cell not in static_blocked_cells():
        dynamic_obstacles.add(cell)
    replan_requested=True

def spawn_candidate_cells():
    """Prefer cells on the current route, but never spawn on/next to the rover."""
    current = pixel_cell(robot.x, robot.y)
    forbidden = (static_blocked_cells() | dynamic_obstacles |
                 set(appearing_obstacles) | {START, GOAL})
    candidates = []
    # Prefer a cell on the route so the robot has to replan visibly.
    for cell in planned_path[path_index:path_index+12]:
        if cell in forbidden or cell == current:
            continue
        if heuristic(cell, current) >= 3:
            candidates.append(cell)
    if candidates:
        return candidates
    # Otherwise, choose another free cell away from the robot.
    for row in range(ROWS):
        for col in range(COLS):
            cell=(col,row)
            if cell not in forbidden and cell != current and heuristic(cell,current)>=3:
                candidates.append(cell)
    return candidates

def spawn_appearing_obstacle(now):
    """Spawn only if A* can still find a route after reserving the candidate."""
    global replan_requested, spawn_count
    if len(appearing_obstacles) >= MAX_APPEARING_OBSTACLES:
        return False
    candidates = spawn_candidate_cells()
    # Cycle through candidates to keep the demo repeatable.
    if not candidates:
        return False

    cell = candidates[(spawn_count * 3) % len(candidates)]
    proposed = all_blocked_cells() | {cell}
    start = pixel_cell(robot.x, robot.y)
    test_path, _, _ = astar(start, GOAL, proposed)
    if test_path is None:
        # Search alternatives rather than trapping the robot.
        for candidate in candidates:
            test_path, _, _ = astar(start, GOAL, all_blocked_cells() | {candidate})
            if test_path is not None:
                cell = candidate
                break
        else:
            return False
    appearing_obstacles[cell] = AppearingObstacle(cell, now)
    spawn_count += 1
    replan_requested = True
    return True

def update_appearing_obstacles(now):
    """Handle warning-to-active transitions, timed removal, and spawn cadence."""
    global next_spawn_at, replan_requested
    expired = [cell for cell, hazard in appearing_obstacles.items()
               if hazard.phase(now) == "EXPIRED"]
    for cell in expired:
        del appearing_obstacles[cell]
        replan_requested = True

    # Replan when a warning finishes and the obstacle becomes active.
    if any(abs(now-hazard.active_at) < 0.04 for hazard in appearing_obstacles.values()):
        replan_requested = True

    if now >= next_spawn_at:
        if spawn_appearing_obstacle(now):
            next_spawn_at = now + 9.0
        else:
            # Try again shortly if no safe cell is available.
            next_spawn_at = now + 2.0

# Arena rendering
def draw_arena(t):
    rounded(screen,ARENA,(9,20,35),12,(37,115,158),2)
    for row in range(ROWS):
        for col in range(COLS):
            rect=pygame.Rect(ARENA.left+col*CELL,ARENA.top+row*CELL,CELL,CELL)
            pygame.draw.rect(screen,GRID_A if (row+col)%2 else GRID_B,rect)
            pygame.draw.rect(screen,GRID_LINE,rect,1)
            if (col+row*3)%7==0:
                pygame.draw.circle(screen,(35,65,88),(rect.left+5,rect.top+5),1)
    pygame.draw.rect(screen,(19,67,94),ARENA,4,border_radius=10)
    pygame.draw.rect(screen,(62,154,188),ARENA,1,border_radius=10)

    # Show the cells A* has already explored.
    for col,row in explored_cells:
        r=pygame.Rect(ARENA.left+col*CELL+4,ARENA.top+row*CELL+4,CELL-8,CELL-8)
        pygame.draw.rect(screen,(18,67,103),r,border_radius=5)
        pygame.draw.rect(screen,(45,137,190),r,1,border_radius=5)

    # Draw the current route from the robot to the goal.
    if planned_path and search_status=="ROUTE FOUND":
        route_points=[(int(robot.x),int(robot.y))]
        route_points.extend(cell_center(c,r) for c,r in planned_path[path_index:])
        if len(route_points)>1:
            route_layer=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA)
            pygame.draw.lines(route_layer,(20,185,255,45),False,route_points,10)
            pygame.draw.lines(route_layer,(54,220,255,210),False,route_points,3)
            screen.blit(route_layer,(0,0))
            for p in route_points[1:]:
                pygame.draw.circle(screen,ICE,p,2)

    sx,sy=cell_center(*START); gx,gy=cell_center(*GOAL)
    glow(screen,(sx,sy),16,GREEN,75)
    rounded(screen,pygame.Rect(sx-13,sy-13,26,26),(20,91,67),6,GREEN,2)
    txt(screen,"S",(sx-5,sy-8),ICE,F_SMALL)
    glow(screen,(gx,gy),17,GOLD,75)
    rounded(screen,pygame.Rect(gx-13,gy-13,26,26),(91,63,17),6,GOLD,2)
    txt(screen,"G",(gx-5,gy-8),ICE,F_SMALL)

    for obj in crates: obj.draw(screen,t)
    for obj in barriers: obj.draw(screen,t)
    for obj in turrets: obj.draw(screen,t)

    # Temporary obstacles show a warning before becoming solid.
    for hazard in appearing_obstacles.values():
        hazard.draw(screen, t)

    # Draw obstacles added by the user.
    for col,row in dynamic_obstacles:
        r=pygame.Rect(ARENA.left+col*CELL+3,ARENA.top+row*CELL+3,CELL-6,CELL-6)
        glow(screen,r.center,9,RED,40)
        rounded(screen,r,(79,23,43),5,RED,2)
        pygame.draw.line(screen,(255,151,164),r.topleft,r.bottomright,2)
        pygame.draw.line(screen,(255,151,164),r.topright,r.bottomleft,2)

    # Draw simulated LiDAR rays before drawing the robot.
    global lidar_hit_count
    lidar_hit_count=draw_lidar(screen)
    robot.draw(screen,t)
    # Briefly flash the robot after a laser hit.
    if elapsed_time < robot.damage_flash_until:
        pulse=8+int(5*abs(math.sin(elapsed_time*18)))
        pygame.draw.circle(screen,RED,(int(robot.x),int(robot.y)),20+pulse,3)

# Mission dashboard
def draw_hud():
    x=825
    rounded(screen,pygame.Rect(x,112,427,390),PANEL,12,BORDER)
    txt(screen,"MISSION TELEMETRY",(x+17,128),ICE,F_MED)
    mode_label="AUTONOMOUS" if not manual_mode else "MANUAL"
    mode_color=GREEN if not manual_mode else CYAN
    txt(screen,f"INDUSTRIAL YARD  /  {mode_label}",(x+18,155),mode_color,F_TINY)
    elapsed=(pygame.time.get_ticks()-start_ticks)/1000
    route_len=len(planned_path) if search_status=="ROUTE FOUND" else 0
    data=[
        ("MISSION TIME",f"{elapsed:05.1f} s",CYAN),
        ("DISTANCE",f"{robot.distance:05.0f} px",ICE),
        ("COLLISION ATTEMPTS",robot.collisions,GREEN if robot.collisions==0 else RED),
        ("ROBOT GRID CELL",pixel_cell(robot.x,robot.y),CYAN),
        ("PLANNED PATH",f"{route_len} cells",PATH),
        ("NODES EXPLORED",search_steps,ORANGE),
        ("LIDAR RETURNS",lidar_hit_count,CYAN),
        ("SPAWNED HAZARDS",len(appearing_obstacles),ORANGE),
        ("ROBOT HEALTH",f"{robot.health}%",GREEN if robot.health>50 else RED),
    ]
    y=180
    for label,value,color in data:
        pygame.draw.line(screen,(29,58,82),(x+17,y+31),(x+408,y+31),1)
        txt(screen,label,(x+19,y),MUTED,F_TINY)
        txt(screen,value,(x+260,y-2),color,F_BODY)
        y+=31
    # Robot health bar
    bar=pygame.Rect(x+260, y-1, 145, 9)
    rounded(screen,bar,(35,48,65),4)
    fill=bar.copy(); fill.width=int(bar.width*robot.health/100)
    if fill.width>0: rounded(screen,fill,GREEN if robot.health>50 else RED,4)

    rounded(screen,pygame.Rect(x,515,427,65),PANEL,12,BORDER)
    txt(screen,"NAVIGATION / HAZARD STATUS",(x+17,522),ICE,F_MED)
    status_color=GREEN if search_status=="ROUTE FOUND" else RED if ("NO ROUTE" in search_status or "DISABLED" in search_status) else GOLD
    txt(screen,search_status,(x+18,548),status_color,F_SMALL)
    txt(screen,"ORANGE = WARNING  /  RED = ACTIVE",(x+190,550),MUTED,F_TINY)

    rounded(screen,pygame.Rect(x,590,427,47),PANEL,12,BORDER)
    txt(screen,"M AUTO  •  P PLAN  •  CLICK MAP: OBSTACLE",(x+15,599),ICE,F_TINY)
    txt(screen,"WASD MOVE  •  SPACE PAUSE  •  R RESET  •  ESC QUIT",(x+15,619),MUTED,F_TINY)

def draw_footer():
    rounded(screen,pygame.Rect(28,653,1224,43),(8,19,34),9,(27,74,105))
    txt(screen,"ROBOTICS PROTOTYPE  •  BUILD 04",(43,668),CYAN,F_TINY)
    txt(screen,"A* + LiDAR + LASERS + APPEARING OBSTACLES",(430,667),MUTED,F_TINY)
    txt(screen,"60 FPS TARGET",(1138,667),GREEN,F_TINY)

def draw_overlay():
    if not (paused or won or game_over):
        return
    veil=pygame.Surface((WIDTH,HEIGHT),pygame.SRCALPHA); veil.fill((2,7,16,190)); screen.blit(veil,(0,0))
    rect=pygame.Rect(380,260,520,180)
    rounded(screen,rect,(11,27,47),16,GREEN if won else CYAN,2)
    title="MISSION COMPLETE" if won else ("ROBOT DISABLED" if game_over else "SIMULATION PAUSED")
    title_color=GREEN if won else RED if game_over else CYAN
    label=F_BIG.render(title,True,title_color)
    screen.blit(label,label.get_rect(center=(WIDTH//2,315)))
    msg="Press R to restart mission" if (won or game_over) else "Press SPACE to resume"
    label=F_BODY.render(msg,True,ICE); screen.blit(label,label.get_rect(center=(WIDTH//2,365)))

def draw_help():
    if show_help:
        txt(screen,"MISSION BRIEF",(30,704),CYAN,F_TINY)
        txt(screen,"M: AUTO  •  P: PLAN  •  CLICK: MANUAL OBSTACLE  •  ORANGE WARNING -> RED SPAWNED HAZARD.",
            (150,701),MUTED,F_TINY)

# Main event and update loop
running=True
elapsed_time=0.0
last_hazard_plan=0.0
last_laser_states=tuple(turret.is_active(elapsed_time) for turret in turrets)
lidar_hit_count=0
request_plan()

while running:
    dt=min(clock.tick(FPS)/1000.0,.04)
    elapsed_time+=dt

    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            running=False
        elif event.type==pygame.KEYDOWN:
            if event.key==pygame.K_ESCAPE:
                running=False
            elif event.key==pygame.K_SPACE and not (won or game_over):
                paused=not paused
            elif event.key==pygame.K_r:
                robot.reset()
                dynamic_obstacles.clear()
                appearing_obstacles.clear()
                next_spawn_at=elapsed_time+7.0
                spawn_count=0
                paused=False; won=False; game_over=False
                manual_mode=True
                start_ticks=pygame.time.get_ticks()
                replan_requested=True
            elif event.key==pygame.K_h:
                show_help=not show_help
            elif event.key==pygame.K_m:
                manual_mode=not manual_mode
                if not manual_mode:
                    replan_requested=True
            elif event.key==pygame.K_p:
                request_plan()
        elif event.type==pygame.MOUSEBUTTONDOWN and event.button==1:
            cell=pixel_cell(*event.pos)
            if cell:
                add_dynamic_obstacle(cell)

    if not paused and not (won or game_over):
        update_appearing_obstacles(elapsed_time)
        # Update the route when a laser changes state or while auto navigation runs.
        laser_states=tuple(turret.is_active(elapsed_time) for turret in turrets)
        if laser_states!=last_laser_states:
            replan_requested=True
            last_laser_states=laser_states
        if not manual_mode and elapsed_time-last_hazard_plan>0.65:
            replan_requested=True
            last_hazard_plan=elapsed_time
        if manual_mode:
            keys=pygame.key.get_pressed()
            dx=float(keys[pygame.K_d] or keys[pygame.K_RIGHT])-float(keys[pygame.K_a] or keys[pygame.K_LEFT])
            dy=float(keys[pygame.K_s] or keys[pygame.K_DOWN])-float(keys[pygame.K_w] or keys[pygame.K_UP])
            robot.move_vector(dx,dy,dt,solid_rectangles())
            if pixel_cell(robot.x,robot.y)==GOAL:
                won=True
        else:
            update_autonomous(dt)
        # Check laser contact in both manual and autonomous modes.
        if laser_contact(elapsed_time) is not None:
            apply_laser_hit()

    draw_background(elapsed_time)
    draw_arena(elapsed_time)
    draw_hud()
    draw_footer()
    draw_help()
    draw_overlay()
    pygame.display.flip()

pygame.quit()
sys.exit()
