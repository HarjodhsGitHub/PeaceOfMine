#!/usr/bin/env python3
"""Multi-servo MX-64 controller for the official ArbotiX ROS firmware."""
from __future__ import annotations

import argparse
import queue
import threading
import time
from dataclasses import dataclass, field

import pygame
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from serial import SerialException

PORT = "/dev/cu.usbserial-AI049UTL"
HOST_BAUD, GATEWAY_ID, PROTOCOL = 115_200, 253, 1.0
BUS_BAUD = {1_000_000: 1, 57_600: 34}
CW, CCW, TORQUE, LED, GOAL, SPEED = 6, 8, 24, 25, 30, 32
POSITION, PRESENT_SPEED, LOAD, VOLTAGE, TEMP, MOVING = 36, 38, 40, 42, 43, 46
TICKS, JOG = 4095, 114
BG, PANEL, ALT, LINE = (13, 18, 27), (25, 34, 49), (35, 47, 66), (67, 83, 108)
TEXT, MUTED, BLUE, GREEN, RED, AMBER = (238, 243, 251), (158, 174, 196), (70, 160, 255), (62, 202, 132), (239, 94, 102), (246, 181, 66)


@dataclass
class Servo:
    id: int; baud: int; low: int = 0; high: int = TICKS; pos: int = 0; target: int = 0; speed: int = 80
    voltage: float = 0; temp: int = 0; load: int = 0; torque: bool = False; led: bool = False; moving: bool = False
    wheel: bool = False; detail: str = "Ready"


@dataclass
class State:
    connected: bool = False; detail: str = "Opening ArbotiX..."
    servos: dict[int, Servo] = field(default_factory=dict); selected: set[int] = field(default_factory=set)


class Worker:
    def __init__(self, initial):
        self.state, self.lock, self.stop = State(selected=initial), threading.Lock(), threading.Event()
        self.commands: queue.SimpleQueue[tuple[str, object]] = queue.SimpleQueue()
        self.thread = threading.Thread(target=self.run, daemon=True)
    def start(self): self.thread.start()
    def close(self): self.stop.set(); self.thread.join(2)
    def command(self, name, value=None): self.commands.put((name, value))
    def snap(self):
        with self.lock: return State(self.state.connected, self.state.detail, {i: Servo(**s.__dict__) for i,s in self.state.servos.items()}, set(self.state.selected))
    def update(self, **v):
        with self.lock:
            for k,x in v.items(): setattr(self.state,k,x)
    def patch(self, ident, **v):
        with self.lock:
            if ident in self.state.servos:
                for k,x in v.items(): setattr(self.state.servos[ident],k,x)
    @staticmethod
    def check(p,r,e):
        if r != COMM_SUCCESS: raise RuntimeError(p.getTxRxResult(r))
        if e: raise RuntimeError(p.getRxPacketError(e))
    def r1(self,p,h,i,a):
        v,r,e=p.read1ByteTxRx(h,i,a); self.check(p,r,e); return v
    def r2(self,p,h,i,a):
        v,r,e=p.read2ByteTxRx(h,i,a); self.check(p,r,e); return v
    def w1(self,p,h,i,a,v):
        r,e=p.write1ByteTxRx(h,i,a,v); self.check(p,r,e)
    def w2(self,p,h,i,a,v):
        r,e=p.write2ByteTxRx(h,i,a,v); self.check(p,r,e)
    def bus(self,p,h,baud): self.w1(p,h,GATEWAY_ID,4,BUS_BAUD[baud])
    def ready(self,p,h):
        end=time.monotonic()+5
        while time.monotonic()<end:
            v,r,e=p.read1ByteTxRx(h,GATEWAY_ID,0)
            if r==COMM_SUCCESS and not e and v==44: return
            self.stop.wait(.05)
        raise RuntimeError("ArbotiX ROS firmware did not answer within 5 seconds")
    def scan(self,p,h):
        found={}
        for baud in BUS_BAUD:
            self.update(detail=f"Scanning IDs at {baud:,} baud..."); self.bus(p,h,baud)
            for i in range(253):
                v,r,e=p.read1ByteTxRx(h,i,3)
                if r==COMM_SUCCESS and not e and v==i: found[i]=baud
        if not found: raise RuntimeError("No DYNAMIXEL found at 1 Mbps or 57,600 baud")
        data={}
        for i,b in found.items():
            self.bus(p,h,b); low,high=self.r2(p,h,i,CW),self.r2(p,h,i,CCW); pos=self.r2(p,h,i,POSITION)
            data[i]=Servo(i,b,low,high or TICKS,pos,pos,max(1,self.r2(p,h,i,SPEED)&1023),wheel=low==high==0)
        with self.lock:
            self.state.servos=data; self.state.selected &= set(data)
            if not self.state.selected: self.state.selected=set(data)
    def chosen(self):
        s=self.snap(); return [s.servos[i] for i in s.selected if i in s.servos]
    def mode(self,p,h,wheel):
        for s in self.chosen():
            self.bus(p,h,s.baud); self.w1(p,h,s.id,TORQUE,0); self.w2(p,h,s.id,CW,0); self.w2(p,h,s.id,CCW,0 if wheel else TICKS)
            low,high=self.r2(p,h,s.id,CW),self.r2(p,h,s.id,CCW)
            if (low==high==0)!=wheel: raise RuntimeError(f"ID {s.id}: mode verification failed")
            self.patch(s.id,low=low,high=high or TICKS,wheel=wheel,torque=False,detail=("Wheel" if wheel else "Joint")+" mode saved — torque off")
    def torque(self,p,h,on):
        for s in self.chosen():
            self.bus(p,h,s.baud)
            if on and not s.wheel:
                pos=self.r2(p,h,s.id,POSITION); self.w2(p,h,s.id,GOAL,pos); self.w2(p,h,s.id,SPEED,s.speed); self.patch(s.id,pos=pos,target=pos)
            self.w1(p,h,s.id,TORQUE,int(on)); self.patch(s.id,torque=on)
    def emergency_stop(self,p,h):
        """Stop and release every discovered actuator, regardless of selection."""
        for s in self.snap().servos.values():
            self.bus(p,h,s.baud)
            if s.wheel: self.w2(p,h,s.id,SPEED,0)
            self.w1(p,h,s.id,TORQUE,0)
            self.patch(s.id,torque=False,moving=False,detail="Emergency stop — torque off")
    def setting(self,p,h,name,value):
        for s in self.chosen():
            self.bus(p,h,s.baud)
            if name=="led": self.w1(p,h,s.id,LED,int(value)); self.patch(s.id,led=bool(value))
            elif name=="speed": self.w2(p,h,s.id,SPEED,value); self.patch(s.id,speed=value)
            elif name=="target" and not s.wheel:
                target=round(s.low+value*(s.high-s.low));
                if s.torque: self.w2(p,h,s.id,GOAL,target)
                self.patch(s.id,target=target)
            elif name=="jog" and not s.wheel:
                pos=self.r2(p,h,s.id,POSITION); target=max(s.low,min(s.high,pos+value)); self.w2(p,h,s.id,GOAL,target); self.w2(p,h,s.id,SPEED,s.speed); self.w1(p,h,s.id,TORQUE,1); self.patch(s.id,pos=pos,target=target,torque=True)
            elif name=="wheel" and s.wheel:
                self.w2(p,h,s.id,SPEED,0 if value==0 else s.speed|(1024 if value<0 else 0)); self.w1(p,h,s.id,TORQUE,1); self.patch(s.id,torque=True)
    def poll(self,p,h):
        for s in self.snap().servos.values():
            try:
                self.bus(p,h,s.baud); pos=self.r2(p,h,s.id,POSITION); load=self.r2(p,h,s.id,LOAD)
                self.patch(s.id,pos=pos,load=-(load&1023) if load&1024 else load&1023,voltage=self.r1(p,h,s.id,VOLTAGE)/10,temp=self.r1(p,h,s.id,TEMP),moving=bool(self.r1(p,h,s.id,MOVING)),torque=bool(self.r1(p,h,s.id,TORQUE)),led=bool(self.r1(p,h,s.id,LED)),detail="Ready")
            except RuntimeError as e: self.patch(s.id,detail=f"Read error: {e}")
    def dispatch(self,p,h):
        latest={}
        while True:
            try: k,v=self.commands.get_nowait(); latest[k]=v
            except queue.Empty: break
        if "selection" in latest:
            with self.lock: self.state.selected={int(i) for i in latest["selection"]}&set(self.state.servos)
        if "estop" in latest: self.emergency_stop(p,h)
        if not self.chosen(): return
        if "mode" in latest: self.mode(p,h,latest["mode"]=="wheel")
        if "torque" in latest: self.torque(p,h,bool(latest["torque"]))
        for k in ("led","speed","target","jog","wheel"):
            if k in latest: self.setting(p,h,k,latest[k])
    def run(self):
        while not self.stop.is_set():
            h,p=PortHandler(PORT),PacketHandler(PROTOCOL)
            try:
                self.update(connected=False,detail=f"Opening {PORT} at {HOST_BAUD:,} baud...")
                if not h.setBaudRate(HOST_BAUD): raise RuntimeError(f"Could not open {PORT}")
                self.update(detail="Waiting for ArbotiX ROS firmware..."); self.ready(p,h); self.scan(p,h); self.update(connected=True,detail="Connected")
                due=0.
                while not self.stop.is_set():
                    self.dispatch(p,h)
                    if time.monotonic()>=due: self.poll(p,h); due=time.monotonic()+.4
                    self.stop.wait(.01)
            except (RuntimeError, SerialException) as e: self.update(connected=False,detail=f"Disconnected: {e}"); self.stop.wait(1)
            finally:
                if h.is_open:
                    try: h.closePort()
                    except SerialException: pass


def clamp(v,l,h): return min(max(v,l),h)
def label(s,f,t,c,pos): s.blit(f.render(t,True,c),pos)
def box(s,r,c=PANEL): pygame.draw.rect(s,c,r,border_radius=13); pygame.draw.rect(s,LINE,r,1,border_radius=13)
def btn(s,r,title,sub,color=ALT): box(s,r,color); label(s,FONT,title,TEXT,(r.x+13,r.y+9)); label(s,SMALL,sub,MUTED,(r.x+13,r.y+34))
def rail(s,r,f,c):
    x=round(r.left+clamp(f,0,1)*r.width); pygame.draw.line(s,LINE,(r.left,r.centery),(r.right,r.centery),8); pygame.draw.line(s,c,(r.left,r.centery),(x,r.centery),8); pygame.draw.circle(s,TEXT,(x,r.centery),13); pygame.draw.circle(s,c,(x,r.centery),8)


def main():
    global FONT,SMALL
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument("--id",type=int,choices=range(253),action="append",help="initially select an ID; repeat to select several")
    args=parser.parse_args(); pygame.init(); window=pygame.display.set_mode((1280,820)); pygame.display.set_caption("ArbotiX-M MX-64 Controller")
    TITLE=pygame.font.Font(None,42); FONT=pygame.font.Font(None,25); LARGE=pygame.font.Font(None,34); SMALL=pygame.font.Font(None,20); clock=pygame.time.Clock(); worker=Worker(set(args.id or [])); worker.start()
    rail_target,rail_speed=pygame.Rect(340,240,870,28),pygame.Rect(340,580,510,28); cards={}; dragging=None; pending=None; last=0.; running=True
    try:
      while running:
        state=worker.snap(); selected=[state.servos[i] for i in state.selected if i in state.servos]; primary=selected[0] if selected else None; wheel=bool(selected) and all(s.wheel for s in selected); torque=bool(selected) and all(s.torque for s in selected); led=bool(selected) and all(s.led for s in selected)
        for e in pygame.event.get():
          if e.type==pygame.QUIT or(e.type==pygame.KEYDOWN and e.key in(pygame.K_ESCAPE,pygame.K_q)): running=False
          elif e.type==pygame.MOUSEBUTTONDOWN:
            hit=next((i for i,r in cards.items() if r.collidepoint(e.pos)),None)
            if hit is not None:
              new=set(state.selected); new.symmetric_difference_update({hit}); worker.command("selection",new)
            elif state.connected and selected and rail_target.inflate(0,28).collidepoint(e.pos) and not wheel: dragging="target"
            elif state.connected and selected and rail_speed.inflate(0,28).collidepoint(e.pos): dragging="speed"
            elif pygame.Rect(340,322,190,58).collidepoint(e.pos): worker.command("mode","joint")
            elif pygame.Rect(542,322,190,58).collidepoint(e.pos): worker.command("mode","wheel")
            elif pygame.Rect(744,322,220,58).collidepoint(e.pos): worker.command("torque",not torque)
            elif pygame.Rect(976,322,234,58).collidepoint(e.pos): worker.command("led",not led)
            elif pygame.Rect(1008,430,202,58).collidepoint(e.pos): worker.command("estop")
            elif pygame.Rect(340,430,170,58).collidepoint(e.pos): worker.command("wheel" if wheel else "jog",-1 if wheel else -JOG)
            elif pygame.Rect(522,430,170,58).collidepoint(e.pos): worker.command("wheel" if wheel else "jog",1 if wheel else JOG)
            elif wheel and pygame.Rect(704,430,170,58).collidepoint(e.pos): worker.command("wheel",0)
          elif e.type==pygame.MOUSEBUTTONUP: dragging=None
          elif e.type==pygame.MOUSEMOTION and dragging=="target": pending=("target",clamp((e.pos[0]-rail_target.left)/rail_target.width,0,1))
          elif e.type==pygame.MOUSEMOTION and dragging=="speed": pending=("speed",round(1+clamp((e.pos[0]-rail_speed.left)/rail_speed.width,0,1)*1022))
        if pending and time.monotonic()-last>.08: worker.command(*pending); pending=None; last=time.monotonic()
        window.fill(BG); label(window,TITLE,"ArbotiX-M  /  MX-64 Controller",TEXT,(28,25)); color=GREEN if state.connected else AMBER if any(x in state.detail.lower() for x in("waiting","scan","opening")) else RED; pygame.draw.circle(window,color,(35,76),6); label(window,SMALL,state.detail,MUTED,(51,66)); label(window,SMALL,"Detected servos",MUTED,(28,114)); cards={}
        for n,s in enumerate(state.servos.values()):
          r=pygame.Rect(22,144+n*74,286,62); cards[s.id]=r; box(window,r,BLUE if s.id in state.selected else PANEL); tick=pygame.Rect(r.x+12,r.y+18,22,22); pygame.draw.rect(window,GREEN if s.id in state.selected else BG,tick,border_radius=5); pygame.draw.rect(window,TEXT,tick,2,border_radius=5); label(window,FONT,f"ID {s.id}",TEXT,(r.x+47,r.y+10)); label(window,SMALL,f"{s.baud:,} baud  •  {'WHEEL' if s.wheel else 'JOINT'}  •  {s.voltage:.1f} V",MUTED,(r.x+47,r.y+36))
        if not state.servos: box(window,pygame.Rect(22,144,286,78)); label(window,SMALL,"Discovery and reconnect happen automatically.",MUTED,(35,163)); label(window,SMALL,"No controls are enabled until a servo is found.",MUTED,(35,188))
        box(window,pygame.Rect(326,112,930,694)); label(window,LARGE,"No servos selected" if not selected else "Controlling "+", ".join(f"ID {s.id}" for s in selected),TEXT,(340,132)); label(window,SMALL,"Check one or more servos; all commands apply to the checked set.",MUTED,(340,166)); label(window,FONT,"GOAL POSITION",MUTED,(340,205)); rail(window,rail_target,0 if not primary else (primary.target-primary.low)/max(1,primary.high-primary.low),BLUE); label(window,LARGE,"Wheel mode: continuous rotation" if wheel else ("—" if not primary else f"{primary.target*360/TICKS:.1f}°"),AMBER if wheel else TEXT,(340,270))
        btn(window,pygame.Rect(340,322,190,58),"JOINT MODE","0–360° position limits",GREEN); btn(window,pygame.Rect(542,322,190,58),"WHEEL MODE","Continuous rotation",AMBER); btn(window,pygame.Rect(744,322,220,58),"TORQUE ON" if torque else "TORQUE OFF","Toggle checked servos",GREEN if torque else RED); btn(window,pygame.Rect(976,322,234,58),"LED ON" if led else "LED OFF","Toggle checked servos",BLUE)
        label(window,FONT,"WHEEL JOG" if wheel else "JOINT JOG",MUTED,(340,402)); btn(window,pygame.Rect(340,430,170,58),"◀ CCW" if wheel else "−10°","Run" if wheel else "Jog + torque",AMBER if wheel else BLUE); btn(window,pygame.Rect(522,430,170,58),"CW ▶" if wheel else "+10°","Run" if wheel else "Jog + torque",AMBER if wheel else BLUE)
        if wheel: btn(window,pygame.Rect(704,430,170,58),"STOP","Set wheel speed to zero",RED)
        btn(window,pygame.Rect(1008,430,202,58),"E-STOP ALL","Disable torque on every servo",RED)
        label(window,FONT,"SPEED",MUTED,(340,545)); rail(window,rail_speed,((primary.speed if primary else 80)-1)/1022,AMBER); label(window,LARGE,f"{(primary.speed if primary else 80)*.114:.1f} rpm",TEXT,(875,566)); label(window,SMALL,"Joint: speed limit. Wheel: jog speed.",MUTED,(340,625))
        for n,s in enumerate(selected[:4]):
          r=pygame.Rect(340+n*218,682,205,100); box(window,r); label(window,SMALL,f"ID {s.id}  •  {s.detail}",MUTED,(r.x+12,r.y+11)); label(window,FONT,f"{s.pos*360/TICKS:.1f}°",TEXT,(r.x+12,r.y+37)); label(window,SMALL,f"{s.voltage:.1f} V  •  {s.temp}°C  •  {s.load/10.23:+.0f}%",MUTED,(r.x+12,r.y+68))
        pygame.display.flip(); clock.tick(60)
    finally: worker.close(); pygame.quit()


if __name__=="__main__": main()
