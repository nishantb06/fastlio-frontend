from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import numpy as np
import scipy.integrate
from typing import List, Optional

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class KeyEvent(BaseModel):
    key: str
    type: str  # 'keydown' or 'keyup'

class RobotState(BaseModel):
    position: List[float]  # [x, y]
    heading: float  # theta
    
class DifferentialDrive:
    max_v = 2.0
    max_omega = 2.0
    
    def __init__(self):
        self.x = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.u = np.array([0.0, 0.0])  # [v, omega]
        
    def EOM(self, t, y):
        ydot = np.zeros(5)
        theta = y[2]
        v = max(min(y[3], self.max_v), -self.max_v)
        omega = max(min(y[4], self.max_omega), -self.max_omega)
        
        # Fix the equations to properly use heading
        # The car should move in the direction it's pointing
        # x' = v*cos(theta), y' = v*sin(theta)
        ydot[0] = -v * np.sin(theta)  # x velocity
        ydot[1] = -v * np.cos(theta)  # y velocity
        ydot[2] = omega              # angular velocity (theta')
        ydot[3] = 0                  # v' = 0 (constant velocity)
        ydot[4] = 0                  # omega' = 0 (constant angular velocity)
        return ydot
    
    def move_step(self, dt):
        y = np.zeros(5)
        y[:3] = self.x
        y[3:] = self.u
        
        # Use smaller time steps for better numerical integration
        result = scipy.integrate.solve_ivp(
            self.EOM, 
            [0, dt], 
            y,
            method='RK45',  # Use Runge-Kutta 45 method
            max_step=dt/10  # Force smaller steps for better accuracy
        )
        
        self.x = result.y[:3, -1]
        # Keep angle between -pi and pi
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

    def update_controls(self, key_event: KeyEvent):
        if key_event.type == "keydown":
            if key_event.key == "ArrowLeft":
                self.u[1] = self.max_omega
            elif key_event.key == "ArrowRight":
                self.u[1] = -self.max_omega
            elif key_event.key == "ArrowUp":
                self.u[0] = self.max_v
            elif key_event.key == "ArrowDown":
                self.u[0] = -self.max_v
        elif key_event.type == "keyup":
            if key_event.key in ["ArrowLeft", "ArrowRight"]:
                self.u[1] = 0
            elif key_event.key in ["ArrowUp", "ArrowDown"]:
                self.u[0] = 0

# Create global robot instance
robot = DifferentialDrive()

@app.post("/update_robot")
async def update_robot(key_event: KeyEvent):
    robot.update_controls(key_event)
    robot.move_step(dt=0.016)  # Approximately 60fps
    
    return RobotState(
        position=[float(robot.x[0]), float(robot.x[1])],
        heading=float(robot.x[2])
    )
