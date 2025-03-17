from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import numpy as np
import scipy.integrate
from typing import List, Optional

app = FastAPI()

# Update CORS settings for your Amplify domain
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://your-amplify-domain.amplifyapp.com",  # Replace with your domain
        "http://localhost:5173",  # For local development
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---> Robot Parameters
n_state = 3 # Number of state variables
n_landmarks = 1 # Number of landmarks

# ---> EKF Estimation Variables
mu = np.zeros((n_state+2*n_landmarks,1)) # State estimate (robot pose and landmark positions)
sigma = np.zeros((n_state+2*n_landmarks,n_state+2*n_landmarks)) # State uncertainty, covariance matrix

# ---> Helpful matrix
Fx = np.block([[np.eye(3),np.zeros((n_state,2*n_landmarks))]]) # Used in both prediction and measurement updates

def prediction_update(mu, sigma, u, dt):
    rx,py,theta = mu[0],mu[1],mu[2]
    v,w = u[0],u[1]
    # Update state estimate mu with model
    state_model_mat = np.zeros((n_state,1)) # Initialize state update matrix from model
    state_model_mat[0] = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Update in the robot x position
    state_model_mat[1] = (v/w)*np.cos(theta)-(v/w)*np.cos(theta+w*dt) if w>0.01 else v*np.sin(theta)*dt # Update in the robot y position
    state_model_mat[2] = w*dt # Update for robot heading theta
    mu = mu + np.matmul(np.transpose(Fx),state_model_mat) # Update state estimate, simple use model with current state estimate

    return mu,sigma

def measurement_update(mu, sigma, z, R):
    return mu,sigma
class RobotState(BaseModel):
    position: List[float]  # [x, y]
    heading: float  # theta
    mu: List[float]  # State estimate
    sigma: List[List[float]]  # Covariance matrix

class KeyEvent(BaseModel):
    key: str
    type: str  # 'keydown' or 'keyup'
    current_state: RobotState

class DifferentialDrive:
    max_v = 2.0
    max_omega = 2.0
    
    def __init__(self):
        self.x = np.array([0.0, 0.0, np.pi/2])  # [x, y, theta]
        self.u = np.array([0.0, 0.0])  # [v, omega]
        
    def EOM(self, t, y):
        ydot = np.zeros(5)
        theta = y[2]
        v = max(min(y[3], self.max_v), -self.max_v)
        omega = max(min(y[4], self.max_omega), -self.max_omega)
        
        # Fix the equations to properly use heading
        # The car should move in the direction it's pointing
        # x' = v*cos(theta), y' = v*sin(theta)
        ydot[0] = v * np.cos(theta)  # x velocity
        ydot[1] = v * np.sin(theta)  # y velocity
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
    # Update robot state from frontend
    robot.x[0] = key_event.current_state.position[0]
    robot.x[1] = key_event.current_state.position[1]
    robot.x[2] = key_event.current_state.heading
    
    # Convert mu and sigma from frontend format to numpy arrays
    mu = np.array(key_event.current_state.mu).reshape(-1, 1)
    sigma = np.array(key_event.current_state.sigma)
    
    robot.update_controls(key_event)
    robot.move_step(dt=0.016)  # Approximately 60fps

    # Prediction Update
    mu, sigma = prediction_update(mu, sigma, robot.u, 0.016)
    print(mu)
    print(sigma)
    return RobotState(
        position=[float(robot.x[0]), float(robot.x[1])],
        heading=float(robot.x[2]),
        mu=mu.flatten().tolist(),  # Convert numpy array to list for JSON serialization
        sigma=sigma.tolist()
    )
