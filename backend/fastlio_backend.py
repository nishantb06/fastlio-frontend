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
robot_fov = 5 # Field of view of robot

# ---> Noise parameters
R = np.diag([0.002,0.002,0.0005]) # sigma_x, sigma_y, sigma_theta
Q = np.diag([0.003,0.005]) # sigma_r, sigma_phi

# ---> EKF Estimation Variables
mu = np.zeros((n_state+2*n_landmarks,1)) # State estimate (robot pose and landmark positions)
sigma = np.zeros((n_state+2*n_landmarks,n_state+2*n_landmarks)) # State uncertainty, covariance matrix

# ---> Helpful matrix
Fx = np.block([[np.eye(3),np.zeros((n_state,2*n_landmarks))]]) # Used in both prediction and measurement updates


def sim_measurement(x, landmarks):
    '''
    This function simulates a measurement between robot and landmark
    Outputs:
     - zs: list of lists [dist, phi, lidx, is_in_fov]
    '''
    rx, ry, rtheta = x[0], x[1], x[2]
    zs = []  # List of measurements
    for lidx, landmark in enumerate(landmarks):
        lx, ly = landmark
        dist = float(np.linalg.norm(np.array([lx-rx, ly-ry])))  # Convert to float
        phi = float(np.arctan2(ly-ry, lx-rx) - rtheta)  # Convert to float
        phi = float(np.arctan2(np.sin(phi), np.cos(phi)))  # Keep phi bounded
        is_in_fov = dist < robot_fov
        zs.append([dist, phi, float(lidx), float(is_in_fov)])  # Convert all values to float/list
    return zs

def prediction_update(mu, sigma, u, dt):
    rx,py,theta = mu[0],mu[1],mu[2]
    v,w = u[0],u[1]
    # Update state estimate mu with model
    state_model_mat = np.zeros((n_state,1)) # Initialize state update matrix from model
    state_model_mat[0] = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Update in the robot x position
    state_model_mat[1] = (v/w)*np.cos(theta)-(v/w)*np.cos(theta+w*dt) if w>0.01 else v*np.sin(theta)*dt # Update in the robot y position
    state_model_mat[2] = w*dt # Update for robot heading theta
    mu = mu + np.matmul(np.transpose(Fx),state_model_mat) # Update state estimate, simple use model with current state estimate

    # Update state uncertainty sigma
    state_jacobian = np.zeros((3,3)) # Initialize model jacobian
    state_jacobian[0,2] = (v/w)*np.cos(theta) - (v/w)*np.cos(theta+w*dt) if w>0.01 else -v*np.sin(theta)*dt # Jacobian element, how small changes in robot theta affect robot x
    state_jacobian[1,2] = (v/w)*np.sin(theta) - (v/w)*np.sin(theta+w*dt) if w>0.01 else v*np.cos(theta)*dt # Jacobian element, how small changes in robot theta affect robot y
    G = np.eye(sigma.shape[0]) + np.transpose(Fx).dot(state_jacobian).dot(Fx) # How the model transforms uncertainty
    sigma = G.dot(sigma).dot(np.transpose(G)) + np.transpose(Fx).dot(R).dot(Fx) # Combine model effects and stochastic noise
    return mu,sigma

def sigma2transform(sigma):
    '''
    Finds the transform for a covariance matrix, to be used for visualizing the uncertainty ellipse
    '''
    [eigenvals,eigenvecs] = np.linalg.eig(sigma) # Finding eigenvalues and eigenvectors of the covariance matrix
    angle = 180.*np.arctan2(eigenvecs[1][0],eigenvecs[0][0])/np.pi # Find the angle of rotation for the first eigenvalue
    return eigenvals, angle

def measurement_update(mu, sigma, z, R):
    return mu,sigma
class RobotState(BaseModel):
    position: List[float]  # [x, y]
    heading: float  # theta
    mu: List[float]  # State estimate
    sigma: List[List[float]]  # Covariance matrix
    eigenvals: List[float]  # Eigenvalues for uncertainty ellipse
    angle: float  # Angle of uncertainty ellipse
    landmarks: Optional[List[List[float]]] = None  # Make landmarks optional with default None
    measurements: Optional[List[List[float]]] = None  # Make measurements optional with default None

class KeyEvent(BaseModel):
    key: Optional[str]  # Make key optional
    type: str  # 'keydown', 'keyup', or 'frame_update'
    current_state: RobotState

class DifferentialDrive:
    max_v = 3.0
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
    
    # Only update controls if this is a key event
    if key_event.type in ['keydown', 'keyup']:
        robot.update_controls(key_event)
    
    # Always move robot and update state
    robot.move_step(dt=0.016)  # Approximately 60fps

    landmarks = key_event.current_state.landmarks
    zs = sim_measurement(robot.x, landmarks)
    
    # Prediction Update
    mu, sigma = prediction_update(mu, sigma, robot.u, 0.016)
    eigenvals, angle = sigma2transform(sigma[0:2,0:2])
    
    return RobotState(
        position=[float(robot.x[0]), float(robot.x[1])],
        heading=float(robot.x[2]),
        mu=mu.flatten().tolist(),
        sigma=sigma.tolist(),
        eigenvals=eigenvals.tolist(),
        angle=angle,
        measurements=zs
    )
