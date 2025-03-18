import * as THREE from 'three';

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff); // White background

// Define landmarks
const landmarks = [
    [8, 0],    // Right
    [5.66, 5.66],   // Top right
    [0, 8],    // Top
    [-5.66, 5.66],  // Top left
    [-8, 0],   // Left
    [-5.66, -5.66], // Bottom left
    [0, -8],   // Bottom
    [5.66, -5.66]   // Bottom right
];
// const n_landmarks = landmarks.length;

// Create landmarks visualization
function createLandmark(radius = 0.2) {
    const geometry = new THREE.CircleGeometry(radius, 32);
    const material = new THREE.MeshBasicMaterial({ 
        color: 0x00ffff,  // Cyan color (equivalent to RGB(0,255,255))
        side: THREE.DoubleSide 
    });
    return new THREE.Mesh(geometry, material);
}

// Add landmarks to scene
const landmarkObjects = landmarks.map(([x, y]) => {
    const landmark = createLandmark(0.4);
    landmark.position.set(x, y, 0);
    scene.add(landmark);
    return landmark;
});

// Create lines for landmark distances
const lineGeometry = new THREE.BufferGeometry();
const lineMaterial = new THREE.LineBasicMaterial({ 
    color: 0x808080,  // Grey color
    opacity: 0.5,
    transparent: true 
});
const landmarkLines = new THREE.LineSegments(lineGeometry, lineMaterial);
scene.add(landmarkLines);

// Function to update landmark distance lines
function updateLandmarkLines(measurements) {
    const linePoints = [];
    
    measurements.forEach(([dist, phi, lidx, is_in_fov]) => {
        if (is_in_fov === 1) {  // Check for 1.0 since we're getting floats from Python
            // Add car position as start point
            linePoints.push(car.position.x, car.position.y, 0);
            // Add landmark position as end point
            linePoints.push(landmarks[Math.floor(lidx)][0], landmarks[Math.floor(lidx)][1], 0);
        }
    });
    
    // Update line geometry
    const positions = new Float32Array(linePoints);
    landmarkLines.geometry.setAttribute('position', 
        new THREE.BufferAttribute(positions, 3)
    );
    landmarkLines.geometry.attributes.position.needsUpdate = true;
}

// Camera setup
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
// Move camera further up to get top-down 2D view
camera.position.z = 15;
// camera.rotation.z = -Math.PI / 2;

// Renderer setup
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Car geometry - create two halves of the car
const frontGeometry = new THREE.BoxGeometry(0.4, 0.4, 0.01); // Front half
const backGeometry = new THREE.BoxGeometry(0.4, 0.4, 0.01);  // Back half
const frontMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 }); // Red color
const backMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 }); // Black color

// Create front and back meshes
const frontHalf = new THREE.Mesh(frontGeometry, frontMaterial);
const backHalf = new THREE.Mesh(backGeometry, backMaterial);

// Position the halves
frontHalf.position.x = 0.2; // Move front half forward
backHalf.position.x = -0.2;   // Move back half backward

// Create a group to hold both halves
const car = new THREE.Group();
car.add(frontHalf);
car.add(backHalf);

// Create an ellipse around the car
function createEllipse(radiusX, radiusY) {
    const curve = new THREE.EllipseCurve(
        0, 0,            // Center x, y
        radiusX, radiusY,// xRadius, yRadius
        0, 2 * Math.PI,  // startAngle, endAngle
        false,           // clockwise
        0                // rotation
    );

    const points = curve.getPoints(50);
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({ color: 0x0000ff }); // Blue color
    const ellipse = new THREE.Line(geometry, material);
    
    // Create a group to handle scaling and rotation independently
    const ellipseGroup = new THREE.Group();
    ellipseGroup.add(ellipse);
    
    return ellipseGroup;
}

// Create the ellipse with initial dimensions
const ellipseGroup = createEllipse(1, 1); // Start with unit circle
// Add ellipse directly to scene instead of car group
scene.add(ellipseGroup);

// Set the car's initial rotation
car.rotation.z = Math.PI / 2;
scene.add(car);

// Car movement variables
const carSpeed = 0.05; // Reduced speed
const carRotationSpeed = 0.05;
let moveForward = false;
let moveBackward = false;
let turnLeft = false;
let turnRight = false;

// Add these variables at the top level
const API_URL = 'http://localhost:8000';
let lastUpdateTime = 0;
const UPDATE_INTERVAL = 16; // approximately 60fps

// Add state tracking variables at the top level
const n_state = 3;
const n_landmarks = 1;
let currentState = {
    position: [0, 0],
    heading: Math.PI/2,
    mu: null,
    sigma: null
};

// Add after the state tracking variables
const FRAME_RATE = 60;
const FRAME_INTERVAL = 1000 / FRAME_RATE;  // ~16.67ms
let lastFrameTime = 0;

// Initialize mu and sigma before animation loop
function initializeEKFState() {
    // Initialize mu (state estimate)
    currentState.mu = new Array(n_state + 2*n_landmarks).fill(0);
    currentState.mu[0] = 0;          // x position
    currentState.mu[1] = 0;          // y position
    currentState.mu[2] = Math.PI/2;  // heading

    // Initialize sigma (covariance matrix)
    currentState.sigma = Array(n_state + 2*n_landmarks).fill().map(() => 
        Array(n_state + 2*n_landmarks).fill(0)
    );
    
    // Set initial covariance for robot state
    for(let i = 0; i < n_state + 2*n_landmarks; i++) {
        currentState.sigma[i][i] = 100;
    }

    for(let i = 0; i < n_state; i++) {
        currentState.sigma[i][i] = 0.05;
    }
    // currentState.sigma[2][2] = 0;  // Zero uncertainty in heading

    currentState.eigenvals = [1, 1, 0];
    currentState.angle = 0;
}

// Call initialization before animation loop
initializeEKFState();

// Handle keyboard controls
document.addEventListener('keydown', (event) => {
    switch (event.key) {
        case 'ArrowUp':
            moveForward = true;
            updateRobotState('ArrowUp', 'keydown');
            break;
        case 'ArrowDown':
            moveBackward = true;
            updateRobotState('ArrowDown', 'keydown');
            break;
        case 'ArrowLeft':
            turnLeft = true;
            updateRobotState('ArrowLeft', 'keydown');
            break;
        case 'ArrowRight':
            turnRight = true;
            updateRobotState('ArrowRight', 'keydown');
            break;
    }
});

document.addEventListener('keyup', (event) => {
    switch (event.key) {
        case 'ArrowUp':
            moveForward = false;
            updateRobotState('ArrowUp', 'keyup');
            break;
        case 'ArrowDown':
            moveBackward = false;
            updateRobotState('ArrowDown', 'keyup');
            break;
        case 'ArrowLeft':
            turnLeft = false;
            updateRobotState('ArrowLeft', 'keyup');
            break;
        case 'ArrowRight':
            turnRight = false;
            updateRobotState('ArrowRight', 'keyup');
            break;
    }
});

// Update the animation loop
function animate(currentTime) {
    requestAnimationFrame(animate);

    // Calculate time since last frame
    const deltaTime = currentTime - lastFrameTime;

    // Only update if enough time has passed (1/60th of a second)
    if (deltaTime > FRAME_INTERVAL) {
        // Update last frame time
        lastFrameTime = currentTime - (deltaTime % FRAME_INTERVAL);

        // Send update to backend regardless of key state
        updateRobotState(null, 'frame_update');
    }

    renderer.render(scene, camera);
}

// Start animation with timestamp
animate(0);

// Modify updateRobotState to handle frame updates
async function updateRobotState(key, eventType) {
    try {
        const response = await fetch(`${API_URL}/update_robot`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                key: key || null,  // Will be null for frame updates
                type: eventType,   // Will be 'frame_update' for non-key updates
                current_state: {
                    position: currentState.position,
                    heading: currentState.heading,
                    mu: currentState.mu,
                    sigma: currentState.sigma,
                    eigenvals: currentState.eigenvals || [1, 1],
                    angle: currentState.angle || 0,
                    landmarks: landmarks
                }
            })
        });

        if (!response.ok) {
            throw new Error('Network response was not ok');
        }

        const data = await response.json();
        
        // Update car position and rotation
        car.position.x = data.position[0];
        car.position.y = data.position[1];
        car.rotation.z = data.heading;
        
        // Update landmark distance lines if measurements are provided
        if (data.measurements && Array.isArray(data.measurements)) {
            updateLandmarkLines(data.measurements);
        }
        
        // Update tracked state
        currentState.position = data.position;
        currentState.heading = data.heading;
        currentState.mu = data.mu;
        currentState.sigma = data.sigma;
        currentState.eigenvals = data.eigenvals;
        currentState.angle = data.angle;

        // Only update ellipse if eigenvals are present
        if (data.eigenvals && data.eigenvals.length >= 2) {
            // Update ellipse position to match car
            ellipseGroup.position.x = car.position.x;
            ellipseGroup.position.y = car.position.y;
            
            // Scale the ellipse based on eigenvalues
            const scaleX = Math.sqrt(Math.abs(data.eigenvals[0])) * 0.5;
            const scaleY = Math.sqrt(Math.abs(data.eigenvals[1])) * 0.5;
            ellipseGroup.scale.set(scaleX, scaleY, 1);

            // Rotation will now be independent of car's rotation
            ellipseGroup.rotation.z = (data.angle * Math.PI) / 180;
        }

        console.log('Updated State:', {
            position: data.position,
            heading: data.heading,
            mu: data.mu.slice(0, 3),
            sigma: data.sigma.slice(0, 3).map(row => row.slice(0, 3)),
            eigenvals: data.eigenvals,
            angle: data.angle
        });
    } catch (error) {
        console.error('Error updating robot state:', error);
    }
}

// Handle window resizing
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Add grid
const size = 100; // Total size of the grid
const divisions = size * 2; // Number of divisions (10x10 pixels each)
const gridHelper = new THREE.GridHelper(size, divisions, 0x888888, 0x888888);
gridHelper.material.opacity = 0.2;
gridHelper.material.transparent = true;
gridHelper.rotation.x = -Math.PI / 2;
scene.add(gridHelper);

// Add X and Z axes
const axesGeometry = new THREE.BufferGeometry();
const axesVertices = new Float32Array([
    // X axis (red)
    -size/2, 0, 0,
    size/2, 0, 0,
    // y axis (blue)
    0, -size/2, 0,
    0, size/2, 0,
]);
axesGeometry.setAttribute('position', new THREE.BufferAttribute(axesVertices, 3));
const axesMaterial = new THREE.LineBasicMaterial({ color: 0x000000, linewidth: 2 });
const axes = new THREE.LineSegments(axesGeometry, axesMaterial);
scene.add(axes);