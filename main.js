import * as THREE from 'three';

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff); // White background

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
    
    // No rotation applied - ellipse is already in XY plane
    // Points from EllipseCurve are in the XY plane by default
    
    return ellipse;
}

// Create the ellipse with fixed dimensions
const ellipse = createEllipse(1.2, 0.8); // Arbitrary fixed values
car.add(ellipse); // Add the ellipse to the car group

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
    for(let i = 0; i < 3; i++) {
        currentState.sigma[i][i] = 0.1;
    }
    currentState.sigma[2][2] = 0;  // Zero uncertainty in heading
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

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    if (moveForward || moveBackward || turnLeft || turnRight) {

        let activeKey = null;
        if (moveForward) activeKey = 'ArrowUp';
        else if (moveBackward) activeKey = 'ArrowDown';
        if (turnLeft) activeKey = 'ArrowLeft';
        else if (turnRight) activeKey = 'ArrowRight';
        if (activeKey) {
            updateRobotState(activeKey, 'keydown');
        }
    }

    renderer.render(scene, camera);
}

// Modify the updateRobotState function to handle mu and sigma
async function updateRobotState(key, eventType) {
    try {
        const response = await fetch(`${API_URL}/update_robot`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                key: key,
                type: eventType,
                current_state: {
                    position: currentState.position,
                    heading: currentState.heading,
                    mu: currentState.mu,
                    sigma: currentState.sigma
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
        
        // Update our tracked state including mu and sigma
        currentState.position = data.position;
        currentState.heading = data.heading;
        currentState.mu = data.mu;
        currentState.sigma = data.sigma;
        
        console.log('Updated State:', {
            position: data.position,
            heading: data.heading,
            mu: data.mu.slice(0, 3),  // Log only robot state part
            sigma: data.sigma.slice(0, 3).map(row => row.slice(0, 3))  // Log only robot state covariance
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

// Start the animation
animate();