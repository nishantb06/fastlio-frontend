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

// Add this function to handle API calls
async function updateRobotState(key, eventType) {
    try {
        const response = await fetch(`${API_URL}/update_robot`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                key: key,
                type: eventType
            })
        });

        if (!response.ok) {
            throw new Error('Network response was not ok');
        }

        const data = await response.json();
        
        // Update car position and rotation
        car.position.x = data.position[0];
        car.position.y = data.position[1];  // Note: THREE.js uses z for depth
        car.rotation.z = data.heading;
        console.log(data.position[0], data.position[1], data.heading);
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