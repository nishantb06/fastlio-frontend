import * as THREE from 'three';

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff); // White background

// Camera setup
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
// Move camera further up to get top-down 2D view
camera.position.y = 10;
camera.rotation.x = -Math.PI / 2;

// Renderer setup
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Car geometry - create two halves of the car
const frontGeometry = new THREE.BoxGeometry(0.4, 0.01, 0.4); // Front half
const backGeometry = new THREE.BoxGeometry(0.4, 0.01, 0.4);  // Back half
const frontMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 }); // Red color
const backMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 }); // Black color

// Create front and back meshes
const frontHalf = new THREE.Mesh(frontGeometry, frontMaterial);
const backHalf = new THREE.Mesh(backGeometry, backMaterial);

// Position the halves
frontHalf.position.z = -0.2; // Move front half forward
backHalf.position.z = 0.2;   // Move back half backward

// Create a group to hold both halves
const car = new THREE.Group();
car.add(frontHalf);
car.add(backHalf);
scene.add(car);

// Car movement variables
const carSpeed = 0.05; // Reduced speed
const carRotationSpeed = 0.05;
let moveForward = false;
let moveBackward = false;
let turnLeft = false;
let turnRight = false;

// Handle keyboard controls
document.addEventListener('keydown', (event) => {
    switch (event.key) {
        case 'ArrowUp':
            moveForward = true;
            break;
        case 'ArrowDown':
            moveBackward = true;
            break;
        case 'ArrowLeft':
            turnLeft = true;
            break;
        case 'ArrowRight':
            turnRight = true;
            break;
    }
});

document.addEventListener('keyup', (event) => {
    switch (event.key) {
        case 'ArrowUp':
            moveForward = false;
            break;
        case 'ArrowDown':
            moveBackward = false;
            break;
        case 'ArrowLeft':
            turnLeft = false;
            break;
        case 'ArrowRight':
            turnRight = false;
            break;
    }
});

// Animation loop
function animate() {
    requestAnimationFrame(animate);

    // Handle car movement - car moves in the direction of its length
    if (moveForward) {
        car.position.x -= Math.sin(car.rotation.y) * carSpeed;
        car.position.z -= Math.cos(car.rotation.y) * carSpeed;
    }
    if (moveBackward) {
        car.position.x += Math.sin(car.rotation.y) * carSpeed;
        car.position.z += Math.cos(car.rotation.y) * carSpeed;
    }
    if (turnLeft) {
        car.rotation.y += carRotationSpeed;
    }
    if (turnRight) {
        car.rotation.y -= carRotationSpeed;
    }

    renderer.render(scene, camera);
}

// Handle window resizing
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Start the animation
animate(); 