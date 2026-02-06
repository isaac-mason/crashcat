import {
    addBroadphaseLayer,
    addObjectLayer,
    createWorld,
    createWorldSettings,
    enableCollision,
    registerAllShapes,
    updateWorld
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import { vec3 } from 'mathcat';
import { GLTFLoader } from 'three/examples/jsm/Addons.js';
import * as THREE from 'three/webgpu';

/* load resources */
const gltfLoader = new GLTFLoader();
const [catGltf] = await Promise.all([gltfLoader.loadAsync('/cat.glb')]);

/* renderer, scene setup */
const container = document.getElementById('root')!;

// scene
const scene = new THREE.Scene();

// camera
const camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.1, 2000);
camera.position.set(0, 0, 5);

// renderer
const renderer = new THREE.WebGPURenderer({ antialias: true, alpha: true });
await renderer.init();
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

renderer.setSize(container.clientWidth, container.clientHeight);
renderer.setPixelRatio(window.devicePixelRatio);

container.appendChild(renderer.domElement);

function onWindowResize() {
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}
window.addEventListener('resize', onWindowResize);

// lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 1.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
directionalLight.position.set(-5, 10, 2);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
directionalLight.shadow.camera.near = 0.5;
directionalLight.shadow.camera.far = 100;
directionalLight.shadow.camera.left = -50;
directionalLight.shadow.camera.right = 50;
directionalLight.shadow.camera.top = 50;
directionalLight.shadow.camera.bottom = -50;
directionalLight.shadow.bias = -0.001;
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();

vec3.set(worldSettings.gravity, 0, -20, 0); // normal gravity
const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

const world = createWorld(worldSettings);

/* cat */

const catObject = catGltf.scene;
catObject.scale.setScalar(0.3);
catObject.traverse((child) => {
    if (child instanceof THREE.Mesh) {
        child.castShadow = true;
        child.receiveShadow = true;
    }
});
scene.add(catObject);

/* physics debug rendering */
const debugRendererState = debugRenderer.init();
debugRendererState.options.bodies.wireframe = true;
scene.add(debugRendererState.object3d);

/* debug toggle */
const toggleDebugButton = document.getElementById('debug-toggle')!;

const toggleDebug = () => {
    const isVisible = !debugRendererState.options.bodies.enabled;
    debugRendererState.options.bodies.enabled = isVisible;

    toggleDebugButton.textContent = isVisible ? 'Hide Debug View [D]' : 'Show Debug View [D]';
};

toggleDebugButton.addEventListener('click', toggleDebug);

window.addEventListener('keydown', (event) => {
    if (event.key === 'd' || event.key === 'D') {
        toggleDebug();
    }
});

/* loop */
let prevTime = performance.now();

function update() {
    const now = performance.now();
    const dt = (now - prevTime) / 1000;
    prevTime = now;

    updateWorld(world, undefined, dt);

    debugRenderer.update(debugRendererState, world);

    renderer.render(scene, camera);

    requestAnimationFrame(update);
}

document.querySelector('#loading')!.remove();

update();
