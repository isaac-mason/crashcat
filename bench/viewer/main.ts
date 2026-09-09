import { debugRenderer } from 'crashcat/three';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type { Scenario, ScenarioInstance } from '../scenarios';
import { SCENARIOS } from '../scenarios';

// the viewer runs the same scenario modules the headless bench measures. nothing here is imported
// by the bench, so attaching a renderer cannot change what is measured.

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 2000);
camera.position.set(30, 25, 30);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
document.body.appendChild(renderer.domElement);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const directionalLight = new THREE.DirectionalLight(0xffffff, 1.2);
directionalLight.position.set(20, 40, 20);
scene.add(directionalLight);

function onResize(): void {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}
window.addEventListener('resize', onResize);
onResize();

const scenarioSelect = document.getElementById('scenario') as HTMLSelectElement;
const restartButton = document.getElementById('restart') as HTMLButtonElement;
const pauseButton = document.getElementById('pause') as HTMLButtonElement;
const stepButton = document.getElementById('step') as HTMLButtonElement;
const readout = document.getElementById('readout') as HTMLDivElement;
const description = document.getElementById('description') as HTMLDivElement;

for (const scenario of SCENARIOS) {
    const option = document.createElement('option');
    option.value = scenario.name;
    option.textContent = scenario.name;
    scenarioSelect.appendChild(option);
}

let debugState = debugRenderer.init();
scene.add(debugState.object3d);

let scenario: Scenario = SCENARIOS[0];
let instance: ScenarioInstance = scenario.create();
let stepIndex = 0;
let paused = false;
let lastStepMs = 0;

function load(name: string): void {
    scenario = SCENARIOS.find((s) => s.name === name) ?? SCENARIOS[0];
    scene.remove(debugState.object3d);
    debugRenderer.dispose(debugState);
    debugState = debugRenderer.init();
    scene.add(debugState.object3d);
    instance = scenario.create();
    stepIndex = 0;
    description.textContent = scenario.description;
    window.location.hash = scenario.name;
}

function advance(): void {
    const start = performance.now();
    instance.step(stepIndex);
    lastStepMs = performance.now() - start;
    stepIndex++;
}

scenarioSelect.addEventListener('change', () => load(scenarioSelect.value));
restartButton.addEventListener('click', () => load(scenario.name));
pauseButton.addEventListener('click', () => {
    paused = !paused;
    pauseButton.textContent = paused ? 'resume' : 'pause';
});
stepButton.addEventListener('click', () => {
    paused = true;
    pauseButton.textContent = 'resume';
    advance();
});

const initial = window.location.hash.slice(1);
scenarioSelect.value = SCENARIOS.some((s) => s.name === initial) ? initial : SCENARIOS[0].name;
load(scenarioSelect.value);

function frame(): void {
    requestAnimationFrame(frame);
    if (!paused) advance();
    debugRenderer.update(debugState, instance.world);
    orbitControls.update();
    renderer.render(scene, camera);
    readout.textContent = `step ${stepIndex} / ${scenario.steps} measured  ${lastStepMs.toFixed(2)} ms  ${instance.world.bodies.activeBodyCount} awake`;
}
frame();
