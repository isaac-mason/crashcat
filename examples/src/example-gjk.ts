import GUI from 'lil-gui';
import { quat, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import {
    box,
    capsule,
    convexHull,
    createShapeSupportPool,
    createSimplex,
    createTransformedSupport,
    createTriangleSupport,
    getShapeSupportFunction,
    gjkClosestPoints,
    type ShapeSupportPool,
    type Simplex,
    type Support,
    SupportFunctionMode,
    setTransformedSupport,
    setTriangleSupport,
    sphere,
    registerAll,
} from 'crashcat';
import { loadGLTFPoints } from './utils/gltf.js';

registerAll();

const GUI_SHAPE_OPTIONS = {
    Sphere: 'sphere',
    Box: 'box',
    Capsule: 'capsule',
    Triangle: 'triangle',
    'Convex Hull (Suzanne)': 'convexhull',
} as const;

const GUI_CONVEX_RADIUS_MODES = {
    'Exclude Convex Radius': SupportFunctionMode.EXCLUDE_CONVEX_RADIUS,
    'Include Convex Radius': SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
    Default: SupportFunctionMode.DEFAULT,
} as const;

type ShapeOption = (typeof GUI_SHAPE_OPTIONS)[keyof typeof GUI_SHAPE_OPTIONS];

const config: {
    shapeA: ShapeOption;
    shapeB: ShapeOption;
    convexRadiusMode: SupportFunctionMode;
} = {
    shapeA: 'sphere',
    shapeB: 'box',
    convexRadiusMode: SupportFunctionMode.INCLUDE_CONVEX_RADIUS,
};

let isLoading = false;

const gui = new GUI();
gui.add(config, 'shapeA', GUI_SHAPE_OPTIONS).name('Shape A Type').onChange(setup);
gui.add(config, 'shapeB', GUI_SHAPE_OPTIONS).name('Shape B Type').onChange(setup);
gui.add(config, 'convexRadiusMode', GUI_CONVEX_RADIUS_MODES).name('Convex Radius Mode').onChange(setup);

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(5, 5, 5);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* setup example */

const TRANSFORM_MODES: ('translate' | 'rotate')[] = ['translate', 'rotate'];
let modeIndexA = 0;
let modeIndexB = 0;

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

const originGeometry = new THREE.SphereGeometry(0.15, 16, 16);
const originMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const originMarker = new THREE.Mesh(originGeometry, originMaterial);
scene.add(originMarker);

const controlsA = new TransformControls(camera, renderer.domElement);
controlsA.setMode('translate');
scene.add(controlsA.getHelper());

const controlsB = new TransformControls(camera, renderer.domElement);
controlsB.setMode('translate');
scene.add(controlsB.getHelper());

let state: ReturnType<typeof init>;

const suzannePoints = await loadGLTFPoints('./models/suzi.glb');

/* helper functions */

function getMinkowskiSupport(supportA: Support, supportB: Support, direction: Vec3, out: Vec3): void {
    const supportPointA = vec3.create();
    const supportPointB = vec3.create();
    const negDirection = vec3.create();

    supportA.getSupport(direction, supportPointA);
    vec3.negate(negDirection, direction);
    supportB.getSupport(negDirection, supportPointB);

    vec3.subtract(out, supportPointA, supportPointB);
}

function generateSphereDirections(count: number): Vec3[] {
    const directions: Vec3[] = [];
    const goldenRatio = (1 + Math.sqrt(5)) / 2;
    const angleIncrement = 2 * Math.PI * goldenRatio;

    for (let i = 0; i < count; i++) {
        const t = i / count;
        const phi = Math.acos(1 - 2 * t);
        const theta = angleIncrement * i;

        const x = Math.sin(phi) * Math.cos(theta);
        const y = Math.sin(phi) * Math.sin(theta);
        const z = Math.cos(phi);

        directions.push(vec3.fromValues(x, y, z));
    }

    return directions;
}

const MINKOWSKI_SAMPLE_COUNT = 100;

function updateMinkowskiPoints(instancedMesh: THREE.InstancedMesh, supportA: Support, supportB: Support): void {
    const directions = generateSphereDirections(MINKOWSKI_SAMPLE_COUNT);
    const supportPoint = vec3.create();

    const matrix = new THREE.Matrix4();
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3(1, 1, 1);

    for (let i = 0; i < MINKOWSKI_SAMPLE_COUNT; i++) {
        getMinkowskiSupport(supportA, supportB, directions[i], supportPoint);
        matrix.compose(position.set(supportPoint[0], supportPoint[1], supportPoint[2]), quaternion, scale);
        instancedMesh.setMatrixAt(i, matrix);
    }

    instancedMesh.instanceMatrix.needsUpdate = true;
    instancedMesh.count = MINKOWSKI_SAMPLE_COUNT;
}

function createSupportAndMesh(type: ShapeOption, supportPool: ShapeSupportPool): { support: Support; mesh: THREE.Mesh } {
    switch (type) {
        case 'sphere': {
            const sphereShape = sphere.create({ radius: 1.0 });
            const support = getShapeSupportFunction(supportPool, sphereShape, config.convexRadiusMode, vec3.fromValues(1, 1, 1));
            const geometry = new THREE.SphereGeometry(1.0, 32, 32);
            const material = new THREE.MeshStandardMaterial({ color: 0x00ff00, side: THREE.DoubleSide });
            const mesh = new THREE.Mesh(geometry, material);
            return { support, mesh };
        }
        case 'box': {
            const boxShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
            const support = getShapeSupportFunction(supportPool, boxShape, config.convexRadiusMode, vec3.fromValues(1, 1, 1));
            const geometry = new THREE.BoxGeometry(2, 2, 2);
            const material = new THREE.MeshStandardMaterial({ color: 0x00ff00, side: THREE.DoubleSide });
            const mesh = new THREE.Mesh(geometry, material);
            return { support, mesh };
        }
        case 'capsule': {
            const capsuleShape = capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 });
            const support = getShapeSupportFunction(supportPool, capsuleShape, config.convexRadiusMode, vec3.fromValues(1, 1, 1));
            const geometry = new THREE.CapsuleGeometry(0.5, 2.0, 16, 32);
            const material = new THREE.MeshStandardMaterial({ color: 0x00ff00, side: THREE.DoubleSide });
            const mesh = new THREE.Mesh(geometry, material);
            return { support, mesh };
        }
        case 'triangle': {
            const a = vec3.fromValues(0, 0, 0);
            const b = vec3.fromValues(1.5, 0, 0);
            const c = vec3.fromValues(0.75, 1.3, 0);
            const triangleSupport = createTriangleSupport();
            setTriangleSupport(triangleSupport, a, b, c);

            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute(
                'position',
                new THREE.BufferAttribute(new Float32Array([a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]]), 3),
            );
            geometry.setIndex(new THREE.BufferAttribute(new Uint32Array([0, 1, 2]), 1));
            geometry.computeVertexNormals();

            const material = new THREE.MeshStandardMaterial({ color: 0x00ff00, side: THREE.DoubleSide });
            const mesh = new THREE.Mesh(geometry, material);
            return { support: triangleSupport, mesh };
        }
        case 'convexhull': {
            const hullShape = convexHull.create({ positions: suzannePoints, convexRadius: 0.0 });
            const support = getShapeSupportFunction(supportPool, hullShape, config.convexRadiusMode, vec3.fromValues(1, 1, 1));

            // Create visual mesh from hull shape faces
            const geometry = new THREE.BufferGeometry();
            const positions: number[] = [];
            const indices: number[] = [];

            // Build geometry from faces
            for (const face of hullShape.faces) {
                const faceStart = face.firstVertex;
                const faceCount = face.numVertices;

                // Triangulate the face (assuming convex faces)
                const v0Index = hullShape.vertexIndices[faceStart];

                for (let i = 1; i < faceCount - 1; i++) {
                    const baseIndex = positions.length / 3;

                    const v0 = hullShape.points[v0Index].position;
                    const v1 = hullShape.points[hullShape.vertexIndices[faceStart + i]].position;
                    const v2 = hullShape.points[hullShape.vertexIndices[faceStart + i + 1]].position;

                    positions.push(v0[0], v0[1], v0[2]);
                    positions.push(v1[0], v1[1], v1[2]);
                    positions.push(v2[0], v2[1], v2[2]);

                    indices.push(baseIndex, baseIndex + 1, baseIndex + 2);
                }
            }

            geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
            geometry.setIndex(indices);
            geometry.computeVertexNormals();

            const material = new THREE.MeshStandardMaterial({
                color: 0x00ff00,
                side: THREE.DoubleSide,
                flatShading: false,
            });
            const mesh = new THREE.Mesh(geometry, material);

            return { support, mesh };
        }
    }
}

function init() {
    const supportPool = createShapeSupportPool();

    const { support: supportA, mesh: meshA } = createSupportAndMesh(config.shapeA, supportPool);
    meshA.position.set(-2, 0, 0);
    scene.add(meshA);
    controlsA.attach(meshA);

    const { support: supportB, mesh: meshB } = createSupportAndMesh(config.shapeB, supportPool);
    meshB.position.set(2, 0, 0);
    scene.add(meshB);
    controlsB.attach(meshB);

    const transformedSupportA = createTransformedSupport();
    const transformedSupportB = createTransformedSupport();

    // simplex viz
    const pointGeometry = new THREE.SphereGeometry(0.04, 16, 16);
    const pointMaterial = new THREE.MeshBasicMaterial({
        color: 0xffff00,
        transparent: true,
        opacity: 0.5,
        depthWrite: false,
        depthTest: false,
    });
    const simplexPoints = new THREE.InstancedMesh(pointGeometry, pointMaterial, 4);
    simplexPoints.count = 0;
    scene.add(simplexPoints);

    // minkowski points viz
    const geometry = new THREE.SphereGeometry(0.03, 8, 8);
    const material = new THREE.MeshBasicMaterial({
        color: 0x00ffff,
        transparent: true,
        opacity: 0.6,
        depthTest: false,
        depthWrite: false,
    });
    const minkowskiPoints = new THREE.InstancedMesh(geometry, material, MINKOWSKI_SAMPLE_COUNT);
    minkowskiPoints.count = 0;
    scene.add(minkowskiPoints);

    return {
        supportPool,
        supportA,
        supportB,
        meshA,
        meshB,
        transformedSupportA,
        transformedSupportB,
        simplexPoints,
        minkowskiPoints,
        simplexLines: [] as THREE.Line[],
    };
}

type ExampleState = ReturnType<typeof init>;

function dispose(state: ExampleState) {
    scene.remove(state.meshA);
    scene.remove(state.meshB);
    controlsA.detach();
    controlsB.detach();
    state.meshA.geometry.dispose();
    state.meshB.geometry.dispose();
    (state.meshA.material as THREE.MeshStandardMaterial).dispose();
    (state.meshB.material as THREE.MeshStandardMaterial).dispose();

    scene.remove(state.simplexPoints);
    state.simplexPoints.geometry.dispose();
    state.simplexPoints.material.dispose();

    scene.remove(state.minkowskiPoints);
    state.minkowskiPoints.geometry.dispose();
    (state.minkowskiPoints.material as THREE.MeshBasicMaterial).dispose();

    for (const line of state.simplexLines) {
        scene.remove(line);
        line.geometry.dispose();
        (line.material as THREE.LineBasicMaterial).dispose();
    }
}

function visualizeSimplex(state: ExampleState, simplex: Simplex) {
    // Hide all points by setting count to 0
    state.simplexPoints.count = 0;

    // Remove all lines
    for (const line of state.simplexLines) {
        scene.remove(line);
        line.geometry.dispose();
        (line.material as THREE.LineBasicMaterial).dispose();
    }
    state.simplexLines.length = 0;

    // Show simplex points (using .y for Minkowski difference)
    const matrix = new THREE.Matrix4();
    const position = new THREE.Vector3();
    const quaternion = new THREE.Quaternion();
    const scale = new THREE.Vector3(1, 1, 1);

    for (let i = 0; i < simplex.size; i++) {
        const y = simplex.points[i].y;
        matrix.compose(position.set(y[0], y[1], y[2]), quaternion, scale);
        state.simplexPoints.setMatrixAt(i, matrix);
    }
    state.simplexPoints.count = simplex.size;
    state.simplexPoints.instanceMatrix.needsUpdate = true;

    // Draw lines for simplex edges
    if (simplex.size === 2) {
        const y0 = simplex.points[0].y;
        const y1 = simplex.points[1].y;
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(y0[0], y0[1], y0[2]),
            new THREE.Vector3(y1[0], y1[1], y1[2]),
        ]);
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0xffff00 });
        const line = new THREE.Line(lineGeometry, lineMaterial);
        scene.add(line);
        state.simplexLines.push(line);
    } else if (simplex.size === 3) {
        const y0 = simplex.points[0].y;
        const y1 = simplex.points[1].y;
        const y2 = simplex.points[2].y;
        const lineGeometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(y0[0], y0[1], y0[2]),
            new THREE.Vector3(y1[0], y1[1], y1[2]),
            new THREE.Vector3(y2[0], y2[1], y2[2]),
            new THREE.Vector3(y0[0], y0[1], y0[2]),
        ]);
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0xffff00 });
        const line = new THREE.Line(lineGeometry, lineMaterial);
        scene.add(line);
        state.simplexLines.push(line);
    } else if (simplex.size === 4) {
        const y0 = simplex.points[0].y;
        const y1 = simplex.points[1].y;
        const y2 = simplex.points[2].y;
        const y3 = simplex.points[3].y;
        const edges = [
            [y0, y1],
            [y0, y2],
            [y0, y3],
            [y1, y2],
            [y1, y3],
            [y2, y3],
        ];
        for (const edge of edges) {
            const lineGeometry = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(edge[0][0], edge[0][1], edge[0][2]),
                new THREE.Vector3(edge[1][0], edge[1][1], edge[1][2]),
            ]);
            const lineMaterial = new THREE.LineBasicMaterial({ color: 0xffff00 });
            const line = new THREE.Line(lineGeometry, lineMaterial);
            scene.add(line);
            state.simplexLines.push(line);
        }
    }
}

function update(state: ExampleState) {
    setTransformedSupport(
        state.transformedSupportA,
        vec3.fromValues(state.meshA.position.x, state.meshA.position.y, state.meshA.position.z),
        quat.fromValues(state.meshA.quaternion.x, state.meshA.quaternion.y, state.meshA.quaternion.z, state.meshA.quaternion.w),
        state.supportA,
    );

    setTransformedSupport(
        state.transformedSupportB,
        vec3.fromValues(state.meshB.position.x, state.meshB.position.y, state.meshB.position.z),
        quat.fromValues(state.meshB.quaternion.x, state.meshB.quaternion.y, state.meshB.quaternion.z, state.meshB.quaternion.w),
        state.supportB,
    );

    // Update Minkowski point cloud
    updateMinkowskiPoints(state.minkowskiPoints, state.transformedSupportA, state.transformedSupportB);

    // Run GJK collision detection
    const gjkResult = {
        squaredDistance: 0,
        penetrationAxis: vec3.create(),
        pointA: vec3.create(),
        pointB: vec3.create(),
        simplex: createSimplex(),
    };
    const tolerance = 1e-4;
    const initialDirection = vec3.fromValues(1, 0, 0);
    gjkClosestPoints(
        gjkResult,
        state.transformedSupportA,
        state.transformedSupportB,
        tolerance,
        initialDirection,
        Number.MAX_VALUE,
    );
    const isColliding = gjkResult.squaredDistance <= tolerance * tolerance;

    // Visualize the final simplex
    visualizeSimplex(state, gjkResult.simplex);

    // Update colors based on collision
    const color = isColliding ? 0x00ff00 : 0xff0000;
    (state.meshA.material as THREE.MeshStandardMaterial).color.setHex(color);
    (state.meshB.material as THREE.MeshStandardMaterial).color.setHex(color);
}

function setup() {
    if (isLoading) return;

    isLoading = true;

    if (state) dispose(state);
    state = init();

    // Reset mode indices
    modeIndexA = 0;
    modeIndexB = 0;

    update(state);

    isLoading = false;
}

/* event handlers */

renderer.domElement.addEventListener('contextmenu', (event: MouseEvent) => {
    event.preventDefault();

    if (!state) return;

    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;

    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects([state.meshA, state.meshB]);

    if (intersects.length > 0) {
        const clickedMesh = intersects[0].object;

        if (clickedMesh === state.meshA) {
            modeIndexA = (modeIndexA + 1) % TRANSFORM_MODES.length;
            controlsA.setMode(TRANSFORM_MODES[modeIndexA]);
        } else if (clickedMesh === state.meshB) {
            modeIndexB = (modeIndexB + 1) % TRANSFORM_MODES.length;
            controlsB.setMode(TRANSFORM_MODES[modeIndexB]);
        }
    }
});

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

/* instructions panel */

const instructions = document.createElement('div');
instructions.style.position = 'absolute';
instructions.style.top = '10px';
instructions.style.left = '10px';
instructions.style.color = 'white';
instructions.style.fontFamily = 'monospace';
instructions.style.fontSize = '14px';
instructions.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
instructions.style.padding = '10px';
instructions.style.borderRadius = '5px';
instructions.innerHTML = `
<b>GJK Collision Detection</b><br>
<br>
<b>Controls:</b><br>
Right-click meshes to toggle translate/rotate<br>
Drag meshes to transform them<br>
<br>
<b>Shapes:</b><br>
Sphere, Box, Triangle<br>
<b>Convex Hull (Suzanne)</b>: 3D model loaded from GLB<br>
<br>
<b>Visualization:</b><br>
<span style="color: #00ff00;">Green</span>: Colliding<br>
<span style="color: #ff0000;">Red</span>: Not colliding<br>
<span style="color: #ffff00;">Yellow points/lines</span>: Final simplex<br>
<span style="color: #00ffff;">Cyan points</span>: Minkowski difference (A - B)<br>
<span style="color: #ff0000;">Red sphere</span>: Origin<br>
<br>
<b>Convex Radius Modes:</b><br>
<b>Exclude</b>: GJK mode - radius stored separately<br>
<b>Include</b>: Ray cast mode - radius in surface points<br>
<b>Default</b>: Same as Exclude
`;
document.body.appendChild(instructions);

setup();

controlsA.addEventListener('change', () => update(state));
controlsB.addEventListener('change', () => update(state));

function animate() {
    requestAnimationFrame(animate);

    update(state);

    orbitControls.enabled = !controlsA.dragging && !controlsB.dragging;
    orbitControls.update();
    renderer.render(scene, camera);
}

animate();
