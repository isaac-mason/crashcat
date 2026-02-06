import type { Vec3, Mat3 } from 'mathcat';
import { quat, vec3, mat3, mat4 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { RigidBody, Listener } from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    castRay,
    CastRayStatus,
    ConstraintSpace,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    distanceConstraint,
    enableCollision,
    filter,
    MotionType,
    rigidBody,
    swingTwistConstraint,
    updateWorld,
    massProperties,
    motionProperties,
    registerAllShapes,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* ragdoll configuration */

enum BodyPart {
    UPPER_BODY = 0,
    HEAD = 1,
    UPPER_LEFT_ARM = 2,
    LOWER_LEFT_ARM = 3,
    UPPER_RIGHT_ARM = 4,
    LOWER_RIGHT_ARM = 5,
    PELVIS = 6,
    UPPER_LEFT_LEG = 7,
    LOWER_LEFT_LEG = 8,
    UPPER_RIGHT_LEG = 9,
    LOWER_RIGHT_LEG = 10,
}

type ShapeConfig = {
    args: Vec3;
    density: number;
    position: Vec3;
};

type JointConfig = {
    pivotA: Vec3;
    pivotB: Vec3;
    axisA: Vec3;
    axisB: Vec3;
    angle: number;
    twistAngle: number;
};

type SkeletonJoint = {
    bodyPart: BodyPart;
    parentBodyPart: BodyPart | null;
};

type RagdollSettings = {
    shapes: Map<BodyPart, ShapeConfig>;
    joints: Record<string, JointConfig>;
    skeleton: SkeletonJoint[];
};

function createRagdollSettings(scale: number, angleA: number, angleB: number, twistAngle: number): RagdollSettings {
    const shouldersDistance = 0.45 * scale;
    const upperArmLength = 0.4 * scale;
    const lowerArmLength = 0.4 * scale;
    const upperArmSize = 0.15 * scale;
    const lowerArmSize = 0.15 * scale;
    const neckLength = 0.1 * scale;
    const headRadius = 0.2 * scale;
    const upperBodyLength = 0.6 * scale;
    const pelvisLength = 0.2 * scale;
    const pelvisSize = 0.25 * scale;
    const upperLegLength = 0.5 * scale;
    const upperLegSize = 0.15 * scale;
    const lowerLegSize = 0.15 * scale;
    const lowerLegLength = 0.5 * scale;

    // lower legs
    const lowerLeftLegPos: Vec3 = [-shouldersDistance / 3, lowerLegLength / 2, 0];
    const lowerRightLegPos: Vec3 = [shouldersDistance / 3, lowerLegLength / 2, 0];

    // upper legs
    const upperLeftLegPos: Vec3 = [-shouldersDistance / 3, lowerLeftLegPos[1] + lowerLegLength / 2 + upperLegLength / 2, 0];
    const upperRightLegPos: Vec3 = [shouldersDistance / 3, lowerRightLegPos[1] + lowerLegLength / 2 + upperLegLength / 2, 0];

    // pelvis
    const pelvisPos: Vec3 = [0, upperLeftLegPos[1] + upperLegLength / 2 + pelvisLength / 2, 0];

    // upper body
    const upperBodyPos: Vec3 = [0, pelvisPos[1] + pelvisLength / 2 + upperBodyLength / 2, 0];

    // head
    const headPos: Vec3 = [0, upperBodyPos[1] + upperBodyLength / 2 + headRadius / 2 + neckLength, 0];

    // upper arms (horizontal)
    const upperLeftArmPos: Vec3 = [-shouldersDistance / 2 - upperArmLength / 2, upperBodyPos[1] + upperBodyLength / 2, 0];
    const upperRightArmPos: Vec3 = [shouldersDistance / 2 + upperArmLength / 2, upperBodyPos[1] + upperBodyLength / 2, 0];

    // lower arms (horizontal)
    const lowerLeftArmPos: Vec3 = [upperLeftArmPos[0] - lowerArmLength / 2 - upperArmLength / 2, upperLeftArmPos[1], 0];
    const lowerRightArmPos: Vec3 = [upperRightArmPos[0] + lowerArmLength / 2 + upperArmLength / 2, upperRightArmPos[1], 0];

    const shapes = new Map<BodyPart, ShapeConfig>([
        [
            BodyPart.LOWER_LEFT_LEG,
            {
                args: [lowerLegSize * 0.5, lowerLegLength * 0.5, lowerLegSize * 0.5],
                density: scale,
                position: lowerLeftLegPos,
            },
        ],
        [
            BodyPart.LOWER_RIGHT_LEG,
            {
                args: [lowerLegSize * 0.5, lowerLegLength * 0.5, lowerLegSize * 0.5],
                density: scale,
                position: lowerRightLegPos,
            },
        ],
        [
            BodyPart.UPPER_LEFT_LEG,
            {
                args: [upperLegSize * 0.5, upperLegLength * 0.5, upperLegSize * 0.5],
                density: scale,
                position: upperLeftLegPos,
            },
        ],
        [
            BodyPart.UPPER_RIGHT_LEG,
            {
                args: [upperLegSize * 0.5, upperLegLength * 0.5, upperLegSize * 0.5],
                density: scale,
                position: upperRightLegPos,
            },
        ],
        [
            BodyPart.PELVIS,
            {
                args: [shouldersDistance * 0.5, pelvisLength * 0.5, pelvisSize * 0.5],
                density: scale,
                position: pelvisPos,
            },
        ],
        [
            BodyPart.UPPER_BODY,
            {
                args: [shouldersDistance * 0.5, upperBodyLength * 0.5, lowerArmSize * 0.75],
                density: scale,
                position: upperBodyPos,
            },
        ],
        [
            BodyPart.HEAD,
            {
                args: [headRadius * 0.6, headRadius * 0.7, headRadius * 0.6],
                density: scale,
                position: headPos,
            },
        ],
        [
            BodyPart.UPPER_LEFT_ARM,
            {
                args: [upperArmLength * 0.5, upperArmSize * 0.5, upperArmSize * 0.5],
                density: scale,
                position: upperLeftArmPos,
            },
        ],
        [
            BodyPart.UPPER_RIGHT_ARM,
            {
                args: [upperArmLength * 0.5, upperArmSize * 0.5, upperArmSize * 0.5],
                density: scale,
                position: upperRightArmPos,
            },
        ],
        [
            BodyPart.LOWER_LEFT_ARM,
            {
                args: [lowerArmLength * 0.5, lowerArmSize * 0.5, lowerArmSize * 0.5],
                density: scale,
                position: lowerLeftArmPos,
            },
        ],
        [
            BodyPart.LOWER_RIGHT_ARM,
            {
                args: [lowerArmLength * 0.5, lowerArmSize * 0.5, lowerArmSize * 0.5],
                density: scale,
                position: lowerRightArmPos,
            },
        ],
    ]);

    const joints: Record<string, JointConfig> = {
        neckJoint: {
            pivotA: [0, -headRadius - neckLength / 2, 0],
            pivotB: [0, upperBodyLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        leftKneeJoint: {
            pivotA: [0, lowerLegLength / 2, 0],
            pivotB: [0, -upperLegLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        rightKneeJoint: {
            pivotA: [0, lowerLegLength / 2, 0],
            pivotB: [0, -upperLegLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        leftHipJoint: {
            pivotA: [0, upperLegLength / 2, 0],
            pivotB: [-shouldersDistance / 3, -pelvisLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        rightHipJoint: {
            pivotA: [0, upperLegLength / 2, 0],
            pivotB: [shouldersDistance / 3, -pelvisLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        spineJoint: {
            pivotA: [0, pelvisLength / 2, 0],
            pivotB: [0, -upperBodyLength / 2, 0],
            axisA: [0, 1, 0],
            axisB: [0, 1, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        leftShoulder: {
            pivotA: [-shouldersDistance / 2, upperBodyLength / 2, 0],
            pivotB: [upperArmLength / 2, 0, 0],
            axisA: [1, 0, 0],
            axisB: [1, 0, 0],
            angle: angleB,
            twistAngle: twistAngle,
        },
        rightShoulder: {
            pivotA: [shouldersDistance / 2, upperBodyLength / 2, 0],
            pivotB: [-upperArmLength / 2, 0, 0],
            axisA: [1, 0, 0],
            axisB: [1, 0, 0],
            angle: angleB,
            twistAngle: twistAngle,
        },
        leftElbowJoint: {
            pivotA: [lowerArmLength / 2, 0, 0],
            pivotB: [-upperArmLength / 2, 0, 0],
            axisA: [1, 0, 0],
            axisB: [1, 0, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
        rightElbowJoint: {
            pivotA: [-lowerArmLength / 2, 0, 0],
            pivotB: [upperArmLength / 2, 0, 0],
            axisA: [1, 0, 0],
            axisB: [1, 0, 0],
            angle: angleA,
            twistAngle: twistAngle,
        },
    };

    // Define skeleton hierarchy
    const skeleton: SkeletonJoint[] = [
        { bodyPart: BodyPart.PELVIS, parentBodyPart: null }, // root
        { bodyPart: BodyPart.UPPER_BODY, parentBodyPart: BodyPart.PELVIS },
        { bodyPart: BodyPart.HEAD, parentBodyPart: BodyPart.UPPER_BODY },
        { bodyPart: BodyPart.UPPER_LEFT_ARM, parentBodyPart: BodyPart.UPPER_BODY },
        { bodyPart: BodyPart.LOWER_LEFT_ARM, parentBodyPart: BodyPart.UPPER_LEFT_ARM },
        { bodyPart: BodyPart.UPPER_RIGHT_ARM, parentBodyPart: BodyPart.UPPER_BODY },
        { bodyPart: BodyPart.LOWER_RIGHT_ARM, parentBodyPart: BodyPart.UPPER_RIGHT_ARM },
        { bodyPart: BodyPart.UPPER_LEFT_LEG, parentBodyPart: BodyPart.PELVIS },
        { bodyPart: BodyPart.LOWER_LEFT_LEG, parentBodyPart: BodyPart.UPPER_LEFT_LEG },
        { bodyPart: BodyPart.UPPER_RIGHT_LEG, parentBodyPart: BodyPart.PELVIS },
        { bodyPart: BodyPart.LOWER_RIGHT_LEG, parentBodyPart: BodyPart.UPPER_RIGHT_LEG },
    ];

    return { shapes, joints, skeleton };
}

function createRagdoll(
    world: ReturnType<typeof createWorld>,
    settings: RagdollSettings,
    options: {
        objectLayer: number;
        position?: Vec3;
        stabilize?: boolean;
    },
) {
    const bodies = new Map<BodyPart, RigidBody>();
    const offset = options.position ?? vec3.fromValues(0, 0, 0);

    // Create all body parts
    for (const [part, config] of settings.shapes) {
        const [halfW, halfH, halfD] = config.args;
        const position = vec3.fromValues(
            config.position[0] + offset[0],
            config.position[1] + offset[1],
            config.position[2] + offset[2],
        );

        const body = rigidBody.create(world, {
            shape: box.create({ halfExtents: vec3.fromValues(halfW, halfH, halfD), convexRadius: 0.05, density: config.density }),
            objectLayer: options.objectLayer,
            motionType: MotionType.DYNAMIC,
            position,
            quaternion: quat.create(),
            linearDamping: 0.05,
            angularDamping: 0.05,
            restitution: 0,
        });

        bodies.set(part, body);
    }

    // Apply stabilization if requested
    if (options.stabilize !== false) {
        stabilizeRagdoll(bodies, settings.skeleton);
    }

    // Create all joints
    const jointEntries = Object.entries(settings.joints) as [string, JointConfig][];
    for (const [jointName, jointConfig] of jointEntries) {
        // Determine which bodies to connect based on joint name
        let bodyA: RigidBody | undefined;
        let bodyB: RigidBody | undefined;

        if (jointName === 'neckJoint') {
            bodyA = bodies.get(BodyPart.HEAD);
            bodyB = bodies.get(BodyPart.UPPER_BODY);
        } else if (jointName === 'leftKneeJoint') {
            bodyA = bodies.get(BodyPart.LOWER_LEFT_LEG);
            bodyB = bodies.get(BodyPart.UPPER_LEFT_LEG);
        } else if (jointName === 'rightKneeJoint') {
            bodyA = bodies.get(BodyPart.LOWER_RIGHT_LEG);
            bodyB = bodies.get(BodyPart.UPPER_RIGHT_LEG);
        } else if (jointName === 'leftHipJoint') {
            bodyA = bodies.get(BodyPart.UPPER_LEFT_LEG);
            bodyB = bodies.get(BodyPart.PELVIS);
        } else if (jointName === 'rightHipJoint') {
            bodyA = bodies.get(BodyPart.UPPER_RIGHT_LEG);
            bodyB = bodies.get(BodyPart.PELVIS);
        } else if (jointName === 'spineJoint') {
            bodyA = bodies.get(BodyPart.PELVIS);
            bodyB = bodies.get(BodyPart.UPPER_BODY);
        } else if (jointName === 'leftShoulder') {
            bodyA = bodies.get(BodyPart.UPPER_BODY);
            bodyB = bodies.get(BodyPart.UPPER_LEFT_ARM);
        } else if (jointName === 'rightShoulder') {
            bodyA = bodies.get(BodyPart.UPPER_BODY);
            bodyB = bodies.get(BodyPart.UPPER_RIGHT_ARM);
        } else if (jointName === 'leftElbowJoint') {
            bodyA = bodies.get(BodyPart.LOWER_LEFT_ARM);
            bodyB = bodies.get(BodyPart.UPPER_LEFT_ARM);
        } else if (jointName === 'rightElbowJoint') {
            bodyA = bodies.get(BodyPart.LOWER_RIGHT_ARM);
            bodyB = bodies.get(BodyPart.UPPER_RIGHT_ARM);
        }

        if (!bodyA || !bodyB) {
            console.warn(`Could not find bodies for joint ${jointName}`);
            continue;
        }

        const position1 = vec3.fromValues(...jointConfig.pivotA);
        const position2 = vec3.fromValues(...jointConfig.pivotB);
        const twistAxis1 = vec3.fromValues(...jointConfig.axisA);
        const twistAxis2 = vec3.fromValues(...jointConfig.axisB);

        // Calculate plane axes (perpendicular to twist axes)
        const planeAxis1 = vec3.create();
        const planeAxis2 = vec3.create();

        const getTangent = (out: Vec3, axis: Vec3): Vec3 => {
            const ax = Math.abs(axis[0]);
            const ay = Math.abs(axis[1]);
            const az = Math.abs(axis[2]);

            if (ax <= ay && ax <= az) {
                vec3.set(out, 0, -axis[2], axis[1]);
            } else if (ay <= az) {
                vec3.set(out, axis[2], 0, -axis[0]);
            } else {
                vec3.set(out, -axis[1], axis[0], 0);
            }
            vec3.normalize(out, out);
            return out;
        };

        getTangent(planeAxis1, twistAxis1);
        getTangent(planeAxis2, twistAxis2);

        swingTwistConstraint.create(world, {
            bodyIdA: bodyA.id,
            bodyIdB: bodyB.id,
            position1,
            position2,
            twistAxis1,
            planeAxis1,
            twistAxis2,
            planeAxis2,
            space: ConstraintSpace.LOCAL,
            normalHalfConeAngle: jointConfig.angle,
            planeHalfConeAngle: jointConfig.angle,
            twistMinAngle: jointConfig.twistAngle !== undefined ? -jointConfig.twistAngle : 0,
            twistMaxAngle: jointConfig.twistAngle !== undefined ? jointConfig.twistAngle : 0,
        });
    }

    return { bodies };
}

const ragdollSettings = createRagdollSettings(1.8, Math.PI / 4, Math.PI / 4, 0);

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 5, 15);
camera.lookAt(0, 5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const onResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
};

window.addEventListener('resize', onResize);
onResize();

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* physics world */

registerAllShapes();

const worldSettings = createWorldSettings();
worldSettings.gravity = [0, -20, 0];

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NON_MOVING = addBroadphaseLayer(worldSettings);
const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_NON_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NON_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NON_MOVING);

const world = createWorld(worldSettings);

const listener: Listener = {
    onBodyPairValidate: (bodyA: RigidBody, bodyB: RigidBody): boolean => {
        // skip collision if bodies are connected by a constraint
        return !rigidBody.bodiesShareConstraint(bodyA, bodyB);
    },
};

/* orbit controls */

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.dampingFactor = 0.05;
orbitControls.target.set(0, 0, 0);

/* debug renderer */

const options = debugRenderer.createDefaultOptions();
options.bodies.enabled = true;
options.constraints.enabled = true;
options.constraints.drawLimits = true;
options.constraints.size = 0.2;
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui.title('Ragdoll Demo');

/* ground plane */

rigidBody.create(world, {
    shape: box.create({ halfExtents: vec3.fromValues(500, 1, 500) }),
    objectLayer: OBJECT_LAYER_NON_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, -5, 0),
});

/* ragdoll bodies */

const ragdolls: Array<ReturnType<typeof createRagdoll>> = [];

// create 3 ragdolls at increasing heights
for (let i = 0; i < 3; i++) {
    const ragdoll = createRagdoll(world, ragdollSettings, {
        objectLayer: OBJECT_LAYER_MOVING,
        position: vec3.fromValues(-5 + i * 4, i * 3, 0),
        stabilize: true,
    });
    ragdolls.push(ragdoll);
}

for (const ragdoll of ragdolls) {
    for (const body of ragdoll.bodies.values()) {
        rigidBody.addLinearVelocity(world, body, vec3.fromValues(0, 5, 0));
        rigidBody.addAngularVelocity(world, body, vec3.fromValues(0, 0, 20));
    }
}
/**
 * Stabilizes ragdoll mass and inertia properties.
 * from JoltPhysics - Based on "Stop my Constraints from Blowing Up!" - Oliver Strunk (Havok)
 * Does two things:
 * 1. Limits mass ratios between parents and children
 * 2. Increases parent inertia to be >= sum of children inertia
 */
function stabilizeRagdoll(bodies: Map<BodyPart, RigidBody>, skeleton: SkeletonJoint[]): void {
    const MIN_MASS_RATIO = 0.8;
    const MAX_MASS_RATIO = 1.2;
    const MAX_INERTIA_INCREASE = 2.0;

    // Track visited parts
    const visited = new Set<BodyPart>();
    const massRatios = new Map<BodyPart, number>();

    // Find root parts (those with no parent)
    const roots = skeleton.filter((joint) => joint.parentBodyPart === null);

    for (const root of roots) {
        // Process this chain
        const chain: BodyPart[] = [];
        const toProcess: BodyPart[] = [root.bodyPart];

        while (toProcess.length > 0) {
            const current = toProcess.shift()!;
            if (visited.has(current)) continue;

            visited.add(current);
            chain.push(current);

            // Find children
            for (const joint of skeleton) {
                if (joint.parentBodyPart === current && !visited.has(joint.bodyPart)) {
                    toProcess.push(joint.bodyPart);
                }
            }
        }

        if (chain.length === 1) continue;

        // Step 1: Ensure mass ratios are within range
        let totalMassRatio = 1.0;
        massRatios.set(chain[0], 1.0);

        for (let i = 1; i < chain.length; i++) {
            const childPart = chain[i];
            const parentPart = skeleton.find((j) => j.bodyPart === childPart)!.parentBodyPart!;

            const childBody = bodies.get(childPart)!;
            const parentBody = bodies.get(parentPart)!;

            const ratio = childBody.massProperties.mass / parentBody.massProperties.mass;
            const clampedRatio = Math.max(MIN_MASS_RATIO, Math.min(MAX_MASS_RATIO, ratio));

            const parentRatio = massRatios.get(parentPart)!;
            massRatios.set(childPart, parentRatio * clampedRatio);
            totalMassRatio += massRatios.get(childPart)!;
        }

        // Calculate total mass
        let totalMass = 0;
        for (const part of chain) {
            totalMass += bodies.get(part)!.massProperties.mass;
        }

        // Calculate mass scaling factor
        const ratioToMass = totalMass / totalMassRatio;

        // Adjust masses and inertias
        for (const part of chain) {
            const body = bodies.get(part)!;
            const oldMass = body.massProperties.mass;
            const newMass = massRatios.get(part)! * ratioToMass;

            body.massProperties.mass = newMass;

            // Scale inertia by mass ratio
            const massScale = newMass / oldMass;
            for (let i = 0; i < 15; i++) {
                body.massProperties.inertia[i] *= massScale;
            }
            body.massProperties.inertia[15] = 1.0;
        }

        // Step 2: Increase parent inertia based on children
        type Principal = {
            rotation: Mat3;
            diagonal: Vec3;
            childSum: number;
        };

        const principals = new Map<BodyPart, Principal>();

        // Decompose all inertia tensors
        for (const part of chain) {
            const body = bodies.get(part)!;
            const rotation = mat3.create();
            const diagonal = vec3.create();

            if (!motionProperties.decomposePrincipalMomentsOfInertia(body.massProperties.inertia, rotation, diagonal)) {
                console.warn(`Failed to decompose inertia tensor for body part ${part}`);
                continue;
            }

            principals.set(part, { rotation, diagonal, childSum: 0 });
        }

        // Calculate sum of child inertias (walk backwards)
        for (let i = chain.length - 1; i > 0; i--) {
            const childPart = chain[i];
            const parentPart = skeleton.find((j) => j.bodyPart === childPart)!.parentBodyPart!;

            const childPrincipal = principals.get(childPart);
            const parentPrincipal = principals.get(parentPart);

            if (childPrincipal && parentPrincipal) {
                parentPrincipal.childSum += childPrincipal.diagonal[0] + childPrincipal.childSum;
            }
        }

        // Adjust parent inertias
        for (const part of chain) {
            const principal = principals.get(part);
            if (!principal || principal.childSum === 0) continue;

            const body = bodies.get(part)!;

            // Calculate minimum inertia based on children
            const minimum = Math.min(MAX_INERTIA_INCREASE * principal.diagonal[0], principal.childSum);

            // Ensure all diagonal elements are at least the minimum
            principal.diagonal[0] = Math.max(principal.diagonal[0], minimum);
            principal.diagonal[1] = Math.max(principal.diagonal[1], minimum);
            principal.diagonal[2] = Math.max(principal.diagonal[2], minimum);

            // Reconstruct inertia tensor: I = R * D * R^T
            const scale = mat4.create();
            mat4.fromScaling(scale, principal.diagonal);

            const rot4x4 = mat4.create();
            mat4.identity(rot4x4);
            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    rot4x4[i + j * 4] = principal.rotation[i + j * 3];
                }
            }

            const temp1 = mat4.create();
            const temp2 = mat4.create();
            mat4.multiply(temp1, rot4x4, scale);

            // Transpose rotation
            mat4.transpose(temp2, rot4x4);

            // Final multiply
            mat4.multiply(body.massProperties.inertia, temp1, temp2);
            body.massProperties.inertia[15] = 1.0;
        }
    }

    // Update all motion properties with new mass/inertia
    for (const [_, body] of bodies) {
        if (body.motionType === MotionType.DYNAMIC) {
            // Re-set mass properties to update inverse mass and inertia
            const mp = massProperties.create();
            massProperties.copy(mp, body.massProperties);
            body.massPropertiesOverride = rigidBody.MassPropertiesOverride.MASS_AND_INERTIA_PROVIDED;

            // This will recalculate inverse mass and inverse inertia
            const tempMp = body.motionProperties;
            if (mp.mass > 0) {
                tempMp.invMass = 1.0 / mp.mass;
            } else {
                tempMp.invMass = 0;
            }

            // Decompose and set inverse inertia
            const rotation = mat3.create();
            const diagonal = vec3.create();
            if (motionProperties.decomposePrincipalMomentsOfInertia(mp.inertia, rotation, diagonal)) {
                vec3.set(
                    tempMp.invInertiaDiagonal,
                    diagonal[0] !== 0 ? 1.0 / diagonal[0] : 0,
                    diagonal[1] !== 0 ? 1.0 / diagonal[1] : 0,
                    diagonal[2] !== 0 ? 1.0 / diagonal[2] : 0,
                );
                quat.fromMat3(tempMp.inertiaRotation, rotation);
            }
        }
    }
}

/* pointer interaction */

let dragConstraint: distanceConstraint.DistanceConstraint | null = null;
let pointerBody: RigidBody | null = null;
const mouse = new THREE.Vector2();
const raycaster = new THREE.Raycaster();
const movementPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(100, 100),
    new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, side: THREE.DoubleSide }),
);
scene.add(movementPlane);

const pointerBodyMesh = new THREE.Mesh(new THREE.SphereGeometry(0.2), new THREE.MeshBasicMaterial({ color: 0xff0000 }));
pointerBodyMesh.visible = false;
scene.add(pointerBodyMesh);

// Create empty body for pointer interaction
pointerBody = rigidBody.create(world, {
    shape: null,
    objectLayer: OBJECT_LAYER_MOVING,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, -10000),
});

// Create ray collector and settings for pointer interaction
const rayCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
const queryFilter = filter.create(world.settings.layers);

/* pointer event handlers */

function updateMousePosition(event: PointerEvent) {
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
}

window.addEventListener('pointerdown', (event) => {
    updateMousePosition(event);
    raycaster.setFromCamera(mouse, camera);

    // Raycast
    const rayOrigin = vec3.fromValues(raycaster.ray.origin.x, raycaster.ray.origin.y, raycaster.ray.origin.z);
    const rayDirection = vec3.fromValues(raycaster.ray.direction.x, raycaster.ray.direction.y, raycaster.ray.direction.z);
    const rayLength = 1000;

    rayCollector.reset();
    castRay(world, rayCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

    if (rayCollector.hit.status === CastRayStatus.COLLIDING && pointerBody) {
        const hitBody = rigidBody.get(world, rayCollector.hit.bodyIdB);
        if (!hitBody) return;

        const fraction = rayCollector.hit.fraction;

        // Calculate hit point
        const hitPoint = new THREE.Vector3(
            rayOrigin[0] + rayDirection[0] * rayLength * fraction,
            rayOrigin[1] + rayDirection[1] * rayLength * fraction,
            rayOrigin[2] + rayDirection[2] * rayLength * fraction,
        );

        // Disable orbit controls while dragging
        orbitControls.enabled = false;

        // Position movement plane at hit point and orient it to face the camera
        movementPlane.position.copy(hitPoint);
        movementPlane.quaternion.copy(camera.quaternion);
        movementPlane.updateMatrixWorld();

        // Set pointer body to hit point
        rigidBody.setPosition(world, pointerBody, vec3.fromValues(hitPoint.x, hitPoint.y, hitPoint.z), false);

        // Create distance constraint at hit point
        const pointerPos = vec3.fromValues(hitPoint.x, hitPoint.y, hitPoint.z);

        dragConstraint = distanceConstraint.create(world, {
            bodyIdA: pointerBody.id,
            bodyIdB: hitBody.id,
            pointA: pointerPos,
            pointB: pointerPos,
            space: ConstraintSpace.WORLD,
            minDistance: 0,
            maxDistance: 1,
            constraintPriority: 100,
        });

        // Wake up the hit body
        rigidBody.wake(world, hitBody);

        // Show pointer body mesh
        pointerBodyMesh.visible = true;
    }
});

window.addEventListener('pointermove', (event) => {
    updateMousePosition(event);
});

window.addEventListener('pointerup', () => {
    if (dragConstraint) {
        distanceConstraint.remove(world, dragConstraint);
        dragConstraint = null;

        // Re-enable orbit controls
        orbitControls.enabled = true;

        // Hide pointer body mesh
        pointerBodyMesh.visible = false;
    }
});

/* simulation loop */

const maxDelta = 1 / 30;
let lastTime = performance.now();

function loop() {
    requestAnimationFrame(loop);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Update pointer body position based on mouse
    if (pointerBody && dragConstraint) {
        raycaster.setFromCamera(mouse, camera);
        const intersects = raycaster.intersectObject(movementPlane);

        if (intersects.length > 0) {
            const hitPoint = intersects[0].point;
            rigidBody.setPosition(world, pointerBody, vec3.fromValues(hitPoint.x, hitPoint.y, hitPoint.z), false);

            // Update visual mesh position
            pointerBodyMesh.position.copy(hitPoint);
        }
    }

    // Update orbit controls
    orbitControls.update();

    // Update physics
    debugUI.beginPerf(ui);
    updateWorld(world, listener, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    renderer.render(scene, camera);
}

loop();
