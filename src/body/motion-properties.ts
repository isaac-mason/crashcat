import type { Mat3, Mat4, Quat, Vec3 } from 'mathcat';
import { mat3, quat, vec3 } from 'mathcat';
import { assert } from '../utils/assert';
import { DOF_ALL } from './dof';
import type { MassProperties } from './mass-properties';

/** motion quality options for collision detection */
export enum MotionQuality {
    /**
     * Discrete collision detection - standard discrete update.
     * Fast but can tunnel through thin geometry at high velocities.
     */
    DISCRETE = 0,

    /**
     * Continuous collision detection via linear cast.
     * Prevents fast-moving objects from tunneling through thin geometry
     * by casting shapes along their velocity vectors.
     * More expensive than DISCRETE.
     */
    LINEAR_CAST = 1,
}

/** motion properties of a rigid body */
export type MotionProperties = {
    /** linear velocity of the body */
    linearVelocity: Vec3;
    /** angular velocity of the body */
    angularVelocity: Vec3;

    /** inverse inertia diagonal */
    invInertiaDiagonal: Vec3;
    /** inertia rotation */
    inertiaRotation: Quat;

    /** force accumulator */
    force: Vec3;
    /** torque accumulator */
    torque: Vec3;

    /** inverse mass (1/mass) */
    invMass: number;
    /** linear damping coefficient */
    linearDamping: number;
    /** angular damping coefficient */
    angularDamping: number;
    /** maximum linear velocity */
    maxLinearVelocity: number;
    /** maximum angular velocity */
    maxAngularVelocity: number;
    /** gravity multiplier factor */
    gravityFactor: number;

    /** motion quality for collision detection */
    motionQuality: MotionQuality;
    /** allowed degrees of freedom bitmask */
    allowedDegreesOfFreedom: number;
    /** velocity solver iterations override (0 = use default) */
    numVelocityStepsOverride: number;
    /** position solver iterations override (0 = use default) */
    numPositionStepsOverride: number;

    /** spheres used for sleeping test */
    sleepTestSpheres: Array<{ center: Vec3; radius: number }>;
    /** whether the body is allowed to sleep */
    allowSleeping: boolean;
    /** timer for sleeping test */
    sleepTestTimer: number;
};

export function create(): MotionProperties {
    return {
        linearVelocity: vec3.create(),
        angularVelocity: vec3.create(),

        invInertiaDiagonal: vec3.create(),
        inertiaRotation: quat.create(),

        force: vec3.create(),
        torque: vec3.create(),

        invMass: 0,
        linearDamping: 0,
        angularDamping: 0,
        maxLinearVelocity: Infinity,
        maxAngularVelocity: Infinity,
        gravityFactor: 1,

        motionQuality: MotionQuality.DISCRETE,
        allowedDegreesOfFreedom: DOF_ALL,
        numVelocityStepsOverride: 0,
        numPositionStepsOverride: 0,

        sleepTestSpheres: [
            { center: vec3.create(), radius: 0 },
            { center: vec3.create(), radius: 0 },
            { center: vec3.create(), radius: 0 },
        ],
        allowSleeping: true,
        sleepTestTimer: 0,
    };
}

const _momentArm = /* @__PURE__ */ vec3.create();
const _torque = /* @__PURE__ */ vec3.create();
const _linearDelta = /* @__PURE__ */ vec3.create();
const _angularDelta = /* @__PURE__ */ vec3.create();

/** Adds a force to the force accumulator. */
export function addForce(motionProperties: MotionProperties, force: Vec3): void {
    vec3.add(motionProperties.force, motionProperties.force, force);
}

/** Adds a torque to the torque accumulator. */
export function addTorque(motionProperties: MotionProperties, torque: Vec3): void {
    vec3.add(motionProperties.torque, motionProperties.torque, torque);
}

/** Adds a force at a specific world-space position, generating both force and torque. */
export function addForceAtPosition(
    motionProperties: MotionProperties,
    force: Vec3,
    worldPosition: Vec3,
    centerOfMassPosition: Vec3,
): void {
    // add linear force
    vec3.add(motionProperties.force, motionProperties.force, force);
    // calculate torque: τ = r × F, where r is moment arm from center of mass
    vec3.subtract(_momentArm, worldPosition, centerOfMassPosition);
    vec3.cross(_torque, _momentArm, force);
    vec3.add(motionProperties.torque, motionProperties.torque, _torque);
}

/** Applies an impulse at the center of mass, instantly changing linear velocity. */
export function addImpulse(motionProperties: MotionProperties, impulse: Vec3): void {
    if (motionProperties.invMass <= 0) return;
    vec3.scale(_linearDelta, impulse, motionProperties.invMass);
    vec3.add(motionProperties.linearVelocity, motionProperties.linearVelocity, _linearDelta);
    clampLinearVelocity(motionProperties);
}

/** Applies an angular impulse, instantly changing angular velocity. */
export function addAngularImpulse(motionProperties: MotionProperties, impulse: Vec3, quaternion: Quat): void {
    const allowedRotationAxis = (motionProperties.allowedDegreesOfFreedom >> 3) & 0b111;
    if (allowedRotationAxis === 0) return;
    multiplyWorldSpaceInverseInertiaByVector(_angularDelta, motionProperties, quaternion, impulse);
    vec3.add(motionProperties.angularVelocity, motionProperties.angularVelocity, _angularDelta);
    clampAngularVelocity(motionProperties);
}

/** Applies an impulse at a specific world-space position, changing both linear and angular velocity. */
export function addImpulseAtPosition(
    motionProperties: MotionProperties,
    impulse: Vec3,
    worldPosition: Vec3,
    centerOfMassPosition: Vec3,
    quaternion: Quat,
): void {
    // apply linear impulse
    if (motionProperties.invMass > 0) {
        vec3.scale(_linearDelta, impulse, motionProperties.invMass);
        vec3.add(motionProperties.linearVelocity, motionProperties.linearVelocity, _linearDelta);
        clampLinearVelocity(motionProperties);
    }

    // apply angular impulse from moment arm: τ = r × impulse, Δω = I⁻¹ * τ
    const allowedRotationAxis = (motionProperties.allowedDegreesOfFreedom >> 3) & 0b111;

    if (allowedRotationAxis !== 0) {
        vec3.subtract(_momentArm, worldPosition, centerOfMassPosition);
        vec3.cross(_torque, _momentArm, impulse);
        multiplyWorldSpaceInverseInertiaByVector(_angularDelta, motionProperties, quaternion, _torque);
        vec3.add(motionProperties.angularVelocity, motionProperties.angularVelocity, _angularDelta);
        clampAngularVelocity(motionProperties);
    }
}

const EPSILON = 1e-10;
const MAX_JACOBI_ITERATIONS = 50;

const _decomposePrincipalMomentsOfInertia_mat3 = /* @__PURE__ */ mat3.create();
const _decomposePrincipalMomentsOfInertia_tempDiagonal = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_tempRotation = /* @__PURE__ */ mat3.create();
const _decomposePrincipalMomentsOfInertia_axisX = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_axisY = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_axisZ = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_cross = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_b = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_z = /* @__PURE__ */ vec3.create();
const _decomposePrincipalMomentsOfInertia_indices = /* @__PURE__ */ new Uint8Array(3);

/**
 * Decomposes the inertia tensor into principal moments (eigenvalues) and rotation matrix (eigenvectors)
 * Uses Jacobi's eigenvalue algorithm for 3x3 symmetric matrices
 *
 * Based on Jolt Physics EigenValueSymmetric implementation from Numerical Recipes paragraph 11.1
 * @see https://en.wikipedia.org/wiki/Eigenvalues_and_eigenvectors
 */
export function decomposePrincipalMomentsOfInertia(inertia: Mat4, outRotation: Mat3, outDiagonal: Vec3): boolean {
    // Extract 3x3 inertia tensor from 4x4 matrix
    // Mat4 is column-major: col0=[0,1,2,3], col1=[4,5,6,7], col2=[8,9,10,11], col3=[12,13,14,15]
    // Mat3 is column-major: col0=[0,1,2], col1=[3,4,5], col2=[6,7,8]
    // mat3.set takes: m00, m01, m02, m10, m11, m12, m20, m21, m22 where mXY = column X, row Y
    const m = mat3.set(
        _decomposePrincipalMomentsOfInertia_mat3,
        inertia[0],
        inertia[1],
        inertia[2], // column 0
        inertia[4],
        inertia[5],
        inertia[6], // column 1
        inertia[8],
        inertia[9],
        inertia[10], // column 2
    );

    // Initialize rotation matrix to identity
    mat3.identity(outRotation);

    // Initialize tracking arrays
    const b = _decomposePrincipalMomentsOfInertia_b;
    const z = _decomposePrincipalMomentsOfInertia_z;

    // Note: For column-major mat3, element at (row, col) is at index [col * 3 + row]
    // Diagonal elements: (0,0) at [0], (1,1) at [4], (2,2) at [8]
    for (let ip = 0; ip < 3; ip++) {
        // Initialize b to diagonal of matrix: a(ip, ip) = m[ip * 3 + ip]
        b[ip] = m[ip * 3 + ip];
        // Initialize output to diagonal of matrix
        outDiagonal[ip] = m[ip * 3 + ip];
        // Reset z
        z[ip] = 0;
    }

    // Jacobi eigenvalue iteration
    let converged = false;
    for (let sweep = 0; sweep < MAX_JACOBI_ITERATIONS; sweep++) {
        // Get the sum of the off-diagonal elements of a
        // For symmetric matrix, only need upper triangle: a(ip, iq) where ip < iq
        let sm = 0;
        for (let ip = 0; ip < 2; ip++) {
            for (let iq = ip + 1; iq < 3; iq++) {
                // a(ip, iq) = m[iq * 3 + ip] (column iq, row ip)
                sm += Math.abs(m[iq * 3 + ip]);
            }
        }
        const avgSm = sm / (3 * 3); // average off-diagonal magnitude: sm / n^2

        // Normal return, convergence to machine underflow
        if (avgSm < Number.MIN_VALUE) {
            converged = true;
            break;
        }

        // On the first three sweeps use a fraction of the sum of the off diagonal elements as threshold
        const thresh = sweep < 4 ? 0.2 * avgSm : Number.MIN_VALUE;

        for (let ip = 0; ip < 2; ip++) {
            for (let iq = ip + 1; iq < 3; iq++) {
                // a(ip, iq) = m[iq * 3 + ip]
                const a_pq_idx = iq * 3 + ip;
                const abs_a_pq = Math.abs(m[a_pq_idx]);
                const g = 100.0 * abs_a_pq;

                // After four sweeps, skip the rotation if the off-diagonal element is small
                if (
                    sweep > 4 &&
                    Math.abs(outDiagonal[ip]) + g === Math.abs(outDiagonal[ip]) &&
                    Math.abs(outDiagonal[iq]) + g === Math.abs(outDiagonal[iq])
                ) {
                    m[a_pq_idx] = 0;
                } else if (abs_a_pq > thresh) {
                    const h = outDiagonal[iq] - outDiagonal[ip];
                    const abs_h = Math.abs(h);

                    let t: number;
                    if (abs_h + g === abs_h) {
                        t = m[a_pq_idx] / h;
                    } else {
                        const theta = (0.5 * h) / m[a_pq_idx];
                        t = 1.0 / (Math.abs(theta) + Math.sqrt(1.0 + theta * theta));
                        if (theta < 0) t = -t;
                    }

                    const c = 1.0 / Math.sqrt(1.0 + t * t);
                    const s = t * c;
                    const tau = s / (1.0 + c);
                    const rotation_h = t * m[a_pq_idx];

                    m[a_pq_idx] = 0;

                    // Update change tracking
                    z[ip] -= rotation_h;
                    z[iq] += rotation_h;

                    outDiagonal[ip] -= rotation_h;
                    outDiagonal[iq] += rotation_h;

                    // Apply Jacobi rotation to matrix elements
                    // JPH_EVS_ROTATE(a, i, j, k, l): g = a(i,j), h = a(k,l), a(i,j) = g - s*(h + g*tau), a(k,l) = h + s*(g - h*tau)

                    // for j in [0, ip): ROTATE(a, j, ip, j, iq) - a(j, ip) and a(j, iq)
                    for (let j = 0; j < ip; j++) {
                        // a(j, ip) = m[ip * 3 + j], a(j, iq) = m[iq * 3 + j]
                        const idx_j_ip = ip * 3 + j;
                        const idx_j_iq = iq * 3 + j;
                        const g_val = m[idx_j_ip];
                        const h_val = m[idx_j_iq];
                        m[idx_j_ip] = g_val - s * (h_val + g_val * tau);
                        m[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                    // for j in [ip+1, iq): ROTATE(a, ip, j, j, iq) - a(ip, j) and a(j, iq)
                    for (let j = ip + 1; j < iq; j++) {
                        // a(ip, j) = m[j * 3 + ip], a(j, iq) = m[iq * 3 + j]
                        const idx_ip_j = j * 3 + ip;
                        const idx_j_iq = iq * 3 + j;
                        const g_val = m[idx_ip_j];
                        const h_val = m[idx_j_iq];
                        m[idx_ip_j] = g_val - s * (h_val + g_val * tau);
                        m[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                    // for j in [iq+1, n): ROTATE(a, ip, j, iq, j) - a(ip, j) and a(iq, j)
                    for (let j = iq + 1; j < 3; j++) {
                        // a(ip, j) = m[j * 3 + ip], a(iq, j) = m[j * 3 + iq]
                        const idx_ip_j = j * 3 + ip;
                        const idx_iq_j = j * 3 + iq;
                        const g_val = m[idx_ip_j];
                        const h_val = m[idx_iq_j];
                        m[idx_ip_j] = g_val - s * (h_val + g_val * tau);
                        m[idx_iq_j] = h_val + s * (g_val - h_val * tau);
                    }

                    // Apply rotation to eigenvector matrix
                    // for j in [0, n): ROTATE(outEigVec, j, ip, j, iq) - outEigVec(j, ip) and outEigVec(j, iq)
                    for (let j = 0; j < 3; j++) {
                        // outEigVec(j, ip) = outRotation[ip * 3 + j], outEigVec(j, iq) = outRotation[iq * 3 + j]
                        const idx_j_ip = ip * 3 + j;
                        const idx_j_iq = iq * 3 + j;
                        const g_val = outRotation[idx_j_ip];
                        const h_val = outRotation[idx_j_iq];
                        outRotation[idx_j_ip] = g_val - s * (h_val + g_val * tau);
                        outRotation[idx_j_iq] = h_val + s * (g_val - h_val * tau);
                    }
                }
            }
        }

        // Update eigenvalues with the sum and reinitialize z
        for (let ip = 0; ip < 3; ip++) {
            b[ip] += z[ip];
            outDiagonal[ip] = b[ip];
            z[ip] = 0;
        }
    }

    // Sort eigenvalues so that the biggest value goes first (descending order)
    const indices = _decomposePrincipalMomentsOfInertia_indices;
    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;

    // Bubble sort 3 elements, descending by eigenvalue
    if (outDiagonal[indices[0]] < outDiagonal[indices[1]]) {
        const temp = indices[0];
        indices[0] = indices[1];
        indices[1] = temp;
    }
    if (outDiagonal[indices[1]] < outDiagonal[indices[2]]) {
        const temp = indices[1];
        indices[1] = indices[2];
        indices[2] = temp;
    }
    if (outDiagonal[indices[0]] < outDiagonal[indices[1]]) {
        const temp = indices[0];
        indices[0] = indices[1];
        indices[1] = temp;
    }

    // Reorder eigenvalues and eigenvectors
    const tempDiagonal = _decomposePrincipalMomentsOfInertia_tempDiagonal;
    const tempRotation = _decomposePrincipalMomentsOfInertia_tempRotation;
    vec3.copy(tempDiagonal, outDiagonal);
    mat3.copy(tempRotation, outRotation);

    for (let i = 0; i < 3; i++) {
        outDiagonal[i] = tempDiagonal[indices[i]];
        // Copy column indices[i] from tempRotation to column i of outRotation
        // Column c is at indices [c*3, c*3+1, c*3+2]
        for (let j = 0; j < 3; j++) {
            outRotation[i * 3 + j] = tempRotation[indices[i] * 3 + j];
        }
    }

    // Make sure that the rotation matrix is a right handed matrix
    // Column 0 = axisX, Column 1 = axisY, Column 2 = axisZ
    const axisX = _decomposePrincipalMomentsOfInertia_axisX;
    const axisY = _decomposePrincipalMomentsOfInertia_axisY;
    const axisZ = _decomposePrincipalMomentsOfInertia_axisZ;
    vec3.set(axisX, outRotation[0], outRotation[1], outRotation[2]); // column 0
    vec3.set(axisY, outRotation[3], outRotation[4], outRotation[5]); // column 1
    vec3.set(axisZ, outRotation[6], outRotation[7], outRotation[8]); // column 2

    const cross = _decomposePrincipalMomentsOfInertia_cross;
    vec3.cross(cross, axisX, axisY);
    if (vec3.dot(cross, axisZ) < 0) {
        // Flip Z axis to make it right-handed
        outRotation[6] = -outRotation[6];
        outRotation[7] = -outRotation[7];
        outRotation[8] = -outRotation[8];
    }

    // Return true if converged, false if max iterations exceeded
    return converged;
}

const _setMassProperties_rotation = /* @__PURE__ */ mat3.create();
const _setMassProperties_diagonal = /* @__PURE__ */ vec3.create();

export function setMassProperties(motionProperties: MotionProperties, allowedDOFs: number, massProperties: MassProperties) {
    // store allowed DOFs
    motionProperties.allowedDegreesOfFreedom = allowedDOFs;

    // decompose DOFs
    const allowed_translation_axis = allowedDOFs & 0b111;
    const allowed_rotation_axis = (allowedDOFs >> 3) & 0b111;

    // set inverse mass
    if (allowed_translation_axis === 0) {
        // no translation possible
        motionProperties.invMass = 0.0;
    } else {
        assert(
            massProperties.mass > 0,
            'Invalid mass. Some shapes like TriangleMeshShape cannot calculate mass automatically, in this case you need to provide massProperties to the shape options.',
        );
        motionProperties.invMass = 1.0 / massProperties.mass;
    }

    if (allowed_rotation_axis === 0) {
        // no rotation possible
        vec3.zero(motionProperties.invInertiaDiagonal);
        quat.identity(motionProperties.inertiaRotation);
    } else {
        // set inverse inertia
        const rotation = _setMassProperties_rotation;
        const diagonal = _setMassProperties_diagonal;

        if (decomposePrincipalMomentsOfInertia(massProperties.inertia, rotation, diagonal)) {
            // check if diagonal is near zero
            const diagonalLengthSq = diagonal[0] * diagonal[0] + diagonal[1] * diagonal[1] + diagonal[2] * diagonal[2];

            if (diagonalLengthSq > EPSILON * EPSILON) {
                // calculate reciprocal of diagonal elements
                vec3.set(
                    motionProperties.invInertiaDiagonal,
                    diagonal[0] !== 0 ? 1.0 / diagonal[0] : 0,
                    diagonal[1] !== 0 ? 1.0 / diagonal[1] : 0,
                    diagonal[2] !== 0 ? 1.0 / diagonal[2] : 0,
                );

                // convert rotation matrix to quaternion
                quat.fromMat3(motionProperties.inertiaRotation, rotation);
            } else {
                // diagonal is near zero, fall back to sphere
                const sphere_inertia = 2.5 * motionProperties.invMass;
                vec3.set(motionProperties.invInertiaDiagonal, sphere_inertia, sphere_inertia, sphere_inertia);
                quat.identity(motionProperties.inertiaRotation);
            }
        } else {
            // failed! fall back to inertia tensor of sphere with radius 1.
            const sphere_inertia = 2.5 * motionProperties.invMass;
            vec3.set(motionProperties.invInertiaDiagonal, sphere_inertia, sphere_inertia, sphere_inertia);
            quat.identity(motionProperties.inertiaRotation);
        }
    }

    assert(
        motionProperties.invMass !== 0.0 || vec3.squaredLength(motionProperties.invInertiaDiagonal) !== 0,
        "Can't lock all axes, use a static body for this. This will crash with a division by zero later!",
    );
}

/**
 * Computes the world-space inverse inertia matrix for a given body rotation.
 *
 * Formula: I_inv_world = R * diag(invInertiaDiagonal) * R * mInertiaRotation * R^T
 * where R is the body's rotation matrix.
 *
 * @param out output Mat4 to store the result
 * @param motionProperties motion properties containing inertia data
 * @param bodyRotation body's rotation matrix (Mat4)
 * @returns out parameter
 */
export function getInverseInertiaForRotation(out: Mat4, motionProperties: MotionProperties, bodyRotation: Mat4): Mat4 {
    // step 1: inline mat4.fromQuat(inertiaRotation) -> inertiaRotMat4
    const q = motionProperties.inertiaRotation;
    const qx = q[0];
    const qy = q[1];
    const qz = q[2];
    const qw = q[3];
    const qx2 = qx + qx;
    const qy2 = qy + qy;
    const qz2 = qz + qz;
    const qxx = qx * qx2;
    const qyx = qy * qx2;
    const qyy = qy * qy2;
    const qzx = qz * qx2;
    const qzy = qz * qy2;
    const qzz = qz * qz2;
    const qwx = qw * qx2;
    const qwy = qw * qy2;
    const qwz = qw * qz2;

    // inertiaRotMat4 (3x3 part):
    // column 0: [1 - qyy - qzz, qyx + qwz, qzx - qwy]
    // column 1: [qyx - qwz, 1 - qxx - qzz, qzy + qwx]
    // column 2: [qzx + qwy, qzy - qwx, 1 - qxx - qyy]
    const i00 = 1 - qyy - qzz;
    const i01 = qyx + qwz;
    const i02 = qzx - qwy;
    const i10 = qyx - qwz;
    const i11 = 1 - qxx - qzz;
    const i12 = qzy + qwx;
    const i20 = qzx + qwy;
    const i21 = qzy - qwx;
    const i22 = 1 - qxx - qyy;

    // step 2: inline mat4.multiply3x3(rotation, bodyRotation, inertiaRotMat4)
    // extract bodyRotation 3x3
    const b00 = bodyRotation[0];
    const b01 = bodyRotation[1];
    const b02 = bodyRotation[2];
    const b10 = bodyRotation[4];
    const b11 = bodyRotation[5];
    const b12 = bodyRotation[6];
    const b20 = bodyRotation[8];
    const b21 = bodyRotation[9];
    const b22 = bodyRotation[10];

    // rotation = bodyRotation * inertiaRotMat4
    // column j of result = bodyRotation * (column j of inertiaRotMat4)
    // column 0 of result = bodyRotation * [i00, i01, i02]
    const r00 = b00 * i00 + b10 * i01 + b20 * i02;
    const r01 = b01 * i00 + b11 * i01 + b21 * i02;
    const r02 = b02 * i00 + b12 * i01 + b22 * i02;
    // column 1 of result = bodyRotation * [i10, i11, i12]
    const r10 = b00 * i10 + b10 * i11 + b20 * i12;
    const r11 = b01 * i10 + b11 * i11 + b21 * i12;
    const r12 = b02 * i10 + b12 * i11 + b22 * i12;
    // column 2 of result = bodyRotation * [i20, i21, i22]
    const r20 = b00 * i20 + b10 * i21 + b20 * i22;
    const r21 = b01 * i20 + b11 * i21 + b21 * i22;
    const r22 = b02 * i20 + b12 * i21 + b22 * i22;

    // step 3: scale rotation columns by invInertiaDiagonal
    const d0 = motionProperties.invInertiaDiagonal[0];
    const d1 = motionProperties.invInertiaDiagonal[1];
    const d2 = motionProperties.invInertiaDiagonal[2];

    const s00 = d0 * r00;
    const s01 = d0 * r01;
    const s02 = d0 * r02;
    const s10 = d1 * r10;
    const s11 = d1 * r11;
    const s12 = d1 * r12;
    const s20 = d2 * r20;
    const s21 = d2 * r21;
    const s22 = d2 * r22;

    // step 4: inline mat4.multiply3x3RightTransposed(out, rotation, rotationScaled)
    // result = rotation * rotationScaled^T
    // column 0 of result (multiply rotation by transposed column 0 of scaled, which is row 0)
    out[0] = s00 * r00 + s10 * r10 + s20 * r20;
    out[1] = s00 * r01 + s10 * r11 + s20 * r21;
    out[2] = s00 * r02 + s10 * r12 + s20 * r22;
    out[3] = 0;

    // column 1 of result (multiply rotation by transposed column 1 of scaled, which is row 1)
    out[4] = s01 * r00 + s11 * r10 + s21 * r20;
    out[5] = s01 * r01 + s11 * r11 + s21 * r21;
    out[6] = s01 * r02 + s11 * r12 + s21 * r22;
    out[7] = 0;

    // column 2 of result (multiply rotation by transposed column 2 of scaled, which is row 2)
    out[8] = s02 * r00 + s12 * r10 + s22 * r20;
    out[9] = s02 * r01 + s12 * r11 + s22 * r21;
    out[10] = s02 * r02 + s12 * r12 + s22 * r22;
    out[11] = 0;

    // column 3
    out[12] = 0;
    out[13] = 0;
    out[14] = 0;
    out[15] = 1;

    // step 5: mask out DOFs that are not allowed
    const allowedRotationAxis = (motionProperties.allowedDegreesOfFreedom >> 3) & 0b111;
    if (allowedRotationAxis !== 0b111) {
        // create mask for each axis (1.0 if allowed, 0.0 if not)
        const maskX = allowedRotationAxis & 0b001 ? 1.0 : 0.0;
        const maskY = allowedRotationAxis & 0b010 ? 1.0 : 0.0;
        const maskZ = allowedRotationAxis & 0b100 ? 1.0 : 0.0;

        // mask column 0
        out[0] *= maskX * maskX;
        out[1] *= maskY * maskX;
        out[2] *= maskZ * maskX;

        // mask column 1
        out[4] *= maskX * maskY;
        out[5] *= maskY * maskY;
        out[6] *= maskZ * maskY;

        // mask column 2
        out[8] *= maskX * maskZ;
        out[9] *= maskY * maskZ;
        out[10] *= maskZ * maskZ;
    }

    return out;
}

/** Clamps linear velocity to the maximum allowed value */
export function clampLinearVelocity(motionProperties: MotionProperties): void {
    const len_sq = vec3.squaredLength(motionProperties.linearVelocity);
    if (len_sq > motionProperties.maxLinearVelocity * motionProperties.maxLinearVelocity) {
        vec3.normalize(motionProperties.linearVelocity, motionProperties.linearVelocity);
        vec3.scale(motionProperties.linearVelocity, motionProperties.linearVelocity, motionProperties.maxLinearVelocity);
    }
}

/** Clamps angular velocity to the maximum allowed value */
export function clampAngularVelocity(motionProperties: MotionProperties): void {
    const len_sq = vec3.squaredLength(motionProperties.angularVelocity);
    if (len_sq > motionProperties.maxAngularVelocity * motionProperties.maxAngularVelocity) {
        vec3.normalize(motionProperties.angularVelocity, motionProperties.angularVelocity);
        vec3.scale(motionProperties.angularVelocity, motionProperties.angularVelocity, motionProperties.maxAngularVelocity);
    }
}

/**
 * Sets linear velocity and clamps it to the maximum allowed value.
 * This setter combines assignment with clamping to prevent invalid states.
 * @param motionProperties motion properties to update
 * @param velocity new linear velocity
 */
export function setLinearVelocity(motionProperties: MotionProperties, velocity: Vec3): void {
    // vec3.copy(motionProperties.linearVelocity, velocity);
    motionProperties.linearVelocity[0] = velocity[0];
    motionProperties.linearVelocity[1] = velocity[1];
    motionProperties.linearVelocity[2] = velocity[2];
    clampLinearVelocity(motionProperties);
}

/**
 * Sets angular velocity and clamps it to the maximum allowed value.
 * This setter combines assignment with clamping to prevent invalid states.
 * @param motionProperties motion properties to update
 * @param velocity new angular velocity
 */
export function setAngularVelocity(motionProperties: MotionProperties, velocity: Vec3): void {
    // vec3.copy(motionProperties.angularVelocity, velocity);
    motionProperties.angularVelocity[0] = velocity[0];
    motionProperties.angularVelocity[1] = velocity[1];
    motionProperties.angularVelocity[2] = velocity[2];
    clampAngularVelocity(motionProperties);
}

/**
 * Adds to the linear velocity and clamps to the maximum allowed value.
 *
 * @param motionProperties motion properties to update
 * @param velocityDelta velocity change to add
 */
export function addLinearVelocity(motionProperties: MotionProperties, velocityDelta: Vec3): void {
    // vec3.add(motionProperties.linearVelocity, motionProperties.linearVelocity, velocityDelta);
    motionProperties.linearVelocity[0] += velocityDelta[0];
    motionProperties.linearVelocity[1] += velocityDelta[1];
    motionProperties.linearVelocity[2] += velocityDelta[2];
    clampLinearVelocity(motionProperties);
}

/**
 * Adds to the angular velocity and clamps to the maximum allowed value.
 * @param motionProperties motion properties to update
 * @param velocityDelta angular velocity change to add
 */
export function addAngularVelocity(motionProperties: MotionProperties, velocityDelta: Vec3): void {
    // vec3.add(motionProperties.angularVelocity, motionProperties.angularVelocity, velocityDelta);
    motionProperties.angularVelocity[0] += velocityDelta[0];
    motionProperties.angularVelocity[1] += velocityDelta[1];
    motionProperties.angularVelocity[2] += velocityDelta[2];
    clampAngularVelocity(motionProperties);
}

/**
 * Add a linear velocity step (used during constraint solving).
 * Applies translation locking based on allowed degrees of freedom.
 * @param motionProperties motion properties to update
 * @param linearVelocityChange velocity change to add
 */
export function addLinearVelocityStep(motionProperties: MotionProperties, linearVelocityChange: Vec3): void {
    // mLinearVelocity = LockTranslation(mLinearVelocity + inLinearVelocityChange)
    vec3.add(motionProperties.linearVelocity, motionProperties.linearVelocity, linearVelocityChange);
    applyTranslationDOFConstraint(motionProperties.linearVelocity, motionProperties.allowedDegreesOfFreedom);
}

/**
 * Subtract a linear velocity step (used during constraint solving).
 * Applies translation locking based on allowed degrees of freedom.
 * @param motionProperties motion properties to update
 * @param linearVelocityChange velocity change to subtract
 */
export function subLinearVelocityStep(motionProperties: MotionProperties, linearVelocityChange: Vec3): void {
    // vec3.sub(motionProperties.linearVelocity, motionProperties.linearVelocity, linearVelocityChange);
    motionProperties.linearVelocity[0] -= linearVelocityChange[0];
    motionProperties.linearVelocity[1] -= linearVelocityChange[1];
    motionProperties.linearVelocity[2] -= linearVelocityChange[2];
    applyTranslationDOFConstraint(motionProperties.linearVelocity, motionProperties.allowedDegreesOfFreedom);
}

/**
 * Add an angular velocity step (used during constraint solving).
 * @param motionProperties motion properties to update
 * @param angularVelocityChange velocity change to add
 */
export function addAngularVelocityStep(motionProperties: MotionProperties, angularVelocityChange: Vec3): void {
    // vec3.add(motionProperties.angularVelocity, motionProperties.angularVelocity, angularVelocityChange);
    motionProperties.angularVelocity[0] += angularVelocityChange[0];
    motionProperties.angularVelocity[1] += angularVelocityChange[1];
    motionProperties.angularVelocity[2] += angularVelocityChange[2];
}

/**
 * Subtract an angular velocity step (used during constraint solving).
 * @param motionProperties motion properties to update
 * @param angularVelocityChange velocity change to subtract
 */
export function subAngularVelocityStep(motionProperties: MotionProperties, angularVelocityChange: Vec3): void {
    // vec3.sub(motionProperties.angularVelocity, motionProperties.angularVelocity, angularVelocityChange);
    motionProperties.angularVelocity[0] -= angularVelocityChange[0];
    motionProperties.angularVelocity[1] -= angularVelocityChange[1];
    motionProperties.angularVelocity[2] -= angularVelocityChange[2];
}

/**
 * Scales the inverse inertia diagonal when mass changes at runtime.
 * This maintains the inertia tensor's proportionality to mass.
 *
 * @param motionProperties motion properties to scale
 * @param newMass new mass value (must be > 0)
 */
export function scaleToMass(motionProperties: MotionProperties, newMass: number): void {
    assert(motionProperties.invMass > 0, 'Body must have finite mass to scale');
    assert(newMass > 0, 'New mass cannot be zero or negative');

    const newInvMass = 1.0 / newMass;
    // scale inverse inertia diagonal: I^-1 *= (newInvMass / oldInvMass)
    vec3.scale(motionProperties.invInertiaDiagonal, motionProperties.invInertiaDiagonal, newInvMass / motionProperties.invMass);
    motionProperties.invMass = newInvMass;
}

const _moveKinematic_axis = /* @__PURE__ */ vec3.create();

/**
 * Set velocity of body such that it will be rotate/translate by inDeltaPosition/Rotation in inDeltaTime seconds.
 *
 * @param motionProperties motion properties to update
 * @param deltaPosition desired position change
 * @param deltaRotation desired rotation change as quaternion
 * @param deltaTime time step (must be > 0)
 */
export function moveKinematic(
    motionProperties: MotionProperties,
    deltaPosition: Vec3,
    deltaRotation: Quat,
    deltaTime: number,
): void {
    // delta time must be positive, otherwise no-op
    if (deltaTime <= 0) {
        // TODO: assert?
        return;
    }

    // calculate required linear velocity
    vec3.scale(motionProperties.linearVelocity, deltaPosition, 1 / deltaTime);
    applyTranslationDOFConstraint(motionProperties.linearVelocity, motionProperties.allowedDegreesOfFreedom);

    // calculate required angular velocity
    getAxisAngleFromQuat(_moveKinematic_axis, deltaRotation);
    const angle = 2 * Math.acos(Math.max(-1, Math.min(1, deltaRotation[3]))); // w is at index 3
    vec3.scale(motionProperties.angularVelocity, _moveKinematic_axis, angle / deltaTime);
    applyRotationDOFConstraint(motionProperties.angularVelocity, motionProperties.allowedDegreesOfFreedom);
}

/**
 * Applies translation DOF constraint to a velocity vector.
 * Zeros out velocity components for locked translation axes.
 */
export function applyTranslationDOFConstraint(velocity: Vec3, allowedDOFs: number): void {
    const allowedTranslation = allowedDOFs & 0b111;
    if (!(allowedTranslation & 0b001)) velocity[0] = 0; // x locked
    if (!(allowedTranslation & 0b010)) velocity[1] = 0; // y locked
    if (!(allowedTranslation & 0b100)) velocity[2] = 0; // z locked
}

/**
 * Applies rotation DOF constraint to an angular velocity vector.
 * Zeros out angular velocity components for locked rotation axes.
 */
export function applyRotationDOFConstraint(angularVelocity: Vec3, allowedDOFs: number): void {
    const allowedRotation = (allowedDOFs >> 3) & 0b111;
    if (!(allowedRotation & 0b001)) angularVelocity[0] = 0; // x rotation locked
    if (!(allowedRotation & 0b010)) angularVelocity[1] = 0; // y rotation locked
    if (!(allowedRotation & 0b100)) angularVelocity[2] = 0; // z rotation locked
}

/** helper to extract axis from a quaternion (for axis-angle representation) */
function getAxisAngleFromQuat(outAxis: Vec3, q: Quat): void {
    // For a unit quaternion q = [x, y, z, w], the axis is [x, y, z] / sin(θ/2)
    // where θ is the rotation angle
    const sinHalfAngleSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2];
    if (sinHalfAngleSq > 1e-10) {
        const sinHalfAngle = Math.sqrt(sinHalfAngleSq);
        outAxis[0] = q[0] / sinHalfAngle;
        outAxis[1] = q[1] / sinHalfAngle;
        outAxis[2] = q[2] / sinHalfAngle;
    } else {
        // quaternion is near identity, axis is arbitrary
        outAxis[0] = 1;
        outAxis[1] = 0;
        outAxis[2] = 0;
    }
}

const _multiplyWorldSpaceInverseInertiaByVector_combinedQuat = /* @__PURE__ */ quat.create();
const _multiplyWorldSpaceInverseInertiaByVector_rotation = /* @__PURE__ */ mat3.create();
const _multiplyWorldSpaceInverseInertiaByVector_maskedV = /* @__PURE__ */ vec3.create();
const _multiplyWorldSpaceInverseInertiaByVector_temp = /* @__PURE__ */ vec3.create();

/**
 * Multiplies a vector by the world-space inverse inertia matrix more efficiently than computing the full matrix.
 * This applies both the inertia transform and DOF constraints.
 *
 * @param out output vector to store result
 * @param motionProperties motion properties containing inertia data
 * @param bodyQuaternion body quaternion
 * @param vector vector to multiply
 * @returns out parameter
 */
export function multiplyWorldSpaceInverseInertiaByVector(
    out: Vec3,
    motionProperties: MotionProperties,
    bodyQuaternion: Quat,
    vector: Vec3,
): Vec3 {
    // mask out columns of DOFs that are not allowed
    const allowedRotationAxis = (motionProperties.allowedDegreesOfFreedom >> 3) & 0b111;
    const v = _multiplyWorldSpaceInverseInertiaByVector_maskedV;
    vec3.copy(v, vector);
    if (!(allowedRotationAxis & 0b001)) v[0] = 0;
    if (!(allowedRotationAxis & 0b010)) v[1] = 0;
    if (!(allowedRotationAxis & 0b100)) v[2] = 0;

    // compute rotation matrix from combined quaternion
    const combinedQuat = _multiplyWorldSpaceInverseInertiaByVector_combinedQuat;
    quat.multiply(combinedQuat, bodyQuaternion, motionProperties.inertiaRotation);
    const rotation = _multiplyWorldSpaceInverseInertiaByVector_rotation;
    mat3.fromQuat(rotation, combinedQuat);

    // apply transformation
    const temp = _multiplyWorldSpaceInverseInertiaByVector_temp;
    mat3.transpose(rotation, rotation);
    vec3.transformMat3(temp, v, rotation);

    temp[0] *= motionProperties.invInertiaDiagonal[0];
    temp[1] *= motionProperties.invInertiaDiagonal[1];
    temp[2] *= motionProperties.invInertiaDiagonal[2];

    mat3.transpose(rotation, rotation); // restore non-transposed
    vec3.transformMat3(out, temp, rotation);

    // mask out rows of DOFs that are not allowed
    if (!(allowedRotationAxis & 0b001)) out[0] = 0;
    if (!(allowedRotationAxis & 0b010)) out[1] = 0;
    if (!(allowedRotationAxis & 0b100)) out[2] = 0;

    return out;
}

/**
 * Get velocity of a point on the body (point relative to center of mass).
 * Velocity = v_linear + ω × r
 *
 * @param out output vector to store result
 * @param motionProperties motion properties containing velocity data
 * @param pointRelativeToCOM point position relative to center of mass
 * @returns out parameter
 */
export function getPointVelocityCOM(out: Vec3, motionProperties: MotionProperties, pointRelativeToCOM: Vec3): Vec3 {
    const [avX, avY, avZ] = motionProperties.angularVelocity;
    const [rX, rY, rZ] = pointRelativeToCOM;

    // v_point = v_linear + ω × r
    
    // cross: angular velocity x pointRelativeToCOM
    const angularContribX = avY * rZ - avZ * rY;
    const angularContribY = avZ * rX - avX * rZ;
    const angularContribZ = avX * rY - avY * rX;

    // add: linearVelocity + angularContrib
    out[0] = motionProperties.linearVelocity[0] + angularContribX;
    out[1] = motionProperties.linearVelocity[1] + angularContribY;
    out[2] = motionProperties.linearVelocity[2] + angularContribZ;

    return out;
}

const _applyGyroscopicForce_localInertia = /* @__PURE__ */ vec3.create();
const _applyGyroscopicForce_localAngularVelocity = /* @__PURE__ */ vec3.create();
const _applyGyroscopicForce_localMomentum = /* @__PURE__ */ vec3.create();
const _applyGyroscopicForce_newLocalMomentum = /* @__PURE__ */ vec3.create();
const _applyGyroscopicForce_crossProduct = /* @__PURE__ */ vec3.create();
const _applyGyroscopicForce_inertiaSpaceToWorldSpace = /* @__PURE__ */ quat.create();
const _applyGyroscopicForce_conjugated = /* @__PURE__ */ quat.create();
const _applyGyroscopicForce_newLocalAngularVelocity = /* @__PURE__ */ vec3.create();

/**
 * Apply the gyroscopic force (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
 * This simulates the realistic behavior of spinning bodies by applying torque T = -ω × (I·ω)
 *
 * @param motionProperties motion properties to update
 * @param bodyQuaternion body quaternion
 * @param deltaTime time step for integration
 */
export function applyGyroscopicForce(motionProperties: MotionProperties, bodyQuaternion: Quat, deltaTime: number): void {
    // calculate local space inertia tensor (a diagonal in local space)
    const localInertia = _applyGyroscopicForce_localInertia;
    for (let i = 0; i < 3; i++) {
        if (motionProperties.invInertiaDiagonal[i] === 0) {
            localInertia[i] = 0; // avoid dividing by zero, inertia in this axis will be zero
        } else {
            localInertia[i] = 1.0 / motionProperties.invInertiaDiagonal[i]; // I = 1/I_inv
        }
    }

    // calculate local space angular momentum
    const inertiaSpaceToWorldSpace = _applyGyroscopicForce_inertiaSpaceToWorldSpace;
    quat.multiply(inertiaSpaceToWorldSpace, bodyQuaternion, motionProperties.inertiaRotation);

    const localAngularVelocity = _applyGyroscopicForce_localAngularVelocity;
    const conjugated = _applyGyroscopicForce_conjugated;
    quat.conjugate(conjugated, inertiaSpaceToWorldSpace);
    vec3.transformQuat(localAngularVelocity, motionProperties.angularVelocity, conjugated);

    const localMomentum = _applyGyroscopicForce_localMomentum;
    vec3.multiply(localMomentum, localInertia, localAngularVelocity); // L = I·ω

    // the gyroscopic force applies a torque: T = -ω × I·ω = -ω × L
    // calculate the new angular momentum by applying the gyroscopic force
    const crossProduct = _applyGyroscopicForce_crossProduct;
    vec3.cross(crossProduct, localAngularVelocity, localMomentum);

    const newLocalMomentum = _applyGyroscopicForce_newLocalMomentum;
    vec3.copy(newLocalMomentum, localMomentum);
    vec3.scaleAndAdd(newLocalMomentum, newLocalMomentum, crossProduct, -deltaTime);

    // make sure the new magnitude is the same as the old one to avoid introducing energy into the system due to the Euler step
    const newLocalMomentumLenSq = vec3.squaredLength(newLocalMomentum);
    if (newLocalMomentumLenSq > 0) {
        const oldLocalMomentumLenSq = vec3.squaredLength(localMomentum);
        const scale = Math.sqrt(oldLocalMomentumLenSq / newLocalMomentumLenSq);
        vec3.scale(newLocalMomentum, newLocalMomentum, scale);
    } else {
        vec3.zero(newLocalMomentum);
    }

    // convert back to world space angular velocity: ω = I^(-1) · L
    const newLocalAngularVelocity = _applyGyroscopicForce_newLocalAngularVelocity;
    vec3.multiply(newLocalAngularVelocity, motionProperties.invInertiaDiagonal, newLocalMomentum);

    vec3.transformQuat(motionProperties.angularVelocity, newLocalAngularVelocity, inertiaSpaceToWorldSpace);
}
