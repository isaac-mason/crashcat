import { describe, it, expect } from 'vitest';
import { mat4, quat, vec3 } from 'mathcat';
import * as motionProperties from '../../src/body/motion-properties';
import { DOF_ALL } from '../../src/body/dof';

describe('multiplyWorldSpaceInverseInertiaByVector', () => {
    it('should produce same result as getInverseInertiaForRotation followed by matrix-vector multiply', () => {
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        // non-identity inertia rotation
        quat.setAxisAngle(mp.inertiaRotation, vec3.normalize(vec3.create(), vec3.fromValues(1, 2, 3)), 0.7);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        // non-trivial body rotation
        const axis = vec3.normalize(vec3.create(), vec3.fromValues(0.5, 0.8, 0.2));
        const bodyQuat = quat.setAxisAngle(quat.create(), axis, 1.1);
        const bodyRot = mat4.fromQuat(mat4.create(), bodyQuat);

        // input vector
        const inputVec = vec3.fromValues(10, -5, 7);

        // method 1: full matrix computation
        const invInertia = mat4.create();
        motionProperties.getInverseInertiaForRotation(invInertia, mp, bodyRot);
        const result1 = mat4.multiply3x3Vec(vec3.create(), invInertia, inputVec);

        // method 2: direct vector multiply
        const result2 = motionProperties.multiplyWorldSpaceInverseInertiaByVector(
            vec3.create(),
            mp,
            bodyQuat,
            inputVec,
        );

        expect(result1[0]).toBeCloseTo(result2[0], 10);
        expect(result1[1]).toBeCloseTo(result2[1], 10);
        expect(result1[2]).toBeCloseTo(result2[2], 10);
    });

    it('should correctly mask locked DOFs', () => {
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        quat.identity(mp.inertiaRotation);
        // lock Y rotation
        mp.allowedDegreesOfFreedom = 0b101111; // all except rotation Y (bit 4)

        const bodyQuat = quat.create(); // identity
        const inputVec = vec3.fromValues(1, 1, 1);

        const result = motionProperties.multiplyWorldSpaceInverseInertiaByVector(
            vec3.create(),
            mp,
            bodyQuat,
            inputVec,
        );

        // Y component should be 0 (locked), X and Z should be non-zero
        expect(result[0]).toBeCloseTo(1, 10);  // invInertiaDiag[0] * 1
        expect(result[1]).toBeCloseTo(0, 10);  // locked
        expect(result[2]).toBeCloseTo(3, 10);  // invInertiaDiag[2] * 1
    });
});

describe('getInverseInertiaForRotation', () => {
    it('should return identity-like inverse inertia for uniform sphere at identity rotation', () => {
        // for a uniform sphere, all principal moments are equal
        // with identity inertia rotation and identity body rotation,
        // the world-space inverse inertia should equal the local-space inverse inertia
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(2.5, 2.5, 2.5); // uniform sphere-like
        quat.identity(mp.inertiaRotation);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        const bodyRotation = mat4.create(); // identity
        const out = mat4.create();
        
        motionProperties.getInverseInertiaForRotation(out, mp, bodyRotation);

        // diagonal should be 2.5, off-diagonals should be 0
        expect(out[0]).toBeCloseTo(2.5, 10);
        expect(out[5]).toBeCloseTo(2.5, 10);
        expect(out[10]).toBeCloseTo(2.5, 10);
        
        // off-diagonals should be ~0
        expect(out[1]).toBeCloseTo(0, 10);
        expect(out[2]).toBeCloseTo(0, 10);
        expect(out[4]).toBeCloseTo(0, 10);
        expect(out[6]).toBeCloseTo(0, 10);
        expect(out[8]).toBeCloseTo(0, 10);
        expect(out[9]).toBeCloseTo(0, 10);
    });

    it('should transform inverse inertia correctly for rotated body', () => {
        // for a box with different moments along each axis,
        // rotating the body should rotate the inverse inertia tensor
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        // different principal moments: Ixx^-1=1, Iyy^-1=2, Izz^-1=3
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        quat.identity(mp.inertiaRotation);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        // identity rotation: diagonal should match invInertiaDiagonal
        const identityRot = mat4.create();
        const out1 = mat4.create();
        motionProperties.getInverseInertiaForRotation(out1, mp, identityRot);
        
        expect(out1[0]).toBeCloseTo(1, 10);
        expect(out1[5]).toBeCloseTo(2, 10);
        expect(out1[10]).toBeCloseTo(3, 10);

        // rotate 90 degrees around Z axis: X->Y, Y->-X
        // so I_world_xx = I_yy, I_world_yy = I_xx
        const rotZ90 = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 0, 1), Math.PI / 2);
        const rotZ90Mat = mat4.fromQuat(mat4.create(), rotZ90);
        const out2 = mat4.create();
        motionProperties.getInverseInertiaForRotation(out2, mp, rotZ90Mat);

        // after 90deg Z rotation:
        // new X axis is old Y axis -> invInertia_xx = old invInertia_yy = 2
        // new Y axis is old -X axis -> invInertia_yy = old invInertia_xx = 1
        // Z stays same -> invInertia_zz = 3
        expect(out2[0]).toBeCloseTo(2, 6);  // Ixx^-1
        expect(out2[5]).toBeCloseTo(1, 6);  // Iyy^-1
        expect(out2[10]).toBeCloseTo(3, 6); // Izz^-1
    });

    it('should handle non-identity inertia rotation', () => {
        // when inertiaRotation is non-identity, it represents the rotation from
        // body local space to principal axes
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        // inertia is rotated 90deg around Z in body space
        quat.setAxisAngle(mp.inertiaRotation, vec3.fromValues(0, 0, 1), Math.PI / 2);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        // at identity body rotation, the effective inertia is rotated by inertiaRotation
        const identityRot = mat4.create();
        const out = mat4.create();
        motionProperties.getInverseInertiaForRotation(out, mp, identityRot);

        // with inertiaRotation = 90deg around Z:
        // principal X -> body Y, principal Y -> body -X
        // so body Ixx^-1 = principal Iyy^-1 = 2
        // body Iyy^-1 = principal Ixx^-1 = 1
        expect(out[0]).toBeCloseTo(2, 6);
        expect(out[5]).toBeCloseTo(1, 6);
        expect(out[10]).toBeCloseTo(3, 6);
    });

    it('should mask out locked rotation DOFs', () => {
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        quat.identity(mp.inertiaRotation);
        // lock X rotation (bit 3 = 0b1000)
        // DOF_ALL = 0b111111, remove rotation X (bit 3)
        mp.allowedDegreesOfFreedom = 0b110111; // all except rotation X

        const identityRot = mat4.create();
        const out = mat4.create();
        motionProperties.getInverseInertiaForRotation(out, mp, identityRot);

        // row 0 and column 0 should be masked to 0
        expect(out[0]).toBeCloseTo(0, 10);  // col0, row0
        expect(out[1]).toBeCloseTo(0, 10);  // col0, row1
        expect(out[2]).toBeCloseTo(0, 10);  // col0, row2
        expect(out[4]).toBeCloseTo(0, 10);  // col1, row0
        expect(out[8]).toBeCloseTo(0, 10);  // col2, row0
        
        // remaining should be unchanged
        expect(out[5]).toBeCloseTo(2, 10);  // col1, row1 (Iyy^-1)
        expect(out[10]).toBeCloseTo(3, 10); // col2, row2 (Izz^-1)
    });

    it('should be symmetric (inverse inertia tensor is always symmetric)', () => {
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        // arbitrary rotation
        quat.setAxisAngle(mp.inertiaRotation, vec3.normalize(vec3.create(), vec3.fromValues(1, 1, 1)), 0.5);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        // arbitrary body rotation
        const bodyQuat = quat.setAxisAngle(quat.create(), vec3.fromValues(0.5, 0.7, 0.3), 1.2);
        const bodyRot = mat4.fromQuat(mat4.create(), bodyQuat);
        const out = mat4.create();
        motionProperties.getInverseInertiaForRotation(out, mp, bodyRot);

        // check symmetry: out[row, col] === out[col, row]
        expect(out[1]).toBeCloseTo(out[4], 10);  // (0,1) vs (1,0)
        expect(out[2]).toBeCloseTo(out[8], 10);  // (0,2) vs (2,0)
        expect(out[6]).toBeCloseTo(out[9], 10);  // (1,2) vs (2,1)
    });

    it('should produce result matching manual R * D * R^T computation', () => {
        const mp = motionProperties.create();
        mp.invMass = 1.0;
        mp.invInertiaDiagonal = vec3.fromValues(1, 2, 3);
        quat.identity(mp.inertiaRotation);
        mp.allowedDegreesOfFreedom = DOF_ALL;

        // 45 degree rotation around Y
        const bodyQuat = quat.setAxisAngle(quat.create(), vec3.fromValues(0, 1, 0), Math.PI / 4);
        const R = mat4.fromQuat(mat4.create(), bodyQuat);

        const out = mat4.create();
        motionProperties.getInverseInertiaForRotation(out, mp, R);

        // manually compute R * D * R^T where D = diag(1, 2, 3)
        // for 45deg rotation around Y:
        // R = [[c, 0, s], [0, 1, 0], [-s, 0, c]] where c=cos(45)=s=sin(45)=sqrt(2)/2
        const c = Math.cos(Math.PI / 4);
        const s = Math.sin(Math.PI / 4);
        
        // R * D has columns: [c*1, 0, -s*1], [0, 2*1, 0], [s*3, 0, c*3]
        // then (R*D) * R^T...
        // I_world[0][0] = (c*1)*c + 0*0 + (s*3)*s = c²*1 + s²*3
        // I_world[0][2] = (c*1)*(-s) + 0*0 + (s*3)*c = -cs + 3cs = 2cs
        // I_world[1][1] = 0 + 2*1 + 0 = 2
        // I_world[2][2] = (-s*1)*(-s) + 0 + (c*3)*c = s²*1 + c²*3
        
        const expected00 = c * c * 1 + s * s * 3;
        const expected11 = 2;
        const expected22 = s * s * 1 + c * c * 3;
        const expected02 = -c * s * 1 + s * c * 3; // = 2*c*s
        
        expect(out[0]).toBeCloseTo(expected00, 10);
        expect(out[5]).toBeCloseTo(expected11, 10);
        expect(out[10]).toBeCloseTo(expected22, 10);
        expect(out[2]).toBeCloseTo(expected02, 10);
        expect(out[8]).toBeCloseTo(expected02, 10); // symmetric
    });
});
