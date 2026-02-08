import { type Mat4, mat3, mat4, type Vec3, vec3 } from 'mathcat';

export type MassProperties = {
    /** the mass of the shape (kg) */
    mass: number;
    /** the inertia tensor of the shape (kg m^2) */
    inertia: Mat4;
};

/** creates mass properties with default values */
export function create(): MassProperties {
    return {
        mass: 0,
        inertia: mat4.create(),
    };
}

/** resets mass properties to default values */
export function reset(out: MassProperties): MassProperties {
    out.mass = 0;
    mat4.identity(out.inertia);
    return out;
}

/** copies mass properties from source to out */
export function copy(out: MassProperties, source: MassProperties): MassProperties {
    out.mass = source.mass;
    mat4.copy(out.inertia, source.inertia);
    return out;
}

const _setMassAndInertiaOfSolidBox_sizeSq = /* @__PURE__ */ vec3.create();
const _setMassAndInertiaOfSolidBox_scale = /* @__PURE__ */ vec3.create();

/** sets mass and inertia of a solid box */
export function setMassAndInertiaOfSolidBox(out: MassProperties, boxSize: Vec3, density: number): MassProperties {
    // calculate mass
    out.mass = boxSize[0] * boxSize[1] * boxSize[2] * density;

    // calculate inertia
    const sizeSq = _setMassAndInertiaOfSolidBox_sizeSq;
    vec3.multiply(sizeSq, boxSize, boxSize);

    const scale = _setMassAndInertiaOfSolidBox_scale;
    scale[0] = (sizeSq[1] + sizeSq[2]) * (out.mass / 12.0);
    scale[1] = (sizeSq[0] + sizeSq[2]) * (out.mass / 12.0);
    scale[2] = (sizeSq[0] + sizeSq[1]) * (out.mass / 12.0);

    mat4.fromScaling(out.inertia, scale);

    return out;
}

const _scale_diagonal = /* @__PURE__ */ vec3.create();
const _scale_xyz_sq = /* @__PURE__ */ vec3.create();
const _scale_xyz_scaled_sq = /* @__PURE__ */ vec3.create();

/** scales mass properties by given scale factors */
export function scale(out: MassProperties, source: MassProperties, inScale: Vec3): MassProperties {
    // extract diagonal of 3x3 inertia tensor (elements at indices 0, 5, 10)
    const diagonal = _scale_diagonal;
    diagonal[0] = source.inertia[0];
    diagonal[1] = source.inertia[5];
    diagonal[2] = source.inertia[10];

    // calculate: xyz_sq = 0.5 * dot(diagonal) - diagonal
    const halfDot = 0.5 * (diagonal[0] + diagonal[1] + diagonal[2]);
    const xyz_sq = _scale_xyz_sq;
    xyz_sq[0] = halfDot - diagonal[0];
    xyz_sq[1] = halfDot - diagonal[1];
    xyz_sq[2] = halfDot - diagonal[2];

    // scale: xyz_scaled_sq = scale * scale * xyz_sq (component-wise)
    const xyz_scaled_sq = _scale_xyz_scaled_sq;
    xyz_scaled_sq[0] = inScale[0] * inScale[0] * xyz_sq[0];
    xyz_scaled_sq[1] = inScale[1] * inScale[1] * xyz_sq[1];
    xyz_scaled_sq[2] = inScale[2] * inScale[2] * xyz_sq[2];

    // calculate new diagonal elements
    const i_xx = xyz_scaled_sq[1] + xyz_scaled_sq[2];
    const i_yy = xyz_scaled_sq[0] + xyz_scaled_sq[2];
    const i_zz = xyz_scaled_sq[0] + xyz_scaled_sq[1];
    
    // calculate new off-diagonal elements
    const i_xy = inScale[0] * inScale[1] * source.inertia[4]; // a.inertia[4] is the (0,1) element (column-major)
    const i_xz = inScale[0] * inScale[2] * source.inertia[8]; // a.inertia[8] is the (0,2) element
    const i_yz = inScale[1] * inScale[2] * source.inertia[9]; // a.inertia[9] is the (1,2) element

    // copy and update inertia tensor
    mat4.copy(out.inertia, source.inertia);
    out.inertia[0] = i_xx;
    out.inertia[1] = i_xy;
    out.inertia[4] = i_xy;
    out.inertia[5] = i_yy;
    out.inertia[2] = i_xz;
    out.inertia[8] = i_xz;
    out.inertia[6] = i_yz;
    out.inertia[9] = i_yz;
    out.inertia[10] = i_zz;

    // mass scales by volume (absolute value of scale factors product)
    const mass_scale = Math.abs(inScale[0] * inScale[1] * inScale[2]);
    out.mass = source.mass * mass_scale;

    // inertia scales linearly with mass
    for (let i = 0; i < 15; i++) {
        out.inertia[i] *= mass_scale;
    }

    // ensure bottom-right element is 1
    out.inertia[15] = 1.0;

    return out;
}

const _rotate_temp1 = /* @__PURE__ */ mat4.create();
const _rotate_temp2 = /* @__PURE__ */ mat4.create();
const _rotate_temp3 = /* @__PURE__ */ mat3.create();

/** rotates mass properties by given rotation matrix */
export function rotate(out: MassProperties, source: MassProperties, rotation: Mat4): MassProperties {
    // copy mass from input (rotation doesn't affect mass)
    out.mass = source.mass;

    // extract 3x3 rotation from the 4x4 matrix
    const rot3x3 = _rotate_temp3;
    mat3.fromMat4(rot3x3, rotation);

    // create a 4x4 version of the inertia for multiplication
    mat4.copy(out.inertia, source.inertia);

    // extract the 3x3 inertia tensor and apply rotation: I' = R * I * R^T
    // we need to:
    // 1. multiply rotation * inertia
    // 2. multiply result * rotation^T
    const temp1 = _rotate_temp1;
    const temp2 = _rotate_temp2;

    // convert rot3x3 to mat4 for matrix multiplication
    const rot4x4 = temp1;
    mat4.identity(rot4x4);
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            rot4x4[i + j * 4] = rot3x3[i + j * 3];
        }
    }

    // multiply: temp2 = rot4x4 * out.inertia
    mat4.multiply(temp2, rot4x4, out.inertia);

    // transpose the rotation matrix for the right multiplication
    mat4.transpose(rot4x4, rot4x4);

    // multiply: out.inertia = temp2 * rot4x4^T
    mat4.multiply(out.inertia, temp2, rot4x4);

    // ensure bottom-right element is 1
    out.inertia[15] = 1.0;

    return out;
}

const _translate_scaleMatrix = /* @__PURE__ */ mat4.create();
const _translate_outerProduct = /* @__PURE__ */ mat4.create();
const _translate_temp = /* @__PURE__ */ mat4.create();

/** translates mass properties by given translation vector */
export function translate(out: MassProperties, source: MassProperties, translation: Vec3): MassProperties {
    // copy input to output first
    mat4.copy(out.inertia, source.inertia);
    out.mass = source.mass;

    // apply parallel axis theorem: I' = I + m * (translation^2 * E - translation * translation^T)
    // where E is identity and translation^2 is the dot product of translation with itself

    const translationDotSelf = vec3.dot(translation, translation);

    // create scaling matrix: translation^2 * E (identity times scalar)
    const scaleMatrix = _translate_scaleMatrix;
    mat4.identity(scaleMatrix);
    scaleMatrix[0] *= translationDotSelf;
    scaleMatrix[5] *= translationDotSelf;
    scaleMatrix[10] *= translationDotSelf;

    // create outer product matrix: translation * translation^T
    const outerProduct = _translate_outerProduct;
    mat4.identity(outerProduct);
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            outerProduct[i + j * 4] = translation[i] * translation[j];
        }
    }

    // calculate: scaleMatrix - outerProduct
    const temp = _translate_temp;
    mat4.copy(temp, scaleMatrix);
    mat4.subtract(temp, temp, outerProduct);

    // multiply by mass and add to inertia
    for (let i = 0; i < 12; i++) {
        out.inertia[i] += source.mass * temp[i];
    }

    // ensure bottom-right element and padding are correct
    out.inertia[13] = 0;
    out.inertia[14] = 0;
    out.inertia[15] = 1.0;

    return out;
}

/** scales the inertia tensor to match a new mass, preserves the shape's moment of inertia distribution while matching the new mass */
export function scaleToMass(massProperties: MassProperties, newMass: number): MassProperties {
    if (massProperties.mass > 0) {
        // calculate how much we have to scale the inertia tensor
        const massScale = newMass / massProperties.mass;
        
        // update mass
        massProperties.mass = newMass;
        
        // scale the 3x3 inertia tensor (upper-left of the 4x4 matrix)
        for (let col = 0; col < 3; col++) {
            for (let row = 0; row < 3; row++) {
                massProperties.inertia[col * 4 + row] *= massScale;
            }
        }
    } else {
        // just set the mass if the current mass is zero
        massProperties.mass = newMass;
    }
    
    return massProperties;
}
