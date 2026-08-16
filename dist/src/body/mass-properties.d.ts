import { type Mat4, type Vec3 } from 'math';
export type MassProperties = {
    /** the mass of the shape (kg) */
    mass: number;
    /** the inertia tensor of the shape (kg m^2) */
    inertia: Mat4;
};
/** creates mass properties with default values */
export declare function create(): MassProperties;
/** resets mass properties to default values */
export declare function reset(out: MassProperties): MassProperties;
/** copies mass properties from source to out */
export declare function copy(out: MassProperties, source: MassProperties): MassProperties;
/** sets mass and inertia of a solid box */
export declare function setMassAndInertiaOfSolidBox(out: MassProperties, boxSize: Vec3, density: number): MassProperties;
/** scales mass properties by given scale factors */
export declare function scale(out: MassProperties, source: MassProperties, inScale: Vec3): MassProperties;
/** rotates mass properties by given rotation matrix */
export declare function rotate(out: MassProperties, source: MassProperties, rotation: Mat4): MassProperties;
/** translates mass properties by given translation vector */
export declare function translate(out: MassProperties, source: MassProperties, translation: Vec3): MassProperties;
/** scales the inertia tensor to match a new mass, preserves the shape's moment of inertia distribution while matching the new mass */
export declare function scaleToMass(massProperties: MassProperties, newMass: number): MassProperties;
