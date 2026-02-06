export const DOF_ALL = dof(true, true, true, true, true, true);
export const DOF_TRANSLATION_ONLY = dof(true, true, true, false, false, false);
export const DOF_ROTATION_ONLY = dof(false, false, false, true, true, true);

/** encodes degrees of freedom into a bitmask */
export function dof(
    translationX: boolean,
    translationY: boolean,
    translationZ: boolean,
    rotationX: boolean,
    rotationY: boolean,
    rotationZ: boolean,
): number {
    let dof = 0;
    if (translationX) dof |= 0b000001;
    if (translationY) dof |= 0b000010;
    if (translationZ) dof |= 0b000100;
    if (rotationX) dof |= 0b001000;
    if (rotationY) dof |= 0b010000;
    if (rotationZ) dof |= 0b100000;
    return dof;
}