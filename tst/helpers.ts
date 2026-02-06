import { addBroadphaseLayer, addObjectLayer, createWorld, createWorldSettings, enableCollision } from '../src';

export const createTestWorld = () => {
    const worldSettings = createWorldSettings();

    const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
    const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

    const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
    const OBJECT_LAYER_NOT_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
    enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_NOT_MOVING);

    const world = createWorld(worldSettings);

    return {
        world,
        layers: {
            BROADPHASE_LAYER_MOVING,
            BROADPHASE_LAYER_NOT_MOVING,
            OBJECT_LAYER_MOVING,
            OBJECT_LAYER_NOT_MOVING,
        },
    };
};
