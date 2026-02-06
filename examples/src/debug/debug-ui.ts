import { GUI } from 'lil-gui';
import type { World } from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import { createStats } from './stats';

export type DebugStats = {
    bodies: {
        count: number;
        activeBodies: number;
        poolSize: number;
    };
    contacts: {
        count: number;
    };
    contactConstraints: {
        count: number;
    };
    userConstraints: {
        count: number;
    };
    broadphase: {
        numPairs: number;
        pairsPoolSize: number;
        dbvtTotalNodes: number;
        dbvtActiveNodes: number;
        dbvtMaxDepth: number;
    };
};

export type DebugUI = {
    gui: GUI;
    stats: DebugStats;
    perfStats: ReturnType<typeof createStats>;
};

/**
 * initialize the debug ui with controls for the debug renderer options
 */
export function init(options: debugRenderer.DebugRendererOptions): DebugUI {
    const gui = new GUI();
    gui.title('Debug Renderer');

    const bodiesFolder = gui.addFolder('Bodies');
    bodiesFolder.add(options.bodies, 'enabled').name('Enabled');
    bodiesFolder
        .add(options.bodies, 'color', {
            Instance: debugRenderer.BodyColorMode.INSTANCE,
            'Motion Type': debugRenderer.BodyColorMode.MOTION_TYPE,
            Sleeping: debugRenderer.BodyColorMode.SLEEPING,
            Island: debugRenderer.BodyColorMode.ISLAND,
        })
        .name('Color Mode');
    bodiesFolder.add(options.bodies, 'wireframe').name('Wireframe');
    bodiesFolder.add(options.bodies, 'showLinearVelocity').name('Show Linear Velocity');
    bodiesFolder.add(options.bodies, 'showAngularVelocity').name('Show Angular Velocity');
    bodiesFolder.open();

    const contactsFolder = gui.addFolder('Contacts');
    contactsFolder.add(options.contacts, 'enabled').name('Enabled');
    contactsFolder.open();

    const contactConstraintsFolder = gui.addFolder('Contact Constraints');
    contactConstraintsFolder.add(options.contactConstraints, 'enabled').name('Enabled');
    contactConstraintsFolder.open();

    const constraintsFolder = gui.addFolder('Constraints');
    constraintsFolder.add(options.constraints, 'enabled').name('Enabled');
    constraintsFolder.add(options.constraints, 'drawLimits').name('Draw Limits');
    constraintsFolder.add(options.constraints, 'size', 0.1, 2.0).name('Size');
    constraintsFolder.open();

    const broadphaseDbvtFolder = gui.addFolder('Broadphase DBVT');
    broadphaseDbvtFolder.add(options.broadphaseDbvt, 'enabled').name('Enabled');
    broadphaseDbvtFolder.add(options.broadphaseDbvt, 'showLeafNodes').name('Show Leaf Nodes');
    broadphaseDbvtFolder.add(options.broadphaseDbvt, 'showNonLeafNodes').name('Show Non-Leaf Nodes');
    broadphaseDbvtFolder.open();

    const triangleMeshBvhFolder = gui.addFolder('Triangle Mesh BVH');
    triangleMeshBvhFolder.add(options.triangleMeshBvh, 'enabled').name('Enabled');
    triangleMeshBvhFolder.add(options.triangleMeshBvh, 'showLeafNodes').name('Show Leaf Nodes');
    triangleMeshBvhFolder.add(options.triangleMeshBvh, 'showNonLeafNodes').name('Show Non-Leaf Nodes');
    triangleMeshBvhFolder.open();

    const stats: DebugStats = {
        bodies: {
            count: 0,
            activeBodies: 0,
            poolSize: 0,
        },
        contacts: {
            count: 0,
        },
        contactConstraints: {
            count: 0,
        },
        userConstraints: {
            count: 0,
        },
        broadphase: {
            numPairs: 0,
            pairsPoolSize: 0,
            dbvtTotalNodes: 0,
            dbvtActiveNodes: 0,
            dbvtMaxDepth: 0,
        },
    };

    const statsFolder = gui.addFolder('Stats');
    statsFolder.add(stats.bodies, 'count').name('Bodies').listen().disable();
    statsFolder.add(stats.bodies, 'activeBodies').name('Active Bodies').listen().disable();
    statsFolder.add(stats.bodies, 'poolSize').name('Bodies Pool Size').listen().disable();
    statsFolder.add(stats.contacts, 'count').name('Contacts').listen().disable();
    statsFolder.add(stats.contactConstraints, 'count').name('Contact Constraints').listen().disable();
    statsFolder.add(stats.userConstraints, 'count').name('User Constraints').listen().disable();
    statsFolder.add(stats.broadphase, 'numPairs').name('Broadphase Pairs').listen().disable();
    statsFolder.add(stats.broadphase, 'pairsPoolSize').name('Broadphase Pairs Pool Size').listen().disable();
    statsFolder.add(stats.broadphase, 'dbvtTotalNodes').name('Broadphase DBVT Total Nodes').listen().disable();
    statsFolder.add(stats.broadphase, 'dbvtActiveNodes').name('Broadphase DBVT Active Nodes').listen().disable();
    statsFolder.add(stats.broadphase, 'dbvtMaxDepth').name('Broadphase DBVT Max Depth').listen().disable();

    // create performance stats panel
    const perfStats = createStats();
    perfStats.dom.style.position = 'relative';
    perfStats.dom.style.top = 'unset';
    perfStats.dom.style.left = 'unset';
    perfStats.dom.style.width = '100%';
    perfStats.dom.style.cursor = 'pointer';
    statsFolder.domElement.appendChild(perfStats.dom);

    statsFolder.open();

    return {
        gui,
        stats,
        perfStats,
    };
}

/**
 * update stats from world data
 */
export function updateStats(ui: DebugUI, world: World): void {
    ui.stats.bodies.count = world.bodies.pool.length - world.bodies.freeIndices.length;
    ui.stats.bodies.activeBodies = world.bodies.activeBodyCount;
    ui.stats.bodies.poolSize = world.bodies.pool.length;
    ui.stats.contacts.count = world.contacts.contacts.length - world.contacts.contactsFreeIndices.length;
    ui.stats.contactConstraints.count = world.contactConstraints.constraints.length;

    // count user constraints
    let userConstraintCount = 0;
    userConstraintCount +=
        world.constraints.pointConstraints.constraints.length - world.constraints.pointConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.distanceConstraints.constraints.length - world.constraints.distanceConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.hingeConstraints.constraints.length - world.constraints.hingeConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.fixedConstraints.constraints.length - world.constraints.fixedConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.swingTwistConstraints.constraints.length - world.constraints.swingTwistConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.sliderConstraints.constraints.length - world.constraints.sliderConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.coneConstraints.constraints.length - world.constraints.coneConstraints.freeIndices.length;
    userConstraintCount +=
        world.constraints.sixDOFConstraints.constraints.length - world.constraints.sixDOFConstraints.freeIndices.length;
    ui.stats.userConstraints.count = userConstraintCount;

    ui.stats.broadphase.numPairs = world.broadphase.pairs.n;
    ui.stats.broadphase.pairsPoolSize = world.broadphase.pairs.pool.length;

    // calculate dbvt stats across all broadphase layers
    let totalNodes = 0;
    let activeNodes = 0;
    let maxDepth = 0;

    for (const dbvt of world.broadphase.dbvts) {
        totalNodes += dbvt.nodes.length;
        activeNodes += dbvt.nodes.length - dbvt.freeNodeIndices.length;

        if (dbvt.root !== -1) {
            const stack: Array<{ nodeIndex: number; depth: number }> = [{ nodeIndex: dbvt.root, depth: 0 }];

            while (stack.length > 0) {
                const { nodeIndex, depth } = stack.pop()!;
                const node = dbvt.nodes[nodeIndex];

                maxDepth = Math.max(maxDepth, depth);

                if (node.left !== -1) {
                    stack.push({ nodeIndex: node.left, depth: depth + 1 });
                }
                if (node.right !== -1) {
                    stack.push({ nodeIndex: node.right, depth: depth + 1 });
                }
            }
        }
    }

    ui.stats.broadphase.dbvtTotalNodes = totalNodes;
    ui.stats.broadphase.dbvtActiveNodes = activeNodes;
    ui.stats.broadphase.dbvtMaxDepth = maxDepth;
}

/**
 * begin performance measurement - call before physics step
 */
export function beginPerf(ui: DebugUI): void {
    ui.perfStats.begin();
}

/**
 * end performance measurement - call after physics step
 */
export function endPerf(ui: DebugUI): number {
    return ui.perfStats.end();
}

/**
 * update performance stats display
 */
export function updatePerf(ui: DebugUI): void {
    ui.perfStats.update();
}
