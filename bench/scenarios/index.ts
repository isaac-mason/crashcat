import { boxStacks } from './box-stacks';
import { ccdCascade } from './ccd-cascade';
import { characterTerrain } from './character-terrain';
import { compoundPile } from './compound-pile';
import { convexPile } from './convex-pile';
import { convexVsMesh } from './convex-vs-mesh';
import { hingeChain } from './hinge-chain';
import { pyramid } from './pyramid';
import { ragdollPile } from './ragdoll-pile';
import { raycastMesh } from './raycast-mesh';
import type { Scenario } from './scenario';
import { seaOfStaticBoxes } from './sea-of-static-boxes';
import { tenThousandBoxes } from './ten-thousand-boxes';

export type { Scenario, ScenarioInstance } from './scenario';
export { runScenario } from './scenario';

export const SCENARIOS: Scenario[] = [
    boxStacks,
    pyramid,
    convexPile,
    compoundPile,
    tenThousandBoxes,
    seaOfStaticBoxes,
    convexVsMesh,
    raycastMesh,
    ragdollPile,
    hingeChain,
    ccdCascade,
    characterTerrain,
];

export function getScenario(name: string): Scenario {
    const scenario = SCENARIOS.find((s) => s.name === name);
    if (!scenario) {
        throw new Error(`unknown scenario "${name}", expected one of: ${SCENARIOS.map((s) => s.name).join(', ')}`);
    }
    return scenario;
}
