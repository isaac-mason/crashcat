import * as bodies from './body/bodies.js';
import * as broadphase from './broadphase/broadphase.js';
import * as ccd from './ccd.js';
import type { Constraints } from './constraints/constraints.js';
import * as contactConstraints from './constraints/contact-constraints.js';
import * as contacts from './contacts.js';
import * as islands from './islands.js';
import * as pairs from './pairs.js';
import type { WorldSettings } from './world-settings.js';
/** physics world state */
export type World = {
    /** world options */
    settings: WorldSettings;
    /** broadphase state */
    broadphase: broadphase.Broadphase;
    /** persistent overlapping-pair state (pair set, per-pair pose cache, per-frame emitted pairs) */
    pairs: pairs.Pairs;
    /** bodies state */
    bodies: bodies.Bodies;
    /** contact constraint state (manages all active constraints and cache) */
    contactConstraints: contactConstraints.ContactConstraints;
    /** user constraints (joints/constraints between bodies) */
    constraints: Constraints;
    /** contacts from the last tick */
    contacts: contacts.Contacts;
    /** island builder for grouping connected bodies */
    islands: islands.Islands;
    /** continuous collision detection state */
    ccd: ccd.CCD;
    /** previous frame's delta time (for warm start ratio calculation) */
    previousTimeStep: number;
};
/**
 * Creates a new physics world with the given settings
 * @param settings world settings
 * @returns a new physics world
 */
export declare function createWorld(settings: WorldSettings): World;
