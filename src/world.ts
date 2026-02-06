import * as bodies from './body/bodies';
import * as broadphase from './broadphase/broadphase';
import * as ccd from './ccd';
import type { Constraints } from './constraints/constraints';
import * as constraints from './constraints/constraints';
import * as contactConstraints from './constraints/contact-constraints';
import * as contacts from './contacts';
import * as islands from './islands';
import type { WorldSettings } from './world-settings';

/** physics world state */
export type World = {
    /** world options */
    settings: WorldSettings;

    /** broadphase state */
    broadphase: broadphase.Broadphase;

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
export function createWorld(settings: WorldSettings): World {
    return {
        settings,
        bodies: bodies.init(),
        broadphase: broadphase.init(settings.layers),
        contactConstraints: contactConstraints.init(),
        constraints: constraints.init(),
        contacts: contacts.init(),
        islands: islands.init(),
        ccd: ccd.init(),
        previousTimeStep: 0,
    };
}
