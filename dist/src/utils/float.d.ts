/**
 * the smallest positive normal double.
 *
 * dividing by anything below this overflows to Infinity, so it is the cutoff for treating an
 * inverse effective mass as zero rather than inverting it.
 */
export declare const MIN_NORMAL = 2.2250738585072014e-308;
