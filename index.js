'use strict';

const native = require('./native/index.node');
const CheapRuler = native.CheapRuler;

// A collection of very fast approximations to common geodesic measurements. Useful for performance-sensitive code that measures things on a city scale.
function cheapRuler(lat /*: number */, units /*: ?string */) {
    return units ? new CheapRuler(lat, units) : new CheapRuler(lat);
}

//  Creates a ruler object from tile coordinates (y and z). Convenient in tile-reduce scripts.
cheapRuler.fromTile = function (y, z, units) {
    let n = Math.PI * (1 - 2 * (y + 0.5) / Math.pow(2, z));
    let lat = Math.atan(0.5 * (Math.exp(n) - Math.exp(-n))) * 180 / Math.PI;
    return cheapRuler(lat, units);
};

module.exports = cheapRuler;
