#[macro_use]
extern crate neon;
extern crate neon_serde;
#[macro_use]
extern crate serde_derive;

use std::f64::{self, consts::PI, INFINITY};

use neon::prelude::*;

#[derive(Clone, Copy)]
pub struct CheapRuler {
    kx: f64,
    ky: f64,
}

#[derive(Serialize)]
struct PoL {
    point: (f64, f64),
    index: usize,
    t: f64
}

impl CheapRuler {
    fn pointOnLine(&self, line: &Vec<(f64, f64)>, p: &(f64, f64)) -> PoL {
        let mut minDist = INFINITY;
        let mut minX = 0.;
        let mut minY = 0.;
        let mut minI = 0;
        let mut minT = INFINITY;

        for i in 0..(line.len() - 1) {

            let mut x = line[i].0;
            let mut y = line[i].1;
            let mut dx = (line[i + 1].0 - x) * self.kx;
            let mut dy = (line[i + 1].1 - y) * self.ky;
            let mut t = 0.;

            if (dx != 0. || dy != 0.) {
                t = ((p.0 - x) * self.kx * dx + (p.1 - y) * self.ky * dy) / (dx * dx + dy * dy);

                if (t > 1.) {
                    x = line[i + 1].0;
                    y = line[i + 1].1;

                } else if (t > 0.) {
                    x += (dx / self.kx) * t;
                    y += (dy / self.ky) * t;
                }
            }

            dx = (p.0 - x) * self.kx;
            dy = (p.1 - y) * self.ky;

            let sqDist = dx * dx + dy * dy;
            if (sqDist < minDist) {
                minDist = sqDist;
                minX = x;
                minY = y;
                minI = i;
                minT = t;
            }
        }

        PoL {
            point: (minX, minY),
            index: minI,
            t: 0f64.max(1f64.min(minT))
        }
    }

    fn distance(&self, a: (f64, f64), b: (f64, f64)) -> f64 {
        let dx = (a.0 - b.0) * self.kx;
        let dy = (a.1 - b.1) * self.ky;
        (dx * dx + dy * dy).sqrt()
    }
}

declare_types! {
    pub class JsCheapRuler for CheapRuler {
        init(mut cx) {
            let lat = cx.argument::<JsNumber>(0)?.value();
            let m = if cx.len() > 1 {
                let units = cx.argument::<JsString>(1)?.value();
                match units.as_str() {
                    "kilometers" => 1.,
                    "miles" => 1000. / 1609.344,
                    "nauticalmiles" => 1000. / 1852.,
                    "meters" => 1000.,
                    "metres" => 1000.,
                    "yards" => 1000. / 0.9144,
                    "feet" => 1000. / 0.3048,
                    "inches" => 1000. / 0.0254,
                    _ => return cx.throw_type_error("Unknown unit"),
                }
            } else {
                1.
            };

            let cos = (lat * PI / 180.).cos();
            let cos2 = 2. * cos * cos - 1.;
            let cos3 = 2. * cos * cos2 - cos;
            let cos4 = 2. * cos * cos3 - cos2;
            let cos5 = 2. * cos * cos4 - cos3;

            // multipliers for converting longitude and latitude degrees into distance (http://1.usa.gov/1Wb1bv7)
            Ok(CheapRuler {
                kx: m * (111.41513 * cos - 0.09455 * cos3 + 0.00012 * cos5),
                ky: m * (111.13209 - 0.56605 * cos2 + 0.0012 * cos4),
            })
        }

        // Given two points of the form [longitude, latitude], returns the distance.
        method distance(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let (arg0, arg1) = (cx.argument(0)?, cx.argument(1)?);
            let a: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let b: (f64, f64) = neon_serde::from_value(&mut cx, arg1)?;

            Ok(cx.number(ruler.distance(a, b)).upcast())
        }

        // Returns the bearing between two points in angles.
        method bearing(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let (arg0, arg1) = (cx.argument(0)?, cx.argument(1)?);
            let a: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let b: (f64, f64) = neon_serde::from_value(&mut cx, arg1)?;

            let dx = (b.0 - a.0) * ruler.kx;
            let dy = (b.1 - a.1) * ruler.ky;
            if dx == 0. && dy == 0. {
                return Ok(cx.number(0.).upcast());
            }
            let mut bearing = dx.atan2(dy) * 180. / PI;
            if bearing > 180. {
                bearing -= 360.;
            }
            Ok(cx.number(bearing).upcast())
        }

        // Returns a new point given distance and bearing from the starting point.
        method destination(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let p: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let dist = cx.argument::<JsNumber>(1)?.value();
            let bearing = cx.argument::<JsNumber>(2)?.value();
            let a = bearing * PI / 180.;

            let dx = a.sin() * dist;
            let dy = a.cos() * dist;
            return Ok(neon_serde::to_value(&mut cx, &vec![
                p.0 + dx / ruler.kx,
                p.1 + dy / ruler.ky
            ])?);
        }

        // Returns a new point given easting and northing offsets (in ruler units) from the starting point.
        method offset(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let p: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let dx = cx.argument::<JsNumber>(1)?.value();
            let dy = cx.argument::<JsNumber>(2)?.value();

            return Ok(neon_serde::to_value(&mut cx, &vec![
                p.0 + dx / ruler.kx,
                p.1 + dy / ruler.ky
            ])?);
        }

        // Given a line (an array of points), returns the total line distance.
        method lineDistance(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let points: Vec<(f64, f64)> = neon_serde::from_value(&mut cx, arg0)?;

            let mut total = 0.;
            for i in 0..(points.len() - 1) {
                total += ruler.distance(points[i], points[i + 1])
            }
            Ok(cx.number(total).upcast())
        }


        // Given a polygon (an array of rings, where each ring is an array of points), returns the area.
        method area(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let polygon: Vec<Vec<(f64, f64)>> = neon_serde::from_value(&mut cx, arg0)?;

            let mut sum = 0.;

            for (i, ring) in polygon.iter().enumerate() {
                let mut j = 0;
                let mut k = ring.len() - 1;
                while j < ring.len() {
                    sum += (ring[j].0 - ring[k].0) * (ring[j].1 + ring[k].1) * (if i > 0 { -1. } else { 1. });
                    k = j;
                    j = j + 1;
                }
            }

            Ok(cx.number((sum.abs() / 2.) * ruler.kx * ruler.ky).upcast())
        }

        // Returns the point at a specified distance along the line.
        method along(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let line: Vec<(f64, f64)> = neon_serde::from_value(&mut cx, arg0)?;
            let dist = cx.argument::<JsNumber>(1)?.value();

            let mut sum = 0.;

            if dist <= 0. {
                return Ok(neon_serde::to_value(&mut cx, &line[0])?);
            }

            for i in 0..(line.len() - 1) {
                let p0 = line[i];
                let p1 = line[i + 1];
                let d = ruler.distance(p0, p1);
                sum += d;
                if sum > dist {
                    return Ok(neon_serde::to_value(&mut cx, &interpolate(p0, p1, (dist - (sum - d)) / d))?);
                }
            }

            return Ok(neon_serde::to_value(&mut cx, &line[line.len() - 1])?);
        }

        // Returns an object of the form {point, index, t}, where point is closest point on the line
        method pointOnLine(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let (arg0, arg1) = (cx.argument(0)?, cx.argument(1)?);
            let line: Vec<(f64, f64)> = neon_serde::from_value(&mut cx, arg0)?;
            let p: (f64, f64) = neon_serde::from_value(&mut cx, arg1)?;

            Ok(neon_serde::to_value(&mut cx, &ruler.pointOnLine(&line, &p))?)
        }

        // Returns a part of the given line between the start and the stop points (or their closest points on the line).
        method lineSlice(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let (arg0, arg1, arg2) = (cx.argument(0)?, cx.argument(1)?, cx.argument(2)?);
            let start: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let stop: (f64, f64) = neon_serde::from_value(&mut cx, arg1)?;
            let line: Vec<(f64, f64)> = neon_serde::from_value(&mut cx, arg2)?;

            let mut p1 = ruler.pointOnLine(&line, &start);
            let mut p2 = ruler.pointOnLine(&line, &stop);

            if (p1.index > p2.index || (p1.index == p2.index && p1.t > p2.t)) {
                let tmp = p1;
                p1 = p2;
                p2 = tmp;
            }

            let mut slice = vec![p1.point];

            let l = p1.index + 1;
            let r = p2.index;

            if (line[l] != slice[0]) && l <= r {
                slice.push(line[l]);
            }

            for i in (l + 1)..=r {
                slice.push(line[i]);
            }

            if line[r] != p2.point {
                slice.push(p2.point);
            }

            Ok(neon_serde::to_value(&mut cx, &slice)?)
        }

        // Returns a part of the given line between the start and the stop points indicated by distance along the line.
        method lineSliceAlong(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let start: f64 = cx.argument::<JsNumber>(0)?.value();
            let stop: f64 = cx.argument::<JsNumber>(1)?.value();
            let arg2 = cx.argument(2)?;
            let line: Vec<(f64, f64)> = neon_serde::from_value(&mut cx, arg2)?;

            let mut sum = 0.;
            let mut slice = vec![];

            for i in 0..(line.len() - 1) {
                let p0 = line[i];
                let p1 = line[i + 1];
                let d = ruler.distance(p0, p1);

                sum += d;

                if (sum > start && slice.len() == 0) {
                    slice.push(interpolate(p0, p1, (start - (sum - d)) / d));
                }

                if (sum >= stop) {
                    slice.push(interpolate(p0, p1, (stop - (sum - d)) / d));
                    return Ok(neon_serde::to_value(&mut cx, &slice)?);
                }

                if (sum > start) {
                    slice.push(p1);
                }
            }

            Ok(neon_serde::to_value(&mut cx, &slice)?)
        }

        // Given a point, returns a bounding box object ([w, s, e, n]) created from the given point buffered by a given distance.
        method bufferPoint(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let p: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let buffer: f64 = cx.argument::<JsNumber>(1)?.value();

            let v = buffer / ruler.ky;
            let h = buffer / ruler.kx;
            let out = vec![
                p.0 - h,
                p.1 - v,
                p.0 + h,
                p.1 + v
            ];
            Ok(neon_serde::to_value(&mut cx, &out)?)
        }

        // Given a bounding box, returns the box buffered by a given distance.
        method bufferBBox(mut cx) {
            let this = cx.this();
            let ruler = {
                let guard = cx.lock();
                let obj = this.borrow(&guard);
                obj.clone()
            };
            let arg0 = cx.argument(0)?;
            let bbox: Vec<f64> = neon_serde::from_value(&mut cx, arg0)?;
            let buffer: f64 = cx.argument::<JsNumber>(1)?.value();

            let v = buffer / ruler.ky;
            let h = buffer / ruler.kx;
            let out = vec![
                bbox[0] - h,
                bbox[1] - v,
                bbox[2] + h,
                bbox[3] + v
            ];
            Ok(neon_serde::to_value(&mut cx, &out)?)
        }

        // Returns true if the given point is inside in the given bounding box, otherwise false.
        method insideBBox(mut cx) {
            let (arg0, arg1) = (cx.argument(0)?, cx.argument(1)?);
            let p: (f64, f64) = neon_serde::from_value(&mut cx, arg0)?;
            let bbox: Vec<f64> = neon_serde::from_value(&mut cx, arg1)?;

            Ok(cx.boolean(
                p.0 >= bbox[0] &&
                p.0 <= bbox[2] &&
                p.1 >= bbox[1] &&
                p.1 <= bbox[3]
           ).upcast())
        }
    }
}

register_module!(mut m, {
    m.export_class::<JsCheapRuler>("CheapRuler")?;

    Ok(())
});


fn interpolate(a: (f64, f64), b: (f64, f64), t: f64) -> (f64, f64) {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    (
        a.0 + dx * t,
        a.1 + dy * t
    )
}
