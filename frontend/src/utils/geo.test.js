import { describe, it, expect } from 'vitest'
import {
  polygonArea,
  pointInPolygon,
  isPolygonInside,
  polygonsOverlap,
  wouldSelfIntersect,
} from './geo.js'

// ---------------------------------------------------------------------------
// Shared fixtures
// ---------------------------------------------------------------------------

// 100×100 m field
const FIELD = [[0,0],[100,0],[100,100],[0,100]]

// 20×20 m obstacle fully inside FIELD
const OBS_INSIDE = [[10,10],[30,10],[30,30],[10,30]]

// 20×20 m obstacle partially outside FIELD (straddles right edge)
const OBS_STRADDLE = [[90,10],[110,10],[110,30],[90,30]]

// 20×20 m obstacle fully outside FIELD
const OBS_OUTSIDE = [[200,200],[220,200],[220,220],[200,220]]

// Two non-overlapping obstacles inside FIELD
const OBS_A = [[10,10],[30,10],[30,30],[10,30]]
const OBS_B = [[50,50],[70,50],[70,70],[50,70]]

// Two overlapping obstacles inside FIELD
const OBS_OVERLAP_1 = [[10,10],[40,10],[40,40],[10,40]]
const OBS_OVERLAP_2 = [[30,30],[60,30],[60,60],[30,60]]

// ---------------------------------------------------------------------------
// polygonArea
// ---------------------------------------------------------------------------

describe('polygonArea', () => {
  it('square 10×10 = 100 m²', () => {
    expect(polygonArea([[0,0],[10,0],[10,10],[0,10]])).toBeCloseTo(100)
  })

  it('rectangle 20×5 = 100 m²', () => {
    expect(polygonArea([[0,0],[20,0],[20,5],[0,5]])).toBeCloseTo(100)
  })

  it('right triangle base 10 height 10 = 50 m²', () => {
    expect(polygonArea([[0,0],[10,0],[0,10]])).toBeCloseTo(50)
  })

  it('large field 100×100 = 10000 m²', () => {
    expect(polygonArea(FIELD)).toBeCloseTo(10_000)
  })

  it('CCW and CW winding return same area', () => {
    const cw  = [[0,0],[0,10],[10,10],[10,0]]
    const ccw = [[0,0],[10,0],[10,10],[0,10]]
    expect(polygonArea(cw)).toBeCloseTo(polygonArea(ccw))
  })
})

// ---------------------------------------------------------------------------
// pointInPolygon
// ---------------------------------------------------------------------------

describe('pointInPolygon', () => {
  it('center of square → inside', () => {
    expect(pointInPolygon([50,50], FIELD)).toBe(true)
  })

  it('far outside → false', () => {
    expect(pointInPolygon([500,500], FIELD)).toBe(false)
  })

  it('negative coords → false', () => {
    expect(pointInPolygon([-1,-1], FIELD)).toBe(false)
  })

  it('just inside bottom-left corner', () => {
    expect(pointInPolygon([1,1], FIELD)).toBe(true)
  })

  it('point inside obstacle is inside obstacle', () => {
    expect(pointInPolygon([20,20], OBS_INSIDE)).toBe(true)
  })

  it('point outside obstacle is not inside obstacle', () => {
    expect(pointInPolygon([5,5], OBS_INSIDE)).toBe(false)
  })
})

// ---------------------------------------------------------------------------
// isPolygonInside
// ---------------------------------------------------------------------------

describe('isPolygonInside', () => {
  it('obstacle fully inside field → true', () => {
    expect(isPolygonInside(OBS_INSIDE, FIELD)).toBe(true)
  })

  it('obstacle straddling field boundary → false', () => {
    expect(isPolygonInside(OBS_STRADDLE, FIELD)).toBe(false)
  })

  it('obstacle fully outside field → false', () => {
    expect(isPolygonInside(OBS_OUTSIDE, FIELD)).toBe(false)
  })

  it('field is not inside itself (same polygon) → false', () => {
    // All vertices are on boundary, not strictly inside
    expect(isPolygonInside(FIELD, FIELD)).toBe(false)
  })

  it('large polygon is not inside small polygon', () => {
    const small = [[40,40],[60,40],[60,60],[40,60]]
    expect(isPolygonInside(FIELD, small)).toBe(false)
  })
})

// ---------------------------------------------------------------------------
// polygonsOverlap
// ---------------------------------------------------------------------------

describe('polygonsOverlap', () => {
  it('two separate obstacles → false', () => {
    expect(polygonsOverlap(OBS_A, OBS_B)).toBe(false)
  })

  it('two overlapping polygons → true', () => {
    expect(polygonsOverlap(OBS_OVERLAP_1, OBS_OVERLAP_2)).toBe(true)
  })

  it('small polygon fully inside larger → true', () => {
    const large = [[0,0],[100,0],[100,100],[0,100]]
    const inner = [[10,10],[20,10],[20,20],[10,20]]
    expect(polygonsOverlap(inner, large)).toBe(true)
  })

  it('symmetry: overlap(A,B) === overlap(B,A)', () => {
    expect(polygonsOverlap(OBS_OVERLAP_1, OBS_OVERLAP_2))
      .toBe(polygonsOverlap(OBS_OVERLAP_2, OBS_OVERLAP_1))
  })

  it('completely outside polygons → false', () => {
    expect(polygonsOverlap(OBS_A, OBS_OUTSIDE)).toBe(false)
  })
})

// ---------------------------------------------------------------------------
// wouldSelfIntersect
// ---------------------------------------------------------------------------

describe('wouldSelfIntersect', () => {
  it('less than 2 existing points → never intersects', () => {
    expect(wouldSelfIntersect([], [5,5])).toBe(false)
    expect(wouldSelfIntersect([[0,0]], [5,5])).toBe(false)
  })

  it('valid triangle construction → no intersection', () => {
    const pts = [[0,0],[10,0],[10,10]]
    expect(wouldSelfIntersect(pts, [0,10])).toBe(false)
  })

  it('bowtie: crossing diagonal creates self-intersection', () => {
    // Building bowtie: (0,0)→(10,10)→(10,0) then add (0,10)
    // The closing edge (0,10)→(0,0) crosses (0,0)→(10,10)?
    // Actually test the forward edge: (10,0)→(0,10) crosses (0,0)→(10,10)
    const pts = [[0,0],[10,10],[10,0]]
    expect(wouldSelfIntersect(pts, [0,10])).toBe(true)
  })

  it('valid convex polygon construction does not trigger warning', () => {
    // Building a regular convex polygon step by step
    const pts = [[0,0],[50,0],[50,50]]
    expect(wouldSelfIntersect(pts, [0,50])).toBe(false)
  })

  it('spiral back causes intersection', () => {
    // L-shape that then tries to cross itself
    const pts = [[0,0],[10,0],[10,5],[2,5]]
    // Adding [2,-1] forward edge (2,5)→(2,-1) crosses (0,0)→(10,0)
    expect(wouldSelfIntersect(pts, [2,-1])).toBe(true)
  })

  it('adjacent segments share endpoint — not counted as crossing', () => {
    const pts = [[0,0],[10,0]]
    // Forward edge (10,0)→(5,5) does not cross (0,0)→(10,0) — they share (10,0)
    expect(wouldSelfIntersect(pts, [5,5])).toBe(false)
  })
})
