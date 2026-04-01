import { getEdgeSegments, formatDistanceMeters } from '../utils/viewScene.js'

export default function EdgeLabelsSvg({ points, px, color, closed = true, toSvg }) {
  if (!points || points.length < 2) {
    return null
  }

  return getEdgeSegments(points, closed).map(segment => {
    const [sx1, sy1] = toSvg(segment.p1[0], segment.p1[1])
    const [sx2, sy2] = toSvg(segment.p2[0], segment.p2[1])

    return (
      <text
        key={segment.index}
        x={(sx1 + sx2) / 2}
        y={(sy1 + sy2) / 2 - 5 * px}
        fontSize={11 * px}
        textAnchor="middle"
        fill={color}
        stroke="#fff"
        strokeWidth={3 * px}
        paintOrder="stroke fill"
        style={{ userSelect: 'none', pointerEvents: 'none' }}
      >
        {formatDistanceMeters(segment.length)}
      </text>
    )
  })
}