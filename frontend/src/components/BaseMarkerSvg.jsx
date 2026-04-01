const DEFAULT_COLOR = '#ef4444'

export default function BaseMarkerSvg({
  sx,
  sy,
  r,
  strokeWidth,
  color = DEFAULT_COLOR,
}) {
  const arm = r * 0.55

  return (
    <g>
      <circle
        cx={sx}
        cy={sy}
        r={r}
        fill={color}
        stroke="#fff"
        strokeWidth={strokeWidth}
      />
      <line
        x1={sx - arm}
        y1={sy}
        x2={sx + arm}
        y2={sy}
        stroke="#fff"
        strokeWidth={strokeWidth}
      />
      <line
        x1={sx}
        y1={sy - arm}
        x2={sx}
        y2={sy + arm}
        stroke="#fff"
        strokeWidth={strokeWidth}
      />
    </g>
  )
}