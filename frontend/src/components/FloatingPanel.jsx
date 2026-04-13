import { C } from '../utils/colors.js'

const BASE_STYLE = {
  position: 'absolute',
  backdropFilter: 'blur(6px)',
  borderRadius: 8,
}

const TONE_STYLE = {
  default: {
    background: `${C.bg}cc`,
    border: `1px solid ${C.border}`,
    color: C.text,
  },
  accent: {
    background: `${C.bg}cc`,
    border: `1px solid ${C.border}`,
    color: C.accent,
  },
  error: {
    background: '#3d0f0fcc',
    border: `1px solid ${C.error}`,
    color: '#ffa198',
  },
}

export default function FloatingPanel({
  children,
  tone = 'default',
  style,
  ...props
}) {
  return (
    <div
      style={{
        ...BASE_STYLE,
        ...(TONE_STYLE[tone] ?? TONE_STYLE.default),
        ...style,
      }}
      {...props}
    >
      {children}
    </div>
  )
}
