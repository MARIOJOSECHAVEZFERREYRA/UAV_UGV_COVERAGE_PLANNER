import { useState } from 'react'
import { C } from '../utils/colors.js'

function formatSimTime(seconds) {
  const mins = Math.floor(seconds / 60)
  const secs = Math.floor(seconds % 60)
  return `${mins}:${secs.toString().padStart(2, '0')}`
}

function CtrlBtn({ onClick, title, children, variant = 'default' }) {
  const colors = {
    default: { bg: 'transparent',      border: C.border,  color: C.muted   },
    active:  { bg: `${C.accent}22`,    border: C.accent,  color: C.accent  },
    danger:  { bg: `${C.danger}20`,    border: C.danger,  color: '#ff7b72' },
    warning: { bg: `${C.warning}20`,   border: `${C.warning}80`, color: '#e3b341' },
  }
  const v = colors[variant] ?? colors.default

  return (
    <button
      onClick={onClick}
      title={title}
      style={{
        display: 'flex', alignItems: 'center', justifyContent: 'center',
        width: 28, height: 28,
        padding: 0,
        border: `1px solid ${v.border}`,
        borderRadius: 5,
        background: v.bg,
        color: v.color,
        fontSize: 13,
        cursor: 'pointer',
        fontFamily: 'inherit',
        transition: 'all 0.15s',
        flexShrink: 0,
      }}
    >
      {children}
    </button>
  )
}

function Sep() {
  return <div style={{ width: 1, height: 18, background: C.border, flexShrink: 0 }} />
}

const badgeBase = {
  position: 'absolute',
  bottom: 16,
  left: 16,
  zIndex: 10,
  display: 'flex',
  alignItems: 'center',
  background: `${C.bg}e6`,
  backdropFilter: 'blur(6px)',
  border: `1px solid ${C.border}`,
  borderRadius: 8,
  fontSize: 12,
  color: C.text,
}

export default function SimulationBadge({
  connected, simTimeS,
  playbackSpeed, isPaused,
  onPlaybackChange, onPause, onResume, onRestart, onDismiss,
}) {
  const [minimized, setMinimized] = useState(false)
  const speeds = [1, 2, 5, 10]

  const dot = (
    <span style={{
      width: 7, height: 7, borderRadius: '50%', flexShrink: 0,
      background: connected ? C.success : C.muted,
      boxShadow: connected ? `0 0 5px ${C.success}` : 'none',
    }} />
  )

  if (minimized) {
    return (
      <div style={{ ...badgeBase, gap: 6, padding: '5px 8px' }}>
        {dot}
        <span style={{ color: C.muted, fontVariantNumeric: 'tabular-nums' }}>
          {formatSimTime(simTimeS)}
        </span>
        <button
          title="Expand simulation controls"
          onClick={() => { setMinimized(false); onResume() }}
          style={{
            background: 'transparent', border: 'none',
            color: C.accent, cursor: 'pointer',
            fontSize: 13, padding: '0 2px',
            fontFamily: 'inherit', lineHeight: 1,
          }}
        >
          ⤢
        </button>
      </div>
    )
  }

  return (
    <div style={{ ...badgeBase, gap: 8, padding: '6px 10px' }}>
      {/* Status + time */}
      <span style={{ display: 'flex', alignItems: 'center', gap: 6, flexShrink: 0 }}>
        {dot}
        <span style={{ color: C.muted, fontVariantNumeric: 'tabular-nums' }}>
          {formatSimTime(simTimeS)}
        </span>
      </span>

      <Sep />

      {/* Playback controls */}
      <div style={{ display: 'flex', gap: 4, alignItems: 'center' }}>
        <CtrlBtn
          title={isPaused ? 'Resume' : 'Pause'}
          variant={isPaused ? 'warning' : 'default'}
          onClick={isPaused ? onResume : onPause}
        >
          {isPaused ? '▶' : '⏸'}
        </CtrlBtn>

        <CtrlBtn title="Restart from beginning" onClick={onRestart}>
          ⟳
        </CtrlBtn>

        <CtrlBtn title="Minimize" onClick={() => { setMinimized(true); onPause() }}>
          ✕
        </CtrlBtn>
      </div>

      <Sep />

      {/* Speed selector */}
      <div style={{ display: 'flex', gap: 3 }}>
        {speeds.map((speed) => {
          const active = !isPaused && playbackSpeed === speed
          return (
            <button
              key={speed}
              onClick={() => onPlaybackChange(speed)}
              style={{
                padding: '3px 7px',
                border: `1px solid ${active ? C.accent : C.border}`,
                background: active ? `${C.accent}22` : 'transparent',
                color: active ? C.accent : C.muted,
                borderRadius: 4,
                cursor: 'pointer',
                fontSize: 11,
                fontFamily: 'inherit',
                transition: 'all 0.15s',
              }}
            >
              {speed}x
            </button>
          )
        })}
      </div>
    </div>
  )
}
