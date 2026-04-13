// ─── UI shell theme ────────────────────────────────────────────────────────────
export const C = {
  bg:        '#0d1117',
  surface:   '#161b22',
  border:    '#21262d',
  accent:    '#58a6ff',
  accentDim: '#1f6feb',
  text:      '#e6edf3',
  muted:     '#8b949e',
  success:   '#3fb950',
  warning:   '#d29922',
  error:     '#f85149',
  danger:    '#da3633',
}

// ─── Trajectory layer colors ───────────────────────────────────────────────────
export const TRAJ = {
  sweep:    '#22c55e',
  ferry:    '#f59e0b',
  deadhead: '#ef4444',
}

// ─── Per-cycle color palette ───────────────────────────────────────────────────
// Sweep and deadhead of the same cycle share the same color.
// Ferry always uses TRAJ.ferry regardless of cycle.
export const CYCLE_PALETTE = [
  '#22c55e',  // green
  '#38bdf8',  // sky
  '#a78bfa',  // violet
  '#f472b6',  // pink
  '#2dd4bf',  // teal
  '#f87171',  // coral
  '#84cc16',  // lime
  '#818cf8',  // indigo
]

// ─── Field and drawing colors ──────────────────────────────────────────────────
export const DRAW = {
  field:    '#38BDF8',
  obstacle: '#ef4444',
  ugv:      '#fb923c',
}
