import { C, TRAJ } from '../utils/colors.js'
import FloatingPanel from './FloatingPanel.jsx'

export default function LayerToggle({ highlight, onHighlight }) {
  function renderButton(label, keyName, color) {
    const isActive = highlight === keyName

    return (
      <button
        key={keyName}
        type="button"
        onClick={() => onHighlight(isActive ? 'all' : keyName)}
        aria-label={
          isActive
            ? 'Show all layers'
            : `Highlight ${label.toLowerCase()} layer`
        }
        aria-pressed={isActive}
        style={{
          padding: '5px 13px',
          fontSize: 12,
          fontFamily: 'inherit',
          cursor: 'pointer',
          border: `1px solid ${isActive ? color : C.border}`,
          borderRadius: 5,
          background: isActive ? `${color}28` : C.surface,
          color: isActive ? color : C.muted,
          fontWeight: isActive ? 600 : 400,
          transition: 'all 0.15s',
        }}
      >
        {label}
      </button>
    )
  }

  return (
    <FloatingPanel
      style={{
        top: 10,
        right: 50,
        zIndex: 20,
        display: 'flex',
        gap: 6,
        alignItems: 'center',
        padding: '6px 10px',
      }}
    >
      {renderButton('Spray', 'sweep', TRAJ.sweep)}
      {renderButton('Transit', 'ferry', TRAJ.ferry)}
      {renderButton('Deadhead', 'deadhead', TRAJ.deadhead)}
    </FloatingPanel>
  )
}
