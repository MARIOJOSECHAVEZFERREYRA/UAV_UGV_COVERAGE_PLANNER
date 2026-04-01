const VIEW_TOGGLE_CONTAINER = {
  position: 'absolute',
  top: 12,
  left: 12,
  zIndex: 20,
  display: 'flex',
  borderRadius: 6,
  overflow: 'hidden',
  border: '1px solid #374151',
}

export default function ViewToggle({ view, onChangeView }) {
  return (
    <div style={VIEW_TOGGLE_CONTAINER}>
      {['map', 'canvas'].map(currentView => (
        <button
          key={currentView}
          type="button"
          onClick={() => onChangeView(currentView)}
          aria-label={`Switch to ${currentView} view`}
          aria-pressed={view === currentView}
          style={{
            padding: '5px 14px',
            fontSize: 12,
            fontFamily: 'inherit',
            cursor: 'pointer',
            border: 'none',
            background: view === currentView ? '#38BDF8' : '#1f2937',
            color: view === currentView ? '#0f172a' : '#94a3b8',
            fontWeight: view === currentView ? 600 : 400,
          }}
        >
          {currentView === 'map' ? 'Map' : 'Canvas'}
        </button>
      ))}
    </div>
  )
}