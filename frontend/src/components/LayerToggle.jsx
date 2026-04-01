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
          border: `1px solid ${isActive ? color : '#374151'}`,
          borderRadius: 5,
          background: isActive ? color + '28' : '#1f2937',
          color: isActive ? color : '#94a3b8',
          fontWeight: isActive ? 600 : 400,
          transition: 'all 0.15s',
        }}
      >
        {label}
      </button>
    )
  }

  return (
    <div
      style={{
        position: 'absolute',
        bottom: 16,
        left: 16,
        zIndex: 20,
        display: 'flex',
        gap: 6,
        alignItems: 'center',
        background: '#0d1117cc',
        backdropFilter: 'blur(6px)',
        border: '1px solid #21262d',
        borderRadius: 8,
        padding: '6px 10px',
      }}
    >
      {renderButton('Spray', 'sweep', '#22c55e')}
      {renderButton('Transit', 'ferry', '#f59e0b')}
    </div>
  )
}