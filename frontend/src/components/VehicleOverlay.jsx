import { useEffect, useRef } from 'react'
import maplibregl from 'maplibre-gl'
import { xyToLngLat } from '../utils/geo.js'

const VEHICLE_COLORS = {
  uav: '#FF5722',
  ugv: '#3F51B5',
}

const FALLBACK_COLOR = '#9E9E9E'

function getVehicleColor(vehicleId) {
  return VEHICLE_COLORS[vehicleId] ?? FALLBACK_COLOR
}

function createMapMarkerElement(color) {
  const el = document.createElement('div')
  el.style.cssText = `
    width: 14px;
    height: 14px;
    border-radius: 50%;
    background: ${color};
    border: 2px solid #fff;
    box-shadow: 0 0 8px rgba(0,0,0,.6);
  `
  return el
}

function VehicleSvgMarker({ vehicle, toSvg, px }) {
  const [sx, sy] = toSvg(vehicle.x, vehicle.y)
  const color = getVehicleColor(vehicle.vehicle_id)
  const radius = 6.5 * px
  const labelOffsetY = 10 * px

  return (
    <g>
      <circle
        cx={sx}
        cy={sy}
        r={radius}
        fill={color}
        stroke="#fff"
        strokeWidth={2 * px}
      />
      <text
        x={sx}
        y={sy - labelOffsetY}
        fontSize={11 * px}
        textAnchor="middle"
        fill="#111827"
        stroke="#fff"
        strokeWidth={3 * px}
        paintOrder="stroke fill"
        style={{ userSelect: 'none', pointerEvents: 'none' }}
      >
        {vehicle.vehicle_id.toUpperCase()}
      </text>
    </g>
  )
}

export default function VehicleOverlay({
  renderer,
  vehicles,
  mapRef,
  ready,
  toSvg,
  px,
}) {
  const markersRef = useRef({})

  useEffect(() => {
    if (renderer !== 'map' || !ready || !mapRef?.current) {
      return
    }

    const activeIds = new Set()

    for (const vehicle of vehicles ?? []) {
      activeIds.add(vehicle.vehicle_id)

      const lngLat = xyToLngLat(vehicle.x, vehicle.y)

      if (markersRef.current[vehicle.vehicle_id]) {
        markersRef.current[vehicle.vehicle_id].setLngLat(lngLat)
      } else {
        const color = getVehicleColor(vehicle.vehicle_id)
        const markerElement = createMapMarkerElement(color)

        markersRef.current[vehicle.vehicle_id] = new maplibregl.Marker({
          element: markerElement,
        })
          .setLngLat(lngLat)
          .setPopup(
            new maplibregl.Popup({ offset: 16 }).setText(
              vehicle.vehicle_id.toUpperCase()
            )
          )
          .addTo(mapRef.current)
      }
    }

    for (const [vehicleId, marker] of Object.entries(markersRef.current)) {
      if (!activeIds.has(vehicleId)) {
        marker.remove()
        delete markersRef.current[vehicleId]
      }
    }
  }, [renderer, ready, vehicles, mapRef])

  useEffect(() => {
    return () => {
      for (const marker of Object.values(markersRef.current)) {
        marker.remove()
      }
      markersRef.current = {}
    }
  }, [])

  if (renderer !== 'svg') {
    return null
  }

  return (
    <>
      {(vehicles ?? []).map(vehicle => (
        <VehicleSvgMarker
          key={vehicle.vehicle_id}
          vehicle={vehicle}
          toSvg={toSvg}
          px={px}
        />
      ))}
    </>
  )
}