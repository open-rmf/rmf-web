import React from 'react'
import { Polyline, Marker, CircleMarker } from 'react-leaflet'
// import { webSocketManager } from '../..'
import { TrajectoryResponse, fromRawKnotsToKnots } from '../../models/Trajectory'
import { LatLngExpression } from 'leaflet'
import { IMAGE_SCALE } from '../../constants'

export interface Trajectory {
  shape: string
  dimensions: number[]
  positions: LatLngExpression[]
}

export default function RobotTrajectoriesOverlay() {
  const [trajectories, setTrajectories] = React.useState<Trajectory[]>([])

  const onMessageCallback = React.useRef(async (event: WebSocketMessageEvent) => {
    const data: TrajectoryResponse = JSON.parse(event.data)

    if (data.response !== 'trajectory') return

    const { values } = data
    const trajectories: Trajectory[] = []

    if (!values) {
      setTrajectories([])
      return
    }

    for (const value of values) {
      const { shape, dimensions } = value
      const trajectory: Trajectory = {
        shape,
        dimensions,
        positions: [],
      }

      const { segments } = value
      const knots = fromRawKnotsToKnots(segments)

      for (const knot of knots) {
        const { pose } = knot
        const { x, y } = pose
        trajectory.positions.push([
          y / IMAGE_SCALE,
          x / IMAGE_SCALE,
        ])
      }
      trajectories.push(trajectory)
    }

    console.log(trajectories)

    setTrajectories(trajectories)
  })

  // React.useEffect(() => {
  //   webSocketManager.addOnMessageCallback(onMessageCallback.current)
  //   return function cleanup() {
  //     webSocketManager.removeOnMessageCallback(onMessageCallback.current)
  //   }
  // }, [onMessageCallback])

  return (
    <React.Fragment>
      <CircleMarker center={[0, 0]} radius={5}/>
      {
        trajectories.map((trajectory, i) => (
          <Polyline key={i} positions={trajectory.positions} />
        ))
      }
    </React.Fragment>
  )
}