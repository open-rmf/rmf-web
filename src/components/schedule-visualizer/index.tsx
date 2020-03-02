import React from 'react'
import {
  AttributionControl,
  ImageOverlay,
  LayersControl,
  Map as _Map,
} from 'react-leaflet'
import { produce } from 'immer'
import * as L from 'leaflet'
import styled from 'styled-components'

import { IFloor as _IFloor } from '../../models/Floor'
import { IAffineImage as _IAffineImage } from '../../models/AffineImage'
import { getFloors } from '../../mock'
import { rawCompressedSVGToSVGSVGElement, SVGSVGElementToDataURI } from '../../util'

import { IMAGE_SCALE } from '../../constants'
import RobotTrajectoriesOverlay from './robot-trajectories-overlay'
import ServerDateControl from './server-date-control'
// import SliderControl from './slider-control'
// import { clockSource } from '../..'

const Map = styled(_Map)`
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
`

export interface IAffineImage extends Omit<_IAffineImage, 'data'> {
  data: string
}

export interface IFloor extends Omit<_IFloor, 'image'> {
  image: IAffineImage
  bounds?: L.LatLngBounds
}

export interface State {
  floors: IFloor[]
  date: Date
  maxBounds: L.LatLngBounds
}

export interface Props {}

export default function ScheduleVisualizer() {
  const mapRef = React.useRef<_Map>(null)
  const { current: mapElement } = mapRef
  const [floors, setFloors] = React.useState<IFloor[]>([])
  const [maxBounds, setMaxBounds] = React.useState(new L.LatLngBounds([0, 0], [0, 0]))
  const [date, setDate] = React.useState(new Date())

  React.useEffect(() => {
    const cb = async (time: number) => {
      setDate(new Date(time))
    }

    // clockSource.addOnClockUpdateCallback(cb)

    return function cleanup() {
      // clockSource.removeOnClockUpdateCallback(cb)
    }
  }, []);

  React.useEffect(() => {
    getFloors().then((floors) => {
      setFloors(produce(floors, (draft: any) => {
        for (const floor of draft) {
          const { elevation, image } = floor
          const { pose, scale } = image
          const { x, y } = pose

          const svgElement = rawCompressedSVGToSVGSVGElement(image.data)
          const height = svgElement.height.baseVal.value * scale / IMAGE_SCALE
          const width = svgElement.width.baseVal.value * scale / IMAGE_SCALE

          const offsetPixelsX = x / IMAGE_SCALE
          const offsetPixelsY = y / IMAGE_SCALE

          floor.bounds = new L.LatLngBounds(
            new L.LatLng(offsetPixelsY, offsetPixelsX, elevation),
            new L.LatLng(
              offsetPixelsY - height,
              offsetPixelsX + width,
              elevation,
            ),
          )

          floor.image.data = SVGSVGElementToDataURI(svgElement)

          setMaxBounds(maxBounds.extend(floor.bounds))
        }
        return draft as IFloor[]
      }))
    })

    if (mapElement) {
      mapElement.leafletElement.fitBounds(maxBounds);
    }
  }, [maxBounds, mapElement])

  return (
    <Map
      ref={mapRef}
      attributionControl={false}
      crs={L.CRS.Simple}
      minZoom={-2}
      maxZoom={2}
      maxBounds={maxBounds}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      {/* <ServerDateControl date={date} position="topright" /> */}
      <LayersControl position="topright">
        {
          floors.map((floor, i) => (
            <LayersControl.BaseLayer checked={i === 0} name={floor.name} key={floor.name}>
              <ImageOverlay bounds={floor.bounds} url={floor.image.data} />
            </LayersControl.BaseLayer>
          ))
        }
        <LayersControl.Overlay name="Robots Trajectories" checked>
          {/* <RobotTrajectoriesOverlay /> */}
        </LayersControl.Overlay>
      </LayersControl>
      {/* <SliderControl /> */}
    </Map>
  )
}