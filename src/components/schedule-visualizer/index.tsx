import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import {
  AttributionControl,
  ImageOverlay,
  LayersControl,
  Map as _Map,
  MapProps,
} from 'react-leaflet';
import styled from 'styled-components';
import { IMAGE_SCALE } from '../../constants';
import { toBlobUrl } from '../../util';

const Map = styled(_Map)`
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
`;

interface MapFloor {
  name: string;
  elevation: number;
  imageUrl: any;
  bounds: L.LatLngBounds;
}

export interface State {
  floors: MapFloor[];
  date: Date;
  maxBounds: L.LatLngBounds;
}

export interface ScheduleVisualizerProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
}

export default function ScheduleVisualizer(props: ScheduleVisualizerProps) {
  const mapRef = React.useRef<_Map>(null);
  const { current: mapElement } = mapRef;
  const [floors, setFloors] = React.useState<MapFloor[]>([]);
  const [mapProps, setMapProps] = React.useState<Partial<MapProps>>({});

  React.useEffect(() => {
    if (!mapElement) {
      return;
    }

    (async () => {
      const promises: Promise<any>[] = [];
      const mapFloors: MapFloor[] = [];
      let maxBounds = new L.LatLngBounds([0, 0], [0, 0]);

      for (const level of props.buildingMap.levels) {
        const { elevation, images } = level;
        const image = images[0]; // when will there be > 1 image?
        const { x_offset, y_offset, scale } = image;

        const imageElement = new Image();
        const blobUrl = toBlobUrl(image.data);
        imageElement.src = blobUrl;
        promises.push(
          new Promise(res => {
            const listener = () => {
              imageElement.removeEventListener('load', listener);
              const width = imageElement.naturalWidth * scale;
              const height = imageElement.naturalHeight * scale;

              // TODO: support both svg and image
              // const svgElement = rawCompressedSVGToSVGSVGElement(image.data);
              // const height = (svgElement.height.baseVal.value * scale) / IMAGE_SCALE;
              // const width = (svgElement.width.baseVal.value * scale) / IMAGE_SCALE;

              const offsetPixelsX = x_offset * IMAGE_SCALE;
              const offsetPixelsY = y_offset * IMAGE_SCALE;

              const floor: MapFloor = {
                name: level.name,
                elevation: elevation,
                bounds: new L.LatLngBounds(
                  new L.LatLng(offsetPixelsY, offsetPixelsX, elevation),
                  new L.LatLng(offsetPixelsY + height, offsetPixelsX + width, elevation),
                ),
                // imageData: SVGSVGElementToDataURI(svgElement),
                imageUrl: blobUrl,
              };
              maxBounds = maxBounds.extend(floor.bounds);
              mapFloors.push(floor);
              res();
            };
            imageElement.addEventListener('load', listener);
          }),
        );
      }

      for (const p of promises) {
        await p;
      }
      setMapProps({ ...mapProps, maxBounds: maxBounds });
      const bestZoom = 10 - ~~Math.log2(maxBounds.getEast());
      mapElement.leafletElement.setMinZoom(bestZoom - 2);
      mapElement.leafletElement.setMaxZoom(bestZoom + 2);
      mapElement.leafletElement.fitBounds(mapFloors[0].bounds);
      setFloors(mapFloors.slice());
      L.control.scale().addTo(mapElement.leafletElement);
    })();
  }, [props.buildingMap, mapElement]);

  return (
    <Map
      ref={mapRef}
      attributionControl={false}
      crs={L.CRS.Simple}
      zoomDelta={0.5}
      zoomSnap={0.5}
      wheelPxPerZoomLevel={120}
      {...mapProps}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      {/* <ServerDateControl date={date} position="topright" /> */}
      <LayersControl position="topleft">
        {floors.map((floor, i) => (
          <LayersControl.BaseLayer checked={i === 0} name={floor.name} key={floor.name}>
            <ImageOverlay bounds={floor.bounds} url={floor.imageUrl} />
          </LayersControl.BaseLayer>
        ))}
        <LayersControl.Overlay name="Robots Trajectories" checked>
          {/* <RobotTrajectoriesOverlay /> */}
        </LayersControl.Overlay>
      </LayersControl>
      {/* <SliderControl /> */}
    </Map>
  );
}
