import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { AttributionControl, ImageOverlay, LayersControl, Map as _Map } from 'react-leaflet';
import styled from 'styled-components';
import { toBlobUrl } from '../../util';

const Map = styled(_Map)`
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
`;

interface MapFloorState {
  name: string;
  imageUrl: string;
  bounds: L.LatLngBounds;
}

export interface ScheduleVisualizerProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
}

export default function ScheduleVisualizer(props: ScheduleVisualizerProps) {
  const mapRef = React.useRef<_Map>(null);
  const { current: mapElement } = mapRef;
  const [mapFloorStates, setMapFloorStates] = React.useState<Record<string, MapFloorState>>({});

  // TODO: listen to overlayadded event to detect when an overlay is changed.
  const [currentLevel, setCurrentLevel] = React.useState<string>(props.buildingMap.levels[0].name);

  function handleMapImageLoad(target: L.ImageOverlay, floor: MapFloorState): void {}

  React.useEffect(() => {
    if (!mapElement) {
      return;
    }

    // We need the image to be loaded to know the bounds, but the image cannot be loaded without a
    // bounds, it is possible to use a temporary bounds but that would cause the viewport to move
    // when we replace the temporary bounds. A solution is to load the image in a temporary HTML
    // image element, then load the ImageOverlay in leaflet, the downside is that the image gets
    // loaded twice.
    (async () => {
      const promises: Promise<any>[] = [];
      for (const level of props.buildingMap.levels) {
        const { elevation, images } = level;
        const image = images[0]; // when will there be > 1 image?

        promises.push(
          new Promise(res => {
            const imageElement = new Image();
            const imageUrl = toBlobUrl(image.data);
            imageElement.src = imageUrl;

            const listener = () => {
              imageElement.removeEventListener('load', listener);
              const width = imageElement.naturalWidth * image.scale;
              const height = imageElement.naturalHeight * image.scale;
              // TODO: support both svg and image
              // const svgElement = rawCompressedSVGToSVGSVGElement(image.data);
              // const height = (svgElement.height.baseVal.value * scale) / IMAGE_SCALE;
              // const width = (svgElement.width.baseVal.value * scale) / IMAGE_SCALE;

              const bounds = new L.LatLngBounds(
                [image.y_offset, image.x_offset],
                [image.y_offset + height, image.x_offset + width],
              );

              mapFloorStates[level.name] = {
                name: level.name,
                imageUrl: imageUrl,
                bounds: bounds,
              };
              res();
            };
            imageElement.addEventListener('load', listener);
          }),
        );
      }

      for (const p of promises) {
        await p;
      }
      setMapFloorStates({ ...mapFloorStates });
    })();
  }, [props.buildingMap, mapElement]);

  const currentMapFloorState = mapFloorStates[currentLevel];
  return (
    <Map
      ref={mapRef}
      attributionControl={false}
      crs={L.CRS.Simple}
      minZoom={4}
      maxZoom={8}
      zoomDelta={0.5}
      zoomSnap={0.5}
      bounds={currentMapFloorState?.bounds}
      maxBounds={currentMapFloorState?.bounds}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <LayersControl position="topleft">
        {Object.values(mapFloorStates).map((floorState, i) => (
          <LayersControl.BaseLayer checked={i === 0} name={floorState.name} key={floorState.name}>
            <ImageOverlay
              bounds={floorState.bounds}
              url={floorState.imageUrl}
              onload={(event: L.LayerEvent) => handleMapImageLoad(event.target, floorState)}
            />
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
