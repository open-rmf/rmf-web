import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { AttributionControl, ImageOverlay, LayersControl, Map as LMap } from 'react-leaflet';
import { Trajectory } from '../../robot-trajectory-manager';
import { toBlobUrl } from '../../util';
import ColorManager from './colors';
import PlacesOverlay from './places-overlay';
import RobotTrajectoriesOverlay from './robot-trajectories-overlay';
import RobotsOverlay from './robots-overlay';
import { makeStyles } from '@material-ui/core';

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

interface MapFloorState {
  name: string;
  imageUrl: string;
  bounds: L.LatLngBounds;
}

export interface ScheduleVisualizerProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  fleets: Readonly<RomiCore.FleetState[]>;
  trajs: readonly Trajectory[];
  onPlaceClick?(place: RomiCore.Place): void;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

export default function ScheduleVisualizer(props: ScheduleVisualizerProps): JSX.Element {
  const classes = useStyles();
  const mapRef = React.useRef<LMap>(null);
  const { current: mapElement } = mapRef;
  const [mapFloorStates, setMapFloorStates] = React.useState<Record<string, MapFloorState>>({});
  const colorManager = React.useMemo(() => new ColorManager(), []);

  // TODO: listen to overlayadded event to detect when an overlay is changed.
  const [currentLevel, setCurrentLevel] = React.useState<RomiCore.Level>(
    props.buildingMap.levels[0],
  );

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
      const newMapFloorStates: Record<string, MapFloorState> = {};
      for (const level of props.buildingMap.levels) {
        const image = level.images[0]; // when will there be > 1 image?
        if (!image) {
          continue;
        }

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
                [image.y_offset - height, image.x_offset + width],
              );

              newMapFloorStates[level.name] = {
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
      setMapFloorStates(m => ({ ...m, ...newMapFloorStates }));
    })();
  }, [props.buildingMap, mapElement]);

  const currentMapFloorState = mapFloorStates[currentLevel.name];
  const bounds = currentMapFloorState?.bounds;
  return (
    <LMap
      ref={mapRef}
      className={classes.map}
      attributionControl={false}
      crs={L.CRS.Simple}
      minZoom={4}
      maxZoom={8}
      zoomDelta={0.5}
      zoomSnap={0.5}
      bounds={bounds}
      maxBounds={currentMapFloorState?.bounds}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <LayersControl position="topleft">
        {Object.values(mapFloorStates).map((floorState, i) => (
          <LayersControl.BaseLayer checked={i === 0} name={floorState.name} key={floorState.name}>
            <ImageOverlay bounds={floorState.bounds} url={floorState.imageUrl} />
          </LayersControl.BaseLayer>
        ))}
        <LayersControl.Overlay name="Places" checked>
          {bounds && (
            <PlacesOverlay
              bounds={bounds}
              places={currentLevel.places}
              onPlaceClick={props.onPlaceClick}
            />
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Robot Trajectories" checked>
          {bounds && (
            <RobotTrajectoriesOverlay
              bounds={bounds}
              trajs={props.trajs}
              colorManager={colorManager}
            />
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Robots" checked>
          {bounds && (
            <RobotsOverlay
              bounds={bounds}
              fleets={props.fleets}
              colorManager={colorManager}
              onRobotClick={props.onRobotClick}
            />
          )}
        </LayersControl.Overlay>
      </LayersControl>
    </LMap>
  );
}
