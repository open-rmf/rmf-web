import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import * as L from 'leaflet';
import React from 'react';
import { AttributionControl, ImageOverlay, LayersControl, Map as LMap, Pane } from 'react-leaflet';
import {
  Conflict,
  RobotTrajectoryManager,
  Trajectory,
  TrajectoryResponse,
} from '../../robot-trajectory-manager';
import { AnimationSpeed, SettingsContext, TrajectoryAnimation } from '../../settings';
import { toBlobUrl } from '../../util';
import ColorManager from './colors';
import PlacesOverlay from './places-overlay';
import RobotTrajectoriesOverlay, { RobotTrajectoryContext } from './robot-trajectories-overlay';
import RobotTrajectory from './robot-trajectory';
import RobotsOverlay from './robots-overlay';
import {
  withFillAnimation,
  withFollowAnimation,
  withOutlineAnimation,
} from './trajectory-animations';
import DoorsOverlay from './doors-overlay';

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

interface MapFloorLayer {
  level: RomiCore.Level;
  imageUrl: string;
  bounds: L.LatLngBounds;
}

export interface ScheduleVisualizerProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  // doors: Readonly<RomiCore.DoorState[]>;
  fleets: Readonly<RomiCore.FleetState[]>;
  trajManager?: Readonly<RobotTrajectoryManager>;
  onDoorClick?(door: RomiCore.Door): void;
  onPlaceClick?(place: RomiCore.Place): void;
  onRobotClick?(robot: RomiCore.RobotState): void;
}

function calcMaxBounds(mapFloorLayers: readonly MapFloorLayer[]): L.LatLngBounds | undefined {
  if (!mapFloorLayers.length) {
    return undefined;
  }
  const bounds = new L.LatLngBounds([0, 0], [0, 0]);
  Object.values(mapFloorLayers).forEach(x => bounds.extend(x.bounds));
  return bounds.pad(0.2);
}

export default function ScheduleVisualizer(props: ScheduleVisualizerProps): React.ReactElement {
  const classes = useStyles();
  const mapRef = React.useRef<LMap>(null);
  const { current: mapElement } = mapRef;

  const [mapFloorLayers, setMapFloorLayers] = React.useState<
    Readonly<Record<string, MapFloorLayer>>
  >({});
  const mapFloorLayerSort = React.useMemo<string[]>(
    () => props.buildingMap.levels.sort((a, b) => a.elevation - b.elevation).map(x => x.name),
    [props.buildingMap],
  );
  const [curLevelName, setCurLevelName] = React.useState(() => mapFloorLayerSort[0]);
  const curMapFloorLayer = React.useMemo(() => mapFloorLayers[curLevelName], [
    curLevelName,
    mapFloorLayers,
  ]);

  const [trajectories, setTrajectories] = React.useState<Record<string, TrajectoryResponse>>({});

  const initialBounds = React.useMemo<Readonly<L.LatLngBounds> | undefined>(() => {
    const initialLayer = mapFloorLayers[mapFloorLayerSort[0]];
    if (!initialLayer) {
      return undefined;
    }
    return initialLayer.bounds;
  }, [mapFloorLayers, mapFloorLayerSort]);
  const [maxBounds, setMaxBounds] = React.useState<Readonly<L.LatLngBounds> | undefined>(() =>
    calcMaxBounds(Object.values(mapFloorLayers)),
  );

  const robotsInCurLevel = React.useMemo(() => {
    if (!curMapFloorLayer) {
      return [];
    }
    return props.fleets.flatMap(x =>
      x.robots.filter(r => r.location.level_name === curMapFloorLayer.level.name),
    );
  }, [props.fleets, curMapFloorLayer]);

  // const doorsInCurLevel = React.useMemo(() => {
  //   if (!curMapFloorLayer) {
  //     return [];
  //   }

  //   return props.buildingMap.levels.flatMap((level: any) => {
  //     const doors = level.name === curMapFloorLayer.level.name && level.doors;
  //     return doors;
  //   });
  // }, [curMapFloorLayer]);

  const colorManager = React.useMemo(() => new ColorManager(), []);

  const settings = React.useContext(SettingsContext);
  const trajLookahead = 60000; // 1 min
  const trajAnimDuration = React.useMemo(() => {
    switch (settings.trajectoryAnimationSpeed) {
      case AnimationSpeed.Slow:
        return 4000;
      case AnimationSpeed.Normal:
        return 2000;
      case AnimationSpeed.Fast:
        return 1000;
    }
  }, [settings]);
  const TrajectoryComponent = React.useMemo(() => {
    const animationScale = trajLookahead / trajAnimDuration;
    switch (settings.trajectoryAnimation) {
      case TrajectoryAnimation.None:
        return RobotTrajectory;
      case TrajectoryAnimation.Fill:
        return withFillAnimation(RobotTrajectory, animationScale);
      case TrajectoryAnimation.Follow:
        return withFollowAnimation(RobotTrajectory, animationScale);
      case TrajectoryAnimation.Outline:
        return withOutlineAnimation(RobotTrajectory, animationScale);
    }
  }, [settings.trajectoryAnimation, trajAnimDuration]);

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
      const mapFloorLayers: Record<string, MapFloorLayer> = {};
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

              mapFloorLayers[level.name] = {
                level: level,
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
      setMapFloorLayers(mapFloorLayers);
      setMaxBounds(calcMaxBounds(Object.values(mapFloorLayers)));
    })();
  }, [props.buildingMap, mapElement]);

  React.useEffect(() => {
    let interval: number;
    (async () => {
      const trajManager = props.trajManager;

      async function updateTrajectory() {
        if (!curMapFloorLayer || !trajManager) {
          return;
        }
        const resp = await trajManager.latestTrajectory({
          request: 'trajectory',
          param: {
            map_name: curMapFloorLayer.level.name,
            duration: trajLookahead,
            trim: true,
          },
        });
        setTrajectories(prev => ({
          ...prev,
          [curMapFloorLayer.level.name]: resp,
        }));
      }

      await updateTrajectory();
      interval = window.setInterval(updateTrajectory, trajAnimDuration);
    })();
    return () => clearInterval(interval);
  }, [props.trajManager, curMapFloorLayer, trajAnimDuration]);

  function handleBaseLayerChange(e: L.LayersControlEvent): void {
    setCurLevelName(e.name);
  }

  function getTrajectory(levelName: string): Trajectory[] {
    const resp = trajectories[levelName];
    return resp ? resp.values : [];
  }

  function getConflicts(levelName: string): Conflict[] {
    const resp = trajectories[levelName];
    return resp ? resp.conflicts : [];
  }

  const sortedMapFloorLayers = mapFloorLayerSort.map(x => mapFloorLayers[x]);
  const ref = React.useRef<ImageOverlay>(null);

  if (ref.current) {
    ref.current.leafletElement.setZIndex(0);
  }

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
      bounds={initialBounds}
      maxBounds={maxBounds}
      onbaselayerchange={handleBaseLayerChange}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <LayersControl position="topleft">
        {sortedMapFloorLayers.every(x => x) &&
          sortedMapFloorLayers.map((floorLayer, i) => (
            <LayersControl.BaseLayer
              checked={i === 0}
              name={floorLayer.level.name}
              key={floorLayer.level.name}
            >
              <ImageOverlay bounds={floorLayer.bounds} url={floorLayer.imageUrl} ref={ref} />
            </LayersControl.BaseLayer>
          ))}
        <LayersControl.Overlay name="Places" checked>
          {curMapFloorLayer && (
            <Pane>
              <PlacesOverlay
                bounds={curMapFloorLayer.bounds}
                places={curMapFloorLayer.level.places}
                onPlaceClick={props.onPlaceClick}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Robot Trajectories" checked>
          {curMapFloorLayer && (
            <Pane>
              <RobotTrajectoryContext.Provider value={{ Component: TrajectoryComponent }}>
                <RobotTrajectoriesOverlay
                  bounds={curMapFloorLayer.bounds}
                  trajs={getTrajectory(curMapFloorLayer.level.name)}
                  conflicts={getConflicts(curMapFloorLayer.level.name)}
                  colorManager={colorManager}
                />
              </RobotTrajectoryContext.Provider>
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Robots" checked>
          {curMapFloorLayer && (
            <Pane>
              <RobotsOverlay
                bounds={curMapFloorLayer.bounds}
                robots={robotsInCurLevel}
                colorManager={colorManager}
                onRobotClick={props.onRobotClick}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Doors" checked>
          {curMapFloorLayer && (
            <Pane>
              <DoorsOverlay
                bounds={curMapFloorLayer.bounds}
                doors={curMapFloorLayer.level.doors}
                onDoorClick={props.onDoorClick}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
      </LayersControl>
    </LMap>
  );
}
