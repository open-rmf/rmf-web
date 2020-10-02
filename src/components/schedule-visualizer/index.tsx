import { makeStyles } from '@material-ui/core';
import * as RomiCore from '@osrf/romi-js-core-interfaces';
import Debug from 'debug';
import * as L from 'leaflet';
import React from 'react';
import { AttributionControl, ImageOverlay, LayersControl, Map as LMap, Pane } from 'react-leaflet';
import {
  Conflict,
  DefaultTrajectoryManager,
  RobotTrajectoryManager,
  Trajectory,
  TrajectoryResponse,
} from '../../robot-trajectory-manager';
import { NegotiationTrajectoryResponse } from '../../negotiation-status-manager';
import { AnimationSpeed, TrajectoryAnimation } from '../../settings';
import { toBlobUrl } from '../../util';
import { SettingsContext } from '../app-contexts';
import ColorManager from './colors';
import DoorsOverlay from './doors-overlay';
import LiftsOverlay from './lift-overlay';
import RobotTrajectoriesOverlay, { RobotTrajectoryContext } from './robot-trajectories-overlay';
import RobotTrajectory from './robot-trajectory';
import RobotsOverlay from './robots-overlay';
import {
  withFillAnimation,
  withFollowAnimation,
  withOutlineAnimation,
} from './trajectory-animations';
import WaypointsOverlay from './waypoints-overlay';
import DispensersOverlay from './dispensers-overlay';

const debug = Debug('ScheduleVisualizer');

const useStyles = makeStyles(() => ({
  map: {
    height: '100%',
    width: '100%',
    margin: 0,
    padding: 0,
  },
}));

export interface MapFloorLayer {
  level: RomiCore.Level;
  imageUrl: string;
  bounds: L.LatLngBounds;
}

export interface ScheduleVisualizerProps {
  buildingMap: Readonly<RomiCore.BuildingMap>;
  fleets: Readonly<RomiCore.FleetState[]>;
  trajManager?: Readonly<RobotTrajectoryManager>;
  negotiationTrajStore: Readonly<Record<string, NegotiationTrajectoryResponse>>;
  mapFloorLayerSorted: Readonly<string[]>;
  onDoorClick?(door: RomiCore.Door): void;
  onLiftClick?(lift: RomiCore.Lift): void;
  onRobotClick?(fleet: string, robot: RomiCore.RobotState): void;
  onDispenserClick?(dispenser: RomiCore.DispenserState): void;
}

function calcMaxBounds(mapFloorLayers: readonly MapFloorLayer[]): L.LatLngBounds | undefined {
  if (!mapFloorLayers.length) {
    return undefined;
  }
  const bounds = new L.LatLngBounds([0, 0], [0, 0]);
  Object.values(mapFloorLayers).forEach((x) => bounds.extend(x.bounds));
  return bounds.pad(0.2);
}

export default function ScheduleVisualizer(props: ScheduleVisualizerProps): React.ReactElement {
  debug('render');

  const { negotiationTrajStore, mapFloorLayerSorted } = props;
  const classes = useStyles();

  const [mapFloorLayers, setMapFloorLayers] = React.useState<
    Readonly<Record<string, MapFloorLayer>>
  >({});
  const [curLevelName, setCurLevelName] = React.useState(() => mapFloorLayerSorted[0]);
  const curMapFloorLayer = React.useMemo(() => mapFloorLayers[curLevelName], [
    curLevelName,
    mapFloorLayers,
  ]);

  const [trajectories, setTrajectories] = React.useState<Record<string, TrajectoryResponse>>({});
  const [conflictRobotNames, setConflictRobotNames] = React.useState<string[][]>(() => []);
  const [curMapTrajectories, setCurMapTrajectories] = React.useState<Trajectory[]>(() => []);
  const [curMapConflicts, setCurMapConflicts] = React.useState<Conflict[]>(() => []);

  const initialBounds = React.useMemo<Readonly<L.LatLngBounds> | undefined>(() => {
    const initialLayer = mapFloorLayers[mapFloorLayerSorted[0]];
    if (!initialLayer) {
      return undefined;
    }

    return initialLayer.bounds;
  }, [mapFloorLayers, mapFloorLayerSorted]);
  const [maxBounds, setMaxBounds] = React.useState<Readonly<L.LatLngBounds> | undefined>(() =>
    calcMaxBounds(Object.values(mapFloorLayers)),
  );
  const [bound, setBound] = React.useState(initialBounds);

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

  const RobotTrajContextValue = React.useMemo<RobotTrajectoryContext>(() => {
    const animationScale = trajLookahead / trajAnimDuration;
    switch (settings.trajectoryAnimation) {
      case TrajectoryAnimation.None:
        return { Component: RobotTrajectory };
      case TrajectoryAnimation.Fill:
        return { Component: withFillAnimation(RobotTrajectory, animationScale) };
      case TrajectoryAnimation.Follow:
        return { Component: withFollowAnimation(RobotTrajectory, animationScale) };
      case TrajectoryAnimation.Outline:
        return { Component: withOutlineAnimation(RobotTrajectory, animationScale) };
    }
  }, [settings.trajectoryAnimation, trajAnimDuration]);

  React.useEffect(() => {
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
          new Promise((res) => {
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
                [image.y_offset - height, image.x_offset],
                [image.y_offset, image.x_offset + width],
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
      debug('set map floor layers');
      setMapFloorLayers(mapFloorLayers);
      debug('set max bounds');
      setMaxBounds(calcMaxBounds(Object.values(mapFloorLayers)));
    })();
  }, [props.buildingMap]);

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
        debug('set trajectories');
        setTrajectories((prev) => ({
          ...prev,
          [curMapFloorLayer.level.name]: resp,
        }));
      }

      await updateTrajectory();
      interval = window.setInterval(updateTrajectory, trajAnimDuration);
    })();
    return () => clearInterval(interval);
  }, [props.trajManager, curMapFloorLayer, trajAnimDuration]);

  function determineMinZoom(width: number, height: number): number {
    if (Math.abs(width) >= Math.abs(height)) {
      // Right now baseWidth and baseZoom are set to 50 and 4
      // We might need to change this in future if a range of
      // values is given
      const baseWidth = 50;
      const baseZoom = 4;
      const widthToBaseRatio = width / baseWidth;
      const power = Math.log(widthToBaseRatio) / Math.log(2);
      // since we set zoomDelta and snap to 0.5, we can give min zoom a greater precision to a delta of 0.5
      return baseZoom - power - ((baseZoom - power) % 0.5);
    } else {
      const baseHeight = 40;
      const baseZoom = 4;
      const heightToBaseRatio = Math.abs(height) / baseHeight;
      const power = Math.log(heightToBaseRatio) / Math.log(2);
      return baseZoom - power - ((baseZoom - power) % 0.5);
    }
  }

  function handleBaseLayerChange(e: L.LayersControlEvent): void {
    debug('set current level name');
    setCurLevelName(e.name);
    const width = mapFloorLayers[e.name].bounds.getNorthEast().lng;
    const height = mapFloorLayers[e.name].bounds.getSouthWest().lat;
    const calculatedMinZoom = determineMinZoom(width, height);
    mapRef.current?.leafletElement.setMinZoom(calculatedMinZoom);
    mapRef.current?.leafletElement.setMaxZoom(calculatedMinZoom + 4);
    setBound(mapFloorLayers[e.name].bounds);
  }

  function getConflicts(levelName: string): Conflict[] {
    const resp = trajectories[levelName];
    return resp ? resp.conflicts : [];
  }

  const sortedMapFloorLayers = mapFloorLayerSorted.map((x) => mapFloorLayers[x]);
  const ref = React.useRef<ImageOverlay>(null);
  const mapRef = React.useRef<LMap>(null);

  if (ref.current) {
    ref.current.leafletElement.setZIndex(0);
  }

  React.useEffect(() => {
    function getTrajectory(levelName: string): Trajectory[] {
      const resp = trajectories[levelName];
      return resp ? resp.values : [];
    }

    function getConflicts(levelName: string): Conflict[] {
      const resp = trajectories[levelName];
      return resp ? resp.conflicts : [];
    }

    function getConflictRobotsName(conflicts: Conflict[], trajs: Trajectory[]): string[][] {
      let conflictRobotNames: string[][] = [];
      if (conflicts.length === 0) {
        return [];
      }
      conflicts.forEach((conflictPair) => {
        let robotNames: string[] = [];
        conflictPair.forEach((conflictId) => {
          const robotName = DefaultTrajectoryManager.getRobotNameFromPathId(conflictId, trajs);
          robotName && robotNames.push(robotName);
        });
        robotNames && conflictRobotNames.push(robotNames);
      });
      return conflictRobotNames;
    }

    if (curMapFloorLayer) {
      const mapTrajectories = getTrajectory(curMapFloorLayer.level.name);
      const mapConflicts = getConflicts(curMapFloorLayer.level.name);
      debug('set current map trajectories');
      setCurMapTrajectories(mapTrajectories);
      debug('set current map conflicts');
      setCurMapConflicts(mapConflicts);
      debug('set conflicting robots');
      setConflictRobotNames(getConflictRobotsName(mapConflicts, mapTrajectories));
    }
  }, [curMapFloorLayer, trajectories]);

  return (
    <LMap
      id="ScheduleVisualizer" // # data-* attrs are not set on the leaflet container
      className={classes.map}
      attributionControl={false}
      crs={L.CRS.Simple}
      minZoom={0}
      maxZoom={8}
      zoomDelta={0.5}
      zoomSnap={0.5}
      bounds={bound ? bound : initialBounds}
      maxBounds={maxBounds}
      onbaselayerchange={handleBaseLayerChange}
      ref={mapRef}
    >
      <AttributionControl position="bottomright" prefix="OSRC-SG" />
      <LayersControl position="topleft">
        {sortedMapFloorLayers.every((x) => x) &&
          sortedMapFloorLayers.map((floorLayer, i) => (
            <LayersControl.BaseLayer
              checked={i === 0}
              name={floorLayer.level.name}
              key={floorLayer.level.name}
            >
              <ImageOverlay bounds={floorLayer.bounds} url={floorLayer.imageUrl} ref={ref} />
            </LayersControl.BaseLayer>
          ))}

        <LayersControl.Overlay name="Robot Trajectories" checked>
          {curMapFloorLayer && (
            <Pane>
              <RobotTrajectoryContext.Provider value={RobotTrajContextValue}>
                <RobotTrajectoriesOverlay
                  bounds={curMapFloorLayer.bounds}
                  trajs={curMapTrajectories}
                  conflicts={curMapConflicts}
                  colorManager={colorManager}
                  conflictRobotNames={conflictRobotNames}
                />
              </RobotTrajectoryContext.Provider>
            </Pane>
          )}
        </LayersControl.Overlay>

        <LayersControl.Overlay
          name="Negotiation Trajectories"
          data-component="NegotiationTrajCheckbox"
          checked
        >
          {curMapFloorLayer && (
            <Pane>
              <RobotTrajectoryContext.Provider value={RobotTrajContextValue}>
                <RobotTrajectoriesOverlay
                  bounds={curMapFloorLayer.bounds}
                  trajs={
                    negotiationTrajStore[curLevelName] &&
                    props.negotiationTrajStore[curLevelName].values
                  }
                  conflicts={getConflicts(curLevelName)}
                  colorManager={colorManager}
                  conflictRobotNames={conflictRobotNames}
                  overridePathColor={'orange'}
                />
              </RobotTrajectoryContext.Provider>
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
        <LayersControl.Overlay name="Lifts" checked>
          {curMapFloorLayer && (
            <Pane>
              <LiftsOverlay
                bounds={curMapFloorLayer.bounds}
                currentFloor={curLevelName}
                lifts={props.buildingMap.lifts}
                onLiftClick={props.onLiftClick}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Robots" checked>
          {curMapFloorLayer && (
            <Pane>
              <RobotsOverlay
                currentFloorName={curLevelName}
                bounds={curMapFloorLayer.bounds}
                fleets={props.fleets}
                colorManager={colorManager}
                onRobotClick={props.onRobotClick}
                conflictRobotNames={conflictRobotNames}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Waypoints" checked>
          {curMapFloorLayer && (
            <Pane>
              <WaypointsOverlay
                bounds={curMapFloorLayer.bounds}
                currentLevel={curMapFloorLayer.level}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
        <LayersControl.Overlay name="Dispensers" checked>
          {curMapFloorLayer && (
            <Pane>
              <DispensersOverlay
                currentFloorName={curLevelName}
                bounds={curMapFloorLayer.bounds}
                onDispenserClick={props.onDispenserClick}
              />
            </Pane>
          )}
        </LayersControl.Overlay>
      </LayersControl>
    </LMap>
  );
}
