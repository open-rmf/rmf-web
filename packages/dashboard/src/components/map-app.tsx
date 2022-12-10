import { styled } from '@mui/material';
import { BuildingMap, Dispenser, FleetState, Ingestor, Level } from 'api-client';
import Debug from 'debug';
import React from 'react';
import {
  affineImageBounds,
  ColorManager,
  getPlaces,
  LMap,
  loadAffineImage,
  Place,
  TrajectoryTimeControl,
} from 'react-components';
import { AttributionControl, ImageOverlay, LayersControl, Pane } from 'react-leaflet';
import { Subscription } from 'rxjs';
import appConfig from '../app-config';
import { ResourcesContext } from './app-contexts';
import { AppEvents } from './app-events';
import { DoorsOverlay } from './doors-overlay';
import { LiftsOverlay } from './lifts-overlay';
import { createMicroApp } from './micro-app';
import { RmfAppContext } from './rmf-app';
import { RobotData, RobotsOverlay } from './robots-overlay';
import { TrajectoriesOverlay, TrajectoryData } from './trajectories-overlay';
import { WaypointsOverlay } from './waypoints-overlay';
import { WorkcellData, WorkcellsOverlay } from './workcells-overlay';

import { Canvas, useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { PerspectiveCamera, PositionalAudio } from '@react-three/drei';
import { OrbitControls } from '@react-three/drei';
import { Level as BuidlingLevel } from './level';

const debug = Debug('MapApp');

const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const SettingsKey = 'mapAppSettings';
const colorManager = new ColorManager();

interface MapSettings {
  trajectoryTime: number;
}

export const MapApp = styled(
  createMicroApp('Map', () => {
    const rmf = React.useContext(RmfAppContext);
    const resourceManager = React.useContext(ResourcesContext);
    const [currentLevel, setCurrentLevel] = React.useState<Level | undefined>(undefined);
    const [disabledLayers, setDisabledLayers] = React.useState<Record<string, boolean>>({});

    const [buildingMap, setBuildingMap] = React.useState<BuildingMap | null>(null);

    const [dispensers, setDispensers] = React.useState<Dispenser[]>([]);
    const [dispensersData, setDispensersData] = React.useState<WorkcellData[]>([]);
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }
      const dispenserManager = resourceManager?.dispensers;
      if (!dispenserManager) return;
      (async () => {
        const dispenserResources = dispenserManager.dispensers;
        const availableData = dispensers.filter(
          (wc) =>
            wc.guid in dispenserResources &&
            dispenserResources[wc.guid].location.level_name === currentLevel.name,
        );
        const promises = availableData.map((wc) => dispenserManager.getIconPath(wc.guid));
        const icons = await Promise.all(promises);
        setDispensersData(
          availableData.map((wc, i) => ({
            guid: wc.guid,
            location: [
              dispenserResources[wc.guid].location.x,
              dispenserResources[wc.guid].location.y,
            ],
            iconPath: icons[i] || undefined,
          })),
        );
      })();
    }, [resourceManager?.dispensers, dispensers, currentLevel]);

    const [ingestors, setIngestors] = React.useState<Ingestor[]>([]);
    const [ingestorsData, setIngestorsData] = React.useState<WorkcellData[]>([]);
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }
      const dispenserManager = resourceManager?.dispensers;
      if (!dispenserManager) return;
      (async () => {
        const dispenserResources = dispenserManager.dispensers;
        const availableData = ingestors.filter(
          (wc) =>
            wc.guid in dispenserResources &&
            dispenserResources[wc.guid].location.level_name === currentLevel.name,
        );
        const promises = availableData.map((wc) => dispenserManager.getIconPath(wc.guid));
        const icons = await Promise.all(promises);
        setIngestorsData(
          availableData.map((wc, i) => ({
            guid: wc.guid,
            location: [
              dispenserResources[wc.guid].location.x,
              dispenserResources[wc.guid].location.y,
            ],
            iconPath: icons[i] || undefined,
          })),
        );
      })();
    }, [resourceManager?.dispensers, ingestors, currentLevel]);

    const [fleets, setFleets] = React.useState<FleetState[]>([]);

    const [waypoints, setWaypoints] = React.useState<Place[]>([]);

    const [trajectories, setTrajectories] = React.useState<TrajectoryData[]>([]);
    const [mapSettings, setMapSettings] = React.useState<MapSettings>(() => {
      const settings = window.localStorage.getItem(SettingsKey);
      return settings ? JSON.parse(settings) : { trajectoryTime: 300000 /* 5 min */ };
    });
    const trajectoryTime = mapSettings.trajectoryTime;
    const trajectoryAnimScale = trajectoryTime / (0.9 * TrajectoryUpdateInterval);
    const trajManager = rmf?.trajectoryManager;
    React.useEffect(() => {
      if (!currentLevel) {
        return;
      }

      let interval: number;
      let cancel = false;

      const updateTrajectory = async () => {
        debug('updating trajectories');

        if (cancel || !trajManager) return;

        const resp = await trajManager.latestTrajectory({
          request: 'trajectory',
          param: {
            map_name: currentLevel.name,
            duration: trajectoryTime,
            trim: true,
          },
          token: appConfig.authenticator.token,
        });
        const flatConflicts = resp.conflicts.flatMap((c) => c);

        debug('set trajectories');
        setTrajectories(
          resp.values.map((v) => ({
            trajectory: v,
            color: 'green',
            conflict: flatConflicts.includes(v.id),
            animationScale: trajectoryAnimScale,
            loopAnimation: false,
          })),
        );
      };

      updateTrajectory();
      interval = window.setInterval(updateTrajectory, TrajectoryUpdateInterval);
      debug(`created trajectory update interval ${interval}`);

      return () => {
        cancel = true;
        clearInterval(interval);
        debug(`cleared interval ${interval}`);
      };
    }, [trajManager, currentLevel, trajectoryTime, trajectoryAnimScale]);

    // TODO: There is no way to switch to negotiation mode
    // const { current: negotiationTrajStore } = React.useRef<
    //   Record<string, NegotiationTrajectoryResponse>
    // >({});
    // const [mode, setMode] = React.useState<'normal' | 'negotiation'>('normal');
    // const negoTrajectories = React.useMemo<TrajectoryData[]>(() => {
    //   if (mode !== 'negotiation' || !currentLevel) return [];
    //   const negoTrajs = negotiationTrajStore[currentLevel.name];
    //   return negoTrajs
    //     ? negoTrajs.values.map((v) => ({
    //         trajectory: v,
    //         color: 'orange',
    //         animationScale: trajectoryAnimScale,
    //         loopAnimation: false,
    //         conflict: false,
    //       }))
    //     : [];
    // }, [mode, negotiationTrajStore, currentLevel, trajectoryAnimScale]);
    // const renderedTrajectories = React.useMemo(() => {
    //   switch (mode) {
    //     case 'normal':
    //       return trajectories;
    //     case 'negotiation':
    //       return negoTrajectories;
    //   }
    // }, [mode, trajectories, negoTrajectories]);

    React.useEffect(() => {
      if (!rmf) {
        return;
      }

      const subs: Subscription[] = [];
      subs.push(
        rmf.buildingMapObs.subscribe((newMap) => {
          setBuildingMap(newMap);
          const currentLevel = newMap.levels[0];
          setCurrentLevel(currentLevel);
          setWaypoints(
            getPlaces(newMap).filter(
              (p) => p.level === currentLevel.name && p.vertex.name.length > 0,
            ),
          );
        }),
      );
      subs.push(rmf.dispensersObs.subscribe(setDispensers));
      subs.push(rmf.ingestorsObs.subscribe(setIngestors));
      subs.push(rmf.fleetsObs.subscribe(setFleets));

      return () => {
        for (const sub of subs) {
          sub.unsubscribe();
        }
      };
    }, [rmf]);

    const [imageUrl, setImageUrl] = React.useState<string | null>(null);
    const [bounds, setBounds] = React.useState<L.LatLngBoundsLiteral | null>(null);
    React.useEffect(() => {
      if (!currentLevel?.images[0]) {
        setImageUrl(null);
        return;
      }

      (async () => {
        const affineImage = await loadAffineImage(currentLevel.images[0]);
        setBounds(
          affineImageBounds(
            currentLevel.images[0],
            affineImage.naturalWidth,
            affineImage.naturalHeight,
          ),
        );
        setImageUrl(affineImage.src);
      })();

      buildingMap &&
        setWaypoints(
          getPlaces(buildingMap).filter(
            (p) => p.level === currentLevel.name && p.vertex.name.length > 0,
          ),
        );
    }, [buildingMap, currentLevel]);

    const [robots, setRobots] = React.useState<RobotData[]>([]);
    const { current: robotsStore } = React.useRef<Record<string, RobotData>>({});
    React.useEffect(() => {
      (async () => {
        if (!currentLevel) {
          return;
        }
        const promises = Object.values(fleets).flatMap((fleetState) => {
          const robotKey = fleetState.robots && Object.keys(fleetState.robots);
          const fleetName = fleetState.name ? fleetState.name : '';
          return robotKey?.map(async (r) => {
            const robotId = `${fleetState.name}/${r}`;
            if (robotId in robotsStore) return;
            robotsStore[robotId] = {
              fleet: fleetName,
              name: r,
              // no model name
              model: '',
              footprint: 0.5,
              color: await colorManager.robotPrimaryColor(fleetName, r, ''),
              iconPath: (await resourceManager?.robots.getIconPath(fleetName, r)) || undefined,
            };
          });
        });
        await Promise.all(promises);
        const newRobots = Object.values(fleets).flatMap((fleetState) => {
          const robotKey = fleetState.robots ? Object.keys(fleetState.robots) : [];
          return robotKey
            ?.filter(
              (r) =>
                fleetState.robots &&
                fleetState.robots[r].location?.map === currentLevel.name &&
                `${fleetState.name}/${r}` in robotsStore,
            )
            .map((r) => robotsStore[`${fleetState.name}/${r}`]);
        });
        setRobots(newRobots);
      })();
    }, [fleets, robotsStore, resourceManager, currentLevel]);

    const registeredLayersHandlers = React.useRef(false);

    const ready = buildingMap && currentLevel && bounds;

    return ready ? (
      <Canvas>
        <ambientLight />
        <OrbitControls up={[0, 0, 1]} />
        <pointLight position={[10, 10, 10]} />
        <BuidlingLevel level={buildingMap.levels[0]} show={true} />
        {/* {buildingMap.levels.map((level, i)=>(<BuidlingLevel level={level} show={true}/>))} */}
        {/* {buildingMap.lifts.map((lift, i)=>{(null)})}      */}
      </Canvas>
    ) : null;
  }),
)({
  // This ensures that the resize handle is above the map.
  '& > .react-resizable-handle': {
    zIndex: 1001,
  },
});
