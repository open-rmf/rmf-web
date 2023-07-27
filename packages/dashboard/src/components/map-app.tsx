import { styled } from '@mui/material';
import {
  BuildingMap,
  Dispenser,
  ApiServerModelsRmfApiFleetStateFleetState,
  Ingestor,
  Level,
} from 'api-client';
import Debug from 'debug';
import React, { Suspense } from 'react';
import {
  affineImageBounds,
  ColorManager,
  fromRmfCoords,
  getPlaces,
  LMap,
  loadAffineImage,
  Place,
  RobotTableData,
  TrajectoryTimeControl,
} from 'react-components';
import { AttributionControl, ImageOverlay, LayersControl, Pane, Viewport } from 'react-leaflet';
import { EMPTY, merge, scan, Subscription, switchMap } from 'rxjs';
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
import { RobotSummary } from './robots/robot-summary';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import * as THREE from 'three';
import {
  PerspectiveCamera,
  PositionalAudio,
  OrbitControls,
  Environment,
  Stats,
  Stage,
} from '@react-three/drei';
import { BuildingCubes, findSceneBoundingBox } from './level';
import { RobotShape } from './robot-loader';

type FleetState = ApiServerModelsRmfApiFleetStateFleetState;

const debug = Debug('MapApp');

const TrajectoryUpdateInterval = 2000;
// schedule visualizer manages it's own settings so that it doesn't cause a re-render
// of the whole app when it changes.
const SettingsKey = 'mapAppSettings';
const colorManager = new ColorManager();

function getRobotId(fleetName: string, robotName: string): string {
  return `${fleetName}/${robotName}`;
}

interface MapSettings {
  trajectoryTime: number;
}

export const MapApp = styled(
  createMicroApp('Map', () => {
    const rmf = React.useContext(RmfAppContext);
    const resourceManager = React.useContext(ResourcesContext);
    const [currentLevel, setCurrentLevel] = React.useState<Level | undefined>(undefined);
    const [disabledLayers, setDisabledLayers] = React.useState<Record<string, boolean>>({});
    const [openRobotSummary, setOpenRobotSummary] = React.useState(false);
    const [selectedRobot, setSelectedRobot] = React.useState<RobotTableData>();

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
    const [center, setCenter] = React.useState<L.LatLngTuple>([0, 0]);
    const [zoom, setZoom] = React.useState<number>(5);

    React.useEffect(() => {
      const sub = AppEvents.zoom.subscribe((currentValue) => {
        setZoom(currentValue);
      });
      return () => sub.unsubscribe();
    }, []);

    React.useEffect(() => {
      const sub = AppEvents.mapCenter.subscribe((currentValue) => {
        setCenter((prev) => {
          const newCenter: L.LatLngTuple = [...currentValue];
          // react-leaftlet does not properly update state when the previous LatLng is the same,
          // even when a new array is passed.
          if (prev[0] === newCenter[0]) {
            newCenter[0] += 0.00001;
          }
          if (prev[1] === newCenter[1]) {
            newCenter[1] += 0.00001;
          }
          return newCenter;
        });
      });
      return () => sub.unsubscribe();
    }, []);

    React.useEffect(() => {
      if (!currentLevel?.images[0]) {
        setImageUrl(null);
        return;
      }

      (async () => {
        const affineImage = await loadAffineImage(currentLevel.images[0]);
        const bounds = affineImageBounds(
          currentLevel.images[0],
          affineImage.naturalWidth,
          affineImage.naturalHeight,
        );
        setBounds(bounds);
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
            const robotId = getRobotId(fleetName, r);
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

    const { current: robotLocations } = React.useRef<Record<string, [number, number]>>({});
    // updates the robot location
    React.useEffect(() => {
      if (!rmf) {
        return;
      }
      const sub = rmf.fleetsObs
        .pipe(
          switchMap((fleets) =>
            merge(...fleets.map((f) => (f.name ? rmf.getFleetStateObs(f.name) : EMPTY))),
          ),
        )
        .subscribe((fleetState) => {
          const fleetName = fleetState.name;
          if (!fleetName || !fleetState.robots) {
            console.warn('Map: Fail to update robot location (missing fleet name or robots)');
            return;
          }
          Object.entries(fleetState.robots).forEach(([robotName, robotState]) => {
            const robotId = getRobotId(fleetName, robotName);
            if (!robotState.location) {
              console.warn(`Map: Fail to update robot location for ${robotId} (missing location)`);
              return;
            }
            robotLocations[robotId] = [robotState.location.x, robotState.location.y];
          });
        });
      return () => sub.unsubscribe();
    }, [rmf, robotLocations]);

    //Accumulate values over time to persist between tabs
    React.useEffect(() => {
      const sub = AppEvents.disabledLayers
        .pipe(scan((acc, value) => ({ ...acc, ...value }), {}))
        .subscribe((layers) => {
          setDisabledLayers(layers);
        });
      return () => sub.unsubscribe();
    }, []);

    // zoom to robot on select
    React.useEffect(() => {
      const sub = AppEvents.robotSelect.subscribe((data) => {
        if (!data) {
          return;
        }
        const [fleetName, robotName] = data;
        const robotId = getRobotId(fleetName, robotName);
        const robotLocation = robotLocations[robotId];
        if (!robotLocation) {
          console.warn(`Map: Failed to zoom to robot ${robotId} (robot location was not found)`);
          return;
        }
        if (!bounds) {
          console.warn(
            `Map: Fail to zoom to robot ${robotId} (missing bounds, map was not loaded?)`,
          );
          return;
        }
        const mapCoords = fromRmfCoords(robotLocation);
        const newCenter: L.LatLngTuple = [mapCoords[1], mapCoords[0]];
        AppEvents.mapCenter.next(newCenter);
        AppEvents.zoom.next(6);
      });
      return () => sub.unsubscribe();
    }, [robotLocations, bounds]);

    const onViewportChanged = (viewport: Viewport) => {
      if (viewport.zoom && viewport.center) {
        AppEvents.zoom.next(viewport.zoom);
        AppEvents.mapCenter.next(viewport.center);
      }
    };

    const registeredLayersHandlers = React.useRef(false);
    const ready = buildingMap && currentLevel && bounds;

    const ref = React.useRef(null);
    const [lerping, setLerping] = React.useState(false);
    const [to, setTo] = React.useState();
    const [target, setTarget] = React.useState();
    const [selected, setSelected] = React.useState(-1);
    const [sceneBoundingBox, setSceneBoundingBox] = React.useState<THREE.Box3 | undefined>(
      undefined,
    );
    const [distance, setDistance] = React.useState<number>(0);

    React.useMemo(() => setSceneBoundingBox(findSceneBoundingBox(currentLevel)), [currentLevel]);

    React.useEffect(() => {
      if (!sceneBoundingBox) {
        return;
      }

      const size = sceneBoundingBox.getSize(new THREE.Vector3());
      setDistance(Math.max(size.x, size.y, size.z) * 0.9);
    }, [sceneBoundingBox]);

    // const [overallSceneBoundingBox, setOverallSceneBoundingBox] = React.useState<
    //   THREE.Box3 | undefined
    // >(undefined);

    // React.useMemo(() => {
    //   const combinedBoundingBox = new THREE.Box3();
    //   buildingMap?.levels.forEach((level) => {
    //     const levelBoundingBox = findSceneBoundingBox(level);
    //     combinedBoundingBox.expandByPoint(levelBoundingBox.min);
    //     combinedBoundingBox.expandByPoint(levelBoundingBox.max);
    //   });
    //   setOverallSceneBoundingBox(combinedBoundingBox);
    // }, [buildingMap?.levels]);

    // React.useEffect(() => {
    //   if (!overallSceneBoundingBox) {
    //     return;
    //   }

    //   const size = overallSceneBoundingBox.getSize(new THREE.Vector3());
    //   setDistance(Math.max(size.x, size.y, size.z) * 0.9);
    // }, [overallSceneBoundingBox]);

    return ready ? (
      // <LMap
      //   ref={(cur) => {
      //     if (registeredLayersHandlers.current || !cur) return;
      //     cur.leafletElement.on('overlayadd', (ev: L.LayersControlEvent) => {
      //       AppEvents.disabledLayers.next({ [ev.name]: false });
      //     });
      //     cur.leafletElement.on('overlayremove', (ev: L.LayersControlEvent) => {
      //       AppEvents.disabledLayers.next({ [ev.name]: true });
      //     });
      //     registeredLayersHandlers.current = true;
      //   }}
      //   attributionControl={false}
      //   zoomDelta={0.5}
      //   zoomSnap={0.5}
      //   center={center}
      //   onViewportChanged={onViewportChanged}
      //   zoom={zoom}
      //   bounds={bounds}
      //   maxBounds={bounds}
      //   onbaselayerchange={({ name }: L.LayersControlEvent) => {
      //     setCurrentLevel(
      //       buildingMap.levels.find((l: Level) => l.name === name) || buildingMap.levels[0],
      //     );
      //   }}
      // >
      //   <AttributionControl position="bottomright" prefix="OSRC-SG" />
      //   <LayersControl position="topleft">
      //     <Pane name="image" style={{ zIndex: 0 }} />
      //     {buildingMap.levels.map((level: Level) =>
      //       currentLevel.name === level.name ? (
      //         <LayersControl.BaseLayer key={level.name} name={level.name} checked>
      //           {currentLevel.images.length > 0 && imageUrl && (
      //             <ImageOverlay bounds={bounds} url={imageUrl} pane="image" />
      //           )}
      //         </LayersControl.BaseLayer>
      //       ) : (
      //         <LayersControl.BaseLayer key={level.name} name={level.name}>
      //           {currentLevel.images.length > 0 && imageUrl && (
      //             <ImageOverlay bounds={bounds} url={imageUrl} pane="image" />
      //           )}
      //         </LayersControl.BaseLayer>
      //       ),
      //     )}

      //     <LayersControl.Overlay name="Waypoints" checked={!disabledLayers['Waypoints']}>
      //       <WaypointsOverlay
      //         bounds={bounds}
      //         waypoints={waypoints}
      //         hideLabels={disabledLayers['Waypoints']}
      //       />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Dispensers" checked={!disabledLayers['Dispensers']}>
      //       <WorkcellsOverlay
      //         bounds={bounds}
      //         workcells={dispensersData}
      //         hideLabels={disabledLayers['Dispensers']}
      //         onWorkcellClick={(_ev, workcell) =>
      //           AppEvents.dispenserSelect.next({ guid: workcell })
      //         }
      //       />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Ingestors" checked={!disabledLayers['Ingestors']}>
      //       <WorkcellsOverlay
      //         bounds={bounds}
      //         workcells={ingestorsData}
      //         hideLabels={disabledLayers['Ingestors']}
      //         onWorkcellClick={(_ev, workcell) => AppEvents.ingestorSelect.next({ guid: workcell })}
      //       />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Lifts" checked={!disabledLayers['Lifts']}>
      //       <LiftsOverlay
      //         bounds={bounds}
      //         currentLevel={currentLevel.name}
      //         lifts={buildingMap.lifts}
      //         hideLabels={disabledLayers['Lifts']}
      //         onLiftClick={(_ev, lift) => AppEvents.liftSelect.next(lift)}
      //       />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Doors" checked={!disabledLayers['Doors']}>
      //       <DoorsOverlay
      //         bounds={bounds}
      //         doors={currentLevel.doors}
      //         hideLabels={disabledLayers['Doors']}
      //         onDoorClick={(_ev, door) => AppEvents.doorSelect.next(door)}
      //       />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Trajectories" checked>
      //       <TrajectoriesOverlay bounds={bounds} trajectoriesData={trajectories} />
      //     </LayersControl.Overlay>

      //     <LayersControl.Overlay name="Robots" checked={!disabledLayers['Robots']}>
      //       <RobotsOverlay
      //         bounds={bounds}
      //         robots={robots}
      //         hideLabels={disabledLayers['Robots']}
      //         onRobotClick={(_ev, robot) => {
      //           setOpenRobotSummary(true);
      //           setSelectedRobot(robot);
      //         }}
      //       />
      //     </LayersControl.Overlay>
      //   </LayersControl>
      //   {openRobotSummary && selectedRobot && (
      //     <RobotSummary robot={selectedRobot} onClose={() => setOpenRobotSummary(false)} />
      //   )}

      //   <TrajectoryTimeControl
      //     position="topleft"
      //     value={trajectoryTime}
      //     min={60000}
      //     max={600000}
      //     onChange={(_ev, newValue) =>
      //       setMapSettings((prev) => {
      //         const newSettings = { ...prev, trajectoryTime: newValue };
      //         window.localStorage.setItem(SettingsKey, JSON.stringify(newSettings));
      //         return newSettings;
      //       })
      //     }
      //   />
      // </LMap>
      // <MapConstruction levels={buildingMap.levels} robots={robots} robotLocations={robotLocations} />

      <Suspense fallback={null}>
        <Canvas
          onPointerDown={() => setLerping(false)}
          onWheel={() => setLerping(false)}
          onCreated={({ camera }) => {
            if (!sceneBoundingBox) {
              return;
            }
            const center = sceneBoundingBox.getCenter(new THREE.Vector3());
            camera.position.set(center.x, center.y, center.z + distance);
            camera.updateProjectionMatrix();
          }}
        >
          {/* <OrbitControls
          ref={ref}
          enableZoom
          enablePan
          enableRotate
          target={sceneBoundingBox?.getCenter(new THREE.Vector3())}
          maxDistance={distance}
        /> */}
          <OrbitControls
            target={sceneBoundingBox?.getCenter(new THREE.Vector3())}
            ref={ref}
            enableZoom
            enablePan
            enableDamping
            dampingFactor={0.1}
          />
          <BuildingCubes level={currentLevel} />
          <RobotShape robots={robots} robotLocations={robotLocations} />
          <ambientLight />
        </Canvas>
        <div id="annotationsPanel">
          <ul>
            {buildingMap.levels.map((level, i) => {
              const { name } = level;
              return (
                <li key={i}>
                  <button
                    className="annotationButton"
                    name={name}
                    onClick={(event: React.MouseEvent<HTMLButtonElement>) => {
                      setCurrentLevel(
                        buildingMap.levels.find((l) => l.name === event.currentTarget?.name) ||
                          currentLevel,
                      );
                    }}
                  >
                    {name}
                  </button>
                </li>
              );
            })}
          </ul>
        </div>
      </Suspense>
    ) : null;
  }),
)();
