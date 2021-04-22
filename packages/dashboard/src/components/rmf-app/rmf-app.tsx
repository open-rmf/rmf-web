import { MessageType, Topic } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import appConfig from '../../app-config';
import RmfHealthStateManager from '../../managers/rmf-health-state-manager';
import {
  DefaultTrajectoryManager,
  RobotTrajectoryManager,
} from '../../managers/robot-trajectory-manager';
import { UserContext } from '../auth/contexts';
import {
  BuildingMapContext,
  DispenserStateContext,
  DoorStateContext,
  FleetStateContext,
  LiftStateContext,
  NegotiationStatusContext,
  RmfHealthContext,
  RmfIngressContext,
  TasksContext,
} from './contexts';
import { RmfIngress } from './rmf-ingress';

function rmfStateContextProviderHOC<TopicT extends Topic>(
  topic: TopicT,
  Context: React.Context<Record<string, MessageType[TopicT]>>,
  keyMapper: (msg: MessageType[TopicT]) => string,
) {
  return (props: React.PropsWithChildren<{}>) => {
    const { sioClient } = React.useContext(RmfIngressContext) || {};
    const [state, setState] = React.useState<Record<string, MessageType[TopicT]>>({});

    React.useEffect(() => {
      if (!sioClient) {
        return;
      }
      sioClient.emit('subscribe', topic);
      sioClient.on(topic, (msg: MessageType[TopicT]) =>
        setState((prev) => ({ ...prev, [keyMapper(msg)]: msg })),
      );

      // TODO: unsubscribe
    }, [sioClient]);

    return <Context.Provider value={state}>{props.children}</Context.Provider>;
  };
}

function BuildingMapProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { sioClient } = React.useContext(RmfIngressContext) || {};
  const [buildingMap, setBuildingMap] = React.useState<RmfModels.BuildingMap | null>(null);

  React.useEffect(() => {
    if (!sioClient) {
      return;
    }
    sioClient.emit('subscribe', 'building_map');
    sioClient.on('building_map', setBuildingMap);

    // TODO: unsubscribe
  }, [sioClient]);

  // TODO:
  // const { showLoadingScreen } = React.useContext(AppControllerContext);
  // const downloadingRef = React.useRef(false);
  // React.useEffect(() => {
  //   if (buildingMap || downloadingRef.current) {
  //     return;
  //   }
  //   (async () => {
  //     downloadingRef.current = true;
  //     showLoadingScreen({ caption: 'Downloading building map...' });
  //     const resp = await transport.call(buildingMapService, {});
  //     downloadingRef.current = false;
  //     showLoadingScreen({});
  //     setBuildingMap(resp.building_map);
  //   })();
  // }, [buildingMap, transport, downloadingRef, showLoadingScreen]);

  return (
    <BuildingMapContext.Provider value={buildingMap}>{props.children}</BuildingMapContext.Provider>
  );
}

const DispenserContextsProvider = rmfStateContextProviderHOC(
  'dispenser_states',
  DispenserStateContext,
  (msg) => msg.guid,
);

const DoorContextsProvider = rmfStateContextProviderHOC(
  'door_states',
  DoorStateContext,
  (msg) => msg.door_name,
);

const LiftContextsProvider = rmfStateContextProviderHOC(
  'lift_states',
  LiftStateContext,
  (msg) => msg.lift_name,
);

const FleetContextsProvider = rmfStateContextProviderHOC(
  'fleet_states',
  FleetStateContext,
  (msg) => msg.name,
);

const TaskContextsProvider = rmfStateContextProviderHOC(
  'task_summaries',
  TasksContext,
  (msg) => msg.task_id,
);

function NegotiationContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { negotiationStatusManager } = React.useContext(RmfIngressContext) || {};
  const [negotiationStatus, setNegotiationStatus] = React.useState(
    negotiationStatusManager?.allConflicts() || {},
  );

  React.useEffect(() => {
    if (!negotiationStatusManager) {
      return;
    }
    negotiationStatusManager.startSubscription();
    const onUpdated = () => setNegotiationStatus(negotiationStatusManager.allConflicts());
    negotiationStatusManager.on('updated', onUpdated);

    return () => {
      negotiationStatusManager.off('updated', onUpdated);
    };
  }, [negotiationStatusManager]);

  return (
    <NegotiationStatusContext.Provider value={negotiationStatus}>
      {props.children}
    </NegotiationStatusContext.Provider>
  );
}

function RmfHealthContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const healthManager = React.useMemo(() => new RmfHealthStateManager(), []);

  return (
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      {props.children}
    </RmfHealthContext.Provider>
  );
}

function RmfIngressProvider(props: React.PropsWithChildren<{}>) {
  const user = React.useContext(UserContext);
  const [trajMgr, setTrajMgr] = React.useState<RobotTrajectoryManager | undefined>(undefined);
  React.useEffect(() => {
    (async () => {
      setTrajMgr(await DefaultTrajectoryManager.create(appConfig.trajServerUrl));
    })();
  }, []);

  const rmfIngress = React.useMemo(() => new RmfIngress(user, trajMgr), [user, trajMgr]);
  return (
    <RmfIngressContext.Provider value={rmfIngress}>{props.children}</RmfIngressContext.Provider>
  );
}

export interface RmfAppProps extends React.PropsWithChildren<{}> {}

/**
 * Provides the following contexts:
 *
 * - BuildingMapContext
 * - DispenserStateContext
 * - DoorStateContext
 * - FleetStateContext
 * - LiftStateContext
 * - NegotiationStatusContext
 * - RmfIngressContext
 * - TasksContext
 * - TransportContext
 */
export function RmfApp(props: RmfAppProps): JSX.Element {
  return (
    <RmfIngressProvider>
      <BuildingMapProvider>
        <DoorContextsProvider>
          <LiftContextsProvider>
            <DispenserContextsProvider>
              <FleetContextsProvider>
                <NegotiationContextsProvider>
                  <RmfHealthContextsProvider>
                    <TaskContextsProvider>{props.children}</TaskContextsProvider>
                  </RmfHealthContextsProvider>
                </NegotiationContextsProvider>
              </FleetContextsProvider>
            </DispenserContextsProvider>
          </LiftContextsProvider>
        </DoorContextsProvider>
      </BuildingMapProvider>
    </RmfIngressProvider>
  );
}
