import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { MessageType, Topic } from 'api-client';
import React from 'react';
import RmfHealthStateManager from '../../managers/rmf-health-state-manager';
import { AppConfigContext, AppControllerContext } from '../app-contexts';
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
  TransportContext,
} from './contexts';

enum ConnectionState {
  Connected,
  Disconnected,
  Connecting,
}

function TransportContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const appConfig = React.useContext(AppConfigContext);
  const [transport, setTransport] = React.useState<RomiCore.Transport | null>(null);
  const { showLoadingScreen } = React.useContext(AppControllerContext);
  const [connectionState, setConnectionState] = React.useState(() =>
    transport ? ConnectionState.Connected : ConnectionState.Disconnected,
  );
  const [error, setError] = React.useState<CloseEvent | null>(null);

  React.useEffect(() => {
    (async () => {
      try {
        setConnectionState(ConnectionState.Connecting);
        setTransport(await appConfig.transportFactory());
        setConnectionState(ConnectionState.Connected);
      } catch (e) {
        console.error(e);
        setError(e);
        setConnectionState(ConnectionState.Disconnected);
      }
    })();
  }, [appConfig]);

  React.useEffect(() => {
    switch (connectionState) {
      case ConnectionState.Connecting:
        showLoadingScreen({ caption: 'Connecting to server...' });
        break;
      case ConnectionState.Disconnected:
        if (error) {
          showLoadingScreen({
            caption: `Unable to connect to server (${error.code})`,
            variant: 'error',
          });
        } else {
          showLoadingScreen({
            caption: `Unable to connect to server (unknown error)`,
            variant: 'error',
          });
        }
        break;
    }
  }, [connectionState, error, showLoadingScreen]);

  return <TransportContext.Provider value={transport}>{props.children}</TransportContext.Provider>;
}

// FIXME: Temp workaround for api breakage in RMF. Eventually we will move away from ros2-bridge
// and use only api-server.
const buildingMapService = {
  ...RomiCore.getBuildingMap,
  type: 'rmf_building_map_msgs/srv/GetBuildingMap',
};

function BuildingMapProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const transport = React.useContext(TransportContext);
  const [buildingMap, setBuildingMap] = React.useState<RomiCore.BuildingMap | null>(null);
  const { showLoadingScreen } = React.useContext(AppControllerContext);
  const downloadingRef = React.useRef(false);

  React.useEffect(() => {
    if (buildingMap || !transport || downloadingRef.current) {
      return;
    }
    (async () => {
      downloadingRef.current = true;
      showLoadingScreen({ caption: 'Downloading building map...' });
      const resp = await transport.call(buildingMapService, {});
      downloadingRef.current = false;
      showLoadingScreen({});
      setBuildingMap(resp.building_map);
    })();
  }, [buildingMap, transport, downloadingRef, showLoadingScreen]);

  return (
    <BuildingMapContext.Provider value={buildingMap}>{props.children}</BuildingMapContext.Provider>
  );
}

function rmfContextProviderHOC<TopicT extends Topic>(
  topic: TopicT,
  Context: React.Context<Record<string, MessageType[TopicT]>>,
  keyMapper: (msg: MessageType[TopicT]) => string,
) {
  return (props: React.PropsWithChildren<{}>) => {
    const { sioClient } = React.useContext(RmfIngressContext);
    const [state, setState] = React.useState<Record<string, MessageType[TopicT]>>({});

    React.useEffect(() => {
      sioClient.emit('subscribe', topic);
      sioClient.on(topic, (msg: MessageType[TopicT]) =>
        setState((prev) => ({ ...prev, [keyMapper(msg)]: msg })),
      );

      // TODO: unsubscribe
    }, [sioClient]);

    return <Context.Provider value={state}>{props.children}</Context.Provider>;
  };
}

const DispenserContextsProvider = rmfContextProviderHOC(
  'dispenser_states',
  DispenserStateContext,
  (msg) => msg.guid,
);

const DoorContextsProvider = rmfContextProviderHOC(
  'door_states',
  DoorStateContext,
  (msg) => msg.door_name,
);

const LiftContextsProvider = rmfContextProviderHOC(
  'lift_states',
  LiftStateContext,
  (msg) => msg.lift_name,
);

const FleetContextsProvider = rmfContextProviderHOC(
  'fleet_states',
  FleetStateContext,
  (msg) => msg.name,
);

function NegotiationContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const { negotiationStatusManager } = React.useContext(RmfIngressContext);
  const [negotiationStatus, setNegotiationStatus] = React.useState(
    negotiationStatusManager.allConflicts(),
  );

  React.useEffect(() => {
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

function TaskContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const transport = React.useContext(TransportContext);
  const { taskManager } = React.useContext(RmfIngressContext);
  const [tasks, setTasks] = React.useState(() => [...Object.values(taskManager.tasks)]);

  React.useEffect(() => {
    if (transport) {
      taskManager.startSubscription(transport);
    }
    const onUpdated = () => setTasks([...Object.values(taskManager.tasks)]);
    taskManager.on('updated', onUpdated);

    return () => {
      taskManager.off('updated', onUpdated);
    };
  }, [transport, taskManager]);

  return <TasksContext.Provider value={tasks}>{props.children}</TasksContext.Provider>;
}

function RmfHealthContextsProvider(props: React.PropsWithChildren<{}>): JSX.Element {
  const healthManager = React.useMemo(() => new RmfHealthStateManager(), []);

  return (
    <RmfHealthContext.Provider value={healthManager.getHealthStatus()}>
      {props.children}
    </RmfHealthContext.Provider>
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
    <TransportContextsProvider>
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
    </TransportContextsProvider>
  );
}
