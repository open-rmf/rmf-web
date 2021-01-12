import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';
import appConfig from '../app-config';
import DispenserStateManager from '../dispenser-state-manager';
import DoorStateManager from '../door-state-manager';
import FleetManager from '../fleet-manager';
import LiftStateManager from '../lift-state-manager';
import TaskManager from '../managers/task-manager';
import { NegotiationConflict, NegotiationStatusManager } from '../negotiation-status-manager';
import { LoadingScreenContext } from './loading-screen';

export const DoorStateContext = React.createContext<Record<string, RomiCore.DoorState>>({});
export const LiftStateContext = React.createContext<Record<string, RomiCore.LiftState>>({});
export const DispenserStateContext = React.createContext<Record<string, RomiCore.DispenserState>>(
  {},
);
export const FleetStateContext = React.createContext<Record<string, RomiCore.FleetState>>({});
export const NegotiationStatusContext = React.createContext<Record<number, NegotiationConflict>>(
  {},
);
export const TasksContext = React.createContext<RomiCore.TaskSummary[]>([]);

/**
 * A bunch of dependencies that components really shouldn't depend on but is needed by some less
 * well designed components.
 */
export interface DepHacks {
  transport: RomiCore.Transport;
  negotiationStatusManager: NegotiationStatusManager;
}

/**
 * Value of this MUST be provided.
 */
export const DepHacksContext = React.createContext<DepHacks>({} as DepHacks);

export interface RmfContextProviderProps extends React.PropsWithChildren<{}> {}

/**
 * Provides and manages state updates from rmf.
 */
export function RmfContextProvider(props: RmfContextProviderProps): React.ReactElement {
  const { children } = props;

  const setLoadingScreen = React.useContext(LoadingScreenContext);

  const doorStateManager = React.useMemo(() => new DoorStateManager(), []);
  const [doorStates, setDoorStates] = React.useState(() => doorStateManager.doorStates());

  const liftStateManager = React.useMemo(() => new LiftStateManager(), []);
  const [liftStates, setLiftStates] = React.useState(() => liftStateManager.liftStates());

  const dispenserStateManager = React.useMemo(() => new DispenserStateManager(), []);
  const [dispenserStates, setDispenserStates] = React.useState(() =>
    dispenserStateManager.dispenserStates(),
  );

  const fleetManager = React.useMemo(() => new FleetManager(), []);
  const [fleetStates, setFleetStates] = React.useState(fleetManager.fleets());

  const negotiationStatusManager = React.useMemo(
    () => new NegotiationStatusManager(appConfig.trajServerUrl),
    [],
  );
  const [negotiationStatus, setNegotiationStatus] = React.useState(
    negotiationStatusManager.allConflicts(),
  );

  const taskManager = React.useMemo(() => new TaskManager(), []);
  const [tasks, setTasks] = React.useState(taskManager.tasks());

  const transportRef = React.useRef<RomiCore.Transport>();

  React.useEffect(() => {
    (async () => {
      try {
        setLoadingScreen({ caption: 'Connecting to server...' });
        const transport = await appConfig.transportFactory();
        doorStateManager.startSubscription(transport);
        doorStateManager.on('updated', () => setDoorStates(doorStateManager.doorStates()));
        liftStateManager.startSubscription(transport);
        liftStateManager.on('updated', () => setLiftStates(liftStateManager.liftStates()));
        dispenserStateManager.startSubscription(transport);
        dispenserStateManager.on('updated', () =>
          setDispenserStates(dispenserStateManager.dispenserStates()),
        );
        fleetManager.startSubscription(transport);
        fleetManager.on('updated', () => setFleetStates(fleetManager.fleets()));
        negotiationStatusManager.startSubscription();
        negotiationStatusManager.on('updated', () =>
          setNegotiationStatus(negotiationStatusManager.allConflicts()),
        );
        taskManager.startSubscription(transport);
        taskManager.on('updated', () => setTasks(taskManager.tasks()));
        transportRef.current = transport;
      } catch (e) {
        setLoadingScreen({ caption: `Unable to connect to server (${e.code})`, variant: 'error' });
      }
    })();
  });

  return (
    <DoorStateContext.Provider value={doorStates}>
      <LiftStateContext.Provider value={liftStates}>
        <DispenserStateContext.Provider value={dispenserStates}>
          <FleetStateContext.Provider value={fleetStates}>
            <NegotiationStatusContext.Provider value={negotiationStatus}>
              <TasksContext.Provider value={tasks}>
                <DepHacksContext.Provider
                  value={{
                    transport: transportRef.current!,
                    negotiationStatusManager,
                  }}
                >
                  {children}
                </DepHacksContext.Provider>
              </TasksContext.Provider>
            </NegotiationStatusContext.Provider>
          </FleetStateContext.Provider>
        </DispenserStateContext.Provider>
      </LiftStateContext.Provider>
    </DoorStateContext.Provider>
  );
}
