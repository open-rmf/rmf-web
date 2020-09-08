import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export const DoorStateContext = React.createContext<Record<string, RomiCore.DoorState>>({});
export const LiftStateContext = React.createContext<Record<string, RomiCore.LiftState>>({});

export interface RmfContextProviderProps extends React.PropsWithChildren<{}> {
  doorStates: Record<string, RomiCore.DoorState>;
  liftStates: Record<string, RomiCore.LiftState>;
}

export function RmfContextProvider(props: RmfContextProviderProps): React.ReactElement {
  const { doorStates, liftStates, children } = props;
  return (
    <DoorStateContext.Provider value={doorStates}>
      <LiftStateContext.Provider value={liftStates}>{children}</LiftStateContext.Provider>
    </DoorStateContext.Provider>
  );
}
