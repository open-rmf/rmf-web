import * as RomiCore from '@osrf/romi-js-core-interfaces';
import React from 'react';

export const DoorStateContext = React.createContext<Record<string, RomiCore.DoorState>>({});
export const LiftStateContext = React.createContext<Record<string, RomiCore.LiftState>>({});
export const DispenserStateContext = React.createContext<
  Readonly<Record<string, RomiCore.DispenserState>>
>({});

export interface RmfContextProviderProps extends React.PropsWithChildren<{}> {
  doorStates: Record<string, RomiCore.DoorState>;
  liftStates: Record<string, RomiCore.LiftState>;
  dispenserStates: Record<string, RomiCore.DispenserState>;
}

export function RmfContextProvider(props: RmfContextProviderProps): React.ReactElement {
  const { doorStates, liftStates, dispenserStates, children } = props;
  return (
    <DoorStateContext.Provider value={doorStates}>
      <LiftStateContext.Provider value={liftStates}>
        <DispenserStateContext.Provider value={dispenserStates}>
          {children}
        </DispenserStateContext.Provider>
      </LiftStateContext.Provider>
    </DoorStateContext.Provider>
  );
}
