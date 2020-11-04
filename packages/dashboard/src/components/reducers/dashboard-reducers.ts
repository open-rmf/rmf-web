import * as RomiCore from '@osrf/romi-js-core-interfaces';

type ActionFormat<T, K> = {
  type: T;
  payload: K;
};

export enum DashboardActionType {
  DOOR_STATE = 'doorStates',
  DOORS = 'doors',
  LIFTS = 'lifts',
  LIFT_STATE = 'liftStates',
}

type Action =
  | ActionFormat<'doorStates', Record<string, RomiCore.DoorState>>
  | ActionFormat<'doors', RomiCore.Door[]>
  | ActionFormat<'liftStates', Record<string, RomiCore.LiftState>>
  | ActionFormat<'lifts', RomiCore.Lift[]>;

export type DashboardState = {
  [DashboardActionType.DOOR_STATE]: Record<string, RomiCore.DoorState>;
  [DashboardActionType.DOORS]: RomiCore.Door[];
  [DashboardActionType.LIFT_STATE]: Record<string, RomiCore.LiftState>;
  [DashboardActionType.LIFTS]: RomiCore.Lift[];
};

export const dashboardReducer = (state: DashboardState, action: Action): DashboardState => {
  switch (action.type) {
    case DashboardActionType.DOOR_STATE:
      return { ...state, [DashboardActionType.DOOR_STATE]: action.payload };
    case DashboardActionType.DOORS:
      return { ...state, [DashboardActionType.DOORS]: action.payload };
    case DashboardActionType.LIFT_STATE:
      return { ...state, [DashboardActionType.LIFT_STATE]: action.payload };
    case DashboardActionType.LIFTS:
      return { ...state, [DashboardActionType.LIFTS]: action.payload };

    default:
      throw new Error('Unexpected action');
  }
};
