import React from 'react';

type ReporterActionFormat<T, K = void> = K extends void
  ? {
      type: T;
    }
  : {
      type: T;
      payload: K;
    };

export enum ReporterActionType {
  queryAllLogs = 'queryAllLogs',
  showChargerStateReport = 'showChargerStateReport',
  showDoorStateReport = 'showDoorStateReport',
  showLiftStateReport = 'showLiftStateReport',
  showRobotStateReport = 'showRobotStateReport',
  showRobotMotionPlansReport = 'showRobotMotionPlansReport',
  showRobotActionReport = 'showRobotActionReport',
  showTasksReport = 'showTasksReport',
  showUserActionsReport = 'showUserActionsReport',
  showLoginsReport = 'showLoginsReport',
  showLogoutsReport = 'showLogoutsReport',
  showLoginFailuresReport = 'showLoginFailuresReport',
  showWorkCellStatesReport = 'showWorkCellStatesReport',
}

export type ReporterState = {
  [ReporterActionType.queryAllLogs]: boolean;
  [ReporterActionType.showChargerStateReport]: boolean;
  [ReporterActionType.showDoorStateReport]: boolean;
  [ReporterActionType.showLiftStateReport]: boolean;
  [ReporterActionType.showRobotStateReport]: boolean;
  [ReporterActionType.showRobotMotionPlansReport]: boolean;
  [ReporterActionType.showRobotActionReport]: boolean;
  [ReporterActionType.showTasksReport]: boolean;
  [ReporterActionType.showUserActionsReport]: boolean;
  [ReporterActionType.showLoginsReport]: boolean;
  [ReporterActionType.showLogoutsReport]: boolean;
  [ReporterActionType.showLoginFailuresReport]: boolean;
  [ReporterActionType.showWorkCellStatesReport]: boolean;
};

export type ReporterAction =
  | ReporterActionFormat<'queryAllLogs', ReporterState['queryAllLogs']>
  | ReporterActionFormat<'showChargerStateReport', ReporterState['showChargerStateReport']>
  | ReporterActionFormat<'showDoorStateReport', ReporterState['showDoorStateReport']>
  | ReporterActionFormat<'showLiftStateReport', ReporterState['showLiftStateReport']>
  | ReporterActionFormat<'showRobotStateReport', ReporterState['showRobotStateReport']>
  | ReporterActionFormat<'showRobotMotionPlansReport', ReporterState['showRobotMotionPlansReport']>
  | ReporterActionFormat<'showRobotActionReport', ReporterState['showRobotActionReport']>
  | ReporterActionFormat<'showTasksReport', ReporterState['showTasksReport']>
  | ReporterActionFormat<'showUserActionsReport', ReporterState['showUserActionsReport']>
  | ReporterActionFormat<'showLoginsReport', ReporterState['showLoginsReport']>
  | ReporterActionFormat<'showLogoutsReport', ReporterState['showLogoutsReport']>
  | ReporterActionFormat<'showLoginFailuresReport', ReporterState['showLoginFailuresReport']>
  | ReporterActionFormat<'showWorkCellStatesReport', ReporterState['showWorkCellStatesReport']>;

export const reporterReducer = (state: ReporterState, action: ReporterAction): ReporterState => {
  const cleanReportStates = () => {
    return {
      [ReporterActionType.queryAllLogs]: false,
      [ReporterActionType.showChargerStateReport]: false,
      [ReporterActionType.showDoorStateReport]: false,
      [ReporterActionType.showLiftStateReport]: false,
      [ReporterActionType.showRobotStateReport]: false,
      [ReporterActionType.showRobotMotionPlansReport]: false,
      [ReporterActionType.showRobotActionReport]: false,
      [ReporterActionType.showTasksReport]: false,
      [ReporterActionType.showUserActionsReport]: false,
      [ReporterActionType.showLoginsReport]: false,
      [ReporterActionType.showLogoutsReport]: false,
      [ReporterActionType.showLoginFailuresReport]: false,
      [ReporterActionType.showWorkCellStatesReport]: false,
    };
  };

  switch (action.type) {
    case ReporterActionType.queryAllLogs:
      return { ...cleanReportStates(), [ReporterActionType.queryAllLogs]: action.payload };
    case ReporterActionType.showChargerStateReport:
      return {
        ...cleanReportStates(),
        [ReporterActionType.showChargerStateReport]: action.payload,
      };
    case ReporterActionType.showDoorStateReport:
      return { ...cleanReportStates(), [ReporterActionType.showDoorStateReport]: action.payload };
    case ReporterActionType.showLiftStateReport:
      return { ...cleanReportStates(), [ReporterActionType.showLiftStateReport]: action.payload };

    case ReporterActionType.showRobotStateReport:
      return { ...cleanReportStates(), [ReporterActionType.showRobotStateReport]: action.payload };

    case ReporterActionType.showRobotMotionPlansReport:
      return {
        ...cleanReportStates(),
        [ReporterActionType.showRobotMotionPlansReport]: action.payload,
      };
    case ReporterActionType.showRobotActionReport:
      return { ...cleanReportStates(), [ReporterActionType.showRobotActionReport]: action.payload };

    case ReporterActionType.showTasksReport:
      return { ...cleanReportStates(), [ReporterActionType.showTasksReport]: action.payload };

    case ReporterActionType.showUserActionsReport:
      return { ...cleanReportStates(), [ReporterActionType.showUserActionsReport]: action.payload };

    case ReporterActionType.showLoginsReport:
      return { ...cleanReportStates(), [ReporterActionType.showLoginsReport]: action.payload };

    case ReporterActionType.showLogoutsReport:
      return { ...cleanReportStates(), [ReporterActionType.showLogoutsReport]: action.payload };

    case ReporterActionType.showLoginFailuresReport:
      return {
        ...cleanReportStates(),
        [ReporterActionType.showLoginFailuresReport]: action.payload,
      };
    case ReporterActionType.showWorkCellStatesReport:
      return {
        ...cleanReportStates(),
        [ReporterActionType.showWorkCellStatesReport]: action.payload,
      };

    default:
      console.error('Unexpected action');
      return state;
  }
};

export interface ReducerReporterDispatch {
  setQueryAllLogs: (payload: boolean) => void;
  setShowChargerStateReport: (payload: boolean) => void;
  setShowDoorStateReport: (payload: boolean) => void;

  setShowLiftStateReport: (payload: boolean) => void;
  setShowRobotStateReport: (payload: boolean) => void;
  setShowRobotMotionPlansReport: (payload: boolean) => void;
  setShowRobotActionReport: (payload: boolean) => void;
  setShowTasksReport: (payload: boolean) => void;
  setShowUserActionsReport: (payload: boolean) => void;
  setShowLoginsReport: (payload: boolean) => void;
  setShowLogoutsReport: (payload: boolean) => void;
  setShowLoginFailuresReport: (payload: boolean) => void;
  setShowWorkCellStatesReport: (payload: boolean) => void;
}

export interface ReducerReporterProps {
  state: ReporterState;
  dispatch: ReducerReporterDispatch;
}

export const useReporterReducer = (initialValue: ReporterState): ReducerReporterProps => {
  const [_state, _dispatch] = React.useReducer(reporterReducer, initialValue);
  // We add a useMemo here because React identifies that the state and dispatch props are always
  // changing, which causes a huge performance issue.
  const state = React.useMemo(() => _state, [_state]);
  const dispatch: ReducerReporterDispatch = React.useMemo(() => {
    return {
      setQueryAllLogs: (payload) =>
        _dispatch({ type: ReporterActionType.queryAllLogs, payload: payload }),
      setShowChargerStateReport: (payload) =>
        _dispatch({ type: ReporterActionType.showChargerStateReport, payload: payload }),
      setShowDoorStateReport: (payload) =>
        _dispatch({ type: ReporterActionType.showDoorStateReport, payload: payload }),
      setShowLiftStateReport: (payload) =>
        _dispatch({ type: ReporterActionType.showLiftStateReport, payload: payload }),
      setShowRobotStateReport: (payload) =>
        _dispatch({ type: ReporterActionType.showRobotStateReport, payload: payload }),
      setShowRobotMotionPlansReport: (payload) =>
        _dispatch({ type: ReporterActionType.showRobotMotionPlansReport, payload: payload }),
      setShowRobotActionReport: (payload) =>
        _dispatch({ type: ReporterActionType.showRobotActionReport, payload: payload }),
      setShowTasksReport: (payload) =>
        _dispatch({ type: ReporterActionType.showTasksReport, payload: payload }),
      setShowUserActionsReport: (payload) =>
        _dispatch({ type: ReporterActionType.showUserActionsReport, payload: payload }),
      setShowLoginsReport: (payload) =>
        _dispatch({ type: ReporterActionType.showLoginsReport, payload: payload }),
      setShowLogoutsReport: (payload) =>
        _dispatch({ type: ReporterActionType.showLogoutsReport, payload: payload }),
      setShowLoginFailuresReport: (payload) =>
        _dispatch({ type: ReporterActionType.showLoginFailuresReport, payload: payload }),
      setShowWorkCellStatesReport: (payload) =>
        _dispatch({ type: ReporterActionType.showWorkCellStatesReport, payload: payload }),
    } as ReducerReporterDispatch;
  }, []);
  return {
    state,
    dispatch,
  };
};
