import React from 'react';
import AllLogsReport from './reports/all-logs';

export enum Reports {
  queryAllLogs = 'queryAllLogs',
  showChargerStateReport = 'showChargerStateReport',
  showDoorStateReport = 'showDoorStateReport',
  showLiftStateReport = 'showLiftStateReport',
  showNegotiationsReport = 'showNegotiationsReport',
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

/**
 * Report dictionary with it associated component
 */

export const ReportContainer: Record<string, React.ReactElement> = {
  [Reports.queryAllLogs]: <AllLogsReport />,
  [Reports.showChargerStateReport]: <h1>Still not implemented</h1>,
  [Reports.showDoorStateReport]: <h1>Still not implemented</h1>,
  [Reports.showLiftStateReport]: <h1>Still not implemented</h1>,
  [Reports.showNegotiationsReport]: <h1>Still not implemented</h1>,
  [Reports.showRobotStateReport]: <h1>Still not implemented</h1>,
  [Reports.showRobotMotionPlansReport]: <h1>Still not implemented</h1>,
  [Reports.showRobotActionReport]: <h1>Still not implemented</h1>,
  [Reports.showTasksReport]: <h1>Still not implemented</h1>,
  [Reports.showUserActionsReport]: <h1>Still not implemented</h1>,
  [Reports.showLoginsReport]: <h1>Still not implemented</h1>,
  [Reports.showLogoutsReport]: <h1>Still not implemented</h1>,
  [Reports.showLoginFailuresReport]: <h1>Still not implemented</h1>,
  [Reports.showWorkCellStatesReport]: <h1>Still not implemented</h1>,
};
