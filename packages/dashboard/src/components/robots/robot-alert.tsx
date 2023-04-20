import React from 'react';
import {
  ApiServerModelsTortoiseModelsAlertsAlertLeaf,
  ApiServerModelsRmfApiFleetStateFleetState,
  RobotState,
  Status2,
} from 'api-client';
import { RmfAppContext } from '../rmf-app';
import { AlertContent, AlertDialog, CloseAlertDialog } from 'react-components';
import { base } from 'react-components';
import { Subscription } from 'rxjs';
import { AppControllerContext } from '../app-contexts';

type Alert = ApiServerModelsTortoiseModelsAlertsAlertLeaf;
type FleetState = ApiServerModelsRmfApiFleetStateFleetState;

interface RobotAlert extends RobotState {
  fleet: string;
  show: boolean;
}

export interface RobotAlertHandlerProps {
  alerts: Alert[];
  removeAlert: (id: string) => void;
}

export function RobotAlertHandler({ alerts, removeAlert }: RobotAlertHandlerProps): JSX.Element {
  // Splits the alert ID into the fleet name and robot name, using 2 underscores
  // as delimiters
  const parseFleetAndRobotNames = (alertId: string): [string, string] | null => {
    const result = alertId.split('__', 2);
    if (result.length === 2) {
      return [result[0], result[1]];
    }
    return null;
  };

  const getAlertTitle = (alert: RobotAlert): string => {
    if (alert.status && alert.status === Status2.Error) {
      return 'Robot error';
    }
    return 'Robot alert';
  };

  const getAlertContent = (alert: RobotAlert): AlertContent[] => {
    let content: AlertContent[] = [
      {
        title: 'Fleet',
        value: alert.fleet,
      },
      {
        title: 'Robot name',
        value: alert.name ? alert.name : 'Name unavailable',
      },
      {
        title: 'Battery percentage',
        value: alert.battery ? `${Math.round(alert.battery * 100)}%` : 'Battery level unavailable',
      },
      {
        title: 'Current task',
        value: alert.task_id ? alert.task_id : `-`,
      },
      {
        title: 'Last updated',
        value: alert.unix_millis_time
          ? `${Math.round((new Date().getTime() - alert.unix_millis_time) / 1000)}s ago`
          : '-',
      },
    ];

    if (!alert.issues || alert.issues.length === 0) {
      content = [
        ...content,
        {
          title: 'Issues',
          value: 'No issues found',
        },
      ];
      return content;
    }

    let consolidatedIssues = '';
    for (let issue of alert.issues) {
      const category = issue.category ? `${issue.category} - ` : '';
      const details = issue.detail ? JSON.stringify(issue.detail) : '';
      consolidatedIssues += `${category}${details}\n`;
    }
    content = [
      ...content,
      {
        title: 'Issues',
        value: consolidatedIssues,
      },
    ];
    return content;
  };

  const getAlertColor = (alert: RobotAlert): string => {
    if (alert.status && alert.status === Status2.Error) {
      return base.palette.error.main;
    }
    return base.palette.background.default;
  };

  const rmf = React.useContext(RmfAppContext);
  const { showAlert } = React.useContext(AppControllerContext);
  const [robotAlerts, setRobotAlerts] = React.useState<Record<string, RobotAlert>>({});

  React.useEffect(() => {
    if (!rmf) {
      return;
    }
    const subs = alerts.map((alert) => {
      const names = parseFleetAndRobotNames(alert.original_id);
      if (!names || names.length !== 2) {
        console.log(`Failed to retrieve fleet and robot name from alert ${alert.original_id}`);
        return;
      }

      const fleet = names[0];
      const robot = names[1];
      return rmf.getFleetStateObs(fleet).subscribe(async (fleetState) => {
        if (fleetState.robots && fleetState.robots[robot]) {
          const robotState = fleetState.robots[robot];
          setRobotAlerts((prev) => {
            return {
              ...prev,
              [alert.original_id]: {
                fleet: fleet,
                show: robotState.status && robotState.status === Status2.Error ? true : false,
                ...robotState,
              },
            };
          });
        }
      });
    });
    return () => subs.forEach((sub) => sub && sub.unsubscribe());
  }, [rmf, alerts]);

  return (
    <>
      {Object.keys(robotAlerts).map((alertId) => {
        const dismissAlert = () => {
          removeAlert(alertId);
        };
        const acknowledgeAlert = () => {
          if (!rmf) {
            throw new Error('alerts api not available');
          }
          (async () => {
            const ackResponse = (await rmf?.alertsApi.acknowledgeAlertAlertsIdPost(alertId)).data;
            if (ackResponse.id !== ackResponse.original_id) {
              let showAlertMessage = `Alert ${ackResponse.original_id} acknowledged`;
              if (ackResponse.acknowledged_by) {
                showAlertMessage += ` by User ${ackResponse.acknowledged_by}`;
              }
              if (ackResponse.unix_millis_acknowledged_time) {
                const ackSecondsAgo =
                  (new Date().getTime() - ackResponse.unix_millis_acknowledged_time) / 1000;
                showAlertMessage += ` ${Math.round(ackSecondsAgo)}s ago`;
              }
              showAlert('success', showAlertMessage);
            } else {
              console.log(`Failed to acknowledge alert ID ${alertId}`);
              showAlert('error', `Failed to acknowledge alert ID ${alertId}`);
            }
          })();
        };

        const alert = robotAlerts[alertId];
        if (alert.show) {
          return (
            <AlertDialog
              key={alertId}
              dismiss={dismissAlert}
              acknowledge={acknowledgeAlert}
              title={getAlertTitle(alert)}
              alertContents={getAlertContent(alert)}
              backgroundColor={getAlertColor(alert)}
            />
          );
        } else {
          delete robotAlerts[alertId];
          return <CloseAlertDialog key={alertId} title={getAlertTitle(alert)} />;
        }
      })}
    </>
  );
}
