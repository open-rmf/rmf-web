import {
  Button,
  ButtonProps,
  Checkbox,
  FormControlLabel,
  FormGroup,
  Tooltip,
  Typography,
} from '@mui/material';
import { RobotState } from 'api-client';
import React from 'react';
import { ConfirmationDialog } from 'react-components';

import { useAppController } from '../../hooks/use-app-controller';
import { useRmfApi } from '../../hooks/use-rmf-api';
import { AppEvents } from '../app-events';

export interface RobotDecommissionButtonProp extends Omit<ButtonProps, 'onClick' | 'autoFocus'> {
  fleet: string;
  robotState: RobotState | null;
}

export function RobotDecommissionButton({
  fleet,
  robotState,
  ...otherProps
}: RobotDecommissionButtonProp) {
  const rmfApi = useRmfApi();
  const appController = useAppController();
  const [reassignTasks, setReassignTasks] = React.useState(true);
  const [allowIdleBehavior, setAllowIdleBehavior] = React.useState(false);
  const [openConfirmDialog, setOpenConfirmDialog] = React.useState(false);

  const handleReassignTasksChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setReassignTasks(event.target.checked);
  };

  const handleAllowIdleBehaviorChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setAllowIdleBehavior(event.target.checked);
  };

  const resetDecommissionConfiguration = () => {
    setReassignTasks(true);
    setAllowIdleBehavior(false);
    setOpenConfirmDialog(false);
  };

  const robotDecommissioned =
    robotState &&
    robotState.commission &&
    robotState.commission.dispatch_tasks === false &&
    robotState.commission.direct_tasks === false;

  // TODO: use authz for decommissioning robot
  // const profile: UserProfile | null = React.useContext(UserProfileContext);
  // const userCanDecommissionRobot = profile && Enforcer.canDecommissionRobot(profile);

  const handleDecommission = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      const resp = await rmfApi.fleetsApi?.decommissionRobotFleetsNameDecommissionPost(
        fleet,
        robotState.name,
        reassignTasks,
        allowIdleBehavior,
      );

      if (!resp.data.commission.success) {
        appController.showAlert(
          'error',
          `Failed to decommission ${fleet}:${robotState.name}: ${
            resp.data.commission.errors ?? ''
          }`,
        );
      } else {
        const errors = [];
        if (
          resp.data.pending_direct_tasks_policy &&
          !resp.data.pending_direct_tasks_policy?.success
        ) {
          errors.push(`${resp.data.pending_direct_tasks_policy.errors}`);
        }
        if (
          resp.data.pending_dispatch_tasks_policy &&
          !resp.data.pending_dispatch_tasks_policy?.success
        ) {
          errors.push(`${resp.data.pending_dispatch_tasks_policy.errors}`);
        }

        appController.showAlert(
          errors.length !== 0 ? 'warning' : 'success',
          `Decommission of ${fleet}:${robotState.name} requested, ${
            reassignTasks ? 'with' : 'without'
          } task re-assignment, ${allowIdleBehavior ? 'allow' : 'not allowing'} idle behaviors${
            errors.length !== 0 ? `, but with errors: ${errors}` : ''
          }`,
        );
      }
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to decommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    resetDecommissionConfiguration();
    AppEvents.refreshRobotApp.next();
  }, [appController, fleet, robotState, reassignTasks, allowIdleBehavior, rmfApi]);

  const handleRecommission = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      const resp = await rmfApi.fleetsApi?.recommissionRobotFleetsNameRecommissionPost(
        fleet,
        robotState.name,
      );
      if (!resp.data.commission.success) {
        appController.showAlert(
          'error',
          `Failed to recommission ${fleet}:${robotState.name}: ${
            resp.data.commission.errors ?? ''
          }`,
        );
      } else {
        appController.showAlert('success', `Recommission of ${fleet}:${robotState.name} requested`);
      }
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to recommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    resetDecommissionConfiguration();
    AppEvents.refreshRobotApp.next();
  }, [appController, fleet, robotState, rmfApi]);

  return (
    <>
      {robotState ? (
        <Button onClick={() => setOpenConfirmDialog(true)} autoFocus {...otherProps}>
          {robotDecommissioned ? 'Recommission' : 'Decommission'}
        </Button>
      ) : (
        <Tooltip title={`Robot from fleet ${fleet} cannot be decommissioned/recommissioned.`}>
          <Button
            disabled
            sx={{
              '&.Mui-disabled': {
                pointerEvents: 'auto',
              },
            }}
            {...otherProps}
          >
            {'Decommission'}
          </Button>
        </Tooltip>
      )}
      {openConfirmDialog && (
        <ConfirmationDialog
          confirmText="Confirm"
          cancelText="Cancel"
          open={true}
          title={`${robotDecommissioned ? 'Recommission' : 'Decommission'} [${fleet}:${
            robotState?.name || 'n/a'
          }]`}
          submitting={undefined}
          onClose={() => {
            resetDecommissionConfiguration();
          }}
          onSubmit={robotDecommissioned ? handleRecommission : handleDecommission}
        >
          {robotDecommissioned ? (
            <>
              <Typography>Confirm recommission robot?</Typography>
              <Typography color="warning.main">
                Warning: the robot could immediately be assigned a new task after recommission,
                please ensure that the robot is fully functional, localized, and its location
                reflecting accurately on the Open-RMF map before recommissioning.
              </Typography>
            </>
          ) : (
            <>
              <Typography>Confirm decommission robot?</Typography>
              <Typography color="warning.main">
                Warning: ongoing tasks will not be affected. If manual intervention is required,
                please ensure the ongoing task is cancelled after decommission, before executing
                manual intervention.
              </Typography>
              <FormGroup>
                <Tooltip title="Attempts to reassign all queued tasks to other robots if possible">
                  <FormControlLabel
                    control={
                      <Checkbox checked={reassignTasks} onChange={handleReassignTasksChange} />
                    }
                    label="Re-assign queued tasks"
                  />
                </Tooltip>
                <Tooltip title="Allows the robot to perform its idle behavior (charging, parking, etc)">
                  <FormControlLabel
                    control={
                      <Checkbox
                        checked={allowIdleBehavior}
                        onChange={handleAllowIdleBehaviorChange}
                      />
                    }
                    label="Allow idle behavior"
                  />
                </Tooltip>
              </FormGroup>
            </>
          )}
        </ConfirmationDialog>
      )}
    </>
  );
}
