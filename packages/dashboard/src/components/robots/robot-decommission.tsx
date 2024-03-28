import {
  Button,
  ButtonProps,
  Checkbox,
  FormGroup,
  FormControlLabel,
  Theme,
  Tooltip,
  Typography,
} from '@mui/material';
import { AppEvents } from '../app-events';
import { RobotState } from 'api-client';
import React from 'react';
import { AppControllerContext } from '../app-contexts';
import { RmfAppContext } from '../rmf-app';
import { makeStyles, createStyles } from '@mui/styles';
import { ConfirmationDialog } from 'react-components';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    enableHover: {
      '&.Mui-disabled': {
        pointerEvents: 'auto',
      },
    },
  }),
);

export interface RobotDecommissionButtonProp extends ButtonProps {
  fleet: string;
  robotState: RobotState | null;
}

export function RobotDecommissionButton({
  fleet,
  robotState,
  ...otherProps
}: RobotDecommissionButtonProp): JSX.Element {
  const classes = useStyles();
  const rmf = React.useContext(RmfAppContext);
  const appController = React.useContext(AppControllerContext);
  const [reassignTasks, setReassignTasks] = React.useState(true);

  const handleReassignTasksChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setReassignTasks(event.target.checked);
  };

  enum ConfirmDialogType {
    None,
    Decommission,
    Recommission,
  }
  const [openConfirmDialog, setOpenConfirmDialog] = React.useState<ConfirmDialogType>(
    ConfirmDialogType.None,
  );

  const robotDecommissioned =
    robotState &&
    robotState.commission &&
    robotState.commission.dispatch_tasks === false &&
    robotState.commission.direct_tasks === false &&
    robotState.commission.idle_behavior === false;

  // TODO: use authz for decommissioning robot
  // const profile: UserProfile | null = React.useContext(UserProfileContext);
  // const userCanDecommissionRobot = profile && Enforcer.canDecommissionRobot(profile);

  const handleDecommission = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      if (!rmf) {
        throw new Error('fleets api not available');
      }
      const resp = await rmf.fleetsApi?.decommissionRobotFleetsNameDecommissionPost(
        fleet,
        robotState.name,
        reassignTasks,
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
          'success',
          `Decommission of ${fleet}:${robotState.name} requested, ${
            reassignTasks ? 'with' : 'without'
          } task re-assignment${errors.length !== 0 ? `, but with errors: ${errors}` : ''}`,
        );
      }
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to decommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    setReassignTasks(true);
    AppEvents.refreshRobotApp.next();
    setOpenConfirmDialog(ConfirmDialogType.None);
  }, [appController, fleet, robotState, reassignTasks, rmf, ConfirmDialogType]);

  const handleRecommission = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      if (!rmf) {
        throw new Error('fleets api not available');
      }
      const resp = await rmf.fleetsApi?.recommissionRobotFleetsNameRecommissionPost(
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
    AppEvents.refreshRobotApp.next();
    setOpenConfirmDialog(ConfirmDialogType.None);
  }, [appController, fleet, robotState, rmf, ConfirmDialogType]);

  return (
    <>
      {robotDecommissioned ? (
        <Button
          onClick={() => setOpenConfirmDialog(ConfirmDialogType.Recommission)}
          autoFocus
          {...otherProps}
        >
          {'Recommission robot'}
        </Button>
      ) : robotState && !robotDecommissioned ? (
        <Button
          onClick={() => setOpenConfirmDialog(ConfirmDialogType.Decommission)}
          autoFocus
          {...otherProps}
        >
          {'Decommission robot'}
        </Button>
      ) : (
        <Tooltip title={`Robot from fleet ${fleet} cannot be decommissioned/recommissioned.`}>
          <Button disabled className={classes['enableHover']} {...otherProps}>
            {'Decommission/Recommission robot'}
          </Button>
        </Tooltip>
      )}
      {openConfirmDialog === ConfirmDialogType.Decommission ? (
        <ConfirmationDialog
          confirmText="Confirm"
          cancelText="Cancel"
          open={openConfirmDialog === ConfirmDialogType.Decommission}
          title={`Decommission [${fleet}:${robotState?.name || 'n/a'}]`}
          submitting={undefined}
          onClose={() => {
            setOpenConfirmDialog(ConfirmDialogType.None);
          }}
          onSubmit={handleDecommission}
        >
          <Typography>Confirm decommission robot?</Typography>
          <FormGroup>
            <FormControlLabel
              control={<Checkbox checked={reassignTasks} onChange={handleReassignTasksChange} />}
              label="Re-assign tasks"
            />
          </FormGroup>
        </ConfirmationDialog>
      ) : openConfirmDialog === ConfirmDialogType.Recommission ? (
        <ConfirmationDialog
          confirmText="Confirm"
          cancelText="Cancel"
          open={openConfirmDialog === ConfirmDialogType.Recommission}
          title={`Recommission [${fleet}:${robotState?.name || 'n/a'}]`}
          submitting={undefined}
          onClose={() => {
            setOpenConfirmDialog(ConfirmDialogType.None);
          }}
          onSubmit={handleRecommission}
        >
          <Typography>Confirm recommission robot?</Typography>
        </ConfirmationDialog>
      ) : (
        <></>
      )}
    </>
  );
}
