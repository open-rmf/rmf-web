import { Button, ButtonProps, Theme, Tooltip, Typography } from '@mui/material';
import { AppEvents } from '../app-events';
import { RobotState, ApiServerModelsRmfApiRobotStateStatus as Status } from 'api-client';
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

  enum ConfirmDialogType {
    None,
    Decommission,
    Reinstate,
  }
  const [openConfirmDialog, setOpenConfirmDialog] = React.useState<ConfirmDialogType>(
    ConfirmDialogType.None,
  );

  const robotDecommissioned =
    robotState && robotState.status && robotState.status === Status.Uninitialized;
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
      const id = `decommission-${fleet}-${robotState.name}-${Date.now()}`;
      await rmf.fleetsApi?.decommissionRobotFleetsNameDecommissionPost(fleet, robotState.name, id);
      appController.showAlert('success', `Decommission of ${fleet}:${robotState.name} requested`);
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to decommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    AppEvents.refreshRobotApp.next();
    setOpenConfirmDialog(ConfirmDialogType.None);
  }, [appController, fleet, robotState, rmf, ConfirmDialogType]);

  const handleRecommission = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      if (!rmf) {
        throw new Error('fleets api not available');
      }
      const id = `recommission-${fleet}-${robotState.name}-${Date.now()}`;
      await rmf.fleetsApi?.recommissionRobotFleetsNameRecommissionPost(fleet, robotState.name, id);
      appController.showAlert('success', `Recommission of ${fleet}:${robotState.name} requested`);
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
          onClick={() => setOpenConfirmDialog(ConfirmDialogType.Reinstate)}
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
        </ConfirmationDialog>
      ) : openConfirmDialog === ConfirmDialogType.Reinstate ? (
        <ConfirmationDialog
          confirmText="Confirm"
          cancelText="Cancel"
          open={openConfirmDialog === ConfirmDialogType.Reinstate}
          title={`Reinstate [${fleet}:${robotState?.name || 'n/a'}]`}
          submitting={undefined}
          onClose={() => {
            setOpenConfirmDialog(ConfirmDialogType.None);
          }}
          onSubmit={handleRecommission}
        >
          <Typography>Confirm reinstate robot?</Typography>
        </ConfirmationDialog>
      ) : (
        <></>
      )}
    </>
  );
}
