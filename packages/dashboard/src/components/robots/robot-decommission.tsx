import { Button, ButtonProps, Theme, Tooltip, Typography } from '@mui/material';
import { RobotState, Status2 } from 'api-client';
import React from 'react';
import { AppControllerContext } from '../app-contexts';
import { RmfAppContext } from '../rmf-app';
import { UserProfile, UserProfileContext } from 'rmf-auth';
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
  const profile: UserProfile | null = React.useContext(UserProfileContext);

  enum ConfirmDialogType {
    None,
    Decommission,
    Reinstate,
  }
  const [openConfirmDialog, setOpenConfirmDialog] = React.useState<ConfirmDialogType>(
    ConfirmDialogType.None,
  );

  const robotDecommissioned =
    robotState && robotState.status && robotState.status === Status2.Uninitialized;
  // TODO: use authz for decommissioning robot
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
      const labels = profile ? [profile.user.username] : [];
      await rmf.fleetsApi?.decommissionRobotFleetsNameDecommissionPost(
        fleet,
        robotState.name,
        id,
        labels,
      );
      appController.showAlert('success', `Decommission of ${fleet}:${robotState.name} requested`);
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to decommission ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    setOpenConfirmDialog(ConfirmDialogType.None);
  }, [appController, fleet, robotState, rmf, profile, ConfirmDialogType]);

  const handleReinstate = React.useCallback<React.MouseEventHandler>(async () => {
    if (!robotState || !robotState.name) {
      return;
    }
    try {
      if (!rmf) {
        throw new Error('fleets api not available');
      }
      const id = `reinstate-${fleet}-${robotState.name}-${Date.now()}`;
      const labels = profile ? [profile.user.username] : [];
      await rmf.fleetsApi?.reinstateRobotFleetsNameReinstatePost(
        fleet,
        robotState.name,
        id,
        labels,
      );
      appController.showAlert('success', `Reinstatement of ${fleet}:${robotState.name} requested`);
    } catch (e) {
      appController.showAlert(
        'error',
        `Failed to reinstate ${fleet}:${robotState.name}: ${(e as Error).message}`,
      );
    }
    setOpenConfirmDialog(ConfirmDialogType.None);
  }, [appController, fleet, robotState, rmf, profile, ConfirmDialogType]);

  return (
    <>
      {robotDecommissioned ? (
        <Button
          onClick={() => setOpenConfirmDialog(ConfirmDialogType.Reinstate)}
          autoFocus
          {...otherProps}
        >
          {'Reinstate robot'}
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
        <Tooltip title={`Robot from fleet ${fleet} cannot be decommissioned/reinstated.`}>
          <Button disabled className={classes['enableHover']} {...otherProps}>
            {'Decommission/Reinstate robot'}
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
          onSubmit={handleReinstate}
        >
          <Typography>Confirm reinstate robot?</Typography>
        </ConfirmationDialog>
      ) : (
        <></>
      )}
    </>
  );
}
