import { Button } from '@mui/material';
import React from 'react';
import { LiftRequestDialog, LiftRequestDialogProps } from './lift-request-dialog';
import { requestDoorModes, requestModes } from './lift-utils';

export interface LiftControlsProps
  extends Omit<
    LiftRequestDialogProps,
    'showFormDialog' | 'currentLevel' | 'availableDoorModes' | 'availableRequestTypes' | 'onClose'
  > {
  currentLevel?: string;
  onClose?: LiftRequestDialogProps['onClose'];
}

export function LiftControls({
  currentLevel,
  onClose,
  ...otherProps
}: LiftControlsProps): JSX.Element {
  const [showDialog, setShowDialog] = React.useState(false);

  return (
    <>
      <Button
        variant="contained"
        color="primary"
        fullWidth
        size="small"
        onClick={() => setShowDialog(true)}
      >
        Request
      </Button>
      <LiftRequestDialog
        showFormDialog={showDialog}
        currentLevel={currentLevel ? currentLevel : 'Unknown'}
        availableDoorModes={requestDoorModes}
        availableRequestTypes={requestModes}
        onClose={(...args) => {
          setShowDialog(false);
          onClose && onClose(...args);
        }}
        {...otherProps}
      />
    </>
  );
}
