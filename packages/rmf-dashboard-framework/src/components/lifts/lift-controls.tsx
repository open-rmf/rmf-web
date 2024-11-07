import { Button, useMediaQuery } from '@mui/material';
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
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');
  const [showDialog, setShowDialog] = React.useState(false);
  // Doing `{showDialog && <Form .../>}` will unomunt it before the animations are done.
  // Instead we give a `key` to the form to make react spawn a new instance.
  const [resetForm, setResetForm] = React.useState(0);

  return (
    <>
      <Button
        variant="contained"
        color="primary"
        size="small"
        onClick={() => {
          setResetForm((prev) => prev + 1);
          setShowDialog(true);
        }}
        sx={{
          minWidth: 'auto',
          fontSize: isScreenHeightLessThan800 ? 10 : 16,
        }}
      >
        Request
      </Button>
      <LiftRequestDialog
        key={resetForm}
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
