import { Button, ButtonGroup } from '@mui/material';
import React from 'react';

export interface DoorControlsProps {
  doorName?: string;
  onOpenClick?(event: React.MouseEvent): void;
  onCloseClick?(event: React.MouseEvent): void;
}

export function DoorControls({
  doorName,
  onOpenClick,
  onCloseClick,
}: DoorControlsProps): JSX.Element {
  return (
    <ButtonGroup variant="contained" size="small" sx={{ alignSelf: 'center' }}>
      <Button onClick={onOpenClick} aria-label={`open-${doorName}`}>
        Open
      </Button>
      <Button onClick={onCloseClick} aria-label={`close-${doorName}`}>
        Close
      </Button>
    </ButtonGroup>
  );
}
