import { Button, ButtonGroup } from '@mui/material';
import React from 'react';

export interface DoorControlsProps {
  onOpenClick?(event: React.MouseEvent): void;
  onCloseClick?(event: React.MouseEvent): void;
}

export function DoorControls({ onOpenClick, onCloseClick }: DoorControlsProps): JSX.Element {
  return (
    <ButtonGroup variant="contained" size="small" sx={{ alignSelf: 'center' }}>
      <Button onClick={onOpenClick}>Open</Button>
      <Button onClick={onCloseClick}>Close</Button>
    </ButtonGroup>
  );
}
