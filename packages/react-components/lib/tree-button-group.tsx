import { Button, ButtonGroup } from '@material-ui/core';
import ClearAllIcon from '@material-ui/icons/ClearAll';
import RestoreIcon from '@material-ui/icons/Restore';
import RestoreFromTrashIcon from '@material-ui/icons/RestoreFromTrash';
import React from 'react';

export interface TreeButtonGroupProps {
  disableReset?: boolean;
  disableClear?: boolean;
  disableRestore?: boolean;
  handleResetClick?: () => void;
  handleClearClick?: () => void;
  handleRestoreClick?: () => void;
  fullWidth?: boolean;
}

export const TreeButtonGroup = (props: TreeButtonGroupProps): JSX.Element => {
  const {
    disableReset,
    disableClear,
    disableRestore,
    handleResetClick,
    handleClearClick,
    handleRestoreClick,
    fullWidth = true,
  } = props;
  return (
    <div>
      <ButtonGroup fullWidth={fullWidth}>
        <Button
          id="reset-button"
          disabled={disableReset}
          onClick={() => handleResetClick && handleResetClick()}
        >
          <RestoreIcon />
          Reset
        </Button>
        <Button
          id="clear-button"
          disabled={disableClear}
          onClick={() => handleClearClick && handleClearClick()}
        >
          <ClearAllIcon />
          Clear
        </Button>
        <Button
          id="restore-button"
          disabled={disableRestore}
          onClick={() => handleRestoreClick && handleRestoreClick()}
        >
          <RestoreFromTrashIcon />
          Restore
        </Button>
      </ButtonGroup>
    </div>
  );
};

export default TreeButtonGroup;
