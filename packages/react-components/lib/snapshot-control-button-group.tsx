import { Button, ButtonGroup } from '@material-ui/core';
import ClearAllIcon from '@material-ui/icons/ClearAll';
import RestoreIcon from '@material-ui/icons/Restore';
import RestoreFromTrashIcon from '@material-ui/icons/RestoreFromTrash';
import SaveIcon from '@material-ui/icons/Save';
import React from 'react';

export interface SnapshotControlButtonGroupProps extends React.HTMLAttributes<HTMLDivElement> {
  disableReset?: boolean;
  disableClear?: boolean;
  disableRestore?: boolean;
  disableSave?: boolean;
  showReset?: boolean;
  showClear?: boolean;
  showRestore?: boolean;
  showSave?: boolean;
  onResetClick?: () => void;
  onClearClick?: () => void;
  onRestoreClick?: () => void;
  onSaveClick?: () => void;
  fullWidth?: boolean;
}

export const SnapshotControlButtonGroup = React.forwardRef(function (
  props: SnapshotControlButtonGroupProps,
  ref: React.Ref<HTMLDivElement>,
): React.ReactElement {
  const {
    disableReset,
    disableClear,
    disableRestore,
    disableSave,
    showReset = true,
    showClear = true,
    showRestore = true,
    showSave = true,
    onResetClick,
    onClearClick,
    onRestoreClick,
    onSaveClick,
    fullWidth = true,
    ...otherProps
  } = props;

  return (
    <div {...otherProps} ref={ref}>
      <ButtonGroup fullWidth={fullWidth}>
        {showReset && (
          <Button
            id="reset-button"
            disabled={disableReset}
            onClick={() => onResetClick && onResetClick()}
          >
            <RestoreIcon />
            Reset
          </Button>
        )}
        {showClear && (
          <Button
            id="clear-button"
            disabled={disableClear}
            onClick={() => onClearClick && onClearClick()}
          >
            <ClearAllIcon />
            Clear
          </Button>
        )}
        {showRestore && (
          <Button
            id="restore-button"
            disabled={disableRestore}
            onClick={() => onRestoreClick && onRestoreClick()}
          >
            <RestoreFromTrashIcon />
            Restore
          </Button>
        )}
        {showSave && (
          <Button
            id="save-button"
            disabled={disableSave}
            onClick={() => onSaveClick && onSaveClick()}
          >
            <SaveIcon />
            Save
          </Button>
        )}
      </ButtonGroup>
    </div>
  );
});

export default SnapshotControlButtonGroup;
