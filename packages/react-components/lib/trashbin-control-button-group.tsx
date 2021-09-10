import { Button, ButtonGroup } from '@material-ui/core';
import ClearAllIcon from '@material-ui/icons/ClearAll';
import RestoreIcon from '@material-ui/icons/Restore';
import RestoreFromTrashIcon from '@material-ui/icons/RestoreFromTrash';
import SaveIcon from '@material-ui/icons/Save';
import React from 'react';

export interface TrashBinControlButtonGroupProps extends React.HTMLAttributes<HTMLDivElement> {
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

/**
 * @param disableReset disable reset button.
 * @param disableClear disable clear button.
 * @param disableRestore disable restore button.
 * @param disableSave disable save button.
 * @param showReset shows reset button. Default value `true`.
 * @param showClear shows clear button. Default value `true`.
 * @param showRestore shows restore button. Default value `true`.
 * @param showSave shows show button. Default value `true`.
 * @param onResetClick handle the click event on the reset button.
 * @param onClearClick handle the click event on the clear button.
 * @param onRestoreClick handle the click event on the restore button.
 * @param onSaveClick handle the click event on the save button.
 * @param fullWidth set property full width. Default value `true`.
 */
export const TrashBinControlButtonGroup = React.forwardRef(function (
  props: TrashBinControlButtonGroupProps,
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

export default TrashBinControlButtonGroup;
