import {
  Button,
  Dialog,
  DialogProps,
  DialogActions,
  DialogContent,
  DialogTitle,
  Divider,
  IconButton,
  Typography,
  styled,
} from '@material-ui/core';
import CloseIcon from '@material-ui/icons/Close';
import React from 'react';
import { getApplicationKeyMap } from 'react-hotkeys';

export interface HotKeysDialogProps {
  handleClose(): void;
  open: boolean;
}

const classes = {
  closeButton: 'close-button',
  dialogContent: 'dialog-content',
  dialogActions: 'dialog-actions',
  detailLine: 'detail-line',
  detail: 'detail',
};
const HotKeysDialogRoot = styled((props: DialogProps) => <Dialog {...props} />)(({ theme }) => ({
  [`& .${classes.closeButton}`]: {
    position: 'absolute',
    right: theme.spacing(1),
    top: theme.spacing(1),
    color: theme.palette.grey[500],
  },
  [`& .${classes.dialogContent}`]: {
    padding: theme.spacing(2),
  },
  [`& .${classes.dialogActions}`]: {
    margin: 0,
    padding: theme.spacing(1),
  },
  [`& .${classes.detailLine}`]: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
    width: '100%',
  },
  [`& .${classes.detail}`]: {
    display: 'flex',
    flexFlow: 'column',
    padding: '1rem',
  },
}));

export default function HotKeysDialog(props: HotKeysDialogProps): React.ReactElement {
  const { open, handleClose } = props;
  const keyMap = getApplicationKeyMap();
  return (
    <HotKeysDialogRoot
      onClose={handleClose}
      aria-labelledby="customized-dialog-title"
      open={open}
      fullWidth={true}
      maxWidth={'sm'}
    >
      <DialogTitle id="customized-dialog-title">
        <div>
          <Typography variant="h6">Hotkeys</Typography>
          <IconButton aria-label="close" className={classes.closeButton} onClick={handleClose}>
            <CloseIcon />
          </IconButton>
        </div>
      </DialogTitle>
      <DialogContent className={classes.dialogContent} dividers>
        <div className={classes.detail}>
          {Object.values(keyMap).map((hotkey) => {
            return (
              hotkey.name && (
                <React.Fragment key={hotkey.name}>
                  <div
                    detail-type={'hotkeyDetail'}
                    className={classes.detailLine}
                    data-testid="hotkeyDetail"
                  >
                    <Typography variant="body1">{hotkey.name}:</Typography>
                    <Typography variant="body1">{hotkey.sequences[0].sequence}</Typography>
                  </div>
                  <Divider />
                </React.Fragment>
              )
            );
          })}
        </div>
      </DialogContent>
      <DialogActions className={classes.dialogActions}>
        <Button autoFocus onClick={handleClose} color="primary">
          OK
        </Button>
      </DialogActions>
    </HotKeysDialogRoot>
  );
}
