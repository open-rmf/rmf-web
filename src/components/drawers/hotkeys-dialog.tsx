import { Divider, makeStyles, Typography, Dialog, Button } from '@material-ui/core';
import React from 'react';
import { getApplicationKeyMap } from 'react-hotkeys';
import { DialogTitle, DialogContent, DialogActions } from '../dialog';

export interface HotKeysDialogProps {
  handleClose(): void;
  open: boolean;
}

export default function HotKeysDialog(props: HotKeysDialogProps): React.ReactElement {
  const { open, handleClose } = props;
  const classes = useStyles();
  const keyMap = getApplicationKeyMap();
  return (
    <Dialog
      onClose={handleClose}
      aria-labelledby="customized-dialog-title"
      open={open}
      fullWidth={true}
      maxWidth={'sm'}
    >
      <DialogTitle id="customized-dialog-title" onClose={handleClose}>
        Hotkeys
      </DialogTitle>
      <DialogContent dividers>
        <div className={classes.detail}>
          {Object.values(keyMap).map((hotkey) => {
            return (
              hotkey.name && (
                <>
                  <div
                    detail-type={'hotkeyDetail'}
                    key={hotkey.name}
                    className={classes.detailLine}
                  >
                    <Typography variant="body1">{hotkey.name}:</Typography>
                    <Typography variant="body1">{hotkey.sequences[0].sequence}</Typography>
                  </div>
                  <Divider />
                </>
              )
            );
          })}
        </div>
      </DialogContent>
      <DialogActions>
        <Button autoFocus onClick={handleClose} color="primary">
          Ok
        </Button>
      </DialogActions>
    </Dialog>
  );
}

const useStyles = makeStyles((theme) => ({
  detailLine: {
    display: 'inline-flex',
    justifyContent: 'space-between',
    padding: theme.spacing(0.5),
    width: '100%',
  },
  detail: {
    display: 'flex',
    flexFlow: 'column',
    padding: '1rem',
  },
}));
