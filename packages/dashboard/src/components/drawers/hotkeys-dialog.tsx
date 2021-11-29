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
} from '@mui/material';
import CloseIcon from '@mui/icons-material/Close';
import React from 'react';
import { getApplicationKeyMap } from 'react-hotkeys';

export interface HotKeysDialogProps {
  handleClose(): void;
  open: boolean;
}

const prefix = 'hotkey-dialog';
const classes = {
  closeButton: `${prefix}-close-button`,
  dialogContent: `${prefix}-content`,
  dialogActions: `${prefix}-actions`,
  detailLine: `${prefix}-detail-line`,
  detail: `${prefix}-detail`,
};
const StyledDialog = styled((props: DialogProps) => <Dialog {...props} />)(({ theme }) => ({
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
    <StyledDialog
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
    </StyledDialog>
  );
}
