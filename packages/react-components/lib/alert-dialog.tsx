import * as React from 'react';
import {
  Box,
  Button,
  LinearProgress,
  LinearProgressProps,
  TextField,
  Theme,
  Typography,
  Divider,
} from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    textField: {
      background: theme.palette.background.default,
      pointerEvents: 'none',
    },
  }),
);

export interface AlertContent {
  title: string;
  value: string;
}

export interface CloseAlertDialogProps {
  title: string;
}

export const CloseAlertDialog = React.memo((props: CloseAlertDialogProps) => {
  const { title } = props;
  return <Dialog key={title} open={false} />;
});

export interface DialogAlertProps {
  onDismiss: () => void;
  onAcknowledge?: () => void;
  acknowledgedBy?: string;
  title: string;
  progress?: number;
  alertContents: AlertContent[];
  backgroundColor: string;
}

export const AlertDialog = React.memo((props: DialogAlertProps) => {
  const LinearProgressWithLabel = (props: LinearProgressProps & { value: number }) => {
    return (
      <Box sx={{ display: 'flex', alignItems: 'center' }}>
        <Box sx={{ width: '100%', mr: 1 }}>
          <LinearProgress variant="determinate" {...props} />
        </Box>
        <Box sx={{ minWidth: 35 }}>
          <Typography variant="body2" color="text.secondary">{`${Math.round(
            props.value * 100,
          )}%`}</Typography>
        </Box>
      </Box>
    );
  };

  const returnDialogContent = (alertContents: AlertContent[]) => {
    return (
      <>
        {alertContents.map((message, index) => (
          <div key={index}>
            <TextField
              label={message.title}
              id="standard-size-small"
              size="small"
              variant="filled"
              InputProps={{ readOnly: true, className: classes.textField }}
              fullWidth={true}
              multiline
              maxRows={4}
              margin="dense"
              value={message.value}
            />
          </div>
        ))}
      </>
    );
  };

  const {
    onDismiss,
    onAcknowledge,
    acknowledgedBy,
    title,
    progress,
    alertContents,
    backgroundColor,
  } = props;
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const [acknowledged, setAcknowledged] = React.useState(acknowledgedBy !== undefined);

  return (
    <Dialog
      PaperProps={{
        style: {
          backgroundColor: backgroundColor,
          boxShadow: 'none',
        },
      }}
      maxWidth="sm"
      fullWidth={true}
      open={isOpen}
      key={title}
    >
      <DialogTitle align="center">{title}</DialogTitle>
      <Divider />
      {progress ? (
        <Box sx={{ width: '100%' }}>
          <LinearProgressWithLabel value={progress} />
        </Box>
      ) : null}
      <DialogContent>{returnDialogContent(alertContents)}</DialogContent>

      <DialogActions>
        {acknowledged || onAcknowledge === undefined ? (
          <Button size="small" variant="contained" disabled={true} autoFocus>
            {acknowledgedBy ? `Acknowledged by ${acknowledgedBy}` : 'Acknowledged'}
          </Button>
        ) : (
          <Button
            size="small"
            variant="contained"
            onClick={() => {
              setAcknowledged(true);
              onAcknowledge();
            }}
            disabled={false}
            autoFocus
          >
            Acknowledge
          </Button>
        )}
        <Button
          size="small"
          variant="contained"
          onClick={() => {
            setIsOpen(false);
            onDismiss();
          }}
          autoFocus
        >
          {acknowledged ? 'Close' : 'Dismiss'}
        </Button>
      </DialogActions>
    </Dialog>
  );
});
