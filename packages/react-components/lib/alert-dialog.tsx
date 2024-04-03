import {
  Box,
  Button,
  Divider,
  LinearProgress,
  LinearProgressProps,
  TextField,
  Typography,
  useTheme,
} from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import React from 'react';

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
  onInspect?: () => void;
  acknowledgedBy?: string;
  title: string;
  progress?: number;
  alertContents: AlertContent[];
  backgroundColor: string;
}

export const AlertDialog = React.memo((props: DialogAlertProps) => {
  const theme = useTheme();
  const LinearProgressWithLabel = (props: LinearProgressProps & { value: number }) => {
    return (
      <Box component="div" sx={{ display: 'flex', alignItems: 'center' }}>
        <Box component="div" sx={{ width: '100%', mr: 1 }}>
          <LinearProgress variant="determinate" {...props} value={props.value * 100} />
        </Box>
        <Box component="div" sx={{ minWidth: 35 }}>
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
              sx={{
                background: theme.palette.background.default,
                pointerEvents: 'none',
              }}
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
    onInspect,
    acknowledgedBy,
    title,
    progress,
    alertContents,
    backgroundColor,
  } = props;
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
        <>
          <Typography variant="body2" fontWeight="bold" ml={3} mt={1}>
            Task progress
          </Typography>
          <Box component="div" width={0.95} ml={3}>
            <LinearProgressWithLabel value={progress} />
          </Box>
        </>
      ) : null}
      <DialogContent>{returnDialogContent(alertContents)}</DialogContent>

      <DialogActions>
        {onInspect ? (
          <Button size="small" variant="contained" onClick={onInspect} disabled={false} autoFocus>
            Inspect
          </Button>
        ) : null}
        {acknowledged ? (
          <Button size="small" variant="contained" disabled={true} autoFocus>
            {acknowledgedBy ? `Acknowledged by ${acknowledgedBy}` : 'Acknowledged'}
          </Button>
        ) : onAcknowledge === undefined ? null : (
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
