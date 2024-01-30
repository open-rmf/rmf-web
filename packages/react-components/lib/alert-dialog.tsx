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
  useMediaQuery,
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
  onInspect?: () => void;
  acknowledgedBy?: string;
  title: string;
  progress?: number;
  alertContents: AlertContent[];
  backgroundColor: string;
}

export const AlertDialog = React.memo((props: DialogAlertProps) => {
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
                '& .MuiFilledInput-root': {
                  fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1.15',
                },
              }}
              InputLabelProps={{ style: { fontSize: isScreenHeightLessThan800 ? 16 : 20 } }}
              InputProps={{ readOnly: true, className: classes.textField }}
              fullWidth={true}
              multiline
              maxRows={isScreenHeightLessThan800 ? 5 : 10}
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
  const classes = useStyles();
  const [isOpen, setIsOpen] = React.useState(true);
  const [acknowledged, setAcknowledged] = React.useState(acknowledgedBy !== undefined);
  const isScreenHeightLessThan800 = useMediaQuery('(max-height:800px)');

  return (
    <Dialog
      PaperProps={{
        style: {
          backgroundColor: backgroundColor,
          boxShadow: 'none',
        },
      }}
      maxWidth={isScreenHeightLessThan800 ? 'xs' : 'sm'}
      fullWidth={true}
      open={isOpen}
      key={title}
    >
      <DialogTitle
        align="center"
        sx={{ fontSize: isScreenHeightLessThan800 ? '1.2rem' : '1.5rem' }}
      >
        {title}
      </DialogTitle>
      <Divider />
      {progress ? (
        <>
          <Typography variant="body2" fontWeight="bold" ml={3} mt={1}>
            Task progress
          </Typography>
          <Box component="div" width={isScreenHeightLessThan800 ? 0.9 : 0.95} ml={3}>
            <LinearProgressWithLabel value={progress} />
          </Box>
        </>
      ) : null}
      <DialogContent>{returnDialogContent(alertContents)}</DialogContent>

      <DialogActions>
        {onInspect ? (
          <Button
            size="small"
            variant="contained"
            onClick={onInspect}
            disabled={false}
            autoFocus
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
          >
            Inspect
          </Button>
        ) : null}
        {acknowledged ? (
          <Button
            size="small"
            variant="contained"
            disabled={true}
            autoFocus
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
          >
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
            sx={{
              fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
              padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
            }}
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
          sx={{
            fontSize: isScreenHeightLessThan800 ? '0.8rem' : '1rem',
            padding: isScreenHeightLessThan800 ? '4px 8px' : '6px 12px',
          }}
        >
          {acknowledged ? 'Close' : 'Dismiss'}
        </Button>
      </DialogActions>
    </Dialog>
  );
});
