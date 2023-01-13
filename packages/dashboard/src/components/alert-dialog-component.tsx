import React from 'react';
import {
  Box,
  Button,
  Grid,
  LinearProgress,
  LinearProgressProps,
  Theme,
  Typography,
  Divider,
  TextField,
} from '@mui/material';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';
import { RobotWithTask } from './alert-store';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    title: {
      '&.MuiDialogTitle-root': {
        padding: theme.spacing(1),
        textAlign: 'center',
        fontWeight: 'bold',
      },
    },
    subtitle: {
      '&.MuiTypography-root': {
        fontWeight: 'bold',
      },
    },
    textField: {
      background: theme.palette.background.default,
    },
  }),
);

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

interface AlertToDisplay extends RobotWithTask {
  show: boolean;
}

export interface AlertContent {
  title: string;
  value: string;
}
export interface DialogAlertProps {
  alertToDisplay: AlertToDisplay;
  setValue: (value: React.SetStateAction<AlertToDisplay[]>) => void;
  dialogTitle: string;
  progress: number;
  alertContents: AlertContent[];
  backgroundColor: string;
}

export const AlertDialog = React.memo((props: DialogAlertProps) => {
  const classes = useStyles();

  const returnDialogContent = (alertContents: AlertContent[]) => {
    return (
      <>
        {alertContents.map((message, index) => (
          <Grid key={index} container mb={1} alignItems="center" spacing={1}>
            <Grid item xs={3}>
              <Typography className={classes.subtitle}>{message.title}</Typography>
            </Grid>
            <Grid item xs={9}>
              <TextField
                size="small"
                value={message.value}
                InputProps={{
                  readOnly: true,
                  className: classes.textField,
                }}
              />
            </Grid>
          </Grid>
        ))}
      </>
    );
  };

  const { setValue, alertToDisplay, dialogTitle, progress, alertContents, backgroundColor } = props;

  return (
    <Dialog
      key={alertToDisplay.robot.name}
      PaperProps={{
        style: {
          backgroundColor: backgroundColor,
          boxShadow: 'none',
        },
      }}
      open={alertToDisplay.show}
      maxWidth="xs"
    >
      <DialogTitle className={classes.title}>Alert</DialogTitle>
      <Divider />
      <DialogTitle className={classes.title}>{dialogTitle}</DialogTitle>
      <Box sx={{ width: '100%' }}>
        <LinearProgressWithLabel value={progress} />
      </Box>
      <DialogContent>{returnDialogContent(alertContents)}</DialogContent>
      <DialogActions>
        <Button
          onClick={() =>
            setValue((prev) =>
              prev.map((obj) => {
                if (obj.robot.name === alertToDisplay.robot.name) {
                  return { ...obj, show: false };
                }
                return obj;
              }),
            )
          }
          autoFocus
        >
          Close
        </Button>
      </DialogActions>
    </Dialog>
  );
});
