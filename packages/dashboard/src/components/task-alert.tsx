import { Button, Grid, IconButton, Theme, Typography } from '@mui/material';
import React from 'react';
import { useTaskStore } from './store';
import Dialog from '@mui/material/Dialog';
import DialogActions from '@mui/material/DialogActions';
import DialogContent from '@mui/material/DialogContent';
import DialogTitle from '@mui/material/DialogTitle';
import { makeStyles, createStyles } from '@mui/styles';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    paper: { minWidth: '200px' },
  }),
);

export const AlertComponentStore = () => {
  const classes = useStyles();
  const taskStore = useTaskStore((state) => state.task);
  const [count, setCount] = React.useState(0);

  const handleClose = () => {
    setCount(count + 1);
  };

  return (
    <>
      <IconButton id="alarm-btn" color={'secondary'}>
        {taskStore && count < taskStore.length ? (
          <>
            <Dialog open={count < taskStore.length} onClose={handleClose} maxWidth="xs">
              <DialogTitle>Alert</DialogTitle>
              <DialogTitle>Task State</DialogTitle>
              <DialogContent>
                <Grid container>
                  <Grid item xs={3}>
                    <Typography>Task</Typography>
                  </Grid>
                  <Grid item xs={3}>
                    <Typography>Task</Typography>
                  </Grid>
                </Grid>
                <Grid container>
                  <Grid item xs={3}>
                    <Typography>Location</Typography>
                  </Grid>
                  <Grid item xs={3}>
                    <Typography>CSSD</Typography>
                  </Grid>
                </Grid>
                <Grid container>
                  <Grid item xs={3}>
                    <Typography>Message</Typography>
                  </Grid>
                  <Grid item xs={9}>
                    <Typography>
                      AMR ready for pickup at CSSD LOT3; cart not found, please push cart in CSSD
                      LOT3
                    </Typography>
                  </Grid>
                </Grid>
              </DialogContent>
              <DialogActions>
                <Button onClick={handleClose}>Disagree</Button>
                <Button onClick={handleClose} autoFocus>
                  Agree
                </Button>
              </DialogActions>
            </Dialog>
          </>
        ) : null}
      </IconButton>
    </>
  );
};
