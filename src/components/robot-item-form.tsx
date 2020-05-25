import { makeStyles, TextField, Fab, Button } from '@material-ui/core';
import React from 'react';
import { successMsg, errorMsg } from '../util/alerts';

export const RobotLoopForm = () => {
  const classes = useStyles();
  /**
   * Handle the change of the elements of the form updating the state.
   * @param {Object} change
   */
  // handleFormChange = (change:any) => {
  // 	this.setState(() => change);
  // };

  // const onFormClick = (event:)

  // const _handleInput = (event: any) => {
  //   onClick({ [event.target.name]: event.target.value });
  // };

  return (
    <form className={classes.form}>
      <div className={classes.divForm}>
        <TextField
          name="robotType"
          placeholder="Robot Type"
          InputProps={{
            autoComplete: 'off',
          }}
          multiline
          // onChange={_handleInput}
          // value={name}
        />
      </div>
      <div className={classes.divForm}>
        <TextField
          name="numLoops"
          placeholder="Number of loops"
          InputProps={{
            autoComplete: 'off',
          }}
          multiline
          // onChange={_handleInput}
          // value={description}
        />
      </div>
      <div className={classes.divForm}>
        <TextField
          name="startName"
          placeholder="Start Location"
          InputProps={{
            autoComplete: 'off',
          }}
          // onChange={_handleInput}
          // value={description}
        />
      </div>
      <div className={classes.divForm}>
        <TextField
          name="finishName"
          placeholder="Finish Location"
          InputProps={{
            autoComplete: 'off',
          }}
          // onChange={_handleInput}
          // value={description}
        />
      </div>
      <div>
        <Button variant="contained" color="primary" onClick={() => successMsg('Success')}>
          {' '}
          Request
        </Button>
        <Button variant="contained" color="secondary" onClick={() => errorMsg('Error')}>
          {' '}
          Cancel
        </Button>
      </div>
    </form>
  );
};
//  className={classes.formButton} onClick={cancelRoleEditionMode}
// className={classes.formButton} onClick={onClick}

const useStyles = makeStyles(theme => ({
  form: {
    paddingLeft: '2rem',
  },
  divForm: {
    padding: '0.5rem',
  },
}));
