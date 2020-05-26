import { makeStyles, TextField, Button, Input } from '@material-ui/core';
import Autocomplete from '@material-ui/lab/Autocomplete';
import React from 'react';
import { successMsg, errorMsg } from '../util/alerts';

const top100Films = [
  { title: 'The Shawshank Redemption', year: 1994 },
  { title: 'The Godfather', year: 1972 },
  { title: 'The Godfather: Part II', year: 1974 },
  { title: 'The Dark Knight', year: 2008 },
];

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
        <h4>Robot Type </h4>
        <TextField
          name="robotType"
          placeholder="Name of the fleet"
          InputProps={{
            autoComplete: 'off',
          }}
          multiline
          // onChange={_handleInput}
          // value={name}
        />
      </div>

      <h4>Loops </h4>
      <div className={classes.divForm}>
        <Input
          type="number"
          name="numLoops"
          placeholder="Number of loops"
          // onChange={_handleInput}
          // value={description}
        />
      </div>

      <h4>Start Location </h4>
      <div className={classes.divForm}>
        <Autocomplete
          options={top100Films}
          getOptionLabel={option => option.title}
          style={{ width: 200 }}
          renderInput={params => <TextField {...params} label="Pick a place" variant="outlined" />}
        />
      </div>

      <h4>Finish Location </h4>
      <div className={classes.divForm}>
        <Autocomplete
          options={top100Films}
          getOptionLabel={option => option.title}
          style={{ width: 200 }}
          renderInput={params => <TextField {...params} label="Pick a place" variant="outlined" />}
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
    display: 'flex',
    flexDirection: 'column',
  },
  divForm: {
    padding: '0.2rem',
  },
}));
