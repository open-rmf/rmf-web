import React from 'react';
import {
  Button,
  Radio,
  RadioGroup,
  FormControlLabel,
  FormControl,
  FormLabel,
  Typography,
  Paper,
  makeStyles,
} from '@material-ui/core';

const useStyles = makeStyles((theme) => ({
  root: {
    padding: theme.spacing(4),
    height: '100vh',
  },
  formLabel: {
    marginRight: 'auto',
  },
}));

interface TaskFormProps {
  placeNames: string[];
}

export const TaskForm = (props: TaskFormProps) => {
  const classes = useStyles();
  const { placeNames } = props;
  const [place, setPlace] = React.useState('');
  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setPlace(event.target.value);
  };

  return (
    <Paper className={classes.root}>
      <Typography variant="h5">Loading Bay</Typography>
      <FormControl component="fieldset">
        <FormLabel component="legend">Destination</FormLabel>
        <RadioGroup onChange={handleChange} row>
          {placeNames.map((p) => (
            <FormControlLabel value={p} control={<Radio />} label={p} />
          ))}
        </RadioGroup>
      </FormControl>
      <Button variant="contained" color="primary" size="large">
        Submit
      </Button>
    </Paper>
  );
};
