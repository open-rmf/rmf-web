import React from 'react';
import { createStyles, makeStyles, Theme } from '@material-ui/core/styles';
import TextField from '@material-ui/core/TextField';

const useStyles = makeStyles((theme: Theme) =>
  createStyles({
    container: {
      display: 'flex',
      flexWrap: 'wrap',
    },
    textField: {
      marginLeft: theme.spacing(1),
      marginRight: theme.spacing(1),
      width: 200,
    },
  }),
);

interface DateTimePickerProps {
  label: string;
  name: string;
}

export default function DateAndTimePickers(props: DateTimePickerProps): React.ReactElement {
  const classes = useStyles();
  return (
    <form className={classes.container} noValidate>
      <TextField
        id={`${props.name}-datetime-local`}
        label={props.label}
        type="datetime-local"
        defaultValue={new Date().toISOString().substr(0, 16)}
        className={classes.textField}
        InputLabelProps={{
          shrink: true,
        }}
      />
    </form>
  );
}
