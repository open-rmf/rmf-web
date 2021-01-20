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
  error?: string;
  date: string;
  handleDateChange: (event: React.ChangeEvent<{ name?: string; value: unknown }>) => void;
}

export default function DateAndTimePickers(props: DateTimePickerProps): React.ReactElement {
  const classes = useStyles();
  const { name, label, date, error, handleDateChange } = props;
  return (
    <form className={classes.container} noValidate>
      <TextField
        id={`${name}-datetime-local`}
        label={label}
        type="datetime-local"
        value={date ? date : new Date().toISOString().substr(0, 16)}
        className={classes.textField}
        error={!!error}
        helperText={error}
        onChange={(e) => handleDateChange(e)}
        InputLabelProps={{
          shrink: true,
        }}
      />
    </form>
  );
}
