import { makeStyles } from '@material-ui/core';

export const useFormStyles = makeStyles((theme) => ({
  form: {
    display: 'flex',
    flexDirection: 'column',
    width: '100%',
    backgroundColor: theme.palette.background.paper,
  },
  divForm: {
    padding: '0.46rem',
    paddingRight: '0.5rem',
    width: '100%',
  },
  input: {
    width: '100%',
  },
  button: {
    width: '100%',
  },
  buttonContainer: {
    paddingTop: '0.5rem',
    paddingLeft: '0.5rem',
    width: '100%',
  },
}));
