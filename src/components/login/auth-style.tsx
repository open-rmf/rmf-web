import { makeStyles } from '@material-ui/core';

const authStyles = makeStyles(theme => ({
  flexColumnContainer: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    fontFamily: 'monospace',
  },
  fullPage: {
    width: '100%',
    height: '100%',
    backgroundColor: '#44497a',
  },
  loginContainer: {
    borderStyle: 'none',
    borderRadius: 20,
    borderColor: 'black',
    padding: '0px 50px 0 50px',
    width: 'fit-content',
    height: '40%',
    minWidth: 250,
    marginTop: 'auto',
    backgroundColor: 'snow',
    boxShadow: theme.shadows[12],
    alignContent: 'center',
    justifyContent: 'center',
  },
  loginTitle: {
    color: '#44497a',
    marginBottom: 10,
  },
  loginForm: {
    marginTop: 10,
  },
  loginFailSnackbar: {
    marginBottom: 50,
  },
  buttonContainer: {
    border: '0',
    margin: '0.5rem',
    display: 'inline-flex',

    position: 'relative',
    minWidth: 0,
    flexDirection: 'column',
    verticalAlign: 'top',
    width: '100%',
    justifyContent: 'center',
    alignItems: 'center',
  },
  button: {
    width: '100%',
    backgroundColor: '#44497a',
  },
  resetPassword: {
    alignSelf: 'center',
    color: '#918e8e',
  },
  termOfServices: {
    alignSelf: 'center',
    color: '#918e8e',
    paddingTop: '0.5rem',
    marginBottom: 'auto',
  },
  logo: {
    height: '10%',
  },
}));

export default authStyles;
