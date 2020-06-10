import { makeStyles } from '@material-ui/core';

const authStyles = makeStyles(theme => ({
  flexColumnContainer: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
  },
  fullPage: {
    width: '100%',
    height: '100%',
    backgroundColor: '#44497a',
  },
  authContainer: {
    borderStyle: 'none',
    borderRadius: 20,
    borderColor: 'black',
    padding: '0px 50px 0 50px',
    width: 'fit-content',
    minWidth: 250,
    marginTop: 'auto',
    backgroundColor: 'snow',
    boxShadow: theme.shadows[12],
    alignContent: 'center',
    justifyContent: 'center',
  },
  authTitle: {
    color: '#44497a',
    marginBottom: 10,
  },
  authForm: {
    marginTop: 10,
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
  termOfServices: {
    alignSelf: 'center',
    color: '#918e8e',
    paddingTop: '0.5rem',
    marginBottom: 'auto',
  },
  logo: {
    height: '10%',
  },
  resetPassword: {
    alignSelf: 'center',
    color: '#918e8e',
  },
  loginContainer: {
    height: '40%',
  },
  registerContainer: {
    height: '50%',
  },
}));

export default authStyles;
