import { makeStyles } from '@material-ui/core';

const authStyles = makeStyles(theme => ({
  flexColumnContainer: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
  },
  fullPage: {
    width: '100%',
    height: '100%',
    backgroundColor: '#44497a',
  },
  authContainer: {
    flexDirection: 'column',
    borderStyle: 'none',
    borderRadius: 20,
    borderColor: 'black',
    padding: '0px 70px 0 70px',
    width: 'fit-content',
    minWidth: 250,
    backgroundColor: 'snow',
    boxShadow: theme.shadows[12],
    alignContent: 'center',
    justifyContent: 'center',
  },
  authTitle: {
    color: '#44497a',
    marginTop: 0,
    paddingBottom: '1rem',
  },
  authForm: {
    marginTop: 10,
    flexDirection: 'column',
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
    display: 'inline-flex',
    alignSelf: 'center',
    color: '#918e8e',
    paddingTop: '0.5rem',
  },
  logo: {
    height: '10%',
  },
  resetPassword: {
    alignSelf: 'center',
    color: '#918e8e',
  },
}));

export default authStyles;
