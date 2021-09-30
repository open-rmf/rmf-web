import { styled } from '@material-ui/core';

export const commandFormsClasses = {
  form: 'command-form-root',
  divForm: 'command-form-div-wrapper',
  input: 'command-form-input',
  button: 'command-form-button',
  buttonContainer: 'command-form-button-container',
};
export const CommandsFormRoot = styled('form')(({ theme }) => ({
  [`&.${commandFormsClasses.form}`]: {
    display: 'flex',
    flexDirection: 'column',
    width: '100%',
    backgroundColor: theme.palette.background.paper,
  },
  [`& .${commandFormsClasses.divForm}`]: {
    padding: '0.46rem',
    paddingRight: '0.5rem',
    width: '100%',
  },
  [`& .${commandFormsClasses.input}`]: {
    width: '100%',
  },
  [`& .${commandFormsClasses.button}`]: {
    width: '100%',
  },
  [`& .${commandFormsClasses.buttonContainer}`]: {
    paddingTop: '0.5rem',
    paddingLeft: '0.5rem',
    width: '100%',
  },
}));
