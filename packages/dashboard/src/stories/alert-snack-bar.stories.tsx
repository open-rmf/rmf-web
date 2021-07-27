import React from 'react';
import AlertSnackBarStory from './baseComponents/alert-snack-bar-component';
import { iniCharger } from '../components/alert-snack-bar';

export default {
  title: 'Alert Snack Bar',
};

export const error = () => (
  <AlertSnackBarStory
    message={'Error message'}
    type={'error'}
    charger={iniCharger}
    showAlert={false}
  />
);

export const warning = () => (
  <AlertSnackBarStory
    message={'Warning message'}
    type={'warning'}
    charger={iniCharger}
    showAlert={false}
  />
);

export const info = () => (
  <AlertSnackBarStory
    message={'Info message'}
    type={'info'}
    charger={iniCharger}
    showAlert={false}
  />
);

export const success = () => (
  <AlertSnackBarStory
    message={'Success message'}
    type={'success'}
    charger={iniCharger}
    showAlert={false}
  />
);
