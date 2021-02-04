import React from 'react';
import AlertSnackBarStory from './baseComponents/alert-snack-bar-component';

export default {
  title: 'Alert Snack Bar',
};

export const error = () => <AlertSnackBarStory message={'Error message'} type={'error'} />;

export const warning = () => <AlertSnackBarStory message={'Warning message'} type={'warning'} />;

export const info = () => <AlertSnackBarStory message={'Info message'} type={'info'} />;

export const success = () => <AlertSnackBarStory message={'Success message'} type={'success'} />;
