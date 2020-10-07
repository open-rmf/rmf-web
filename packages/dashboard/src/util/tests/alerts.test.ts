import { createMuiTheme } from '@material-ui/core';
import { Alerts } from '../alerts';

const theme = createMuiTheme();

describe('Verification Alert', () => {
  test('Correct generation of default params values', () => {
    const verification = Alerts.getVerificationParams({});
    expect(verification).toEqual({
      title: 'Are you sure you want to continue?',
      text: 'Once you accept this there is no turning back.',
      icon: 'warning',
      cancelButtonColor: theme.palette.primary.main,
      confirmButtonColor: theme.palette.error.main,
      showCancelButton: true,
      heightAuto: false,
      confirmButtonText: `Yes, I'm sure.`,
      cancelButtonText: 'No',
    });
  });

  test('Correct generation of custom params values', () => {
    const title = 'test';
    const text = 'test';
    const icon = 'error';
    const confirmButtonText = 'test';
    const cancelButtonText = 'test';

    const verification = Alerts.getVerificationParams({
      title: title,
      body: text,
      icon: icon,
      confirmButtonText: confirmButtonText,
      cancelButtonText: cancelButtonText,
    });
    expect(verification).toEqual({
      title: title,
      text: text,
      icon: icon,
      cancelButtonColor: theme.palette.primary.main,
      confirmButtonColor: theme.palette.error.main,
      showCancelButton: true,
      heightAuto: false,
      confirmButtonText: confirmButtonText,
      cancelButtonText: confirmButtonText,
    });
  });
});

describe('Success Alert', () => {
  test('Correct generation of default params values', () => {
    const success = Alerts.getSuccessMsg();
    expect(success).toEqual({
      title: 'Done!',
      text: 'Successful Operation',
      icon: 'success',
      timer: 2000,
      heightAuto: false,
    });
  });

  test('Correct generation of custom params values', () => {
    const success = Alerts.getSuccessMsg('test');
    expect(success.text).toBe('test');
  });
});

describe('Error Alert', () => {
  test('Correct generation of default params values', () => {
    const error = Alerts.getErrorMsg();
    expect(error).toEqual({
      title: 'Ups',
      text: 'An error has occurred',
      icon: 'error',
      heightAuto: false,
    });
  });

  test('Correct generation of custom params values', () => {
    const error = Alerts.getErrorMsg('test');
    expect(error.text).toBe('test');
  });
});
