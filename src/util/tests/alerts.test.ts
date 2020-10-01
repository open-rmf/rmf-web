import { createMuiTheme } from '@material-ui/core';
import { Alerts } from '../alerts';

const theme = createMuiTheme();

describe('Verification Alert', () => {
  test('Correct generation of default params values', async () => {
    const verification = Alerts.verification({});
    console.log(verification);
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

  test('Correct generation of custom params values', async () => {
    const title = 'test';
    const text = 'test';
    const icon = 'error';
    const confirmButtonText = 'test';
    const cancelButtonText = 'test';

    const verification = Alerts.verification({
      title: title,
      body: text,
      icon: icon,
      confirmButtonText: confirmButtonText,
      cancelButtonText: cancelButtonText,
    });
    console.log(verification);
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

// describe('Success Alert', () => {
//   test('Correct generation of default params values', async () => {
//     // MySwal.fire('hue');
//     // console.log(MySwal)
//     const hue = verificationAlert({});
//     console.log(hue);

//   });

//   test('Correct generation of custom params values', async () => {
//     // MySwal.fire('hue');
//     // console.log(MySwal)
//     const hue = verificationAlert({});
//     console.log(hue);
//   });
// })

// describe('Error Alter', () => {
//   test('Correct generation of default params values', async () => {
//     // MySwal.fire('hue');
//     // console.log(MySwal)
//     const hue = verificationAlert({});
//     console.log(hue);

//   });

//   test('Correct generation of custom params values', async () => {
//     // MySwal.fire('hue');
//     // console.log(MySwal)
//     const hue = verificationAlert({});
//     console.log(hue);
//   });
// })
