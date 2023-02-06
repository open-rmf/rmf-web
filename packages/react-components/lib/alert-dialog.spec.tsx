import { ThemeProvider } from '@mui/material';
import { render } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { AlertContent, AlertDialog } from './alert-dialog';
import defaultTheme from '@mui/material/styles/defaultTheme';

describe('CloseAlertDialog', () => {
  it('calls onClick when OK button is clicked', () => {
    const buildDialogContent = (): AlertContent[] => {
      return [
        {
          title: 'Robot Name',
          value: 'Tiny1',
        },
        {
          title: 'Location',
          value: 'L1',
        },
        {
          title: 'Message',
          value: 'Robot has arrived at its destination ',
        },
      ];
    };
    const stopShowing = jasmine.createSpy();
    const root = render(
      <ThemeProvider theme={defaultTheme}>
        <AlertDialog
          key={'Tinny 1'}
          stopShowing={stopShowing}
          dialogTitle={'Robot State'}
          progress={90}
          alertContents={buildDialogContent()}
          backgroundColor={'ffff'}
          show={true}
        />
      </ThemeProvider>,
    );
    userEvent.click(root.getByText('Close'));
    expect(stopShowing).toHaveBeenCalled();
  });
});
