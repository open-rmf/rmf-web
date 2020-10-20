import { createMuiTheme } from '@material-ui/core';
import { mount } from 'enzyme';
import React from 'react';
import { ConfirmAlertDialog } from '../confirm-alert-dialog';

const theme = createMuiTheme();

describe('Verification Alert', () => {
  test('Correct generation of default params values', () => {
    const root = mount(
      <ConfirmAlertDialog open={true} close={() => jest.mock}></ConfirmAlertDialog>,
    );
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  test('Correct generation of custom params values', () => {
    const root = mount(
      <ConfirmAlertDialog
        open={true}
        title={'test'}
        content={'test'}
        confirmButtonText={'test'}
        cancelButtonText={'test'}
        cancelCallback={() => jest.mock}
        confirmCallback={() => jest.mock}
        close={() => jest.mock}
      ></ConfirmAlertDialog>,
    );
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  test('Cancel callback is executed', () => {});
  test('Confirm callback is executed', () => {});
  test('Modal is closed', () => {});
});
