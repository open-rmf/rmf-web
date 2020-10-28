import { mount, shallow } from 'enzyme';
import React from 'react';
import { ConfirmAlertDialog } from '../confirm-alert-dialog';

describe('Verification Alert', () => {
  test('Correct generation of default params values', () => {
    const root = shallow(<ConfirmAlertDialog open={true}></ConfirmAlertDialog>);
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  test('Correct render with custom params values', () => {
    const root = shallow(
      <ConfirmAlertDialog
        open={true}
        title={'test'}
        content={'test'}
        confirmButtonText={'test'}
        cancelButtonText={'test'}
        cancelCallback={() => jest.mock}
        confirmCallback={() => jest.mock}
      ></ConfirmAlertDialog>,
    );
    expect(root).toMatchSnapshot();
    root.unmount();
  });

  test('Cancel callback is executed', () => {
    const cancelCallback = jest.fn();
    const root = mount(
      <ConfirmAlertDialog
        open={true}
        title={'test'}
        content={'test'}
        confirmButtonText={'test'}
        cancelButtonText={'test'}
        cancelCallback={cancelCallback}
        confirmCallback={() => jest.mock}
      ></ConfirmAlertDialog>,
    );
    root.find('#alert-dialog-cancel-button').first().simulate('click');
    expect(cancelCallback).toHaveBeenCalledTimes(1);
    root.unmount();
  });

  test('Confirm callback is executed', () => {
    const confirmCallback = jest.fn();
    const root = mount(
      <ConfirmAlertDialog
        open={true}
        title={'test'}
        content={'test'}
        confirmButtonText={'test'}
        cancelButtonText={'test'}
        cancelCallback={() => jest.mock}
        confirmCallback={confirmCallback}
      ></ConfirmAlertDialog>,
    );
    root.find('#alert-dialog-confirm-button').first().simulate('click');
    expect(confirmCallback).toHaveBeenCalledTimes(1);
    root.unmount();
  });

  test('Modal is closed', () => {
    let open = true;
    const closeModal = () => {
      open = false;
    };
    const root = mount(
      <ConfirmAlertDialog
        open={open}
        title={'test'}
        content={'test'}
        confirmButtonText={'test'}
        cancelButtonText={'test'}
        cancelCallback={() => jest.mock}
        confirmCallback={closeModal}
      ></ConfirmAlertDialog>,
    );
    root.find('#alert-dialog-confirm-button').first().simulate('click');
    expect(open).toBeFalsy();
    root.unmount();
  });
});
