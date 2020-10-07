import React from 'react';
import { createShallow } from '@material-ui/core/test-utils';
import DrawerHeader from '../drawer-header';
import CloseIcon from '@material-ui/icons/Close';
import { KeyboardBackspace as BackIcon } from '@material-ui/icons';
import { Typography } from '@material-ui/core';

const mount = createShallow();

describe('Drawer header', () => {
  test('Renders correctly', () => {
    const root = mount(
      <DrawerHeader
        handleCloseButton={() => jest.fn()}
        handleBackButton={() => jest.fn()}
        title={'Test'}
      />,
    );
    expect(root.find(Typography).html()).toContain('Test');
    expect(root.find(CloseIcon).exists()).toBeTruthy();
    expect(root.find(BackIcon).exists()).toBeTruthy();
    expect(root).toMatchSnapshot();
  });

  test('Renders without back button if there is no handler', () => {
    const root = mount(<DrawerHeader handleCloseButton={() => jest.fn()} title={'Test'} />);
    expect(root.find(BackIcon).exists()).toBeFalsy();
  });

  test('Click on close button fires an event correctly', () => {
    let clicked = false;
    const root = mount(<DrawerHeader handleCloseButton={() => (clicked = true)} title={'Test'} />);
    root.find('#closeDrawerButton').simulate('click');
    expect(clicked).toBe(true);
  });

  test('Click on back button fires an event correctly', () => {
    let clicked = false;
    const root = mount(
      <DrawerHeader
        handleCloseButton={() => jest.fn()}
        handleBackButton={() => (clicked = true)}
        title={'Test'}
      />,
    );
    root.find('#backDrawerButton').simulate('click');
    expect(clicked).toBe(true);
  });
});
